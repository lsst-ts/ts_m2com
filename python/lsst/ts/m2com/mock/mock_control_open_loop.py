# This file is part of ts_m2com.
#
# Developed for the Vera Rubin Observatory Telescope and Site Systems.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

__all__ = ["MockControlOpenLoop"]

import numpy as np
import pandas as pd

from ..constant import (
    LIMIT_FORCE_AXIAL_OPEN_LOOP,
    LIMIT_FORCE_TANGENT_OPEN_LOOP,
    MAX_LIMIT_FORCE_AXIAL_OPEN_LOOP,
    MAX_LIMIT_FORCE_TANGENT_OPEN_LOOP,
    MIRROR_WEIGHT_KG,
    NUM_ACTUATOR,
    NUM_TANGENT_LINK,
)
from ..enum import ActuatorDisplacementUnit
from ..utility import check_limit_switches


class MockControlOpenLoop:
    """Mock open-loop control.

    Attributes
    ----------
    inclinometer_angle : `float`
        Inclinometer angle in degree.
    open_loop_max_limit_is_enabled : `bool`
        The maximum limit of open-loop control is enabled or not.
    is_running : `bool`
        The open-loop control is running or not.
    actuator_steps : `numpy.ndarray [int]`
        Current positions of the actuator in steps referenced to the home
        position.
    """

    # 1 step equals 1.9967536601e-5 millimeter
    # In the simulation, we just use a single value. In the real system, each
    # actuator has its own calibrated value.
    STEP_TO_MM = 1.9967536601e-5

    def __init__(self):
        self.inclinometer_angle = 0
        self.open_loop_max_limit_is_enabled = False
        self.is_running = False

        # Static transfer matrix used to simulate the delta force from the
        # change of steps
        self._static_transfer_matrix = np.array([])

        self.actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)

        # Selected actuator IDs to do the movement
        self._selected_actuators = list()

        # Displacement of steps to do the movement
        self._displacement_steps = 0

    def read_file_static_transfer_matrix(self, filepath, skiprows=7):
        """Read the file of static transfer matrix (the size is 78 x 78).

        This file comes from the vender's original project used in the
        simulation purpose.

        Parameters
        ----------
        filepath : `str` or `pathlib.PosixPath`
            File path of the static transfer matrix.
        skiprows : `int`, optional
            Number of lines to skip (int) at the start of the file. (the
            default is 7)
        """

        dataframe = pd.read_csv(filepath, skiprows=skiprows)
        data = np.array(dataframe.iloc[:, 0])

        self._static_transfer_matrix = data.reshape(NUM_ACTUATOR, NUM_ACTUATOR)

    def update_actuator_steps(self, actuator_steps):
        """Update the current actuator steps referenced to the home position.

        Parameters
        ----------
        actuator_steps : `numpy.ndarray [int]`
            New current positions of the actuator in steps.

        Raises
        ------
        `ValueError`
            When the length of actuators does not match.
        `ValueError`
            When the data type is not integer.
        """

        num_actuators = len(actuator_steps)
        if num_actuators != NUM_ACTUATOR:
            raise ValueError(
                f"Received actuator length ({num_actuators}) doesn't match the expectation: {NUM_ACTUATOR}."
            )

        if actuator_steps.dtype != int:
            raise ValueError(f"Expected integer data type, got {actuator_steps.dtype}.")

        self.actuator_steps = actuator_steps

    def correct_inclinometer_angle(self, angle, offset=0.94):
        """Correct the inclinometer's value and make sure to limit the
        resulting value to the indicated range: (-270, 90).

        This function is translated from vendor's original LabVIEW code.
        The hard-coded range is related to the configuration of inclinometer
        in the mirror assembly.

        Parameters
        ----------
        angle : `float`
            Inclinometer angle in degree.
        offset : `float`, optional
            Offset in degree. The default value is hard-coded in the LabVIEW's
            project. (the default is 0.94)

        Returns
        -------
        angle_correct : `float`
            Corrected inclinometer angle in degree.
        """

        angle_offset = angle + offset
        origin = 360 if (0 <= angle_offset < 90) else 0

        angle_correct = 180 - angle_offset - origin

        # Make sure the calculated angle value is limited to this range
        if angle_correct > 90:
            angle_correct = 90
        elif angle_correct < -270:
            angle_correct = -270

        return angle_correct

    def get_forces_mirror_weight(self, angle):
        """Get the forces that bear the weight of mirror.

        This function is translated from vendor's original LabVIEW code.
        This is just an estimation of forces.

        Parameters
        ----------
        angle : `float`
            Inclinometer angle in degree.

        Returns
        -------
        forces : `numpy.ndarray`
            Forces to support the mirror in Newton.
        """

        forces = np.zeros(NUM_ACTUATOR)

        angle_correct = self.correct_inclinometer_angle(angle)

        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK

        gravitation_acceleration = 9.8
        force_mirror_weight = MIRROR_WEIGHT_KG * gravitation_acceleration

        forces[:num_axial_actuators] = (
            force_mirror_weight
            * np.sin(np.deg2rad(angle_correct))
            / num_axial_actuators
        )

        # Tangent actuators A1 and A4 do not bear the weight of mirror.
        # A2 and A3 have the reversed direction compared with A5 and A6.
        index_tangent_link = num_axial_actuators + np.array([1, 2, 4, 5])
        forces[index_tangent_link] = (
            force_mirror_weight * np.cos(np.deg2rad(angle_correct)) / 4
        ) * np.array([-1, -1, 1, 1])

        return forces

    def is_actuator_force_out_limit(self):
        """The actuator force is out of limit or not. The result will depend
        on self.open_loop_max_limit_is_enabled.

        Returns
        -------
        `bool`
            True if the actuator force is out of limit. Otherwise, False.
        `list`
            Triggered retracted limit switches.
        `list`
            Triggered extended limit switches.
        """

        forces = self.calculate_steps_to_forces(self.actuator_steps)

        limit_force_axial = (
            MAX_LIMIT_FORCE_AXIAL_OPEN_LOOP
            if self.open_loop_max_limit_is_enabled
            else LIMIT_FORCE_AXIAL_OPEN_LOOP
        )
        limit_force_tangent = (
            MAX_LIMIT_FORCE_TANGENT_OPEN_LOOP
            if self.open_loop_max_limit_is_enabled
            else LIMIT_FORCE_TANGENT_OPEN_LOOP
        )

        return check_limit_switches(forces, limit_force_axial, limit_force_tangent)

    def calculate_steps_to_forces(self, steps):
        """Calculate the steps to forces.

        This function is translated from vendor's original LabVIEW code.
        The static transfer matrix is used to simlulate the force change of
        actuator's movement.

        Parameters
        ----------
        steps : `numpy.ndarray [int]`
            Actuator steps.

        Returns
        -------
        `numpy.ndarray`
            Actuator forces in Newton.
        """

        return self._static_transfer_matrix.dot(
            steps.reshape(-1, 1)
        ).ravel() + self.get_forces_mirror_weight(self.inclinometer_angle)

    def calculate_forces_to_steps(self, forces):
        """Calculate the forces to steps.

        This is the reverse of self.calculate_steps_to_forces().

        Parameters
        ----------
        forces : `numpy.ndarray`
            Actuator forces in Newton.

        Returns
        -------
        `numpy.ndarray [int]`
            Actuator steps.
        """

        forces_delta = forces - self.get_forces_mirror_weight(self.inclinometer_angle)

        steps = (
            np.linalg.inv(self._static_transfer_matrix)
            .dot(forces_delta.reshape(-1, 1))
            .astype(int)
        )

        return steps.ravel()

    def calculate_forces_to_positions(self, forces):
        """Calculate the forces to positions.

        This is just an extension of self.calculate_forces_to_steps().

        Parameters
        ----------
        forces : `numpy.ndarray`
            Actuator forces in Newton.

        Returns
        -------
        `numpy.ndarray`
            Actuator positions in millimeter.
        """
        return self.calculate_forces_to_steps(forces) * self.STEP_TO_MM

    def get_actuator_positions(self):
        """Get the actuator positions in millimeter.

        Returns
        -------
        `numpy.ndarray`
            Actuator positions in millimeter.
        """
        return self.actuator_steps * self.STEP_TO_MM

    def start(self, actuators, displacement, unit):
        """Start the movement.

        Parameters
        ----------
        actuators : `list [int]`
            Actuator IDs to do the movement.
        displacement : `int` or `float`
            Displacement to move.
        unit : enum `ActuatorDisplacementUnit`
            Unit of the displacement.

        Raises
        ------
        `RuntimeError`
            When the actuators are moving.
        `ValueError`
            When no actuators are selected.
        """

        if self.is_running:
            raise RuntimeError("The actuators are moving now.")

        if len(actuators) == 0:
            raise ValueError("No actuators are selected to move.")

        self._selected_actuators = actuators
        self._displacement_steps = self._calculate_steps(displacement, unit)

        self.is_running = True

    def _calculate_steps(self, displacement, unit):
        """Calculate the steps of displacement.

        Parameters
        ----------
        displacement : `int` or `float`
            Displacement to move.
        unit : enum `ActuatorDisplacementUnit`
            Unit of the displacement.

        Returns
        -------
        `int`
            Steps of the displacement.
        """
        return (
            int(displacement / self.STEP_TO_MM)
            if unit == ActuatorDisplacementUnit.Millimeter
            else displacement
        )

    def stop(self):
        """Stop the movement."""

        self.is_running = False

        # Need to set the self._displacement_steps to be 0 to avoid the
        # self.resume() can put the self.is_running to be True again.
        # By doing this, the behaviors between self.stop() and self.pause()
        # are different.
        self._displacement_steps = 0

    def pause(self):
        """Pause the movement."""
        self.is_running = False

    def resume(self):
        """Resume the movement.

        Raises
        ------
        `RuntimeError`
            When the movement is done.
        """

        if self._displacement_steps != 0:
            self.is_running = True
        else:
            raise RuntimeError("The movement is done.")

    def run_steps(self, steps):
        """Run the steps.

        If the requested displacement is done or the actuator force is out of
        limit, the value of self.is_running will change to False.

        Parameters
        ----------
        steps : `int`
            Absolute steps (>=0) to move. The internal calculation will
            consider the direction of target displacement by itself.

        Raises
        ------
        `RuntimeError`
            When the actuators are not running.
        `ValueError`
            When the steps is less than 0.
        """

        if not self.is_running:
            raise RuntimeError("The actuators are not running now.")

        if steps < 0:
            raise ValueError(f"The steps (={steps}) should be >= 0.")

        steps_to_move = np.sign(self._displacement_steps) * int(steps)

        if abs(self._displacement_steps) < abs(steps_to_move):
            self.move_actuator_steps(self._selected_actuators, self._displacement_steps)
            self._displacement_steps = 0
        else:
            self.move_actuator_steps(self._selected_actuators, steps_to_move)
            self._displacement_steps -= steps_to_move

        if (self._displacement_steps == 0) or self.is_actuator_force_out_limit()[0]:
            self.is_running = False

    def move_actuator_steps(self, actuators, steps):
        """Move the actuator steps.

        Parameters
        ----------
        actuators : `list [int]` or `numpy.ndarray [int]`
            Actuator IDs to do the movement.
        steps : `int`, `list [int]` or `numpy.ndarray [int]`
            Actuator steps to move.

        Raises
        ------
        `ValueError`
            When the data type is not integer.
        `RuntimeError`
            When the actuator force is out of limit.
        """

        actuators = np.array(actuators)
        steps = np.array(steps)

        if (actuators.dtype != int) or (steps.dtype != int):
            raise ValueError("The data type is not integer.")

        if self.is_actuator_force_out_limit()[0]:
            raise RuntimeError("The actuator force is out of limit now.")
        else:
            self.actuator_steps[actuators] += steps
