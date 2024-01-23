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
import numpy.typing

from ..constant import (
    LIMIT_FORCE_AXIAL_OPEN_LOOP,
    LIMIT_FORCE_TANGENT_OPEN_LOOP,
    MAX_LIMIT_FORCE_AXIAL_OPEN_LOOP,
    MAX_LIMIT_FORCE_TANGENT_OPEN_LOOP,
    NUM_ACTUATOR,
)
from ..enum import ActuatorDisplacementUnit
from ..utility import check_limit_switches
from .mock_plant import MockPlant


class MockControlOpenLoop:
    """Mock open-loop control.

    Attributes
    ----------
    open_loop_max_limit_is_enabled : `bool`
        The maximum limit of open-loop control is enabled or not.
    is_running : `bool`
        The open-loop control is running or not.
    """

    def __init__(self) -> None:
        self.open_loop_max_limit_is_enabled = False
        self.is_running = False

        # Selected actuator IDs to do the movement
        self._selected_actuators: list[int] = list()

        # Displacement of steps to do the movement
        self._displacement_steps = 0

    def is_actuator_force_out_limit(
        self, actuator_force: numpy.typing.NDArray[np.float64]
    ) -> tuple[bool, list[int], list[int]]:
        """The actuator force is out of limit or not. The result will depend
        on self.open_loop_max_limit_is_enabled.

        Notes
        -----
        If the force is out of limit, stop the running of open-loop control.

        Parameters
        ----------
        actuator_force : `numpy.ndarray`
            Actuator forces in Newton.

        Returns
        -------
        is_triggered : `bool`
            True if the actuator force is out of limit. Otherwise, False.
        limit_switch_retract : `list`
            Triggered retracted limit switches.
        limit_switch_extend : `list`
            Triggered extended limit switches.
        """

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

        is_triggered, limit_switch_retract, limit_switch_extend = check_limit_switches(
            actuator_force, limit_force_axial, limit_force_tangent
        )

        # Stop the running if the limit switch is triggered
        if is_triggered:
            self.is_running = False

        return is_triggered, limit_switch_retract, limit_switch_extend

    def start(
        self,
        actuators: list[int],
        displacement: int | float,
        unit: ActuatorDisplacementUnit,
    ) -> None:
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

    def _calculate_steps(
        self, displacement: int | float, unit: ActuatorDisplacementUnit
    ) -> int:
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
            int(displacement / MockPlant.STEP_TO_MM)
            if unit == ActuatorDisplacementUnit.Millimeter
            else int(displacement)
        )

    def stop(self) -> None:
        """Stop the movement."""

        self.is_running = False

        # Need to set the self._displacement_steps to be 0 to avoid the
        # self.resume() can put the self.is_running to be True again.
        # By doing this, the behaviors between self.stop() and self.pause()
        # are different.
        self._displacement_steps = 0

    def pause(self) -> None:
        """Pause the movement."""
        self.is_running = False

    def resume(self) -> None:
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

    def get_steps_to_move(self, steps: int) -> numpy.typing.NDArray[np.int64]:
        """Get the steps to move.

        If the requested displacement is done or the actuator force is out of
        limit, the value of self.is_running will change to False.

        Parameters
        ----------
        steps : `int`
            Absolute steps (>=0) to move. The internal calculation will
            consider the direction of target displacement by itself.

        Returns
        -------
        actuator_steps : `numpy.ndarray` [`int`]
            78 actuator steps.

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

        # Deside the step to move
        steps_to_move = np.sign(self._displacement_steps) * int(steps)

        if abs(self._displacement_steps) < abs(steps_to_move):
            actuator_step = self._displacement_steps
            self._displacement_steps = 0
        else:
            actuator_step = steps_to_move
            self._displacement_steps -= steps_to_move

        # Finish the running after the final movement
        if self._displacement_steps == 0:
            self.is_running = False

        # Return the actuator steps to move
        actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)
        for idx in self._selected_actuators:
            actuator_steps[idx] = actuator_step

        return actuator_steps
