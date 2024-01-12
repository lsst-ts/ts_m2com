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

__all__ = ["MockPlant"]

import numpy as np
import numpy.typing

from ..constant import NUM_ACTUATOR
from ..utility import get_forces_mirror_weight


class MockPlant:
    """Mock plant model to simulate the actuator's force feedback.

    Parameters
    ----------
    static_transfer_matrix : `numpy.ndarray`
        Static transfer matrix used to simulate the delta force from the change
        of steps. This is a 78 x 78 matrix. The unit of row is the Newton and
        the unit of column is the actuator's step.
    inclinometer_angle : `float`
        Inclinometer angle in degree.

    Attributes
    ----------
    actuator_steps : `numpy.ndarray` [`int`]
        Current positions of the actuator in steps referenced to the home
        position.
    """

    # 1 step equals 1.9967536601e-5 millimeter
    # In the simulation, we just use a single value. In the real system, each
    # actuator has its own calibrated value.
    STEP_TO_MM = 1.9967536601e-5

    def __init__(
        self,
        static_transfer_matrix: numpy.typing.NDArray[np.float64],
        inclinometer_angle: float,
    ) -> None:
        self._static_transfer_matrix = static_transfer_matrix

        self.actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)

        # Actuator force based on the mirror's weight in Newton.
        self._actuator_force_weight = get_forces_mirror_weight(inclinometer_angle)

        # Actuator force based on the step in Newton
        self._actuator_force_step = np.zeros(NUM_ACTUATOR)

    def get_actuator_forces(self) -> numpy.typing.NDArray[np.float64]:
        """Get the actuator forces.

        Returns
        -------
        `numpy.ndarray`
            Actuator forces in Newton.
        """

        return self._actuator_force_step + self._actuator_force_weight

    def get_actuator_positions(self) -> numpy.typing.NDArray[np.float64]:
        """Get the actuator positions in millimeter.

        Returns
        -------
        `numpy.ndarray`
            Actuator positions in millimeter.
        """

        return self.actuator_steps * self.STEP_TO_MM

    def reset_actuator_steps(self) -> None:
        """Reset the actuator steps."""

        self.actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)
        self._actuator_force_step = np.zeros(NUM_ACTUATOR)

    def update_actuator_force_weight(self, inclinometer_angle: float) -> None:
        """Update the component of actuator's force according to the mirror's
        weight.

        Parameters
        ----------
        inclinometer_angle : `float`
            Inclinometer angle in degree.
        """

        self._actuator_force_weight = get_forces_mirror_weight(inclinometer_angle)

    def move_actuator_steps(
        self, actuator_steps: numpy.typing.NDArray[np.int64]
    ) -> None:
        """Move the actuator steps.

        Notes
        -----
        This function is translated from vendor's original LabVIEW code:
        "Steps to Force.vi" in ts_mtm2_cell.
        The static transfer matrix is used to simlulate the force change of
        actuator's movement.

        Parameters
        ----------
        actuator_steps : `numpy.ndarray` [`int`]
            78 actuator steps.

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

        self.actuator_steps += actuator_steps

        forces = self._static_transfer_matrix.dot(actuator_steps.reshape(-1, 1)).ravel()
        self._actuator_force_step += forces
