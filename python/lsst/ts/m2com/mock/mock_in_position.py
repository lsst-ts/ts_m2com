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

__all__ = ["MockInPosition"]

from collections import deque

import numpy as np
import numpy.typing

from ..constant import NUM_ACTUATOR, NUM_HARDPOINTS_AXIAL, NUM_TANGENT_LINK


class MockInPosition:
    """Mock InPosition class that translates from the "InPosition.lvclass"
    in ts_mtm2_cell.

    Parameters
    ----------
    window_size : `int`
        Window size in second.
    control_frequency : `float`
        Control frequency in Hz.
    threshold_axial : `float`
        Threshold of the force error of axial actuator in Newton.
    threshold_tangent : `float`
        Threshold of the force error of tangent actuator in Newton.

    Attributes
    ----------
    threshold_squared_axial : `float`
        Squared threshold of the force error of axial actuator in square
        Newton.
    threshold_squared_tangent : `float`
        Squared threshold of the force error of tangent actuator in square
        Newton.
    """

    def __init__(
        self,
        window_size: int,
        control_frequency: float,
        threshold_axial: float,
        threshold_tangent: float,
    ) -> None:
        self._queue: deque = deque(maxlen=int(window_size * control_frequency))
        self._running_sum = np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)

        self.threshold_squared_axial = threshold_axial**2
        self.threshold_squared_tangent = threshold_tangent**2

    def is_in_position(self, force_error: numpy.typing.NDArray[np.float64]) -> bool:
        """Mirror is in position or not based on the thresholds of actuator
        force error.

        Parameters
        ----------
        force_error : `numpy.ndarray`
            Force error of the 72 active actuators in Newton.

        Returns
        -------
        `bool`
            True if the mirror is in position. Otherwise, False.
        """

        # Workaround the mypy check
        assert self._queue.maxlen is not None

        # If the queue is full, pop out the earliest value
        if len(self._queue) >= self._queue.maxlen:
            # Minus the earliest value in the running sum
            self._running_sum -= self._queue.popleft()

        # Square the force error
        force_error_square = np.square(force_error)

        # Put the new value into queue and update the running sum
        self._queue.append(force_error_square)
        self._running_sum += force_error_square

        num_axial_active = NUM_ACTUATOR - NUM_TANGENT_LINK - NUM_HARDPOINTS_AXIAL
        num_queue = len(self._queue)
        return np.all(
            self._running_sum[:num_axial_active]
            <= (num_queue * self.threshold_squared_axial)
        ) and np.all(
            self._running_sum[num_axial_active:]
            <= (num_queue * self.threshold_squared_tangent)
        )

    def reset(self) -> None:
        """Reset the internal data."""

        self._queue.clear()
        self._running_sum = np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)
