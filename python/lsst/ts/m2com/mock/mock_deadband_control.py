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

__all__ = ["MockDeadbandControl"]

import numpy as np
import numpy.typing

from ..constant import NUM_HARDPOINTS_AXIAL


class MockDeadbandControl:
    """Mock deadband control to select the force error of hardpoints.

    Notes
    -----
    Force error values are latched when the largest force error falls below the
    low threshold and the output remains constant until the largest force error
    exceeds the high threshold.

    Parameters
    ----------
    threshold_lower : `float`
        Lower threshold in Newton.
    threshold_upper : `float`
        Upper threshold in Newton.
    """

    def __init__(self, threshold_lower: float, threshold_upper: float) -> None:
        self._threshold_lower = threshold_lower
        self._threshold_upper = threshold_upper

        self._keep_tracking = False
        self._hardpoint_error_track = np.zeros(NUM_HARDPOINTS_AXIAL)

    def reset(self, reset_all: bool = False) -> None:
        """Reset the deadband.

        Parameters
        ----------
        reset_all : `bool`, optional
            Reset all of the internal data or not. (the default is False)
        """

        self._keep_tracking = False

        if reset_all:
            self._hardpoint_error_track = np.zeros(NUM_HARDPOINTS_AXIAL)

    def select(
        self,
        hardpoint_error: numpy.typing.NDArray[np.float64],
        is_enabled: bool = True,
    ) -> numpy.typing.NDArray[np.float64]:
        """Select the hardpoint error.

        Parameters
        ----------
        hardpoint_error : `numpy.ndarray`
            Force error of the hardpoints in Newton.
        is_enabled : `bool`, optional
            Enable the deadzone or not. (the default is True)

        Returns
        -------
        `numpy.ndarray`
            Selected hardpoint error.
        """

        threshold = (
            self._threshold_upper if self._keep_tracking else self._threshold_lower
        )
        keep_tracking = (np.max(np.abs(hardpoint_error)) < threshold) and is_enabled

        if not (keep_tracking and self._keep_tracking):
            self._hardpoint_error_track = hardpoint_error.copy()

        self._keep_tracking = keep_tracking

        return self._hardpoint_error_track
