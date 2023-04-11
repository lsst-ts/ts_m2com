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

from ..enum import InnerLoopControlMode

__all__ = ["MockInnerLoopController"]


class MockInnerLoopController:
    """Mock Inner-Loop Controller (ILC) to simulate the behavior of hardware.

    Attributes
    ----------
    mode : enum `InnerLoopControlMode`
        mode of ILC.
    """

    def __init__(self) -> None:
        self.mode = InnerLoopControlMode.Standby

    def set_mode(self, mode: InnerLoopControlMode) -> None:
        """Set the mode.

        Notes
        -----
        This is translated from the SystemController.setILC_Mode.vi used in the
        simulation mode in ts_mtm2_cell.

        Parameters
        ----------
        mode : enum `InnerLoopControlMode`
            Inner-loop control mode.
        """

        if mode in (InnerLoopControlMode.NoChange, InnerLoopControlMode.Unknown):
            return

        elif mode == InnerLoopControlMode.ClearFaults:
            self.mode = InnerLoopControlMode.Standby

        else:
            self.mode = mode
