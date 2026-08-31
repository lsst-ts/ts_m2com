# This file is part of ts_m2com.
#
# Developed for the Vera C. Rubin Observatory Telescope and Site Systems.
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
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

from lsst.ts.xml.enums import MTM2

__all__ = ["MockInnerLoopController"]


class MockInnerLoopController:
    """Mock Inner-Loop Controller (ILC) to simulate the behavior of hardware.

    Attributes
    ----------
    mode : enum `MTM2.InnerLoopControlMode`
        mode of ILC.
    """

    DEFAULT_SCAN_RATE = 8

    DEFAULT_GAIN = 1.455705e-05
    DEFAULT_OFFSET = 0.0
    DEFAULT_SENSITIVITY = 1.60876

    def __init__(self) -> None:
        self.mode = MTM2.InnerLoopControlMode.Standby

        self.scan_rate = self.DEFAULT_SCAN_RATE

        NUM_CHANNEL = 4
        self.gains = [0.0, self.DEFAULT_GAIN, 0.0, 0.0]
        self.offsets = [self.DEFAULT_OFFSET] * NUM_CHANNEL
        self.sensitivities = [self.DEFAULT_SENSITIVITY] * NUM_CHANNEL

    def set_mode(self, mode: MTM2.InnerLoopControlMode) -> None:
        """Set the mode.

        Notes
        -----
        This is translated from the SystemController.setILC_Mode.vi used in the
        simulation mode in ts_mtm2_cell.

        Parameters
        ----------
        mode : enum `MTM2.InnerLoopControlMode`
            Inner-loop control mode.
        """

        if mode in (
            MTM2.InnerLoopControlMode.NoChange,
            MTM2.InnerLoopControlMode.Unknown,
        ):
            return

        elif mode == MTM2.InnerLoopControlMode.ClearFaults:
            self.mode = MTM2.InnerLoopControlMode.Standby

        else:
            self.mode = mode

    def set_offset_and_sensitivity(self, channel: int, offset: float, sensitivity: float) -> None:
        """Set the offset and sensitivity.

        Parameters
        ----------
        channel : `int`
            0-based channel (0-3).
        offset : `float`
            Offset value.
        sensitivity : `float`
            Sensitivity value.
        """

        self.offsets[channel] = offset
        self.sensitivities[channel] = sensitivity
