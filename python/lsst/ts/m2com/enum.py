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

import typing
from enum import Enum, IntEnum, auto

__all__ = [
    "MsgType",
    "CommandStatus",
    "DetailedState",
    "CommandScript",
    "CommandActuator",
    "ActuatorDisplacementUnit",
    "PowerType",
    "PowerSystemState",
    "DigitalOutput",
    "DigitalOutputStatus",
    "DigitalInput",
    "LimitSwitchType",
    "ClosedLoopControlMode",
    "InnerLoopControlMode",
    "MockErrorCode",
]


class BitEnum(Enum):
    """Bit enum to deal with some communication with M2 cell controller that
    use the bit value directly. You need to inherit this class to use it. The
    order is from bit 0 to bit n. For the values of fields, use auto()
    directly. By doing this, the values will be 1, 2, 4, ..., 2 ** n, etc."""

    @staticmethod
    def _generate_next_value_(
        name: str, start: int, count: int, last_values: list[typing.Any]
    ) -> int:
        """Override parent method to generate power of 2 sequence of numbers,
        starting from 1 (e.g. 1, 2, 4, 8, ...)."""
        return 2**count


class MsgType(IntEnum):
    """Message type to the cell controller."""

    Command = 1
    Event = auto()
    Telemetry = auto()


class CommandStatus(IntEnum):
    """Command status."""

    Success = 1
    Fail = auto()
    Ack = auto()
    NoAck = auto()
    Unknown = auto()


class DetailedState(IntEnum):
    """Detailed substates of the Offline state."""

    PublishOnly = 1
    Available = auto()


class CommandScript(IntEnum):
    """Action to command the script execution."""

    LoadScript = 1
    Clear = auto()
    Run = auto()
    Stop = auto()
    Pause = auto()
    Resume = auto()


class CommandActuator(IntEnum):
    """Action to command the actuators."""

    Start = 1
    Stop = auto()
    Pause = auto()
    Resume = auto()


class ActuatorDisplacementUnit(IntEnum):
    """Unit of the actuator displacement."""

    Millimeter = 1
    Step = auto()


class PowerType(IntEnum):
    """Type of the power."""

    Motor = 1
    Communication = auto()


class PowerSystemState(IntEnum):
    """State of the power system. This is copied from the ts_mtm2_cell."""

    Init = 1
    PoweredOff = auto()
    PoweringOn = auto()
    ResettingBreakers = auto()
    PoweredOn = auto()
    PoweringOff = auto()


class DigitalOutput(BitEnum):
    """Bit of digital output."""

    MotorPower = auto()
    CommunicationPower = auto()
    InterlockEnable = auto()
    ResetMotorBreakers = auto()
    ResetCommunicationBreakers = auto()
    SpareOutput_5 = auto()
    SpareOutput_6 = auto()
    SpareOutput_7 = auto()


class DigitalOutputStatus(IntEnum):
    """Digital output status to switch the individual bit value of
    digital output."""

    BinaryLowLevel = 1
    BinaryHighLevel = auto()
    ToggleBit = auto()


class DigitalInput(BitEnum):
    """Bit of digital input."""

    RedundancyOK = auto()
    LoadDistributionOK = auto()
    PowerSupplyDC_2_OK = auto()
    PowerSupplyDC_1_OK = auto()
    PowerSupplyCurrent_2_OK = auto()
    PowerSupplyCurrent_1_OK = auto()
    J1_W9_1_MotorPowerBreaker = auto()
    J1_W9_2_MotorPowerBreaker = auto()
    J1_W9_3_MotorPowerBreaker = auto()
    J2_W10_1_MotorPowerBreaker = auto()
    J2_W10_2_MotorPowerBreaker = auto()
    J2_W10_3_MotorPowerBreaker = auto()
    J3_W11_1_MotorPowerBreaker = auto()
    J3_W11_2_MotorPowerBreaker = auto()
    J3_W11_3_MotorPowerBreaker = auto()
    J1_W12_1_CommunicationPowerBreaker = auto()
    SpareInput_16 = auto()
    SpareInput_17 = auto()
    SpareInput_18 = auto()
    SpareInput_19 = auto()
    SpareInput_20 = auto()
    SpareInput_21 = auto()
    SpareInput_22 = auto()
    SpareInput_23 = auto()
    J1_W12_2_CommunicationPowerBreaker = auto()
    J2_W13_1_CommunicationPowerBreaker = auto()
    J2_W13_2_CommunicationPowerBreaker = auto()
    J3_W14_1_CommunicationPowerBreaker = auto()
    J3_W14_2_CommunicationPowerBreaker = auto()
    SpareInput_29 = auto()
    SpareInput_30 = auto()
    InterlockPowerReplay = auto()


class LimitSwitchType(IntEnum):
    """Type of the limit switches on actuators."""

    Retract = 1
    Extend = auto()


class ClosedLoopControlMode(IntEnum):
    """Closed loop control mode. This is copied from the ts_mtm2_cell."""

    Idle = 1
    TelemetryOnly = auto()
    OpenLoop = auto()
    ClosedLoop = auto()


class InnerLoopControlMode(IntEnum):
    """Inner-loop control mode. This is copied from the ts_mtm2_cell."""

    Standby = 1
    Disabled = auto()
    Enabled = auto()
    FirmwareUpdate = auto()
    Fault = auto()
    ClearFaults = auto()
    NoChange = auto()
    Unknown = auto()


class MockErrorCode(IntEnum):
    """Mock error code used by the mock server.

    The values here are based on the ts_m2gui/policy/error_code_m2.tsv.
    """

    NoError = 1
    MonitoringIlcReadError = 6052
    LostConnection = 6053
    LimitSwitchTriggeredClosedloop = 6056
    LimitSwitchTriggeredOpenloop = 6057
    TangentLoadCellFault = 6088
