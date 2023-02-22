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

from ..utility import write_json_packet

__all__ = ["MockMessageEvent"]


class MockMessageEvent:
    """Mock message of event to simulate the message from real hardware.

    Parameters
    ----------
    writer : `asyncio.StreamWriter` or None
        Writer of the socket.

    Attributes
    ----------
    writer : `asyncio.StreamWriter` or None
        Writer of the socket.
    """

    def __init__(self, writer):
        self.writer = writer

    async def write_m2_assembly_in_position(self, in_position):
        """Write the message: M2 assembly is in position or not.

        Parameters
        ----------
        in_position : `bool`
            M2 assembly is in position or not.
        """
        await write_json_packet(
            self.writer, {"id": "m2AssemblyInPosition", "inPosition": in_position}
        )

    async def write_cell_temperature_high_warning(self, hi_warning):
        """Write the message: cell temperature is high or not.

        Parameters
        ----------
        hi_warning : `bool`
            Cell temperature is high or not.
        """
        await write_json_packet(
            self.writer, {"id": "cellTemperatureHiWarning", "hiWarning": hi_warning}
        )

    async def write_detailed_state(self, detailed_state):
        """Write the message: detailed state.

        Parameters
        ----------
        detailed_state : enum `DetailedState`
            M2 detailed state.
        """
        await write_json_packet(
            self.writer, {"id": "detailedState", "detailedState": int(detailed_state)}
        )

    async def write_summary_state(self, summary_state):
        """Write the message: summary state.

        Parameters
        ----------
        summary_state : enum `lsst.ts.salobj.State`
            M2 summary state.
        """
        await write_json_packet(
            self.writer, {"id": "summaryState", "summaryState": int(summary_state)}
        )

    async def write_error_code(self, error_code):
        """Write the message: error code.

        Parameters
        ----------
        error_code : int
            Error code.
        """
        await write_json_packet(
            self.writer, {"id": "errorCode", "errorCode": int(error_code)}
        )

    async def write_commandable_by_dds(self, state):
        """Write the message: commandable by DDS or not.

        Parameters
        ----------
        state : `bool`
            Commandable by DDS or not.
        """
        await write_json_packet(self.writer, {"id": "commandableByDDS", "state": state})

    async def write_interlock(self, state):
        """Write the message: interlock.

        Parameters
        ----------
        state : `bool`
            Interlock is on or not.
        """
        await write_json_packet(self.writer, {"id": "interlock", "state": state})

    async def write_tcp_ip_connected(self, is_connected):
        """Write the message: TCP/IP connection is on or not.

        Parameters
        ----------
        is_connected : `bool`
            TCP/IP connection is on or not.
        """
        await write_json_packet(
            self.writer, {"id": "tcpIpConnected", "isConnected": is_connected}
        )

    async def write_hardpoint_list(self, actuators):
        """Write the message: hardpoint list.

        Parameters
        ----------
        actuators : `list`
            Hardpoint list.
        """
        await write_json_packet(
            self.writer, {"id": "hardpointList", "actuators": actuators}
        )

    async def write_force_balance_system_status(self, status):
        """Write the message: force balance system is on or not.

        Parameters
        ----------
        status : `bool`
            Force balance system is on or not.
        """
        await write_json_packet(
            self.writer, {"id": "forceBalanceSystemStatus", "status": status}
        )

    async def write_inclination_telemetry_source(self, source):
        """Write the message: inclination telemetry source.

        Parameters
        ----------
        source : enum `MTM2.InclinationTelemetrySource`
            Inclination telemetry source.
        """
        await write_json_packet(
            self.writer, {"id": "inclinationTelemetrySource", "source": int(source)}
        )

    async def write_temperature_offset(self, ring):
        """Write the message: temperature offset in degree C.

        Parameters
        ----------
        ring : `list`
            Offset of ring temperatures.
        """

        # TODO: Remove the 'intake' and 'exhaust' after updating the ts_xml.
        # This will happen after finishing the update of ts_mtm2_cell. No
        # timeline yet.
        await write_json_packet(
            self.writer,
            {
                "id": "temperatureOffset",
                "ring": ring,
                "intake": [0] * 2,
                "exhaust": [0] * 2,
            },
        )

    async def write_script_execution_status(self, percentage):
        """Write the message: script execution status.

        Parameters
        ----------
        percentage : `int` or `float`
            Percentage of the script execution.
        """
        await write_json_packet(
            self.writer, {"id": "scriptExecutionStatus", "percentage": percentage}
        )

    async def write_digital_output(self, digital_output):
        """Write the message: digital output.

        Parameters
        ----------
        digital_output : `int`
            Digital output. The bit value can follow the enum 'DigitalOutput'.
        """
        await write_json_packet(
            self.writer, {"id": "digitalOutput", "value": digital_output}
        )

    async def write_digital_input(self, digital_input):
        """Write the message: digital input.

        Parameters
        ----------
        digital_input : `int`
            Digital input. The bit value can follow the enum 'DigitalInput'.
        """
        await write_json_packet(
            self.writer, {"id": "digitalInput", "value": digital_input}
        )

    async def write_config(self):
        """Write the message: config."""
        await write_json_packet(
            self.writer,
            {
                "id": "config",
                "configuration": "Configurable_File_Description_surrogate_handling.csv",
                "version": "20180831T092556",
                "controlParameters": "CtrlParameterFiles_surg",
                "lutParameters": "FinalHandlingLUTs",
                "powerWarningMotor": 5.0,
                "powerFaultMotor": 10.0,
                "powerThresholdMotor": 20.0,
                "powerWarningComm": 5.0,
                "powerFaultComm": 10.0,
                "powerThresholdComm": 10.0,
                "inPositionAxial": 0.158,
                "inPositionTangent": 1.1,
                "inPositionSample": 1.0,
                "timeoutSal": 15.0,
                "timeoutCrio": 1.0,
                "timeoutIlc": 3,
                "inclinometerDelta": 2.0,
                "inclinometerDiffEnabled": True,
                "cellTemperatureDelta": 2.0,
            },
        )

    async def write_open_loop_max_limit(self, status):
        """Write the message: open-loop maximum limit is enabled or not.

        Parameters
        ----------
        status : `bool`
            Open-loop maximum limit is enabled or not.
        """
        await write_json_packet(
            self.writer, {"id": "openLoopMaxLimit", "status": status}
        )

    async def write_limit_switch_status(
        self, limit_switch_retract, limit_switch_extend
    ):
        """Write the message: limit switch status.

        Parameters
        ----------
        limit_switch_retract : `list`
            Triggered retracted limit switch.
        limit_switch_extend : `list`
            Triggered extended limit switch.
        """
        await write_json_packet(
            self.writer,
            {
                "id": "limitSwitchStatus",
                "retract": limit_switch_retract,
                "extend": limit_switch_extend,
            },
        )

    async def write_power_system_state(self, power_type, status, power_system_state):
        """Write the message: power system state.

        Parameters
        ----------
        power_type : enum `PowerType`
            Power type.
        status : `bool`
            Power status is on or not.
        power_system_state : enum `PowerSystemState`
            Power system state.
        """
        await write_json_packet(
            self.writer,
            {
                "id": "powerSystemState",
                "powerType": int(power_type),
                "status": status,
                "state": int(power_system_state),
            },
        )

    async def write_closed_loop_control_mode(self, mode):
        """Write the message: closed-loop control mode.

        Parameters
        ----------
        mode : enum `ClosedLoopControlMode`
            Closed-loop control mode.
        """
        await write_json_packet(
            self.writer,
            {
                "id": "closedLoopControlMode",
                "mode": int(mode),
            },
        )

    async def write_inner_loop_control_mode(self, address, mode):
        """Write the message: inner-loop control mode.

        Parameters
        ----------
        address : `int`
            0-based address.
        mode : enum `InnerLoopControlMode`
            Inner-loop control mode.
        """
        await write_json_packet(
            self.writer,
            {
                "id": "innerLoopControlMode",
                "address": address,
                "mode": int(mode),
            },
        )
