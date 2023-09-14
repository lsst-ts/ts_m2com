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

import re

from lsst.ts.tcpip import OneClientServer
from lsst.ts.xml.enums import MTM2

from ..constant import NUM_TEMPERATURE_EXHAUST, NUM_TEMPERATURE_INTAKE

__all__ = ["MockMessageEvent"]


class MockMessageEvent:
    """Mock message of event to simulate the message from real hardware.

    Parameters
    ----------
    server : `tcpip.OneClientServer` or None
        Command server.

    Attributes
    ----------
    server : `tcpip.OneClientServer` or None
        Command server.
    configuration_file : `str`
        Configuration file.
    """

    def __init__(self, server: OneClientServer | None) -> None:
        self.server = server

        self.configuration_file = (
            "Configurable_File_Description_20180831T092556_surrogate_handling.csv"
        )

    async def write_m2_assembly_in_position(self, in_position: bool) -> None:
        """Write the message: M2 assembly is in position or not.

        Parameters
        ----------
        in_position : `bool`
            M2 assembly is in position or not.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "m2AssemblyInPosition", "inPosition": in_position}
            )

    async def write_cell_temperature_high_warning(self, hi_warning: bool) -> None:
        """Write the message: cell temperature is high or not.

        Parameters
        ----------
        hi_warning : `bool`
            Cell temperature is high or not.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "cellTemperatureHiWarning", "hiWarning": hi_warning}
            )

    async def write_commandable_by_dds(self, state: bool) -> None:
        """Write the message: commandable by DDS or not.

        Parameters
        ----------
        state : `bool`
            Commandable by DDS or not.
        """

        if self.server is not None:
            await self.server.write_json({"id": "commandableByDDS", "state": state})

    async def write_interlock(self, state: bool) -> None:
        """Write the message: interlock.

        Parameters
        ----------
        state : `bool`
            Interlock is on or not.
        """

        if self.server is not None:
            await self.server.write_json({"id": "interlock", "state": state})

    async def write_tcp_ip_connected(self, is_connected: bool) -> None:
        """Write the message: TCP/IP connection is on or not.

        Parameters
        ----------
        is_connected : `bool`
            TCP/IP connection is on or not.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "tcpIpConnected", "isConnected": is_connected}
            )

    async def write_hardpoint_list(self, actuators: list[int]) -> None:
        """Write the message: hardpoint list.

        Parameters
        ----------
        actuators : `list`
            Hardpoint list.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "hardpointList", "actuators": actuators}
            )

    async def write_force_balance_system_status(self, status: bool) -> None:
        """Write the message: force balance system is on or not.

        Parameters
        ----------
        status : `bool`
            Force balance system is on or not.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "forceBalanceSystemStatus", "status": status}
            )

    async def write_inclination_telemetry_source(
        self, is_external_source: bool
    ) -> None:
        """Write the message: inclination telemetry source.

        Parameters
        ----------
        is_external_source : `bool`
            Is the external inclination telemetry source or not.
        """

        if self.server is not None:
            inclination_source = (
                MTM2.InclinationTelemetrySource.MTMOUNT
                if is_external_source
                else MTM2.InclinationTelemetrySource.ONBOARD
            )
            await self.server.write_json(
                {"id": "inclinationTelemetrySource", "source": int(inclination_source)},
            )

    async def write_temperature_offset(self, ring: list[float]) -> None:
        """Write the message: temperature offset in degree C.

        Parameters
        ----------
        ring : `list`
            Offset of ring temperatures.
        """

        # TODO: Remove the 'intake' and 'exhaust' after updating the ts_xml.
        # This will happen after finishing the update of ts_mtm2_cell. No
        # timeline yet.
        if self.server is not None:
            await self.server.write_json(
                {
                    "id": "temperatureOffset",
                    "ring": ring,
                    "intake": [0.0] * NUM_TEMPERATURE_INTAKE,
                    "exhaust": [0.0] * NUM_TEMPERATURE_EXHAUST,
                },
            )

    async def write_script_execution_status(self, percentage: int | float) -> None:
        """Write the message: script execution status.

        Parameters
        ----------
        percentage : `int` or `float`
            Percentage of the script execution.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "scriptExecutionStatus", "percentage": percentage}
            )

    async def write_digital_output(self, digital_output: int) -> None:
        """Write the message: digital output.

        Parameters
        ----------
        digital_output : `int`
            Digital output. The bit value can follow the enum 'DigitalOutput'.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "digitalOutput", "value": digital_output}
            )

    async def write_digital_input(self, digital_input: int) -> None:
        """Write the message: digital input.

        Parameters
        ----------
        digital_input : `int`
            Digital input. The bit value can follow the enum 'DigitalInput'.
        """

        if self.server is not None:
            await self.server.write_json({"id": "digitalInput", "value": digital_input})

    async def write_config(self) -> None:
        """Write the message: config."""

        if self.server is not None:
            (
                version,
                control_parameters,
                lut_parameters,
            ) = self._get_configuration_file_details()
            await self.server.write_json(
                {
                    "id": "config",
                    "configuration": self.configuration_file,
                    "version": version,
                    "controlParameters": control_parameters,
                    "lutParameters": lut_parameters,
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

    def _get_configuration_file_details(self) -> tuple[str, str, str]:
        """Get the details of configuration file.

        Returns
        -------
        `str`
            Version.
        control_parameters : `str`
            Control parameters.
        lut_parameters : `str`
            Look-up table parameters.
        """

        result = re.match(
            r"\S+_\S+_(\w+)_(\w+)_([a-zA-z]+).csv", self.configuration_file
        )
        if result is not None:
            control_parameters = (
                "CtrlParameterFiles_2018-07-19_104257_m2"
                if (result.groups()[1].lower() == "m2")
                else "CtrlParameterFiles_2018-07-19_104314_surg"
            )
            lut_parameters = (
                "FinalOpticalLUTs"
                if (result.groups()[2].lower() == "optical")
                else "FinalHandlingLUTs"
            )

            return result.groups()[0], control_parameters, lut_parameters

        return "", "", ""

    async def write_open_loop_max_limit(self, status: bool) -> None:
        """Write the message: open-loop maximum limit is enabled or not.

        Parameters
        ----------
        status : `bool`
            Open-loop maximum limit is enabled or not.
        """

        if self.server is not None:
            await self.server.write_json({"id": "openLoopMaxLimit", "status": status})

    async def write_limit_switch_status(
        self,
        limit_switch_retract: list[int],
        limit_switch_extend: list[int],
    ) -> None:
        """Write the message: limit switch status.

        Parameters
        ----------
        limit_switch_retract : `list`
            Triggered retracted limit switch.
        limit_switch_extend : `list`
            Triggered extended limit switch.
        """

        if self.server is not None:
            await self.server.write_json(
                {
                    "id": "limitSwitchStatus",
                    "retract": limit_switch_retract,
                    "extend": limit_switch_extend,
                },
            )

    async def write_power_system_state(
        self,
        power_type: MTM2.PowerType,
        status: bool,
        power_system_state: MTM2.PowerSystemState,
    ) -> None:
        """Write the message: power system state.

        Parameters
        ----------
        power_type : enum `MTM2.PowerType`
            Power type.
        status : `bool`
            Power status is on or not.
        power_system_state : enum `MTM2.PowerSystemState`
            Power system state.
        """

        if self.server is not None:
            await self.server.write_json(
                {
                    "id": "powerSystemState",
                    "powerType": int(power_type),
                    "status": status,
                    "state": int(power_system_state),
                },
            )

    async def write_closed_loop_control_mode(
        self, mode: MTM2.ClosedLoopControlMode
    ) -> None:
        """Write the message: closed-loop control mode.

        Parameters
        ----------
        mode : enum `MTM2.ClosedLoopControlMode`
            Closed-loop control mode.
        """

        if self.server is not None:
            await self.server.write_json(
                {
                    "id": "closedLoopControlMode",
                    "mode": int(mode),
                },
            )

    async def write_inner_loop_control_mode(
        self, address: int, mode: MTM2.InnerLoopControlMode
    ) -> None:
        """Write the message: inner-loop control mode.

        Parameters
        ----------
        address : `int`
            0-based address.
        mode : enum `MTM2.InnerLoopControlMode`
            Inner-loop control mode.
        """

        if self.server is not None:
            await self.server.write_json(
                {
                    "id": "innerLoopControlMode",
                    "address": address,
                    "mode": int(mode),
                },
            )

    async def write_summary_faults_status(self, status: int) -> None:
        """Write the message: summary faults status.

        Parameters
        ----------
        status : `int`
            Summary faults status.
        """

        if self.server is not None:
            await self.server.write_json(
                {"id": "summaryFaultsStatus", "status": status}
            )

    async def write_enabled_faults_mask(self, mask: int) -> None:
        """Write the message: enabled faults mask.

        Parameters
        ----------
        mask : `int`
            Enabled faults mask.
        """

        if self.server is not None:
            await self.server.write_json({"id": "enabledFaultsMask", "mask": mask})

    async def write_configuration_files(self) -> None:
        """Write the message: configuration files."""

        if self.server is not None:
            await self.server.write_json(
                {
                    "id": "configurationFiles",
                    "files": [
                        "Configurable_File_Description_20180831T091922_M2_optical.csv",
                        "Configurable_File_Description_20180831T092326_M2_handling.csv",
                        "Configurable_File_Description_20180831T092423_surrogate_optical.csv",
                        "Configurable_File_Description_20180831T092556_surrogate_handling.csv",
                    ],
                },
            )
