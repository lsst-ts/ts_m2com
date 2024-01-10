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

import asyncio
import json
import logging

from lsst.ts import tcpip
from lsst.ts.utils import make_done_future
from lsst.ts.xml.enums import MTM2

from ..enum import CommandStatus, LimitSwitchType, MockErrorCode
from ..utility import cancel_task_and_wait
from .mock_command import MockCommand
from .mock_message_event import MockMessageEvent
from .mock_message_telemetry import MockMessageTelemetry
from .mock_model import MockModel
from .mock_power_system import MockPowerSystem

__all__ = ["MockServer"]


class MockServer:
    """Mock server of M2.

    Parameters
    ----------
    host : `str`
        IP address for this server; typically "LOCALHOST_IPV4" for IP4
        or "LOCALHOST_IPV6" for IP6.
    port_command : `int`, optional
        IP port for the command server. (the default is 50000)
    port_telemetry : `int`, optional
        IP port for the telemetry server. (the default is 50001)
    timeout_in_second : `float`, optional
        Read timeout in second. (the default is 0.05)
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    is_csc : `bool`, optional
        Is called by the commandable SAL component (CSC) or not. (the default
        is True)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    model : `MockModel`
        Mock model to simulate the M2 hardware behavior.
    server_command : `tcpip.OneClientServer`
        Command server.
    server_telemetry : `tcpip.OneClientServer`
        Telemetry server.
    timeout_in_second : `float`
        Read timeout in second.
    """

    # 20 Hz (= 0.05 second)
    PERIOD_TELEMETRY_IN_SECOND = 0.05

    def __init__(
        self,
        host: str,
        port_command: int = 50000,
        port_telemetry: int = 50001,
        timeout_in_second: float = 0.05,
        log: logging.Logger | None = None,
        is_csc: bool = True,
    ) -> None:
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        # Instantiate the MockModel class and do the configuration
        self.model = MockModel(
            log=self.log, telemetry_interval=self.PERIOD_TELEMETRY_IN_SECOND
        )

        self.server_command = tcpip.OneClientServer(
            host,
            port_command,
            self.log,
            self._connect_state_changed_callback_command,
            name="Commands",
        )
        self.server_telemetry = tcpip.OneClientServer(
            host,
            port_telemetry,
            self.log,
            self._connect_state_changed_callback_telemetry,
            name="Telemetry",
        )

        self.timeout_in_second = timeout_in_second

        # Following two attributes have the type of asyncio.Future
        self._monitor_loop_task_command = make_done_future()
        self._monitor_loop_task_telemetry = make_done_future()

        # This is used to send the initial messages when the connection is just
        # on. Check the self._send_welcome_message() for the details.
        self._welcome_message_sent = False

        # Simulate the messages
        self._message_event: MockMessageEvent | None = None
        self._message_telemetry: MockMessageTelemetry | None = None

        self._is_csc = is_csc

        self._command = MockCommand()
        self._command_response = {
            "cmd_applyForces": self._command.apply_forces,
            "cmd_positionMirror": self._command.position_mirror,
            "cmd_resetForceOffsets": self._command.reset_force_offsets,
            "cmd_clearErrors": self._command.clear_errors,
            "cmd_switchForceBalanceSystem": self._command.switch_force_balance_system,
            "cmd_setTemperatureOffset": self._command.set_temperature_offset,
            "cmd_switchCommandSource": self._command.switch_command_source,
            "cmd_runScript": self._command.run_script,
            "cmd_moveActuators": self._command.move_actuators,
            "cmd_resetBreakers": self._command.reset_breakers,
            "cmd_rebootController": self._command.reboot_controller,
            "cmd_enableOpenLoopMaxLimit": self._command.enable_open_loop_max_limit,
            "cmd_saveMirrorPosition": self._command.save_mirror_position,
            "cmd_setMirrorHome": self._command.set_mirror_home,
            "cmd_switchDigitalOutput": self._command.switch_digital_output,
            "cmd_power": self._command.power,
            "cmd_resetActuatorSteps": self._command.reset_actuator_steps,
            "cmd_setClosedLoopControlMode": self._command.set_closed_loop_control_mode,
            "cmd_setInnerLoopControlMode": self._command.set_inner_loop_control_mode,
            "cmd_getInnerLoopControlMode": self._command.get_inner_loop_control_mode,
            "cmd_loadConfiguration": self._command.load_configuration,
            "cmd_setControlParameters": self._command.set_control_parameters,
            "cmd_setEnabledFaultsMask": self._command.set_enabled_faults_mask,
            "cmd_setConfigurationFile": self._command.set_configuration_file,
            "cmd_setHardpointList": self._command.set_hardpoint_list,
        }

    async def _connect_state_changed_callback_command(
        self, server_command: tcpip.OneClientServer
    ) -> None:
        """Called when the command server connection state changes.

        Notes
        -----
        This function needs to be asynchronous because of the upstream
        OneClientServer in ts_tcpip.

        Parameters
        ----------
        server_command : `tcpip.OneClientServer`
            Command server.
        """
        self._monitor_loop_task_command.cancel()

        if self.server_command.connected:
            self._message_event = MockMessageEvent(self.server_command)

            self._monitor_loop_task_command = asyncio.create_task(
                self._monitor_message_command()
            )

    async def _monitor_message_command(self, counts_min: int = 5) -> None:
        """Monitor the message from command server.

        Parameters
        ----------
        counts_min : `int`, optional
            Minimum counts to decide to update the actuator steps or not. (the
            default is 5)
        """

        # Decide the counts per second to update the actuator steps
        counts_per_second = int(1 / self.PERIOD_TELEMETRY_IN_SECOND)
        counts_per_second = (
            counts_per_second if counts_per_second >= counts_min else counts_min
        )

        try:
            count = 0
            update_steps = False
            while self.server_command.connected:
                if not self._welcome_message_sent:
                    await self._send_welcome_message()
                    self._welcome_message_sent = True

                await self._monitor_and_report_system_status()

                await asyncio.sleep(self.PERIOD_TELEMETRY_IN_SECOND)

                await self._process_message_command()

                await self._run_and_report_script_engine_status()
                self._run_control_open_loop()

                # Balance the forces and steps. Because the calculation of
                # steps takes some CPU resource, we will do it in a slow pace
                # to decrease the CPU usage.
                is_updated = self.model.balance_forces_and_steps(
                    force_rms=0.1, update_steps=update_steps
                )

                # Decide the value of update_steps
                if is_updated:
                    update_steps = False

                if count >= counts_per_second:
                    count = 0
                    update_steps = True
                else:
                    count += 1

                # Check the force error
                self._check_error_force()

                # Check the inclinometer error. Do not check this for CSC
                # simulation. Otherwise, the test stand will have the trouble
                # because the MTMount CSC is running as well.
                if not self._is_csc:
                    self._check_error_inclinometer()

            self.log.info("Command server disconnected; stopping monitor loop.")

        except ConnectionError:
            self.log.info("Command reader disconnected.")
            self._monitor_loop_task_command.cancel()

        except asyncio.IncompleteReadError:
            self.log.info("EOF is reached.")

        await self.server_command.close_client()

        self.model.script_engine.pause()
        self.model.script_engine.clear()

    async def _send_welcome_message(self) -> None:
        """Send the welcome message to describe the system status and
        configuration..

        Most of messages are just the events.
        """

        # Workaround of the mypy checking
        assert self._message_event is not None

        await self._message_event.write_tcp_ip_connected(True)
        await self._message_event.write_commandable_by_dds(True)

        # Note the index begins from 0 in Python
        hardpoints = [
            (hardpoint + 1) for hardpoint in self.model.control_closed_loop.hardpoints
        ]
        await self._message_event.write_hardpoint_list(hardpoints)

        await self._message_event.write_interlock(False)

        # Workaround of the mypy checking
        is_external_source = (
            self.model.control_parameters["use_external_elevation_angle"] is True
        )

        await self._message_event.write_inclination_telemetry_source(is_external_source)

        await self._message_event.write_temperature_offset(
            self.model.control_closed_loop.temperature["ref"],
        )

        # Send the digital input and output
        digital_input = self.model.get_digital_input()
        await self._message_event.write_digital_input(digital_input)

        digital_output = self.model.get_digital_output()
        await self._message_event.write_digital_output(digital_output)

        await self._message_event.write_config()

        await self._message_event.write_closed_loop_control_mode(
            MTM2.ClosedLoopControlMode.Idle
        )

        await self._message_event.write_enabled_faults_mask(
            self.model.error_handler.enabled_faults_mask
        )

        await self._message_event.write_configuration_files()

    async def _monitor_and_report_system_status(self) -> None:
        """Monitor the system status and report the specific events."""

        # Workaround of the mypy checking
        assert self._message_event is not None

        if self.model.control_closed_loop.is_cell_temperature_high():
            await self._message_event.write_cell_temperature_high_warning(True)

        if self.model.in_position:
            await self._message_event.write_m2_assembly_in_position(True)

        # Report the error code and related events
        error_handler = self.model.error_handler
        if error_handler.exists_new_error():
            await self._message_event.write_force_balance_system_status(
                self.model.control_closed_loop.is_running
            )

        if error_handler.exists_new_error() or error_handler.exists_new_warning():
            await self._report_summary_faults_status()

        # Report the triggered limit switches
        if error_handler.exists_new_limit_switch(
            LimitSwitchType.Retract
        ) or error_handler.exists_new_limit_switch(LimitSwitchType.Extend):
            limit_switches_retract = error_handler.get_limit_switches_to_report(
                LimitSwitchType.Retract
            )
            limit_switches_extend = error_handler.get_limit_switches_to_report(
                LimitSwitchType.Extend
            )
            await self._message_event.write_limit_switch_status(
                sorted(limit_switches_retract), sorted(limit_switches_extend)
            )

    async def _report_summary_faults_status(self) -> None:
        """Report the summary faults status."""

        # Workaround of the mypy checking
        assert self._message_event is not None

        status = self.model.error_handler.get_summary_faults_status_to_report()
        await self._message_event.write_summary_faults_status(status)

    async def _process_message_command(self) -> None:
        """Process the incoming message from command server."""

        try:
            msg = await asyncio.wait_for(
                self.server_command.read_json(),
                self.timeout_in_second,
            )

            # Process the command
            name = msg["id"]
            if self._is_command(name):
                # Acknowledge the command
                sequence_id = msg["sequence_id"]
                command_name = msg["id"]
                if command_name in self._command_response.keys():
                    await self._acknowledge_command(sequence_id)

                command_status = await self._process_command(msg)

                # Command result
                await self._reply_command(sequence_id, command_status)

                # Need to shutdown the server
                if (command_name == "cmd_rebootController") and (
                    command_status == CommandStatus.Success
                ):
                    await asyncio.sleep(0.5)
                    await self.close()

            # Process the event
            if self._is_event(name):
                self._get_event_data(msg)

        except asyncio.TimeoutError:
            pass

        except json.JSONDecodeError:
            self.log.exception("Error when decoding message in command server.")

        except asyncio.IncompleteReadError:
            raise

    def _is_command(self, message_name: str) -> bool:
        """Is the command or not.

        Parameters
        ----------
        message_name : `str`
            Message name in the header.

        Returns
        -------
        `bool`
            True if this is a command.
        """

        return message_name.startswith("cmd_")

    async def _acknowledge_command(self, sequence_id: int) -> None:
        """Acknowledge the command with the sequence ID.

        Parameters
        ----------
        sequence_id : `int`
            Sequence ID that should be >= 0.
        """

        id_ack = CommandStatus.Ack
        msg_ack = {"id": id_ack.name.lower(), "sequence_id": sequence_id}
        await self.server_command.write_json(msg_ack)

    async def _process_command(self, message: dict) -> CommandStatus:
        """Process the command.

        Parameters
        ----------
        message : `dict`
            Command message.

        Returns
        -------
        `CommandStatus`
            Status of command execution.
        """

        # Workaround of the mypy checking
        assert self._message_event is not None

        command_name = message["id"]
        available_commands = list(self._command_response.keys())
        if command_name in available_commands:
            self.model, command_status = await self._command_response[command_name](
                message, self.model, self._message_event
            )
            return command_status

        else:
            self.log.debug(
                f"Unrecognized command: {command_name}. Must be one of {available_commands}."
            )
            return CommandStatus.NoAck

    async def _reply_command(
        self, sequence_id: int, command_status: CommandStatus
    ) -> None:
        """Reply the command with the sequence ID.

        Parameters
        ----------
        sequence_id : `int`
            Sequence ID that should be >= 0.
        command_status : `CommandStatus`
            Status of command execution.
        """

        msg_command_status = {
            "id": command_status.name.lower(),
            "sequence_id": sequence_id,
        }
        await self.server_command.write_json(msg_command_status)

    def _is_event(self, message_name: str) -> bool:
        """Is the event or not.

        Parameters
        ----------
        message_name : `str`
            Message name in the header.

        Returns
        -------
        `bool`
            True if this is a event.
        """

        return message_name.startswith("evt_")

    def _get_event_data(self, message: dict) -> None:
        """Get the event data.

        Parameters
        ----------
        message : `dict`
            Event message.
        """

        # In the real M2 LabVIEW code, we will compare the string with the
        # lower case
        name = message["id"].lower()
        component = message["compName"].lower()
        if name == "evt_mountinposition" and component == "mtmount":
            self.model.mtmount_in_position = message["inPosition"]

    async def _run_and_report_script_engine_status(
        self, steps: int | float = 1
    ) -> None:
        """Run the script engine and report the status.

        Parameters
        ----------
        steps : `int` or `float`, optional
            Steps to run. The value should be between 0 and 100. (the default
            is 1)
        """

        # Workaround of the mypy checking
        assert self._message_event is not None

        script_engine = self.model.script_engine
        if script_engine.is_running:
            try:
                script_engine.run_steps(steps)

            except Exception as error:
                self.log.debug(f"Error when run the script: {error}")

            await self._message_event.write_script_execution_status(
                script_engine.percentage
            )

    def _run_control_open_loop(self, steps: int = 500) -> None:
        """Run the open-loop control.

        Parameters
        ----------
        steps : `int`, optional
            Steps to run. (the default is 500)
        """

        control_open_loop = self.model.control_open_loop
        if control_open_loop.is_running:
            try:
                control_open_loop.run_steps(steps)

            except Exception as error:
                self.log.debug(f"Error when run the open-loop control: {error}")

    def _check_error_force(self) -> None:
        """Check the force error and fault the system if needed."""

        # Check the actutor force
        (
            is_out_limit_actuator_force,
            error_code_actuator_force,
            limit_switches_retract,
            limit_switches_extend,
        ) = self.model.is_actuator_force_out_limit()

        if is_out_limit_actuator_force:
            self.model.fault(error_code_actuator_force)

        # Check the force error of tangent links
        (
            is_out_limit_force_error_tangent,
            error_code_force_error_tangent,
        ) = self.model.is_force_error_tangent_out_limit()

        if is_out_limit_force_error_tangent:
            self.model.fault(error_code_force_error_tangent)

        # Only trigger the error of limit switch if the open-loop maximum is
        # enabled
        if self.model.control_open_loop.open_loop_max_limit_is_enabled:
            for switch in limit_switches_retract:
                self.model.error_handler.add_new_limit_switch(
                    switch, LimitSwitchType.Retract
                )

            for switch in limit_switches_extend:
                self.model.error_handler.add_new_limit_switch(
                    switch, LimitSwitchType.Extend
                )

    def _check_error_inclinometer(self) -> None:
        """Check the inclinometer error and fault the system if needed."""

        control_parameters = self.model.control_parameters
        if control_parameters["enable_angle_comparison"]:
            control_open_loop = self.model.control_open_loop
            lut_angle = control_open_loop.correct_inclinometer_angle(
                control_open_loop.inclinometer_angle
            )

            if (
                abs(self.model.inclinometer_angle_external - lut_angle)
                > control_parameters["max_angle_difference"]
            ):
                self.model.fault(MockErrorCode.InclinometerDifference)

    async def _connect_state_changed_callback_telemetry(
        self, server_telemetry: tcpip.OneClientServer
    ) -> None:
        """Called when the telemetry server connection state changes.

        Notes
        -----
        This function needs to be asynchronous because of the upstream
        OneClientServer in ts_tcpip.

        Parameters
        ----------
        server_telemetry : `tcpip.OneClientServer`
            Telemetry server.
        """
        self._monitor_loop_task_telemetry.cancel()

        if self.server_telemetry.connected:
            self._message_telemetry = MockMessageTelemetry(self.server_telemetry)

            self._monitor_loop_task_telemetry = asyncio.create_task(
                self._monitor_message_telemetry()
            )

    async def _monitor_message_telemetry(self) -> None:
        """Monitor the message of incoming telemetry."""

        try:
            while self.server_telemetry.connected:
                await self._write_message_telemetry()
                await asyncio.sleep(self.PERIOD_TELEMETRY_IN_SECOND)

                await self._process_message_telemetry()

            self.log.info("Telemetry server disconnected; stopping monitor loop.")

        except ConnectionError:
            self.log.info("Telemetry reader disconnected.")
            self._monitor_loop_task_telemetry.cancel()

        except asyncio.IncompleteReadError:
            self.log.exception("EOF is reached.")

        await self.server_telemetry.close_client()

    async def _write_message_telemetry(self) -> None:
        """Write the telemetry message."""

        # Workaround of the mypy checking
        assert self._message_telemetry is not None

        telemetry_data = self.model.get_telemetry_data()

        await self._message_telemetry.write_power_status_raw(
            telemetry_data["powerStatusRaw"]
        )
        await self._message_telemetry.write_power_status(telemetry_data["powerStatus"])
        await self._message_telemetry.write_displacement_sensors(
            telemetry_data["displacementSensors"]
        )

        if self.model.power_motor.is_power_on():
            await self._message_telemetry.write_ilc_data(telemetry_data["ilcData"])
            await self._message_telemetry.write_net_forces_total(
                telemetry_data["netForcesTotal"]
            )
            await self._message_telemetry.write_net_moments_total(
                telemetry_data["netMomentsTotal"]
            )
            await self._message_telemetry.write_axial_force(
                telemetry_data["axialForce"]
            )
            await self._message_telemetry.write_tangent_force(
                telemetry_data["tangentForce"]
            )
            await self._message_telemetry.write_force_balance(
                telemetry_data["forceBalance"]
            )
            await self._message_telemetry.write_position(telemetry_data["position"])
            await self._message_telemetry.write_position_ims(
                telemetry_data["positionIMS"]
            )
            await self._message_telemetry.write_temperature(
                telemetry_data["temperature"]
            )
            await self._message_telemetry.write_zenith_angle(
                telemetry_data["zenithAngle"]
            )

            await self._message_telemetry.write_axial_encoder_positions(
                telemetry_data["axialEncoderPositions"]
            )
            await self._message_telemetry.write_tangent_encoder_positions(
                telemetry_data["tangentEncoderPositions"]
            )
            await self._message_telemetry.write_axial_actuator_steps(
                telemetry_data["axialActuatorSteps"]
            )
            await self._message_telemetry.write_tangent_actuator_steps(
                telemetry_data["tangentActuatorSteps"]
            )
            await self._message_telemetry.write_force_error_tangent(
                telemetry_data["forceErrorTangent"]
            )
            await self._message_telemetry.write_inclinometer_angle_tma(
                telemetry_data["inclinometerAngleTma"]
            )

    async def _process_message_telemetry(self) -> None:
        """Read and process data from telemetry server."""

        try:
            msg_tel = await asyncio.wait_for(
                self.server_telemetry.read_json(),
                self.timeout_in_second,
            )

            # In the real M2 LabVIEW code, we will compare the string with the
            # lower case
            name = msg_tel["id"].lower()
            component = msg_tel["compName"].lower()
            if (name == "tel_elevation") and (component == "mtmount"):
                self.model.set_inclinometer_angle(
                    msg_tel["actualPosition"], is_external=True
                )

        except asyncio.TimeoutError:
            pass

        except json.JSONDecodeError:
            self.log.exception("Error when decoding message in telemetry server.")

        except asyncio.IncompleteReadError:
            raise

    def are_servers_connected(self) -> bool:
        """The command and telemetry sockets are connected or not.

        Returns
        -------
        `bool`
            True if servers are connected. Else, False.
        """
        return self.server_command.connected and self.server_telemetry.connected

    async def start(self) -> None:
        """Start the command and telemetry TCP/IP servers."""
        await asyncio.gather(
            self.server_command.start_task, self.server_telemetry.start_task
        )

        self.model.error_handler.add_new_error(MockErrorCode.LostConnection.value)

    async def close(self) -> None:
        """Cancel the tasks and close the connections.

        Note: this function is safe to call even though there is no connection.
        """

        # Close the connections
        await self.server_command.close()
        await self.server_telemetry.close()

        # Cancel the tasks
        await cancel_task_and_wait(self._monitor_loop_task_command)
        await cancel_task_and_wait(self._monitor_loop_task_telemetry)

        # Reset some attributes
        self._welcome_message_sent = False

        self._message_event = None
        self._message_telemetry = None

        self.model.control_open_loop.open_loop_max_limit_is_enabled = False
        self.model.control_closed_loop.is_running = False

        await self._power_off_fully(self.model.power_motor)
        await self._power_off_fully(self.model.power_communication)

        self.model.error_handler.clear()

        self.model.script_engine.pause()
        self.model.script_engine.clear()

    async def _power_off_fully(self, power_system: MockPowerSystem) -> None:
        """Fully power off the system.

        Parameters
        ----------
        power_system : `MockPowerSystem`
            Power system.
        """

        await power_system.power_off()
        await power_system.wait_power_fully_off()
