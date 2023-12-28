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
import logging
import time
import typing

import numpy as np
from lsst.ts.utils import make_done_future
from lsst.ts.xml.enums import MTM2

from .constant import (
    DEFAULT_ENABLED_FAULTS_MASK,
    NUM_ACTUATOR,
    NUM_INNER_LOOP_CONTROLLER,
)
from .enum import (
    ActuatorDisplacementUnit,
    CommandActuator,
    CommandScript,
    CommandStatus,
    DigitalOutputStatus,
    MsgType,
)
from .error_handler import ErrorHandler
from .tcp_client import TcpClient
from .utility import camel_case, cancel_task_and_wait, get_config_dir

__all__ = ["Controller"]


class Controller:
    """Controller class.

    Parameters
    ----------
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    error_handler : `ErrorHandler`
        Error handler.
    client_command : `TcpClient` or None
        Command client.
    client_telemetry : `TcpClient` or None
        Telemetry client.
    last_command_status : `CommandStatus`
        Last command status.
    timeout : `float`
        Time limit for reading data from the TCP/IP interface (sec).
    power_system_status : `dict`
        Power system status in the cell controller.
    closed_loop_control_mode : enum `MTM2.ClosedLoopControlMode`
        Closed loop control mode in the cell controller.
    ilc_modes : `numpy.ndarray [MTM2.InnerLoopControlMode]`
        Modes of the inner-loop controller (ILC).
    control_parameters : `dict`
        Control parameters in the closed-loop controller (CLC).
    """

    def __init__(
        self,
        log: logging.Logger | None = None,
    ) -> None:
        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.error_handler = self._get_error_handler()

        self.client_command: TcpClient | None = None
        self.client_telemetry: TcpClient | None = None

        self.last_command_status = CommandStatus.Unknown

        self.power_system_status = {
            "motor_power_is_on": False,
            "motor_power_state": MTM2.PowerSystemState.Init,
            "communication_power_is_on": False,
            "communication_power_state": MTM2.PowerSystemState.Init,
        }

        self.closed_loop_control_mode = MTM2.ClosedLoopControlMode.Idle

        self.ilc_modes = np.array([])
        self.set_ilc_modes_to_unknown()

        self.control_parameters = {
            "enable_lut_temperature": True,
            "enable_lut_inclinometer": True,
            "use_external_elevation_angle": False,
            "enable_angle_comparison": False,
            "max_angle_difference": 2.0,
        }

        # Start the connection task or not
        self._start_connection = False

        # Task to do the connection (asyncio.Future)
        self._task_connection = make_done_future()

        # Callback functions and related arguments to process the
        # event and telemetry
        self._callback_process_event: typing.Callable[
            ..., typing.Coroutine
        ] | None = None
        self._args_callback_process_event: typing.Any = None

        self._callback_process_telemetry: typing.Callable[
            ..., typing.Coroutine
        ] | None = None
        self._args_callback_process_telemetry: typing.Any = None

        # Callback function to deal with the lost of connection
        self._callback_process_lost_connection: typing.Callable[
            ..., typing.Coroutine
        ] | None = None
        self._args_callback_process_lost_connection: typing.Any = None

    def _get_error_handler(self, filename: str = "error_code.tsv") -> ErrorHandler:
        """Get the error handler.

        Parameters
        ----------
        filename : `str`, optional
            Name of the error code file. (the default is "error_code.tsv")

        Returns
        -------
        error_handler : `ErrorHandler`
            Error handler.
        """

        error_handler = ErrorHandler()
        error_handler.read_error_list_file(get_config_dir() / filename)

        return error_handler

    def set_ilc_modes_to_unknown(self) -> None:
        """Set the inner-loop controller (ILC) modes to unknown."""

        self.ilc_modes = np.array(
            [MTM2.InnerLoopControlMode.Unknown] * NUM_INNER_LOOP_CONTROLLER
        )

    def are_ilc_modes_enabled(self) -> bool:
        """All the inner-loop controller (ILC) modes are enabled or not.

        Returns
        -------
        `bool`
            True if all ILC modes are enabled. Otherwise, False.
        """

        return np.all(self.ilc_modes == MTM2.InnerLoopControlMode.Enabled)

    def set_callback_process_event(
        self, process_event: typing.Callable[..., typing.Coroutine], *args: typing.Any
    ) -> None:
        """Set the callback function to process the event.

        Parameters
        ----------
        process_event : `coroutine`
            Function to process the event. It must has a keyward argument of
            "message" to receive the event as a dictionary.
        args : `args`
            Arguments needed in "process_event" function call.
        """

        self._callback_process_event = process_event
        self._args_callback_process_event = args

    def set_callback_process_telemetry(
        self,
        process_telemetry: typing.Callable[..., typing.Coroutine],
        *args: typing.Any,
    ) -> None:
        """Set the callback function to process the telemetry.

        Parameters
        ----------
        process_telemetry : `coroutine`
            Function to process the telemetry. It must has a keyward argument
            of "message" to receive the telemetry as a dictionary.
        args : `args`
            Arguments needed in "process_telemetry" function call.
        """

        self._callback_process_telemetry = process_telemetry
        self._args_callback_process_telemetry = args

    def set_callback_process_lost_connection(
        self,
        process_lost_connection: typing.Callable[..., typing.Coroutine],
        *args: typing.Any,
    ) -> None:
        """Set the callback function to process the lost of connection.

        Parameters
        ----------
        process_lost_connection : `coroutine`
            Function to process the lost of connection.
        args : `args`
            Arguments needed in "process_lost_connection" function call.
        """

        self._callback_process_lost_connection = process_lost_connection
        self._args_callback_process_lost_connection = args

    def start(
        self,
        host: str,
        port_command: int,
        port_telemetry: int,
        sequence_generator: typing.Generator | None = None,
        timeout: float = 10.0,
    ) -> None:
        """Start the task and connection.

        Parameters
        ----------
        host : `str`
            Host address.
        port_command : `int`
            IP port for the command server.
        port_telemetry : `int`
            IP port for the telemetry server.
        sequence_generator : `generator` or `None`, optional
            Sequence generator. (the default is None)
        timeout : `float`, optional
            Connection timeout in second. (default is 10.0)
        """

        # Create the tasks
        self._start_connection = True

        self._task_connection = asyncio.create_task(
            self._connect(
                host,
                port_command,
                port_telemetry,
                sequence_generator,
                timeout,
            )
        )

    async def _connect(
        self,
        host: str,
        port_command: int,
        port_telemetry: int,
        sequence_generator: typing.Generator | None,
        timeout: float,
    ) -> None:
        """Connect to the servers.

        Parameters
        ----------
        host : `str`
            Host address.
        port_command : `int`
            IP port for the command server.
        port_telemetry : `int`
            IP port for the telemetry server.
        sequence_generator : `generator` or `None`
            Sequence generator.
        timeout : `float`
            Connection timeout in second.
        """

        # Workaround the mypy check
        assert self._callback_process_telemetry is not None
        assert self._callback_process_lost_connection is not None

        self.log.info("Begin the connection loop with servers.")

        were_clients_connected = False
        while self._start_connection:
            if not self.are_clients_connected():
                if were_clients_connected:
                    were_clients_connected = False
                    self.log.info("Lost the TCP/IP connection in the connection loop.")

                    # Process the lost of connection
                    await self._callback_process_lost_connection(
                        *self._args_callback_process_lost_connection
                    )

                    await self._close_clients()

                else:
                    # Always use the new instances to request new connections
                    self.client_command = TcpClient(
                        host,
                        port_command,
                        self._analyze_command_status_and_event,
                        log=self.log,
                        sequence_generator=sequence_generator,
                        name="command",
                    )
                    self.client_telemetry = TcpClient(
                        host,
                        port_telemetry,
                        self._callback_process_telemetry,
                        *self._args_callback_process_telemetry,
                        log=self.log,
                        name="telemetry",
                        check_message_rate=True,
                    )

                    try:
                        await asyncio.gather(
                            self.client_command.connect(timeout=timeout),
                            self.client_telemetry.connect(timeout=timeout),
                        )
                        self.log.info("Servers are connected.")

                        were_clients_connected = True

                    except Exception as error:
                        self.log.debug(
                            "Error when connecting to servers - "
                            f"{self.client_command.host}:{self.client_command.port} "
                            f"and/or {self.client_telemetry.host}:{self.client_telemetry.port}: {str(error)}"
                        )
                        await self.close()

            await asyncio.sleep(1)

        self.log.info("Stop the connection loop with servers.")

    async def _analyze_command_status_and_event(
        self, message: dict | None = None
    ) -> None:
        """Analyze the command status and event.

        Parameters
        ----------
        message : `dict` or None, optional
            Incoming message. (the default is None)
        """

        if message is not None:
            self.log.debug(f"Receive the event message: {message}")

            if self._is_command_status(message):
                self.last_command_status = self._get_command_status(message)
                return

            # Check and update the power status
            self._update_power_status(message)

            # Check and update the closed-loop control mode
            self._update_closed_loop_control_mode(message)

            # Check and update the inner-loop control mode
            self._update_inner_loop_control_mode(message)

            # Add the received error code to the error handler
            self._process_error_code(message)

            # Process the summary faults status and update the internal state
            list_event_error_code = self._process_summary_faults_status(message)

            # Process the event with the callback function
            await self._process_event_with_callback(message)

            for event_error_code in list_event_error_code:
                await self._process_event_with_callback(event_error_code)

    def _is_command_status(self, message: dict) -> bool:
        """Is the command status or not.

        Parameters
        ----------
        message : `dict`
            Incoming message.

        Returns
        -------
        bool
            True if the message is related to the command status. Else, False.
        """

        message_name = message["id"]
        if message_name in [
            CommandStatus.Success.name.lower(),
            CommandStatus.Fail.name.lower(),
            CommandStatus.Ack.name.lower(),
            CommandStatus.NoAck.name.lower(),
        ]:
            return True
        else:
            return False

    def _get_command_status(self, message: dict) -> CommandStatus:
        """Get the command status.

        Parameters
        ----------
        message : `dict`
            Incoming message.

        Returns
        -------
        `CommandStatus`
            Command status.
        """

        # Workaround of the mypy checking
        assert self.client_command is not None

        message_name = message["id"]
        sequence_id = message["sequence_id"]
        if sequence_id == self.client_command.last_sequence_id:
            return self._get_command_status_from_message_name(message_name)
        else:
            self.log.info(
                f"Get the command status = {message_name}. The sequence id = {sequence_id}."
            )
            return CommandStatus.Unknown

    def _get_command_status_from_message_name(self, message_name: str) -> CommandStatus:
        """Get the command status from message name.

        Parameters
        ----------
        message_name : `str`
            Message name.

        Returns
        -------
        `CommandStatus`
            Command status.
        """

        if message_name == CommandStatus.Success.name.lower():
            return CommandStatus.Success
        elif message_name == CommandStatus.Fail.name.lower():
            return CommandStatus.Fail
        elif message_name == CommandStatus.Ack.name.lower():
            return CommandStatus.Ack
        elif message_name == CommandStatus.NoAck.name.lower():
            return CommandStatus.NoAck
        else:
            return CommandStatus.Unknown

    def _update_power_status(self, message: dict) -> None:
        """Check and update the power status.

        Parameters
        ----------
        message : `dict`
            Incoming message.
        """

        if message["id"] == "powerSystemState":
            power_type = None
            state = None
            try:
                power_type_value = message["powerType"]
                state_value = message["state"]

                power_type = MTM2.PowerType(power_type_value)
                state = MTM2.PowerSystemState(state_value)
            except ValueError:
                self.log.debug(
                    f"Get the unknown power type ({power_type_value}) or "
                    f"state ({state_value})."
                )

            if (power_type == MTM2.PowerType.Motor) and (state is not None):
                self.power_system_status["motor_power_is_on"] = message["status"]
                self.power_system_status["motor_power_state"] = state

            elif (power_type == MTM2.PowerType.Communication) and (state is not None):
                self.power_system_status["communication_power_is_on"] = message[
                    "status"
                ]
                self.power_system_status["communication_power_state"] = state

    def _update_closed_loop_control_mode(self, message: dict) -> None:
        """Check and update the closed-loop control mode.

        Parameters
        ----------
        message : `dict`
            Incoming message.
        """

        if message["id"] == "closedLoopControlMode":
            try:
                mode = message["mode"]
                self.closed_loop_control_mode = MTM2.ClosedLoopControlMode(mode)
            except ValueError:
                self.log.debug(f"Get the unknown closed-loop control mode ({mode}).")

    def _update_inner_loop_control_mode(self, message: dict) -> None:
        """Check and update the inner-loop control mode.

        Parameters
        ----------
        message : `dict`
            Incoming message.
        """

        if message["id"] == "innerLoopControlMode":
            try:
                address = message["address"]
                mode = message["mode"]
                self.ilc_modes[address] = MTM2.InnerLoopControlMode(mode)
            except (IndexError, ValueError):
                self.log.debug(
                    f"Get the unknown address ({address}) or inner "
                    f"loop control mode ({mode})."
                )

    def _process_error_code(self, message: dict) -> None:
        """Process the error code.

        Note. Remove this function after we get ride of ts_mtm2 on summit.

        Parameters
        ----------
        message : `dict`
            Incoming message.
        """

        if message["id"] == "errorCode":
            code = message["errorCode"]

            # Code would be either error or warning, but not both.
            if self.error_handler.is_error(code):
                self.error_handler.add_new_error(code)

            if self.error_handler.is_warning(code):
                self.error_handler.add_new_warning(code)

    def _process_summary_faults_status(self, message: dict) -> list[dict]:
        """Process the summary faults status. The decoded bit will be put into
        the 'errorCode' event.

        Parameters
        ----------
        message : `dict`
            Incoming message.

        Returns
        -------
        list_event_error_code : `list`
            List of the error code event.
        """

        list_event_error_code = list()
        if message["id"] == "summaryFaultsStatus":
            self.error_handler.decode_summary_faults_status(message["status"])

            # Get the error and warning codes
            # Note the union() will return a new set object
            codes: set[int] = set()
            if self.error_handler.exists_new_error():
                codes = codes.union(self.error_handler.get_errors_to_report())

            if self.error_handler.exists_new_warning():
                codes = codes.union(self.error_handler.get_warnings_to_report())

            # Put the message into the queue for CSC/GUI to publish
            for code in codes:
                list_event_error_code.append({"id": "errorCode", "errorCode": code})

        return list_event_error_code

    async def _process_event_with_callback(self, message: dict) -> None:
        """Process the event with the callback function.

        Parameters
        ----------
        message : `dict`
            Message.
        """

        # Workaround the mypy check
        assert self._callback_process_event is not None

        try:
            await self._callback_process_event(
                *self._args_callback_process_event, message=message
            )

        except Exception as error:
            self.log.debug(f"Error in processing the event: {message}. {error!r}.")

    def are_clients_connected(self) -> bool:
        """The command and telemetry sockets are connected or not.

        Returns
        -------
        bool
            True if clients are connected. Else, False.
        """
        return (
            (self.client_command is not None)
            and (self.client_telemetry is not None)
            and self.client_command.connected
            and self.client_telemetry.connected
        )

    async def close(self) -> None:
        """Cancel the task and close the connection.

        Note: this function is safe to call even though there is no connection.
        """

        self._start_connection = False
        await cancel_task_and_wait(self._task_connection)

        await self._close_clients()

        self.last_command_status = CommandStatus.Unknown

        self.power_system_status = {
            "motor_power_is_on": False,
            "motor_power_state": MTM2.PowerSystemState.Init,
            "communication_power_is_on": False,
            "communication_power_state": MTM2.PowerSystemState.Init,
        }

        self.closed_loop_control_mode = MTM2.ClosedLoopControlMode.Idle
        self.set_ilc_modes_to_unknown()

    async def _close_clients(self) -> None:
        """Close the clients."""

        if self.client_command is not None:
            await self.client_command.close()

        if self.client_telemetry is not None:
            await self.client_telemetry.close()

        self.client_command = None
        self.client_telemetry = None

    async def clear_errors(self, timeout: float = 10.0) -> None:
        """Clear the errors.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        self.error_handler.clear()

        await self.write_command_to_server("clearErrors", timeout=timeout)

    async def write_command_to_server(
        self,
        message_name: str,
        message_details: dict | None = None,
        timeout: float = 10.0,
    ) -> None:
        """Write the command (message_name) to server.

        Parameters
        ----------
        message_name : `str`
            Message name to server.
        message_details : `dict` or None, optional
            Message details. (the default is None)
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)

        Raises
        ------
        `OSError`
            When no TCP/IP connection.
        `RuntimeError`
            When the command failed.
        """

        if not self.are_clients_connected():
            raise OSError("No TCP/IP connection.")

        # Workaround of the mypy checking
        assert self.client_command is not None

        # Send the command
        self.last_command_status = CommandStatus.Unknown
        await self.client_command.write_message(
            MsgType.Command, message_name, msg_details=message_details
        )

        is_successful = await self._check_command_status(message_name, timeout)

        if not is_successful:
            raise RuntimeError(f"{message_name} command failed.")

    async def _check_command_status(self, command_name: str, timeout: float) -> bool:
        """Check the command status from the controller.

        Parameters
        ----------
        command_name : `str`
            Command name.
        timeout : `float`
            Timeout of command acknowledgement in second.

        Returns
        -------
        `bool`
            True if the command succeeds. Else, False.
        """

        # Track the command status
        time_wait_command_status_update = 0.5
        time_start = time.monotonic()
        while (time.monotonic() - time_start) < timeout:
            last_command_status = self.last_command_status

            if last_command_status == CommandStatus.Success:
                return True

            elif last_command_status in (CommandStatus.Fail, CommandStatus.NoAck):
                return False

            await asyncio.sleep(time_wait_command_status_update)

        if last_command_status == CommandStatus.Ack:
            self.log.debug(f"Only get the acknowledgement of {command_name}.")

        return False

    async def power(
        self,
        power_type: MTM2.PowerType,
        status: bool,
        expected_state: MTM2.PowerSystemState | None = None,
        timeout: float = 30.0,
    ) -> None:
        """Power on/off the motor/communication system.

        Parameters
        ----------
        power_type : enum `MTM2.PowerType`
            Power type.
        status : `bool`
            True if turn on the power; False if turn off the power.
        expected_state : enum `MTM2.PowerSystemState` or `None`, optional
            Expected state of the power system. This is used in the unit test
            only. Put None in general. (the default is None)
        timeout : `float`, optional
            Timeout in second. (the default is 30.0)

        Raises
        ------
        `RuntimeError`
            When the power system status is not expected.
        """

        await self.write_command_to_server(
            "power",
            message_details={"powerType": int(power_type), "status": status},
        )

        if expected_state is None:
            expected_state = (
                MTM2.PowerSystemState.PoweredOn
                if status is True
                else MTM2.PowerSystemState.PoweredOff
            )

        is_expected = await self._check_expected_value(
            self._callback_check_power_status,
            power_type,
            status,
            expected_state,
            timeout=timeout,
        )
        if not is_expected:
            state = (
                self.power_system_status["motor_power_state"]
                if power_type == MTM2.PowerType.Motor
                else self.power_system_status["communication_power_state"]
            )
            raise RuntimeError(
                f"{power_type!r} system is in {state!r} instead of {expected_state!r} in timeout."
            )

    async def _check_expected_value(
        self,
        callback_check: typing.Callable,
        *args: typing.Any,
        time_wait_update: float = 0.5,
        timeout: float = 10.0,
        **kwargs: dict[str, typing.Any],
    ) -> bool:
        """Check the expected value.

        Notes
        -----
        This function is a work-around method to continuously checking the
        expected values in a specific frequency before the timeout. In the cell
        controller, it will need some time to finish the action (command) and
        publish the events of internal state. We would like to judge a command
        is successful or not based on these events. And fail a command if we
        could not get the expected events/states before timeout.

        Parameters
        ----------
        callback_check : `func`
            Callback function to check the expected value. It should return
            True if get the expected value. Otherwise, return False.
        *args : `args`
            Arguments needed in "callback_check" function call.
        time_wait_update : `float`, optional
            Time to wait for the udpate. (the default is 0.5)
        timeout : `float`, optional
            Timeout in second. (the default is 10.0)
        **kwargs : `dict`, optional
            Additional keyword arguments needed in "callback_check" function
            call.

        Return
        ------
        `bool`
            True if get the expected value. Otherwise, False.
        """

        time_start = time.monotonic()
        while (time.monotonic() - time_start) < timeout:
            if callback_check(*args, **kwargs):
                return True

            await asyncio.sleep(time_wait_update)

        return callback_check(*args, **kwargs)

    def _callback_check_power_status(
        self,
        power_type: MTM2.PowerType,
        status: bool,
        expected_state: MTM2.PowerSystemState,
    ) -> bool:
        """Callback function to check the power status is expected or not.

        Parameters
        ----------
        power_type : enum `MTM2.PowerType`
            Power type.
        status : `bool`
            True if turn on the power; False if turn off the power.
        expected_state : enum `MTM2.PowerSystemState`
            Expected state of the power system.

        Return
        ------
        `bool`
            True if the power status is expected. Otherwise, False.
        """

        if power_type == MTM2.PowerType.Motor:
            if (self.power_system_status["motor_power_is_on"] == status) and (
                self.power_system_status["motor_power_state"] == expected_state
            ):
                return True

        else:
            # cRIO simulator doesn't put the communication power to be off.
            # Therefore, do not check
            # 'self.power_system_status["communication_power_is_on"]' here.
            if self.power_system_status["communication_power_state"] == expected_state:
                return True

        return False

    async def set_closed_loop_control_mode(
        self, mode: MTM2.ClosedLoopControlMode, timeout: float = 10.0
    ) -> None:
        """Set the closed-loop control mode.

        Parameters
        ----------
        mode : enum `MTM2.ClosedLoopControlMode`
            Closed-loop control mode.
        timeout : `float`, optional
            Timeout in second. (the default is 10.0)

        Raises
        ------
        `RuntimeError`
            When the closed-loop control mode is not expected.
        """

        await self.write_command_to_server(
            "setClosedLoopControlMode",
            message_details={"mode": int(mode)},
        )

        is_expected = await self._check_expected_value(
            self._callback_check_clc_mode, mode, timeout=timeout
        )
        if not is_expected:
            raise RuntimeError(
                f"Closed-loop control mode is {self.closed_loop_control_mode!r} "
                f"instead of {mode!r} in timeout."
            )

    def _callback_check_clc_mode(
        self, expected_mode: MTM2.ClosedLoopControlMode
    ) -> bool:
        """Callback function to check the closed-loop control (CLC) mode is
        expected or not.

        Parameters
        ----------
        expected_mode : enum `MTM2.ClosedLoopControlMode`
            Expected CLC mode.

        Return
        ------
        `bool`
            True if the CLC mode is expected. Otherwise, False.
        """
        return self.closed_loop_control_mode == expected_mode

    async def set_ilc_to_enabled(
        self, retry_times: int = 3, timeout: float = 20.0
    ) -> None:
        """Set the inner-loop control (ILC) mode to Enabled.

        Notes
        -----
        1. This is translated from the SequenceEngine.set_ILC_mode.vi in
        ts_mtm2. The "retry_times" is the work-around method to deal with ILC,
        which is not very reliable to set the new mode.

        2. There are two state machines based on the ILC's type:
        (i) Actuator ILC: Standby --> Disabled --> Enabled
        (ii) Sensor ILC: Standby --> Enabled

        3. The ModBUS ID begins from 1 based on: "CellConfiguration.xlsx" in
        ts_mtm2.

        Parameters
        ----------
        retry_times : `int`, optional
            Retry times. (the default is 3)
        timeout : `float`, optional
            Timeout in second. (the default is 20.0)

        Raises
        ------
        `RuntimeError`
            When no response from ILC.
        `RuntimeError`
            When the ILC has the unknown state.
        `RuntimeError`
            When not all ILCs are Enabled.
        """

        # Set all the ILC modes to NaN first
        self.ilc_modes = np.array([np.nan] * NUM_INNER_LOOP_CONTROLLER)

        # Transition the ILCs to Enabled state
        addresses_nan = list()
        all_modes_are_received = False
        all_modes_are_clear = True
        all_modes_are_enabled = False
        for idx in range(1 + retry_times):
            # Try to get all the ILC modes first. If not, continue to the next
            # try.
            addresses_nan = np.where(np.isnan(self.ilc_modes))[0].tolist()
            if len(addresses_nan) != 0:
                await self.write_command_to_server(
                    "getInnerLoopControlMode",
                    message_details={"addresses": addresses_nan},
                )
                all_modes_are_received = await self._check_expected_value(
                    self._callback_check_ilc_mode_nan, timeout=timeout
                )

                if not all_modes_are_received:
                    continue

            # For the unknown state, try to get the state again
            addresses_unknown = np.where(
                self.ilc_modes == MTM2.InnerLoopControlMode.Unknown
            )[0].tolist()
            if len(addresses_unknown) != 0:
                await self.write_command_to_server(
                    "getInnerLoopControlMode",
                    message_details={"addresses": addresses_unknown},
                )
                all_modes_are_clear = await self._check_expected_value(
                    self._callback_check_ilc_mode_unknown, timeout=timeout
                )

                if not all_modes_are_clear:
                    continue

            # Do the state transition step by step. Note the state transitions
            # between the actuator ILC and sensor ILC are different.

            # Fault state
            mode_expected = MTM2.InnerLoopControlMode.Standby
            mode_are_set_clear_faults = await self._set_ilc_mode(
                MTM2.InnerLoopControlMode.Fault,
                mode_expected,
                MTM2.InnerLoopControlMode.ClearFaults,
                timeout=timeout,
            )
            if not mode_are_set_clear_faults:
                continue

            # Firmware update
            mode_expected = MTM2.InnerLoopControlMode.Standby
            mode_are_set_standby = await self._set_ilc_mode(
                MTM2.InnerLoopControlMode.FirmwareUpdate,
                mode_expected,
                MTM2.InnerLoopControlMode.Standby,
                timeout=timeout,
            )
            if not mode_are_set_standby:
                continue

            # Standby state (actuator ILC)
            mode_expected = MTM2.InnerLoopControlMode.Disabled
            mode_are_set_disabled = await self._set_ilc_mode(
                MTM2.InnerLoopControlMode.Standby,
                mode_expected,
                MTM2.InnerLoopControlMode.Disabled,
                is_actuator_ilc=True,
                timeout=timeout,
            )
            if not mode_are_set_disabled:
                continue

            # Standby state (sensor ILC)
            mode_expected = MTM2.InnerLoopControlMode.Enabled
            mode_are_set_enabled_sensor = await self._set_ilc_mode(
                MTM2.InnerLoopControlMode.Standby,
                mode_expected,
                MTM2.InnerLoopControlMode.Enabled,
                is_sensor_ilc=True,
                timeout=timeout,
            )
            if not mode_are_set_enabled_sensor:
                continue

            # Disabled state (actuator ILC)
            mode_expected = MTM2.InnerLoopControlMode.Enabled
            mode_are_set_enabled_actuator = await self._set_ilc_mode(
                MTM2.InnerLoopControlMode.Disabled,
                mode_expected,
                MTM2.InnerLoopControlMode.Enabled,
                is_actuator_ilc=True,
                timeout=timeout,
            )
            if not mode_are_set_enabled_actuator:
                continue

            # Break the loop if all ILCs are in Enabled state.
            addresses_not_enabled = np.where(
                self.ilc_modes != MTM2.InnerLoopControlMode.Enabled
            )[0].tolist()
            all_modes_are_enabled = len(addresses_not_enabled) == 0
            if all_modes_are_enabled:
                break

        # Raise the error if needed
        if not all_modes_are_received:
            addresses_nan = np.where(np.isnan(self.ilc_modes))[0].tolist()
            raise RuntimeError(
                f"No response for the following ILCs: {addresses_nan} after "
                f"{retry_times} times of retrying."
            )

        if not all_modes_are_clear:
            addresses_unknown = np.where(
                self.ilc_modes == MTM2.InnerLoopControlMode.Unknown
            )[0].tolist()
            modbus_ids_unknown = [address + 1 for address in addresses_unknown]
            raise RuntimeError(
                f"Following ILCs have the unknown states: {addresses_unknown}. "
                f"The ModBUS IDs are: {modbus_ids_unknown}."
            )

        if not all_modes_are_enabled:
            addresses_not_enabled = np.where(
                self.ilc_modes != MTM2.InnerLoopControlMode.Enabled
            )[0].tolist()
            for address in addresses_not_enabled:
                self.log.debug(
                    f"ILC {address} is {MTM2.InnerLoopControlMode(self.ilc_modes[address])!r} "
                    f"with the ModBUS ID: {address+1}."
                )

            raise RuntimeError(
                f"Following ILCs are not Enabled: {addresses_not_enabled}."
            )

    def _callback_check_ilc_mode_nan(self) -> bool:
        """Callback function to check the inner-loop control (ILC) mode is NaN
        or not. The "NaN" value means the controller does not receive the
        related event of ILC mode.

        Return
        ------
        `bool`
            True if all ILC modes are received. Otherwise, False.
        """

        indexes = np.where(np.isnan(self.ilc_modes))[0]
        return len(indexes) == 0

    def _callback_check_ilc_mode_unknown(self) -> bool:
        """Callback function to check the inner-loop control (ILC) mode is
        unknown or not.

        Return
        ------
        `bool`
            True if all ILC modes are known. Otherwise, False.
        """

        indexes = np.where(self.ilc_modes == MTM2.InnerLoopControlMode.Unknown)[0]
        return len(indexes) == 0

    async def _set_ilc_mode(
        self,
        mode_current: MTM2.InnerLoopControlMode,
        mode_expected: MTM2.InnerLoopControlMode,
        mode_command: MTM2.InnerLoopControlMode,
        is_actuator_ilc: bool = False,
        is_sensor_ilc: bool = False,
        timeout: float = 10.0,
    ) -> bool:
        """Set the inner-loop control (ILC) mode.

        Parameters
        ----------
        mode_current : enum `MTM2.InnerLoopControlMode`
            Current mode.
        mode_expected : enum `MTM2.InnerLoopControlMode`
            Expected mode.
        mode_command : enum `MTM2.InnerLoopControlMode`
            Mode command to issue.
        is_actuator_ilc : bool, optional
            Is the actuator ILC or not. (the default is False)
        is_sensor_ilc : bool, optional
            Is the sensor ILC or not. (the default is False)
        timeout : `float`, optional
            Timeout in second. (the default is 10.0)

        Returns
        -------
        `bool`
            True if the ILC mode is set or can not find any ILC has the
            expected current mode. Otherwise, False.
        """

        # Get the addresses to set the mode
        addresses = np.where(self.ilc_modes == mode_current)[0].tolist()

        if is_actuator_ilc:
            addresses = [address for address in addresses if address < NUM_ACTUATOR]
        elif is_sensor_ilc:
            addresses = [address for address in addresses if address >= NUM_ACTUATOR]

        if len(addresses) == 0:
            return True
        else:
            await self.write_command_to_server(
                "setInnerLoopControlMode",
                message_details={
                    "addresses": addresses,
                    "mode": int(mode_command),
                },
            )
            return await self._check_expected_value(
                self._callback_check_ilc_mode,
                mode_expected,
                timeout=timeout,
                is_actuator_ilc=is_actuator_ilc,  # type: ignore[arg-type]
                is_sensor_ilc=is_sensor_ilc,  # type: ignore[arg-type]
            )

    def _callback_check_ilc_mode(
        self,
        expected_mode: MTM2.InnerLoopControlMode,
        is_actuator_ilc: bool = False,
        is_sensor_ilc: bool = False,
    ) -> bool:
        """Callback function to check the inner-loop control (ILC) mode is
        expected or not.

        Parameters
        ----------
        expected_mode : enum `MTM2.InnerLoopControlMode`
            Expected ILC mode.
        is_actuator_ilc : `bool`, optional
            Is actuator ILC or not. Put True only when you want to check the
            specific set of ILCs. (the default is False).
        is_sensor_ilc : `bool`, optional
            Is sensor ILC or not. Put True only when you want to check the
            specific set of ILCs. (the default is False).

        Return
        ------
        `bool`
            True if the ILC mode is expected. Otherwise, False.
        """

        if is_actuator_ilc:
            indexes = np.where(self.ilc_modes[:NUM_ACTUATOR] != expected_mode)[0]

        elif is_sensor_ilc:
            indexes = np.where(self.ilc_modes[NUM_ACTUATOR:] != expected_mode)[0]

        else:
            indexes = np.where(self.ilc_modes != expected_mode)[0]

        return len(indexes) == 0

    async def reset_force_offsets(self, timeout: float = 10.0) -> None:
        """Reset the user defined forces to zero.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """
        await self.write_command_to_server("resetForceOffsets", timeout=timeout)

    async def reset_actuator_steps(self, timeout: float = 10.0) -> None:
        """Resets the user defined actuator steps to zero.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """
        await self.write_command_to_server("resetActuatorSteps", timeout=timeout)

    async def load_configuration(self, timeout: float = 10.0) -> None:
        """Load the configuration.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """
        await self.write_command_to_server("loadConfiguration", timeout=timeout)

    async def set_control_parameters(self, timeout: float = 10.0) -> None:
        """Set the control parameters of closed-loop controller (CLC).

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        message_details = dict()
        for key, value in self.control_parameters.items():
            message_details[camel_case(key)] = value

        await self.write_command_to_server(
            "setControlParameters",
            message_details=message_details,
            timeout=timeout,
        )

    async def set_enabled_faults_mask(self, mask: int, timeout: float = 10.0) -> None:
        """Set the enabled faults mask.

        Parameters
        ----------
        mask : `int`
            Enabled faults mask.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "setEnabledFaultsMask",
            message_details={"mask": int(mask)},
            timeout=timeout,
        )

    async def reset_enabled_faults_mask(self, timeout: float = 10.0) -> None:
        """Reset the enabled faults mask to default.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """
        await self.set_enabled_faults_mask(DEFAULT_ENABLED_FAULTS_MASK, timeout=timeout)

    async def set_configuration_file(self, file: str, timeout: float = 10.0) -> None:
        """Set the configuration file.

        Parameters
        ----------
        file : `str`
            Configuration file.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "setConfigurationFile",
            message_details={"file": file},
            timeout=timeout,
        )

    async def apply_forces(
        self,
        force_axial: list[float],
        force_tangent: list[float],
        timeout: float = 10.0,
    ) -> None:
        """Apply the actuator forces.

        Parameters
        ----------
        force_axial : `list`
            72 axial actuator forces in Newton.
        force_tangent : `list`
            6 tangent actuator forces in Newton.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "applyForces",
            message_details={"axial": force_axial, "tangent": force_tangent},
            timeout=timeout,
        )

    async def position_mirror(
        self,
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
        timeout: float = 10.0,
    ) -> None:
        """Position the mirror by the rigid body movement.

        Parameters
        ----------
        x : `float`
            Position x in um.
        y : `float`
            Position y in um.
        z : `float`
            Position z in um.
        rx : `float`
            Rotation x in arcsec.
        ry : `float`
            Rotation y in arcsec.
        rz : `float`
            Rotation z in arcsec.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "positionMirror",
            message_details={
                "x": x,
                "y": y,
                "z": z,
                "xRot": rx,
                "yRot": ry,
                "zRot": rz,
            },
            timeout=timeout,
        )

    def select_inclination_source(
        self,
        use_external_elevation_angle: bool,
        max_angle_difference: float | None = None,
        enable_angle_comparison: bool | None = None,
    ) -> None:
        """Select the inclination source. This will affect the angle used to
        do the look-up table (LUT) calculation.

        Parameters
        ----------
        use_external_elevation_angle : `bool`
            Use the external elevation angle (e.g. telescope mount assembly,
            TMA) or not.
        max_angle_difference : `float` or None, optional
            Maximum angle difference between the internal and external values
            in degree. If None, the default value is applied. (the default is
            None)
        enable_angle_comparison : `bool` or None, optional
            Enable the comparison of angles or not. If the external angle is
            used, the value should be True. (the default is None)
        """

        # Update the control parameters
        self.control_parameters[
            "use_external_elevation_angle"
        ] = use_external_elevation_angle

        if max_angle_difference is not None:
            self.control_parameters["max_angle_difference"] = max_angle_difference

        # When using the external elevation angle, need to make sure the
        # comparison with the internal inclinometer to avoid the breaking of
        # system.
        if use_external_elevation_angle:
            enable_angle_comparison = True

        if enable_angle_comparison is not None:
            self.control_parameters["enable_angle_comparison"] = enable_angle_comparison

    async def set_temperature_offset(
        self,
        ring: list[float],
        intake: list[float],
        exhaust: list[float],
        timeout: float = 10.0,
    ) -> None:
        """Set the temperature offset used in the look-up table (LUT)
        calculation.

        Parameters
        ----------
        ring : `list`
            Ring temperature offset in degree C.
        intake : `list`
            Intake temperature offset in degree C.
        exhaust : `list`
            Exhaust temperature offset in degree C.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "setTemperatureOffset",
            message_details={
                "ring": ring,
                "intake": intake,
                "exhaust": exhaust,
            },
            timeout=timeout,
        )

    async def switch_force_balance_system(
        self, status: bool, timeout: float = 10.0
    ) -> None:
        """Switch the force balance system.

        Parameters
        ----------
        status : `bool`
            True if turn on the force balance system. Otherwise, False.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "switchForceBalanceSystem",
            message_details={"status": status},
            timeout=timeout,
        )

    async def set_external_elevation_angle(self, angle: float) -> None:
        """ ""Set the external elevation angle in degree.

        Parameters
        ----------
        angle : `float`
            Angle in degree.
        """

        if self.client_telemetry is not None:
            await self.client_telemetry.write_message(
                MsgType.Telemetry,
                "elevation",
                msg_details=dict(actualPosition=angle),
                comp_name="MTMount",
            )

    async def enable_open_loop_max_limit(
        self, status: bool, timeout: float = 10.0
    ) -> None:
        """Enable the maximum limit in open-loop control.

        Parameters
        ----------
        status : `bool`
            Enable the maximum limit or not.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)

        Raises
        ------
        `RuntimeError`
            If in the closed-loop control.
        """

        closed_loop = MTM2.ClosedLoopControlMode.ClosedLoop
        if self.closed_loop_control_mode != closed_loop:
            await self.write_command_to_server(
                "enableOpenLoopMaxLimit",
                message_details={"status": status},
                timeout=timeout,
            )
        else:
            raise RuntimeError(
                f"Failed to enable the maximum limit. Forbidden in {closed_loop!r}."
            )

    async def save_position(self, timeout: float = 10.0) -> None:
        """Save the rigid body position.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server("saveMirrorPosition", timeout=timeout)

    async def set_home(self, timeout: float = 10.0) -> None:
        """Set the home position.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server("setMirrorHome", timeout=timeout)

    async def reboot_controller(self, timeout: float = 10.0) -> None:
        """Reboot the cell controller.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        try:
            await self.write_command_to_server("rebootController", timeout=timeout)
        # If the controller reboots, it can not reply the command is executed
        # successfully. Therefore, bypass the RuntimeError here.
        except RuntimeError:
            self.log.exception("Bypassing RuntimeError after rebooting the controller.")

    async def switch_command_source(
        self, is_remote: bool, timeout: float = 10.0
    ) -> None:
        """Switch the command source.

        Parameters
        ----------
        is_remote : `bool`
            Remote commandable SAL component (CSC) is the commander or not.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "switchCommandSource",
            message_details={"isRemote": is_remote},
            timeout=timeout,
        )

    async def set_bit_digital_status(
        self,
        idx: int,
        status: DigitalOutputStatus,
        timeout: float = 10.0,
    ) -> None:
        """Set the bit value of digital status.

        Parameters
        ----------
        idx : `int`
            Bit index that begins from 0, which should be >= 0.
        status : enum `DigitalOutputStatus`
            Digital output status.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "switchDigitalOutput",
            message_details={"bit": 2**idx, "status": int(status)},
            timeout=timeout,
        )

    async def reset_breakers(
        self, power_type: MTM2.PowerType, timeout: float = 10.0
    ) -> None:
        """Reset the breakers.

        Parameters
        ----------
        power_type : enum `MTM2.PowerType`
            Power type.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "resetBreakers", message_details={"powerType": power_type}, timeout=timeout
        )

    async def command_script(
        self,
        command: CommandScript,
        script_name: str | None = None,
        timeout: float = 10.0,
    ) -> None:
        """Run the script command.

        Parameters
        ----------
        command : enum `CommandScript`
            Script command.
        script_name : `str` or None, optional
            Name of the script. (the default is None)
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        message_details = {"scriptCommand": command}
        if script_name is not None:
            message_details["scriptName"] = script_name  # type: ignore[assignment]

        await self.write_command_to_server(
            "runScript",
            message_details=message_details,
            timeout=timeout,
        )

    async def command_actuator(
        self,
        command: CommandActuator,
        actuators: list[int] | None = None,
        target_displacement: float | int = 0,
        unit: ActuatorDisplacementUnit = ActuatorDisplacementUnit.Millimeter,
        timeout: float = 10.0,
    ) -> None:
        """Run the actuator command.

        Parameters
        ----------
        command : enum `CommandActuator`
            Actuator command.
        actuators : `list [int]` or None, optional
            Selected actuators to do the movement. If the empty list [] is
            passed, the function will raise the RuntimeError. (the default is
            None)
        target_displacement : `float` or `int`, optional
            Target displacement of the actuators. (the default is 0)
        unit : enum `ActuatorDisplacementUnit`, optional
            Displacement unit. (the default is
            ActuatorDisplacementUnit.Millimeter)
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)

        Raises
        ------
        `RuntimeError`
            No actuator is selected.
        `RuntimeError`
            Not in the open-loop control.
        """

        if (actuators is not None) and (len(actuators) == 0):
            raise RuntimeError("No actuator is selected.")

        if unit == ActuatorDisplacementUnit.Step:
            target_displacement = int(target_displacement)

        open_loop = MTM2.ClosedLoopControlMode.OpenLoop
        if self.closed_loop_control_mode == open_loop:
            message_details = {"actuatorCommand": command}
            if actuators is not None:
                message_details["actuators"] = actuators  # type: ignore[assignment]
                message_details["displacement"] = target_displacement  # type: ignore[assignment]
                message_details["unit"] = unit  # type: ignore[assignment]

            await self.write_command_to_server(
                "moveActuators",
                message_details=message_details,
                timeout=timeout,
            )

        else:
            raise RuntimeError(
                f"Failed to command the actuator. Only allow in {open_loop!r}."
            )

    async def set_hardpoint_list(
        self, hardpoints: list[int], timeout: float = 10.0
    ) -> None:
        """Set the hardpoint list.

        Parameters
        ----------
        hardpoints : `list`
            List of the 0-based hardpoints. There are 6 actuators. The first
            three are the axial actuators and the latter three are the tangent
            links.
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        """

        await self.write_command_to_server(
            "setHardpointList",
            message_details={"actuators": hardpoints},
            timeout=timeout,
        )
