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

import numpy as np
from lsst.ts import salobj
from lsst.ts.utils import make_done_future

from . import (
    NUM_ACTUATOR,
    NUM_INNER_LOOP_CONTROLLER,
    ClosedLoopControlMode,
    CommandStatus,
    InnerLoopControlMode,
    MsgType,
    PowerSystemState,
    PowerType,
    TcpClient,
    check_queue_size,
)

__all__ = ["Controller"]


class Controller:
    """Controller class.

    Parameters
    ----------
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    timeout_in_second : `float`, optional
        Time limit for reading data from the TCP/IP interface (sec). (the
        default is 0.05)
    maxsize_queue : `int`, optional
        Maximum size of queue. (the default is 1000)
    is_csc : `bool`, optional
        Is called by the commandable SAL component (CSC) or not. This is a
        temporary option to separate the CSC and engineering user interface
        (EUI). This will be removed in the future after the state machines in
        the cell controller are unified to a single one. (the default is True)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    client_command : `TcpClient` or None
        Command client.
    client_telemetry : `TcpClient` or None
        Telemetry client.
    queue_event : `asyncio.Queue`
        Queue of the event.
    last_command_status : `CommandStatus`
        Last command status.
    timeout : `float`
        Time limit for reading data from the TCP/IP interface (sec).
    is_csc : `bool`
        Is CSC or not. Remove this after the state machines in cell controller
        are unified.
    controller_state : enum `salobj.State`
        Controller's state.
    power_system_status : `dict`
        Power system status in the cell controller.
    closed_loop_control_mode : enum `ClosedLoopControlMode`
        Closed loop control mode in the cell controller.
    ilc_modes : `numpy.ndarray [InnerLoopControlMode]`
        Modes of the inner-loop controller (ILC).
    """

    def __init__(
        self, log=None, timeout_in_second=0.05, maxsize_queue=1000, is_csc=True
    ):
        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.client_command = None
        self.client_telemetry = None

        self.queue_event = asyncio.Queue(maxsize=int(maxsize_queue))

        self.last_command_status = CommandStatus.Unknown

        self.timeout = timeout_in_second

        self.is_csc = is_csc

        # In EUI, there is no OFFLINE state.
        self.controller_state = (
            salobj.State.OFFLINE if self.is_csc else salobj.State.STANDBY
        )

        self.power_system_status = {
            "motor_power_is_on": False,
            "motor_power_state": PowerSystemState.Init,
            "communication_power_is_on": False,
            "communication_power_state": PowerSystemState.Init,
        }

        self.closed_loop_control_mode = ClosedLoopControlMode.Idle
        self.ilc_modes = np.array(
            [InnerLoopControlMode.Unknown] * NUM_INNER_LOOP_CONTROLLER
        )

        # Start the connection task or not
        self._start_connection = False

        # Task to do the connection (asyncio.Future)
        self._task_connection = make_done_future()

        # Task to analyze the message from server (asyncio.Future)
        self._task_analyze_message = make_done_future()

    def start(
        self, host, port_command, port_telemetry, sequence_generator=None, timeout=10.0
    ):
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

        # Instantiate the TCP/IP clients
        maxsize_queue = self.queue_event.maxsize
        self.client_command = TcpClient(
            host,
            port_command,
            timeout_in_second=self.timeout,
            log=self.log,
            sequence_generator=sequence_generator,
            maxsize_queue=maxsize_queue,
            name="command",
        )
        self.client_telemetry = TcpClient(
            host,
            port_telemetry,
            timeout_in_second=self.timeout,
            log=self.log,
            maxsize_queue=maxsize_queue,
            name="telemetry",
        )

        # Create the tasks
        self._start_connection = True

        self._task_connection = asyncio.create_task(self._connect(timeout))
        self._task_analyze_message = asyncio.create_task(self._analyze_message())

    async def _connect(self, timeout):
        """Connect to the servers.

        Parameters
        ----------
        timeout : `float`
            Connection timeout in second.
        """

        self.log.info("Begin to connect the servers.")

        while self._start_connection:
            if self.are_clients_connected():
                await asyncio.sleep(1)
            else:
                try:
                    await asyncio.gather(
                        self.client_command.connect(timeout=timeout),
                        self.client_telemetry.connect(timeout=timeout),
                    )

                except asyncio.TimeoutError as error:
                    self.log.debug(
                        "Timeouted when connecting to servers - "
                        f"{self.client_command.host}:{self.client_command.port} "
                        f"and/or {self.client_telemetry.host}:{self.client_telemetry.port}: {str(error)}"
                    )
                    await self.close()

                self.log.info("Servers are connected.")

        self.log.info("Stop the connection with servers.")

    async def _analyze_message(self):
        """Analyze the message from the command server."""

        self.log.info("Begin to analyze the message from the command server.")

        while self._start_connection:
            try:
                if not self.client_command.queue.empty():
                    message = self.client_command.queue.get_nowait()
                    self.log.debug(f"Receive the event message: {message}")

                    self._analyze_command_status_and_event(message)
                else:
                    await asyncio.sleep(self.timeout)

            except asyncio.QueueFull:
                self.log.exception("Internal queue of event is full.")

        self.log.info("Stop the analysis of message.")

    def _analyze_command_status_and_event(self, message):
        """Analyze the command status and event.

        Parameters
        ----------
        message : `dict`
            Incoming message.
        """

        if self._is_command_status(message):
            self.last_command_status = self._get_command_status(message)
            return

        # Update the controller state
        if self._is_controller_state(message):
            self.controller_state = salobj.State(message["summaryState"])

        # Check and update the power status
        self._update_power_status(message)

        # Check and update the closed-loop control mode
        self._update_closed_loop_control_mode(message)

        # Check and update the inner-loop control mode
        self._update_inner_loop_control_mode(message)

        # Put the event message into the queue for CSC/GUI to publish
        self.queue_event.put_nowait(message)
        check_queue_size(self.queue_event, self.log)

    def _is_command_status(self, message):
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

    def _get_command_status(self, message):
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

        message_name = message["id"]
        sequence_id = message["sequence_id"]
        if sequence_id == self.client_command.last_sequence_id:
            return self._get_command_status_from_message_name(message_name)
        else:
            self.log.info(
                f"Get the command status = {message_name}. The sequence id = {sequence_id}."
            )
            return CommandStatus.Unknown

    def _get_command_status_from_message_name(self, message_name):
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

    def _is_controller_state(self, message):
        """Is the controller's state or not.

        Parameters
        ----------
        message : `dict`
            Incoming message.

        Returns
        -------
        `bool`
            True if the message is the controller's state. Else, False.
        """

        return message["id"] == "summaryState"

    def _update_power_status(self, message):
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

                power_type = PowerType(power_type_value)
                state = PowerSystemState(state_value)
            except ValueError:
                self.log.debug(
                    f"Get the unknown power type ({power_type_value}) or "
                    f"state ({state_value})."
                )

            if (power_type == PowerType.Motor) and (state is not None):
                self.power_system_status["motor_power_is_on"] = message["status"]
                self.power_system_status["motor_power_state"] = state

            elif (power_type == PowerType.Communication) and (state is not None):
                self.power_system_status["communication_power_is_on"] = message[
                    "status"
                ]
                self.power_system_status["communication_power_state"] = state

    def _update_closed_loop_control_mode(self, message):
        """Check and update the closed-loop control mode.

        Parameters
        ----------
        message : `dict`
            Incoming message.
        """

        if message["id"] == "closedLoopControlMode":
            try:
                mode = message["mode"]
                self.closed_loop_control_mode = ClosedLoopControlMode(mode)
            except ValueError:
                self.log.debug(f"Get the unknown closed-loop control mode ({mode}).")

    def _update_inner_loop_control_mode(self, message):
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
                self.ilc_modes[address] = InnerLoopControlMode(mode)
            except (IndexError, ValueError):
                self.log.debug(
                    f"Get the unknown address ({address}) or inner "
                    f"loop control mode ({mode})."
                )

    def are_clients_connected(self):
        """The command and telemetry sockets are connected or not.

        Returns
        -------
        bool
            True if clients are connected. Else, False.
        """
        return (
            (self.client_command is not None)
            and (self.client_telemetry is not None)
            and self.client_command.is_connected()
            and self.client_telemetry.is_connected()
        )

    async def close(self):
        """Cancel the task and close the connection.

        Note: this function is safe to call even though there is no connection.
        """

        self._start_connection = False

        self._task_connection.cancel()
        self._task_analyze_message.cancel()

        if self.client_command is not None:
            await self.client_command.close()

        if self.client_telemetry is not None:
            await self.client_telemetry.close()

        self.client_command = None
        self.client_telemetry = None

        self.last_command_status = CommandStatus.Unknown

        # In EUI, there is no OFFLINE state.
        self.controller_state = (
            salobj.State.OFFLINE if self.is_csc else salobj.State.STANDBY
        )

        self.power_system_status = {
            "motor_power_is_on": False,
            "motor_power_state": PowerSystemState.Init,
            "communication_power_is_on": False,
            "communication_power_state": PowerSystemState.Init,
        }

        self.closed_loop_control_mode = ClosedLoopControlMode.Idle
        self.ilc_modes = np.array(
            [InnerLoopControlMode.Unknown] * NUM_INNER_LOOP_CONTROLLER
        )

    def assert_controller_state(self, command_name, allowed_curr_states):
        """Assert the current controller's state is allowed to do the command
        or not.

        Parameters
        ----------
        command_name : `str`
            Command name.
        allowed_curr_states : `list [salobj.State]`
            Allowed current states.

        Raises
        ------
        ValueError
            When the command is not allowed in current controller's state.
        """

        # Make sure the data type of allowed_curr_states is list
        if not isinstance(allowed_curr_states, list):
            allowed_curr_states = [allowed_curr_states]

        curr_state = self.controller_state
        if curr_state not in allowed_curr_states:
            raise ValueError(
                f"{command_name} command is not allowed in controller's state {curr_state!r}."
            )

    async def clear_errors(self):
        """Clear the errors."""

        # In EUI, there is no OFFLINE state.
        controller_state_expected = (
            salobj.State.OFFLINE if self.is_csc else salobj.State.STANDBY
        )
        await self.write_command_to_server(
            "clearErrors", controller_state_expected=controller_state_expected
        )

    async def write_command_to_server(
        self,
        message_name,
        message_details=None,
        timeout=10.0,
        controller_state_expected=None,
    ):
        """Write the command (message_name) to server.

        Parameters
        ----------
        message_name : `str`
            Message name to server.
        message_details : `dict` or None, optional
            Message details. (the default is None)
        timeout : `float`, optional
            Timeout of command in second. (the default is 10.0)
        controller_state_expected : enum `salobj.State` or None, optional
            Expected controller's state. This is only used for the commands
            related to the state transition. (the default is None)

        Raises
        ------
        `OSError`
            When no TCP/IP connection.
        `RuntimeError`
            When the command failed.
        """

        if not self.are_clients_connected():
            raise OSError("No TCP/IP connection.")

        # Send the command
        self.last_command_status = CommandStatus.Unknown
        await self.client_command.write(
            MsgType.Command, message_name, msg_details=message_details
        )

        is_successful = await self._check_command_status(
            message_name, timeout, controller_state_expected=controller_state_expected
        )

        if not is_successful:
            raise RuntimeError(f"{message_name} command failed.")

    async def _check_command_status(
        self, command_name, timeout, controller_state_expected=None
    ):
        """Check the command status from the controller.

        Parameters
        ----------
        command_name : `str`
            Command name.
        timeout : `float`
            Timeout of command acknowledgement in second.
        controller_state_expected : enum `salobj.State` or None, optional
            Expected controller's state. This is only used for the commands
            related to the state transition. (the default is None)

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
                if controller_state_expected is None:
                    return True
                else:
                    # Wait some time to let the event loop have the time to
                    # analyze the messages
                    await asyncio.sleep(time_wait_command_status_update)

                    # Check the controller's state is expected or not
                    if self.controller_state == controller_state_expected:
                        return True

            # If false, return immediately
            elif last_command_status in (CommandStatus.Fail, CommandStatus.NoAck):
                return False

            await asyncio.sleep(time_wait_command_status_update)

        # Log the condition that the state transition is successful, but no
        # result received
        if (controller_state_expected is not None) and (
            self.controller_state == controller_state_expected
        ):
            self.log.debug(
                f"Controller's state is expected for {command_name}. But no result received."
            )
            return True

        if last_command_status == CommandStatus.Ack:
            self.log.debug(f"Only get the acknowledgement of {command_name}.")

        return False

    async def power(self, power_type, status, expected_state=None, timeout=20.0):
        """Power on/off the motor/communication system.

        Parameters
        ----------
        power_type : enum `PowerType`
            Power type.
        status : `bool`
            True if turn on the power; False if turn off the power.
        expected_state : enum `PowerSystemState` or `None`, optional
            Expected state of the power system. This is used in the unit test
            only. Put None in general. (the default is None)
        timeout : `float`, optional
            Timeout in second. (the default is 20.0)

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
                PowerSystemState.PoweredOn
                if status is True
                else PowerSystemState.PoweredOff
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
                if power_type == PowerType.Motor
                else self.power_system_status["communication_power_state"]
            )
            raise RuntimeError(
                f"{power_type!r} system is in {state!r} instead of {expected_state!r} in timeout."
            )

    async def _check_expected_value(
        self,
        callback_check,
        *args,
        time_wait_update=0.5,
        timeout=10.0,
        **kwargs,
    ):
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

    def _callback_check_power_status(self, power_type, status, expected_state):
        """Callback function to check the power status is expected or not.

        Parameters
        ----------
        power_type : enum `PowerType`
            Power type.
        status : `bool`
            True if turn on the power; False if turn off the power.
        expected_state : enum `PowerSystemState`
            Expected state of the power system.

        Return
        ------
        `bool`
            True if the power status is expected. Otherwise, False.
        """

        if power_type == PowerType.Motor:
            if (self.power_system_status["motor_power_is_on"] == status) and (
                self.power_system_status["motor_power_state"] == expected_state
            ):
                return True

        else:
            if (self.power_system_status["communication_power_is_on"] == status) and (
                self.power_system_status["communication_power_state"] == expected_state
            ):
                return True

        return False

    async def set_closed_loop_control_mode(self, mode, timeout=10.0):
        """Set the closed-loop control mode.

        Parameters
        ----------
        mode : enum `ClosedLoopControlMode`
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

    def _callback_check_clc_mode(self, expected_mode):
        """Callback function to check the closed-loop control (CLC) mode is
        expected or not.

        Parameters
        ----------
        expected_mode : enum `ClosedLoopControlMode`
            Expected CLC mode.

        Return
        ------
        `bool`
            True if the CLC mode is expected. Otherwise, False.
        """
        return self.closed_loop_control_mode == expected_mode

    async def set_ilc_to_enabled(self, retry_times=3, timeout=10.0):
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
            Timeout in second. (the default is 10.0)

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
        all_modes_are_clear = False
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

            # Break the for-loop if any ILC has the unknown state
            addresses_unknown = np.where(
                self.ilc_modes == InnerLoopControlMode.Unknown
            )[0].tolist()

            all_modes_are_clear = len(addresses_unknown) == 0
            if not all_modes_are_clear:
                break

            # Do the state transition step by step. Note the state transitions
            # between the actuator ILC and sensor ILC are different.

            # Fault state
            mode_expected = InnerLoopControlMode.Standby
            mode_are_set_clear_faults = await self._set_ilc_mode(
                InnerLoopControlMode.Fault,
                mode_expected,
                InnerLoopControlMode.ClearFaults,
                timeout=timeout,
            )
            if not mode_are_set_clear_faults:
                continue

            # Firmware update
            mode_expected = InnerLoopControlMode.Standby
            mode_are_set_standby = await self._set_ilc_mode(
                InnerLoopControlMode.FirmwareUpdate,
                mode_expected,
                InnerLoopControlMode.Standby,
                timeout=timeout,
            )
            if not mode_are_set_standby:
                continue

            # Standby state (actuator ILC)
            mode_expected = InnerLoopControlMode.Disabled
            mode_are_set_disabled = await self._set_ilc_mode(
                InnerLoopControlMode.Standby,
                mode_expected,
                InnerLoopControlMode.Disabled,
                is_actuator_ilc=True,
                timeout=timeout,
            )
            if not mode_are_set_disabled:
                continue

            # Standby state (sensor ILC)
            mode_expected = InnerLoopControlMode.Enabled
            mode_are_set_enabled_sensor = await self._set_ilc_mode(
                InnerLoopControlMode.Standby,
                mode_expected,
                InnerLoopControlMode.Enabled,
                is_sensor_ilc=True,
                timeout=timeout,
            )
            if not mode_are_set_enabled_sensor:
                continue

            # Disabled state (actuator ILC)
            mode_expected = InnerLoopControlMode.Enabled
            mode_are_set_enabled_actuator = await self._set_ilc_mode(
                InnerLoopControlMode.Disabled,
                mode_expected,
                InnerLoopControlMode.Enabled,
                is_actuator_ilc=True,
                timeout=timeout,
            )
            if not mode_are_set_enabled_actuator:
                continue

            # Break the loop if all ILCs are in Enabled state.
            addresses_not_enabled = np.where(
                self.ilc_modes != InnerLoopControlMode.Enabled
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
            modbus_ids_unknown = [address + 1 for address in addresses_unknown]
            raise RuntimeError(
                f"Following ILCs have the unknown states: {addresses_unknown}. "
                f"The ModBUS IDs are: {modbus_ids_unknown}."
            )

        if not all_modes_are_enabled:
            for address in addresses_not_enabled:
                self.log.debug(
                    f"ILC {address} is {InnerLoopControlMode(self.ilc_modes[address])!r} "
                    f"with the ModBUS ID: {address+1}."
                )

            raise RuntimeError(
                f"Following ILCs are not Enabled: {addresses_not_enabled}."
            )

    def _callback_check_ilc_mode_nan(self):
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

    async def _set_ilc_mode(
        self,
        mode_current,
        mode_expected,
        mode_command,
        is_actuator_ilc=False,
        is_sensor_ilc=False,
        timeout=10.0,
    ):
        """Set the inner-loop control (ILC) mode.

        Parameters
        ----------
        mode_current : enum `InnerLoopControlMode`
            Current mode.
        mode_expected : enum `InnerLoopControlMode`
            Expected mode.
        mode_command : enum `InnerLoopControlMode`
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
                is_actuator_ilc=is_actuator_ilc,
                is_sensor_ilc=is_sensor_ilc,
            )

    def _callback_check_ilc_mode(
        self, expected_mode, is_actuator_ilc=False, is_sensor_ilc=False
    ):
        """Callback function to check the inner-loop control (ILC) mode is
        expected or not.

        Parameters
        ----------
        expected_mode : enum `InnerLoopControlMode`
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

    async def reset_force_offsets(self):
        """Reset the user defined forces to zero."""
        await self.write_command_to_server("resetForceOffsets")

    async def reset_actuator_steps(self):
        """Resets the user defined actuator steps to zero."""
        await self.write_command_to_server("resetActuatorSteps")

    async def load_configuration(self):
        """Load the configuration."""
        await self.write_command_to_server("loadConfiguration")
