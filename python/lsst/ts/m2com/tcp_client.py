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
import copy
import json
import logging
import time
import typing

from lsst.ts.tcpip import Client
from lsst.ts.utils import index_generator, make_done_future

from .enum import MsgType
from .utility import cancel_task_and_wait

__all__ = ["TcpClient"]


class TcpClient(Client):
    """TCP/IP client.

    Parameters
    ----------
    host : `str`
        Host address.
    port : `int`
        Port to connect.
    callback_process_message : `coroutine`
        Function to process the received message. It must has a keyward
        argument of "message" to receive the message as a dictionary.
    *args : `args`
        Arguments needed in "callback_process_message" function call.
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    sequence_generator : `generator` or `None`, optional
        Sequence generator. (the default is None)
    name : `str`, optional
        Name of the tcp-client. Used for logging/debugging purposes. The
        default is "tcp-client")
    check_message_rate : `bool`, optional
        Check the message rate or not. If True, the rate of message will be
        calculated and recorded in the debug logging message. (the default is
        False)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    timeout : `float`
        Read timeout in second.
    last_sequence_id : `int`
        Last sequence ID of command.
    """

    def __init__(
        self,
        host: str,
        port: int,
        callback_process_message: typing.Callable[..., typing.Coroutine],
        *args: typing.Any,
        log: logging.Logger | None = None,
        sequence_generator: typing.Generator | None = None,
        name: str = "tcp-client",
        check_message_rate: bool = False,
    ) -> None:
        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        super().__init__(
            host,
            port,
            self.log,
            connect_callback=self._connect_state_changed_callback,
            name=name,
        )

        # Callback function to process the received message
        self._callback_process_message = callback_process_message
        self._args_callback_process_message = args

        # Sequence ID generator
        self._sequence_id_generator = (
            sequence_generator if sequence_generator is not None else index_generator()
        )
        self.last_sequence_id = -1

        # Monitor loop task (asyncio.Future)
        self._monitor_loop_task = make_done_future()

        self._check_message_rate = check_message_rate

    async def connect(self, timeout: float = 10.0) -> None:
        """Connect to the server.

        Parameters
        ----------
        timeout : `float`, optional
            Timeout in second. (default is 10.0)
        """

        self.log.info("Try to open the connection.")

        try:
            await asyncio.wait_for(self.start_task, timeout=timeout)

        except Exception as error:
            self.log.exception("Error happens in the connection request.")
            raise error

        self.log.info("Connection is on.")

    async def _connect_state_changed_callback(self, client: Client) -> None:
        """Called when the client connection state changes.

        Notes
        -----
        The 'client' is a reserved input from upstream.

        Parameters
        ----------
        client : `tcpip.Client`
            Client.
        """
        self._monitor_loop_task.cancel()

        if self.connected:
            self._monitor_loop_task = asyncio.create_task(self._monitor_message())

    async def _monitor_message(self) -> None:
        """Monitor the message."""

        self.log.info("Begin to monitor the incoming message.")

        consumed_messages = 0
        time_start = time.monotonic()

        try:
            while self.connected:
                has_message = await self._get_and_process_message()

                # Evaluate the message rate
                if self._check_message_rate and has_message:
                    consumed_messages, time_start = self._log_message_rate(
                        consumed_messages, time_start
                    )

        except ConnectionError:
            self.log.info("Reader disconnected; closing client")
            await self.close()

        except asyncio.IncompleteReadError:
            self.log.exception("EOF is reached.")

        self.log.info("Stop to monitor the incoming message.")

    async def _get_and_process_message(self) -> bool:
        """Get and process the read message.

        Returns
        -------
        `bool`
            True if there is the new message. Otherwise, False.
        """

        try:
            message = await self.read_json()
            await self._process_message(message)

            return True

        except json.JSONDecodeError:
            self.log.exception("Ignoring the received message.")

        except (asyncio.IncompleteReadError, ConnectionError):
            raise

        return False

    async def _process_message(self, message: dict) -> None:
        """Process the message.

        Parameters
        ----------
        message : `dict`
            Message.
        """

        try:
            await self._callback_process_message(
                *self._args_callback_process_message, message=message
            )

        except Exception as error:
            self.log.debug(f"Error in processing the message: {message}. {error!r}.")

    def _log_message_rate(
        self, consumed_messages: int, time_start: float, period: float = 2.0
    ) -> tuple[int, float]:
        """Log the message rate.

        Parameters
        ----------
        consumed_messages : `int`
            Consumed messages.
        time_start : `float`
            Start time in second.
        period : `float`, optional
            Period to check the message rate in second. (the default is 2.0)

        Returns
        -------
        consumed_messages : `int`
            Updated consumed messages.
        time_start : `float`
            Updated start time in second.
        """

        consumed_messages += 1

        time_now = time.monotonic()
        time_delta = time_now - time_start
        if time_delta >= period:
            # Log the message rate
            self.log.debug(f"Consumed {consumed_messages // time_delta} messages/s.")

            # Reset the values
            consumed_messages = 0
            time_start = time_now

        return consumed_messages, time_start

    async def write_message(
        self,
        msg_type: MsgType,
        msg_name: str,
        msg_details: dict | None = None,
        comp_name: str | None = None,
    ) -> None:
        """Writes message to the server.

        Parameters
        ----------
        msg_type : enum `MsgType`
            Message type.
        msg_name : `str`
            Message name.
        msg_details : `dict` or None, optional
            Message details. (the default is None)
        comp_name : `str` or None, optional
            Specific component name used in the event or telemetry. (the
            default is None)

        Raises
        ------
        RuntimeError
            When there is no TCP/IP connection.
        ValueError
            If 'id' is in the message details already.
        ValueError
            When the message type is not supported.
        """

        if not self.connected:
            raise RuntimeError("Client not connected with tcp/ip server.")

        if msg_details is None:
            msg_details_with_header = dict()
        else:
            msg_details_with_header = copy.copy(msg_details)

        if "id" in msg_details_with_header.keys():
            raise ValueError("The 'id' is in the message details already.")

        if msg_type == MsgType.Command:
            msg_details_with_header = self._add_cmd_header(
                msg_name, msg_details_with_header
            )
        elif msg_type == MsgType.Event:
            msg_details_with_header = self._add_evt_header(
                msg_name, msg_details_with_header, comp_name=comp_name
            )
        elif msg_type == MsgType.Telemetry:
            msg_details_with_header = self._add_tel_header(
                msg_name, msg_details_with_header, comp_name=comp_name
            )
        else:
            raise ValueError(f"The message type: {msg_type} is not supported.")

        await self.write_json(msg_details_with_header)

    def _add_cmd_header(self, msg_name: str, msg_details: dict) -> dict:
        """Add the command header.

        Note: This method will modify the input: msg_details.

        Parameters
        ----------
        msg_name : `str`
            Message name.
        msg_details : `dict`
            Message details.

        Returns
        -------
        msg_details : dict
            Message details with the header.
        """

        msg_details["id"] = "cmd_" + msg_name

        self.last_sequence_id = next(self._sequence_id_generator)
        msg_details["sequence_id"] = self.last_sequence_id

        return msg_details

    def _add_evt_header(
        self, msg_name: str, msg_details: dict, comp_name: str | None = None
    ) -> dict:
        """Add the event header.

        Note: This method will modify the input: msg_details.

        Parameters
        ----------
        msg_name : `str`
            Message name.
        msg_details : `dict`
            Message details.
        comp_name : `str` or None, optional
            Specific component name. (the default is None)

        Returns
        -------
        msg_details : dict
            Message details with the header.
        """

        msg_details["id"] = "evt_" + msg_name

        if comp_name is not None:
            msg_details["compName"] = comp_name

        return msg_details

    def _add_tel_header(
        self, msg_name: str, msg_details: dict, comp_name: str | None = None
    ) -> dict:
        """Add the telemetry header.

        Note: This method will modify the input: msg_details.

        Parameters
        ----------
        msg_name : `str`
            Message name.
        msg_details : `dict`
            Message details.
        comp_name : `str` or None, optional
            Specific component name. (the default is None)

        Returns
        -------
        msg_details : dict
            Message details with the header.
        """

        msg_details["id"] = "tel_" + msg_name

        if comp_name is not None:
            msg_details["compName"] = comp_name

        return msg_details

    async def close(self) -> None:
        """Cancel the task and close the connection.

        Note: this function is safe to call even though there is no connection.
        """

        # Close the connection
        await super().close()

        # Cancel the task
        await cancel_task_and_wait(self._monitor_loop_task)
