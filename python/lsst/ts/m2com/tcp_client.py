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
import typing

from lsst.ts.tcpip import Client
from lsst.ts.utils import index_generator, make_done_future

from .enum import MsgType
from .utility import check_queue_size

__all__ = ["TcpClient"]


class TcpClient(Client):
    """TCP/IP client.

    Parameters
    ----------
    host : `str`
        Host address.
    port : `int`
        Port to connect.
    timeout_in_second : `float`, optional
        Read timeout in second. (the default is 0.05)
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    sequence_generator : `generator` or `None`, optional
        Sequence generator. (the default is None)
    maxsize_queue : `int`, optional
        Maximum size of queue. (the default is 1000)
    name : `str`, optional
        Name of the tcp-client. Used for logging/debugging purposes.

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    timeout : `float`
        Read timeout in second.
    last_sequence_id : `int`
        Last sequence ID of command.
    queue : `asyncio.Queue`
        Queue of the message.
    queue_full_log_interval : `float`
        When queue is full, how long to wait until logging condition again
        (in seconds)?
    queue_full_messages_lost : `int`
        How many messages were lost while queue was full.
    """

    def __init__(
        self,
        host: str,
        port: int,
        timeout_in_second: float = 0.05,
        log: logging.Logger | None = None,
        sequence_generator: typing.Generator | None = None,
        maxsize_queue: int = 1000,
        name: str = "tcp-client",
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

        self.timeout = timeout_in_second

        # Sequence ID generator
        self._sequence_id_generator = (
            sequence_generator if sequence_generator is not None else index_generator()
        )
        self.last_sequence_id = -1

        self.queue: asyncio.Queue = asyncio.Queue(maxsize=int(maxsize_queue))

        self.queue_full_log_interval = 1.0  # seconds
        self.queue_full_messages_lost = 0

        # Monitor loop task (asyncio.Future)
        self._monitor_loop_task = make_done_future()

        # Timer for queue full log message
        self._timer_queue_full_task = make_done_future()
        self._timer_check_queue_size_task = make_done_future()

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
            self._monitor_loop_task = asyncio.create_task(self._monitor_msg())

    async def _monitor_msg(self) -> None:
        """Monitor the message."""

        self.log.info("Begin to monitor the incoming message.")

        try:
            while self.connected:
                await self._put_read_msg_to_queue()

        except ConnectionError:
            self.log.info("Reader disconnected; closing client")
            await self.close()

        except asyncio.IncompleteReadError:
            self.log.exception("EOF is reached.")

        self.log.info("Stop to monitor the incoming message.")

    async def _put_read_msg_to_queue(self) -> None:
        """Put the read message to self.queue."""

        try:
            msg = await asyncio.wait_for(self.read_json(), self.timeout)
            self.queue.put_nowait(msg)

            if self._timer_check_queue_size_task.done():
                if check_queue_size(self.queue, self.log, self.name):
                    self._timer_check_queue_size_task = asyncio.create_task(
                        asyncio.sleep(self.queue_full_log_interval)
                    )

        except asyncio.TimeoutError:
            await asyncio.sleep(self.timeout)

        except json.JSONDecodeError:
            self.log.exception("Ignoring the received message.")

        except asyncio.QueueFull:
            self.queue_full_messages_lost += 1
            if self._timer_queue_full_task.done():
                self.log.exception(
                    f"{self.name}::Internal queue is full. "
                    f"Lost {self.queue_full_messages_lost} messages since last report."
                )
                self.queue_full_messages_lost = 0
                self._timer_queue_full_task = asyncio.create_task(
                    asyncio.sleep(self.queue_full_log_interval)
                )

        except (asyncio.IncompleteReadError, ConnectionError):
            raise

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

        # Cancel the task
        if not self._monitor_loop_task.done():
            self._monitor_loop_task.cancel()

        await super().close()
