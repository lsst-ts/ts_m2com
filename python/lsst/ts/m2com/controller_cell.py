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
import time

from lsst.ts.tcpip import LOCALHOST_IPV4
from lsst.ts.utils import index_generator, make_done_future

from . import Controller, MockServer, is_coroutine

__all__ = ["ControllerCell"]


class ControllerCell(Controller):
    """Cell controller class.

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
    host : `str` or None, optional
        Host address. (the default is None)
    port_command : `int` or None, optional
        Command port to connect. (the default is None)
    port_telemetry : `int` or None, optional
        Telemetry port to connect. (the default is None)
    timeout_connection : `int` or `float`, optional
        Connection timeout in second. (the default is 10)

    Attributes
    ----------
    host : `str` or None
        Host address.
    port_command : `int` or None
        Command port to connect.
    port_telemetry : `int` or None
        Telemetry port to connect.
    timeout_connection : `int` or `float`
        Connection timeout in second.
    mock_server : `MockServer` or None
        Mock server to support the simulation.
    run_loops : `bool`
        The event and telemetry loops are running or not.
    stop_loop_timeout : `float`
        Timeout of stoping loop in second.
    """

    # Maximum timeout to wait the telemetry in second
    TELEMETRY_WAIT_TIMEOUT = 900

    SLEEP_TIME_CLOSE_MOCK_SERVER = 10

    def __init__(
        self,
        log=None,
        timeout_in_second=0.05,
        maxsize_queue=1000,
        is_csc=True,
        host=None,
        port_command=None,
        port_telemetry=None,
        timeout_connection=10,
    ):
        super().__init__(
            log=log,
            timeout_in_second=timeout_in_second,
            maxsize_queue=maxsize_queue,
            is_csc=is_csc,
        )

        self.host = host
        self.port_command = port_command
        self.port_telemetry = port_telemetry

        self.timeout_connection = timeout_connection

        # Mock server that is only needed in the simulation
        self.mock_server = None

        # Sequence generator
        self._sequence_generator = index_generator()

        self.run_loops = False

        self.stop_loop_timeout = 5.0

        # Task of the telemetry loop from component (asyncio.Future)
        self._task_telemetry_loop = make_done_future()

        # Task of the event loop from component (asyncio.Future)
        self._task_event_loop = make_done_future()

        # Task to monitor the connection status actively (asyncio.Future)
        self._task_connection_monitor_loop = make_done_future()

    async def run_mock_server(self, config_dir, lut_path):
        """Run the mock server to support the simulation mode.

        Parameters
        ----------
        config_dir : `pathlib.PosixPath`
            Configuration directory.
        lut_path : `str`
            Look-up table (LUT) path.
        """

        # Close the running mock server if any
        if self.mock_server is not None:
            await self.mock_server.close()
            self.mock_server = None

        # Run a new mock server
        self.mock_server = MockServer(
            LOCALHOST_IPV4,
            port_command=0,
            port_telemetry=0,
            log=self.log,
            is_csc=self.is_csc,
        )
        self.mock_server.model.configure(config_dir, lut_path)
        await self.mock_server.start()

    async def connect_server(self):
        """Connect the TCP/IP server.

        Raises
        ------
        `RuntimeError`
            If timeout in connection.
        """

        host = self.host
        port_command = self.port_command
        port_telemetry = self.port_telemetry

        # Overwrite the host of _summit.yaml file in ts_config_mttcs to run the
        # simulation mode on summit.
        # The ports of mock server are randomly assigned by the operation
        # system.
        if self.mock_server is not None:
            host = LOCALHOST_IPV4
            port_command = self.mock_server.server_command.port
            port_telemetry = self.mock_server.server_telemetry.port

        self.log.debug(
            f"Host in connection request: {host} with command port: "
            f"{port_command} and telemetry port: {port_telemetry}."
        )

        self.start(
            host,
            port_command,
            port_telemetry,
            sequence_generator=self._sequence_generator,
            timeout=self.timeout_connection,
        )

        time_start = time.monotonic()
        connection_pooling_time = 0.1
        while not self.are_clients_connected() and (
            (time.monotonic() - time_start) < self.timeout_connection
        ):
            await asyncio.sleep(connection_pooling_time)

        if not self.are_clients_connected():
            raise RuntimeError(
                "Timeout in connection - Conection timeouted, connection "
                f"failed or cannot connect. Host: {host}, ports: "
                f"{port_command} and {port_telemetry}."
            )

    def start_task_event_loop(self, process_event, *args, **kwargs):
        """Start the task of event loop.

        Parameters
        ----------
        process_event : `func` or `coroutine`
            Function to process the event. It must has a keyward argument of
            "message" to receive the message as a dictionary.
        *args : `args`
            Arguments needed in "process_event" function call.
        **kwargs : `dict`, optional
            Additional keyword arguments in "process_event" function
            call.
        """
        self._task_event_loop = asyncio.create_task(
            self._event_loop(process_event, *args, **kwargs)
        )

    async def _event_loop(self, process_event, *args, **kwargs):
        """Update and output event information from component.

        Parameters
        ----------
        process_event : `func` or `coroutine`
            Function to process the event. It must has a keyward argument of
            "message" to receive the message as a dictionary.
        *args : `args`
            Arguments needed in "process_event" function call.
        **kwargs : `dict`, optional
            Additional keyword arguments in "process_event" function
            call.
        """

        self.log.debug("Begin to run the event loop from component.")

        is_coroutine_function = is_coroutine(process_event)

        while self.run_loops:
            try:
                message = (
                    self.queue_event.get_nowait()
                    if not self.queue_event.empty()
                    else await self.queue_event.get()
                )

            # When there is no instance in the self.queue_event, the get() will
            # be used to wait for the new coming event. It is a blocking call
            # and will get the asyncio.CancelledError if we cancel the
            # self._task_event_loop.
            except asyncio.CancelledError:
                message = ""

            # Process the event
            if is_coroutine_function:
                await process_event(*args, message=message, **kwargs)
            else:
                process_event(*args, message=message, **kwargs)

            await asyncio.sleep(self.timeout)

        self.log.debug("Component's event loop exited.")

    def start_task_telemetry_loop(self, process_telemetry, *args, period=2, **kwargs):
        """Start the task of telemetry loop.

        Parameters
        ----------
        process_telemetry : `func` or `coroutine`
            Function to process the telemetry. It must has a keyward argument
            of "message" to receive the message as a dictionary.
        *args : `args`
            Arguments needed in "process_telemetry" function call.
        period : `float` or `int`, optional
            Period to check the telemetry rate in second. (the default is 2)
        **kwargs : `dict`, optional
            Additional keyword arguments in "process_telemetry" function
            call.
        """
        self._task_telemetry_loop = asyncio.create_task(
            self._telemetry_loop(process_telemetry, *args, period=period, **kwargs)
        )

    async def _telemetry_loop(self, process_telemetry, *args, period=2, **kwargs):
        """Update and output telemetry information from component.

        Parameters
        ----------
        process_telemetry : `func` or `coroutine`
            Function to process the telemetry. It must has a keyward argument
            of "message" to receive the message as a dictionary.
        *args : `args`
            Arguments needed in "process_telemetry" function call.
        period : `float` or `int`, optional
            Period to check the telemetry rate in second. (the default is 2)
        **kwargs : `dict`, optional
            Additional keyword arguments in "process_telemetry" function
            call.
        """

        self.log.debug("Starting telemetry loop from component.")

        is_coroutine_function = is_coroutine(process_telemetry)

        time_wait_telemetry = 0.0
        is_telemetry_timed_out = False

        messages_consumed = 0
        messages_consumed_log_timer = asyncio.create_task(asyncio.sleep(period))
        while self.run_loops:
            if self.are_clients_connected():
                if (
                    time_wait_telemetry >= self.TELEMETRY_WAIT_TIMEOUT
                    and not is_telemetry_timed_out
                ):
                    self.log.warning(
                        (
                            "No telemetry update for more than "
                            f"{self.TELEMETRY_WAIT_TIMEOUT} seconds. Can you "
                            f"ping the controller at {self.host} and initiate "
                            "connection to one of it's ports (at "
                            f"{self.port_command} and {self.port_telemetry})?"
                        )
                    )
                    time_wait_telemetry = 0.0
                    is_telemetry_timed_out = True

                # If there is no telemetry, sleep for some time
                if self.client_telemetry.queue.empty():
                    await asyncio.sleep(self.timeout)
                    time_wait_telemetry += self.timeout
                    continue

                # There is the telemetry to publish
                time_wait_telemetry = 0.0
                if is_telemetry_timed_out:
                    self.log.info("Telemetry is up and running after failure.")

                # Intentional to express this variable will be False if there
                # is the new message.
                is_telemetry_timed_out = False

                message = self.client_telemetry.queue.get_nowait()

                # Process the telemetry
                if is_coroutine_function:
                    await process_telemetry(*args, message=message, **kwargs)
                else:
                    process_telemetry(*args, message=message, **kwargs)

                # Evaluate the telemetry rate
                messages_consumed += 1
                if messages_consumed_log_timer.done():
                    self.log.debug(f"Consumed {messages_consumed/period} messages/s.")
                    messages_consumed = 0
                    messages_consumed_log_timer = asyncio.create_task(
                        asyncio.sleep(period)
                    )

            else:
                self.log.debug(f"Clients not connected. Waiting {self.timeout}s...")
                await asyncio.sleep(self.timeout)

        self.log.debug("The component's telemetry loop exited/ended.")

    def start_task_connection_monitor_loop(
        self, process_lost_connection, *args, period=1, **kwargs
    ):
        """Start the task of connection monitor loop.

        Parameters
        ----------
        process_lost_connection : `func` or `coroutine`
            Function to process the lost of connection.
        *args : `args`
            Arguments needed in "process_lost_connection" function call.
        period : `float` or `int`, optional
            Period to check the connection status in second. (the default is 1)
        **kwargs : `dict`, optional
            Additional keyword arguments in "process_lost_connection" function
            call.
        """
        self._task_connection_monitor_loop = asyncio.create_task(
            self._connection_monitor_loop(
                process_lost_connection, *args, period=period, **kwargs
            )
        )

    async def _connection_monitor_loop(
        self, process_lost_connection, *args, period=1, **kwargs
    ):
        """Actively monitor the connection status from component. Disconnect
        the system if the connection is lost by itself.

        Parameters
        ----------
        process_lost_connection : `func` or `coroutine`
            Function to process the lost of connection.
        *args : `args`
            Arguments needed in "process_lost_connection" function call.
        period : `float` or `int`, optional
            Period to check the connection status in second. (the default is 1)
        **kwargs : `dict`, optional
            Additional keyword arguments in "process_lost_connection" function
            call.
        """

        self.log.debug("Begin to run the connection monitor loop from component.")

        is_coroutine_function = is_coroutine(process_lost_connection)

        were_clients_connected = False
        while self.run_loops:
            if self.are_clients_connected():
                were_clients_connected = True
            else:
                if were_clients_connected:
                    were_clients_connected = False
                    self.log.info(
                        "Lost the TCP/IP connection in the active monitoring loop."
                    )

                    await self.close()

                    # Process the lost of connection
                    if is_coroutine_function:
                        await process_lost_connection(*args, **kwargs)
                    else:
                        process_lost_connection(*args, **kwargs)

            await asyncio.sleep(period)

        self.log.debug("Connection monitor loop from component closed.")

    async def close_tasks(self):
        """Close the asynchronous tasks.

        Note: this function is safe to call even though there is no connection.
        """

        await self._close_controller_and_mock_server()

        try:
            await self.stop_loops()
        except Exception:
            self.log.exception("Exception while stopping the loops. Ignoring...")

    async def _close_controller_and_mock_server(self):
        """Close the controller and mock server."""

        await self.close()

        if self.mock_server is not None:
            # Wait some time to let the mock server notices the controller has
            # closed the connection.
            await asyncio.sleep(self.SLEEP_TIME_CLOSE_MOCK_SERVER)

            await self.mock_server.close()
            self.mock_server = None

    async def stop_loops(self):
        """Stop the loops.

        Note: this function is safe to call even though there is no connection.
        """

        self.run_loops = False

        for task in (
            self._task_telemetry_loop,
            self._task_event_loop,
            self._task_connection_monitor_loop,
        ):
            try:
                await asyncio.wait_for(task, timeout=self.stop_loop_timeout)
            except asyncio.TimeoutError:
                self.log.debug("Timed out waiting for the loop to finish. Canceling.")

                task.cancel()
