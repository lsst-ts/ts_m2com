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
from pathlib import Path

from lsst.ts.tcpip import LOCALHOST_IPV4
from lsst.ts.utils import index_generator

from .controller import Controller
from .mock import MockServer

__all__ = ["ControllerCell"]


class ControllerCell(Controller):
    """Cell controller class.

    Parameters
    ----------
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    is_csc : `bool`, optional
        Is called by the commandable SAL component (CSC) or not. (the default
        is True)
    host : `str` or None, optional
        Host address. (the default is None)
    port_command : `int` or None, optional
        Command port to connect. (the default is None)
    port_telemetry : `int` or None, optional
        Telemetry port to connect. (the default is None)
    timeout_connection : `float`, optional
        Connection timeout in second. (the default is 10.0)

    Attributes
    ----------
    host : `str` or None
        Host address.
    port_command : `int` or None
        Command port to connect.
    port_telemetry : `int` or None
        Telemetry port to connect.
    timeout_connection : `float`
        Connection timeout in second.
    mock_server : `MockServer` or None
        Mock server to support the simulation.
    """

    # Maximum timeout to wait the telemetry in second
    TELEMETRY_WAIT_TIMEOUT = 900

    SLEEP_TIME_CLOSE_MOCK_SERVER = 10

    def __init__(
        self,
        log: logging.Logger | None = None,
        is_csc: bool = True,
        host: str | None = None,
        port_command: int | None = None,
        port_telemetry: int | None = None,
        timeout_connection: float = 10.0,
    ) -> None:
        super().__init__(
            log=log,
        )

        self._is_csc = is_csc

        self.host = host
        self.port_command = port_command
        self.port_telemetry = port_telemetry

        self.timeout_connection = timeout_connection

        # Mock server that is only needed in the simulation
        self.mock_server: MockServer | None = None

        # Sequence generator
        self._sequence_generator = index_generator()

    async def run_mock_server(self, config_dir: Path, lut_path: str) -> None:
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
            is_csc=self._is_csc,
        )
        self.mock_server.model.configure(config_dir, lut_path)
        await self.mock_server.start()

    async def connect_server(self) -> None:
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

        # Workaround of the mypy checking
        assert host is not None
        assert port_command is not None
        assert port_telemetry is not None

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

    async def close_controller_and_mock_server(self) -> None:
        """Close the controller and mock server."""

        await self.close()

        if self.mock_server is not None:
            # Wait some time to let the mock server notices the controller has
            # closed the connection.
            await asyncio.sleep(self.SLEEP_TIME_CLOSE_MOCK_SERVER)

            await self.mock_server.close()
            self.mock_server = None
