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
import contextlib
import logging
import typing
import unittest
from pathlib import Path

from lsst.ts import tcpip
from lsst.ts.m2com import (
    MockServer,
    TcpClient,
    get_config_dir,
    get_queue_message_latest,
)


class TestMockServerEui(unittest.IsolatedAsyncioTestCase):
    """Test the MockServer class for the engineering user interface (EUI).

    This test case will be removed in the future after unifying the state
    machines in the cell controller.
    """

    config_dir: Path
    host: str
    log: logging.Logger
    maxsize_queue: int

    @classmethod
    def setUpClass(cls) -> None:
        cls.config_dir = get_config_dir()
        cls.host = tcpip.LOCALHOST_IPV4
        cls.log = logging.getLogger()
        cls.maxsize_queue = 1000

    @contextlib.asynccontextmanager
    async def make_server(self) -> MockServer:
        """Instantiate the mock server of M2 for the test."""

        server = MockServer(
            self.host,
            port_command=0,
            port_telemetry=0,
            log=self.log,
            is_csc=False,
        )
        server.model.configure(self.config_dir, "harrisLUT")
        await server.start()

        try:
            yield server
        finally:
            await server.close()

    @contextlib.asynccontextmanager
    async def make_clients(self, server: MockServer) -> typing.AsyncIterator[TcpClient]:
        """Make two TCP/IP clients that talk to the server and wait for it to
        connect.

        Returns (client_cmd, client_tel).

        Parameters
        ----------
        server : `MockServer`
            Mock server.
        """

        client_cmd = TcpClient(
            server.server_command.host,
            server.server_command.port,
            log=self.log,
            maxsize_queue=self.maxsize_queue,
        )
        client_tel = TcpClient(
            server.server_telemetry.host,
            server.server_telemetry.port,
            log=self.log,
            maxsize_queue=self.maxsize_queue,
        )

        await asyncio.gather(client_cmd.connect(), client_tel.connect())

        try:
            yield (client_cmd, client_tel)
        finally:
            await client_cmd.close()
            await client_tel.close()

    async def test_are_servers_connected(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            # Check the one-time messages
            await asyncio.sleep(1)
            self.assertGreaterEqual(client_cmd.queue.qsize(), 9)

            message_status = get_queue_message_latest(
                client_cmd.queue, "summaryFaultsStatus"
            )
            self.assertEqual(message_status["status"], 2**57)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
