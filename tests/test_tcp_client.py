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
import unittest

from lsst.ts import tcpip
from lsst.ts.m2com import MsgType, TcpClient
from lsst.ts.utils import index_generator

# Read timeout in second
READ_TIMEOUT = 1


class TestTcpClient(unittest.IsolatedAsyncioTestCase):
    """Test the TcpClient class."""

    host: str
    log: logging.Logger
    times_previous_command: int

    @classmethod
    def setUpClass(cls) -> None:
        cls.host = tcpip.LOCALHOST_IPV4
        cls.log = logging.getLogger()
        cls.times_previous_command = 3

    @contextlib.asynccontextmanager
    async def make_server(self) -> tcpip.OneClientServer:
        """Instantiate a TCP/IP server for the test."""

        server = tcpip.OneClientServer(
            host=self.host,
            port=0,
            name="test",
            log=self.log,
            connect_callback=None,
        )
        await server.start_task
        try:
            yield server
        finally:
            await server.close()

    @contextlib.asynccontextmanager
    async def make_client(self, server: tcpip.OneClientServer) -> TcpClient:
        """Make the client and do the connection.

        Parameters
        ----------
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        """

        # Create a sequence generator and run it for a couple times first
        sequence_generator = index_generator()
        for count in range(self.times_previous_command):
            next(sequence_generator)

        client = TcpClient(
            server.host,
            server.port,
            log=self.log,
            sequence_generator=sequence_generator,
            maxsize_queue=8,
        )
        await client.connect()

        try:
            yield client
        finally:
            await client.close()

    async def test_connect_timeout(self) -> None:
        client = TcpClient(self.host, 8888)

        with self.assertRaises(asyncio.TimeoutError):
            await client.connect(timeout=3.0)

    async def test_connect_timeout_wrong_host(self) -> None:
        client = TcpClient("127.0.0.2", 8888)

        with self.assertRaises(asyncio.TimeoutError):
            await client.connect(timeout=3.0)

    async def test_close(self) -> None:
        client = TcpClient(tcpip.LOCALHOST_IPV4, 0)
        await client.close()

    async def test_is_connected(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            self.assertTrue(client.is_connected())

            await server.close_client()

            # Need to add a small time to close the client's connection totally
            await asyncio.sleep(0.1)

            self.assertFalse(client.is_connected())

    async def test_connect_multiple_times(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            self.assertTrue(client.is_connected())
            self.assertTrue(server.connected)

            # Need to add a small time to close the client's connection totally
            await client.close()
            await asyncio.sleep(0.1)

            self.assertFalse(client.is_connected())
            self.assertFalse(server.connected)

            # Try to re-connect to server
            await client.connect()

            self.assertTrue(client.is_connected())
            self.assertTrue(server.connected)

    async def test_write_no_connection(self) -> None:
        tcp_client = TcpClient(self.host, 0, log=self.log)

        with self.assertRaises(RuntimeError):
            await tcp_client.write(MsgType.Event, "inPosition")

    async def test_write_cmd(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            self.assertEqual(client.last_sequence_id, -1)

            msg_name = "move"
            msg_details = {"x": 1, "y": 2, "z": 3}
            await client.write(MsgType.Command, msg_name, msg_details=msg_details)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "cmd_" + msg_name)
            self.assertEqual(message["sequence_id"], self.times_previous_command + 1)
            self.assertEqual(message["x"], msg_details["x"])
            self.assertEqual(message["y"], msg_details["y"])
            self.assertEqual(message["z"], msg_details["z"])

            self.assertEqual(client.last_sequence_id, self.times_previous_command + 1)

    async def _read_msg_in_server(
        self, server: tcpip.OneClientServer, timeout: float
    ) -> dict:
        """Read the received message in server.

        Parameters
        ----------
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        timeout : `float`
            Timeout to read the message in second.

        Returns
        -------
        `dict`
            Received message.
        """

        data = await asyncio.wait_for(server.read_json(), timeout)
        return data

    async def test_write_cmd_multiple(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            msg_name = "move"
            msg_details = {"x": 1, "y": 2, "z": 3}
            for count in range(3):
                await client.write(MsgType.Command, msg_name, msg_details=msg_details)

                message = await self._read_msg_in_server(server, READ_TIMEOUT)

                sequence_id_expected = self.times_previous_command + count + 1
                self.assertEqual(message["sequence_id"], sequence_id_expected)
                self.assertEqual(client.last_sequence_id, sequence_id_expected)

    async def test_write_cmd_no_details(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            msg_name = "enable"
            await client.write(MsgType.Command, msg_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "cmd_" + msg_name)
            self.assertEqual(message["sequence_id"], self.times_previous_command + 1)

    async def test_write_id_error(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            msg_name = "move"
            msg_details = {"id": "cmd_name"}
            with self.assertRaises(ValueError):
                await client.write(MsgType.Command, msg_name, msg_details=msg_details)

    async def test_write_evt(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            msg_name = "inPosition"
            msg_details = {"status": True}
            comp_name = "MTMount"
            await client.write(
                MsgType.Event, msg_name, msg_details=msg_details, comp_name=comp_name
            )

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "evt_" + msg_name)
            self.assertEqual(message["status"], msg_details["status"])
            self.assertEqual(message["compName"], comp_name)

    async def test_write_evt_no_details(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            msg_name = "inPosition"
            comp_name = "MTMount"
            await client.write(MsgType.Event, msg_name, comp_name=comp_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "evt_" + msg_name)
            self.assertEqual(message["compName"], comp_name)

    async def test_write_tel(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            msg_name = "elevation"
            msg_details = {"measured": 1.1}
            comp_name = "MTMount"
            await client.write(
                MsgType.Telemetry,
                msg_name,
                msg_details=msg_details,
                comp_name=comp_name,
            )

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "tel_" + msg_name)
            self.assertEqual(message["measured"], msg_details["measured"])
            self.assertEqual(message["compName"], comp_name)

    async def test_write_tel_no_details(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            msg_name = "inPosition"
            comp_name = "MTMount"
            await client.write(MsgType.Telemetry, msg_name, comp_name=comp_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "tel_" + msg_name)
            self.assertEqual(message["compName"], comp_name)

    async def test_put_read_msg_to_queue(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            input_msg = {"val": 1}
            await server.write_json(input_msg)

            # Sleep a short time to let the monitor loop have a chance to run
            await asyncio.sleep(0.01)

            self.assertEqual(client.queue.qsize(), 1)

            data = client.queue.get_nowait()
            self.assertEqual(data["val"], input_msg["val"])

    async def test_run_monitor_loop(self) -> None:
        async with self.make_server() as server, self.make_client(server) as client:
            input_msg = {"val": 1}
            await self._write_msg_continuously_at_specific_time(
                2, server, input_msg, 5, 1
            )

            self.assertEqual(client.queue.qsize(), 5)

    async def _write_msg_continuously_at_specific_time(
        self,
        time: float,
        server: tcpip.OneClientServer,
        input_msg: dict,
        duration: float,
        frequency: float,
    ) -> None:
        """Write the message continuously in server at the specific time. This
        is to let the client has the enough time to start running the monitor
        loop.

        Parameters
        ----------
        time : `float`
            Time to wait to write the message in second.
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        input_msg : `dict`
            Input message.
        duration : `float`
            Duration of the writing of message in second.
        frequency : `float`
            Frequency of the writing of message in Hz.
        """

        await asyncio.sleep(time)

        count_total = int(duration * frequency)
        sleep_time = 1 / frequency
        for count in range(count_total):
            await server.write_json(input_msg)
            await asyncio.sleep(sleep_time)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
