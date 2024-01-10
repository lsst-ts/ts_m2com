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
from pathlib import Path

from lsst.ts import tcpip
from lsst.ts.m2com import ControllerCell, get_config_dir


class TestControllerCell(unittest.IsolatedAsyncioTestCase):
    """Test the ControllerCell class."""

    log: logging.Logger
    host: str
    config_dir: Path

    @classmethod
    def setUpClass(cls) -> None:
        cls.log = logging.getLogger()
        cls.host = tcpip.LOCALHOST_IPV4

        cls.config_dir = get_config_dir()

    @contextlib.asynccontextmanager
    async def make_controller(self) -> ControllerCell:
        """Make the controller (or TCP/IP client) that talks to the server and
        wait for it to connect.

        Returns Controller.
        """

        controller = ControllerCell(log=self.log, host=self.host)
        controller.set_callback_process_event(self._callback_process_message)
        controller.set_callback_process_telemetry(self._callback_process_message)
        controller.set_callback_process_lost_connection(self._process_lost_connection)

        await controller.run_mock_server(self.config_dir, "harrisLUT")
        await controller.connect_server()

        # Wait a little time to construct the connection
        await asyncio.sleep(2)

        try:
            yield controller
        finally:
            await controller.close_controller_and_mock_server()

        self.assertFalse(controller.are_clients_connected())

    async def _callback_process_message(self, message: dict | None = None) -> None:
        pass

    async def _process_lost_connection(self) -> None:
        pass

    async def test_close_controller_and_mock_server(self) -> None:
        async with self.make_controller() as controller:
            self.assertTrue(controller.are_clients_connected())

            await controller.close_controller_and_mock_server()

            self.assertFalse(controller.are_clients_connected())
            self.assertIsNone(controller.mock_server)

    async def test_close_controller_and_mock_server_no_connection(self) -> None:
        controller = ControllerCell(log=self.log, host=self.host)
        await controller.close_controller_and_mock_server()

        self.assertIsNone(controller.mock_server)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
