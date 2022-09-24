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
from lsst.ts.m2com import ControllerCell, get_config_dir


class TestControllerCell(unittest.IsolatedAsyncioTestCase):
    """Test the ControllerCell class."""

    @classmethod
    def setUpClass(cls):

        cls.log = logging.getLogger()
        cls.host = tcpip.LOCAL_HOST

        cls.config_dir = get_config_dir()

    @contextlib.asynccontextmanager
    async def make_controller(self):
        """Make the controller (or TCP/IP client) that talks to the server and
        wait for it to connect.

        Returns Controller.
        """

        controller = ControllerCell(log=self.log, host=self.host)

        await controller.run_mock_server(self.config_dir, "harrisLUT")
        await controller.connect_server()

        # Wait a little time to construct the connection
        await asyncio.sleep(2)

        try:
            yield controller
        finally:
            await controller.close_tasks()

        self.assertFalse(controller.are_clients_connected())

    async def test_stop_loops(self):
        async with self.make_controller() as controller:

            self.assertTrue(controller.are_clients_connected())

            controller.run_loops = True
            controller.run_task_event_loop(self._process_event)
            controller.run_task_telemetry_loop(self._process_telemetry)
            controller.run_task_connection_monitor_loop(self._process_lost_connection)

            await asyncio.sleep(2)

            await controller.stop_loops()

            self.assertFalse(controller.run_loops)

    def _process_event(self, message=None):
        pass

    def _process_telemetry(self, message=None):
        pass

    def _process_lost_connection(self):
        pass

    async def test_stop_loops_no_connection(self):

        controller = ControllerCell(log=self.log, host=self.host)
        await controller.stop_loops()

        self.assertFalse(controller.run_loops)

    async def test_close_tasks_no_connection(self):

        controller = ControllerCell(log=self.log, host=self.host)
        await controller.close_tasks()

        self.assertFalse(controller.run_loops)
