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
import sys
import contextlib
import unittest
import pathlib

from lsst.ts import tcpip
from lsst.ts import salobj
from lsst.ts.m2com import (
    MockServer,
    Controller,
    CommandStatus,
    CommandScript,
    PowerType,
    DigitalOutput,
    collect_queue_messages,
    get_queue_message_latest,
    TEST_DIGITAL_OUTPUT_POWER_COMM,
    TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
)


class TestControllerEui(unittest.IsolatedAsyncioTestCase):
    """Test the Controller class for the engineering user interface (EUI).

    This test case will be removed in the future after unifying the state
    machines in the cell controller.
    """

    @classmethod
    def setUpClass(cls):
        cls.config_dir = pathlib.Path(__file__).parents[0]
        cls.host = tcpip.LOCAL_HOST
        cls.timeout_in_second = 0.05

        logging.basicConfig(
            level=logging.INFO, handlers=[logging.StreamHandler(sys.stdout)]
        )
        cls.log = logging.getLogger()

    @contextlib.asynccontextmanager
    async def make_server(self):
        """Instantiate the mock server of M2 for the test."""

        server = MockServer(
            self.host,
            timeout_in_second=self.timeout_in_second,
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
    async def make_controller(self, server):
        """Make the controller (or TCP/IP client) that talks to the server and
        wait for it to connect.

        Returns Controller.
        """

        controller = Controller(log=self.log, is_csc=False)
        controller.start(
            server.server_command.host,
            server.server_command.port,
            server.server_telemetry.port,
        )

        # Wait a little time to construct the connection
        await asyncio.sleep(2)

        try:
            yield controller
        finally:
            await controller.close()

    async def test_init(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            self.assertFalse(controller._is_csc)

            await asyncio.sleep(1)
            self.assertEqual(controller.controller_state, salobj.State.STANDBY)

    async def test_clear_errors(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            # Fake the error
            server.model.fault()
            await asyncio.sleep(1)
            self.assertEqual(controller.controller_state, salobj.State.FAULT)

            # Clear the error
            await controller.clear_errors()

            # Check the controller's state
            self.assertTrue(server.model.error_cleared)
            self.assertEqual(controller.controller_state, salobj.State.STANDBY)

    async def test_enter_control(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            with self.assertRaises(RuntimeError):
                await controller.write_command_to_server("enterControl")

    async def test_exit_control(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            with self.assertRaises(RuntimeError):
                await controller.write_command_to_server("exitControl")

    async def test_switch_command_source(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            # Default condition
            is_commandable_by_dds = await self._get_latest_state_commandable_by_dds(
                controller.queue_event
            )
            self.assertTrue(is_commandable_by_dds)

            # Switch to the local control
            await controller.write_command_to_server(
                "switchCommandSource", message_details={"isRemote": False}
            )

            is_commandable_by_dds = await self._get_latest_state_commandable_by_dds(
                controller.queue_event
            )
            self.assertFalse(is_commandable_by_dds)

            # Switch to the remote control
            await controller.write_command_to_server(
                "switchCommandSource", message_details={"isRemote": True}
            )

            is_commandable_by_dds = await self._get_latest_state_commandable_by_dds(
                controller.queue_event
            )
            self.assertTrue(is_commandable_by_dds)

    async def _get_latest_state_commandable_by_dds(self, queue):
        """Get the latest state of "commandableByDDS" event.

        Parameters
        ----------
        queue : `asyncio.Queue`
            Queue of message.

        Returns
        -------
        `bool`
            Is commandable by the data distribution system (DDS) or not.
        """

        await asyncio.sleep(1)

        msg_latest = get_queue_message_latest(queue, "commandableByDDS")
        return msg_latest["state"]

    async def test_run_script_fail(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            with self.assertRaises(RuntimeError):
                await controller.write_command_to_server(
                    "runScript",
                    message_details={
                        "scriptCommand": CommandScript.LoadScript,
                        "scriptName": "",
                    },
                )

    async def test_run_script_success_to_end(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            # Load the script and check
            script_name = "test"
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.LoadScript,
                    "scriptName": script_name,
                },
            )

            # Wait a little time to collect the messages
            await asyncio.sleep(1)

            script_engine = server.model.script_engine
            self.assertEqual(script_engine._name, script_name)

            # Run the script to the end
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.Run,
                },
            )
            await asyncio.sleep(20)

            # Check the results
            messages = collect_queue_messages(
                controller.queue_event, "scriptExecutionStatus"
            )

            self.assertEqual(len(messages), 100)

            for idx in range(0, 100):
                message = messages[idx]
                self.assertEqual(message["percentage"], idx + 1)

            self.assertFalse(script_engine.is_running)

            # Clear the script
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.Clear,
                },
            )

            # Wait a little time to collect the messages
            await asyncio.sleep(1)
            self.assertEqual(script_engine._name, "")

    async def test_run_script_success_pause(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            # Load the script
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.LoadScript,
                    "scriptName": "test",
                },
            )

            # Run the script
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.Run,
                },
            )
            await asyncio.sleep(3)

            # Pause the script
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.Pause,
                },
            )

            # Check the results
            await asyncio.sleep(1)
            message_pause = get_queue_message_latest(
                controller.queue_event, "scriptExecutionStatus"
            )

            self.assertGreater(message_pause["percentage"], 0)

            script_engine = server.model.script_engine
            self.assertFalse(script_engine.is_running)

            # Resume the script again
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.Resume,
                },
            )
            await asyncio.sleep(2)

            # Check the results
            message_resume = get_queue_message_latest(
                controller.queue_event, "scriptExecutionStatus"
            )

            self.assertGreater(
                message_resume["percentage"], message_pause["percentage"]
            )
            self.assertNotEqual(message_resume["percentage"], 100)
            self.assertTrue(script_engine.is_running)

            # Stop the script
            await controller.write_command_to_server(
                "runScript",
                message_details={
                    "scriptCommand": CommandScript.Stop,
                },
            )

            # Check the results
            await asyncio.sleep(1)
            self.assertFalse(script_engine.is_running)

    @unittest.skip("TODO: Implement in DM-35583.")
    async def test_move_actuators(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            await controller.close()

    async def test_reset_breakers_communication(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            server.model.communication_power_on = True
            await controller.write_command_to_server(
                "resetBreakers",
                message_details={"powerType": PowerType.Communication},
            )

            messages = collect_queue_messages(controller.queue_event, "digitalOutput")

            self.assertEqual(
                messages[-2]["value"],
                TEST_DIGITAL_OUTPUT_POWER_COMM
                - DigitalOutput.ResetCommunicationBreakers.value,
            )
            self.assertEqual(messages[-1]["value"], TEST_DIGITAL_OUTPUT_POWER_COMM)

    async def test_reset_breakers_motor(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            server.model.communication_power_on = True
            server.model.motor_power_on = True
            await controller.write_command_to_server(
                "resetBreakers",
                message_details={"powerType": PowerType.Motor},
            )

            messages = collect_queue_messages(controller.queue_event, "digitalOutput")

            self.assertEqual(
                messages[-2]["value"],
                TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR
                - DigitalOutput.ResetMotorBreakers.value,
            )
            self.assertEqual(
                messages[-1]["value"], TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR
            )

    async def test_reboot_controller(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            # Connection is on in the initial beginning
            self.assertTrue(controller.are_clients_connected())

            await controller.write_command_to_server("rebootController")

            # Wait a little time to collect the messages
            await asyncio.sleep(1)
            self.assertFalse(controller.are_clients_connected())

    async def test_enable_open_loop_max_limits(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            await controller.write_command_to_server("enableOpenLoopMaxLimits")

            # Wait a little time to let the internal process to finish
            await asyncio.sleep(1)
            self.assertTrue(server.model.open_loop_max_limits_is_enabled)

    async def test_save_mirror_position(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            await controller.write_command_to_server("saveMirrorPosition")

            # Wait a little time to collect the messages
            await asyncio.sleep(1)
            self.assertEqual(controller.last_command_status, CommandStatus.Success)

    @unittest.skip("TODO: Implement in DM-35583.")
    async def test_set_mirror_home(self):
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            await controller.close()


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
