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
import sys
import unittest
from pathlib import Path

from lsst.ts import tcpip
from lsst.ts.m2com import (
    TEST_DIGITAL_OUTPUT_POWER_COMM,
    TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    ActuatorDisplacementUnit,
    CommandActuator,
    CommandScript,
    CommandStatus,
    Controller,
    DigitalOutput,
    DigitalOutputStatus,
    MockServer,
    collect_queue_messages,
    get_config_dir,
    get_queue_message_latest,
)
from lsst.ts.xml.enums import MTM2

SLEEP_TIME_SHORT = 1
SLEEP_TIME_LONG = 10


class TestControllerEui(unittest.IsolatedAsyncioTestCase):
    """Test the Controller class for the engineering user interface (EUI).

    This test case will be removed in the future after unifying the state
    machines in the cell controller.
    """

    config_dir: Path
    host: str
    timeout_in_second: float
    log: logging.Logger
    maxsize_queue: int

    queue_evt: asyncio.Queue
    queue_tel: asyncio.Queue

    lost_connection: bool

    @classmethod
    def setUpClass(cls) -> None:
        cls.config_dir = get_config_dir()
        cls.host = tcpip.LOCALHOST_IPV4
        cls.timeout_in_second = 0.05

        logging.basicConfig(
            level=logging.INFO, handlers=[logging.StreamHandler(sys.stdout)]
        )
        cls.log = logging.getLogger()
        cls.maxsize_queue = 1000

        cls.queue_evt = asyncio.Queue()
        cls.queue_tel = asyncio.Queue()

        cls.lost_connection = False

    @contextlib.asynccontextmanager
    async def make_server(self) -> MockServer:
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
            # Let the clients close the connection first
            await asyncio.sleep(3)

            await server.close()

    @contextlib.asynccontextmanager
    async def make_controller(self, server: MockServer) -> Controller:
        """Make the controller (or TCP/IP client) that talks to the server and
        wait for it to connect.

        Returns Controller.

        Parameters
        ----------
        server : `MockServer`
            Mock server.
        """

        # Create new queues
        self.queue_evt = asyncio.Queue(maxsize=self.maxsize_queue)
        self.queue_tel = asyncio.Queue(maxsize=self.maxsize_queue)

        self.lost_connection = False

        controller = Controller(log=self.log)
        controller.set_callback_process_event(
            self._callback_process_message, self.queue_evt
        )
        controller.set_callback_process_telemetry(
            self._callback_process_message, self.queue_tel
        )
        controller.set_callback_process_lost_connection(
            self._callback_process_lost_connection
        )

        controller.start(
            server.server_command.host,
            server.server_command.port,
            server.server_telemetry.port,
        )

        # Wait a little time to construct the connection
        await asyncio.sleep(2)

        # Clear the disconnection error
        await controller.clear_errors()

        try:
            yield controller
        finally:
            await controller.close()

    async def _callback_process_message(
        self, queue: asyncio.Queue, message: dict | None = None
    ) -> None:
        if message is not None:
            queue.put_nowait(message)

    async def _callback_process_lost_connection(self) -> None:
        self.lost_connection = True

    async def test_switch_command_source(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Default condition
            is_commandable_by_dds = await self._get_latest_state_commandable_by_dds(
                self.queue_evt
            )
            self.assertTrue(is_commandable_by_dds)

            # Switch to the local control
            await controller.write_command_to_server(
                "switchCommandSource", message_details={"isRemote": False}
            )

            is_commandable_by_dds = await self._get_latest_state_commandable_by_dds(
                self.queue_evt
            )
            self.assertFalse(is_commandable_by_dds)

            # Switch to the remote control
            await controller.write_command_to_server(
                "switchCommandSource", message_details={"isRemote": True}
            )

            is_commandable_by_dds = await self._get_latest_state_commandable_by_dds(
                self.queue_evt
            )
            self.assertTrue(is_commandable_by_dds)

    async def _get_latest_state_commandable_by_dds(self, queue: asyncio.Queue) -> bool:
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

        await asyncio.sleep(SLEEP_TIME_SHORT)

        msg_latest = get_queue_message_latest(queue, "commandableByDDS")
        return msg_latest["state"]

    async def test_run_script_fail(self) -> None:
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

    async def test_run_script_success_to_end(self) -> None:
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
            await asyncio.sleep(SLEEP_TIME_SHORT)

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
            messages = collect_queue_messages(self.queue_evt, "scriptExecutionStatus")

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
            await asyncio.sleep(SLEEP_TIME_SHORT)
            self.assertEqual(script_engine._name, "")

    async def test_run_script_success_pause(self) -> None:
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
            await asyncio.sleep(SLEEP_TIME_SHORT)
            message_pause = get_queue_message_latest(
                self.queue_evt, "scriptExecutionStatus"
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
                self.queue_evt, "scriptExecutionStatus"
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
            await asyncio.sleep(SLEEP_TIME_SHORT)
            self.assertFalse(script_engine.is_running)

    async def test_move_actuators_fail(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            with self.assertRaises(RuntimeError):
                await controller.write_command_to_server(
                    "moveActuators",
                    message_details={"actuatorCommand": CommandActuator.Start},
                )

    async def test_move_actuators_success_to_end(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await server.model.power_communication.power_on()
            await server.model.power_motor.power_on()

            await controller.write_command_to_server(
                "moveActuators",
                message_details={
                    "actuatorCommand": CommandActuator.Start,
                    "actuators": [2, 3],
                    "displacement": 1000,
                    "unit": ActuatorDisplacementUnit.Step,
                },
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)

            # Check the result
            message_axial_steps = get_queue_message_latest(
                self.queue_tel, "axialActuatorSteps"
            )

            self.assertEqual(message_axial_steps["steps"][2], 1000)
            self.assertEqual(message_axial_steps["steps"][3], 1000)

    async def test_move_actuators_out_limit(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            server.model.control_open_loop.open_loop_max_limit_is_enabled = True

            await server.model.power_communication.power_on()
            await server.model.power_motor.power_on()

            await controller.write_command_to_server(
                "moveActuators",
                message_details={
                    "actuatorCommand": CommandActuator.Start,
                    "actuators": [0],
                    "displacement": -8000,
                    "unit": ActuatorDisplacementUnit.Step,
                },
            )

            await asyncio.sleep(SLEEP_TIME_LONG)

            self.assertTrue(server.model.error_handler.exists_error())
            self.assertFalse(server.model.control_open_loop.is_running)

            message_limit_switch_status = get_queue_message_latest(
                self.queue_evt, "limitSwitchStatus", flush=False
            )
            self.assertEqual(message_limit_switch_status["retract"], [0])
            self.assertEqual(message_limit_switch_status["extend"], [])

    async def test_move_actuators_success_pause(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await server.model.power_communication.power_on()
            await server.model.power_motor.power_on()

            # Start
            await controller.write_command_to_server(
                "moveActuators",
                message_details={
                    "actuatorCommand": CommandActuator.Start,
                    "actuators": [2, 3],
                    "displacement": 1.2,
                    "unit": ActuatorDisplacementUnit.Millimeter,
                },
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)

            # Pause
            await controller.write_command_to_server(
                "moveActuators",
                message_details={"actuatorCommand": CommandActuator.Pause},
            )

            # Check the result
            message_axial_steps_pause = get_queue_message_latest(
                self.queue_tel, "axialActuatorSteps"
            )

            self.assertGreater(message_axial_steps_pause["steps"][2], 0)

            # Resume
            await controller.write_command_to_server(
                "moveActuators",
                message_details={"actuatorCommand": CommandActuator.Resume},
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)

            # Stop
            await controller.write_command_to_server(
                "moveActuators",
                message_details={"actuatorCommand": CommandActuator.Stop},
            )

            # Check the result
            message_axial_steps_stop = get_queue_message_latest(
                self.queue_tel, "axialActuatorSteps"
            )

            self.assertGreater(
                message_axial_steps_stop["steps"][2],
                message_axial_steps_pause["steps"][2],
            )

    async def test_reset_breakers_communication(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await server.model.power_communication.power_on()
            await controller.write_command_to_server(
                "resetBreakers",
                message_details={"powerType": MTM2.PowerType.Communication},
            )

            messages = collect_queue_messages(self.queue_evt, "digitalOutput")

            self.assertEqual(
                messages[-2]["value"],
                TEST_DIGITAL_OUTPUT_POWER_COMM
                - DigitalOutput.ResetCommunicationBreakers.value,
            )
            self.assertEqual(messages[-1]["value"], TEST_DIGITAL_OUTPUT_POWER_COMM)

    async def test_reset_breakers_motor(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await server.model.power_communication.power_on()
            await server.model.power_motor.power_on()
            await controller.write_command_to_server(
                "resetBreakers",
                message_details={"powerType": MTM2.PowerType.Motor},
            )

            messages = collect_queue_messages(self.queue_evt, "digitalOutput")

            self.assertEqual(
                messages[-2]["value"],
                TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR
                - DigitalOutput.ResetMotorBreakers.value,
            )
            self.assertEqual(
                messages[-1]["value"], TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR
            )

    async def test_reboot_controller(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Connection is on in the initial beginning
            self.assertTrue(controller.are_clients_connected())

            await controller.write_command_to_server("rebootController")

            # Wait a little time to collect the messages
            await asyncio.sleep(SLEEP_TIME_LONG)
            self.assertFalse(controller.are_clients_connected())

            self.assertTrue(self.lost_connection)

    async def test_enable_open_loop_max_limit(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await controller.write_command_to_server(
                "enableOpenLoopMaxLimit", message_details={"status": True}
            )

            # Wait a little time to let the internal process to finish
            await asyncio.sleep(SLEEP_TIME_SHORT)
            self.assertTrue(
                server.model.control_open_loop.open_loop_max_limit_is_enabled
            )

            msg_open_loop_max_limit = get_queue_message_latest(
                self.queue_evt, "openLoopMaxLimit"
            )
            self.assertTrue(msg_open_loop_max_limit["status"])

    async def test_save_mirror_position(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await controller.write_command_to_server("saveMirrorPosition")

            # Wait a little time to collect the messages
            await asyncio.sleep(SLEEP_TIME_SHORT)
            self.assertEqual(controller.last_command_status, CommandStatus.Success)

    async def test_set_mirror_home(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            server.model.mirror_position["x"] = 1

            await controller.write_command_to_server("setMirrorHome")

            self.assertEqual(server.model.mirror_position["x"], 0)

    async def test_switch_digital_output_fail(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # No this bit value
            with self.assertRaises(RuntimeError):
                await controller.write_command_to_server(
                    "switchDigitalOutput", message_details={"bit": 0, "status": 3}
                )

    async def test_switch_digital_output_success(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Switch the communication power
            await controller.write_command_to_server(
                "switchDigitalOutput",
                message_details={
                    "bit": DigitalOutput.CommunicationPower.value,
                    "status": DigitalOutputStatus.ToggleBit.value,
                },
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)

            msg_latest = get_queue_message_latest(self.queue_evt, "digitalOutput")
            self.assertTrue(
                msg_latest["value"] & DigitalOutput.CommunicationPower.value
            )
            self.assertTrue(server.model.power_communication.is_power_on())

            # Switch the motor power
            await controller.write_command_to_server(
                "switchDigitalOutput",
                message_details={
                    "bit": DigitalOutput.MotorPower.value,
                    "status": DigitalOutputStatus.ToggleBit.value,
                },
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)

            msg_latest = get_queue_message_latest(self.queue_evt, "digitalOutput")
            self.assertTrue(msg_latest["value"] & DigitalOutput.MotorPower.value)
            self.assertTrue(server.model.power_motor.is_power_on())


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
