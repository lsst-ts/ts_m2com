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

import numpy as np
from lsst.ts import tcpip
from lsst.ts.m2com import (
    DEFAULT_ENABLED_FAULTS_MASK,
    NUM_ACTUATOR,
    NUM_INNER_LOOP_CONTROLLER,
    CommandActuator,
    CommandStatus,
    Controller,
    ErrorCodeWarning,
    MockErrorCode,
    MockModel,
    MockServer,
    get_config_dir,
)
from lsst.ts.xml.enums import MTM2

SLEEP_TIME_SHORT = 2


class TestController(unittest.IsolatedAsyncioTestCase):
    """Test the Controller class."""

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
        await asyncio.sleep(SLEEP_TIME_SHORT)

        # Clear the disconnection error
        await controller.clear_errors()

        try:
            yield controller
        finally:
            await controller.close()

        self.assertFalse(controller._start_connection)
        self.assertIsNone(controller.client_command)
        self.assertIsNone(controller.client_telemetry)
        self.assertEqual(controller.ilc_bypassed, list())

    async def _callback_process_message(
        self, queue: asyncio.Queue, message: dict | None = None
    ) -> None:
        if message is not None:
            queue.put_nowait(message)

    async def _callback_process_lost_connection(self) -> None:
        self.lost_connection = True

    def test_set_ilc_modes_to_unknown(self) -> None:
        controller = Controller()
        controller.ilc_modes[0] = MTM2.InnerLoopControlMode.Enabled

        controller.set_ilc_modes_to_unknown()

        self.assertEqual(controller.ilc_modes[0], MTM2.InnerLoopControlMode.Unknown)

    def test_are_ilc_modes_enabled(self) -> None:
        controller = Controller()
        self.assertFalse(controller.are_ilc_modes_enabled())

        controller.ilc_modes = np.array(
            [MTM2.InnerLoopControlMode.Enabled] * NUM_INNER_LOOP_CONTROLLER
        )
        self.assertTrue(controller.are_ilc_modes_enabled())

    def test_are_ilc_modes_enabled_bypass_ilcs(self) -> None:
        controller = Controller()
        self.assertFalse(controller.are_ilc_modes_enabled())

        controller.ilc_bypassed = list(range(NUM_INNER_LOOP_CONTROLLER))
        self.assertTrue(controller.are_ilc_modes_enabled())

    async def test_close(self) -> None:
        controller = Controller()
        await controller.close()

        controller.start(tcpip.LOCALHOST_IPV4, 0, 0)
        await controller.close()

    async def test_are_clients_connected(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            self.assertTrue(controller.are_clients_connected())

    def test_are_clients_connected_no_connection(self) -> None:
        controller = Controller()

        self.assertFalse(controller.are_clients_connected())

    async def test_task_connection(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Connection is on in the initial beginning
            self.assertTrue(controller.are_clients_connected())
            self.assertTrue(controller._start_connection)

            # Close the connection between the client and server
            await asyncio.gather(
                server.server_command.close_client(),
                server.server_telemetry.close_client(),
            )
            self.assertFalse(controller.are_clients_connected())

            # Wait a little time to reconstruct the connection
            max_tries = 10
            for _ in range(max_tries):
                if controller.are_clients_connected():
                    break
                await asyncio.sleep(SLEEP_TIME_SHORT)

            self.assertTrue(controller.are_clients_connected())

            # Capture the previous connection lost
            self.assertTrue(self.lost_connection)

    async def test_task_connection_timeout(self) -> None:
        # Let the controller connects to the wrong host position
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
            "127.0.0.2",
            1,
            2,
            timeout=3.0,
        )

        # Sleep some time for the connection task to be done
        await asyncio.sleep(6)

        self.assertFalse(controller.are_clients_connected())

        # Because the connection timeout, the Controller.close() will be
        # triggered.
        self.assertFalse(controller._start_connection)
        self.assertIsNone(controller.client_command)
        self.assertIsNone(controller.client_telemetry)

    async def test_task_analyze_message(self) -> None:
        async with self.make_server() as server, self.make_controller(server) as _:
            # Wait a little time to collect the event messages
            await asyncio.sleep(SLEEP_TIME_SHORT)
            self.assertGreaterEqual(self.queue_evt.qsize(), 11)

    async def test_write_command_to_server_short_timeout(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            with self.assertRaises(RuntimeError):
                await controller.set_ilc_to_enabled(retry_times=0, timeout=1.0)

    async def test_write_command_to_server_fail(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            with self.assertRaises(RuntimeError):
                await controller.write_command_to_server(
                    "switchForceBalanceSystem", message_details={"status": True}
                )

    async def test_write_command_to_closed_server(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await server.close()

            with self.assertRaises(OSError):
                await controller.write_command_to_server(
                    "switchForceBalanceSystem", message_details={"status": True}
                )

    async def test_write_command_to_server_no_this_command(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            with self.assertRaises(RuntimeError):
                await controller.write_command_to_server("noThisCommand")

    async def test_write_command_to_server_no_connection(self) -> None:
        controller = Controller()
        with self.assertRaises(OSError):
            await controller.write_command_to_server("noConnection")

    async def test_clear_errors(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Fake the error
            server.model.fault(MockErrorCode.LimitSwitchTriggeredClosedloop)
            await asyncio.sleep(SLEEP_TIME_SHORT)

            # Clear the error
            await controller.clear_errors()

            # Error is cleared
            self.assertFalse(server.model.error_handler.exists_error())

    async def test_power_communication(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Power on
            await controller.power(MTM2.PowerType.Communication, True)

            self.assertEqual(
                controller.power_system_status["communication_power_state"],
                MTM2.PowerSystemState.PoweredOn,
            )

            # Power off
            await controller.power(MTM2.PowerType.Communication, False)

            self.assertEqual(
                controller.power_system_status["communication_power_state"],
                MTM2.PowerSystemState.PoweredOff,
            )

    async def test_power_motor(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await controller.power(MTM2.PowerType.Motor, True)

            self.assertEqual(
                controller.power_system_status["motor_power_state"],
                MTM2.PowerSystemState.PoweredOn,
            )

    async def test_power_wrong_expected_state(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            with self.assertRaises(RuntimeError):
                await controller.power(
                    MTM2.PowerType.Motor,
                    True,
                    expected_state=MTM2.PowerSystemState.ResettingBreakers,
                )

    async def test_reset_force_offsets(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await controller.reset_force_offsets()

            self.assertEqual(
                controller._task_check_command_status.result(), CommandStatus.Success
            )

    async def test_reset_actuator_steps(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await controller.reset_actuator_steps()

            self.assertEqual(
                controller._task_check_command_status.result(), CommandStatus.Success
            )

    async def test_set_closed_loop_control_mode(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            await controller.set_closed_loop_control_mode(
                MTM2.ClosedLoopControlMode.TelemetryOnly
            )

            self.assertEqual(
                controller.closed_loop_control_mode,
                MTM2.ClosedLoopControlMode.TelemetryOnly,
            )

    async def test_set_ilc_to_enabled_fail_no_power(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            with self.assertRaises(RuntimeError):
                await controller.set_ilc_to_enabled()

    async def test_set_ilc_to_enabled(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Turn on the communication power
            await server.model.power_communication.power_on()

            # Check the bypassed ILCs
            self.assertEqual(controller.ilc_bypassed, [5])

            # Default is Standby state
            await controller.set_ilc_to_enabled()

            self._assert_ilc_enabled(server.model)

            # This should work directily
            await controller.set_ilc_to_enabled()

            self._assert_ilc_enabled(server.model)

            # Put some ILCs to Disabled state
            for idx in range(3):
                server.model.list_ilc[idx].set_mode(MTM2.InnerLoopControlMode.Disabled)

            await controller.set_ilc_to_enabled()

            self._assert_ilc_enabled(server.model)

    def _assert_ilc_enabled(self, model: MockModel) -> None:
        for ilc in model.list_ilc:
            self.assertEqual(ilc.mode, MTM2.InnerLoopControlMode.Enabled)

    async def test_set_ilc_to_enabled_from_fault(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Turn on the communication power
            await server.model.power_communication.power_on()

            # Put all ILCs to Fault state first
            self._change_ilc_mode(server.model, MTM2.InnerLoopControlMode.Fault)

            await controller.set_ilc_to_enabled()

            self._assert_ilc_enabled(server.model)

    def _change_ilc_mode(
        self, model: MockModel, mode: MTM2.InnerLoopControlMode
    ) -> None:
        for ilc in model.list_ilc:
            ilc.mode = mode

    async def test_set_ilc_to_enabled_from_firmware_update(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Turn on the communication power
            await server.model.power_communication.power_on()

            # Put all ILCs to FirmwareUpdate state first
            self._change_ilc_mode(
                server.model, MTM2.InnerLoopControlMode.FirmwareUpdate
            )

            await controller.set_ilc_to_enabled()

            self._assert_ilc_enabled(server.model)

    async def test_set_ilc_to_enabled_from_unknown(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            # Turn on the communication power
            await server.model.power_communication.power_on()

            # Put all ILCs to Unknown state first
            self._change_ilc_mode(server.model, MTM2.InnerLoopControlMode.Unknown)

            with self.assertRaises(RuntimeError):
                await controller.set_ilc_to_enabled()

    async def test_reset_enabled_faults_mask(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            server.model.error_handler.enabled_faults_mask = 0

            await controller.reset_enabled_faults_mask()
            await asyncio.sleep(SLEEP_TIME_SHORT)

            self.assertEqual(
                server.model.error_handler.enabled_faults_mask,
                DEFAULT_ENABLED_FAULTS_MASK,
            )

    def test_select_inclination_source(self) -> None:
        controller = Controller()

        max_angle_difference = 10

        # Use the external angle
        controller.select_inclination_source(
            use_external_elevation_angle=True, max_angle_difference=max_angle_difference
        )

        # When using the external angle, the angle comparison should be
        # enabled.
        self.assertTrue(controller.control_parameters["use_external_elevation_angle"])
        self.assertTrue(controller.control_parameters["enable_angle_comparison"])
        self.assertEqual(
            controller.control_parameters["max_angle_difference"], max_angle_difference
        )

    async def test_switch_force_balance_system_exception(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:

            controller.control_parameters["enable_lut_inclinometer"] = False
            with self.assertRaises(RuntimeError):
                await controller.switch_force_balance_system(True)

            controller.control_parameters["enable_lut_inclinometer"] = True
            with self.assertRaises(RuntimeError):
                await controller.switch_force_balance_system(True)

            controller.ilc_modes[:NUM_ACTUATOR] = MTM2.InnerLoopControlMode.Enabled
            with self.assertRaises(RuntimeError):
                await controller.switch_force_balance_system(True)

            # Inclinometer ILC (ILC-84)
            controller.ilc_modes[83] = MTM2.InnerLoopControlMode.Enabled
            with self.assertRaises(RuntimeError):
                await controller.switch_force_balance_system(True)

            controller.ilc_modes[:NUM_INNER_LOOP_CONTROLLER] = (
                MTM2.InnerLoopControlMode.Enabled
            )
            controller.error_handler.add_new_warning(
                ErrorCodeWarning.TemperatureSensorOutOfRange
            )
            with self.assertRaises(RuntimeError):
                await controller.switch_force_balance_system(True)

    async def test_enable_open_loop_max_limit_exception(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            controller.closed_loop_control_mode = MTM2.ClosedLoopControlMode.ClosedLoop
            with self.assertRaises(RuntimeError):
                await controller.enable_open_loop_max_limit(True)

    async def test_command_actuator_exception(self) -> None:
        async with self.make_server() as server, self.make_controller(
            server
        ) as controller:
            with self.assertRaises(RuntimeError):
                await controller.command_actuator(CommandActuator.Start, actuators=[])

            with self.assertRaises(RuntimeError):
                await controller.command_actuator(CommandActuator.Stop)

    def test_is_powered_on_communication(self) -> None:

        controller = Controller()
        self.assertFalse(controller.is_powered_on_communication())

        controller.power_system_status["communication_power_is_on"] = True
        controller.power_system_status["communication_power_state"] = (
            MTM2.PowerSystemState.PoweredOn
        )
        self.assertTrue(controller.is_powered_on_communication())

    def test_is_powered_on_motor(self) -> None:

        controller = Controller()
        self.assertFalse(controller.is_powered_on_motor())

        controller.power_system_status["motor_power_is_on"] = True
        controller.power_system_status["motor_power_state"] = (
            MTM2.PowerSystemState.PoweredOn
        )
        self.assertTrue(controller.is_powered_on_motor())


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
