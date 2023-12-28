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

import numpy as np
from lsst.ts import tcpip
from lsst.ts.m2com import (
    DEFAULT_ENABLED_FAULTS_MASK,
    NUM_ACTUATOR,
    NUM_TANGENT_LINK,
    NUM_TEMPERATURE_EXHAUST,
    NUM_TEMPERATURE_INTAKE,
    NUM_TEMPERATURE_RING,
    TEST_DIGITAL_INPUT_NO_POWER,
    TEST_DIGITAL_INPUT_POWER_COMM,
    TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
    TEST_DIGITAL_OUTPUT_NO_POWER,
    TEST_DIGITAL_OUTPUT_POWER_COMM,
    TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    MockErrorCode,
    MockServer,
    MsgType,
    TcpClient,
    collect_queue_messages,
    get_config_dir,
    get_queue_message_latest,
)
from lsst.ts.xml.enums import MTM2


class TestMockServer(unittest.IsolatedAsyncioTestCase):
    """Test the MockServer class."""

    config_dir: Path
    host: str
    log: logging.Logger
    maxsize_queue: int

    queue_cmd: asyncio.Queue
    queue_tel: asyncio.Queue

    @classmethod
    def setUpClass(cls) -> None:
        cls.config_dir = get_config_dir()
        cls.host = tcpip.LOCALHOST_IPV4
        cls.log = logging.getLogger()
        cls.maxsize_queue = 1000

        cls.queue_cmd = asyncio.Queue()
        cls.queue_tel = asyncio.Queue()

    @contextlib.asynccontextmanager
    async def make_server(self) -> MockServer:
        """Instantiate the mock server of M2 for the test."""

        server = MockServer(
            self.host,
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

        self.assertFalse(server._welcome_message_sent)

        self.assertIsNone(server._message_event)
        self.assertIsNone(server._message_telemetry)

        self.assertFalse(server.model.control_closed_loop.is_running)
        self.assertFalse(server.model.power_motor.is_power_on())
        self.assertFalse(server.model.error_handler.exists_error())

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

        # Create new queues
        self.queue_cmd = asyncio.Queue(maxsize=self.maxsize_queue)
        self.queue_tel = asyncio.Queue(maxsize=self.maxsize_queue)

        client_cmd = TcpClient(
            server.server_command.host,
            server.server_command.port,
            self._callback_process_message,
            self.queue_cmd,
            log=self.log,
        )
        client_tel = TcpClient(
            server.server_telemetry.host,
            server.server_telemetry.port,
            self._callback_process_message,
            self.queue_tel,
            log=self.log,
        )

        await asyncio.gather(client_cmd.connect(), client_tel.connect())

        # Wait for some time to clear the disconnection error
        await asyncio.sleep(2)
        server.model.clear_errors()

        try:
            yield (client_cmd, client_tel)
        finally:
            await asyncio.gather(client_cmd.close(), client_tel.close())

    async def _callback_process_message(
        self, queue: asyncio.Queue, message: dict | None = None
    ) -> None:
        if message is not None:
            queue.put_nowait(message)

    async def test_are_servers_connected(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            self.assertTrue(server.are_servers_connected())

            # Check the one-time message (welcome messages)
            self.assertGreaterEqual(self.queue_cmd.qsize(), 12)

            # Check the TCP/IP connection
            msg_tcpip = self.queue_cmd.get_nowait()
            self.assertEqual(msg_tcpip["id"], "tcpIpConnected")
            self.assertTrue(msg_tcpip["isConnected"])

            # Check the commandable by DDS
            msg_commandable_by_dds = self.queue_cmd.get_nowait()
            self.assertEqual(msg_commandable_by_dds["id"], "commandableByDDS")
            self.assertTrue(msg_commandable_by_dds["state"])

            # Check the hardpoint list
            msg_hardpoints = self.queue_cmd.get_nowait()
            self.assertEqual(msg_hardpoints["id"], "hardpointList")
            self.assertEqual(msg_hardpoints["actuators"], [6, 16, 26, 74, 76, 78])

            # Check the interlock
            msg_interlock = self.queue_cmd.get_nowait()
            self.assertEqual(msg_interlock["id"], "interlock")
            self.assertFalse(msg_interlock["state"])

            # Check the inclination telemetry source
            msg_tel_src = self.queue_cmd.get_nowait()
            self.assertEqual(msg_tel_src["id"], "inclinationTelemetrySource")
            self.assertEqual(
                msg_tel_src["source"], int(MTM2.InclinationTelemetrySource.ONBOARD)
            )

            # Check the temperature offset
            msg_temp_offset = self.queue_cmd.get_nowait()
            self.assertEqual(msg_temp_offset["id"], "temperatureOffset")

            self.assertEqual(msg_temp_offset["ring"], [21.0] * NUM_TEMPERATURE_RING)
            self.assertEqual(msg_temp_offset["intake"], [0.0] * NUM_TEMPERATURE_INTAKE)
            self.assertEqual(
                msg_temp_offset["exhaust"], [0.0] * NUM_TEMPERATURE_EXHAUST
            )

            msg_digital_input = self.queue_cmd.get_nowait()
            self.assertEqual(msg_digital_input["id"], "digitalInput")
            self.assertEqual(msg_digital_input["value"], TEST_DIGITAL_INPUT_NO_POWER)

            msg_digital_output = self.queue_cmd.get_nowait()
            self.assertEqual(msg_digital_output["id"], "digitalOutput")
            self.assertEqual(msg_digital_output["value"], TEST_DIGITAL_OUTPUT_NO_POWER)

            msg_config = self.queue_cmd.get_nowait()
            self.assertEqual(len(msg_config), 20)
            self.assertEqual(msg_config["id"], "config")
            self.assertTrue(msg_config["inclinometerDiffEnabled"])

            msg_clc_mode = self.queue_cmd.get_nowait()
            self.assertEqual(msg_clc_mode["id"], "closedLoopControlMode")
            self.assertEqual(msg_clc_mode["mode"], MTM2.ClosedLoopControlMode.Idle)

            msg_mask = self.queue_cmd.get_nowait()
            self.assertEqual(msg_mask["id"], "enabledFaultsMask")
            self.assertEqual(msg_mask["mask"], DEFAULT_ENABLED_FAULTS_MASK)

            msg_config_files = self.queue_cmd.get_nowait()
            self.assertEqual(msg_config_files["id"], "configurationFiles")
            self.assertEqual(len(msg_config_files["files"]), 4)

    async def test_monitor_msg_cmd_ack(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(MsgType.Command, "resetForceOffsets")
            await asyncio.sleep(0.5)

            msg_ack = get_queue_message_latest(self.queue_cmd, "ack")

            self.assertEqual(msg_ack["sequence_id"], 1)

    async def test_monitor_msg_cmd_success(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(MsgType.Command, "resetForceOffsets")
            await asyncio.sleep(0.5)

            msg_success = get_queue_message_latest(self.queue_cmd, "success")

            self.assertEqual(msg_success["sequence_id"], 1)

    async def test_cmd_unknown(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(MsgType.Command, "unknown")
            await asyncio.sleep(0.5)

            msg_noack = get_queue_message_latest(self.queue_cmd, "noack")

            self.assertEqual(msg_noack["id"], "noack")

    async def test_cell_temperature_high_warning(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server.model.control_closed_loop.temperature["exhaust"] = [99.0, 99.0]
            await asyncio.sleep(0.5)

            msg_high_temp = get_queue_message_latest(
                self.queue_cmd, "cellTemperatureHiWarning"
            )

            self.assertTrue(msg_high_temp["hiWarning"])

    async def test_cmd_power_noack_success(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(
                MsgType.Command,
                "power",
                msg_details={"powerType": MTM2.PowerType.Motor, "status": True},
            )
            await asyncio.sleep(0.5)

            # The above short sleep time will not get the acknowledgement of
            # success
            msg_success = collect_queue_messages(self.queue_cmd, "success")

            self.assertEqual(len(msg_success), 0)

    async def test_cmd_reset_breakers_fail(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            # Command should fail because there is no communication power
            await client_cmd.write_message(
                MsgType.Command,
                "resetBreakers",
                msg_details={"powerType": MTM2.PowerType.Communication},
            )

            await asyncio.sleep(0.5)

            msg_fail = get_queue_message_latest(self.queue_cmd, "fail")
            self.assertEqual(msg_fail["id"], "fail")

    async def test_cmd_power_success(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await server.model.power_communication.power_on()

            await client_cmd.write_message(
                MsgType.Command,
                "power",
                msg_details={"powerType": MTM2.PowerType.Motor, "status": True},
            )
            await asyncio.sleep(8)

            # Get the success of command because of sleeping time in power
            # command
            msg_success = get_queue_message_latest(
                self.queue_cmd, "success", flush=False
            )
            self.assertEqual(msg_success["id"], "success")

            self.assertTrue(server.model.power_motor.is_power_on())

            msg_digital_input = get_queue_message_latest(
                self.queue_cmd, "digitalInput", flush=False
            )
            self.assertEqual(
                msg_digital_input["value"], TEST_DIGITAL_INPUT_POWER_COMM_MOTOR
            )

            msg_digital_output = get_queue_message_latest(
                self.queue_cmd, "digitalOutput", flush=False
            )
            self.assertEqual(
                msg_digital_output["value"], TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR
            )

            msg_open_loop_max_limit = get_queue_message_latest(
                self.queue_cmd, "openLoopMaxLimit", flush=False
            )
            self.assertFalse(msg_open_loop_max_limit["status"])

    async def test_cmd_power_off_motor(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await server.model.power_communication.power_on()
            await server.model.power_motor.power_on()
            server.model.control_closed_loop.is_running = True

            await client_cmd.write_message(
                MsgType.Command,
                "power",
                msg_details={"powerType": MTM2.PowerType.Motor, "status": False},
            )
            await asyncio.sleep(8)

            msg_fb = get_queue_message_latest(
                self.queue_cmd, "forceBalanceSystemStatus", flush=False
            )
            self.assertFalse(msg_fb["status"])

            self.assertFalse(server.model.power_motor.is_power_on())
            self.assertFalse(server.model.control_closed_loop.is_running)

            msg_digital_input = get_queue_message_latest(
                self.queue_cmd, "digitalInput", flush=False
            )
            self.assertEqual(msg_digital_input["value"], TEST_DIGITAL_INPUT_POWER_COMM)

            msg_digital_output = get_queue_message_latest(
                self.queue_cmd, "digitalOutput", flush=False
            )
            self.assertEqual(
                msg_digital_output["value"], TEST_DIGITAL_OUTPUT_POWER_COMM
            )

            msg_open_loop_max_limit = get_queue_message_latest(
                self.queue_cmd, "openLoopMaxLimit", flush=False
            )
            self.assertFalse(msg_open_loop_max_limit["status"])

    async def test_cmd_power_off_communication(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await server.model.power_communication.power_on()

            await client_cmd.write_message(
                MsgType.Command,
                "power",
                msg_details={
                    "powerType": MTM2.PowerType.Communication,
                    "status": False,
                },
            )
            await asyncio.sleep(3)

            msg_success = get_queue_message_latest(
                self.queue_cmd, "success", flush=False
            )
            self.assertEqual(msg_success["id"], "success")

            msg_digital_input = get_queue_message_latest(
                self.queue_cmd, "digitalInput", flush=False
            )
            self.assertEqual(msg_digital_input["value"], TEST_DIGITAL_INPUT_NO_POWER)

            msg_digital_output = get_queue_message_latest(
                self.queue_cmd, "digitalOutput", flush=False
            )
            self.assertEqual(msg_digital_output["value"], TEST_DIGITAL_OUTPUT_NO_POWER)

    async def test_cmd_power_on_check_state(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(
                MsgType.Command,
                "power",
                msg_details={"powerType": int(MTM2.PowerType.Motor), "status": True},
            )

            await asyncio.sleep(5)

            msg_power_system_state = collect_queue_messages(
                self.queue_cmd, "powerSystemState", flush=False
            )

            self.assertEqual(len(msg_power_system_state), 2)
            self.assertEqual(
                msg_power_system_state[0]["powerType"], MTM2.PowerType.Motor.value
            )
            self.assertTrue(msg_power_system_state[0]["status"])
            self.assertEqual(
                msg_power_system_state[0]["state"], MTM2.PowerSystemState.PoweringOn
            )
            self.assertEqual(
                msg_power_system_state[1]["state"], MTM2.PowerSystemState.PoweredOn
            )

            msg_summary_faults_status = get_queue_message_latest(
                self.queue_cmd, "summaryFaultsStatus"
            )

            self.assertEqual(msg_summary_faults_status["status"], 2**34)

    async def test_cmd_power_off_check_state(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(
                MsgType.Command,
                "power",
                msg_details={
                    "powerType": int(MTM2.PowerType.Communication),
                    "status": False,
                },
            )

            await asyncio.sleep(5)

            msg_power_system_state = collect_queue_messages(
                self.queue_cmd, "powerSystemState"
            )

            self.assertEqual(len(msg_power_system_state), 2)
            self.assertEqual(
                msg_power_system_state[0]["powerType"],
                MTM2.PowerType.Communication.value,
            )
            self.assertFalse(msg_power_system_state[0]["status"])
            self.assertEqual(
                msg_power_system_state[0]["state"], MTM2.PowerSystemState.PoweringOff
            )
            self.assertEqual(
                msg_power_system_state[1]["state"], MTM2.PowerSystemState.PoweredOff
            )

    async def test_cmd_apply_forces(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            force_axial = [1] * (NUM_ACTUATOR - NUM_TANGENT_LINK)
            force_tangent = [2] * NUM_TANGENT_LINK

            await client_cmd.write_message(
                MsgType.Command,
                "applyForces",
                msg_details={"axial": force_axial, "tangent": force_tangent},
            )

            await asyncio.sleep(0.5)

            np.testing.assert_array_equal(
                server.model.control_closed_loop.axial_forces["applied"], force_axial
            )
            np.testing.assert_array_equal(
                server.model.control_closed_loop.tangent_forces["applied"],
                force_tangent,
            )

    async def test_cmd_position_mirror(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await server.model.power_motor.power_on()
            server.model.switch_force_balance_system(True)
            mirror_position_set_point = dict(
                [(axis, 1.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
            )

            await client_cmd.write_message(
                MsgType.Command,
                "positionMirror",
                msg_details=mirror_position_set_point,
            )

            await asyncio.sleep(3.0)

            msg_in_position = get_queue_message_latest(
                self.queue_cmd, "m2AssemblyInPosition"
            )

            self.assertTrue(msg_in_position["inPosition"])

            self.assertEqual(
                server.model.mirror_position_offset, mirror_position_set_point
            )

    async def test_cmd_reset_force_offsets(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server.model.control_closed_loop.axial_forces["applied"] = np.ones(
                NUM_ACTUATOR - NUM_TANGENT_LINK
            )
            server.model.control_closed_loop.tangent_forces["applied"] = np.ones(
                NUM_TANGENT_LINK
            )

            await client_cmd.write_message(MsgType.Command, "resetForceOffsets")
            await asyncio.sleep(0.5)

            self.assertEqual(
                np.sum(
                    np.abs(server.model.control_closed_loop.axial_forces["applied"])
                ),
                0,
            )
            self.assertEqual(
                np.sum(
                    np.abs(server.model.control_closed_loop.tangent_forces["applied"])
                ),
                0,
            )

    async def test_cmd_clear_errors(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            # Put the system into Fault and wait for some error message
            server.model.fault(MockErrorCode.LimitSwitchTriggeredClosedloop)
            await asyncio.sleep(1)

            # Check the error code
            msg_status = get_queue_message_latest(self.queue_cmd, "summaryFaultsStatus")
            self.assertEqual(
                msg_status["status"],
                2**6,
            )

            # Clear the error
            await client_cmd.write_message(MsgType.Command, "clearErrors")
            await asyncio.sleep(3)

            self.assertFalse(server.model.error_handler.exists_error())

    async def test_cmd_switch_force_balance_system_fail(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(
                MsgType.Command,
                "switchForceBalanceSystem",
                msg_details={"status": True},
            )
            await asyncio.sleep(0.5)

            msg_fb = get_queue_message_latest(
                self.queue_cmd, "forceBalanceSystemStatus", flush=False
            )
            self.assertFalse(msg_fb["status"])

            self.assertFalse(server.model.control_closed_loop.is_running)

            msg_open_loop_max_limit = get_queue_message_latest(
                self.queue_cmd, "openLoopMaxLimit", flush=False
            )
            self.assertFalse(msg_open_loop_max_limit["status"])

    async def test_cmd_switch_force_balance_system_success(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await server.model.power_motor.power_on()
            await client_cmd.write_message(
                MsgType.Command,
                "switchForceBalanceSystem",
                msg_details={"status": True},
            )
            await asyncio.sleep(2)

            msg_fb = get_queue_message_latest(
                self.queue_cmd, "forceBalanceSystemStatus", flush=False
            )
            self.assertTrue(msg_fb["status"])

            self.assertTrue(server.model.control_closed_loop.is_running)

            msg_open_loop_max_limit = get_queue_message_latest(
                self.queue_cmd, "openLoopMaxLimit", flush=False
            )
            self.assertFalse(msg_open_loop_max_limit["status"])

    async def test_cmd_set_temperature_offset(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            ring = [11.0] * NUM_TEMPERATURE_RING
            intake = [11.0] * NUM_TEMPERATURE_INTAKE
            exhaust = [11.0] * NUM_TEMPERATURE_EXHAUST
            await client_cmd.write_message(
                MsgType.Command,
                "setTemperatureOffset",
                msg_details={"ring": ring, "intake": intake, "exhaust": exhaust},
            )
            await asyncio.sleep(0.5)

            msg_temp_offset = get_queue_message_latest(
                self.queue_cmd, "temperatureOffset"
            )

            self.assertEqual(msg_temp_offset["ring"], ring)
            self.assertEqual(msg_temp_offset["intake"], [0.0] * NUM_TEMPERATURE_INTAKE)
            self.assertEqual(
                msg_temp_offset["exhaust"], [0.0] * NUM_TEMPERATURE_EXHAUST
            )

            self.assertEqual(server.model.control_closed_loop.temperature["ref"], ring)

    async def test_telemetry_no_motor_power(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            # Check the telemetry
            await asyncio.sleep(1)
            self.assertGreater(self.queue_tel.qsize(), 10)

    async def test_telemetry_with_motor_power(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await server.model.power_motor.power_on()
            server.model.switch_force_balance_system(True)

            # Check the telemetry
            await asyncio.sleep(8)
            self.assertGreater(self.queue_tel.qsize(), 150)

    async def test_telemetry_get_mtmount_elevation(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            elevation_angle = 30.0
            await client_tel.write_message(
                MsgType.Telemetry,
                "elevation",
                msg_details={"actualPosition": elevation_angle},
                comp_name="MTMount",
            )

            await asyncio.sleep(0.5)

            self.assertEqual(server.model.inclinometer_angle_external, elevation_angle)

    async def test_telemetry_power_status(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            # Check the initial telemetry
            await asyncio.sleep(0.5)

            msg_power_status = get_queue_message_latest(self.queue_tel, "powerStatus")
            self.assertLess(abs(msg_power_status["motorVoltage"]), 1)
            self.assertLess(abs(msg_power_status["motorCurrent"]), 1)
            self.assertLess(abs(msg_power_status["commVoltage"]), 1)
            self.assertLess(abs(msg_power_status["commCurrent"]), 1)

            # Power on the communication
            await server.model.power_communication.power_on()
            await asyncio.sleep(1)

            # Check the communication power is on
            msg_power_status = get_queue_message_latest(self.queue_tel, "powerStatus")

            self.assertLess(abs(msg_power_status["motorVoltage"]), 1)
            self.assertLess(abs(msg_power_status["motorCurrent"]), 1)
            self.assertGreater(msg_power_status["commVoltage"], 20)
            self.assertGreater(msg_power_status["commCurrent"], 5)

            # Power on the motor
            await server.model.power_motor.power_on()
            await asyncio.sleep(1)

            # Check the motor power is on
            msg_power_status = get_queue_message_latest(self.queue_tel, "powerStatus")

            self.assertGreater(msg_power_status["motorVoltage"], 22)
            self.assertGreater(msg_power_status["motorCurrent"], 7)
            self.assertGreater(msg_power_status["commVoltage"], 20)
            self.assertGreater(msg_power_status["commCurrent"], 5)

    async def test_event_get_mtmount_mount_in_position(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            inPosition = True
            await client_cmd.write_message(
                MsgType.Event,
                "mountInPosition",
                msg_details={"inPosition": inPosition},
                comp_name="MTMount",
            )

            await asyncio.sleep(0.5)

            self.assertEqual(server.model.mtmount_in_position, inPosition)

    async def test_cmd_load_configuration(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(MsgType.Command, "loadConfiguration")

            await asyncio.sleep(0.5)

            msg_config = collect_queue_messages(self.queue_cmd, "config")

            # 1 from the welcome message and 1 from the load configuration
            # command
            self.assertEqual(len(msg_config), 2)

    async def test_cmd_set_control_parameters(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write_message(
                MsgType.Command,
                "setControlParameters",
                msg_details={
                    "enableLutTemperature": True,
                    "enableLutInclinometer": True,
                    "useExternalElevationAngle": True,
                    "enableAngleComparison": True,
                    "maxAngleDifference": 2.0,
                },
            )

            await asyncio.sleep(0.5)

            msg_source = get_queue_message_latest(
                self.queue_cmd, "inclinationTelemetrySource"
            )
            self.assertEqual(
                msg_source["source"], MTM2.InclinationTelemetrySource.MTMOUNT.value
            )

    async def test_cmd_set_enabled_faults_mask(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            mask = 8
            await client_cmd.write_message(
                MsgType.Command,
                "setEnabledFaultsMask",
                msg_details={"mask": mask},
            )

            await asyncio.sleep(0.5)

            self.assertEqual(server.model.error_handler.enabled_faults_mask, mask)

            msg_mask = get_queue_message_latest(self.queue_cmd, "enabledFaultsMask")
            self.assertEqual(msg_mask["mask"], mask)

    async def test_cmd_set_configuration_file(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            file = "Configurable_File_Description_20180831T092326_M2_handling.csv"
            await client_cmd.write_message(
                MsgType.Command,
                "setConfigurationFile",
                msg_details={"file": file},
            )

            await asyncio.sleep(0.5)

            self.assertEqual(server._message_event.configuration_file, file)

            msg_config = get_queue_message_latest(self.queue_cmd, "config")
            self.assertEqual(msg_config["version"], "20180831T092326")

    async def test_cmd_set_hardpoint_list(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            hardpoints = [3, 13, 23, 73, 75, 77]
            await client_cmd.write_message(
                MsgType.Command,
                "setHardpointList",
                msg_details={"actuators": hardpoints},
            )

            await asyncio.sleep(2)

            self.assertEqual(server.model.control_closed_loop.hardpoints, hardpoints)

            msg_hd = get_queue_message_latest(self.queue_cmd, "hardpointList")
            self.assertEqual(msg_hd["actuators"], [4, 14, 24, 74, 76, 78])

    async def test_check_error_inclinometer(self) -> None:
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server._is_csc = False

            server.model.inclinometer_angle_external = 30.0
            server.model.control_parameters["enable_angle_comparison"] = True

            await asyncio.sleep(1.0)

            # Check the error code
            msg_status = get_queue_message_latest(self.queue_cmd, "summaryFaultsStatus")
            self.assertEqual(
                msg_status["status"],
                2**33,
            )


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
