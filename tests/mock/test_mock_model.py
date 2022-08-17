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

import numpy as np
import unittest

from lsst.ts.idl.enums import MTM2

from lsst.ts.m2com import (
    MockModel,
    PowerType,
    NUM_ACTUATOR,
    NUM_TANGENT_LINK,
    TEST_DIGITAL_OUTPUT_NO_POWER,
    TEST_DIGITAL_OUTPUT_POWER_COMM,
    TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR,
    TEST_DIGITAL_INPUT_NO_POWER,
    TEST_DIGITAL_INPUT_POWER_COMM,
    TEST_DIGITAL_INPUT_POWER_COMM_MOTOR,
    get_config_dir,
)


class TestMockModel(unittest.TestCase):
    """Test the Mock Model class."""

    def setUp(self):

        self.model = MockModel()
        self.model.configure(get_config_dir(), "harrisLUT")

    def test_init(self):

        # In the default condition, there should be no force error
        self.assertFalse(self.model.is_actuator_force_out_limit())

        self.assertAlmostEqual(
            self.model.control_closed_loop.axial_forces["measured"][0], 216.2038167
        )
        self.assertEqual(
            self.model.control_closed_loop.tangent_forces["measured"][0], 0
        )
        self.assertAlmostEqual(
            self.model.control_closed_loop.tangent_forces["measured"][1], -63.8528153
        )

        self.assertAlmostEqual(
            self.model.control_closed_loop.axial_forces["hardpointCorrection"][0],
            -27.8015275,
        )

    def test_get_default_mirror_position(self):

        mirror_position = self.model.get_default_mirror_position()

        self.assertEqual(len(mirror_position), 6)
        for value in mirror_position.values():
            self.assertEqual(value, 0)

    def test_set_inclinometer_angle(self):

        self.model.set_inclinometer_angle(120)

        self.assertEqual(self.model.control_open_loop.inclinometer_angle, 120)
        self.assertAlmostEqual(
            self.model.control_closed_loop.axial_forces["hardpointCorrection"][0],
            -89.3301199,
        )

    def test_is_actuator_force_out_limit_closed_loop(self):

        self.model.control_closed_loop.is_running = True
        self.model.control_closed_loop.axial_forces["applied"][0] = 1000

        self.assertTrue(self.model.is_actuator_force_out_limit())

    def test_is_actuator_force_out_limit(self):

        # By default, use the result from the open-loop control.
        self.model.control_open_loop.actuator_steps[0] -= 6000

        self.assertTrue(self.model.is_actuator_force_out_limit())

    def test_fault_motor_power_not_on(self):

        self.model.fault()
        self.assertFalse(self.model.error_cleared)
        self.assertFalse(self.model.control_closed_loop.is_running)

    def test_fault_motor_power_on(self):

        self.model.script_engine.is_running = True
        self.model.control_open_loop.is_running = True

        # Turn on the power and the switch should work
        self.model.motor_power_on = True
        self.assertTrue(self.model.switch_force_balance_system(True))
        self.assertTrue(self.model.control_closed_loop.is_running)

        # Fault the model
        self.model.fault()
        self.assertFalse(self.model.error_cleared)

        self.assertFalse(self.model.script_engine.is_running)
        self.assertFalse(self.model.control_open_loop.is_running)

        self.assertFalse(self.model.control_closed_loop.is_running)

    def test_switch_force_balance_system(self):

        # This should fail
        self.assertFalse(self.model.switch_force_balance_system(True))
        self.assertFalse(self.model.control_closed_loop.is_running)

        # The system should not run the open-loop control when the force
        # balance system is on
        self.model.control_open_loop.is_running = True

        # Turn on the power and the switch should work
        self.model.motor_power_on = True

        self.assertTrue(self.model.switch_force_balance_system(True))
        self.assertFalse(self.model.control_open_loop.is_running)
        self.assertTrue(self.model.control_closed_loop.is_running)

        # Switch off the force balance system
        self.assertTrue(self.model.switch_force_balance_system(False))
        self.assertFalse(self.model.control_closed_loop.is_running)

    def test_clear_errors(self):

        self.model.error_cleared = False
        self.model.clear_errors()

        self.assertTrue(self.model.error_cleared)

    def test_select_inclination_source(self):

        self.assertEqual(
            self.model.inclination_source, MTM2.InclinationTelemetrySource.ONBOARD
        )

        self.model.select_inclination_source(2)

        self.assertEqual(
            self.model.inclination_source, MTM2.InclinationTelemetrySource.MTMOUNT
        )

    def test_get_telemetry_data(self):

        # No power
        telemetry_data = self.model.get_telemetry_data()

        self.assertFalse(self.model.in_position)
        self.assertEqual(len(telemetry_data), 2)

        # With power
        self.model.motor_power_on = True
        self.model.switch_force_balance_system(True)
        force_axial, force_tangent = self._apply_forces()

        telemetry_data = self.model.get_telemetry_data()

        self.assertFalse(self.model.in_position)
        self.assertEqual(len(telemetry_data), 16)

        # Check the steps and positions
        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK

        axial_actuator_steps = telemetry_data["axialActuatorSteps"]
        self.assertEqual(len(axial_actuator_steps["steps"]), num_axial_actuators)

        axial_actuator_positions = telemetry_data["axialEncoderPositions"]
        self.assertEqual(len(axial_actuator_positions["position"]), num_axial_actuators)

        tangent_actuator_steps = telemetry_data["tangentActuatorSteps"]
        self.assertEqual(len(tangent_actuator_steps["steps"]), NUM_TANGENT_LINK)

        tangent_actuator_positions = telemetry_data["tangentEncoderPositions"]
        self.assertEqual(len(tangent_actuator_positions["position"]), NUM_TANGENT_LINK)

    def _apply_forces(self):

        force_axial = [1] * (NUM_ACTUATOR - NUM_TANGENT_LINK)
        force_tangent = [2] * NUM_TANGENT_LINK
        self.model.control_closed_loop.apply_forces(force_axial, force_tangent)

        return force_axial, force_tangent

    def test_get_power_status(self):

        # No power
        power_status_no_power = self.model._get_power_status()

        self.assertLess(abs(power_status_no_power["motorVoltage"]), 1)
        self.assertLess(abs(power_status_no_power["motorCurrent"]), 1)
        self.assertLess(abs(power_status_no_power["commVoltage"]), 1)
        self.assertLess(abs(power_status_no_power["commCurrent"]), 1)

        # Only communication power
        self.model.communication_power_on = True
        power_status_power_comm = self.model._get_power_status()

        self.assertLess(abs(power_status_power_comm["motorVoltage"]), 1)
        self.assertLess(abs(power_status_power_comm["motorCurrent"]), 1)
        self.assertGreater(power_status_power_comm["commVoltage"], 20)
        self.assertGreater(power_status_power_comm["commCurrent"], 5)

        # All powers on
        self.model.motor_power_on = True
        power_status_power_comm_motor = self.model._get_power_status()

        self.assertGreater(power_status_power_comm_motor["motorVoltage"], 22)
        self.assertGreater(power_status_power_comm_motor["motorCurrent"], 7)
        self.assertGreater(power_status_power_comm_motor["commVoltage"], 20)
        self.assertGreater(power_status_power_comm_motor["commCurrent"], 5)

    def test_get_ilc_data(self):

        for idx in range(30):
            ilc_status = self.model._get_ilc_data()["status"]
            self.assertEqual(ilc_status[0], idx % 16)

    def test_get_displacement_sensors(self):

        # Zero position
        displacement_ims = self.model._get_displacement_sensors()

        position = self._calc_position_ims(displacement_ims)

        np.testing.assert_array_almost_equal(
            position, list(self.model.mirror_position.values())
        )

        # Non-zero position
        mirror_position = {
            "x": 10,
            "y": 20,
            "z": 30,
            "xRot": 40,
            "yRot": 50,
            "zRot": 60,
        }
        displacement_ims = self.model._get_displacement_sensors(
            mirror_position=mirror_position
        )

        position = self._calc_position_ims(displacement_ims)

        np.testing.assert_array_almost_equal(position, list(mirror_position.values()))

    def _calc_position_ims(self, displacement_ims):
        """Calculate the position based on the reading of IMS.

        Parameters
        ----------
        displacement_ims : `dict`
            Data of displacement sensors.

        Returns
        -------
        `numpy.ndarray`
            Calculated mirror position based on IMS.
        """

        # Get the position with the same order of matrix and offset
        # The unit needs to change to mm from um
        theta_z = np.array(displacement_ims["thetaZ"])
        delta_z = np.array(displacement_ims["deltaZ"])
        position_ims = np.append(theta_z, delta_z) * 1e-3

        index_array = np.array([8, 10, 4, 6, 0, 2, 9, 11, 5, 7, 1, 3])
        position_ims_update = position_ims[index_array.argsort()]

        matrix = np.array(self.model._disp_ims["matrix"])
        offset = np.array(self.model._disp_ims["offset"])

        return matrix.dot((position_ims_update - offset).reshape(-1, 1)).ravel()

    def test_balance_forces_and_steps_exception(self):

        self.model.control_open_loop.is_running = True
        self.model.control_closed_loop.is_running = True

        self.assertRaises(RuntimeError, self.model.balance_forces_and_steps)

    def test_balance_forces_and_steps(self):

        self.model.control_closed_loop.is_running = True

        # In the initial beginning, the actuators are not in position
        self.model.balance_forces_and_steps()
        self.assertFalse(self.model.in_position)

        # After some running of closed-loop control, the actuators are in
        # position
        in_position_happened = False
        for idx in range(100):
            self.model.balance_forces_and_steps()

            if (not in_position_happened) and self.model.in_position:
                in_position_happened = True

        self.assertTrue(in_position_happened)

        self.assertTrue(self.model.control_closed_loop.in_position_hardpoints)

    def test_simulate_zenith_angle(self):

        zenith_angle = self.model._simulate_zenith_angle()
        self.assertLess(np.abs(zenith_angle["measured"]), 2)
        self.assertLess(np.abs(zenith_angle["inclinometerRaw"] - 90), 2)
        self.assertLess(np.abs(zenith_angle["inclinometerProcessed"] - 90), 2)

    def test_handle_position_mirror(self):

        mirror_position_set_point = dict(
            [(axis, 1.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
        )

        # This should fail
        self.assertFalse(self.model.handle_position_mirror(mirror_position_set_point))

        # This should fail again
        self.model.motor_power_on = True
        self.assertFalse(self.model.handle_position_mirror(mirror_position_set_point))

        # This should succeed in the final
        self.model.switch_force_balance_system(True)
        result = self.model.handle_position_mirror(mirror_position_set_point)

        self.assertTrue(result)
        self.assertEqual(self.model.mirror_position, mirror_position_set_point)

    def test_enable_open_loop_max_limit(self):

        self.model.enable_open_loop_max_limit(True)
        self.assertTrue(self.model.control_open_loop.open_loop_max_limit_is_enabled)

        self.model.enable_open_loop_max_limit(False)
        self.assertFalse(self.model.control_open_loop.open_loop_max_limit_is_enabled)

        # The closed-loop control does not allow the maximum limit of
        # open-loop control
        self.model.enable_open_loop_max_limit(True)

        self.model.motor_power_on = True
        self.assertTrue(self.model.switch_force_balance_system(True))

        self.assertFalse(self.model.control_open_loop.open_loop_max_limit_is_enabled)

    def test_reset_breakers(self):

        # These should fail
        self.assertFalse(self.model.reset_breakers(PowerType.Motor))
        self.assertFalse(self.model.reset_breakers(PowerType.Communication))
        self.assertFalse(self.model.reset_breakers(3))

        # Reset the breakers of communication
        self.model.communication_power_on = True
        self.assertTrue(self.model.reset_breakers(PowerType.Communication))

        # Reset the breakers of motor
        self.model.motor_power_on = True
        self.assertTrue(self.model.reset_breakers(PowerType.Motor))

    def test_get_digital_output(self):

        self.assertEqual(self.model.get_digital_output(), TEST_DIGITAL_OUTPUT_NO_POWER)

        self.model.communication_power_on = True
        self.assertEqual(
            self.model.get_digital_output(), TEST_DIGITAL_OUTPUT_POWER_COMM
        )

        self.model.motor_power_on = True
        self.assertEqual(
            self.model.get_digital_output(), TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR
        )

    def test_get_digital_input(self):

        self.assertEqual(self.model.get_digital_input(), TEST_DIGITAL_INPUT_NO_POWER)

        self.model.communication_power_on = True
        self.assertEqual(self.model.get_digital_input(), TEST_DIGITAL_INPUT_POWER_COMM)

        self.model.motor_power_on = True
        self.assertEqual(
            self.model.get_digital_input(), TEST_DIGITAL_INPUT_POWER_COMM_MOTOR
        )


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
