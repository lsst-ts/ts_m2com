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

import unittest

import numpy as np
from lsst.ts.m2com import (
    NUM_ACTUATOR,
    NUM_TANGENT_LINK,
    MockControlClosedLoop,
    MockControlOpenLoop,
    get_config_dir,
)


class TestMockControlClosedLoop(unittest.TestCase):
    """Test the Mock closed-loop control class."""

    def setUp(self):

        self.control_closed_loop = MockControlClosedLoop()

        filepath_lut = get_config_dir() / "harrisLUT"
        self.control_closed_loop.load_file_lut(filepath_lut)

        filepath_cell_geometry = filepath_lut / "cell_geom.yaml"
        self.control_closed_loop.load_file_cell_geometry(filepath_cell_geometry)

        filepath_hardpoint = filepath_lut / "Hd_ax_Matrix_Params.csv"
        self.control_closed_loop.load_file_hardpoint_compensation(filepath_hardpoint)

    def test_init(self):

        self.assertEqual(len(self.control_closed_loop.temperature), 5)
        self.assertEqual(len(self.control_closed_loop._lut), 10)
        self.assertEqual(len(self.control_closed_loop._cell_geom), 3)
        self.assertEqual(self.control_closed_loop._hd_comp.shape, (72, 6))

    def test_simulate_temperature_and_update(self):

        temperature_original = self.control_closed_loop.temperature.copy()
        self.control_closed_loop.simulate_temperature_and_update()

        temperature_updated = self.control_closed_loop.temperature

        self.assertNotEqual(
            temperature_updated["ring"][0], temperature_original["ring"][0]
        )
        self.assertNotEqual(
            temperature_updated["intake"][0], temperature_original["intake"][0]
        )
        self.assertNotEqual(
            temperature_updated["exhaust"][0], temperature_original["exhaust"][0]
        )

    def test_set_measured_forces_exception(self):

        # Wrong dimension of axail actuators
        self.assertRaises(
            ValueError,
            self.control_closed_loop.set_measured_forces,
            np.zeros(NUM_ACTUATOR),
            np.zeros(NUM_TANGENT_LINK),
        )

        # Wrong dimension of tangent links
        self.assertRaises(
            ValueError,
            self.control_closed_loop.set_measured_forces,
            np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK),
            np.zeros(1),
        )

    def test_set_measured_forces(self):

        self.control_closed_loop.set_measured_forces(
            np.ones(NUM_ACTUATOR - NUM_TANGENT_LINK), 2 * np.ones(NUM_TANGENT_LINK)
        )

        self.assertEqual(
            np.sum(self.control_closed_loop.axial_forces["measured"]),
            NUM_ACTUATOR - NUM_TANGENT_LINK,
        )
        self.assertEqual(
            np.sum(self.control_closed_loop.tangent_forces["measured"]),
            2 * NUM_TANGENT_LINK,
        )

    def test_is_cell_temperature_high(self):

        # Temperature is normal
        self.assertFalse(self.control_closed_loop.is_cell_temperature_high())

        # Temperature is high
        self.control_closed_loop.temperature["exhaust"] = [99, 99]

        self.assertTrue(self.control_closed_loop.is_cell_temperature_high())

    def test_apply_forces(self):

        force_axial, force_tangent = self._apply_forces()

        np.testing.assert_array_equal(
            self.control_closed_loop.axial_forces["applied"], force_axial
        )
        np.testing.assert_array_equal(
            self.control_closed_loop.tangent_forces["applied"], force_tangent
        )

    def _apply_forces(self):

        force_axial = [1] * (NUM_ACTUATOR - NUM_TANGENT_LINK)
        force_tangent = [2] * NUM_TANGENT_LINK
        self.control_closed_loop.apply_forces(force_axial, force_tangent)

        return force_axial, force_tangent

    def test_get_demanded_force_axial(self):

        # Without the applying of force
        self.control_closed_loop.calc_look_up_forces(59.06)
        demanded_force = self.control_closed_loop.get_demanded_force()

        self.assertAlmostEqual(demanded_force[0], 130.3581281)
        self.assertAlmostEqual(demanded_force[1], 166.0471629)

        # Applying the force
        self._apply_forces()
        demanded_force_apply = self.control_closed_loop.get_demanded_force()

        self.assertAlmostEqual(demanded_force_apply[0], 131.3581281)

    def test_get_demanded_force_tangent(self):

        # Without the applying of force
        self.control_closed_loop.calc_look_up_forces(59.06)
        demanded_force = self.control_closed_loop.get_demanded_force()

        self.assertAlmostEqual(demanded_force[72], 16.113)
        self.assertAlmostEqual(demanded_force[73], -131.72)

        # Applying the force
        self._apply_forces()
        demanded_force_apply = self.control_closed_loop.get_demanded_force()

        self.assertAlmostEqual(demanded_force_apply[72], 18.113)

    def test_is_actuator_force_out_limit_axial(self):

        applied_force_axial = [0] * (NUM_ACTUATOR - NUM_TANGENT_LINK)
        applied_force_axial[2] = 999

        self.assertTrue(
            self.control_closed_loop.is_actuator_force_out_limit(
                applied_force_axial=applied_force_axial
            )[0]
        )

    def test_is_actuator_force_out_limit_tangent(self):

        applied_force_tangent = [0] * NUM_TANGENT_LINK
        applied_force_tangent[2] = 9999

        self.assertTrue(
            self.control_closed_loop.is_actuator_force_out_limit(
                applied_force_tangent=applied_force_tangent
            )[0]
        )

    def test_reset_force_offsets(self):

        self._apply_forces()

        self.control_closed_loop.reset_force_offsets()

        np.testing.assert_array_equal(
            self.control_closed_loop.axial_forces["applied"],
            [0] * (NUM_ACTUATOR - NUM_TANGENT_LINK),
        )
        np.testing.assert_array_equal(
            self.control_closed_loop.tangent_forces["applied"], [0] * NUM_TANGENT_LINK
        )

    def test_get_net_forces_total(self):

        self.control_closed_loop.axial_forces["measured"][1:3] = np.array([1, 2])
        self.control_closed_loop.tangent_forces["measured"] = np.array(
            [1, 2, 3, 4, 5, 6]
        )

        net_forces_total = self.control_closed_loop.get_net_forces_total()

        self.assertAlmostEqual(net_forces_total["fx"], -3)
        self.assertAlmostEqual(net_forces_total["fy"], -5.1961524)
        self.assertAlmostEqual(net_forces_total["fz"], 3)

    def test_get_net_moments_total(self):

        self.control_closed_loop.axial_forces["measured"][1:4] = np.array([1, 2, 3])
        self.control_closed_loop.tangent_forces["measured"] = np.array(
            [1, 2, 3, 4, 5, 6]
        )

        net_moments_total = self.control_closed_loop.get_net_moments_total()

        self.assertAlmostEqual(net_moments_total["mx"], 8.37691)
        self.assertAlmostEqual(net_moments_total["my"], 4.45836999)
        self.assertAlmostEqual(net_moments_total["mz"], 37.3839844)

    def test_get_force_balance(self):

        force_balance = self.control_closed_loop.get_force_balance()
        self.assertEqual(len(force_balance), 6)

    def test_calc_look_up_forces(self):

        self.control_closed_loop.calc_look_up_forces(59.06)

        # Check the length
        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK

        self.assertEqual(
            len(self.control_closed_loop.axial_forces["lutTemperature"]),
            num_axial_actuators,
        )
        self.assertEqual(
            len(self.control_closed_loop.axial_forces["hardpointCorrection"]),
            num_axial_actuators,
        )

        self.assertEqual(
            len(self.control_closed_loop.axial_forces["lutGravity"]),
            num_axial_actuators,
        )
        self.assertEqual(
            len(self.control_closed_loop.tangent_forces["lutGravity"]), NUM_TANGENT_LINK
        )
        self.assertEqual(
            len(self.control_closed_loop.tangent_forces["hardpointCorrection"]),
            NUM_TANGENT_LINK,
        )

        # Check the value
        self.assertAlmostEqual(
            self.control_closed_loop.axial_forces["lutTemperature"][0], 3.5022743
        )
        self.assertAlmostEqual(
            self.control_closed_loop.axial_forces["lutGravity"][0], 126.8558538
        )

        self.assertAlmostEqual(
            self.control_closed_loop.tangent_forces["lutGravity"][0], 16.113
        )
        self.assertAlmostEqual(
            self.control_closed_loop.tangent_forces["lutGravity"][1], -131.72
        )

        np.testing.assert_array_equal(
            self.control_closed_loop.tangent_forces["lutTemperature"], np.array([])
        )

    def test_calc_look_up_forces_temperature(self):

        lut_temperature = self._get_temperature()
        (
            force_r,
            force_x,
            force_y,
            force_u,
        ) = self.control_closed_loop._calc_look_up_forces_temperature(
            lut_temperature, 21 * np.ones(12)
        )

        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK

        self.assertEqual(len(force_r), num_axial_actuators)
        self.assertEqual(len(force_x), num_axial_actuators)
        self.assertEqual(len(force_y), num_axial_actuators)
        self.assertEqual(len(force_u), num_axial_actuators)

        self.assertAlmostEqual(force_r[0], 0.0394233)
        self.assertAlmostEqual(force_r[1], 0.1676541)
        self.assertAlmostEqual(force_r[2], -0.1255652)

        self.assertAlmostEqual(force_x[0], -0.0233514)
        self.assertAlmostEqual(force_x[1], -0.0780565)
        self.assertAlmostEqual(force_x[2], 0.044407)

        self.assertAlmostEqual(force_y[0], 0.0115937)
        self.assertAlmostEqual(force_y[1], -0.0090808)
        self.assertAlmostEqual(force_y[2], 0.020446)

        self.assertAlmostEqual(force_u[0], 3.4746087)
        self.assertAlmostEqual(force_u[1], -4.4150503)
        self.assertAlmostEqual(force_u[2], -0.2455199)

    def _get_temperature(self):
        return np.array(
            [
                24.49,
                24.49,
                26.53,
                24.49,
                24.49,
                24.49,
                24.49,
                26.53,
                26.53,
                26.53,
                24.49,
                24.49,
            ]
        )

    def test_calc_look_up_forces_gravity(self):

        (
            force_elevation,
            force_0g_component,
            force_actuator_bias,
            force_factory_offset,
        ) = self.control_closed_loop._calc_look_up_forces_gravity(59.06)

        self.assertAlmostEqual(force_elevation[0], 261.1468838)
        self.assertAlmostEqual(force_elevation[1], 213.176983)
        self.assertAlmostEqual(force_elevation[2], 183.7221491)

        self.assertAlmostEqual(force_0g_component[0], -137.444)
        self.assertAlmostEqual(force_0g_component[1], -42.924)
        self.assertAlmostEqual(force_0g_component[2], -26.955)

        self.assertAlmostEqual(force_actuator_bias[0], 3.15297)
        self.assertAlmostEqual(force_actuator_bias[1], 0.1287134)
        self.assertAlmostEqual(force_actuator_bias[2], 4.3287288)
        self.assertAlmostEqual(force_actuator_bias[-3], -30.391)
        self.assertAlmostEqual(force_actuator_bias[-2], -9.0112)
        self.assertAlmostEqual(force_actuator_bias[-1], -28.964)

        self.assertAlmostEqual(force_factory_offset[0], 0.0)
        self.assertAlmostEqual(force_factory_offset[1], 0.0)
        self.assertAlmostEqual(force_factory_offset[2], 0.0)

    def test_calc_force_hardpoint(self):

        angle = 120
        force_measured, lut_angle = self._get_force_measured_and_lut_angle(angle)
        force_demanded = self._get_force_demanded(lut_angle)

        force_hardpoint = self.control_closed_loop._calc_look_up_forces_hardpoint(
            force_demanded, force_measured
        )

        self.assertAlmostEqual(force_hardpoint[0], -58.5897104)
        self.assertAlmostEqual(force_hardpoint[1], -58.0630293)
        self.assertAlmostEqual(force_hardpoint[2], -56.7941411)
        self.assertAlmostEqual(force_hardpoint[5], 0)
        self.assertAlmostEqual(force_hardpoint[15], 0)
        self.assertAlmostEqual(force_hardpoint[-6], -96.9923333)
        self.assertAlmostEqual(force_hardpoint[-5], 0)
        self.assertAlmostEqual(force_hardpoint[-4], 1902.7131771)
        self.assertAlmostEqual(force_hardpoint[-3], 0)
        self.assertAlmostEqual(force_hardpoint[-2], -1996.7958437)

    def _get_force_measured_and_lut_angle(self, angle):

        control_open_loop = MockControlOpenLoop()
        return control_open_loop.get_forces_mirror_weight(
            angle
        ), control_open_loop.correct_inclinometer_angle(angle)

    def _get_force_demanded(self, angle):

        (
            force_elevation,
            force_0g_component,
            force_actuator_bias,
            force_factory_offset,
        ) = self.control_closed_loop._calc_look_up_forces_gravity(angle)

        lut_temperature = self._get_temperature()
        (
            force_r,
            force_x,
            force_y,
            force_u,
        ) = self.control_closed_loop._calc_look_up_forces_temperature(
            lut_temperature, 21 * np.ones(12)
        )
        force_r = np.append(force_r, np.zeros(NUM_TANGENT_LINK))
        force_x = np.append(force_x, np.zeros(NUM_TANGENT_LINK))
        force_y = np.append(force_y, np.zeros(NUM_TANGENT_LINK))
        force_u = np.append(force_u, np.zeros(NUM_TANGENT_LINK))

        return (
            force_elevation
            + force_0g_component
            + force_actuator_bias
            + force_factory_offset
            + force_r
            + force_x
            + force_y
            + force_u
        )

    def test_handle_forces_function(self):

        # No update of force now
        self.control_closed_loop.handle_forces()

        value_original = self.control_closed_loop.tangent_forces["measured"][0]
        self.assertEqual(value_original, 0.0)

        # There is the update of force
        self.control_closed_loop.is_running = True
        self.control_closed_loop.handle_forces()

        value_updated = self.control_closed_loop.tangent_forces["measured"][0]
        self.assertNotEqual(value_updated, 0.0)

    def test_handle_forces(self):

        self.control_closed_loop.is_running = True

        angle = 120
        force_measured, lut_angle = self._get_force_measured_and_lut_angle(angle)
        self.control_closed_loop.calc_look_up_forces(lut_angle)

        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        self.control_closed_loop.set_measured_forces(
            force_measured[:num_axial_actuators], force_measured[-NUM_TANGENT_LINK:]
        )

        in_position_happened = False
        for counter in range(1000):
            in_position = self.control_closed_loop.handle_forces(
                force_rms=0.5, force_per_cycle=5
            )
            if (not in_position_happened) and in_position:
                in_position_happened = True

        self.assertTrue(in_position_happened)

        self.assertTrue(self.control_closed_loop.in_position_hardpoints)

    def test_force_dynamics_in_position(self):

        self._prepare_force(0.01, 0.001)
        in_position, final_force = self.control_closed_loop._force_dynamics(0.5, 5)

        self.assertTrue(in_position)
        self.assertEqual(final_force[0], 0)
        self.assertEqual(final_force[5], 0)

        self.assertTrue(self.control_closed_loop.in_position_hardpoints)

    def _prepare_force(self, coef_lut, coef_hardpoint):

        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        self.control_closed_loop.axial_forces["lutGravity"] = coef_lut * np.ones(
            num_axial_actuators
        )
        self.control_closed_loop.axial_forces[
            "hardpointCorrection"
        ] = coef_hardpoint * np.ones(num_axial_actuators)

        self.control_closed_loop.tangent_forces["lutGravity"] = coef_lut * np.ones(
            NUM_TANGENT_LINK
        )
        self.control_closed_loop.tangent_forces[
            "hardpointCorrection"
        ] = coef_hardpoint * np.ones(NUM_TANGENT_LINK)

    def test_force_dynamics_not_in_position_small(self):

        self._prepare_force(5, 1)
        in_position, final_force = self.control_closed_loop._force_dynamics(0.5, 5)

        self.assertFalse(in_position)
        self.assertEqual(final_force[0], 4)
        self.assertEqual(final_force[5], 5)

        self.assertFalse(self.control_closed_loop.in_position_hardpoints)

    def test_force_dynamics_not_in_position_big(self):

        self._prepare_force(10, 1)
        in_position, final_force = self.control_closed_loop._force_dynamics(0.5, 5)

        self.assertFalse(in_position)
        self.assertEqual(final_force[0], 5)
        self.assertEqual(final_force[5], 5)

        self.assertFalse(self.control_closed_loop.in_position_hardpoints)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
