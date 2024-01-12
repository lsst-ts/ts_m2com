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
    MockControlClosedLoop,
    MockControlLoop,
    MockPlant,
    correct_inclinometer_angle,
    get_config_dir,
    read_yaml_file,
)


class TestMockControlLoop(unittest.TestCase):
    """Test the Mock Control Loop class."""

    def setUp(self) -> None:
        (
            gain_prefilter,
            params_prefilter,
        ) = MockControlClosedLoop.calc_cmd_prefilter_params()

        params_cmd_delay = MockControlClosedLoop.calc_cmd_delay_filter_params(
            is_mirror=False
        )

        (
            gain_control_filter,
            params_control_filter,
        ) = MockControlClosedLoop.calc_force_control_filter_params(is_mirror=False)

        self.control_closed_loop = self._get_control_closed_loop()

        # Influence matrix and decoupling matrix are the same at this moment
        self.control_loop = MockControlLoop(
            gain_prefilter,
            np.array(params_prefilter),
            gain_prefilter,
            np.array(params_prefilter),
            params_cmd_delay,
            params_cmd_delay,
            gain_control_filter,
            np.array(params_control_filter),
            gain_control_filter,
            np.array(params_control_filter),
            self.control_closed_loop._kdc,
            self.control_closed_loop._kdc,
            self.control_closed_loop._hd_comp,
            [0.0, 0.0],
            [0.0, 0.0],
            0.28,
            0.1,
            1,
            10,
            40,
            75,
            75,
        )

        inclinometer_angle = 120.0
        self.plant = self._get_plant(inclinometer_angle)

        self.control_closed_loop.calc_look_up_forces(
            correct_inclinometer_angle(inclinometer_angle)
        )

    def _get_control_closed_loop(self) -> MockControlClosedLoop:
        control_closed_loop = MockControlClosedLoop()

        filepath_lut = get_config_dir() / "harrisLUT"
        control_closed_loop.load_file_lut(filepath_lut)

        filepath_cell_geometry = filepath_lut / "cell_geom.yaml"
        control_closed_loop.load_file_cell_geometry(filepath_cell_geometry)

        control_closed_loop.set_hardpoint_compensation()

        filepath_stiffness = filepath_lut / "stiff_matrix_surrogate.yaml"
        control_closed_loop.load_file_stiffness(filepath_stiffness)

        control_closed_loop.set_kinetic_decoupling_matrix()

        return control_closed_loop

    def _get_plant(self, inclinometer_angle: float) -> MockPlant:
        # We intensionally use the M2 stiff matrix here to test the control
        # loop can converge or not.
        filepath = get_config_dir() / "harrisLUT" / "stiff_matrix_m2.yaml"
        stiffness_matrix = read_yaml_file(filepath)
        return MockPlant(np.array(stiffness_matrix["stiff"]), inclinometer_angle)

    def test_calc_actuator_steps(self) -> None:
        # The calculation should converge
        for idx in range(60):
            force_demanded = self.control_closed_loop.get_demanded_force()
            force_measured = self.plant.get_actuator_forces()

            steps, hardpoint_correction = self.control_loop.calc_actuator_steps(
                force_demanded,
                force_measured,
                self.control_closed_loop.hardpoints,
                False,
            )

            # Check the saturated hardpoint correction before the integration
            # with the plant model.
            if idx == 10:
                self.assertAlmostEqual(hardpoint_correction[0], 58.5897286)
                self.assertAlmostEqual(hardpoint_correction[1], 58.0630028)
                self.assertAlmostEqual(hardpoint_correction[-1], 1996.7958437)
                self.assertAlmostEqual(hardpoint_correction[-2], -1902.7131771)

            # We need to let the calculation of hardpoint_correction saturates
            # first before the integration with the plant model.
            if idx >= 10:
                self.plant.move_actuator_steps(steps)

        self.assertAlmostEqual(hardpoint_correction[0], -19.9076253)
        self.assertAlmostEqual(hardpoint_correction[1], -19.5104835)
        self.assertAlmostEqual(hardpoint_correction[-1], 2002.7442391)
        self.assertAlmostEqual(hardpoint_correction[-2], -1947.9413411)

        actuator_steps = self.plant.actuator_steps
        self.assertEqual(actuator_steps[0], -236)
        self.assertEqual(actuator_steps[1], -743)
        self.assertEqual(actuator_steps[-2], -100)
        self.assertEqual(actuator_steps[-4], 573)

        self.assertEqual(np.sum(np.abs(steps)), 0)

    def test_reset(self) -> None:
        self.control_loop._gain_schedular._sample_settle = 0

        self.control_loop.reset()

        self.assertEqual(
            self.control_loop._gain_schedular._sample_settle,
            self.control_loop._gain_schedular._max_sample_settle,
        )

    def test_split_1d_array(self) -> None:
        array_idx, array_no_idx = self.control_loop._split_1d_array(
            np.array([1.0, 2.0, 3.0, 4.0]), [1, 2]
        )

        np.testing.assert_equal(array_idx, np.array([2.0, 3.0]))
        np.testing.assert_equal(array_no_idx, np.array([1.0, 4.0]))

    def test_saturate_actuator_steps(self) -> None:
        actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)
        actuator_steps[0] = 100
        actuator_steps[1] = -100
        actuator_steps[-1] = 100
        actuator_steps[-2] = -100

        actuator_steps_saturated = self.control_loop._saturate_actuator_steps(
            actuator_steps
        )

        self.assertEqual(
            actuator_steps_saturated[0], self.control_loop._step_limit_axial
        )
        self.assertEqual(
            actuator_steps_saturated[1], -self.control_loop._step_limit_axial
        )

        self.assertEqual(
            actuator_steps_saturated[-1], self.control_loop._step_limit_tangent
        )
        self.assertEqual(
            actuator_steps_saturated[-2], -self.control_loop._step_limit_tangent
        )


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
