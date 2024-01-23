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
import numpy.typing
from lsst.ts.m2com import (
    NUM_ACTUATOR,
    ActuatorDisplacementUnit,
    MockControlOpenLoop,
    MockPlant,
    get_config_dir,
    read_yaml_file,
)


class TestMockControlOpenLoop(unittest.TestCase):
    """Test the Mock open-loop control class."""

    def setUp(self) -> None:
        self.control_open_loop = MockControlOpenLoop()

        filepath = get_config_dir() / "harrisLUT" / "stiff_matrix_m2.yaml"
        stiffness_matrix = read_yaml_file(filepath)
        self.plant = MockPlant(np.array(stiffness_matrix["stiff"]), 120.0)

    def test_is_actuator_force_out_limit_default(self) -> None:
        (
            is_out_limit,
            limit_switch_retract,
            limit_switch_extend,
        ) = self.control_open_loop.is_actuator_force_out_limit(
            self.plant.get_actuator_forces()
        )

        self.assertFalse(is_out_limit)
        self.assertEqual(limit_switch_retract, [])
        self.assertEqual(limit_switch_extend, [])

    def test_is_actuator_force_out_limit_axial(self) -> None:
        # Maximum limit is not enabled
        actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)
        actuator_steps[0] = -6000
        self.plant.move_actuator_steps(actuator_steps)

        self.control_open_loop.is_running = True

        (
            is_out_limit,
            limit_switch_retract,
            limit_switch_extend,
        ) = self.control_open_loop.is_actuator_force_out_limit(
            self.plant.get_actuator_forces()
        )

        self.assertTrue(is_out_limit)
        self.assertEqual(limit_switch_retract, [0])
        self.assertEqual(limit_switch_extend, [])

        self.assertFalse(self.control_open_loop.is_running)

        # Maximum limit is enabled
        self.control_open_loop.open_loop_max_limit_is_enabled = True

        (
            is_out_limit,
            limit_switch_retract,
            limit_switch_extend,
        ) = self.control_open_loop.is_actuator_force_out_limit(
            self.plant.get_actuator_forces()
        )

        self.assertFalse(is_out_limit)

        actuator_steps[0] = -2000
        self.plant.move_actuator_steps(actuator_steps)

        (
            is_out_limit,
            limit_switch_retract,
            limit_switch_extend,
        ) = self.control_open_loop.is_actuator_force_out_limit(
            self.plant.get_actuator_forces()
        )

        self.assertTrue(is_out_limit)
        self.assertEqual(limit_switch_retract, [0])
        self.assertEqual(limit_switch_extend, [])

    def test_is_actuator_force_out_limit_tangent(self) -> None:
        # Maximum limit is not enabled
        actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)
        actuator_steps[-1] = 50000
        self.plant.move_actuator_steps(actuator_steps)

        self.control_open_loop.is_running = True

        (
            is_out_limit,
            limit_switch_retract,
            limit_switch_extend,
        ) = self.control_open_loop.is_actuator_force_out_limit(
            self.plant.get_actuator_forces()
        )

        self.assertTrue(is_out_limit)
        self.assertEqual(limit_switch_retract, [77])
        self.assertEqual(limit_switch_extend, [])

        self.assertFalse(self.control_open_loop.is_running)

        # Maximum limit is enabled
        self.control_open_loop.open_loop_max_limit_is_enabled = True

        (
            is_out_limit,
            limit_switch_retract,
            limit_switch_extend,
        ) = self.control_open_loop.is_actuator_force_out_limit(
            self.plant.get_actuator_forces()
        )

        self.assertFalse(is_out_limit)

        actuator_steps[-1] = 1000
        self.plant.move_actuator_steps(actuator_steps)

        (
            is_out_limit,
            limit_switch_retract,
            limit_switch_extend,
        ) = self.control_open_loop.is_actuator_force_out_limit(
            self.plant.get_actuator_forces()
        )

        self.assertTrue(is_out_limit)
        self.assertEqual(limit_switch_retract, [77])
        self.assertEqual(limit_switch_extend, [])

    def test_start_exception(self) -> None:
        self.assertRaises(
            ValueError,
            self.control_open_loop.start,
            [],
            1,
            ActuatorDisplacementUnit.Step,
        )

        self.control_open_loop.is_running = True
        self.assertRaises(
            RuntimeError,
            self.control_open_loop.start,
            [1],
            1,
            ActuatorDisplacementUnit.Step,
        )

    def test_start(self) -> None:
        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)

        self.assertTrue(self.control_open_loop.is_running)
        self.assertEqual(self.control_open_loop._displacement_steps, 2)

    def test_calculate_steps(self) -> None:
        self.assertEqual(
            self.control_open_loop._calculate_steps(
                MockPlant.STEP_TO_MM, ActuatorDisplacementUnit.Millimeter
            ),
            1,
        )

    def test_stop(self) -> None:
        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)

        self.control_open_loop.stop()

        self.assertFalse(self.control_open_loop.is_running)
        self.assertEqual(self.control_open_loop._displacement_steps, 0)

    def test_pause(self) -> None:
        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)

        self.control_open_loop.pause()

        self.assertFalse(self.control_open_loop.is_running)

    def test_resume_exception(self) -> None:
        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)
        self.control_open_loop.pause()

        self.control_open_loop.resume()

        self.assertTrue(self.control_open_loop.is_running)

    def test_resume(self) -> None:
        self.assertRaises(RuntimeError, self.control_open_loop.resume)

    def test_get_steps_to_move_exception(self) -> None:
        self.assertRaises(RuntimeError, self.control_open_loop.get_steps_to_move, 1)

        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)
        self.assertRaises(ValueError, self.control_open_loop.get_steps_to_move, -1)

    def test_get_steps_to_move_done(self) -> None:
        self.control_open_loop.start([1, 2], -2, ActuatorDisplacementUnit.Step)
        actuator_steps = self.control_open_loop.get_steps_to_move(3)

        self.plant.move_actuator_steps(actuator_steps)

        self.assertFalse(self.control_open_loop.is_running)

        self.assertEqual(self.plant.actuator_steps[1], -2)
        self.assertEqual(self.plant.actuator_steps[2], -2)

        self.assertEqual(self.control_open_loop._displacement_steps, 0)

    def test_run_steps_not_done(self) -> None:
        self.control_open_loop.start([1, 2], 4, ActuatorDisplacementUnit.Step)

        steps = 3
        actuator_steps = self.control_open_loop.get_steps_to_move(steps)

        self.plant.move_actuator_steps(actuator_steps)

        self.assertTrue(self.control_open_loop.is_running)

        self.assertEqual(self.plant.actuator_steps[1], steps)
        self.assertEqual(self.plant.actuator_steps[2], steps)

        self.assertEqual(self.control_open_loop._displacement_steps, 1)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
