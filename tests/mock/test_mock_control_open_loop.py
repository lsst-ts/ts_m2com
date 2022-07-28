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
import pathlib

import numpy as np

from lsst.ts.m2com import MockControlOpenLoop, NUM_ACTUATOR, ActuatorDisplacementUnit


class TestMockControlOpenLoop(unittest.TestCase):
    """Test the Mock open-loop control class."""

    def setUp(self):

        self.control_open_loop = MockControlOpenLoop()
        self.control_open_loop.inclinometer_angle = 120

        self.path_file_static_transfer_matrix = (
            pathlib.Path(__file__).parents[0]
            / ".."
            / "harrisLUT"
            / "StaticTransferMatrix.csv"
        )

        self.control_open_loop.read_file_static_transfer_matrix(
            self.path_file_static_transfer_matrix
        )

    def test_read_file_static_transfer_matrix(self):

        self.control_open_loop.read_file_static_transfer_matrix(
            self.path_file_static_transfer_matrix
        )

        matrix = self.control_open_loop._static_transfer_matrix
        self.assertEqual(matrix.shape, (78, 78))
        self.assertAlmostEqual(matrix[0, 0], -0.0571288)
        self.assertAlmostEqual(matrix[0, 1], 0.0060631)
        self.assertAlmostEqual(matrix[1, 0], 0.0060558)
        self.assertAlmostEqual(matrix[1, 1], -0.0540609)

    def test_update_actuator_steps_exception(self):

        self.assertRaises(
            ValueError, self.control_open_loop.update_actuator_steps, np.zeros(3)
        )

        self.assertRaises(
            ValueError,
            self.control_open_loop.update_actuator_steps,
            np.zeros(NUM_ACTUATOR),
        )

    def test_update_actuator_steps(self):

        actuator_steps = np.ones(NUM_ACTUATOR, dtype=int)
        self.control_open_loop.update_actuator_steps(actuator_steps)

        self.assertEqual(np.sum(self.control_open_loop.actuator_steps), NUM_ACTUATOR)

    def test_correct_inclinometer_angle(self):

        self.assertEqual(self.control_open_loop.correct_inclinometer_angle(-10), 90)

        self.assertAlmostEqual(
            self.control_open_loop.correct_inclinometer_angle(0), -180.94
        )
        self.assertAlmostEqual(
            self.control_open_loop.correct_inclinometer_angle(30), -210.94
        )
        self.assertEqual(self.control_open_loop.correct_inclinometer_angle(89.06), 90)
        self.assertAlmostEqual(
            self.control_open_loop.correct_inclinometer_angle(90), 89.06
        )
        self.assertAlmostEqual(
            self.control_open_loop.correct_inclinometer_angle(120), 59.06
        )
        self.assertAlmostEqual(
            self.control_open_loop.correct_inclinometer_angle(200), -20.94
        )

        self.assertEqual(self.control_open_loop.correct_inclinometer_angle(500), -270)

    def test_get_forces_mirror_weight(self):

        forces = self.control_open_loop.get_forces_mirror_weight(120)

        self.assertAlmostEqual(forces[0], 185.4643083)

        self.assertEqual(forces[72], 0)
        self.assertAlmostEqual(forces[73], -2001.1325104)
        self.assertEqual(forces[75], 0)
        self.assertAlmostEqual(forces[76], 2001.1325104)

    def test_is_actuator_force_out_limit_default(self):

        self.assertFalse(self.control_open_loop.is_actuator_force_out_limit())

    def test_is_actuator_force_out_limit_axial(self):

        # Maximum limit is not enabled
        self.control_open_loop.actuator_steps[0] -= 6000
        self.assertTrue(self.control_open_loop.is_actuator_force_out_limit())

        # Maximum limit is enabled
        self.control_open_loop.open_loop_max_limit_is_enabled = True
        self.assertFalse(self.control_open_loop.is_actuator_force_out_limit())

        self.control_open_loop.actuator_steps[0] -= 2000
        self.assertTrue(self.control_open_loop.is_actuator_force_out_limit())

    def test_is_actuator_force_out_limit_tangent(self):

        # Maximum limit is not enabled
        self.control_open_loop.actuator_steps[-1] += 50000
        self.assertTrue(self.control_open_loop.is_actuator_force_out_limit())

        # Maximum limit is enabled
        self.control_open_loop.open_loop_max_limit_is_enabled = True
        self.assertFalse(self.control_open_loop.is_actuator_force_out_limit())

        self.control_open_loop.actuator_steps[-1] += 1000
        self.assertTrue(self.control_open_loop.is_actuator_force_out_limit())

    def test_calculate_steps_to_forces(self):

        forces = self._get_forces_with_step_0_change(600)

        self.assertAlmostEqual(forces[0], 151.187008)
        self.assertAlmostEqual(forces[1], 189.097816)
        self.assertAlmostEqual(forces[2], 188.4147911)
        self.assertAlmostEqual(forces[3], 187.3399497)

    def _get_forces_with_step_0_change(self, step):

        steps = np.zeros(NUM_ACTUATOR)
        steps[0] = step

        return self.control_open_loop.calculate_steps_to_forces(steps)

    def test_calculate_forces_to_steps(self):

        forces = self._get_forces_with_step_0_change(600)
        steps = self.control_open_loop.calculate_forces_to_steps(forces)

        self.assertLess(np.abs(steps[0] - 600), 3)
        self.assertLess(np.max(np.abs(steps[1:])), 1)

    def test_calculate_forces_to_positions(self):

        forces = self._get_forces_with_step_0_change(100)

        positions = self.control_open_loop.calculate_forces_to_positions(forces)

        self.assertAlmostEqual(positions[0], 100 * self.control_open_loop.STEP_TO_MM)

    def test_get_actuator_positions(self):

        self.control_open_loop.actuator_steps[0] = 1

        actuator_positions = self.control_open_loop.get_actuator_positions()

        self.assertEqual(actuator_positions[0], self.control_open_loop.STEP_TO_MM)

    def test_start_exception(self):

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

    def test_start(self):

        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)

        self.assertTrue(self.control_open_loop.is_running)
        self.assertEqual(self.control_open_loop._displacement_steps, 2)

    def test_calculate_steps(self):

        self.assertEqual(
            self.control_open_loop._calculate_steps(
                self.control_open_loop.STEP_TO_MM, ActuatorDisplacementUnit.Millimeter
            ),
            1,
        )

    def test_stop(self):

        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)

        self.control_open_loop.stop()

        self.assertFalse(self.control_open_loop.is_running)
        self.assertEqual(self.control_open_loop._displacement_steps, 0)

    def test_pause(self):

        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)

        self.control_open_loop.pause()

        self.assertFalse(self.control_open_loop.is_running)

    def test_resume_exception(self):

        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)
        self.control_open_loop.pause()

        self.control_open_loop.resume()

        self.assertTrue(self.control_open_loop.is_running)

    def test_resume(self):

        self.assertRaises(RuntimeError, self.control_open_loop.resume)

    def test_run_steps_exception(self):

        self.assertRaises(RuntimeError, self.control_open_loop.run_steps, 1)

        self.control_open_loop.start([1], 2, ActuatorDisplacementUnit.Step)
        self.assertRaises(ValueError, self.control_open_loop.run_steps, -1)

    def test_run_steps_done(self):

        actuator_step_1 = self.control_open_loop.actuator_steps[1]
        actuator_step_2 = self.control_open_loop.actuator_steps[2]

        self.control_open_loop.start([1, 2], -2, ActuatorDisplacementUnit.Step)
        self.control_open_loop.run_steps(3)

        self.assertFalse(self.control_open_loop.is_running)

        self.assertEqual(self.control_open_loop.actuator_steps[1], actuator_step_1 - 2)
        self.assertEqual(self.control_open_loop.actuator_steps[2], actuator_step_2 - 2)

        self.assertEqual(self.control_open_loop._displacement_steps, 0)

    def test_run_steps_not_done(self):

        actuator_step_1 = self.control_open_loop.actuator_steps[1]
        actuator_step_2 = self.control_open_loop.actuator_steps[2]

        self.control_open_loop.start([1, 2], 4, ActuatorDisplacementUnit.Step)

        steps = 3
        self.control_open_loop.run_steps(steps)

        self.assertTrue(self.control_open_loop.is_running)

        self.assertEqual(
            self.control_open_loop.actuator_steps[1], actuator_step_1 + steps
        )
        self.assertEqual(
            self.control_open_loop.actuator_steps[2], actuator_step_2 + steps
        )

        self.assertEqual(self.control_open_loop._displacement_steps, 1)

    def test_move_actuator_steps_exception(self):

        self.assertRaises(
            ValueError, self.control_open_loop.move_actuator_steps, [1.0], [2]
        )

        self.assertRaises(
            ValueError, self.control_open_loop.move_actuator_steps, [1], [2.0]
        )

    def test_move_actuator_steps(self):

        self.control_open_loop.move_actuator_steps([5, 73], [1, 2])

        actuator_steps = self.control_open_loop.actuator_steps
        self.assertEqual(actuator_steps[5], 1)
        self.assertEqual(actuator_steps[73], 2)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
