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
from lsst.ts.m2com import NUM_ACTUATOR, MockPlant, get_config_dir, read_yaml_file


class TestMockPlant(unittest.TestCase):
    """Test the Mock Plant model class."""

    def setUp(self) -> None:
        filepath = get_config_dir() / "harrisLUT" / "stiff_matrix_m2.yaml"
        stiffness_matrix = read_yaml_file(filepath)
        self.plant = MockPlant(np.array(stiffness_matrix["stiff"]), 0.0)

    def test_update_actuator_force_weight(self) -> None:
        self.plant.update_actuator_force_weight(89.06)
        self.assertAlmostEqual(self.plant._actuator_force_weight[0], 216.2329167)

    def test_get_actuator_forces(self) -> None:
        self.plant.update_actuator_force_weight(89.06)

        self.plant._actuator_force_step[0] = 2.1
        forces = self.plant.get_actuator_forces()

        self.assertAlmostEqual(forces[0], 218.3329167)
        self.assertAlmostEqual(forces[1], 216.2329167)

    def test_get_actuator_positions(self) -> None:
        self.plant.actuator_steps[0] = 1

        actuator_positions = self.plant.get_actuator_positions()

        self.assertEqual(actuator_positions[0], self.plant.STEP_TO_MM)

    def test_move_steps_exception(self) -> None:
        self.assertRaises(ValueError, self.plant.move_actuator_steps, np.zeros(3))

        self.assertRaises(
            ValueError,
            self.plant.move_actuator_steps,
            np.zeros(NUM_ACTUATOR),
        )

    def test_move_actuator_steps(self) -> None:
        # Move
        self._move_actuator_steps()

        self.assertEqual(self.plant.actuator_steps[0], 1)
        self.assertAlmostEqual(self.plant._actuator_force_step[0], -0.0571288)

        # Move again
        self._move_actuator_steps()

        self.assertEqual(self.plant.actuator_steps[0], 2)
        self.assertAlmostEqual(self.plant._actuator_force_step[0], -0.1142577)

    def _move_actuator_steps(self) -> None:
        actuator_steps = np.zeros(NUM_ACTUATOR, dtype=int)
        actuator_steps[0] = 1
        self.plant.move_actuator_steps(actuator_steps)

    def test_reset_actuator_steps(self) -> None:
        self._move_actuator_steps()

        self.plant.reset_actuator_steps()

        self.assertEqual(self.plant.actuator_steps[0], 0)
        self.assertEqual(self.plant._actuator_force_step[0], 0.0)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
