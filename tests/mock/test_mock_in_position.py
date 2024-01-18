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
from lsst.ts.m2com import NUM_ACTUATOR, NUM_TANGENT_LINK, MockInPosition


class TestMockInPosition(unittest.TestCase):
    """Test the Mock InPosition class."""

    def setUp(self) -> None:
        self.in_position = MockInPosition(1, 2.0, 2.0, 3.0)

    def test_init(self) -> None:
        self.assertEqual(self.in_position._queue.maxlen, 2)
        self.assertEqual(self.in_position.threshold_squared_axial, 4.0)
        self.assertEqual(self.in_position.threshold_squared_tangent, 9.0)

    def test_is_in_position(self) -> None:
        # First time
        self.assertTrue(self._is_in_position(0.0, 0.0))
        self.assertEqual(len(self.in_position._queue), 1)

        # Second time
        self.assertTrue(self._is_in_position(1.0, 1.0))
        self.assertEqual(len(self.in_position._queue), 2)
        self.assertEqual(self.in_position._running_sum[0], 1.0)

        # Third time
        self.assertTrue(self._is_in_position(0.5, 1.0))
        self.assertEqual(len(self.in_position._queue), 2)
        self.assertEqual(self.in_position._running_sum[0], 1.25)

        # Fourth time
        self.assertTrue(self._is_in_position(1.3, 1.0))
        self.assertEqual(len(self.in_position._queue), 2)
        self.assertAlmostEqual(self.in_position._running_sum[0], 1.94)

        # Fifth time
        self.assertTrue(self._is_in_position(1.4, 1.0))
        self.assertEqual(len(self.in_position._queue), 2)
        self.assertAlmostEqual(self.in_position._running_sum[0], 3.65)

        # Sixth time
        self.assertTrue(self._is_in_position(0.1, 1.0))
        self.assertEqual(len(self.in_position._queue), 2)
        self.assertAlmostEqual(self.in_position._running_sum[0], 1.97)

        # Seventh time
        self.assertFalse(self._is_in_position(0.1, 10.0))
        self.assertEqual(len(self.in_position._queue), 2)
        self.assertAlmostEqual(self.in_position._running_sum[-1], 101.0)

    def _is_in_position(self, value_axial: float, value_tangent: float) -> bool:
        force_error = np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)
        force_error[0] = value_axial
        force_error[-1] = value_tangent

        return self.in_position.is_in_position(force_error)

    def test_reset(self) -> None:
        self._is_in_position(1.0, 1.0)

        self.in_position.reset()

        self.assertEqual(len(self.in_position._queue), 0)
        np.testing.assert_array_equal(
            self.in_position._running_sum, np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)
        )


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
