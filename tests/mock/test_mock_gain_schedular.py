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

from lsst.ts.m2com import MockGainSchedular


class TestMockGainSchedular(unittest.TestCase):
    """Test the Mock Gain Schedular class."""

    def setUp(self) -> None:
        self.min_gain_axial = 0.1
        self.min_gain_tangent = 0.2
        self.num_sample_ramp_up = 5
        self.num_sample_ramp_down = 10
        self.max_sample_settle = 5

        self.schedular = MockGainSchedular(
            self.min_gain_axial,
            self.min_gain_tangent,
            self.num_sample_ramp_up,
            self.num_sample_ramp_down,
            self.max_sample_settle,
        )

    def test_calc_next_gain(self) -> None:
        gain_normal = self.schedular._calc_next_gain(0.1, 0.2, 0.05)
        self.assertAlmostEqual(gain_normal, 0.3)

        gain_max = self.schedular._calc_next_gain(0.1, 1.2, 0.05)
        self.assertEqual(gain_max, 1.0)

        gain_min = self.schedular._calc_next_gain(0.1, -1.2, 0.05)
        self.assertEqual(gain_min, 0.05)

    def test_reset(self) -> None:
        # In position
        self.schedular.get_gain(True)

        self.schedular.reset()

        self.assertEqual(self.schedular._sample_settle, self.max_sample_settle)

        # Not in position
        for idx in range(2):
            self.schedular.get_gain(False)

        self.schedular.reset()

        self.assertEqual(self.schedular._gain_axial, self.min_gain_axial)
        self.assertEqual(self.schedular._gain_tangent, self.min_gain_tangent)

    def test_get_gain(self) -> None:
        # In position in the initial beginning
        self.assertEqual(self.schedular._sample_settle, 5)

        for idx in range(self.max_sample_settle + 1):
            gain_axial_0, gain_tangent_0 = self.schedular.get_gain(True)

            self.assertEqual(gain_axial_0, self.min_gain_axial)
            self.assertEqual(gain_tangent_0, self.min_gain_tangent)

        self.assertEqual(self.schedular._sample_settle, 0)

        # Mirror is not in the position, ramp up the gains to the maximum
        for idx in range(self.num_sample_ramp_up):
            gain_axial_up, gain_tangent_up = self.schedular.get_gain(False)

        self.assertEqual(gain_axial_up, 1.0)
        self.assertEqual(gain_tangent_up, 1.0)

        self.assertEqual(self.schedular._sample_settle, self.max_sample_settle)

        # Settling process
        for idx in range(self.max_sample_settle):
            gain_axial_settle, gain_tangent_settle = self.schedular.get_gain(True)

        self.assertEqual(gain_axial_settle, 1.0)
        self.assertEqual(gain_tangent_settle, 1.0)

        self.assertEqual(self.schedular._sample_settle, 0)

        # Ramp down to the minimum
        for idx in range(self.num_sample_ramp_down):
            gain_axial_down, gain_tangent_down = self.schedular.get_gain(True)

        self.assertAlmostEqual(gain_axial_down, self.min_gain_axial)
        self.assertAlmostEqual(gain_tangent_down, self.min_gain_tangent)

        self.assertEqual(self.schedular._sample_settle, 0)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
