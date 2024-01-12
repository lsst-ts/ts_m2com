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
from lsst.ts.m2com import BiquadraticFilter


class TestBiquadraticFilter(unittest.TestCase):
    """Test the Biquadratic Filter class."""

    def setUp(self) -> None:
        self.gain = 3.0

        coefficients = np.array([[2.0, 3.0, 4.0, 5.0], [2.0, 3.0, 4.0, 5.0]])
        self.bqd_filter = BiquadraticFilter(self.gain, coefficients, 2)

    def test_filter(self) -> None:
        # First time
        value = np.array([0.1, 0.3])
        value_filter_1 = self.bqd_filter.filter(value)

        np.testing.assert_almost_equal(value_filter_1, self.gain * value)

        np.testing.assert_almost_equal(
            self.bqd_filter._bqd_filters[1]._g_m1, np.array([0.2, 0.6])
        )
        np.testing.assert_almost_equal(
            self.bqd_filter._bqd_filters[1]._g_m2, np.array([0.2, 0.6])
        )

        # Second time
        value_filter_2 = self.bqd_filter.filter(value_filter_1 / self.gain)

        np.testing.assert_almost_equal(value_filter_2, np.array([1.5, 4.5]))

        np.testing.assert_almost_equal(
            self.bqd_filter._bqd_filters[0]._g_m1, np.array([0.0, 0.0])
        )
        np.testing.assert_almost_equal(
            self.bqd_filter._bqd_filters[0]._g_m2, np.array([-0.4, -1.2])
        )

    def test_reset(self) -> None:
        self.bqd_filter.filter(np.array([0.1, 0.3]))

        self.bqd_filter.reset()

        np.testing.assert_equal(self.bqd_filter._bqd_filters[0]._g_m1, np.zeros(2))
        np.testing.assert_equal(self.bqd_filter._bqd_filters[0]._g_m2, np.zeros(2))


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
