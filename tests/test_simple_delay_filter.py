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
from lsst.ts.m2com import SimpleDelayFilter


class TestSimpleDelayFilter(unittest.TestCase):
    """Test the Simple Delay Filter class."""

    def setUp(self) -> None:
        self.simple_delay_filter = SimpleDelayFilter([0.1, 0.2], 2)

    def test_filter(self) -> None:
        # First time
        value = np.array([1.0, 3.0])
        value_filter_1 = self.simple_delay_filter.filter(value)

        np.testing.assert_almost_equal(value_filter_1, np.array([0.1, 0.3]))
        np.testing.assert_almost_equal(
            self.simple_delay_filter._delay_history, np.array([[1.0, 3.0], [0.0, 0.0]])
        )

        # Second time
        value_filter_2 = self.simple_delay_filter.filter(value_filter_1)

        np.testing.assert_almost_equal(value_filter_2, np.array([0.21, 0.63]))
        np.testing.assert_almost_equal(
            self.simple_delay_filter._delay_history, np.array([[0.1, 0.3], [1.0, 3.0]])
        )

        # Third time
        value_filter_3 = self.simple_delay_filter.filter(value_filter_2)

        np.testing.assert_almost_equal(value_filter_3, np.array([0.041, 0.123]))
        np.testing.assert_almost_equal(
            self.simple_delay_filter._delay_history,
            np.array([[0.21, 0.63], [0.1, 0.3]]),
        )

    def test_reset(self) -> None:
        self.simple_delay_filter.filter(np.array([0.1, 0.3]))

        self.simple_delay_filter.reset()

        np.testing.assert_equal(
            self.simple_delay_filter._delay_history, np.zeros((2, 2))
        )


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
