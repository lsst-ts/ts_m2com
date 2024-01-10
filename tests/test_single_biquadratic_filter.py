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
from lsst.ts.m2com import SingleBiquadraticFilter


class SingleBiquadraticFilterVerification:
    def __init__(
        self, a11: float, a21: float, b11: float, b21: float, num_element: int
    ) -> None:
        self._a11 = a11
        self._a21 = a21
        self._b11 = b11
        self._b21 = b21

        self._x_m1 = np.zeros(num_element)
        self._x_m2 = np.zeros(num_element)

        self._y_m1 = np.zeros(num_element)
        self._y_m2 = np.zeros(num_element)

    def filter(
        self, value: numpy.typing.NDArray[np.float64]
    ) -> numpy.typing.NDArray[np.float64]:
        # Filtered value
        y = (
            value
            + self._b11 * self._x_m1
            + self._b21 * self._x_m2
            - self._a11 * self._y_m1
            - self._a21 * self._y_m2
        )

        # Update the internal data
        self._x_m2 = self._x_m1.copy()
        self._x_m1 = value.copy()

        self._y_m2 = self._y_m1.copy()
        self._y_m1 = y.copy()

        return y


class TestSingleBiquadraticFilter(unittest.TestCase):
    """Test the Single Biquadratic Filter class."""

    def setUp(self) -> None:
        self.bqd_filter = SingleBiquadraticFilter(2.0, 3.0, 4.0, 5.0, 2)
        self.bqd_filter_verification = SingleBiquadraticFilterVerification(
            2.0, 3.0, 4.0, 5.0, 2
        )

    def test_filter(self) -> None:
        value_bqd = np.array([0.1, 0.3])
        value_verification = np.array([0.1, 0.3])
        for idx in range(20):
            value_bqd = self.bqd_filter.filter(value_bqd)
            value_verification = self.bqd_filter_verification.filter(value_verification)

            self.assertAlmostEqual(np.sum(np.abs(value_bqd - value_verification)), 0.0)

    def test_reset(self) -> None:
        self.bqd_filter.filter(np.array([0.1, 0.3]))

        self.bqd_filter.reset()

        np.testing.assert_equal(self.bqd_filter._g_m1, np.zeros(2))
        np.testing.assert_equal(self.bqd_filter._g_m2, np.zeros(2))


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
