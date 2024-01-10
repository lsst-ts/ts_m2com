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

__all__ = ["SingleBiquadraticFilter", "BiquadraticFilter"]

import numpy as np
import numpy.typing


class SingleBiquadraticFilter:
    """Single biquadratic filter.

    Notes
    -----
    The equation is:

           1 + b11 z^-1 + b21 z^-2    Y(z)
    H(z) = ----------------------- = -----
           1 + a11 z^-1 + a21 z^-2.   X(z)

    This can be considered as the following process:

    g[n-2] = b21 * x[n] - a21 * y[n]
    g[n-1] = g[n-2] + b11 * x[n] - a11 * y[n]

    Therefore, we will have:

    y[n] = x[n] + g[n-1]

    Parameters
    ----------
    a11 : `float`
        The coefficient a11 in the equation.
    a21 : `float`
        The coefficient a21 in the equation.
    b11 : `float`
        The coefficient b11 in the equation.
    b21 : `float`
        The coefficient b21 in the equation.
    num_element : `int`
        Number of the elements.
    """

    def __init__(
        self, a11: float, a21: float, b11: float, b21: float, num_element: int
    ) -> None:
        self._a11 = a11
        self._a21 = a21
        self._b11 = b11
        self._b21 = b21

        self._g_m1 = np.zeros(num_element)
        self._g_m2 = np.zeros(num_element)

    def reset(self) -> None:
        """Reset the filter."""

        self._g_m1 = np.zeros(len(self._g_m1))
        self._g_m2 = np.zeros(len(self._g_m2))

    def filter(
        self, value: numpy.typing.NDArray[np.float64]
    ) -> numpy.typing.NDArray[np.float64]:
        """Filter the input value.

        Parameters
        ----------
        value : `numpy.ndarray`
            Input value.

        Returns
        -------
        value_filter : `numpy.ndarray`
            Filtered value.
        """

        # Filtered value
        value_filter = value + self._g_m1

        # Calculate and update the updated internal values
        g_m2 = self._b21 * value - self._a21 * value_filter
        g_m1 = self._g_m2 + self._b11 * value - self._a11 * value_filter

        self._g_m1 = g_m1
        self._g_m2 = g_m2

        return value_filter


class BiquadraticFilter:
    """Biquadratic filter.

    Notes
    -----
    Compact biquadratic format:

                   1 + b11 z^-1 + b21 z^-2   1 + b12 z^-1 + b22 z^-2
    H(z) = gain * (-----------------------) (-----------------------) ...
                   1 + a11 z^-1 + a21 z^-2   1 + a12 z^-1 + a22 z^-2

            1 + b1N z^-1 + b2N z^-2
           (-----------------------)
            1 + a1N z^-1 + a2N z^-2

    The reference is:
    https://en.wikipedia.org/wiki/Digital_biquad_filter

    Parameters
    ----------
    gain : `float`
        Gain.
    coefficients : `numpy.ndarray`
        Coefficients of the biquadratic filters. This should be a 2D array.
        Each row is the [a1n, a2n, b1n, b2n] for the nth biquadratic filter.
    num_element : `int`
        Number of the elements.
    """

    def __init__(
        self,
        gain: float,
        coefficients: numpy.typing.NDArray[np.float64],
        num_element: int,
    ) -> None:
        self._gain = gain

        self._bqd_filters = list()
        for coefficient in coefficients:
            bqd_filter = SingleBiquadraticFilter(
                coefficient[0],
                coefficient[1],
                coefficient[2],
                coefficient[3],
                num_element,
            )
            self._bqd_filters.append(bqd_filter)

    def reset(self) -> None:
        """Reset the filter."""

        for bqd_filter in self._bqd_filters:
            bqd_filter.reset()

    def filter(
        self, value: numpy.typing.NDArray[np.float64]
    ) -> numpy.typing.NDArray[np.float64]:
        """Filter the input value.

        Parameters
        ----------
        value : `numpy.ndarray`
            Input value.

        Returns
        -------
        `numpy.ndarray`
            Filtered value.
        """

        value_filter = value.copy()
        for bqd_filter in self._bqd_filters:
            value_filter = bqd_filter.filter(value_filter)

        return self._gain * value_filter
