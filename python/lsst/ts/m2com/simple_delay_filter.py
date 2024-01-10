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

__all__ = ["SimpleDelayFilter"]

import numpy as np
import numpy.typing


class SimpleDelayFilter:
    """Simple delay filter

    Notes
    ------
    This is a simplified implementation that considers the denominator of
    transfer function is 1:

    H(z) = b0 + b1 * z ^ (-1) + b2 * z ^ (-2) + ... + bN * z ^ (-N)
         = Y(z) / X(z)

    => y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + ... + bN * x[n-N]

    Parameters
    ----------
    coefficients : `list`
        Filter coefficients: [b0, b1, b2, ..., bN].
    num_element : `int`
        Number of the elements.
    """

    def __init__(self, coefficients: list[float], num_element: int) -> None:
        self._coeffs = coefficients
        self._delay_history = np.zeros((len(coefficients), num_element))

    def reset(self) -> None:
        """Reset the filter."""

        self._delay_history = np.zeros(self._delay_history.shape)

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
        value_filter = self._coeffs[0] * value
        for idx in range(1, len(self._coeffs)):
            value_filter += self._coeffs[idx] * self._delay_history[idx - 1, :]

        # Update the history
        delay_history_updated = np.roll(self._delay_history, 1, axis=0)
        delay_history_updated[0, :] = value
        self._delay_history = delay_history_updated

        return value_filter
