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

__all__ = ["MockGainSchedular"]


class MockGainSchedular:
    """Mock gain schedular to tune the gains for the slewing, settling, and
    imaging processes.

    Notes
    -----
    The maximum gain is 1.0.

    Parameters
    ----------
    min_gain_axial : `float`
        Minimum gain for the axial actuators. The value should be in
        (0.0, 1.0).
    min_gain_tangent : `float`
        Minimum gain for the tangent actuators. The value should be in
        (0.0, 1.0).
    num_sample_ramp_up : `int`
        Number of samples in the ramping up process. This value can not be 0.
    num_sample_ramp_down : `int`
        Number of samples in the ramping down process. This value can not be 0.
    max_sample_settle : `int`
        Maximum number of samples in the settling process. This value can not
        be 0.
    """

    MAX_GAIN = 1.0

    def __init__(
        self,
        min_gain_axial: float,
        min_gain_tangent: float,
        num_sample_ramp_up: int,
        num_sample_ramp_down: int,
        max_sample_settle: int,
    ):
        # Rates of the ramping up process
        self._rate_up_axial = (self.MAX_GAIN - min_gain_axial) / num_sample_ramp_up
        self._rate_up_tangent = (self.MAX_GAIN - min_gain_tangent) / num_sample_ramp_up

        # Rates of the ramping down process
        self._rate_down_axial = (min_gain_axial - self.MAX_GAIN) / num_sample_ramp_down
        self._rate_down_tangent = (
            min_gain_tangent - self.MAX_GAIN
        ) / num_sample_ramp_down

        self._gain_axial = min_gain_axial
        self._gain_tangent = min_gain_tangent

        self._min_gain_axial = min_gain_axial
        self._min_gain_tangent = min_gain_tangent

        self._max_sample_settle = max_sample_settle
        self._sample_settle = max_sample_settle

    def reset(self) -> None:
        """Reset the internal data."""

        self._gain_axial = self._min_gain_axial
        self._gain_tangent = self._min_gain_tangent

        self._sample_settle = self._max_sample_settle

    def get_gain(self, is_in_position: bool) -> tuple[float, float]:
        """Get the gain values.

        Parameters
        ----------
        is_in_position : `bool`
            Mirror is in position or not.

        Returns
        -------
        `float`
            Gain of the axial actuators.
        `float`
            Gain of the tangent actuators.
        """

        # Return the current gains in the settling process at the settling
        # period
        if is_in_position and (self._sample_settle > 0):
            self._sample_settle -= 1
            return self._gain_axial, self._gain_tangent

        # Ramp down the gains after the settling process.
        # We want to ramp down the gains to the minimum to filter out the high
        # frequency noise in the imaging process.
        # If the mirror is not in position, we will ramp up the gains to the
        # maximum in the slewing process.
        is_ramp_down = is_in_position and (self._sample_settle == 0)

        # Because we will begin to ramp up the gains, reset the settling period
        if not is_in_position:
            self._sample_settle = self._max_sample_settle

        # Calculate the new gains
        rate_axial = self._rate_down_axial if is_ramp_down else self._rate_up_axial
        rate_tangent = (
            self._rate_down_tangent if is_ramp_down else self._rate_up_tangent
        )

        self._gain_axial = self._calc_next_gain(
            self._gain_axial, rate_axial, self._min_gain_axial
        )
        self._gain_tangent = self._calc_next_gain(
            self._gain_tangent, rate_tangent, self._min_gain_tangent
        )

        return self._gain_axial, self._gain_tangent

    def _calc_next_gain(
        self, gain_current: float, rate: float, min_gain: float
    ) -> float:
        """Calculate the next gain. The output will be in [min, max].

        Parameters
        ----------
        gain_current : `float`
            Current gain.
        rate : `float`
            Rate of gain.
        min_gain : `float`
            Minimum gain.

        Returns
        -------
        `float`
            Gain.
        """

        gain_new = gain_current + rate

        if gain_new >= self.MAX_GAIN:
            return self.MAX_GAIN

        elif gain_new <= min_gain:
            return min_gain

        else:
            return gain_new
