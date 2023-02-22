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

import asyncio

import numpy as np

from ..enum import PowerSystemState

__all__ = ["MockPowerSystem"]


class MockPowerSystem:
    """Mock power system to simulate the behavior of hardware.

    Parameters
    ----------
    default_voltage : `float`
        Default voltage in volt when the power is on.
    default_current : `float`
        Default current in ampere when the power is on.

    Attributes
    ----------
    state : enum `PowerSystemState`
        State of the power system.
    """

    # Sleep times in second
    SLEEP_TIME_SHORT = 0.5
    SLEEP_TIME_LONG = 3

    def __init__(self, default_voltage, default_current):
        # Power is on or not
        self._is_power_on = False

        self.state = PowerSystemState.Init

        # Default voltage and current when the power is on
        self._default_voltage = default_voltage
        self._default_current = default_current

    async def power_on(self):
        """Power on."""

        self._is_power_on = True

        await asyncio.sleep(self.SLEEP_TIME_SHORT)
        self.state = PowerSystemState.PoweringOn

    async def wait_power_fully_on(self):
        """Wait the power to be fully on."""

        if self.state == PowerSystemState.PoweringOn:
            await asyncio.sleep(self.SLEEP_TIME_LONG)
            self.state = PowerSystemState.PoweredOn

    async def power_off(self):
        """Power off."""

        self._is_power_on = False

        await asyncio.sleep(self.SLEEP_TIME_SHORT)
        self.state = PowerSystemState.PoweringOff

    async def wait_power_fully_off(self):
        """Wait the power to be fully off."""

        if self.state == PowerSystemState.PoweringOff:
            await asyncio.sleep(self.SLEEP_TIME_SHORT)
            self.state = PowerSystemState.PoweredOff

    def is_power_on(self):
        """Power is on or not.

        Returns
        -------
        `bool`
            True if the power is on. Otherwise, False.
        """
        return self._is_power_on and self.state in (
            PowerSystemState.PoweringOn,
            PowerSystemState.ResettingBreakers,
            PowerSystemState.PoweredOn,
        )

    def get_power(self, rms=0.05):
        """Get the power.

        Parameters
        ----------
        rms : `float`, optional
            RMS variation. (the default is 0.05)

        Returns
        -------
        `float`
            Voltage in volt.
        `float`
            Current in ampere.
        """

        rms_power = np.random.normal(scale=rms, size=2)

        if self.is_power_on():
            rms_power = np.random.normal(scale=rms, size=2)
            return (
                self._default_voltage + rms_power[0],
                self._default_current + rms_power[1],
            )
        else:
            return rms_power[0], rms_power[1]
