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
from lsst.ts.idl.enums import MTM2

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
    state : enum `MTM2.PowerSystemState`
        State of the power system.
    """

    # Sleep times in second
    SLEEP_TIME_SHORT = 0.5
    SLEEP_TIME_LONG = 3

    def __init__(self, default_voltage: float, default_current: float) -> None:
        # Power is on or not
        self._is_power_on = False

        self.state = MTM2.PowerSystemState.Init

        # Default voltage and current when the power is on
        self._default_voltage = default_voltage
        self._default_current = default_current

    async def power_on(self) -> None:
        """Power on."""

        self._is_power_on = True

        await asyncio.sleep(self.SLEEP_TIME_SHORT)
        self.state = MTM2.PowerSystemState.PoweringOn

    async def wait_power_fully_on(self) -> None:
        """Wait the power to be fully on."""

        if self.state == MTM2.PowerSystemState.PoweringOn:
            await asyncio.sleep(self.SLEEP_TIME_LONG)
            self.state = MTM2.PowerSystemState.PoweredOn

    async def power_off(self) -> None:
        """Power off."""

        self._is_power_on = False

        await asyncio.sleep(self.SLEEP_TIME_SHORT)
        self.state = MTM2.PowerSystemState.PoweringOff

    async def wait_power_fully_off(self) -> None:
        """Wait the power to be fully off."""

        if self.state == MTM2.PowerSystemState.PoweringOff:
            await asyncio.sleep(self.SLEEP_TIME_SHORT)
            self.state = MTM2.PowerSystemState.PoweredOff

    def is_power_on(self) -> bool:
        """Power is on or not.

        Returns
        -------
        `bool`
            True if the power is on. Otherwise, False.
        """
        return self._is_power_on and self.state in (
            MTM2.PowerSystemState.PoweringOn,
            MTM2.PowerSystemState.ResettingBreakers,
            MTM2.PowerSystemState.PoweredOn,
        )

    def get_power(self, rms: float = 0.05) -> tuple[float, float]:
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
