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

from lsst.ts.m2com import MockPowerSystem
from lsst.ts.xml.enums import MTM2


class TestMockPowerSystem(unittest.IsolatedAsyncioTestCase):
    """Test the Mock Power System class."""

    def setUp(self) -> None:
        self.power_system = MockPowerSystem(1, 2)

    def test_init(self) -> None:
        self.assertEqual(self.power_system.state, MTM2.PowerSystemState.Init)

    async def test_power_on(self) -> None:
        await self.power_system.power_on()

        self.assertTrue(self.power_system._is_power_on)
        self.assertEqual(self.power_system.state, MTM2.PowerSystemState.PoweringOn)

    async def test_wait_power_fully_on(self) -> None:
        # No update
        await self.power_system.wait_power_fully_on()
        self.assertEqual(self.power_system.state, MTM2.PowerSystemState.Init)

        # State is updated
        await self.power_system.power_on()
        await self.power_system.wait_power_fully_on()

        self.assertEqual(self.power_system.state, MTM2.PowerSystemState.PoweredOn)

    async def test_power_off(self) -> None:
        await self.power_system.power_on()
        await self.power_system.power_off()

        self.assertFalse(self.power_system._is_power_on)
        self.assertEqual(self.power_system.state, MTM2.PowerSystemState.PoweringOff)

    async def test_wait_power_fully_off(self) -> None:
        # No update
        await self.power_system.wait_power_fully_off()
        self.assertEqual(self.power_system.state, MTM2.PowerSystemState.Init)

        # State is updated
        await self.power_system.power_on()
        await self.power_system.power_off()
        await self.power_system.wait_power_fully_off()

        self.assertEqual(self.power_system.state, MTM2.PowerSystemState.PoweredOff)

    async def test_is_power_on(self) -> None:
        self.assertFalse(self.power_system.is_power_on())

        await self.power_system.power_on()
        self.assertTrue(self.power_system.is_power_on())

    async def test_get_power(self) -> None:
        self.assertEqual(self.power_system.get_power(rms=0), (0, 0))

        await self.power_system.power_on()
        self.assertEqual(self.power_system.get_power(rms=0), (1, 2))


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
