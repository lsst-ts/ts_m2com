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

from lsst.ts.idl.enums import MTM2
from lsst.ts.m2com import MockInnerLoopController


class TestMockInnerLoopController(unittest.TestCase):
    """Test the Mock Inner-Loop Controller class."""

    def setUp(self) -> None:
        self.inner_loop_controller = MockInnerLoopController()

    def test_set_mode(self) -> None:
        self.inner_loop_controller.set_mode(MTM2.InnerLoopControlMode.Fault)
        self.assertEqual(
            self.inner_loop_controller.mode, MTM2.InnerLoopControlMode.Fault
        )

        self.inner_loop_controller.set_mode(MTM2.InnerLoopControlMode.ClearFaults)
        self.assertEqual(
            self.inner_loop_controller.mode, MTM2.InnerLoopControlMode.Standby
        )

        self.inner_loop_controller.set_mode(MTM2.InnerLoopControlMode.NoChange)
        self.assertEqual(
            self.inner_loop_controller.mode, MTM2.InnerLoopControlMode.Standby
        )


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
