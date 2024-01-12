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
from lsst.ts.m2com import MockDeadbandControl


class TestMockDeadbandControl(unittest.TestCase):
    """Test the Mock Deadband Control class."""

    def setUp(self) -> None:
        self.deadband_control = MockDeadbandControl(1.0, 2.0)

    def test_select_disabled(self) -> None:
        hardpoint_error = np.array([0.1, 0.2, 0.0])

        hardpoint_error_selected = self.deadband_control.select(
            hardpoint_error, is_enabled=False
        )

        np.testing.assert_equal(hardpoint_error_selected, hardpoint_error)
        self.assertNotEqual(id(hardpoint_error_selected), id(hardpoint_error))
        self.assertFalse(self.deadband_control._keep_tracking)

    def test_select_enabled(self) -> None:
        # Latch the hardpoint error
        hardpoint_error_1 = np.array([0.1, 0.2, 0.0])

        hardpoint_error_selected_1 = self.deadband_control.select(
            hardpoint_error_1, is_enabled=True
        )

        np.testing.assert_equal(hardpoint_error_selected_1, hardpoint_error_1)
        self.assertTrue(self.deadband_control._keep_tracking)

        # Use the latched hardpoint error
        hardpoint_error_2 = np.array([0.3, 0.4, 0.0])

        hardpoint_error_selected_2 = self.deadband_control.select(
            hardpoint_error_2, is_enabled=True
        )

        np.testing.assert_equal(hardpoint_error_selected_2, hardpoint_error_selected_1)
        self.assertTrue(self.deadband_control._keep_tracking)

        # Still use the latched hardpoint error because the maximum value is
        # still lower than the upper limit
        hardpoint_error_3 = np.array([-1.3, 0.4, 0.0])

        hardpoint_error_selected_3 = self.deadband_control.select(
            hardpoint_error_3, is_enabled=True
        )

        np.testing.assert_equal(hardpoint_error_selected_3, hardpoint_error_selected_1)
        self.assertTrue(self.deadband_control._keep_tracking)

        # The latch is broken because the maximum error is higher than the
        # upper threshold
        hardpoint_error_4 = np.array([-2.1, 0.4, 0.0])

        hardpoint_error_selected_4 = self.deadband_control.select(
            hardpoint_error_4, is_enabled=True
        )

        np.testing.assert_equal(hardpoint_error_selected_4, hardpoint_error_4)
        self.assertFalse(self.deadband_control._keep_tracking)

        # After the latch is broken, we need to wait until the maximum error
        # is lower than the lower threshold
        hardpoint_error_5 = np.array([-1.3, 0.4, 0.0])

        hardpoint_error_selected_5 = self.deadband_control.select(
            hardpoint_error_5, is_enabled=True
        )

        np.testing.assert_equal(hardpoint_error_selected_5, hardpoint_error_5)
        self.assertFalse(self.deadband_control._keep_tracking)

        # Latch the hardpoint error again
        hardpoint_error_6 = np.array([-0.3, -0.4, 0.0])

        hardpoint_error_selected_6 = self.deadband_control.select(
            hardpoint_error_6, is_enabled=True
        )

        np.testing.assert_equal(hardpoint_error_selected_6, hardpoint_error_6)
        self.assertTrue(self.deadband_control._keep_tracking)

    def test_select_zero_threshold(self) -> None:
        deadband_control = MockDeadbandControl(0.0, 0.0)

        # Disabled
        hardpoint_error_1 = np.array([0.1, 0.2, 0.0])
        hardpoint_error_selected_1 = deadband_control.select(
            hardpoint_error_1, is_enabled=False
        )

        np.testing.assert_equal(hardpoint_error_selected_1, hardpoint_error_1)
        self.assertFalse(deadband_control._keep_tracking)

        # Enabled
        hardpoint_error_2 = np.array([0.2, 0.3, 0.0])
        hardpoint_error_selected_2 = deadband_control.select(
            hardpoint_error_2, is_enabled=True
        )

        np.testing.assert_equal(hardpoint_error_selected_2, hardpoint_error_2)
        self.assertFalse(deadband_control._keep_tracking)

    def test_reset(self) -> None:
        hardpoint_error = np.array([0.1, 0.2, 0.0])
        self.deadband_control.select(hardpoint_error, is_enabled=True)

        # Do not reset all
        self.deadband_control.reset(reset_all=False)

        self.assertFalse(self.deadband_control._keep_tracking)
        np.testing.assert_equal(
            self.deadband_control._hardpoint_error_track, hardpoint_error
        )

        # Reset all
        self.deadband_control.reset(reset_all=True)

        np.testing.assert_equal(
            self.deadband_control._hardpoint_error_track, np.zeros(3)
        )


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
