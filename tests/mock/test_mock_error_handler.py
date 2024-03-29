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

from lsst.ts.m2com import LimitSwitchType, MockErrorHandler, get_config_dir


class TestMockErrorHandler(unittest.TestCase):
    """Test the Mock Error Handler class."""

    def setUp(self) -> None:
        self.error_handler = MockErrorHandler()
        self.error_handler.read_error_list_file(get_config_dir() / "error_code.tsv")

    def test_add_new_error(self) -> None:
        # Enabled fault
        self.error_handler.add_new_error(6051)
        self.assertEqual(len(self.error_handler._errors_new), 1)

        # Not enabled fault
        self.error_handler.add_new_error(1035)
        self.assertEqual(len(self.error_handler._errors_new), 1)

    def test_add_new_warning(self) -> None:
        # Enabled fault
        self.error_handler.add_new_warning(6059)
        self.assertEqual(len(self.error_handler._warnings_new), 1)

        # Not enabled fault
        self.error_handler.add_new_warning(1035)
        self.assertEqual(len(self.error_handler._warnings_new), 1)

    def test_clear(self) -> None:
        self.error_handler._errors_new.add(1)
        self.error_handler._errors_reported.add(2)

        self.error_handler._limit_switches_retract_new.add(3)
        self.error_handler._limit_switches_retract_reported.add(4)

        self.error_handler._limit_switches_extend_new.add(5)
        self.error_handler._limit_switches_extend_reported.add(6)

        self.error_handler.clear()

        self.assertEqual(len(self.error_handler._errors_new), 0)
        self.assertEqual(len(self.error_handler._errors_reported), 0)

        self.assertEqual(len(self.error_handler._limit_switches_retract_new), 0)
        self.assertEqual(len(self.error_handler._limit_switches_retract_reported), 0)

        self.assertEqual(len(self.error_handler._limit_switches_extend_new), 0)
        self.assertEqual(len(self.error_handler._limit_switches_extend_reported), 0)

    def test_add_new_limit_switch_exception(self) -> None:
        for actuator_id in (-1, 78, 79):
            for limit_switch_type in LimitSwitchType:
                self.assertRaises(
                    ValueError,
                    self.error_handler.add_new_limit_switch,
                    actuator_id,
                    limit_switch_type,
                )

    def test_add_new_limit_switch_retract(self) -> None:
        self._test_add_new_limit_switch(
            self.error_handler._limit_switches_retract_new,
            self.error_handler._limit_switches_retract_reported,
            LimitSwitchType.Retract,
        )

    def _test_add_new_limit_switch(
        self, set_new: set, set_reported: set, limit_switch_type: LimitSwitchType
    ) -> None:
        self.error_handler.add_new_limit_switch(1, limit_switch_type)
        self.assertEqual(len(set_new), 1)

        # Repeated limit switch should not be added
        self.error_handler.add_new_limit_switch(1, limit_switch_type)
        self.assertEqual(len(set_new), 1)

        # If the limit switch has been reported, it will not be added.
        set_reported.add(2)
        self.error_handler.add_new_limit_switch(2, limit_switch_type)
        self.assertEqual(len(set_new), 1)

    def test_add_new_limit_switch_extend(self) -> None:
        self._test_add_new_limit_switch(
            self.error_handler._limit_switches_extend_new,
            self.error_handler._limit_switches_extend_reported,
            LimitSwitchType.Extend,
        )

    def test_exists_new_limit_switch(self) -> None:
        for limit_switch_type in LimitSwitchType:
            self.assertFalse(
                self.error_handler.exists_new_limit_switch(limit_switch_type)
            )

            self.error_handler.add_new_limit_switch(1, limit_switch_type)
            self.assertTrue(
                self.error_handler.exists_new_limit_switch(limit_switch_type)
            )

    def test_exists_limit_switch(self) -> None:
        for limit_switch_type in LimitSwitchType:
            self.assertFalse(self.error_handler.exists_limit_switch(limit_switch_type))

            set_reported = (
                self.error_handler._limit_switches_retract_reported
                if limit_switch_type == LimitSwitchType.Retract
                else self.error_handler._limit_switches_extend_reported
            )
            self._test_exists_limit_switch(set_reported, limit_switch_type)

    def _test_exists_limit_switch(
        self, set_reported: set, limit_switch_type: LimitSwitchType
    ) -> None:
        set_reported.add(1)
        self.assertTrue(self.error_handler.exists_limit_switch(limit_switch_type))

    def test_get_limit_switches_to_report(self) -> None:
        for limit_switch_type in LimitSwitchType:
            self.error_handler.add_new_limit_switch(1, limit_switch_type)

            limit_switches = self.error_handler.get_limit_switches_to_report(
                limit_switch_type
            )

            self.assertEqual(limit_switches, {1})

        self.assertEqual(len(self.error_handler._limit_switches_retract_new), 0)
        self.assertEqual(len(self.error_handler._limit_switches_extend_new), 0)

        self.assertEqual(len(self.error_handler._limit_switches_retract_reported), 1)
        self.assertEqual(len(self.error_handler._limit_switches_extend_reported), 1)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
