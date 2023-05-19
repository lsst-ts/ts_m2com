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

from lsst.ts.m2com import ErrorHandler, get_config_dir


class TestErrorHandler(unittest.TestCase):
    """Test the Error Handler class."""

    def setUp(self) -> None:
        self.error_handler = ErrorHandler()
        self.error_handler.read_error_list_file(get_config_dir() / "error_code.tsv")

    def test_init(self) -> None:
        self.assertEqual(len(self.error_handler.list_code_total), 64)
        self.assertEqual(len(self.error_handler._list_code_error), 25)
        self.assertEqual(len(self.error_handler._list_code_warning), 11)

    def test_clear(self) -> None:
        self.error_handler._errors_new.add(1)
        self.error_handler._errors_reported.add(2)

        self.error_handler._warnings_new.add(3)
        self.error_handler._warnings_reported.add(4)

        self.error_handler.clear()

        self.assertEqual(len(self.error_handler._errors_new), 0)
        self.assertEqual(len(self.error_handler._errors_reported), 0)

        self.assertEqual(len(self.error_handler._warnings_new), 0)
        self.assertEqual(len(self.error_handler._warnings_reported), 0)

    def test_decode_summary_faults_status(self) -> None:
        # No error/warning
        self.error_handler.decode_summary_faults_status(0)
        self.assertEqual(len(self.error_handler.get_errors_to_report()), 0)
        self.assertEqual(len(self.error_handler.get_warnings_to_report()), 0)

        # Some error and warning
        self.error_handler.decode_summary_faults_status(60)
        self.assertEqual(len(self.error_handler.get_errors_to_report()), 2)
        self.assertEqual(len(self.error_handler.get_warnings_to_report()), 1)

        # Some repetition
        self.error_handler.decode_summary_faults_status(124)
        self.assertEqual(len(self.error_handler.get_errors_to_report()), 1)
        self.assertEqual(len(self.error_handler.get_warnings_to_report()), 0)

        # Check the maximum value
        self.error_handler.decode_summary_faults_status(2**61 + 2**63)
        self.assertEqual(self.error_handler.get_errors_to_report(), {6073, 6087})

    def test_is_error(self) -> None:
        self.assertFalse(self.error_handler.is_error(1001))
        self.assertTrue(self.error_handler.is_error(6051))

    def test_is_warning(self) -> None:
        self.assertFalse(self.error_handler.is_warning(1001))
        self.assertTrue(self.error_handler.is_warning(6057))

    def test_add_new_error(self) -> None:
        self.error_handler.add_new_error(1)
        self.assertEqual(len(self.error_handler._errors_new), 1)

        # Repeated error should not be added
        self.error_handler.add_new_error(1)
        self.assertEqual(len(self.error_handler._errors_new), 1)

        # If the error has been reported, it will not be added.
        self.error_handler._errors_reported.add(2)
        self.error_handler.add_new_error(2)
        self.assertEqual(len(self.error_handler._errors_new), 1)

    def test_add_new_warning(self) -> None:
        self.error_handler.add_new_warning(1)
        self.assertEqual(len(self.error_handler._warnings_new), 1)

        # Repeated warning should not be added
        self.error_handler.add_new_warning(1)
        self.assertEqual(len(self.error_handler._warnings_new), 1)

        # If the error has been reported, it will not be added.
        self.error_handler._warnings_reported.add(2)
        self.error_handler.add_new_warning(2)
        self.assertEqual(len(self.error_handler._warnings_new), 1)

    def test_exists_new_error(self) -> None:
        self.assertFalse(self.error_handler.exists_new_error())

        self.error_handler.add_new_error(1)
        self.assertTrue(self.error_handler.exists_new_error())

    def test_exists_new_warning(self) -> None:
        self.assertFalse(self.error_handler.exists_new_warning())

        self.error_handler.add_new_warning(1)
        self.assertTrue(self.error_handler.exists_new_warning())

    def test_exists_error(self) -> None:
        self.assertFalse(self.error_handler.exists_error())

        self.error_handler._errors_reported.add(1)
        self.assertTrue(self.error_handler.exists_error())

    def test_exists_warning(self) -> None:
        self.assertFalse(self.error_handler.exists_warning())

        self.error_handler._warnings_reported.add(1)
        self.assertTrue(self.error_handler.exists_warning())

    def test_get_errors_to_report(self) -> None:
        self.error_handler.add_new_error(1)

        errors = self.error_handler.get_errors_to_report()

        self.assertEqual(errors, {1})
        self.assertEqual(len(self.error_handler._errors_new), 0)
        self.assertEqual(len(self.error_handler._errors_reported), 1)

    def test_get_warnings_to_report(self) -> None:
        self.error_handler.add_new_warning(1)

        warnings = self.error_handler.get_warnings_to_report()

        self.assertEqual(warnings, {1})
        self.assertEqual(len(self.error_handler._warnings_new), 0)
        self.assertEqual(len(self.error_handler._warnings_reported), 1)

    def test_has_error(self) -> None:
        self.assertFalse(self.error_handler.has_error(1))

        self.error_handler.add_new_error(1)
        self.assertTrue(self.error_handler.has_error(1))

    def test_has_warning(self) -> None:
        self.assertFalse(self.error_handler.has_warning(1))

        self.error_handler.add_new_warning(1)
        self.assertTrue(self.error_handler.has_warning(1))

    def test_get_bit_from_code(self) -> None:
        self.assertEqual(self.error_handler.get_bit_from_code(6051), 3)

    def test_get_summary_faults_status_from_codes(self) -> None:
        # No code
        self.assertEqual(self.error_handler.get_summary_faults_status_from_codes([]), 0)

        # Some codes
        self.assertEqual(
            self.error_handler.get_summary_faults_status_from_codes([6051, 6055]), 24
        )

    def test_get_summary_faults_status_to_report(self) -> None:
        # No faults
        self.assertEqual(self.error_handler.get_summary_faults_status_to_report(), 0)

        # Some faults
        self.error_handler._errors_reported.add(6051)
        self.error_handler._errors_new.add(6055)
        self.error_handler._warnings_reported.add(6057)
        self.error_handler._warnings_new.add(6059)

        self.assertEqual(
            self.error_handler.get_summary_faults_status_to_report(), 0x138
        )
        self.assertEqual(self.error_handler._errors_reported, {6051, 6055})
        self.assertEqual(self.error_handler._warnings_reported, {6057, 6059})


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
