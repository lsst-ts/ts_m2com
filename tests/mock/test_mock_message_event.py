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

from lsst.ts.m2com import MockMessageEvent


class TestMockMessageEvent(unittest.TestCase):
    """Test the Mock Message Event class."""

    def setUp(self) -> None:
        self.message_event = MockMessageEvent(None)

    def test_get_configuration_file_details(self) -> None:
        # Surrogate
        self.message_event.configuration_file = (
            "Configurable_File_Description_20180831T092556_surrogate_handling.csv"
        )

        (
            version,
            control_parameters,
            lut_parameters,
        ) = self.message_event._get_configuration_file_details()

        self.assertEqual(version, "20180831T092556")
        self.assertEqual(
            control_parameters, "CtrlParameterFiles_2018-07-19_104314_surg"
        )
        self.assertEqual(lut_parameters, "FinalHandlingLUTs")

        # M2
        self.message_event.configuration_file = (
            "Configurable_File_Description_20180831T091922_M2_optical.csv"
        )

        (
            version,
            control_parameters,
            lut_parameters,
        ) = self.message_event._get_configuration_file_details()

        self.assertEqual(version, "20180831T091922")
        self.assertEqual(control_parameters, "CtrlParameterFiles_2018-07-19_104257_m2")
        self.assertEqual(lut_parameters, "FinalOpticalLUTs")


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
