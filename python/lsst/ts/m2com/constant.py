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

__all__ = [
    "TEST_DIGITAL_OUTPUT_NO_POWER",
    "TEST_DIGITAL_OUTPUT_POWER_COMM",
    "TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR",
    "TEST_DIGITAL_INPUT_NO_POWER",
    "TEST_DIGITAL_INPUT_POWER_COMM",
    "TEST_DIGITAL_INPUT_POWER_COMM_MOTOR",
]

TEST_DIGITAL_OUTPUT_NO_POWER = 0x1C
TEST_DIGITAL_OUTPUT_POWER_COMM = 0x1E
TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR = 0x1F

TEST_DIGITAL_INPUT_NO_POWER = 0x9F00FFFF
TEST_DIGITAL_INPUT_POWER_COMM = 0x80007FFF
TEST_DIGITAL_INPUT_POWER_COMM_MOTOR = 0x3F
