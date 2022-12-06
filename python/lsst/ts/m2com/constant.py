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
    "NUM_ACTUATOR",
    "NUM_TANGENT_LINK",
    "MIRROR_WEIGHT_KG",
    "LIMIT_FORCE_AXIAL_CLOSED_LOOP",
    "LIMIT_FORCE_TANGENT_CLOSED_LOOP",
    "LIMIT_FORCE_AXIAL_OPEN_LOOP",
    "LIMIT_FORCE_TANGENT_OPEN_LOOP",
    "MAX_LIMIT_FORCE_AXIAL_OPEN_LOOP",
    "MAX_LIMIT_FORCE_TANGENT_OPEN_LOOP",
    "TEST_DIGITAL_OUTPUT_NO_POWER",
    "TEST_DIGITAL_OUTPUT_POWER_COMM",
    "TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR",
    "TEST_DIGITAL_INPUT_NO_POWER",
    "TEST_DIGITAL_INPUT_POWER_COMM",
    "TEST_DIGITAL_INPUT_POWER_COMM_MOTOR",
]

NUM_ACTUATOR = 78
NUM_TANGENT_LINK = 6

MIRROR_WEIGHT_KG = 1588.65

# Limits of force in Newton
LIMIT_FORCE_AXIAL_CLOSED_LOOP = 444.82  # 100 lbf
LIMIT_FORCE_TANGENT_CLOSED_LOOP = 4893.04  # 1100 lbf

LIMIT_FORCE_AXIAL_OPEN_LOOP = 489.3  # 110 lbf
LIMIT_FORCE_TANGENT_OPEN_LOOP = 6005.1  # 1350 lbf

MAX_LIMIT_FORCE_AXIAL_OPEN_LOOP = 622.75  # 140 lbf
MAX_LIMIT_FORCE_TANGENT_OPEN_LOOP = 6227.51  # 1400 lbf

TEST_DIGITAL_OUTPUT_NO_POWER = 0x1C
TEST_DIGITAL_OUTPUT_POWER_COMM = 0x1E
TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR = 0x1F

TEST_DIGITAL_INPUT_NO_POWER = 0x9F00FFFF
TEST_DIGITAL_INPUT_POWER_COMM = 0x80007FFF
TEST_DIGITAL_INPUT_POWER_COMM_MOTOR = 0x3F
