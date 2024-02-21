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
    "NUM_HARDPOINTS_AXIAL",
    "NUM_INNER_LOOP_CONTROLLER",
    "NUM_TEMPERATURE_RING",
    "NUM_TEMPERATURE_INTAKE",
    "NUM_TEMPERATURE_EXHAUST",
    "MIRROR_WEIGHT_KG",
    "LIMIT_FORCE_AXIAL_CLOSED_LOOP",
    "LIMIT_FORCE_TANGENT_CLOSED_LOOP",
    "LIMIT_FORCE_AXIAL_OPEN_LOOP",
    "LIMIT_FORCE_TANGENT_OPEN_LOOP",
    "MAX_LIMIT_FORCE_AXIAL_OPEN_LOOP",
    "MAX_LIMIT_FORCE_TANGENT_OPEN_LOOP",
    "TANGENT_LINK_TOTAL_WEIGHT_ERROR",
    "TANGENT_LINK_LOAD_BEARING_LINK",
    "TANGENT_LINK_THETA_Z_MOMENT",
    "TANGENT_LINK_NON_LOAD_BEARING_LINK",
    "MINIMUM_ERROR_CODE",
    "DEFAULT_ENABLED_FAULTS_MASK",
    "OUTLIER_INCLINOMETER_RAW",
    "TEST_DIGITAL_OUTPUT_NO_POWER",
    "TEST_DIGITAL_OUTPUT_POWER_COMM",
    "TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR",
    "TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR_CLOSED_LOOP",
    "TEST_DIGITAL_INPUT_NO_POWER",
    "TEST_DIGITAL_INPUT_POWER_COMM",
    "TEST_DIGITAL_INPUT_POWER_COMM_MOTOR",
]

NUM_ACTUATOR = 78
NUM_TANGENT_LINK = 6
NUM_HARDPOINTS_AXIAL = 3

NUM_INNER_LOOP_CONTROLLER = 84

NUM_TEMPERATURE_RING = 12
NUM_TEMPERATURE_INTAKE = 2
NUM_TEMPERATURE_EXHAUST = 2

MIRROR_WEIGHT_KG = 1588.65

# Limits of force in Newton
LIMIT_FORCE_AXIAL_CLOSED_LOOP = 444.82  # 100 lbf

# This value is ~ (MIRROR_WEIGHT_KG * 9.8 / 4 + TANGENT_LINK_LOAD_BEARING_LINK)
# under the condition that only tangent links bear the mirror's weight.
LIMIT_FORCE_TANGENT_CLOSED_LOOP = 4893.04  # 1100 lbf

LIMIT_FORCE_AXIAL_OPEN_LOOP = 489.3  # 110 lbf

# Add the buffer of (1000 lbf / 4) to each tangent link
# compared with LIMIT_FORCE_TANGENT_CLOSED_LOOP.
LIMIT_FORCE_TANGENT_OPEN_LOOP = 6005.1  # 1350 lbf

MAX_LIMIT_FORCE_AXIAL_OPEN_LOOP = 622.75  # 140 lbf

# Add the buffer of (200 lbf / 4) to each tangent link
# compared with LIMIT_FORCE_TANGENT_OPEN_LOOP.
MAX_LIMIT_FORCE_TANGENT_OPEN_LOOP = 6227.51  # 1400 lbf

# Thresholds of the force errors of the tangent links in
# TangentLoadCellFaultDetection.vi in ts_mtm2 LabVIEW project.
# The unit is Newton.

# The excess mass contains both the "dynamic load" and "actual weight error".
# For the dynamic load, consider the TMA acceleration at 100% value is ~0.06 g.
# Therefore the dynamic force = MIRROR_WEIGHT_KG * 9.8 * 0.06 ~ 934.13 N.
# That leaves ~1000 N for the "actual weight error" component in the total
# error budget of 2000 N.
TANGENT_LINK_TOTAL_WEIGHT_ERROR = 2000

TANGENT_LINK_LOAD_BEARING_LINK = 2000
TANGENT_LINK_THETA_Z_MOMENT = 1000
TANGENT_LINK_NON_LOAD_BEARING_LINK = 2000

# Randomly-chosen minimum error code by vendor
MINIMUM_ERROR_CODE = 6000

# Default enabled faults mask in the cell controller
DEFAULT_ENABLED_FAULTS_MASK = 0xFF800007FFFFFFFF

# Outlier threshold from inner-loop controller (ILC) telemetry
OUTLIER_INCLINOMETER_RAW = 1000000

TEST_DIGITAL_OUTPUT_NO_POWER = 0x1C
TEST_DIGITAL_OUTPUT_POWER_COMM = 0x1E
TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR = 0x1F
TEST_DIGITAL_OUTPUT_POWER_COMM_MOTOR_CLOSED_LOOP = 0x3F

TEST_DIGITAL_INPUT_NO_POWER = 0x9F00FFFF
TEST_DIGITAL_INPUT_POWER_COMM = 0x80007FFF
TEST_DIGITAL_INPUT_POWER_COMM_MOTOR = 0x3F
