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
import logging
import unittest

import numpy as np
from lsst.ts.m2com import (
    NUM_ACTUATOR,
    camel_case,
    check_hardpoints,
    check_limit_switches,
    check_queue_size,
    correct_inclinometer_angle,
    get_config_dir,
    get_forces_mirror_weight,
    is_coroutine,
    read_error_code_file,
    read_yaml_file,
    select_axial_hardpoints,
)


class TestUtility(unittest.IsolatedAsyncioTestCase):
    """Test the functions in utility."""

    def setUp(self) -> None:
        yaml_file = get_config_dir() / "harrisLUT" / "cell_geom.yaml"

        self.cell_geom = read_yaml_file(yaml_file)

    def test_check_queue_size(self) -> None:
        # No information is logged
        queue: asyncio.Queue = asyncio.Queue(maxsize=3)
        log = logging.getLogger()
        self.assertFalse(check_queue_size(queue, log))

        queue.put_nowait(1)
        self.assertFalse(check_queue_size(queue, log))

        # Information is logged
        queue.put_nowait(2)
        self.assertTrue(check_queue_size(queue, log))

    def test_read_yaml_file_exception(self) -> None:
        self.assertRaises(IOError, read_yaml_file, "no_this_yaml_file.yaml")

    def test_read_yaml_file(self) -> None:
        yaml_file = get_config_dir() / "harrisLUT" / "cell_geom.yaml"
        content = read_yaml_file(yaml_file)

        self.assertEqual(content["radiusActTangent"], 1.780189734)

    def test_read_error_code_file(self) -> None:
        error_code_file = get_config_dir() / "error_code.tsv"
        content = read_error_code_file(error_code_file)

        self.assertEqual(len(content), 64)
        self.assertEqual(list(content.keys())[0], "1000")
        self.assertEqual(list(content.keys())[-1], "6087")

    def test_get_config_dir(self) -> None:
        path = get_config_dir()

        self.assertTrue(path.exists())
        self.assertEqual(path.name, "v2")

    def test_is_coroutine(self) -> None:
        self.assertTrue(is_coroutine(asyncio.sleep))
        self.assertFalse(is_coroutine(self._function))

    def _function(self) -> None:
        pass

    def test_check_limit_switches(self) -> None:
        # Nothing is triggered
        actuator_forces = np.zeros(NUM_ACTUATOR)
        limit_force_axial = 3
        limit_force_tangent = 10

        is_triggered, limit_switch_retract, limit_switch_extend = check_limit_switches(
            actuator_forces, limit_force_axial, limit_force_tangent
        )

        self.assertFalse(is_triggered)
        self.assertEqual(limit_switch_retract, [])
        self.assertEqual(limit_switch_extend, [])

        # Limit switches are triggered
        actuator_forces[[0, 1, 5, 71, 72, 73, 74, 77]] = [
            4.3,
            -3.1,
            -5.1,
            3,
            10,
            -20.3,
            -10,
            30,
        ]

        is_triggered, limit_switch_retract, limit_switch_extend = check_limit_switches(
            actuator_forces, limit_force_axial, limit_force_tangent
        )

        self.assertTrue(is_triggered)
        self.assertEqual(limit_switch_retract, [0, 71, 72, 77])
        self.assertEqual(limit_switch_extend, [1, 5, 73, 74])

    def test_camel_case(self) -> None:
        self.assertEqual(camel_case("ab"), "ab")
        self.assertEqual(camel_case("ab_cd"), "abCd")

    def test_check_hardpoints(self) -> None:
        # Good hardpoints
        check_hardpoints(
            self.cell_geom["locAct_axial"],
            [5, 15, 25],
            [72, 74, 76],
        )

        # Bad hardpoints
        self.assertRaises(
            ValueError,
            check_hardpoints,
            self.cell_geom["locAct_axial"],
            [5, 15, 24],
            [72, 74, 76],
        )

        self.assertRaises(
            ValueError,
            check_hardpoints,
            self.cell_geom["locAct_axial"],
            [5, 15, 25],
            [72, 73, 74],
        )

    def test_select_axial_hardpoints(self) -> None:
        self.assertEqual(
            select_axial_hardpoints(self.cell_geom["locAct_axial"], 4),
            [4, 14, 24],
        )
        self.assertEqual(
            select_axial_hardpoints(self.cell_geom["locAct_axial"], 15),
            [5, 15, 25],
        )
        self.assertEqual(
            select_axial_hardpoints(self.cell_geom["locAct_axial"], 26),
            [6, 16, 26],
        )

        self.assertEqual(
            select_axial_hardpoints(self.cell_geom["locAct_axial"], 0),
            [0, 10, 20],
        )

    def test_correct_inclinometer_angle(self) -> None:
        self.assertEqual(correct_inclinometer_angle(-10), 90)

        self.assertAlmostEqual(correct_inclinometer_angle(0), -180.94)
        self.assertAlmostEqual(correct_inclinometer_angle(30), -210.94)
        self.assertEqual(correct_inclinometer_angle(89.06), 90)
        self.assertAlmostEqual(correct_inclinometer_angle(90), 89.06)
        self.assertAlmostEqual(correct_inclinometer_angle(120), 59.06)
        self.assertAlmostEqual(correct_inclinometer_angle(200), -20.94)

        self.assertEqual(correct_inclinometer_angle(500), -270)

    def test_get_forces_mirror_weight(self) -> None:
        forces = get_forces_mirror_weight(120.0)

        self.assertAlmostEqual(forces[0], 185.4643083)

        self.assertEqual(forces[72], 0)
        self.assertAlmostEqual(forces[73], -2001.1325104)
        self.assertEqual(forces[75], 0)
        self.assertAlmostEqual(forces[76], 2001.1325104)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
