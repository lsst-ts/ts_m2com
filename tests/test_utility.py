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
    check_limit_switches,
    check_queue_size,
    get_config_dir,
    is_coroutine,
    read_yaml_file,
)


class TestUtility(unittest.IsolatedAsyncioTestCase):
    """Test the functions in utility."""

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


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
