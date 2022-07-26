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

from lsst.ts.m2com import MockScriptEngine


class TestMockScriptEngine(unittest.TestCase):
    """Test the Mock Script Engine class."""

    def setUp(self):

        self.script_engine = MockScriptEngine()

    def test_set_name_exception(self):

        # Empty name
        self.assertRaises(ValueError, self.script_engine.set_name, "")

        # Script is running now
        self._set_script_and_run("test1")

        self.assertRaises(RuntimeError, self.script_engine.set_name, "test2")

    def _set_script_and_run(self, name):

        self.script_engine.set_name(name)
        self.script_engine.run()

    def test_set_name(self):

        name = "test"
        self.script_engine.percentage = 10

        self.script_engine.set_name(name)

        self.assertEqual(self.script_engine._name, name)
        self.assertEqual(self.script_engine.percentage, 0)

    def test_clear_exception(self):

        self._set_script_and_run("test")

        self.assertRaises(RuntimeError, self.script_engine.clear)

    def test_clear(self):

        self._run_script_to_done()

        self.script_engine.clear()

        self.assertEqual(self.script_engine._name, "")
        self.assertEqual(self.script_engine.percentage, 0)
        self.assertFalse(self.script_engine.is_running)

    def _run_script_to_done(self):

        self._set_script_and_run("test")
        self.script_engine.run_steps(100)

    def test_run_exception(self):

        # No script is assigned
        self.assertRaises(RuntimeError, self.script_engine.run)

        # The script is done already
        self._run_script_to_done()

        self.assertRaises(RuntimeError, self.script_engine.run)

        # The script is running now
        self._set_script_and_run("test")

        self.assertRaises(RuntimeError, self.script_engine.run)

    def test_run(self):

        self._set_script_and_run("test")

        self.assertTrue(self.script_engine.is_running)

    def test_stop(self):

        self._set_script_and_run("test")
        self.script_engine.stop()

        self.assertFalse(self.script_engine.is_running)
        self.assertEqual(self.script_engine.percentage, 100)

    def test_pause(self):

        self._set_script_and_run("test")
        self.script_engine.pause()

        self.assertFalse(self.script_engine.is_running)

    def test_run_steps_exception(self):

        # Not running yet
        self.assertRaises(RuntimeError, self.script_engine.run_steps, 10)

        # Value of steps is out of range
        self._set_script_and_run("test")
        self.assertRaises(ValueError, self.script_engine.run_steps, 101)

    def test_run_steps(self):

        self._set_script_and_run("test")

        # Normal step
        self.script_engine.run_steps(10)

        self.assertEqual(self.script_engine.percentage, 10)

        # Finish
        self.script_engine.run_steps(91)

        self.assertEqual(self.script_engine.percentage, 100)
        self.assertFalse(self.script_engine.is_running)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
