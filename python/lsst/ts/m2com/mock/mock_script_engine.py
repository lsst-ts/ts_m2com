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

__all__ = ["MockScriptEngine"]


class MockScriptEngine:
    """Mock script engine class to simulate the execution of binary script.

    Attributes
    ----------
    is_running : `bool`
        The script is running or not.
    percentage : `int` or `float`
        Percentage of the running.
    """

    def __init__(self):

        self._name = ""

        self.is_running = False
        self.percentage = 0

    def set_name(self, name):
        """Set the script name.

        Parameters
        ----------
        name : `str`
            Script name.

        Raises
        ------
        `ValueError`
            When the script name is empty.
        `RuntimeError`
            When the script is running.
        """

        if name == "":
            raise ValueError("The script name is empty.")

        if self.is_running:
            raise RuntimeError("The script is running now.")

        self._name = name
        self.percentage = 0

    def clear(self):
        """Clear the script.

        Raises
        ------
        `RuntimeError`
            When the script is running.
        """

        if self.is_running:
            raise RuntimeError("The script is running now.")
        else:
            self._name = ""
            self.percentage = 0

    def run(self):
        """Run the script.

        Raises
        ------
        `RuntimeError`
            When the script name is empty.
        `RuntimeError`
            When the script is running.
        `RuntimeError`
            When the script is done.
        """

        if self._name == "":
            raise RuntimeError("The script name is empty.")

        if self.is_running:
            raise RuntimeError("The script is running now.")

        if self.percentage < 100:
            self.is_running = True
        else:
            raise RuntimeError("The script is done.")

    def stop(self):
        """Stop the script."""

        self.is_running = False

        # Need to set the self.percentage to be 100 to avoid the self.run()
        # can put the self.is_running to be True again.
        # By doing this, the behaviors between self.stop() and self.pause()
        # are different.
        self.percentage = 100

    def pause(self):
        """Pause the script."""
        self.is_running = False

    def run_steps(self, steps):
        """Run the steps.

        If the function executes successfully, the self.percentage will add the
        value of steps. The maximum value of self.percentage is 100, and the
        script is considered done. When this happens, the execution of this
        function will raise the error.

        Parameters
        ----------
        steps : `int` or `float`
            Steps to run. The value should be between 0 and 100.

        Raises
        ------
        `RuntimeError`
            When the script engine is not running.
        `ValueError`
            When the value of steps is not in [0, 100].
        """

        if not self.is_running:
            raise RuntimeError("The script engine is not running now.")

        if not (0 <= steps <= 100):
            raise ValueError(f"Steps (={steps}) should be between 0 and 100.")

        self.percentage += steps
        if self.percentage >= 100:
            self.percentage = 100
            self.is_running = False
