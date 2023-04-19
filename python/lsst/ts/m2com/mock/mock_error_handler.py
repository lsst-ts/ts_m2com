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

import typing

__all__ = ["MockErrorHandler"]

from ..constant import NUM_ACTUATOR
from ..enum import LimitSwitchType
from ..error_handler import ErrorHandler


class MockErrorHandler(ErrorHandler):
    """Mock Error Handler class to manage the errors."""

    def __init__(self) -> None:
        super().__init__()

        # New triggered retracted limit switches
        self._limit_switches_retract_new: typing.Set[int] = set()

        # Reported triggered retracted limit switches
        self._limit_switches_retract_reported: typing.Set[int] = set()

        # New triggered extended limit switches
        self._limit_switches_extend_new: typing.Set[int] = set()

        # Reported triggered extended limit switches
        self._limit_switches_extend_reported: typing.Set[int] = set()

    def clear(self) -> None:
        """Clear all errors."""
        super().clear()

        self._limit_switches_retract_new.clear()
        self._limit_switches_retract_reported.clear()

        self._limit_switches_extend_new.clear()
        self._limit_switches_extend_reported.clear()

    def add_new_limit_switch(
        self, actuator_id: int, limit_switch_type: LimitSwitchType
    ) -> None:
        """Add the new triggered limit switch.

        Parameters
        ----------
        actuator_id : `int`
            Actuator ID: [0, NUM_ACTUATOR).
        limit_switch_type : enum `LimitSwitchType`
            Limit switch type.

        Raises
        ------
        `ValueError`
            When the actuator ID is not in the range.
        """

        if not (0 <= actuator_id < NUM_ACTUATOR):
            raise ValueError(
                f"Actuator ID ({actuator_id}) should be in [0, {NUM_ACTUATOR})."
            )

        if limit_switch_type == LimitSwitchType.Retract:
            self._add_new_item(
                self._limit_switches_retract_new,
                self._limit_switches_retract_reported,
                actuator_id,
            )

        else:
            self._add_new_item(
                self._limit_switches_extend_new,
                self._limit_switches_extend_reported,
                actuator_id,
            )

    def exists_new_limit_switch(self, limit_switch_type: LimitSwitchType) -> bool:
        """Exists the new triggered limit switch (not reported) or not.

        Parameters
        ----------
        limit_switch_type : enum `LimitSwitchType`
            Limit switch type.

        Returns
        -------
        `bool`
            True if there is the new trigged limit switch. Otherwise, False.
        """
        return (
            (len(self._limit_switches_retract_new) != 0)
            if limit_switch_type == LimitSwitchType.Retract
            else (len(self._limit_switches_extend_new) != 0)
        )

    def exists_limit_switch(self, limit_switch_type: LimitSwitchType) -> bool:
        """Exists the triggered limit switch (new or reported) or not.

        Parameters
        ----------
        limit_switch_type : enum `LimitSwitchType`
            Limit switch type.

        Returns
        -------
        `bool`
            True if there is the triggered limit switch. Otherwise, False.
        """
        return self.exists_new_limit_switch(limit_switch_type) or (
            (len(self._limit_switches_retract_reported) != 0)
            if limit_switch_type == LimitSwitchType.Retract
            else (len(self._limit_switches_extend_reported) != 0)
        )

    def get_limit_switches_to_report(
        self, limit_switch_type: LimitSwitchType
    ) -> typing.Set[int]:
        """Get the limit switches that are not reported yet.

        Parameters
        ----------
        limit_switch_type : enum `LimitSwitchType`
            Limit switch type.

        Returns
        -------
        `set`
            Limit switches to report.
        """

        # Note the union() will return a new set object
        if limit_switch_type == LimitSwitchType.Retract:
            self._limit_switches_retract_reported = (
                self._limit_switches_retract_reported.union(
                    self._limit_switches_retract_new
                )
            )

            return self._get_items_in_set_and_clear(self._limit_switches_retract_new)

        else:
            self._limit_switches_extend_reported = (
                self._limit_switches_extend_reported.union(
                    self._limit_switches_extend_new
                )
            )

            return self._get_items_in_set_and_clear(self._limit_switches_extend_new)
