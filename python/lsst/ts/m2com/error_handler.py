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

__all__ = ["ErrorHandler"]

from pathlib import Path

from .constant import MINIMUM_ERROR_CODE
from .utility import read_error_code_file


class ErrorHandler:
    """Error Handler class to manage the errors.

    Attributes
    ----------
    list_code_total : `list`
        List of the dummy, error, and warning codes to map the bit value of
        summary faults status in cell controller with the related code.
    """

    def __init__(self) -> None:
        self.list_code_total: list[int] = list()

        # List of the defined error code
        self._list_code_error: list[int] = list()

        # List of the defined warning code
        self._list_code_warning: list[int] = list()

        # New errors in the system
        self._errors_new: set[int] = set()

        # Errors that had been reported
        self._errors_reported: set[int] = set()

        # New warnings in the system
        self._warnings_new: set[int] = set()

        # Warnings that had been reported
        self._warnings_reported: set[int] = set()

    def read_error_list_file(self, filepath: str | Path) -> None:
        """Read the tsv file of error list.

        Parameters
        ----------
        filepath : `str` or `pathlib.PosixPath`
            File path.
        """

        # Reset the lists
        self.list_code_total = list()
        self._list_code_error = list()
        self._list_code_warning = list()

        # Read the file
        content = read_error_code_file(filepath)
        for key, value in content.items():
            error_code = int(key)
            self.list_code_total.append(error_code)

            if error_code >= MINIMUM_ERROR_CODE:
                list_code = (
                    self._list_code_error
                    if value[1] == "Fault"
                    else self._list_code_warning
                )
                list_code.append(error_code)

    def clear(self) -> None:
        """Clear all warnings and errors."""

        self._errors_new.clear()
        self._errors_reported.clear()

        self._warnings_new.clear()
        self._warnings_reported.clear()

    def decode_summary_faults_status(self, status: int) -> None:
        """Decode the summary faults status. The decoded error/warning code
        will be put into the internal lists of error and warning.

        Parameters
        ----------
        status : `int`
            Summary faults status.
        """
        for idx, code in enumerate(self.list_code_total):
            mask = 2**idx
            if status < mask:
                break

            # Note some code is just dummy. Only collect the error and warning
            # code.
            if (status & mask) != 0:
                if self.is_error(code):
                    self.add_new_error(code)
                    continue

                if self.is_warning(code):
                    self.add_new_warning(code)

    def is_error(self, code: int) -> bool:
        """Is error or not.

        Parameters
        ----------
        code : `int`
            Code.

        Returns
        -------
        `bool`
            True if the code belongs to error. Otherwise, False.
        """
        return code in self._list_code_error

    def is_warning(self, code: int) -> bool:
        """Is warning or not.

        Parameters
        ----------
        code : `int`
            Code.

        Returns
        -------
        `bool`
            True if the code belongs to warning. Otherwise, False.
        """
        return code in self._list_code_warning

    def add_new_error(self, code: int) -> None:
        """Add the new error code.

        Parameters
        ----------
        code : `int`
            Error code.
        """
        self._add_new_item(self._errors_new, self._errors_reported, code)

    def _add_new_item(
        self, set_new: set[int], set_reported: set[int], item: int
    ) -> None:
        """Add the new item to set.

        The existed item will not be added.

        Parameters
        ----------
        set_new : `set`
            New set.
        set_reported : `set`
            Reported set.
        item : `int`
            Item to add.
        """

        if (item not in set_new) and (item not in set_reported):
            set_new.add(item)

    def add_new_warning(self, code: int) -> None:
        """Add the new warning code.

        Parameters
        ----------
        code : `int`
            Warning code.
        """
        self._add_new_item(self._warnings_new, self._warnings_reported, code)

    def exists_new_error(self) -> bool:
        """Exists the new error (not reported) or not.

        Returns
        -------
        `bool`
            True if there is the new error. Otherwise, False.
        """
        return len(self._errors_new) != 0

    def exists_new_warning(self) -> bool:
        """Exists the new warning (not reported) or not.

        Returns
        -------
        `bool`
            True if there is the new warning. Otherwise, False.
        """
        return len(self._warnings_new) != 0

    def exists_error(self) -> bool:
        """Exists the error (new or reported) or not.

        Returns
        -------
        `bool`
            True if there is the error. Otherwise, False.
        """
        return self.exists_new_error() or (len(self._errors_reported) != 0)

    def exists_warning(self) -> bool:
        """Exists the warning (new or reported) or not.

        Returns
        -------
        `bool`
            True if there is the error. Otherwise, False.
        """
        return self.exists_new_warning() or (len(self._warnings_reported) != 0)

    def get_errors_to_report(self) -> set[int]:
        """Get the errors that are not reported yet.

        Returns
        -------
        `set`
            Errors to report.
        """

        # Note the union() will return a new set object
        self._errors_reported = self._errors_reported.union(self._errors_new)

        return self._get_items_in_set_and_clear(self._errors_new)

    def _get_items_in_set_and_clear(self, specific_set: set[int]) -> set[int]:
        """Get the items in the specific set and clear the set.

        Parameters
        ----------
        specific_set : `set`
            Specific set.

        Returns
        -------
        items : `set`
            Items in the set.
        """

        items = specific_set.copy()
        specific_set.clear()

        return items

    def get_warnings_to_report(self) -> set[int]:
        """Get the warnings that are not reported yet.

        Returns
        -------
        `set`
            Warnings to report.
        """

        # Note the union() will return a new set object
        self._warnings_reported = self._warnings_reported.union(self._warnings_new)

        return self._get_items_in_set_and_clear(self._warnings_new)

    def has_error(self, code: int) -> bool:
        """Has received the specific error or not.

        Parameters
        ----------
        code : `int`
            Error code.

        Returns
        -------
        `bool`
            True if the code is received already. Otherwise, False.
        """
        return (code in self._errors_reported) or (code in self._errors_new)

    def has_warning(self, code: int) -> bool:
        """Has received the specific warning or not.

        Parameters
        ----------
        code : `int`
            Warning code.

        Returns
        -------
        `bool`
            True if the code is received already. Otherwise, False.
        """
        return (code in self._warnings_reported) or (code in self._warnings_new)

    def get_bit_from_code(self, code: int) -> int:
        """Get the bit value from code.

        Parameters
        ----------
        code : `int`
            Code.

        Returns
        -------
        `int`
            0-based bit value.
        """
        return self.list_code_total.index(code)

    def get_summary_faults_status_from_codes(self, codes: list[int]) -> int:
        """Get the summary faults status from codes.

        Parameters
        ----------
        codes : `list` [`int`]
            Collection of the dummy, error, and warning codes to construct the
            summary fault status. There should be no repeated value inside.

        Returns
        -------
        `int`
            Summary faults status.
        """
        summary_faults_status = 0
        for code in codes:
            summary_faults_status += 2 ** self.get_bit_from_code(code)

        return summary_faults_status

    def get_summary_faults_status_to_report(self) -> int:
        """Get the summary faults status to report.

        Note this function will put all unreported errors and warnings to the
        reported items.

        Returns
        -------
        `int`
            Summary faults status.
        """

        self.get_errors_to_report()
        self.get_warnings_to_report()

        errors_and_warnings = self._errors_reported.union(self._warnings_reported)
        return self.get_summary_faults_status_from_codes(list(errors_and_warnings))
