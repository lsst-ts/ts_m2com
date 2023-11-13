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
import csv
import json
import logging
import typing
from copy import deepcopy
from os import getenv
from pathlib import Path
from re import sub

import numpy as np
import numpy.typing
import yaml
from lsst.ts import tcpip
from scipy.spatial import KDTree

from .constant import NUM_ACTUATOR, NUM_TANGENT_LINK

__all__ = [
    "write_json_packet",
    "check_queue_size",
    "read_yaml_file",
    "read_error_code_file",
    "collect_queue_messages",
    "get_queue_message_latest",
    "get_config_dir",
    "is_coroutine",
    "check_limit_switches",
    "camel_case",
    "check_hardpoints",
    "select_axial_hardpoints",
]


async def write_json_packet(writer: asyncio.StreamWriter, msg_input: dict) -> None:
    """Write the json packet.

    Parameters
    ----------
    writer : `asyncio.StreamWriter`
        Writer of the socket.
    msg_input : `dict`
        Input message.
    """

    # Transfer to json string and do the encode
    msg = json.dumps(msg_input).encode() + tcpip.TERMINATOR

    writer.write(msg)
    await writer.drain()


def check_queue_size(queue: asyncio.Queue, log: logging.Logger, name: str = "") -> bool:
    """Check the size of queue and log the information if needed.

    Parameters
    ----------
    queue : `asyncio.Queue`
        Queue.
    log : `logging.Logger`
        A logger.
    name : `str`, optional
        Additional string to add to the log message for tracking purposes. (the
        default is "")

    Returns
    -------
    `bool`
        True if the information is logged or not. Otherwise, Fault.
    """

    queue_size = queue.qsize()
    maxsize = queue.maxsize

    if queue_size > maxsize // 2:
        log.info(f"[{name}] Size of queue is: {queue_size}/{maxsize}.")
        return True

    else:
        return False


def read_yaml_file(filepath: str | Path) -> dict:
    """Read the yaml file.

    Parameters
    ----------
    filepath : `str` or `pathlib.PosixPath`
        Yaml file path.

    Returns
    -------
    content : `dict`
        File content.

    Raises
    ------
    `IOError`
        Cannot open the file.
    """

    content = dict()
    try:
        with open(filepath, "r") as yaml_file:
            content = yaml.safe_load(yaml_file)
    except IOError:
        raise IOError(f"Cannot open the yaml file: {filepath}.")

    return content


def read_error_code_file(filepath: str | Path) -> dict[str, list]:
    """Read the error code file.

    Parameters
    ----------
    filepath : `str` or `pathlib.PosixPath`
        TSV file path.

    Returns
    -------
    content : `dict`
        File content.
    """

    content = dict()
    with open(filepath, "r") as file:
        csv_reader = csv.reader(file, delimiter="\t")

        # Get the error details
        for row in csv_reader:
            if row[0].isdigit():
                content[row[0]] = row[1:]

    return content


def collect_queue_messages(queue: asyncio.Queue, name: str, flush: bool = True) -> list:
    """Collect the specific messages in queue.

    This function is used in the unit test only.

    Parameters
    ----------
    queue : `asyncio.Queue`
        Queue of message.
    name : `str`
        Name of message.
    flush : `bool`, optional
        Flush the queue or not. (the default is True)

    Returns
    -------
    messages : `list`
        Messages.
    """

    queue_check = queue if flush else deepcopy(queue)

    messages = list()
    while not queue_check.empty():
        message = queue_check.get_nowait()
        if message["id"] == name:
            messages.append(message)

    return messages


def get_queue_message_latest(
    queue: asyncio.Queue, name: str, flush: bool = True
) -> dict:
    """Get the latest message in queue.

    This function is used in the unit test only.

    Parameters
    ----------
    queue : `asyncio.Queue`
        Queue of message.
    name : `str`
        Name of message.
    flush : `bool`, optional
        Flush the queue or not. (the default is True)

    Returns
    -------
    message_latest : `dict`
        Latest message.
    """

    messages = collect_queue_messages(queue, name, flush=flush)

    return messages[-1]


def get_config_dir(
    env_variable: str = "TS_CONFIG_MTTCS_DIR",
    relative_path: str = "MTM2/v2",
) -> Path:
    """Get the directory of configuration files.

    Parameters
    ----------
    env_variable : `str`, optional
        Environment variable of "ts_config_mttcs". (the default is
        "TS_CONFIG_MTTCS_DIR")
    relative_path : `str`, optional
        Relative path to the "ts_config_mttcs". (the default is
        "MTM2/v2")

    Returns
    -------
    `pathlib.PosixPath`
        Path of the configuration directory.
    """
    return Path(getenv(env_variable, default="")) / relative_path


def is_coroutine(function: typing.Callable | typing.Coroutine) -> bool:
    """Input function is a coroution or not.

    Parameters
    ----------
    function : `func` or `coroutine`
        Function.

    Returns
    -------
    `bool`
        True if the function is a corountine. Otherwise, False.
    """
    return asyncio.iscoroutine(function) or asyncio.iscoroutinefunction(function)


def check_limit_switches(
    actuator_forces: numpy.typing.NDArray[np.float64],
    limit_force_axial: float,
    limit_force_tangent: float,
) -> tuple[bool, list, list]:
    """Check the limit switches are triggered or not.

    Parameters
    ----------
    actuator_forces : `numpy.ndarray`
        Actuator forces in Newton. The number should be the same as
        "NUM_ACTUATOR".
    limit_force_axial : `float`
        Maximum limit force of the axial actuator in Newton.
    limit_force_tangent : `float`
        Maximum limit force of the tangent actuator in Newton.

    Returns
    -------
    is_triggered : `bool`
        Limit switch is triggered or not.
    `list`
        Triggered retracted limit switches.
    `list`
        Triggered extended limit switches.

    Raises
    ------
    `ValueError`
        If the number of "actuator_forces" is wrong.
    """

    # Check the number of actuators is correct
    num_actuator_forces = len(actuator_forces)
    if num_actuator_forces != NUM_ACTUATOR:
        raise ValueError(
            f"Number of actuator_forces ({num_actuator_forces}) should be {NUM_ACTUATOR}."
        )

    # Get the triggered limit switches
    num_actuator_axial = NUM_ACTUATOR - NUM_TANGENT_LINK

    limit_switch_axial_retract = np.where(
        actuator_forces[:num_actuator_axial] >= limit_force_axial
    )[0]
    limit_switch_axial_extend = np.where(
        actuator_forces[:num_actuator_axial] <= -limit_force_axial
    )[0]

    limit_switch_tangent_retract = (
        np.where(actuator_forces[-NUM_TANGENT_LINK:] >= limit_force_tangent)[0]
        + num_actuator_axial
    )
    limit_switch_tangent_extend = (
        np.where(actuator_forces[-NUM_TANGENT_LINK:] <= -limit_force_tangent)[0]
        + num_actuator_axial
    )

    limit_switch_retract = np.append(
        limit_switch_axial_retract, limit_switch_tangent_retract
    ).astype(int)

    limit_switch_extend = np.append(
        limit_switch_axial_extend, limit_switch_tangent_extend
    ).astype(int)

    is_triggered = (len(limit_switch_retract) != 0) or (len(limit_switch_extend) != 0)

    return is_triggered, limit_switch_retract.tolist(), limit_switch_extend.tolist()


def camel_case(string_python: str) -> str:
    """Formate the string of Python style to camel case. For example, 'ab_cd'
    will be reformated to 'abCd'.

    Copy from:
    https://www.w3resource.com/python-exercises/string/
    python-data-type-string-exercise-96.php

    Parameters
    ----------
    string_python : `str`
        Python-style string.

    Returns
    -------
    `str`
        String of camel case.
    """

    string_reformat = sub(r"(_|-)+", " ", string_python).title().replace(" ", "")
    return "".join([string_reformat[0].lower(), string_reformat[1:]])


def check_hardpoints(
    location_axial_actuator: list[list],
    hardpoints_axial: list[int],
    hardpoints_tangent: list[int],
) -> None:
    """Check the selected hardpoints are good or not.

    Parameters
    ----------
    location_axial_actuator : `list`
        Location of the axial actuators: (x, y). This should be a 72 x 2
        matrix.
    hardpoints_axial : `list`
        Three axial hardpoints. The order is from low to high,
        e.g. [5, 15, 25].
    hardpoints_tangent : `list`
        Three tangential hardpoints. This can only be [72, 74, 76] or
        [73, 75, 77]. The order is from low to high.

    Raises
    ------
    `ValueError`
        If the axial hardpoints are bad.
    `ValueError`
        If the tangential hardpoints are wrong.
    """

    # Axial hardpoints

    # Check the hardpoints by comparing with the expectation
    if (
        select_axial_hardpoints(location_axial_actuator, hardpoints_axial[0])
        != hardpoints_axial
    ):
        raise ValueError("Bad selection of axial hardpoints.")

    # Tangent hardpoints
    option_one = [72, 74, 76]
    option_two = [73, 75, 77]
    if hardpoints_tangent not in (option_one, option_two):
        raise ValueError(
            f"Tangential hardpoints can only be {option_one} or {option_two}."
        )


def select_axial_hardpoints(
    location_axial_actuator: list[list], specific_axial_hardpoint: int
) -> list[int]:
    """Select the axial hardpoints based on the specific axial hardpoint.

    Notes
    -----
    Translate the calculation from the OptAxHardpointSelect.m in
    ts_mtm2_matlab_tools.

    The idea is to maximize the triangle constructed by 3 axial actuators,
    which means it should be closed to the equilateral triangle.

    Parameters
    ----------
    location_axial_actuator : `list`
        Location of the axial actuators: (x, y). This should be a 72 x 2
        matrix.
    specific_axial_hardpoint : `int`
        Specific axial hardpoint.

    Returns
    -------
    hardpoints : `list`
        Selected 3 axial hardpoints that contains the specific axial
        hardpoint. The order is from low to high.
    """

    # Get the polar coordinate of specific
    x, y = location_axial_actuator[specific_axial_hardpoint]
    radius = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)

    # Get the angles of the other two hardpoints
    theta_one = theta + np.deg2rad(120)
    theta_two = theta + np.deg2rad(240)

    # Get the closest actuator index
    hardpoints = list(
        KDTree(location_axial_actuator).query(
            [
                [radius * np.cos(theta_one), radius * np.sin(theta_one)],
                [radius * np.cos(theta_two), radius * np.sin(theta_two)],
            ]
        )[1]
    )

    # Return the sorted hardpoints
    hardpoints.append(specific_axial_hardpoint)
    hardpoints.sort()

    return hardpoints
