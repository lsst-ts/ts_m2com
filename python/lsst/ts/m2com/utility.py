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

import json
import yaml
from copy import deepcopy

from lsst.ts import tcpip

__all__ = [
    "write_json_packet",
    "check_queue_size",
    "read_yaml_file",
    "collect_queue_messages",
    "get_queue_message_latest",
]


async def write_json_packet(writer, msg_input):
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


def check_queue_size(queue, log, name=""):
    """Check the size of queue and log the information if needed.

    Parameters
    ----------
    queue : `asyncio.Queue`
        Queue.
    log : `logging.Logger`
        A logger.
    name : `str`, optional
        Additional string to add to the log message for tracking purposes.

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


def read_yaml_file(filepath):
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


def collect_queue_messages(queue, name, flush=True):
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


def get_queue_message_latest(queue, name, flush=True):
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
