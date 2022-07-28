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

from lsst.ts import salobj
from lsst.ts.idl.enums import MTM2

from ..enum import (
    DetailedState,
    CommandStatus,
    CommandScript,
    CommandActuator,
    ActuatorDisplacementUnit,
    PowerType,
    DigitalOutput,
)


__all__ = ["MockCommand"]


class MockCommand:
    """Mock command to simulate the execution of command in real hardware.

    Parameters
    ----------
    is_csc : `bool`, optional
        Is called by the commandable SAL component (CSC) or not. (the default
        is True)
    """

    SLEEP_TIME_SHORT = 0.01
    SLEEP_TIME_NORMAL = 5

    def __init__(self, is_csc=True):

        # Is CSC or not.
        self._is_csc = is_csc

    async def enable(self, message, model, message_event):
        """Enable the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.motor_power_on = True
        command_success = model.switch_force_balance_system(True)

        if not command_success:
            model.motor_power_on = False

        await message_event.write_m2_assembly_in_position(False)
        await message_event.write_force_balance_system_status(
            model.force_balance_system_status
        )

        # Simulate the real hardware behavior
        await asyncio.sleep(self.SLEEP_TIME_NORMAL)

        await message_event.write_summary_state(salobj.State.ENABLED)

        if command_success:
            await self._report_digital_input_and_ouput(model, message_event)

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def _report_digital_input_and_ouput(self, model, message_event):
        """Report the digital input and output.

        Parameters
        ----------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        digital_input = model.get_digital_input()
        await message_event.write_digital_input(digital_input)

        digital_output = model.get_digital_output()
        await message_event.write_digital_output(digital_output)

    async def disable(self, message, model, message_event):
        """Disable the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.switch_force_balance_system(False)
        model.motor_power_on = False

        await message_event.write_force_balance_system_status(
            model.force_balance_system_status
        )

        await message_event.write_summary_state(salobj.State.DISABLED)

        await self._report_digital_input_and_ouput(model, message_event)

        return model, CommandStatus.Success

    async def standby(self, message, model, message_event):
        """Standby the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.communication_power_on = False
        model.motor_power_on = False

        await message_event.write_summary_state(salobj.State.STANDBY)

        await self._report_digital_input_and_ouput(model, message_event)

        return model, CommandStatus.Success

    async def start(self, message, model, message_event):
        """Start the system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.communication_power_on = True

        await message_event.write_summary_state(salobj.State.DISABLED)

        await self._report_digital_input_and_ouput(model, message_event)

        return model, CommandStatus.Success

    async def enter_control(self, message, model, message_event):
        """Enter the control.

        This is only supported for the commandable SAL component (CSC). In the
        future, this command will be removed after the state machines in cell
        controller are unified to a single one.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        if self._is_csc:
            await message_event.write_summary_state(salobj.State.STANDBY)

            await self._report_digital_input_and_ouput(model, message_event)

            return model, CommandStatus.Success

        else:
            return model, CommandStatus.Fail

    async def exit_control(self, message, model, message_event):
        """Exit the control.

        This is only supported for the commandable SAL component (CSC). In the
        future, this command will be removed after the state machines in cell
        controller are unified to a single one.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        if self._is_csc:

            model.communication_power_on = False
            model.motor_power_on = False

            # Sleep time is to simulate some internal inspection of
            # real system
            await asyncio.sleep(self.SLEEP_TIME_SHORT)
            await message_event.write_detailed_state(DetailedState.PublishOnly)
            await asyncio.sleep(self.SLEEP_TIME_SHORT)
            await message_event.write_detailed_state(DetailedState.Available)

            await message_event.write_summary_state(salobj.State.OFFLINE)

            await self._report_digital_input_and_ouput(model, message_event)

            return model, CommandStatus.Success

        else:
            return model, CommandStatus.Fail

    async def apply_forces(self, message, model, message_event):
        """Apply the forces in addtional to the LUT force.

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        command_status = CommandStatus.Success
        try:
            model.apply_forces(message["axial"], message["tangent"])
            await message_event.write_m2_assembly_in_position(False)

        except RuntimeError:
            command_status = CommandStatus.Fail

        return model, command_status

    async def position_mirror(self, message, model, message_event):
        """Position the mirror.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        mirror_position_set_point = {
            "x": message["x"],
            "y": message["y"],
            "z": message["z"],
            "xRot": message["xRot"],
            "yRot": message["yRot"],
            "zRot": message["zRot"],
        }

        try:
            command_success = model.handle_position_mirror(mirror_position_set_point)
            await message_event.write_m2_assembly_in_position(False)

        except RuntimeError:
            command_success = False

        await message_event.write_m2_assembly_in_position(command_success)

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def reset_force_offsets(self, message, model, message_event):
        """Reset the actuator force offsets (not LUT force).

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.reset_force_offsets()
        await message_event.write_m2_assembly_in_position(False)

        return model, CommandStatus.Success

    async def clear_errors(self, message, model, message_event):
        """Clear the system errors.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.clear_errors()

        # In EUI, there is no OFFLINE state.
        if self._is_csc:
            model, command_status = await self.exit_control(
                message, model, message_event
            )
        else:
            model, command_status = await self.standby(message, model, message_event)

        return model, command_status

    async def switch_force_balance_system(self, message, model, message_event):
        """Switch the force balance system.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        command_success = model.switch_force_balance_system(message["status"])

        await message_event.write_force_balance_system_status(
            model.force_balance_system_status
        )

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def select_inclination_source(self, message, model, message_event):
        """Select the source of inclination.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        source = int(message["source"])
        model.inclination_source = MTM2.InclinationTelemetrySource(source)

        await message_event.write_inclination_telemetry_source(
            MTM2.InclinationTelemetrySource(source)
        )

        return model, CommandStatus.Success

    async def set_temperature_offset(self, message, model, message_event):
        """Set the temperature offset used in the calculation of LUT force.

        LUT: look-up table.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        # The ref can only be a single value at this moment
        # After the M2 server code is updated, we will change the behavior here
        offset = message["ring"].copy()
        offset.extend(message["intake"])
        offset.extend(message["exhaust"])

        command_status = CommandStatus.Success
        if len(set(offset)) == 1:
            model.temperature["ref"] = offset[0]
            await message_event.write_temperature_offset(
                message["ring"], message["intake"], message["exhaust"]
            )
        else:
            command_status = CommandStatus.Fail

        return model, command_status

    async def switch_command_source(self, message, model, message_event):
        """Switch the command source to be the commandable SAL component (CSC)
        or engineering user interface (EUI).

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        is_csc = message["isRemote"]
        await message_event.write_commandable_by_dds(is_csc)

        return model, CommandStatus.Success

    async def run_script(self, message, model, message_event):
        """Run the binary script used in the engineering user interface (EUI).

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        try:
            command = CommandScript(message["scriptCommand"])

            if command == CommandScript.LoadScript:
                model.script_engine.set_name(message["scriptName"])

            elif command == CommandScript.Clear:
                model.script_engine.clear()

            elif command in (CommandScript.Run, CommandScript.Resume):
                model.script_engine.run()

            elif command == CommandScript.Stop:
                model.script_engine.stop()

            elif command == CommandScript.Pause:
                model.script_engine.pause()

        except Exception:
            return model, CommandStatus.Fail

        return model, CommandStatus.Success

    async def move_actuators(self, message, model, message_event):
        """Move the actuators.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        try:
            command = CommandActuator(message["actuatorCommand"])

            if command == CommandActuator.Start:
                actuators = message["actuators"]
                displacement = message["displacement"]
                unit = ActuatorDisplacementUnit(message["unit"])
                model.control_open_loop.start(actuators, displacement, unit)

            elif command == CommandActuator.Stop:
                model.control_open_loop.stop()

            elif command == CommandActuator.Pause:
                model.control_open_loop.pause()

            elif command == CommandActuator.Resume:
                model.control_open_loop.resume()

        except Exception:
            return model, CommandStatus.Fail

        return model, CommandStatus.Success

    async def reset_breakers(self, message, model, message_event):
        """Reset the breakers.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        power_type = PowerType(message["powerType"])
        command_success = model.reset_breakers(power_type)

        # If success, simulate the update of digital output
        if command_success:
            digital_output_default = model.get_digital_output()

            digital_output_reset = digital_output_default
            if power_type == PowerType.Motor:
                digital_output_reset -= DigitalOutput.ResetMotorBreakers.value
            elif power_type == PowerType.Communication:
                digital_output_reset -= DigitalOutput.ResetCommunicationBreakers.value

            await message_event.write_digital_output(digital_output_reset)

            # Sleep a short time to simulate the reset process
            await asyncio.sleep(self.SLEEP_TIME_NORMAL)

            await message_event.write_digital_output(digital_output_default)

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def reboot_controller(self, message, model, message_event):
        """Reboot the cell controller.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        return model, CommandStatus.Success

    async def enable_open_loop_max_limit(self, message, model, message_event):
        """Enable the maximum limit in open-loop control.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        command_success = model.enable_open_loop_max_limit(True)

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def save_mirror_position(self, message, model, message_event):
        """Save the position of mirror.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        return model, CommandStatus.Success

    async def set_mirror_home(self, message, model, message_event):
        """Set the home of mirror.

        Parameters
        ----------
        message : `dict`
            Command message.
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.

        Returns
        -------
        model : `MockModel`
            Mock model to simulate the M2 hardware behavior.
        `CommandStatus`
            Status of command execution.
        """

        model.mirror_position = model.get_default_mirror_position()

        return model, CommandStatus.Success
