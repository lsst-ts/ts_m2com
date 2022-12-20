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
    ActuatorDisplacementUnit,
    ClosedLoopControlMode,
    CommandActuator,
    CommandScript,
    CommandStatus,
    DetailedState,
    DigitalOutput,
    InnerLoopControlMode,
    PowerSystemState,
    PowerType,
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

        self._digital_output = 0

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

        # If there is no communication power, fail the command.
        if not model.power_communication.is_power_on():
            return model, CommandStatus.Fail

        await self._power_on_fully(PowerType.Motor, model.power_motor, message_event)

        # Only turn on the force balance system when the CSC is the commander.
        # Need to unify the behaviors of CSC and EUI after the cell controller
        # in cRIO is fixed (there are two state machines at this moment).
        command_success = True
        if self._is_csc:
            command_success = model.switch_force_balance_system(True)

            if not command_success:
                await self._power_off_fully(
                    PowerType.Motor, model.power_motor, message_event
                )

            await message_event.write_m2_assembly_in_position(False)

        await message_event.write_force_balance_system_status(
            model.control_closed_loop.is_running
        )

        await message_event.write_open_loop_max_limit(
            model.control_open_loop.open_loop_max_limit_is_enabled
        )

        if command_success:
            await message_event.write_summary_state(salobj.State.ENABLED)
            await self._report_digital_input_and_ouput(model, message_event)

        return (
            model,
            CommandStatus.Success if command_success is True else CommandStatus.Fail,
        )

    async def _power_on_fully(self, power_type, power_system, message_event):
        """Fully power on the system.

        Parameters
        ----------
        power_type : enum `PowerType`
            Power type.
        power_system : `MockPowerSystem`
            Power system.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        await power_system.power_on()
        await message_event.write_power_system_state(
            power_type, power_system.is_power_on(), power_system.state
        )

        await power_system.wait_power_fully_on()
        await message_event.write_power_system_state(
            power_type, power_system.is_power_on(), power_system.state
        )

    async def _power_off_fully(self, power_type, power_system, message_event):
        """Fully power off the system.

        Parameters
        ----------
        power_type : enum `PowerType`
            Power type.
        power_system : `MockPowerSystem`
            Power system.
        message_event : `MockMessageEvent`
            Instance of MockMessageEvent to write the event.
        """

        await power_system.power_off()
        await message_event.write_power_system_state(
            power_type, power_system.is_power_on(), power_system.state
        )

        await power_system.wait_power_fully_off()
        await message_event.write_power_system_state(
            power_type, power_system.is_power_on(), power_system.state
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

        self._digital_output = model.get_digital_output()
        await message_event.write_digital_output(self._digital_output)

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
        await self._power_off_fully(PowerType.Motor, model.power_motor, message_event)

        await message_event.write_force_balance_system_status(
            model.control_closed_loop.is_running
        )

        await message_event.write_summary_state(salobj.State.DISABLED)

        await self._report_digital_input_and_ouput(model, message_event)

        await message_event.write_open_loop_max_limit(
            model.control_open_loop.open_loop_max_limit_is_enabled
        )

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

        await self._power_off_fully(PowerType.Motor, model.power_motor, message_event)
        await self._power_off_fully(
            PowerType.Communication, model.power_communication, message_event
        )

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

        await self._power_on_fully(
            PowerType.Communication, model.power_communication, message_event
        )

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

            await self._power_off_fully(
                PowerType.Motor, model.power_motor, message_event
            )
            await self._power_off_fully(
                PowerType.Communication, model.power_communication, message_event
            )

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
            model.control_closed_loop.apply_forces(message["axial"], message["tangent"])
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

        model.control_closed_loop.reset_force_offsets()
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
            model.control_closed_loop.is_running
        )

        await message_event.write_open_loop_max_limit(
            model.control_open_loop.open_loop_max_limit_is_enabled
        )

        # Publish the closed-loop control mode
        if model.control_closed_loop.is_running:
            await message_event.write_closed_loop_control_mode(
                ClosedLoopControlMode.ClosedLoop
            )
        else:
            await message_event.write_closed_loop_control_mode(
                ClosedLoopControlMode.OpenLoop
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

        if len(message["ring"]) == len(model.control_closed_loop.temperature["ref"]):
            model.control_closed_loop.temperature["ref"] = message["ring"]
            await message_event.write_temperature_offset(message["ring"])

            command_status = CommandStatus.Success
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

        try:
            power_type = PowerType(message["powerType"])
        except ValueError:
            return model, CommandStatus.Fail

        command_success = model.reset_breakers(power_type)

        digital_input_default = model.get_digital_input()
        power_system = (
            model.power_motor
            if power_type == PowerType.Motor
            else model.power_communication
        )

        # If success, simulate the update of digital output and events of power
        # system state
        if command_success:
            digital_output_default = model.get_digital_output()

            digital_output_reset = digital_output_default
            digital_input_reset = digital_input_default

            if power_type == PowerType.Motor:
                digital_output_reset -= DigitalOutput.ResetMotorBreakers.value
                digital_input_reset += sum(
                    [item.value for item in model.digital_input_motor]
                )

            elif power_type == PowerType.Communication:
                digital_output_reset -= DigitalOutput.ResetCommunicationBreakers.value
                digital_input_reset += sum(
                    [item.value for item in model.digital_input_communication]
                )

            await message_event.write_digital_output(digital_output_reset)
            await message_event.write_digital_input(digital_input_reset)
            await message_event.write_power_system_state(
                power_type,
                power_system.is_power_on(),
                PowerSystemState.ResettingBreakers,
            )

            # Sleep a short time to simulate the reset process
            await asyncio.sleep(self.SLEEP_TIME_NORMAL)

            await message_event.write_digital_output(digital_output_default)
            await message_event.write_power_system_state(
                power_type, power_system.is_power_on(), power_system.state
            )

        # Report the digital input
        await message_event.write_digital_input(digital_input_default)

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

        command_success = model.enable_open_loop_max_limit(message["status"])

        await message_event.write_open_loop_max_limit(
            model.control_open_loop.open_loop_max_limit_is_enabled
        )

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

    async def switch_digital_output(self, message, model, message_event):
        """Switch the digital output.

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

        # Get the switched bit
        try:
            bit = DigitalOutput(message["bit"])
        except ValueError:
            return model, CommandStatus.Fail

        self._digital_output = model.switch_digital_output(self._digital_output, bit)
        await message_event.write_digital_output(self._digital_output)

        # Turn on/off the power based on the bit value
        if self._digital_output & DigitalOutput.CommunicationPower.value:
            await self._power_on_fully(
                PowerType.Communication, model.power_communication, message_event
            )
        else:
            await self._power_off_fully(
                PowerType.Communication, model.power_communication, message_event
            )

        if self._digital_output & DigitalOutput.MotorPower.value:
            await self._power_on_fully(
                PowerType.Motor, model.power_motor, message_event
            )
        else:
            await self._power_off_fully(
                PowerType.Motor, model.power_motor, message_event
            )

        return model, CommandStatus.Success

    async def power(self, message, model, message_event):
        """Power on/off the motor/communication system.

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

        # Get the power type
        try:
            power_type = PowerType(message["powerType"])
        except ValueError:
            return model, CommandStatus.Fail

        power_system = (
            model.power_motor
            if power_type == PowerType.Motor
            else model.power_communication
        )

        if message["status"] is True:
            await self._power_on_fully(power_type, power_system, message_event)
        else:
            await self._power_off_fully(power_type, power_system, message_event)

        await self._report_digital_input_and_ouput(model, message_event)

        if (
            not model.power_motor.is_power_on()
        ) and model.control_closed_loop.is_running:
            model.switch_force_balance_system(False)
            await message_event.write_force_balance_system_status(
                model.control_closed_loop.is_running
            )

        await message_event.write_open_loop_max_limit(
            model.control_open_loop.open_loop_max_limit_is_enabled
        )

        return model, CommandStatus.Success

    async def reset_actuator_steps(self, message, model, message_event):
        """Reset the actuator steps.

        Notes
        -----
        This will reset the internal state of actuator steps in the cell
        controller. No intention to simulate this in MockControlOpenLoop class,
        which will make it too complex.

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

    async def set_closed_loop_control_mode(self, message, model, message_event):
        """Set the closed-loop control mode.

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

        # Get the mode
        try:
            mode = ClosedLoopControlMode(message["mode"])
        except ValueError:
            return model, CommandStatus.Fail

        await message_event.write_closed_loop_control_mode(mode)

        return model, CommandStatus.Success

    async def set_inner_loop_control_mode(self, message, model, message_event):
        """Set the inner-loop control mode.

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

        # Fail the command if the motor power is not on
        if not model.power_motor.is_power_on():
            return model, CommandStatus.Fail

        # Note the addresses are 0-based
        try:
            addresses = message["addresses"]
            mode = InnerLoopControlMode(message["mode"])

            model.set_mode_ilc(addresses, mode)
        except (IndexError, ValueError):
            return model, CommandStatus.Fail

        for address in addresses:
            await message_event.write_inner_loop_control_mode(
                address, model.list_ilc[address].mode
            )

        return model, CommandStatus.Success

    async def get_inner_loop_control_mode(self, message, model, message_event):
        """get the inner-loop control mode.

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

        # Fail the command if the motor power is not on
        if not model.power_motor.is_power_on():
            return model, CommandStatus.Fail

        # Note the addresses are 0-based
        try:
            addresses = message["addresses"]
            list_mode = model.get_mode_ilc(addresses)
        except IndexError:
            return model, CommandStatus.Fail

        for address, mode in zip(addresses, list_mode):
            await message_event.write_inner_loop_control_mode(address, mode)

        return model, CommandStatus.Success
