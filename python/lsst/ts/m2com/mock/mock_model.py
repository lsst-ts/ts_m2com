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

import logging
import typing
from pathlib import Path

import numpy as np
import numpy.typing
from lsst.ts.idl.enums import MTM2

from ..constant import (
    MIRROR_WEIGHT_KG,
    NUM_ACTUATOR,
    NUM_INNER_LOOP_CONTROLLER,
    NUM_TANGENT_LINK,
)
from ..enum import (
    DigitalInput,
    DigitalOutput,
    InnerLoopControlMode,
    MockErrorCode,
    PowerType,
)
from ..utility import read_yaml_file
from . import (
    MockControlClosedLoop,
    MockControlOpenLoop,
    MockErrorHandler,
    MockInnerLoopController,
    MockPowerSystem,
    MockScriptEngine,
)

__all__ = ["MockModel"]


class MockModel:
    """Mock Model class.

    Parameters
    ----------
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    inclinometer_angle : `float`, optional
        Inclinometer angle in degree. (the default is 90.0)
    telemetry_interval : `float`, optional
        Telemetry interval in second. (the default is 0.05, which means
        20 Hz)
    communication_voltage : `float`, optional
        Communication voltage in volt if the power is on. (the default is 23.0)
    communication_current : `float`, optional
        Communication current in ampere if the power is on. (the default is
        6.5)
    motor_voltage : `float`, optional
        Motor voltage in volt if the power is on. (the default is 23.0)
    motor_current : `float`, optional
        Motor current in ampere if the power is on. (the default is 8.5)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    control_open_loop : `MockControlOpenLoop`
        Open-loop control.
    control_closed_loop : `MockControlClosedLoop`
        Closed-loop control.
    telemetry_interval : `float`
        Telemetry interval in second.
    inclination_source : `MTM2.InclinationTelemetrySource`
        Source of the inclination.
    mirror_position : `dict`
        Mirror's position. The key is the axis (x, y, z, xRot, yRot, zRot) of
        mirror. The units are the micron and arcsec.
    mirror_position_offset : `dict`
        Offset value of the mirror position. The key is the axis (x, y, z,
        xRot, yRot, zRot) of mirror. The units are the micron and arcsec. This
        is a workaround to deal with the rigid body movement at this moment.
        Need to remove this after the translation of forward modeling of
        hardpoint correction.
    power_communication : `MockPowerSystem`
        Power system of communication.
    power_motor : `MockPowerSystem`
        Power system of motor.
    in_position : `bool`
        M2 assembly is in position or not.
    list_ilc : `list [MockInnerLoopController]`
        List of the inner-loop controllers (ILC).
    mtmount_in_position : `bool`
        MTMount in position or not.
    script_engine : `MockScriptEngine`
        Script engine to run the binary script.
    error_handler : `MockErrorHandler`
        Error handler to manage the error codes.
    """

    def __init__(
        self,
        log: logging.Logger | None = None,
        inclinometer_angle: float = 90.0,
        telemetry_interval: float = 0.05,
        communication_voltage: float = 23.0,
        communication_current: float = 6.5,
        motor_voltage: float = 23.0,
        motor_current: float = 8.5,
    ) -> None:
        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.control_open_loop = MockControlOpenLoop()
        self.control_open_loop.inclinometer_angle = inclinometer_angle

        self.control_closed_loop: MockControlClosedLoop = MockControlClosedLoop()

        self.telemetry_interval = telemetry_interval

        self.inclination_source: MTM2.InclinationTelemetrySource = (
            MTM2.InclinationTelemetrySource.ONBOARD
        )

        self.mirror_position = self.get_default_mirror_position()
        self.mirror_position_offset = self.get_default_mirror_position()

        self.power_communication: MockPowerSystem = MockPowerSystem(
            communication_voltage, communication_current
        )
        self.power_motor: MockPowerSystem = MockPowerSystem(
            motor_voltage, motor_current
        )

        self.in_position: bool = False

        # Generator to simulate the signal of ILC status
        self._ilc_status = self._uniq_ilc_status_generator()

        self.list_ilc = list()
        for idx in range(NUM_INNER_LOOP_CONTROLLER):
            self.list_ilc.append(MockInnerLoopController())

        # Parameters of independent displacement sensors (IMS)
        self._disp_ims: typing.Dict = dict()

        self.mtmount_in_position: bool = False

        self.script_engine: MockScriptEngine = MockScriptEngine()
        self.error_handler: MockErrorHandler = MockErrorHandler()

        self._set_default_measured_forces()

    def get_default_mirror_position(self) -> dict:
        """Get the default mirror position.

        Returns
        -------
        `dict`
            Default mirror position. The key is axis.
        """
        return dict([(axis, 0.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")])

    def _uniq_ilc_status_generator(self) -> typing.Generator:
        """Unique inner-loop controller (ILC) status generator.

        The details are in code 67, LTS-346.

        Yields
        ------
        `Generator`
            Unique generator object.
        """

        value = 0
        while True:
            yield value % 16
            value += 1

    def set_inclinometer_angle(self, angle: float) -> None:
        """Set the angle of inclinometer.

        Parameters
        ----------
        angle : `float`
            Inclinometer angle in degree.
        """

        self.control_open_loop.inclinometer_angle = angle

        # In the real hardware, the hardpoint correction based on the look-up
        # table (LUT) should be udpated continuously. But this will make the
        # measured force hard to converge in the closed-loop control for the
        # simplified calculation in the
        # self.control_closed_loop.calc_look_up_forces(). Therefore, we only
        # update it when the "angle" is changed.
        lut_angle = self.control_open_loop.correct_inclinometer_angle(angle)
        self.control_closed_loop.calc_look_up_forces(lut_angle)

        # The change of elevation angle means the hardpoints may need to update
        # the force as well.
        self.control_closed_loop.in_position_hardpoints = False

    def _set_default_measured_forces(self) -> None:
        """Set the default measured forces."""

        forces = self.control_open_loop.get_forces_mirror_weight(
            self.control_open_loop.inclinometer_angle
        )

        self.control_closed_loop.set_measured_forces(
            forces[: (NUM_ACTUATOR - NUM_TANGENT_LINK)], forces[-NUM_TANGENT_LINK:]
        )

    @property
    def position_limit_radial(self) -> float:
        """Mirror radial motion limit (in mm)."""
        return 7.74

    @property
    def position_limit_z(self) -> float:
        """Mirror motion limit in the optical direction (in mm)."""
        return 7.89

    @property
    def digital_output_default(self) -> typing.Iterable[DigitalOutput]:
        """Default digital output defined with the enum 'DigitalOutput'."""
        return (
            DigitalOutput.InterlockEnable,
            DigitalOutput.ResetMotorBreakers,
            DigitalOutput.ResetCommunicationBreakers,
        )

    @property
    def digital_input_default(self) -> typing.Iterable[DigitalInput]:
        """Default digital input defined with the enum 'DigitalInput'."""
        return (
            DigitalInput.RedundancyOK,
            DigitalInput.LoadDistributionOK,
            DigitalInput.PowerSupplyDC_2_OK,
            DigitalInput.PowerSupplyDC_1_OK,
            DigitalInput.PowerSupplyCurrent_2_OK,
            DigitalInput.PowerSupplyCurrent_1_OK,
            DigitalInput.J1_W9_1_MotorPowerBreaker,
            DigitalInput.J1_W9_2_MotorPowerBreaker,
            DigitalInput.J1_W9_3_MotorPowerBreaker,
            DigitalInput.J2_W10_1_MotorPowerBreaker,
            DigitalInput.J2_W10_2_MotorPowerBreaker,
            DigitalInput.J2_W10_3_MotorPowerBreaker,
            DigitalInput.J3_W11_1_MotorPowerBreaker,
            DigitalInput.J3_W11_2_MotorPowerBreaker,
            DigitalInput.J3_W11_3_MotorPowerBreaker,
            DigitalInput.J1_W12_1_CommunicationPowerBreaker,
            DigitalInput.J1_W12_2_CommunicationPowerBreaker,
            DigitalInput.J2_W13_1_CommunicationPowerBreaker,
            DigitalInput.J2_W13_2_CommunicationPowerBreaker,
            DigitalInput.J3_W14_1_CommunicationPowerBreaker,
            DigitalInput.J3_W14_2_CommunicationPowerBreaker,
            DigitalInput.InterlockPowerReplay,
        )

    @property
    def digital_input_communication(self) -> typing.Iterable[DigitalInput]:
        """Updated digital input when the communication power is on."""
        return (
            DigitalInput.J1_W12_1_CommunicationPowerBreaker,
            DigitalInput.J1_W12_2_CommunicationPowerBreaker,
            DigitalInput.J2_W13_1_CommunicationPowerBreaker,
            DigitalInput.J2_W13_2_CommunicationPowerBreaker,
            DigitalInput.J3_W14_1_CommunicationPowerBreaker,
            DigitalInput.J3_W14_2_CommunicationPowerBreaker,
        )

    @property
    def digital_input_motor(self) -> typing.Iterable[DigitalInput]:
        """Updated digital input when the motor power is on."""
        return (
            DigitalInput.J1_W9_1_MotorPowerBreaker,
            DigitalInput.J1_W9_2_MotorPowerBreaker,
            DigitalInput.J1_W9_3_MotorPowerBreaker,
            DigitalInput.J2_W10_1_MotorPowerBreaker,
            DigitalInput.J2_W10_2_MotorPowerBreaker,
            DigitalInput.J2_W10_3_MotorPowerBreaker,
            DigitalInput.J3_W11_1_MotorPowerBreaker,
            DigitalInput.J3_W11_2_MotorPowerBreaker,
            DigitalInput.J3_W11_3_MotorPowerBreaker,
            DigitalInput.InterlockPowerReplay,
        )

    def configure(self, config_dir: Path, lut_path: str) -> None:
        """Do the configuration.

        Parameters
        ----------
        config_dir : `pathlib.PosixPath`
            Configuration directory.
        lut_path : `str`
            Look-up table (LUT) path.
        """

        path_lut = config_dir / lut_path
        self.control_closed_loop.load_file_lut(path_lut)

        path_cell_geom = config_dir / lut_path / "cell_geom.yaml"
        self.control_closed_loop.load_file_cell_geometry(path_cell_geom)

        path_disp_ims = config_dir / lut_path / "disp_ims.yaml"
        self._disp_ims = read_yaml_file(path_disp_ims)

        # Read the static transfer matrix
        path_static_transfer_matrix = config_dir / lut_path / "StaticTransferMatrix.csv"
        self.control_open_loop.read_file_static_transfer_matrix(
            path_static_transfer_matrix
        )

        self.control_closed_loop.set_hardpoint_compensation()

        # By doing this, we can calculate the forces of look-up table.
        self.set_inclinometer_angle(self.control_open_loop.inclinometer_angle)

        # Read the error code file
        self.error_handler.read_error_list_file(config_dir / "error_code.tsv")

    def is_actuator_force_out_limit(
        self,
    ) -> typing.Tuple[bool, MockErrorCode, list, list]:
        """The actuator force is out of limit or not.

        By default, return the judgement based on the open-loop control. If
        the closed-loop control is running, return the result based on it.

        Returns
        -------
        `bool`
            True if the actuator force is out of limit. Otherwise, False.
        enum `MockErrorCode`
            Error code.
        `list`
            Triggered retracted limit switches.
        `list`
            Triggered extended limit switches.
        """

        error_code = MockErrorCode.NoError
        if self.control_closed_loop.is_running:
            (
                is_out_limit,
                limit_switches_retract,
                limit_switches_extend,
            ) = self.control_closed_loop.is_actuator_force_out_limit(
                use_measured_force=True
            )

            if is_out_limit:
                error_code = MockErrorCode.LimitSwitchTriggeredClosedloop

        else:
            (
                is_out_limit,
                limit_switches_retract,
                limit_switches_extend,
            ) = self.control_open_loop.is_actuator_force_out_limit()

            if is_out_limit:
                error_code = MockErrorCode.LimitSwitchTriggeredOpenloop

        return is_out_limit, error_code, limit_switches_retract, limit_switches_extend

    def fault(self, error_code: MockErrorCode) -> None:
        """Fault the model.

        Parameters
        ----------
        error_code : enum `MockErrorCode`
            Error code.
        """

        self.error_handler.add_new_error(int(error_code))

        self.script_engine.stop()
        self.control_open_loop.stop()

        self.switch_force_balance_system(False)
        self.control_closed_loop.reset_force_offsets()

    def switch_force_balance_system(self, status: bool) -> bool:
        """Switch the force balance system.

        Parameters
        ----------
        status : `bool`
            True if turn on the force balance system. Otherwise, False.

        Returns
        -------
        result : `bool`
            True if succeeds. Otherwise, False.
        """

        # In closed-loop control, the maximum limits of open-loop control
        # should be disabled.
        if (status is True) and self.control_open_loop.open_loop_max_limit_is_enabled:
            return False

        # Run the closed-loop control
        result = True

        if (status is True) and (not self.power_motor.is_power_on()):
            self.control_closed_loop.is_running = False
            result = False
        else:
            self.control_closed_loop.is_running = status

        # The system cannot run the closed-loop and open-loop at the same time.
        if self.control_closed_loop.is_running:
            self.control_open_loop.is_running = False

        return result

    def clear_errors(self) -> None:
        """Clear the errors."""
        self.error_handler.clear()

    def select_inclination_source(self, source: int) -> None:
        """Select the inclination source.

        Parameters
        ----------
        source : `int`
            Inclination source based on the enum:
            MTM2.InclinationTelemetrySource.
        """

        self.inclination_source = MTM2.InclinationTelemetrySource(int(source))

    def get_telemetry_data(self) -> dict:
        """Get the telemetry data.

        Returns
        -------
        telemetry_data : `dict`
            Telemetry data.
        """

        telemetry_data = dict()

        position_ims = None
        if self.power_motor.is_power_on():
            telemetry_data["ilcData"] = self._get_ilc_data()
            telemetry_data[
                "netForcesTotal"
            ] = self.control_closed_loop.get_net_forces_total()
            telemetry_data[
                "netMomentsTotal"
            ] = self.control_closed_loop.get_net_moments_total()

            # Get the force data
            telemetry_data["axialForce"] = self.control_closed_loop.axial_forces
            telemetry_data["tangentForce"] = self.control_closed_loop.tangent_forces
            telemetry_data["forceErrorTangent"] = self._calculate_force_error_tangent(
                self.control_closed_loop.tangent_forces["measured"]
            )
            telemetry_data[
                "forceBalance"
            ] = self.control_closed_loop.get_force_balance()

            # Get the position data
            telemetry_data["position"] = self._get_mirror_position_with_offset()

            position_ims = self._simulate_position_mirror()
            telemetry_data["positionIMS"] = position_ims

            # Get the temperature data
            self.control_closed_loop.simulate_temperature_and_update()
            telemetry_data["temperature"] = self.control_closed_loop.temperature

            # Get the zenith angle
            telemetry_data["zenithAngle"] = self._simulate_zenith_angle()

            # Get the actuator steps and positions
            steps = self.control_open_loop.actuator_steps
            positions = self.control_open_loop.get_actuator_positions()

            num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK

            telemetry_data["axialActuatorSteps"] = {
                "steps": steps[:num_axial_actuators].tolist()
            }
            telemetry_data["axialEncoderPositions"] = {
                "position": positions[:num_axial_actuators].tolist()
            }

            telemetry_data["tangentActuatorSteps"] = {
                "steps": steps[-NUM_TANGENT_LINK:].tolist()
            }
            telemetry_data["tangentEncoderPositions"] = {
                "position": positions[-NUM_TANGENT_LINK:].tolist()
            }

        telemetry_data["powerStatus"] = self._get_power_status()
        telemetry_data["powerStatusRaw"] = self._get_power_status()

        telemetry_data["inclinometerAngleTma"] = self._simulate_zenith_angle()

        telemetry_data["displacementSensors"] = self._get_displacement_sensors(
            mirror_position=position_ims
        )

        return telemetry_data

    def _get_power_status(self, rms: float = 0.05) -> dict:
        """Get the power status.

        Parameters
        ----------
        rms : `float`, optional
            RMS variation. (the default is 0.05)

        Returns
        -------
        `dict`
            Power status.
        """

        comm_voltage_update, comm_current_update = self.power_communication.get_power(
            rms=rms
        )
        motor_voltage_update, motor_current_update = self.power_motor.get_power(rms=rms)

        return {
            "motorVoltage": motor_voltage_update,
            "motorCurrent": motor_current_update,
            "commVoltage": comm_voltage_update,
            "commCurrent": comm_current_update,
        }

    def _get_ilc_data(self) -> dict:
        """Get the inner-loop controller (ILC) data.

        Returns
        -------
        `dict`
            Data of ILC data.
        """

        return {"status": [next(self._ilc_status)] * NUM_ACTUATOR}

    def _calculate_force_error_tangent(
        self, tangent_force_current: numpy.typing.NDArray[np.float64]
    ) -> dict:
        """Calculate the tangent force error.

        This function is translated from TangentLoadCellFaultDetection.vi in
        ts_mtm2 LabVIEW project.

        Parameters
        ----------
        tangent_force_current : `numpy.ndarray`
            Currant tangent force (A1-A6) in Newton.

        Returns
        -------
        `dict`
            Data of tangent force error.
        """

        # Calculate the individual tangent force error

        # When the mirror is on tilt orientation, tangential forces (A2, A3,
        # A5, A6) have to compensate gravity force of mirror.
        indexes = [1, 2, 4, 5]

        # The orientation of tangent links is hexagon. Therefore, the
        # projection angle is 30 degree.
        projection_angle = 30
        tangent_force_support = tangent_force_current[indexes] * np.cos(
            np.deg2rad(projection_angle)
        )

        gravitational_acceleration = 9.8
        mirror_weight_projection = (
            MIRROR_WEIGHT_KG
            * gravitational_acceleration
            * np.sin(np.deg2rad(90 - self.control_open_loop.inclinometer_angle))
        )
        divided_mirror_division = mirror_weight_projection / len(indexes)

        direction = np.array([1, 1, -1, -1])
        weight_error = tangent_force_support + direction * divided_mirror_division

        force_error_tangent = [
            tangent_force_current[0],
            weight_error[0],
            weight_error[1],
            tangent_force_current[3],
            weight_error[2],
            weight_error[3],
        ]

        # Calculate the total weight error
        total_weight_error = (
            -tangent_force_support[0]
            - tangent_force_support[1]
            + tangent_force_support[2]
            + tangent_force_support[3]
            - mirror_weight_projection
        )

        return {
            "force": force_error_tangent,
            "weight": total_weight_error,
            "sum": sum(tangent_force_current),
        }

    def _get_displacement_sensors(self, mirror_position: dict | None = None) -> dict:
        """Get the displacement sensors.

        Parameters
        ----------
        mirror_position : `dict` or None, optional
            Position of mirror. (the default is None)

        Returns
        -------
        `dict`
            Data of displacement sensors.
        """

        if mirror_position is None:
            mirror_position = self.mirror_position

        matrix_inv = np.linalg.pinv(self._disp_ims["matrix"])
        position = np.array(list(mirror_position.values()))

        # The number of solutions is infinite. But we do not care.
        output = matrix_inv.dot(position.reshape(-1, 1))

        # Change the unit from mm to um
        reading_ims = (output.ravel() + np.array(self._disp_ims["offset"])) * 1e3

        theta_z = reading_ims[np.array([8, 10, 4, 6, 0, 2])]
        delta_z = reading_ims[np.array([9, 11, 5, 7, 1, 3])]

        return {"thetaZ": theta_z, "deltaZ": delta_z}

    def balance_forces_and_steps(
        self,
        force_rms: float = 0.5,
        force_per_cycle: float = 5.0,
        update_steps: bool = True,
    ) -> bool:
        """Balance the forces and steps.

        This function will check the actuators are in position or not and
        udpate the self.in_position. If the closed-loop control is running, it
        will do the force dynamics based on the "force_per_cycle" to try to
        balance the forces of actuators. In addition, it will try to make sure
        the internal data in open-loop and closed-loop controls are consistent.

        Parameters
        ----------
        force_rms : `float`, optional
            Force rms variation in Newton. (default is 0.5)
        force_per_cycle : `float`, optional
            Force per cycle to apply in Newton. (the default is 5.0)
        update_steps : `bool`, optional
            If True, update the steps based on the force change. Otherwise, no
            update. This takes a significant CPU usage (np.linalg.inv()). (the
            default is True)

        Returns
        -------
        `bool`
            True if the steps had been updated according to the force change.
            Otherwise, False.

        Raises
        ------
        `RuntimeError`
            When the open-loop and closed-loop controls are running at the same
            time.
        """

        if self.control_open_loop.is_running and self.control_closed_loop.is_running:
            raise RuntimeError(
                "The open-loop and closed-loop controls are running at the same time."
            )

        self.in_position = self.control_closed_loop.handle_forces(
            force_rms=force_rms,
            force_per_cycle=force_per_cycle,
        )

        # In the open-loop control, update the measured force
        if self.control_open_loop.is_running:
            forces = self.control_open_loop.calculate_steps_to_forces(
                self.control_open_loop.actuator_steps
            )
            self.control_closed_loop.set_measured_forces(
                forces[: (NUM_ACTUATOR - NUM_TANGENT_LINK)], forces[-NUM_TANGENT_LINK:]
            )

        # In the closed-loop control, update the steps and rigid body position
        # because of the udpated measured forces.
        if self.control_closed_loop.is_running and update_steps:
            # Calculate the steps
            forces = np.append(
                self.control_closed_loop.axial_forces["measured"],
                self.control_closed_loop.tangent_forces["measured"],
            )
            steps = self.control_open_loop.calculate_forces_to_steps(forces)
            self.control_open_loop.update_actuator_steps(steps)

            # Calculate the rigid body position
            x, y, z, rx, ry, rz = MockControlClosedLoop.hardpoint_to_rigid_body(
                self.control_closed_loop.get_actuator_location_axial(),
                self.control_closed_loop.get_actuator_location_tangent(),
                self.control_closed_loop.get_radius(),
                self.control_closed_loop.hardpoints,
                self.get_current_hardpoint_displacement().tolist(),
                self.control_closed_loop.disp_hardpoint_home,
            )

            # Need to update the unit from "m to um" and "rad to arcsec".
            M_TO_UM = 1e6

            # 1 rad × (180/pi) x 3600 arcsec ~ 206265
            RADIAN_TO_ARCSEC = 206265

            self.mirror_position["x"] = x * M_TO_UM
            self.mirror_position["y"] = y * M_TO_UM
            self.mirror_position["z"] = z * M_TO_UM
            self.mirror_position["xRot"] = rx * RADIAN_TO_ARCSEC
            self.mirror_position["yRot"] = ry * RADIAN_TO_ARCSEC
            self.mirror_position["zRot"] = rz * RADIAN_TO_ARCSEC

            return True

        return False

    def get_current_hardpoint_displacement(self) -> numpy.typing.NDArray[np.float64]:
        """Get the current hardpoint displacements in meter.

        Returns
        -------
        `numpy.ndarray`
            Current hardpoint displacements in meter.
        """
        # Change the unit from millimeter to meter.
        return (
            self.control_open_loop.get_actuator_positions()[
                self.control_closed_loop.hardpoints
            ]
            * 1e-3
        )

    def _get_mirror_position_with_offset(self) -> dict:
        """Get the mirror position with the offset.

        Notes
        -----
        Remove this function after the translation of forward modeling of
        hardpoint correction.

        Returns
        -------
        position : `dict`
            Mirror's position with the offset. The key is the axis (x, y, z,
            xRot, yRot, zRot) of mirror. The units are the micron and arcsec.
        """

        position = dict()
        for axis, value in self.mirror_position.items():
            position[axis] = value + self.mirror_position_offset[axis]

        return position

    def _simulate_position_mirror(
        self, position_rms: float = 0.005, angle_rms: float = 0.005
    ) -> dict:
        """Simulate the position of mirror.

        Parameters
        ----------
        position_rms : `float`, optional
            Position rms variation in microns. (default is 0.005)
        angle_rms : `float`, optional
            Angle rms variation in arcsec. (default is 0.005)

        Returns
        -------
        position : `dict`
            Mirror's position. The key is the axis (x, y, z, xRot, yRot, zRot)
            of mirror. The units are the micron and arcsec.
        """

        position = self._get_mirror_position_with_offset()

        for axis, value in position.items():
            if axis in ("x, y, z"):
                scale = position_rms
            else:
                scale = angle_rms

            position[axis] = value + np.random.normal(scale=scale)

        return position

    def _simulate_zenith_angle(self, inclinometer_rms: float = 0.05) -> dict:
        """Simulate the zenith angle data.

        Parameters
        ----------
        inclinometer_rms : `float`, optional
            Inclinometer rms variation in degree. (default is 0.05)

        Returns
        -------
        zenith_angle : `dict`
            Zenith angle data. The unit is degree.
        """

        inclinometer_value = (
            self.control_open_loop.inclinometer_angle
            + np.random.normal(scale=inclinometer_rms)
        )

        zenith_angle = dict()
        zenith_angle["measured"] = 90 - inclinometer_value
        zenith_angle["inclinometerRaw"] = inclinometer_value
        zenith_angle[
            "inclinometerProcessed"
        ] = self.control_open_loop.correct_inclinometer_angle(inclinometer_value)

        return zenith_angle

    def handle_position_mirror(self, mirror_position_set_point: dict) -> None:
        """Handle positioning the mirror.

        Notes
        -----
        This is just a placeholder at this moment. To do this correctly, need:

        1. Translate the calculation of forward modeling of hardpoint
        correction.

        2. Apply MockControlClosedLoop.rigid_body_to_actuator_displacement()
        and get the expected displacement of actuator.

        3. Put the displacement to the forward-modeling and get correct
        hardpoint correction in force.

        4. Put the hardpoint correction to
        MockControlClosedLoop.axial_forces["hardpointCorrection"] and
        MockControlClosedLoop.tangent_forces["hardpointCorrection"].

        Parameters
        ----------
        mirror_position_set_point : `dict`
            Dictionary with the same format as `self.mirror_position`.
        """

        for axis in self.mirror_position_offset:
            self.mirror_position_offset[axis] = mirror_position_set_point[axis]

    def check_set_point_position_mirror(self, mirror_position_set_point: dict) -> bool:
        """Check the set point of mirror's position.

        Parameters
        ----------
        mirror_position_set_point : `dict`
            Dictionary with the same format as `self.mirror_position`.

        Returns
        -------
        `bool`
            True if all check pass. Otherwise, False.
        """

        if (not self.power_motor.is_power_on()) or (
            self.control_closed_loop.is_running is False
        ):
            self.log.debug(
                "Motor power needs to be on and system is in closed-loop control."
            )
            return False

        # Check limits
        # This is a simplication of the radial position, considering that
        # zRot = 0

        # Note the "xRot" and "yRot" are in arcsec
        UM_TO_MM = 1e-3
        radial_position = np.sqrt(
            (
                mirror_position_set_point["x"]
                * UM_TO_MM
                * np.cos(np.radians(mirror_position_set_point["yRot"] / 3600.0))
            )
            ** 2.0
            + (
                mirror_position_set_point["y"]
                * UM_TO_MM
                * np.cos(np.radians(mirror_position_set_point["xRot"] / 3600.0))
            )
            ** 2.0
        )

        if radial_position > self.position_limit_radial:
            self.log.debug(
                f"Requested position outside of radial limits: {radial_position} "
                f"(limit: {self.position_limit_radial})"
            )
            return False

        # Another simplication of the limit in the optical direction.
        set_point_z = mirror_position_set_point["z"] * UM_TO_MM
        if set_point_z > self.position_limit_z:
            self.log.debug(
                f"Requested position out of limits: {set_point_z} "
                f"(limit: {self.position_limit_z})"
            )
            return False

        return True

    def enable_open_loop_max_limit(self, status: bool) -> bool:
        """Enable the maximum limit of open-loop control.

        Parameters
        ----------
        status : `bool`
            True if allow the maximum limit of open-loop control. Otherwise,
            False.

        Returns
        -------
        result : `bool`
            True if succeeds. Otherwise, False.
        """

        result = True

        if (status is True) and self.control_closed_loop.is_running:
            self.control_open_loop.open_loop_max_limit_is_enabled = False
            result = False
        else:
            self.control_open_loop.open_loop_max_limit_is_enabled = status

        return result

    def reset_breakers(self, power_type: PowerType) -> bool:
        """Reset the breakers.

        Parameters
        ----------
        power_type : `PowerType`
            Power type.

        Returns
        -------
        result : `bool`
            True if succeeds. Otherwise, False.
        """

        result = False

        if (power_type == PowerType.Motor) and self.power_motor.is_power_on():
            result = True
        elif (
            power_type == PowerType.Communication
        ) and self.power_communication.is_power_on():
            result = True

        return result

    def get_digital_output(self) -> int:
        """Get the value of digital output.

        Returns
        -------
        digital_output : `int`
            Value of the digital output.
        """

        digital_output = sum([item.value for item in self.digital_output_default])

        if self.power_communication.is_power_on():
            digital_output += DigitalOutput.CommunicationPower.value

        if self.power_motor.is_power_on():
            digital_output += DigitalOutput.MotorPower.value

        return digital_output

    def get_digital_input(self) -> int:
        """Get the value of digital input that represents the current state of
        the system.

        Returns
        -------
        digital_input : `int`
            Value of the digital input.
        """

        digital_input = sum([item.value for item in self.digital_input_default])

        if self.power_communication.is_power_on():
            digital_input -= sum(
                [item.value for item in self.digital_input_communication]
            )

        if self.power_motor.is_power_on():
            digital_input -= sum([item.value for item in self.digital_input_motor])

        return digital_input

    def switch_digital_output(self, digital_output: int, bit: DigitalOutput) -> int:
        """Switch the digital output with the specific bit.

        Parameters
        ----------
        digital_output : `int`
            Digital output.
        bit : enum `DigitalOutput`
            Bit to switch.

        Returns
        -------
        `int`
            Updated value of the digital output.
        """
        return (
            (digital_output - bit.value)
            if (digital_output & bit.value)
            else (digital_output + bit.value)
        )

    def set_mode_ilc(
        self, addresses: typing.List[int], mode: InnerLoopControlMode
    ) -> None:
        """Set the mode of inner-loop controller (ILC).

        Parameters
        ----------
        addresses : `list [int]`
            0-based addresses.
        mode : enum `InnerLoopControlMode`
            ILC mode.
        """

        for address in addresses:
            self.list_ilc[address].set_mode(mode)

    def get_mode_ilc(
        self, addresses: typing.List[int]
    ) -> typing.List[InnerLoopControlMode]:
        """Get the mode of inner-loop controller (ILC).

        Parameters
        ----------
        addresses : `list [int]`
            0-based addresses.

        Returns
        -------
        list_mode : `list [InnerLoopControlMode]`
            List of the ILC mode.
        """

        list_mode = list()
        for address in addresses:
            list_mode.append(self.list_ilc[address].mode)

        return list_mode
