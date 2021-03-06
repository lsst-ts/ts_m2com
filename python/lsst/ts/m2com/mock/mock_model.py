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
import copy

import numpy as np
import pandas as pd

from time import sleep

from lsst.ts.idl.enums import MTM2

from ..constant import NUM_ACTUATOR, NUM_TANGENT_LINK
from ..enum import PowerType, DigitalOutput, DigitalInput
from ..utility import read_yaml_file
from . import MockScriptEngine, MockControlOpenLoop

__all__ = ["MockModel"]


class MockModel:
    """Mock Model class.

    Parameters
    ----------
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    telemetry_interval : `float`, optional
        Telemetry interval in second. (the default is 0.05, which means
        20 Hz)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    control_open_loop : `MockControlOpenLoop`
        Open-loop control.
    telemetry_interval : `float`
        Telemetry interval in second.
    inclination_source : `MTM2.InclinationTelemetrySource`
        Source of the inclination.
    zenith_angle : `float`
        Zenith angle in degree.
    mirror_position : `dict`
        Mirror position. The key is the axis (x, y, z, xRot, yRot, zRot) of
        mirror. The units are the micron and arcsec.
    temperature : `dict`
        Temperature of mirror in degree C.
    axial_forces : `dict`
        Forces of the axial actuators in Newton.
    tangent_forces : `dict`
        Forces of the tangent actuators in Newton.
    force_balance : `dict`
        Force balance system contains the information of force and moment.
        The units are the Newton and Newton * meter.
    force_balance_system_status : `bool`
        Force balance system is on or off.
    communication_power_on : `bool`
        Communication power is on or not.
    motor_power_on : `bool`
        Motor power is on or not.
    in_position : `bool`
        M2 assembly is in position or not.
    lut : `dict`
        Look-up table (LUT).
    error_cleared : `bool`
        Error is cleared or not.
    mtmount_in_position : `bool`
        MTMount in position or not.
    script_engine : `MockScriptEngine`
        Script engine to run the binary script.
    """

    def __init__(self, log=None, telemetry_interval=0.05):

        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.control_open_loop = MockControlOpenLoop()

        self.telemetry_interval = telemetry_interval

        self.inclination_source = MTM2.InclinationTelemetrySource.ONBOARD

        self.zenith_angle = 0.0
        self.update_zenith_angle(0)

        self.mirror_position = self.get_default_mirror_position()

        self.temperature = self._get_default_temperatures()

        self.axial_forces = dict(
            [
                (item, np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK))
                for item in (
                    "lutGravity",
                    "lutTemperature",
                    "applied",
                    "measured",
                    "hardpointCorrection",
                )
            ]
        )
        self.tangent_forces = dict(
            [
                (item, np.zeros(NUM_TANGENT_LINK))
                for item in (
                    "lutGravity",
                    "applied",
                    "measured",
                    "hardpointCorrection",
                )
            ]
        )

        self._set_default_measured_forces()

        # No LUT temperature correction for tangent links
        self.tangent_forces["lutTemperature"] = np.array([])

        self.force_balance = dict(
            [(axis, 0) for axis in ("fx", "fy", "fz", "mx", "my", "mz")]
        )
        self.force_balance_system_status = False

        self.communication_power_on = False
        self.motor_power_on = False
        self.in_position = False

        # Generator to simulate the signal of ILC status
        self._ilc_status = self._uniq_ilc_status_generator()

        # Parameters of independent displacement sensors (IMS)
        self._disp_ims = dict()

        # Cell geometry used in the calculation of net total forces and moments
        self._cell_geom = dict()

        self.lut = dict()

        self.error_cleared = True

        self.mtmount_in_position = False

        self.script_engine = MockScriptEngine()

    def update_zenith_angle(self, angle):
        """Update the zenith angle.

        Parameters
        ----------
        angle : `float`
            Zenith angle in degree.
        """

        self.zenith_angle = angle
        self.control_open_loop.inclinometer_angle = 90 - angle

    def get_default_mirror_position(self):
        """Get the default mirror position.

        Returns
        -------
        `dict`
            Default mirror position. The key is axis.
        """
        return dict([(axis, 0.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")])

    def _get_default_temperatures(
        self, temperature_init=11.0, temperature_ref=21.0, max_difference=2.0
    ):
        """Get the default temperatures. The unit is degree C.

        Parameters
        ----------
        temperature_init : `float`, optional
            Initial temperature in degree C. (default is 11.0)
        temperature_ref : `float`, optional
            Reference temperature in degree C. (default is 21.0)
        max_difference : `float`, optional
           Maximum difference of temperature in degree C. (default is 2.0)

        Returns
        -------
        temperatures : `dict`
            Temperature sensor data. The key is sensor's position or reference.
        """

        temperatures = dict()
        temperatures["ring"] = [temperature_init] * 12
        temperatures["intake"] = [temperature_init] * 2
        temperatures["exhaust"] = [temperature_init] * 2
        temperatures["ref"] = temperature_ref
        temperatures["maxDiff"] = max_difference

        return temperatures

    def _set_default_measured_forces(self):
        """Set the default measured forces."""

        forces = self.control_open_loop.get_forces_mirror_weight(
            self.control_open_loop.inclinometer_angle
        )
        self.axial_forces["measured"] = forces[: (NUM_ACTUATOR - NUM_TANGENT_LINK)]
        self.tangent_forces["measured"] = forces[-NUM_TANGENT_LINK:]

    def _uniq_ilc_status_generator(self):
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

    @property
    def mirror_weight(self):
        """Mirror weight (in Newton)."""
        return 15578.0

    @property
    def max_axial_force(self):
        """Axial force limit (in Newtons).

        According with the vendor documents these limits are hard-coded in
        the real system, so they are also hard-coded in the simulator.
        """
        return 444.82

    @property
    def max_tangent_force(self):
        """Tangent force limit (in Newtons).

        According with the vendor documents these limits are hard-coded in
        the real system, so they are also hard-coded in the simulator.
        """
        return 4893.04

    @property
    def position_limit_radial(self):
        """Mirror radial motion limit (in mm)."""
        return 7.74

    @property
    def position_limit_z(self):
        """Mirror motion limit in the optical direction (in mm)."""
        return 7.89

    @property
    def digital_output_default(self):
        """Default digital output defined with the enum 'DigitalOutput'."""
        return (
            DigitalOutput.InterlockEnable,
            DigitalOutput.ResetMotorBreakers,
            DigitalOutput.ResetCommunicationBreakers,
        )

    @property
    def digital_input_default(self):
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
    def digital_input_communication(self):
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
    def digital_input_motor(self):
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

    def configure(self, config_dir, lut_path):
        """Do the configuration.

        Parameters
        ----------
        config_dir : `pathlib.PosixPath`
            Configuration directory.
        lut_path : `str`
            Look-up table (LUT) path.
        """

        # Look-up table (LUT) factors (filename, basically).
        lut_factors = set(
            [
                "F_0.csv",
                "F_A.csv",
                "F_E.csv",
                "F_F.csv",
                "Tr.csv",
                "Tu.csv",
                "Tx.csv",
                "Ty.csv",
                "temp_inv.txt",
            ]
        )

        for lut in lut_factors:
            self.log.debug(f"Reading {lut}...")
            name, ext = lut.split(".")
            if ext == "csv":
                data = pd.read_csv(
                    config_dir / lut_path / lut,
                    header="infer" if name.startswith("F") else None,
                )
                self.lut[name] = np.float64(data)
                if name == "F_E":
                    self.lut["lutInAngle"] = np.float64(data.keys())
            else:
                self.lut[name] = np.loadtxt(config_dir / lut_path / lut)

        # Read the yaml files
        path_disp_ims = config_dir / lut_path / "disp_ims.yaml"
        self._disp_ims = read_yaml_file(path_disp_ims)

        path_cell_geom = config_dir / lut_path / "cell_geom.yaml"
        self._cell_geom = read_yaml_file(path_cell_geom)

        # Read the static transfer matrix
        path_static_transfer_matrix = config_dir / lut_path / "StaticTransferMatrix.csv"
        self.control_open_loop.read_file_static_transfer_matrix(
            path_static_transfer_matrix
        )

    def is_cell_temperature_high(self):
        """Cell temperature is high or not.

        Returns
        -------
        `bool`
            True if the cell temperature is high. Otherwise, False.
        """

        temp_exhaust = self.temperature["exhaust"]
        temp_intake = self.temperature["intake"]

        diff = (temp_exhaust[0] - temp_intake[0] + temp_exhaust[1] - temp_intake[1]) / 2

        return True if diff > self.temperature["maxDiff"] else False

    def fault(self):
        """Fault the model."""

        self.error_cleared = False

        self.script_engine.stop()
        self.control_open_loop.stop()

        self.switch_force_balance_system(False)
        self.reset_force_offsets()

    def switch_force_balance_system(self, status):
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

        result = True

        if (status is True) and (not self.motor_power_on):
            self.force_balance_system_status = False
            result = False
        else:
            self.force_balance_system_status = status

        # In closed-loop control, the maximum limits of open-loop control is
        # disabled. In addition, the system can not run the closed-loop and
        # open-loop in the same time.
        if self.force_balance_system_status is True:
            self.enable_open_loop_max_limit(False)
            self.control_open_loop.is_running = False

        return result

    def apply_forces(self, force_axial, force_tangent):
        """Apply force.

        Parameters
        ----------
        force_axial : `list` or `numpy.ndarray`
            Force of the axial actuators in Newton.
        force_tangent : `list` or `numpy.ndarray`
            Force of the tangent actuators in Newton.
        """

        # Check limits
        self.check_axial_force_limit(apply=force_axial)
        self.check_tangent_force_limit(apply=force_tangent)

        self.axial_forces["applied"] = np.array(force_axial)
        self.tangent_forces["applied"] = np.array(force_tangent)

    def check_axial_force_limit(self, apply=None):
        """Check if axial forces are out of bounds.

        Parameters
        ----------
        apply : `numpy.ndarray` or `None`, optional
            Forces to apply in Newton. If `None` use current setup. (default is
            None.)

        Returns
        -------
        demanded_axial_force : `numpy.ndarray`
            Array with the combined total force per actuator in Newton.

        Raises
        ------
        RuntimeError
            If force limit is out of range.
        """

        demanded_axial_force = (
            (apply if apply is not None else self.axial_forces["applied"])
            + self.axial_forces["lutGravity"]
            + self.axial_forces["lutTemperature"]
            + self.axial_forces["hardpointCorrection"]
        )

        if np.any(demanded_axial_force > self.max_axial_force):
            actuators = np.where(demanded_axial_force > self.max_axial_force)[0]
            raise RuntimeError(
                f"Maximum axial force limit [{self.max_axial_force}N] reached in actuators {actuators}."
            )

        return demanded_axial_force

    def check_tangent_force_limit(self, apply=None):
        """Check if tangent forces are out of bounds.

        Parameters
        ----------
        apply : `numpy.ndarray` or `None`, optional
            Forces to apply in Newton. If `None` use current setup. (default is
            None.)

        Returns
        -------
        demanded_tanget_force : `numpy.ndarray`
            Array with the combined total force per actuator in Newton.

        Raises
        ------
        RuntimeError
            If force limit is out of range.
        """

        demanded_tanget_force = np.array(
            (apply if apply is not None else self.tangent_forces["applied"])
            + self.tangent_forces["lutGravity"]
            + self.tangent_forces["hardpointCorrection"]
        )

        if np.any(demanded_tanget_force > self.max_tangent_force):
            actuators = np.where(demanded_tanget_force > self.max_tangent_force)[0]
            raise RuntimeError(
                f"Maximum axial force limit [{self.max_tangent_force}N] reached in actuators {actuators}."
            )

        return demanded_tanget_force

    def reset_force_offsets(self):
        """Reset the force offsets."""

        self.axial_forces["applied"] = np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)
        self.tangent_forces["applied"] = np.zeros(NUM_TANGENT_LINK)

    def clear_errors(self):
        """Clear the errors."""

        self.error_cleared = True

    def select_inclination_source(self, source):
        """Select the inclination source.

        Parameters
        ----------
        source : `int`
            Inclination source based on the enum:
            MTM2.InclinationTelemetrySource.
        """

        self.inclination_source = MTM2.InclinationTelemetrySource(int(source))

    def get_telemetry_data(self):
        """Get the telemetry data.

        Returns
        -------
        telemetry_data : `dict`
            Telemetry data.
        """

        telemetry_data = dict()

        telemetry_data["powerStatus"] = self._get_power_status()

        position_ims = None
        if self.motor_power_on:

            telemetry_data["ilcData"] = self._get_ilc_data()
            telemetry_data["netForcesTotal"] = self._get_net_forces_total()
            telemetry_data["netMomentsTotal"] = self._get_net_moments_total()

            # Get the force data
            self.in_position = self.handle_forces()
            telemetry_data["axialForce"] = self.axial_forces
            telemetry_data["tangentForce"] = self.tangent_forces
            telemetry_data["forceBalance"] = self.force_balance

            # Get the position data
            telemetry_data["position"] = self._simulate_position_mirror()

            position_ims = self._simulate_position_mirror()
            telemetry_data["positionIMS"] = position_ims

            # Get the temperature data
            self._simulate_temperature_and_update()
            temperature = copy.copy(self.temperature)
            telemetry_data["temperature"] = temperature

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

        telemetry_data["displacementSensors"] = self._get_displacement_sensors(
            mirror_position=position_ims
        )

        return telemetry_data

    def _get_power_status(
        self,
        motor_voltage=23.0,
        motor_current=8.5,
        comm_voltage=23.0,
        comm_current=6.5,
        rms=0.05,
    ):
        """Get the power status.

        Parameters
        ----------
        motor_voltage : `float`, optional
            Total motor voltage. (the default is 23.0)
        motor_current : `float`, optional
            Total motor current in Ampere. (the default is 8.5)
        comm_voltage : `float`, optional
            Total communication voltage. (the default is 23.0)
        comm_current : `float`, optional
            Total communication current in Ampere. (the default is 6.5)
        rms : `float`, optional
            RMS variation. (the default is 0.5)

        Returns
        -------
        `dict`
            Power status.
        """

        rms_power = np.random.normal(scale=rms, size=4)

        if self.motor_power_on:
            motor_voltage_update = motor_voltage + rms_power[0]
            motor_current_update = motor_current + rms_power[1]
        else:
            motor_voltage_update = rms_power[0]
            motor_current_update = rms_power[1]

        if self.communication_power_on:
            comm_voltage_update = comm_voltage + rms_power[2]
            comm_current_update = comm_current + rms_power[3]
        else:
            comm_voltage_update = rms_power[2]
            comm_current_update = rms_power[3]

        return {
            "motorVoltage": motor_voltage_update,
            "motorCurrent": motor_current_update,
            "commVoltage": comm_voltage_update,
            "commCurrent": comm_current_update,
        }

    def _get_ilc_data(self):
        """Get the inner-loop controller (ILC) data.

        Returns
        -------
        `dict`
            Data of ILC data.
        """

        return {"status": [next(self._ilc_status)] * NUM_ACTUATOR}

    def _get_displacement_sensors(self, mirror_position=None):
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

    def _get_net_forces_total(self):
        """Get the total net forces in Newton.

        Returns
        -------
        `dict`
            Total net forces in Newton.
        """

        axial_forces = self.axial_forces["measured"]
        tangent_forces = self.tangent_forces["measured"]

        angle = np.deg2rad(self._cell_geom["locAct_tangent"])

        fx = np.sum(np.cos(angle) * tangent_forces)
        fy = np.sum(np.sin(angle) * tangent_forces)
        fz = np.sum(axial_forces)

        return {"fx": fx, "fy": fy, "fz": fz}

    def _get_net_moments_total(self):
        """Get the total net moments in Newton * meter.

        Returns
        -------
        `dict`
            Total net moments in Newton * meter.
        """

        axial_forces = self.axial_forces["measured"]
        tangent_forces = self.tangent_forces["measured"]

        location_axial_actuator = np.array(self._cell_geom["locAct_axial"])
        mx = np.sum(axial_forces * location_axial_actuator[:, 1])
        my = np.sum(axial_forces * location_axial_actuator[:, 0])
        mz = np.sum(tangent_forces) * self._cell_geom["radiusActTangent"]

        return {"mx": mx, "my": my, "mz": mz}

    def handle_forces(self, force_rms=0.5):
        """Handle forces.

        The method will check if the force balance system is activated and,
        if yes, will compute the forces based on the look up tables. If not,
        the LUT forces are set to zero. Then, it makes sure the forces are
        inside range and compute the forces dynamics.

        Parameters
        ----------
        force_rms : `float`, optional
            Force rms variation in Newton. (default is 0.5)

        Returns
        -------
        `bool`
            M2 assembly is in position or not.
        """

        # Update the force data
        self.calc_look_up_forces()

        n_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        if self.force_balance_system_status:
            self.axial_forces["hardpointCorrection"] = np.random.normal(
                scale=force_rms,
                size=n_axial_actuators,
            )

            self.tangent_forces["hardpointCorrection"] = np.random.normal(
                scale=force_rms,
                size=NUM_TANGENT_LINK,
            )

            self.force_balance = dict(
                [
                    (axis, np.random.normal(scale=force_rms))
                    for axis in ("fx", "fy", "fz", "mx", "my", "mz")
                ]
            )

        demanded_axial_force = self.check_axial_force_limit()
        demanded_tangent_force = self.check_tangent_force_limit()

        in_position_axial, final_force_axial = self.force_dynamics(
            demanded_axial_force, self.axial_forces["measured"], force_rms
        )
        in_position_tangent, final_force_axial_tangent = self.force_dynamics(
            demanded_tangent_force, self.tangent_forces["measured"], force_rms
        )

        # In the closed-loop control, update the measured forces and steps.
        # Otherwise, do not update the values.
        if self.force_balance_system_status:

            self.axial_forces["measured"] = final_force_axial + np.random.normal(
                scale=force_rms,
                size=n_axial_actuators,
            )
            self.tangent_forces[
                "measured"
            ] = final_force_axial_tangent + np.random.normal(
                scale=force_rms,
                size=NUM_TANGENT_LINK,
            )

            # Update the steps in the open-loop control
            forces = np.append(
                self.axial_forces["measured"], self.tangent_forces["measured"]
            )
            steps = self.control_open_loop.calculate_forces_to_steps(forces)
            self.control_open_loop.update_actuator_steps(steps)

        return in_position_axial and in_position_tangent

    def calc_look_up_forces(self):
        """Calculate look-up table (LUT) forces using current system state
        (position and temperature).
        """

        # Calculate the LUT forces of temperature component of axial actuators
        temperature_m2 = np.concatenate(
            (
                self.temperature["ring"],
                self.temperature["intake"],
                self.temperature["exhaust"],
            )
        )

        # Order temperature data based on a12_temperature.ipynb
        temperature_bin = temperature_m2[
            [0, 1, 2, 3, 12, 15, 14, 13, 8, 9, 10, 11, 4, 5, 6, 7]
        ]
        temperature_lut = temperature_bin[[1, 2, 3, 12, 9, 8, 13, 14, 15, 11, 10, 0]]

        tcoef = self.lut["temp_inv"].dot(temperature_lut - self.temperature["ref"])

        self.axial_forces["lutTemperature"] = np.squeeze(
            tcoef[0] * self.lut["Tr"]
            + tcoef[1] * self.lut["Tx"]
            + tcoef[2] * self.lut["Ty"]
            + tcoef[3] * self.lut["Tu"]
        )

        # Calculate the LUT forces of gravity component of axial actuators
        lut_angle = 90.0 - self.zenith_angle

        self.log.debug(
            f"Requested zAngle = {self.zenith_angle}," f"LUT angle = {lut_angle}."
        )

        n_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        myfe = np.zeros(n_axial_actuators)
        myf0 = np.zeros(n_axial_actuators)
        myfa = np.zeros(n_axial_actuators)
        myff = np.zeros(n_axial_actuators)
        for i in range(n_axial_actuators):
            myfe[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_E"][i, :]
            )
            myf0[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_0"][i, :]
            )
            myfa[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_A"][i, :]
            )
            myff[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_F"][i, :]
            )

        self.axial_forces["lutGravity"] = myfe + myfa

        # Calculate the LUT forces of gravity component of tangent actuators
        tangent_force = (
            self.mirror_weight
            * np.sin(np.radians(self.zenith_angle))
            / 4.0
            / np.cos(np.radians(30.0))
        )

        self.tangent_forces["lutGravity"] = np.array(
            [0.0, tangent_force, tangent_force, 0.0, -tangent_force, -tangent_force]
        )

    def force_dynamics(self, demand, current, force_rms, force_rate=100.0):
        """Handle force dynamics.

        The method works by comparing the `demand` forces with the `current`.
        forces. For each actuator, if the force differs by less than the
        amount of "force change per cycle" then the current value is set to
        the demand value. If the difference is larger, than the current value
        is modified by the amount of force change per cycle.

        If the difference in force is larger than the force change per
        cycle for any element, the system is considered "not in position".

        Parameters
        ----------
        demand : `numpy.ndarray`
            Demand forces in Newton.
        current : `numpy.ndarray`
            Current forces in Newton.
        force_rms : `float`
            force rms variation in Newton.
        force_rate : `float`, optional
            Force rate to be able to change in each cycle. The unit is N/sec.
            (default is 100.0)

        Returns
        -------
        in_position : `bool`
            Are forces in accepted range?
        final_force : `numpy.ndarray`
            Resulting forces in Newton.
        """

        # Check the mirror is in position or not
        force_diff = demand - current
        if np.std(force_diff) > 2.0 * force_rms:
            in_position = False
        else:
            in_position = True

        # Calculate the final fource

        # how much force per cycle it can apply
        force_per_cycle = force_rate * self.telemetry_interval

        actuators_in_cycle = np.where(force_diff <= force_per_cycle)[0]

        final_force = np.array(current, copy=True)
        final_force[actuators_in_cycle] = demand[actuators_in_cycle]

        actuators_outof_cycle = np.where(force_diff > force_per_cycle)[0]

        for actuator in actuators_outof_cycle:
            final_force[actuator] += (
                force_per_cycle * 1.0 if force_diff[actuator] > 0.0 else -1.0
            )
        return in_position, final_force

    def _simulate_position_mirror(self, position_rms=0.005, angle_rms=0.005):
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

        position = copy.copy(self.mirror_position)

        for axis, value in position.items():

            if axis in ("x, y, z"):
                scale = position_rms
            else:
                scale = angle_rms

            position[axis] = value + np.random.normal(scale=scale)

        return position

    def _simulate_temperature_and_update(
        self, temperature_rms=0.05, max_temperature=20.0, min_temperature=0.0
    ):
        """Simulate the temperature change and update the internal value.

        Parameters
        ----------
        temperature_rms : `float`, optional
            Temperature rms variation in degree C. (default is 0.05)
        max_temperature : `float`, optional
            Maximum temperature in degree C. (default is 20.0)
        min_temperature : `float`, optional
            Minimum temperature in degree C. (default is 0.0)
        """

        temperature_ring = self.temperature["ring"]

        # Simulate the change of ring temperature
        temperature_change = np.random.normal(
            scale=temperature_rms, size=len(temperature_ring)
        )

        if np.any(np.array(temperature_ring) > max_temperature):
            temperature_change = -abs(temperature_change)

        if np.any(np.array(temperature_ring) < min_temperature):
            temperature_change = abs(temperature_change)

        # Simulate the new intake temperature
        n_intake_temperatures = len(self.temperature["intake"])
        temperature_intake_new = (
            np.mean(np.array(temperature_ring).reshape(-1, 2), axis=0)
            - 0.5
            + np.random.normal(scale=temperature_rms, size=n_intake_temperatures)
        )

        # Simulate the new exhaust temperature
        n_exhaust_temperatures = len(self.temperature["exhaust"])
        temperature_exhaust_new = (
            np.mean(np.array(temperature_ring).reshape(-1, 2), axis=0)
            + 0.5
            + np.random.normal(scale=temperature_rms, size=n_exhaust_temperatures)
        )

        # Update the internal data
        self.temperature["ring"] = temperature_ring + temperature_change
        self.temperature["intake"] = temperature_intake_new
        self.temperature["exhaust"] = temperature_exhaust_new

    def _simulate_zenith_angle(self, inclinometer_rms=0.05):
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

        if self.inclination_source == MTM2.InclinationTelemetrySource.ONBOARD:
            measured = 90 - inclinometer_value
        else:
            measured = self.zenith_angle

        zenith_angle = dict()
        zenith_angle["measured"] = measured
        zenith_angle["inclinometerRaw"] = inclinometer_value
        zenith_angle[
            "inclinometerProcessed"
        ] = self.control_open_loop.correct_inclinometer_angle(inclinometer_value)

        return zenith_angle

    def handle_position_mirror(self, mirror_position_set_point):
        """Handle positining the mirror.

        Parameters
        ----------
        mirror_position_set_point : `dict`
            Dictionary with the same format as `self.mirror_position`.

        Raises
        ------
        RuntimeError
            Requested position outside of radial limits.
        RuntimeError
            Requested position out of limits.
        RuntimeError
            Failed to position the mirror.

        Returns
        -------
        `bool`
            True if succeeds. Otherwise, False.
        """

        if (not self.motor_power_on) or (self.force_balance_system_status is False):
            return False

        # Check limits
        # This is a simplication of the radial position, considering that
        # zRot = 0
        radial_position = np.sqrt(
            (
                mirror_position_set_point["x"]
                * np.cos(np.radians(mirror_position_set_point["yRot"] * 60.0 * 60.0))
            )
            ** 2.0
            + (
                mirror_position_set_point["y"]
                * np.cos(np.radians(mirror_position_set_point["xRot"] * 60.0 * 60.0))
            )
            ** 2.0
        )
        if radial_position > self.position_limit_radial:
            raise RuntimeError(
                f"Requested position outside of radial limits: {radial_position} "
                f"(limit: {self.position_limit_radial})"
            )

        # Another simplication of the limit in the optical direction.
        if mirror_position_set_point["z"] > self.position_limit_z:
            raise RuntimeError(
                f"Requested position out of limits: {mirror_position_set_point['z']} "
                f"(limit: {self.position_limit_z})"
            )

        mirror_distance = dict(
            [
                (axis, mirror_position_set_point[axis] - self.mirror_position[axis])
                for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
            ]
        )

        mirror_position_speed = dict(
            [(axis, 0.5) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
        )

        time_to_position = dict(
            [
                (axis, abs(mirror_distance[axis]) / mirror_position_speed[axis])
                for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
            ]
        )

        time = max([time_to_position[axis] for axis in time_to_position])

        if time <= self.telemetry_interval:
            sleep(time)
            for axis in self.mirror_position:
                self.mirror_position[axis] = mirror_position_set_point[axis]
        else:
            n_beats = int(np.ceil(time / self.telemetry_interval))
            beats = 0
            self.log.debug(f"It will take {n_beats} cycles to reposition the mirror.")
            while np.any(
                np.array([abs(mirror_distance[axis]) for axis in mirror_distance]) > 0.0
            ):
                self.log.debug(f"{mirror_distance}")
                if beats > n_beats + 1:
                    raise RuntimeError("Failed to position the mirror.")
                beats += 1

                for axis in self.mirror_position:
                    if (
                        abs(mirror_distance[axis]) > 0.0
                        and abs(mirror_distance[axis])
                        < mirror_position_speed[axis] * self.telemetry_interval
                    ):
                        self.mirror_position[axis] = mirror_position_set_point[axis]
                        mirror_distance[axis] = 0.0
                        self.log.debug(f"{axis} in position.")
                    elif (
                        abs(mirror_distance[axis])
                        > mirror_position_speed[axis] * self.telemetry_interval
                    ):
                        delta_pos = (
                            mirror_position_speed[axis]
                            * self.telemetry_interval
                            * (1.0 if mirror_distance[axis] > 0.0 else -1.0)
                        )
                        self.mirror_position[axis] += delta_pos
                        mirror_distance[axis] -= delta_pos
                sleep(self.telemetry_interval)

        return True

    def enable_open_loop_max_limit(self, status):
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

        if (status is True) and (self.force_balance_system_status is True):
            self.control_open_loop.open_loop_max_limit_is_enabled = False
            result = False
        else:
            self.control_open_loop.open_loop_max_limit_is_enabled = status

        return result

    def reset_breakers(self, power_type):
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

        if (power_type == PowerType.Motor) and self.motor_power_on:
            result = True
        elif (power_type == PowerType.Communication) and self.communication_power_on:
            result = True

        return result

    def get_digital_output(self):
        """Get the value of digital output.

        Returns
        -------
        digital_output : `int`
            Value of the digital output.
        """

        digital_output = sum([item.value for item in self.digital_output_default])

        if self.communication_power_on:
            digital_output += DigitalOutput.CommunicationPower.value

        if self.motor_power_on:
            digital_output += DigitalOutput.MotorPower.value

        return digital_output

    def get_digital_input(self):
        """Get the value of digital input that represents the current state of
        the system.

        Returns
        -------
        digital_input : `int`
            Value of the digital input.
        """

        digital_input = sum([item.value for item in self.digital_input_default])

        if self.communication_power_on:
            digital_input -= sum(
                [item.value for item in self.digital_input_communication]
            )

        if self.motor_power_on:
            digital_input -= sum([item.value for item in self.digital_input_motor])

        return digital_input
