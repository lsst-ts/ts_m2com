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

__all__ = ["MockControlClosedLoop"]

import numpy as np
import pandas as pd
from scipy.linalg import block_diag

from ..constant import NUM_ACTUATOR, NUM_TANGENT_LINK
from ..utility import read_yaml_file


class MockControlClosedLoop:
    """Mock closed-loop control.

    Attributes
    ----------
    is_running : `bool`
        The closed-loop control is running or not.
    temperature : `dict`
        Temperature of mirror in degree C.
    axial_forces : `dict`
        Forces of the axial actuators in Newton.
    tangent_forces : `dict`
        Forces of the tangent actuators in Newton.
    force_balance : `dict`
        Force balance system contains the information of force and moment.
        The units are the Newton and Newton * meter.
    hardpoints : `list`
        List of the hardpoints.
    """

    # Limits of force in Newton
    LIMIT_FORCE_AXIAL = 444.82  # 100 lbf
    LIMIT_FORCE_TANGENT = 4893.04  # 1100 lbf

    def __init__(self):

        self.is_running = False

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

        # No LUT temperature correction for tangent links
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
        self.tangent_forces["lutTemperature"] = np.array([])

        self.force_balance = dict(
            [(axis, 0) for axis in ("fx", "fy", "fz", "mx", "my", "mz")]
        )

        self.hardpoints = [5, 15, 25, 73, 75, 77]

        # Look-up table (LUT)
        self._lut = dict()

        # Cell geometry used in the calculation of net total forces and moments
        self._cell_geom = dict()

        # Hardpoint compensation matrix
        self._hd_comp = np.array([])

    def _get_default_temperatures(
        self,
        temperature_init_low=24.49,
        temperature_init_high=26.53,
        temperature_ref=21.0,
        max_difference=2.0,
    ):
        """Get the default temperatures. The unit is degree C.

        Parameters
        ----------
        temperature_init_low : `float`, optional
            Initial temperature (low) in degree C. (default is 24.49)
        temperature_init_high : `float`, optional
            Initial temperature (high) in degree C. (default is 26.53)
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
        temperatures["ring"] = [
            temperature_init_low,
            temperature_init_low,
            temperature_init_low,
            temperature_init_high,
            temperature_init_low,
            temperature_init_low,
            temperature_init_high,
            temperature_init_high,
            temperature_init_low,
            temperature_init_low,
            temperature_init_low,
            temperature_init_high,
        ]
        temperatures["intake"] = [temperature_init_low] * 2
        temperatures["exhaust"] = [temperature_init_low] * 2
        temperatures["ref"] = temperature_ref
        temperatures["maxDiff"] = max_difference

        return temperatures

    def simulate_temperature_and_update(
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

    def set_measured_forces(self, force_axial, force_tangent):
        """Set the measured forces.

        Parameters
        ----------
        force_axial : `numpy.ndarray`
            Axial force in Newton.
        force_tangent : `numpy.ndarray`
            Tangential force in Newton.

        Raises
        ------
        `ValueError`
            When the input dimensions of forces are wrong.
        """

        if (len(force_axial) != (NUM_ACTUATOR - NUM_TANGENT_LINK)) or (
            len(force_tangent) != NUM_TANGENT_LINK
        ):
            raise ValueError("The input dimensions of forces are wrong.")

        self.axial_forces["measured"] = force_axial
        self.tangent_forces["measured"] = force_tangent

    def read_file_lut(self, filepath):
        """Read the look-up table (LUT) files.

        Parameters
        ----------
        filepath : `pathlib.PosixPath`
            File path of LUT directory.
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
            name, ext = lut.split(".")
            if ext == "csv":
                data = pd.read_csv(
                    filepath / lut,
                    header="infer" if name.startswith("F") else None,
                )
                self._lut[name] = np.float64(data)
                if name == "F_E":
                    self._lut["lutInAngle"] = np.float64(data.keys())
            else:
                self._lut[name] = np.loadtxt(filepath / lut)

    def read_file_cell_geometry(self, filepath):
        """Read the file of cell geometry.

        Parameters
        ----------
        filepath : `pathlib.PosixPath`
            File path of cell geometry.
        """

        self._cell_geom = read_yaml_file(filepath)

    def read_file_hardpoint_compensation(self, filepath, skiprows=7):
        """Read the file of hardpoint compensation.

        Parameters
        ----------
        filepath : `pathlib.PosixPath`
            File path of hardpoint compensation.
        """

        dataframe = pd.read_csv(filepath, skiprows=skiprows)
        data = np.array(dataframe.iloc[:, 0])

        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK

        # There are 3 axial actuators to be hardpoints
        hd_comp_axial = data.reshape(num_axial_actuators - 3, 3)

        hd_comp_tangent = np.array(
            [[2 / 3, -1 / 3, 2 / 3], [2 / 3, 2 / 3, -1 / 3], [-1 / 3, 2 / 3, 2 / 3]]
        )

        self._hd_comp = block_diag(hd_comp_axial, hd_comp_tangent)

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
        self.check_axial_force_limit(applied_force=force_axial)
        self.check_tangent_force_limit(applied_force=force_tangent)

        self.axial_forces["applied"] = np.array(force_axial)
        self.tangent_forces["applied"] = np.array(force_tangent)

    def check_axial_force_limit(self, applied_force=None):
        """Check if axial forces are out of bounds.

        Parameters
        ----------
        applied_force : `numpy.ndarray` or `None`, optional
            Forces to apply in Newton. If `None` use current setup. (default is
            None.)

        Returns
        -------
        demanded_axial_force : `numpy.ndarray`
            Array with the combined total force per actuator in Newton.

        Raises
        ------
        `RuntimeError`
            If force limit is out of range.
        """

        demanded_axial_force = (
            (
                applied_force
                if applied_force is not None
                else self.axial_forces["applied"]
            )
            + self.axial_forces["lutGravity"]
            + self.axial_forces["lutTemperature"]
        )

        if np.any(demanded_axial_force > self.LIMIT_FORCE_AXIAL):
            actuators = np.where(demanded_axial_force > self.LIMIT_FORCE_AXIAL)[0]
            raise RuntimeError(
                f"Maximum axial force limit [{self.LIMIT_FORCE_AXIAL}N] reached in actuators {actuators}."
            )

        return demanded_axial_force

    def check_tangent_force_limit(self, applied_force=None):
        """Check if tangent forces are out of bounds.

        Parameters
        ----------
        applied_force : `numpy.ndarray` or `None`, optional
            Forces to apply in Newton. If `None` use current setup. (default is
            None.)

        Returns
        -------
        demanded_tanget_force : `numpy.ndarray`
            Array with the combined total force per actuator in Newton.

        Raises
        ------
        `RuntimeError`
            If force limit is out of range.
        """

        demanded_tanget_force = np.array(
            (
                applied_force
                if applied_force is not None
                else self.tangent_forces["applied"]
            )
            + self.tangent_forces["lutGravity"]
        )

        if np.any(demanded_tanget_force > self.LIMIT_FORCE_TANGENT):
            actuators = np.where(demanded_tanget_force > self.LIMIT_FORCE_TANGENT)[0]
            raise RuntimeError(
                f"Maximum axial force limit [{self.LIMIT_FORCE_TANGENT}N] reached in actuators {actuators}."
            )

        return demanded_tanget_force

    def reset_force_offsets(self):
        """Reset the force offsets."""

        self.axial_forces["applied"] = np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)
        self.tangent_forces["applied"] = np.zeros(NUM_TANGENT_LINK)

    def get_net_forces_total(self):
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

    def get_net_moments_total(self):
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

    def handle_forces(self, lut_angle, force_rms=0.5, force_per_cycle=5):
        """Handle forces.

        The method will check if the force balance system is activated and,
        if yes, will compute the forces based on the look up tables. If not,
        the LUT forces are set to zero. Then, it makes sure the forces are
        inside range and compute the forces dynamics.

        Parameters
        ----------
        lut_angle : `float`
            Angle used to calculate the LUT forces of gravity component of
            axial actuators
        force_rms : `float`, optional
            Force rms variation in Newton. (default is 0.5)
        force_per_cycle : `float`, optional
            Force per cycle to apply in Newton. (the default is 5)

        Returns
        -------
        `bool`
            M2 assembly is in position or not.
        """

        # Update the force data
        self.calc_look_up_forces(lut_angle)

        n_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        if self.is_running:
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
            demanded_axial_force,
            self.axial_forces["measured"],
            force_rms,
            force_per_cycle=force_per_cycle,
        )
        in_position_tangent, final_force_axial_tangent = self.force_dynamics(
            demanded_tangent_force,
            self.tangent_forces["measured"],
            force_rms,
            force_per_cycle=force_per_cycle,
        )

        # In the closed-loop control, update the measured forces. Otherwise, do
        # not update the values.
        if self.is_running:

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

        return in_position_axial and in_position_tangent

    def calc_look_up_forces(self, lut_angle):
        """Calculate look-up table (LUT) forces using current system state
        (position and temperature).

        Parameters
        ----------
        lut_angle : `float`
            Angle used to calculate the LUT forces of gravity component of
            axial actuators
        """

        # Calculate the LUT forces of temperature component
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

        force_r, force_x, force_y, force_u = self._calc_look_up_forces_temperature(
            temperature_lut, self.temperature["ref"]
        )

        self.axial_forces["lutTemperature"] = force_r + force_x + force_y + force_u

        # Calculate the LUT forces of gravity component
        (
            force_elevation,
            force_0g_component,
            force_actuator_bias,
            force_factory_offset,
        ) = self._calc_look_up_forces_gravity(lut_angle)

        force_gravity = (
            force_elevation
            + force_0g_component
            + force_actuator_bias
            + force_factory_offset
        )

        self.axial_forces["lutGravity"] = force_gravity[
            : (NUM_ACTUATOR - NUM_TANGENT_LINK)
        ]
        self.tangent_forces["lutGravity"] = force_gravity[-NUM_TANGENT_LINK:]

        # Calculate the hardpoint correction
        force_demanded = np.append(
            self.check_axial_force_limit(), self.check_tangent_force_limit()
        )
        force_measured = np.append(
            self.axial_forces["measured"], self.tangent_forces["measured"]
        )
        force_hardpoint = self._calc_look_up_forces_hardpoint(
            force_demanded, force_measured
        )

        self.axial_forces["hardpointCorrection"] = force_hardpoint[
            : (NUM_ACTUATOR - NUM_TANGENT_LINK)
        ]
        self.tangent_forces["hardpointCorrection"] = force_hardpoint[-NUM_TANGENT_LINK:]

    def _calc_look_up_forces_temperature(self, lut_temperature, temperature_ref):
        """Calculate the temperature-related forces based on the look-up table
        (LUT) in Newton.

        Note: Only the axial actuators have this correction.

        Parameters
        ----------
        lut_temperature : `numpy.ndarray`
            Temperature used to calculate the LUT forces of temperature
            component in degree C. The order is: [LG2-2, LG2-3, LG2-4,
            LG3-1, LG4-2, LG4-1, LG3-2, LG3-3, LG3-4, LG4-4, LG4-3, LG2-1].
        temperature_ref : `float`
            Reference temperature in degree C.

        Returns
        -------
        `numpy.ndarray`
            Temperature compensation vector for radial gradient.
        `numpy.ndarray`
            Temperature compensation vector for x-gradient.
        `numpy.ndarray`
            Temperature compensation vector for y-gradient.
        `numpy.ndarray`
            Temperature compensation vector for uniform gradient.
        """

        temperature = lut_temperature - temperature_ref
        tcoef = self._lut["temp_inv"].dot(temperature.reshape(-1, 1))

        return (
            np.squeeze(tcoef[0] * self._lut["Tr"]),
            np.squeeze(tcoef[1] * self._lut["Tx"]),
            np.squeeze(tcoef[2] * self._lut["Ty"]),
            np.squeeze(tcoef[3] * self._lut["Tu"]),
        )

    def _calc_look_up_forces_gravity(self, lut_angle):
        """Calculate the gravity-related forces based on the look-up table
        (LUT) in Newton.

        Parameters
        ----------
        lut_angle : `float`
            Angle used to calculate the LUT forces of gravity component in
            degree.

        Returns
        -------
        force_elevation : `numpy.ndarray`
            Force for the elevation component of command in Newton.
        force_0g_component : `numpy.ndarray`
            Force for the 0g component of command in Newton.
        force_actuator_bias : `numpy.ndarray`
            Force for the actuator force bias in Newton.
        force_factory_offset : `numpy.ndarray`
            Force for the factory force offset in Newton.
        """

        force_elevation = np.zeros(NUM_ACTUATOR)
        force_0g_component = np.zeros(NUM_ACTUATOR)
        force_actuator_bias = np.zeros(NUM_ACTUATOR)
        force_factory_offset = np.zeros(NUM_ACTUATOR)

        n_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        for idx in range(n_axial_actuators):
            force_elevation[idx] = np.interp(
                lut_angle, self._lut["lutInAngle"], self._lut["F_E"][idx, :]
            )
            force_0g_component[idx] = np.interp(
                lut_angle, self._lut["lutInAngle"], self._lut["F_0"][idx, :]
            )
            force_factory_offset[idx] = np.interp(
                lut_angle, self._lut["lutInAngle"], self._lut["F_F"][idx, :]
            )

        for idx in range(NUM_ACTUATOR):
            force_actuator_bias[idx] = np.interp(
                lut_angle, self._lut["lutInAngle"], self._lut["F_A"][idx, :]
            )

        return (
            force_elevation,
            force_0g_component,
            force_actuator_bias,
            force_factory_offset,
        )

    def _calc_look_up_forces_hardpoint(self, force_demanded, force_measured):
        """Calculate the forces of harpoint compensation based on the look-up
        table (LUT) in Newton.

        Parameters
        ----------
        force_demanded : `numpy.ndarray`
            Demanded actuator force in Newton.
        force_measured : `numpy.ndarray`
            Measured actuator force in Newton.

        Returns
        -------
        force_hardpoint : `numpy.ndarray`
            Forces for the hardpoint compensation in Newton.
        """

        force_demanded_hardpoints = force_demanded[self.hardpoints]
        force_compensation = np.squeeze(
            self._hd_comp.dot(force_demanded_hardpoints.reshape(-1, 1))
        )

        list_non_hardpoints = list(set(range(NUM_ACTUATOR)) - set(self.hardpoints))
        force_hardpoint = force_compensation - force_measured[list_non_hardpoints]

        # Hardpoints do not do the correction, put 0
        for hardpoint in self.hardpoints:
            force_hardpoint = np.insert(force_hardpoint, hardpoint, 0)

        return force_hardpoint

    def force_dynamics(self, demand, current, force_rms, force_per_cycle=5):
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
        force_per_cycle : `float`, optional
            Force per cycle to apply in Newton. (the default is 5)

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
        actuators_in_cycle = np.where(force_diff <= force_per_cycle)[0]

        final_force = np.array(current, copy=True)
        final_force[actuators_in_cycle] = demand[actuators_in_cycle]

        actuators_outof_cycle = np.where(force_diff > force_per_cycle)[0]

        for actuator in actuators_outof_cycle:
            final_force[actuator] += (
                force_per_cycle * 1.0 if force_diff[actuator] > 0.0 else -1.0
            )
        return in_position, final_force
