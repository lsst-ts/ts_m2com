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
from ..utility import check_limit_switches, read_yaml_file


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
    hardpoints : `list`
        List of the hardpoints. There are 6 actuators. The first three are the
        axial actuators and the latter three are the tangent links.
    in_position_hardpoints : `bool`
        Hardpoints are in position or not.
    """

    # Limits of force in Newton
    LIMIT_FORCE_AXIAL = 444.82  # 100 lbf
    LIMIT_FORCE_TANGENT = 4893.04  # 1100 lbf

    def __init__(self):

        self.is_running = False

        self.temperature = MockControlClosedLoop._get_temperatures()

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

        # No look-up table (LUT) temperature correction in tangent links
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

        # These are hard-coded in the M2 cell LabVIEW project by vendor
        self.hardpoints = [5, 15, 25, 73, 75, 77]

        self.in_position_hardpoints = False

        # Look-up tables (LUTs)
        self._lut = dict()

        # Cell geometry used in the calculation of net total forces and moments
        self._cell_geom = dict()

        # Hardpoint compensation matrix
        self._hd_comp = np.array([])

    @staticmethod
    def _get_temperatures(
        temperature_init_low=24.49,
        temperature_init_high=26.53,
        temperature_ref=21.0,
        max_difference=2.0,
    ):
        """Get the default temperatures. The unit is degree C.

        Parameters
        ----------
        temperature_init_low : `float`, optional
            Initial temperature (low) in degree C. (the default is 24.49)
        temperature_init_high : `float`, optional
            Initial temperature (high) in degree C. (the default is 26.53)
        temperature_ref : `float`, optional
            Reference temperature in degree C. (the default is 21.0)
        max_difference : `float`, optional
           Maximum difference of temperature in degree C. (the default is 2.0)

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
        self, temperature_rms=0.05, max_temperature=28.0, min_temperature=0.0
    ):
        """Simulate the temperature change and update the internal value.

        Parameters
        ----------
        temperature_rms : `float`, optional
            Temperature rms variation in degree C. (the default is 0.05)
        max_temperature : `float`, optional
            Maximum temperature in degree C. (the default is 28.0)
        min_temperature : `float`, optional
            Minimum temperature in degree C. (the default is 0.0)
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
        num_intake_temperatures = len(self.temperature["intake"])
        temperature_intake_new = (
            np.mean(np.array(temperature_ring).reshape(-1, 2), axis=0)
            - 0.5
            + np.random.normal(scale=temperature_rms, size=num_intake_temperatures)
        )

        # Simulate the new exhaust temperature
        num_exhaust_temperatures = len(self.temperature["exhaust"])
        temperature_exhaust_new = (
            np.mean(np.array(temperature_ring).reshape(-1, 2), axis=0)
            + 0.5
            + np.random.normal(scale=temperature_rms, size=num_exhaust_temperatures)
        )

        # Update the internal data
        self.temperature["ring"] = temperature_ring + temperature_change
        self.temperature["intake"] = temperature_intake_new
        self.temperature["exhaust"] = temperature_exhaust_new

    def set_measured_forces(self, force_axial, force_tangent):
        """Set the measured actuator forces.

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
            raise ValueError(
                "The input dimensions of forces are wrong. "
                f"Expected {NUM_ACTUATOR - NUM_TANGENT_LINK}/{NUM_TANGENT_LINK} "
                "for the axial and tangent forcers, "
                f"got {len(force_axial)}/{len(force_tangent)}, respectively."
            )

        self.axial_forces["measured"] = force_axial
        self.tangent_forces["measured"] = force_tangent

    def load_file_lut(self, filepath):
        """Load the look-up table (LUT) files.

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

    def load_file_cell_geometry(self, filepath):
        """Load the file of cell geometry.

        Parameters
        ----------
        filepath : `pathlib.PosixPath`
            File path of cell geometry.
        """

        self._cell_geom = read_yaml_file(filepath)

    def load_file_hardpoint_compensation(self, filepath, skiprows=7):
        """Load the file of hardpoint compensation.

        Parameters
        ----------
        filepath : `pathlib.PosixPath`
            File path of hardpoint compensation.
        skiprows : `int`, optional
            Number of lines to skip (int) at the start of the file. (the
            default is 7)
        """

        dataframe = pd.read_csv(filepath, skiprows=skiprows)
        data = np.array(dataframe.iloc[:, 0])

        # There are 3 axial actuators to be hardpoints
        hd_comp_axial = data.reshape(NUM_ACTUATOR - NUM_TANGENT_LINK - 3, 3)

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

        return diff > self.temperature["maxDiff"]

    def apply_forces(self, force_axial, force_tangent):
        """Apply the actuator forces.

        Parameters
        ----------
        force_axial : `list` or `numpy.ndarray`
            Force of the axial actuators in Newton.
        force_tangent : `list` or `numpy.ndarray`
            Force of the tangent actuators in Newton.

        Raises
        ------
        `RuntimeError`
            When the maximum force limits reached.
        """

        # Check limits
        is_out_limit = self.is_actuator_force_out_limit(
            applied_force_axial=np.array(force_axial),
            applied_force_tangent=np.array(force_tangent),
        )[0]
        if is_out_limit:
            raise RuntimeError("Maximum force limits reached in actuators.")

        self.axial_forces["applied"] = np.array(force_axial)
        self.tangent_forces["applied"] = np.array(force_tangent)

    def is_actuator_force_out_limit(
        self, applied_force_axial=None, applied_force_tangent=None
    ):
        """The actuator force is out of limit or not.

        Parameters
        ----------
        applied_force_axial : `numpy.ndarray` or `None`, optional
            Axial forces to apply in Newton. If `None` use current setup. (the
            default is None.)
        applied_force_tangent : `numpy.ndarray` or `None`, optional
            Tangent forces to apply in Newton. If `None` use current setup. (
            the default is None.)

        Returns
        -------
        `bool`
            True if the actuator force is out of limit. Otherwise, False.
        `list`
            Triggered retracted limit switches.
        `list`
            Triggered extended limit switches.
        """

        demanded_force = self.get_demanded_force(
            applied_force_axial=applied_force_axial,
            applied_force_tangent=applied_force_tangent,
        )

        return check_limit_switches(
            demanded_force, self.LIMIT_FORCE_AXIAL, self.LIMIT_FORCE_TANGENT
        )

    def get_demanded_force(self, applied_force_axial=None, applied_force_tangent=None):
        """Get the demanded force in Newton.

        Parameters
        ----------
        applied_force_axial : `numpy.ndarray` or `None`, optional
            Axial forces to apply in Newton. If `None` use current setup. (the
            default is None.)
        applied_force_tangent : `numpy.ndarray` or `None`, optional
            Tangent forces to apply in Newton. If `None` use current setup. (
            the default is None.)

        Returns
        -------
        `numpy.ndarray`
            Array with the combined total force per actuator in Newton.
        """

        demanded_force_axial = (
            (
                applied_force_axial
                if applied_force_axial is not None
                else self.axial_forces["applied"]
            )
            + self.axial_forces["lutGravity"]
            + self.axial_forces["lutTemperature"]
        )

        # Compared with the axial actuators (demanded_force_axial), there is no
        # temperature correction from the look-up table
        demanded_force_tanget = (
            applied_force_tangent
            if applied_force_tangent is not None
            else self.tangent_forces["applied"]
        ) + self.tangent_forces["lutGravity"]

        return np.append(demanded_force_axial, demanded_force_tanget)

    def reset_force_offsets(self):
        """Reset the force offsets.

        This will put the applied force to be zero.
        """

        self.axial_forces["applied"] = np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)
        self.tangent_forces["applied"] = np.zeros(NUM_TANGENT_LINK)

    def get_net_forces_total(self):
        """Get the total net forces in Newton.

        Returns
        -------
        `dict`
            Total net forces in Newton.
        """
        return self._calculate_xyz_net_forces(
            self.axial_forces["measured"], self.tangent_forces["measured"]
        )

    def _calculate_xyz_net_forces(self, axial_forces, tangent_forces):
        """Calculate the net forces in Newton on the individual axis.

        Parameters
        ----------
        axial_forces : `numpy.ndarray`
            Axial actuator forces in Newton.
        tangent_forces : `numpy.ndarray`
            Tangent actuator forces in Newton.

        Returns
        -------
        `dict`
            Total net forces in Newton.
        """

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
        return self._calculate_xyz_net_moments(
            self.axial_forces["measured"], self.tangent_forces["measured"]
        )

    def _calculate_xyz_net_moments(self, axial_forces, tangent_forces):
        """Calculate the total net moments in Newton * meter on the individual
        axis.

        Parameters
        ----------
        axial_forces : `numpy.ndarray`
            Axial actuator forces in Newton.
        tangent_forces : `numpy.ndarray`
            Tangent actuator forces in Newton.

        Returns
        -------
        `dict`
            Total net moments in Newton * meter.
        """

        location_axial_actuator = np.array(self._cell_geom["locAct_axial"])
        mx = np.sum(axial_forces * location_axial_actuator[:, 1])
        my = np.sum(axial_forces * location_axial_actuator[:, 0])
        mz = np.sum(tangent_forces) * self._cell_geom["radiusActTangent"]

        return {"mx": mx, "my": my, "mz": mz}

    def get_force_balance(self):
        """Get the data of force balance system. This contains the net forces
        (in Newton) and net moments (in Newton * meter).

        Returns
        -------
        force_balance : `dict`
            Data of the force balance system.
        """

        axial_forces = self.axial_forces["hardpointCorrection"]
        tangent_forces = self.tangent_forces["hardpointCorrection"]

        net_forces = self._calculate_xyz_net_forces(axial_forces, tangent_forces)
        net_moments = self._calculate_xyz_net_moments(axial_forces, tangent_forces)

        force_balance = net_forces.copy()
        force_balance.update(net_moments)

        return force_balance

    def calc_look_up_forces(self, lut_angle):
        """Calculate look-up table (LUT) forces using current system state
        (position and temperature).

        Parameters
        ----------
        lut_angle : `float`
            Angle used to calculate the LUT forces of gravity component.
        """

        # Calculate the LUT forces of temperature component
        temperature_m2 = np.concatenate(
            (
                self.temperature["ring"],
                self.temperature["intake"],
                self.temperature["exhaust"],
            )
        )

        # Order temperature data based on a12_temperature.ipynb in
        # https://github.com/lsst-sitcom/M2_summit_2003
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

        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        self.axial_forces["lutGravity"] = force_gravity[:num_axial_actuators]
        self.tangent_forces["lutGravity"] = force_gravity[-NUM_TANGENT_LINK:]

        # Calculate the hardpoint correction
        force_demanded = self.get_demanded_force()
        force_measured = np.append(
            self.axial_forces["measured"], self.tangent_forces["measured"]
        )
        force_hardpoint = self._calc_look_up_forces_hardpoint(
            force_demanded, force_measured
        )

        self.axial_forces["hardpointCorrection"] = force_hardpoint[:num_axial_actuators]
        self.tangent_forces["hardpointCorrection"] = force_hardpoint[-NUM_TANGENT_LINK:]

    def _calc_look_up_forces_temperature(self, lut_temperature, temperature_ref):
        """Calculate the temperature-related forces based on the look-up table
        (LUT) in Newton.

        Only the axial actuators have this correction.

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

        for idx in range(NUM_ACTUATOR - NUM_TANGENT_LINK):
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
        """Calculate the forces of hardpoint compensation based on the look-up
        table (LUT) in Newton.

        Parameters
        ----------
        force_demanded : `numpy.ndarray`
            Demanded actuator forces in Newton.
        force_measured : `numpy.ndarray`
            Measured actuator forces in Newton.

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

    def handle_forces(self, force_rms=0.5, force_per_cycle=5):
        """Handle forces and check the actuators are in position or not based
        on the demanded forces, hardpoint correction, and measured forces.

        If the closed-loop control is running, the measured force will be
        updated based on the "force_per_cycle" as well to decrease the force
        error. This update will add a random error based on "force_rms".
        Therefore, even in the convergend condition, the "in_position" might be
        False, which is an expected behavior for the real hardware.

        This function makes sure the forces are inside range and compute the
        forces dynamics.

        Parameters
        ----------
        force_rms : `float`, optional
            Force rms variation in Newton. (the default is 0.5)
        force_per_cycle : `float`, optional
            Force per cycle to apply in Newton. (the default is 5)

        Returns
        -------
        in_position : `bool`
            M2 assembly is in position or not.
        """

        in_position, final_force = self._force_dynamics(force_rms, force_per_cycle)

        # In the closed-loop control, update the measured forces. Otherwise, do
        # not update the values.
        if self.is_running:

            num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK

            self.axial_forces["measured"] = final_force[
                :num_axial_actuators
            ] + np.random.normal(
                scale=force_rms,
                size=num_axial_actuators,
            )
            self.tangent_forces["measured"] = final_force[
                -NUM_TANGENT_LINK:
            ] + np.random.normal(
                scale=force_rms,
                size=NUM_TANGENT_LINK,
            )

        return in_position

    def _force_dynamics(self, force_rms, force_per_cycle):
        """Handle the force dynamics.

        The method works by comparing the demanded forces with the current
        forces and hardpoint correction. For each actuator, if the force
        differs by less than the amount of "force_per_cycle" then the
        current value is set to the demand value minus the hardpoint
        correction. If the difference is larger, the current value will be
        modified by the amount of "force_per_cycle".

        The target is:
        |force_demanded - force_measured - hardpoints| < force_error

        Parameters
        ----------
        force_rms : `float`
            force rms variation in Newton. This is used to check the actuators
            are in position or not.
        force_per_cycle : `float`
            Force per cycle to apply in Newton.

        Returns
        -------
        in_position : `bool`
            Are forces in accepted range before the application of
            "force_per_cycle"?
        final_force : `numpy.ndarray`
            Resulting forces in Newton. If the actuators are in position, this
            will be the measured forces.
        """

        (
            force_error,
            force_demanded,
            force_hardpoint,
            force_measured,
        ) = self._get_force_error()

        if np.std(force_error) > 2.0 * force_rms:
            in_position = False
        else:
            in_position = True

        if (not self.in_position_hardpoints) and in_position:
            self.in_position_hardpoints = True

        # Calculate the final force
        final_force = force_measured.copy()
        if not in_position:

            # Since the force error in hardpoints are always zero, we need to
            # deal with them seperatily.
            actuators_in_cycle = np.where(np.abs(force_error) <= force_per_cycle)[0]

            actuators_in_cycle_no_hardpoint = set(actuators_in_cycle)
            for hardpoint in self.hardpoints:
                actuators_in_cycle_no_hardpoint.discard(hardpoint)

            actuators_in_cycle_no_hardpoint = np.array(
                list(actuators_in_cycle_no_hardpoint), dtype=int
            )

            final_force[actuators_in_cycle_no_hardpoint] = (
                force_demanded[actuators_in_cycle_no_hardpoint]
                - force_hardpoint[actuators_in_cycle_no_hardpoint]
            )

            actuators_out_of_cycle = np.where(np.abs(force_error) > force_per_cycle)[0]
            final_force[actuators_out_of_cycle] += force_per_cycle * np.sign(
                force_error[actuators_out_of_cycle]
            )

            # We only update the forces of hardpoints if they are not in
            # position yet.
            if not self.in_position_hardpoints:
                # Assume the direction of hardpoint update is the same as its
                # demanded force direction as a random assumption
                final_force[self.hardpoints] += force_per_cycle * np.sign(
                    force_demanded[self.hardpoints]
                )

        return in_position, final_force

    def _get_force_error(self):
        """Get the actuator force error.

        Returns
        ----------
        force_error : `numpy.ndarray`
            Actuator force error in Newton.
        force_demanded : `numpy.ndarray`
            Demanded actuator force in Newton.
        force_hardpoint : `numpy.ndarray`
            Hardpoint correction in Newton.
        force_measured : `numpy.ndarray`
            Measured actuator force in Newton.
        """

        force_demanded = self.get_demanded_force()
        force_hardpoint = np.append(
            self.axial_forces["hardpointCorrection"],
            self.tangent_forces["hardpointCorrection"],
        )
        force_measured = np.append(
            self.axial_forces["measured"], self.tangent_forces["measured"]
        )

        # In the calculation of force error, we minus the measured force
        # directly as a first order assumption. In the real control system,
        # we need to consider a feedback loop to decide the actual value of
        # measured force from the previous loop in hardware controller.
        force_error = force_demanded - force_hardpoint - force_measured

        # Do not consider the force error in hardpoints.
        # This is the logic in M2 cell LabVIEW code by vendor. Need to figure
        # out why it was designed in this way in a latter time.
        force_error[self.hardpoints] = 0

        return force_error, force_demanded, force_hardpoint, force_measured
