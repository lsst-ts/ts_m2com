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

from pathlib import Path

import numpy as np
import numpy.typing
import pandas as pd
from scipy import signal
from scipy.linalg import block_diag

from ..constant import (
    LIMIT_FORCE_AXIAL_CLOSED_LOOP,
    LIMIT_FORCE_TANGENT_CLOSED_LOOP,
    NUM_ACTUATOR,
    NUM_HARDPOINTS_AXIAL,
    NUM_TANGENT_LINK,
    NUM_TEMPERATURE_EXHAUST,
    NUM_TEMPERATURE_INTAKE,
    NUM_TEMPERATURE_RING,
)
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
    disp_hardpoint_home : `list`
        Displacement of the hardpoints at the home position. The unit is meter.
    in_position_hardpoints : `bool`
        Hardpoints are in position or not.
    """

    def __init__(self) -> None:
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

        # The values here are designed intentionally to let the mirror has the
        # nearly-zero home position at the closed-loop control.
        self.disp_hardpoint_home = [
            -0.00168923,
            -0.00647945,
            -0.01085443,
            -0.0016753,
            -0.00323069,
            -0.00281834,
        ]

        self.in_position_hardpoints = False

        # Look-up tables (LUTs)
        self._lut: dict = dict()

        # Cell geometry used in the calculation of net total forces and moments
        self._cell_geom: dict = dict()

        # Hardpoint compensation matrix
        self._hd_comp = np.array([])

        # Stiffness matrix to reflect the force change of actuator's movement
        self._stiffness = np.array([])

        # Kinetic decoupling matrix
        self._kdc = np.array([])

    @staticmethod
    def _get_temperatures(
        temperature_init_low: float = 24.49,
        temperature_init_high: float = 26.53,
        temperature_ref: float = 21.0,
        max_difference: float = 2.0,
    ) -> dict:
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
        temperatures["intake"] = [temperature_init_low] * NUM_TEMPERATURE_INTAKE
        temperatures["exhaust"] = [temperature_init_low] * NUM_TEMPERATURE_EXHAUST
        temperatures["ref"] = [temperature_ref] * NUM_TEMPERATURE_RING
        temperatures["maxDiff"] = max_difference  # type: ignore

        return temperatures

    def update_hardpoints(self, hardpoints: list[int], lut_angle: float) -> None:
        """Update the hardpoints and related internal parameters.

        Parameters
        ----------
        hardpoints : `list`
            List of the 0-based hardpoints. There are 6 actuators. The first
            three are the axial actuators and the latter three are the tangent
            links.
        lut_angle : `float`
            Angle used to calculate the LUT forces of gravity component in
            degree.
        """

        self.hardpoints = hardpoints

        self.set_kinetic_decoupling_matrix()

        self.set_hardpoint_compensation()
        self.calc_look_up_forces(lut_angle)

    def simulate_temperature_and_update(
        self,
        temperature_rms: float = 0.05,
        max_temperature: float = 28.0,
        min_temperature: float = 0.0,
    ) -> None:
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
            temperature_change = -np.abs(temperature_change)

        if np.any(np.array(temperature_ring) < min_temperature):
            temperature_change = np.abs(temperature_change)

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

    def set_measured_forces(
        self,
        force_axial: numpy.typing.NDArray[np.float64],
        force_tangent: numpy.typing.NDArray[np.float64],
    ) -> None:
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

    def load_file_lut(self, filepath: Path) -> None:
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

        self._lut["temp_inv"] = MockControlClosedLoop.calc_temp_inv_matrix()

    @staticmethod
    def calc_temp_inv_matrix() -> numpy.typing.NDArray[np.float64]:
        """Calculate the temperature inversion matrix.

        Based on the "LSST_M2_temperature_sensors_20171102.pdf". The sensor map
        is at "doc/figure/temperature_sensor_map.jpg".

        Radial gradient: T(r) = Tr * r / R
        X gradient: T(x) = Tx * x / R = Tx * r * cos(theta) / R
        Y gradient: T(y) = Ty * y / R = Ty * r * sin(theta) / R
        Uniform bias: T(u) = Tu

        The matrix is: [r/R, r/R * cos(theta), r/R * sin(theta), 1], and we
        have: matrix * [Tr, Tx, Ty, Tu].T = [T1, T2, ..., T12].T

        Returns
        -------
        `numpy.ndarray`
            Temperature inversion matrix.
        """

        # Positions of 12 temperature sensors
        # Radial is in millimeter, theta is in degree
        r_sensors = [937.514, 1634.744, 1734.82]
        theta_sensors = [98.5, 278.5, 0.0, 180.0]

        r_normalized = np.array(r_sensors * len(theta_sensors)) / max(r_sensors)
        theta = np.kron(np.array(theta_sensors), np.ones(len(r_sensors)))

        matrix = np.zeros([len(r_normalized), len(theta_sensors)])
        matrix[:, 0] = r_normalized
        matrix[:, 1] = r_normalized * np.cos(np.deg2rad(theta))
        matrix[:, 2] = r_normalized * np.sin(np.deg2rad(theta))
        matrix[:, 3] = 1

        # Do the pseudo-inverse
        return np.linalg.pinv(matrix)

    def load_file_cell_geometry(self, filepath: Path) -> None:
        """Load the file of cell geometry.

        Parameters
        ----------
        filepath : `pathlib.PosixPath`
            File path of cell geometry.
        """

        self._cell_geom = read_yaml_file(filepath)

    def get_actuator_location_axial(self) -> list:
        """Get the location of axial actuators.

        Returns
        -------
        `list`
            Location (x, y) of the axial actuators in meter.
        """
        return self._cell_geom["locAct_axial"]

    def get_actuator_location_tangent(self) -> list:
        """Get the location of tangent actuators.

        Returns
        -------
        `list`
            Location of the tangential actuators in degree.
        """
        return self._cell_geom["locAct_tangent"]

    def get_radius(self) -> float:
        """Get the radius of the tangential actuators.

        Returns
        -------
        `float`
            Radius of the tangential actuators in meter.
        """
        return self._cell_geom["radiusActTangent"]

    def load_file_stiffness(self, filepath: Path) -> None:
        """Load the file of stiffness matrix.

        Parameters
        ----------
        filepath : `pathlib.PosixPath`
            File path of stiffness matrix.
        """

        data = read_yaml_file(filepath)
        self._stiffness = np.array(data["stiff"])

    def set_hardpoint_compensation(self) -> None:
        """Set the hardpoint compensation matrix."""

        (
            hd_comp_axial,
            hd_comp_tangent,
        ) = MockControlClosedLoop.calc_hp_comp_matrix(
            self.get_actuator_location_axial(),
            self.hardpoints[:NUM_HARDPOINTS_AXIAL],
            self.hardpoints[NUM_HARDPOINTS_AXIAL:],
        )
        self._hd_comp = block_diag(hd_comp_axial, hd_comp_tangent)

    @staticmethod
    def calc_hp_comp_matrix(
        location_axial_actuator: list[list],
        hardpoints_axial: list[int],
        hardpoints_tangent: list[int],
    ) -> tuple[numpy.typing.NDArray[np.float64], numpy.typing.NDArray[np.float64]]:
        """Calculate the hardpoint compensation matrix.

        Notes
        -----
        Translate the calculation from the CalcHPFCInfMat.m in
        ts_mtm2_matlab_tools.

        The axial hardpoint compensation matrix (M) fulfills:

        M * (x_hp, y_hp, 1) = (x_nhp, y_nhp, 1)

        x_hp: x-position of hardpoint
        y_hp: y-position of hardpoint
        x_nhp: x-position of non-hardpoint
        y_nhp: y-position of non-hardpoint

        The idea is to make the x-moment amd y-moment keep the same when
        distributes the force of axial hardpoints to other axial actuators.
        It is the same idea for tangential actuators with z-moment.

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

        Returns
        -------
        hd_comp_axial : `numpy.ndarray`
            Axial hardpoint compensation matrix.
        hd_comp_tangent : `numpy.ndarray`
            Tangential hardpoint compensation matrix.
        """

        # Axial hardpoints
        location_axial_actuator_array = np.array(location_axial_actuator)
        matrix_hp = np.append(
            location_axial_actuator_array[hardpoints_axial, :],
            np.ones((NUM_HARDPOINTS_AXIAL, 1)),
            axis=1,
        )

        active_actuators_axial = MockControlClosedLoop.get_active_actuators(
            hardpoints_axial, hardpoints_tangent
        )[0]
        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        matrix_nhp = np.append(
            location_axial_actuator_array[active_actuators_axial, :],
            np.ones((num_axial_actuators - NUM_HARDPOINTS_AXIAL, 1)),
            axis=1,
        )
        hd_comp_axial = matrix_nhp.dot(np.linalg.pinv(matrix_hp))

        # Tangential hardpoints
        option_one = [72, 74, 76]
        option_two = [73, 75, 77]

        if hardpoints_tangent == option_one:
            hd_comp_tangent = np.array(
                [[2 / 3, 2 / 3, -1 / 3], [-1 / 3, 2 / 3, 2 / 3], [2 / 3, -1 / 3, 2 / 3]]
            )
        elif hardpoints_tangent == option_two:
            hd_comp_tangent = np.array(
                [[2 / 3, -1 / 3, 2 / 3], [2 / 3, 2 / 3, -1 / 3], [-1 / 3, 2 / 3, 2 / 3]]
            )

        return hd_comp_axial, hd_comp_tangent

    @staticmethod
    def get_active_actuators(
        hardpoints_axial: list[int],
        hardpoints_tangent: list[int],
    ) -> tuple[list[int], list[int]]:
        """Get the active actuators.

        Parameters
        ----------
        hardpoints_axial : `list`
            Three axial hardpoints. The order is from low to high,
            e.g. [5, 15, 25].
        hardpoints_tangent : `list`
            Three tangential hardpoints. This can only be [72, 74, 76] or
            [73, 75, 77]. The order is from low to high.

        Returns
        -------
        active_actuators_axial : `list`
            Axial active actuators.
        active_actuators_tangent : `list`
            Tangential active actuators.

        Raises
        ------
        `ValueError`
            If the tangential hardpoints are wrong.
        """

        # Axial hardpoints
        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        active_actuators_axial = [
            idx for idx in range(num_axial_actuators) if idx not in hardpoints_axial
        ]

        # Tangent hardpoints
        option_one = [72, 74, 76]
        option_two = [73, 75, 77]

        if hardpoints_tangent == option_one:
            active_actuators_tangent = option_two
        elif hardpoints_tangent == option_two:
            active_actuators_tangent = option_one
        else:
            raise ValueError(
                f"Tangential hardpoints can only be {option_one} or {option_two}."
            )

        return active_actuators_axial, active_actuators_tangent

    def set_kinetic_decoupling_matrix(self) -> None:
        """Set the kinetic decoupling matrix."""

        self._kdc = MockControlClosedLoop.calc_kinetic_decoupling_matrix(
            self.get_actuator_location_axial(),
            self.hardpoints[:NUM_HARDPOINTS_AXIAL],
            self.hardpoints[NUM_HARDPOINTS_AXIAL:],
            self._stiffness,
        )

    @staticmethod
    def calc_kinetic_decoupling_matrix(
        location_axial_actuator: list[list],
        hardpoints_axial: list[int],
        hardpoints_tangent: list[int],
        stiffness_matrix: numpy.typing.NDArray[np.float64],
    ) -> numpy.typing.NDArray[np.float64]:
        """Calculate the kinetic decoupling matrix.

        Notes
        -----
        Translate the calculation from the CalcHPFCInfMat.m in
        ts_mtm2_matlab_tools.

        The unit of the element is the motor step (could be fraction, row) per
        Newton (column). This matrix is used to multiply with the desired
        actuator's force changes to get the related amounts of motor's step
        changes.

        delta_force = M * delta_step => M^-1 = kdc to have:
        delta_step = kdc * delta_force

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
        stiffness_matrix : `numpy.ndarray`
            Stiffness matrix of M2 mirror or surrogate.

        Returns
        -------
        `numpy.ndarray`
            Kinetic decoupling matrix.
        """

        # Calculate the hardpoint force error
        (
            active_actuators_axial,
            active_actuators_tangent,
        ) = MockControlClosedLoop.get_active_actuators(
            hardpoints_axial, hardpoints_tangent
        )

        # There is a "minus" sign here because the hardpoints are the passive
        # actuators.
        hardpoint_error_axial = -stiffness_matrix[
            np.ix_(active_actuators_axial, hardpoints_axial)
        ]
        hardpoint_error_tangent = -stiffness_matrix[
            np.ix_(active_actuators_tangent, hardpoints_tangent)
        ]

        # Force change of the active actuators
        force_axial = stiffness_matrix[
            np.ix_(active_actuators_axial, active_actuators_axial)
        ]
        force_tangent = stiffness_matrix[
            np.ix_(active_actuators_tangent, active_actuators_tangent)
        ]

        # Calculate the effective force feedback after hardpoint force
        # compensation
        hd_comp_axial, hd_comp_tangent = MockControlClosedLoop.calc_hp_comp_matrix(
            location_axial_actuator, hardpoints_axial, hardpoints_tangent
        )
        force_feedback_axial = force_axial + hd_comp_axial.dot(hardpoint_error_axial.T)
        force_feedback_tangent = force_tangent + hd_comp_tangent.dot(
            hardpoint_error_tangent.T
        )

        force_feedback = block_diag(force_feedback_axial, force_feedback_tangent)

        return np.linalg.pinv(force_feedback)

    @staticmethod
    def calc_cmd_prefilter_params(
        numerator: list[float] | None = None, denominator: list[float] | None = None
    ) -> tuple[float, numpy.typing.NDArray[np.float64]]:
        """Calculate the command pre-filter parameters in biquadratic filters.

        Notes
        -----
        Translate the calculation from the Create_Filter_Params.m in
        ts_mtm2_matlab_tools.

        Currently, pre-filter is a pass-through filter.

        Parameters
        ----------
        numerator : `list` or None, optional
            Numerator polynomial coefficients of the transfer function. If
            None, the linear transfer is assumed (aka. [1.0]). (the default is
            None)
        denominator : `list` or None, optional
            Denominator polynomial coefficients of the transfer function. If
            None, the linear transfer is assumed (aka. [1.0]). (the default is
            None)

        Returns
        -------
        `float`
            Gain.
        `list`
            Coefficients of the command pre-filter parameters.
        """

        numerator = numerator if (numerator is not None) else [1.0]
        denominator = denominator if (denominator is not None) else [1.0]

        return MockControlClosedLoop.transfer_function_to_biquadratic_filter(
            numerator, denominator
        )

    @staticmethod
    def transfer_function_to_biquadratic_filter(
        numerator: list[float], denominator: list[float], num_stage: int = 8
    ) -> tuple[float, numpy.typing.NDArray[np.float64]]:
        """Transfer function to the biquadratic filters.

        Notes
        -----
        Translate the calculation from the bqd.m and sos2pqd.m in
        ts_mtm2_matlab_tools.

        The reference is:
        https://en.wikipedia.org/wiki/Digital_biquad_filter

        Compact biquadratic format:

                       1 + b11 z^-1 + b21 z^-2   1 + b12 z^-1 + b22 z^-2
        H(z) = gain * (-----------------------) (-----------------------) ...
                       1 + a11 z^-1 + a21 z^-2   1 + a12 z^-1 + a22 z^-2

                1 + b1N z^-1 + b2N z^-2
               (-----------------------)
                1 + a1N z^-1 + a2N z^-2

        Format of the coefficent output is:
        [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N b2N]

        Parameters
        ----------
        numerator : `list`
            Numerator polynomial coefficients of the transfer function.
        denominator : `list`
            Denominator polynomial coefficients of the transfer function.
        num_stage : `int`, optional
            Number of the stage of biquadratic filters. (the default is 8)

        Returns
        -------
        gain : `float`
            Gain.
        coefficients : `list`
            Coefficients of the biquadratic filters.
        """

        # The idea is:
        # transfer function (TF) -> second-order sections (SOS)
        # -> biquadratic filters (BQD)

        # SOS is an L by 6 matrix with the following structure:
        #     SOS = [ b01 b11 b21 1 a11 a21
        #             b02 b12 b22 1 a12 a22
        #             ...
        #             b0L b1L b2L 1 a1L a2L ]

        # Each row of the SOS matrix describes a 2nd order transfer function:
        #               b0k + b1k * z^-1 + b2k * z^-2
        #     Hk(z) =  ----------------------------
        #                1 + a1k * z^-1 + a2k * z^-2
        # where k is the row index.

        sos = signal.tf2sos(numerator, denominator)

        # Shift the numerator coefficients when there is a delay
        num_sections = sos.shape[0]
        for idx in range(num_sections):
            if sos[idx, 0] == 0.0:
                if sos[idx, 1] != 0.0:
                    # [0, b1k, b2k] -> [b1k, b2k, 0]
                    sos[idx, 0:3] = np.roll(sos[idx, 0:3], -1)
                else:
                    # [0, 0, b2k] -> [b2k, 0, 0]
                    sos[idx, 0:3] = np.roll(sos[idx, 0:3], -2)

        # Calculate the gain and normalize the second-order sections
        b0 = sos[:, 0]
        a0 = sos[:, 3]

        gain = np.prod(np.divide(b0, a0))

        sos_normalize = np.append(
            np.diag(1 / b0).dot(sos[:, 0:3]), np.diag(1 / a0).dot(sos[:, 3:6]), axis=1
        )

        # Change to the order to be:
        # [[a11 a21 b11 b21]
        #  [a12 a22 b12 b22]
        #  ...
        #  [a1N a2N b1N b2N]]
        coefficients = sos_normalize[:, [4, 5, 1, 2]]

        # Pad the coeffients with 0 to fulfill the number of filter stages

        # Note the constant "4 = 2 * 2" comes from:
        # 1. numerator + denominator in biquadratic filter
        # 2. 2 degrees in biquadratic filter
        max_num_coefficients = num_stage * 4

        return (
            gain,
            np.pad(
                coefficients.reshape(-1), (0, max_num_coefficients - coefficients.size)
            ).tolist(),
        )

    @staticmethod
    def calc_cmd_delay_filter_params(
        is_mirror: bool = True,
        control_frequency: float = 20.0,
        bypass_delay: bool = False,
        num_degree: int = 5,
    ) -> list[float]:
        """Calculate the command delay filter parameters (or more clearly, the
        numerator of the transfer function).

        Notes
        -----
        Translate the calculation from the Create_Filter_Params.m in
        ts_mtm2_matlab_tools.

        For a transfer function, H(z), if the numerator is [1, 3, 3] and the
        denominator is [1, 2, 1], it will be:

        H(z) = (z^2 + 3 * z + 3) / (z^2 + 2 * z + 1)

        Since the denominator of transfer function is the demanded force
        change here, the linear transfer ([1.0]) is applied. Therefore, we only
        care about the delay of command as the output.

        Parameters
        ----------
        is_mirror : `bool`, optional
            Is mirror or not. If not, the surrogate is applied. (the default is
            True)
        control_frequency : `float`, optional
            Frequency of the control loop in Hz. (the default is 20.0)
        bypass_delay : `bool`, optional
            Bypass the command delay or not. (the default is False)
        num_degree : `int`, optional
            Number of the degree in the transfer function. (the default is 5)

        Returns
        -------
        numerator_transfer_function : `list`
            Numerator of the transfer function from the high degree to low
            degree.
        """
        time_sample = 1 / control_frequency

        if bypass_delay:
            time_delay = 0.0
        else:
            # Note the following two constants come from the
            # Create_Filter_Params.m and I do not understand why.
            if is_mirror:
                # Note the delay time of mirror is shorter than the surrogate
                time_delay = 0.0993
            else:
                time_delay = 0.1016

        num_sample_delay = int(time_delay / time_sample)
        time_left = time_delay - num_sample_delay * time_sample

        coeff_delay = [0.0] * num_sample_delay + [
            (1 - time_left / time_sample),
            time_left / time_sample,
        ]

        # 1 is for the constant part in the numerator coefficients
        numerator_transfer_function = [0.0] * (num_degree + 1)
        for idx, value in enumerate(coeff_delay):
            numerator_transfer_function[idx] = value

        return numerator_transfer_function

    @staticmethod
    def calc_force_control_filter_params(
        is_mirror: bool = True,
        numerator: list[float] | None = None,
        denominator: list[float] | None = None,
    ) -> tuple[float, numpy.typing.NDArray[np.float64]]:
        """Calculate the force control filter parameters in biquadratic
        filters.

        Notes
        -----
        Translate the calculation from the Create_Filter_Params.m in
        ts_mtm2_matlab_tools.

        Parameters
        ----------
        is_mirror : `bool`, optional
            Is mirror or not. If not, the surrogate is applied. (the default is
            True)
        numerator : `list` or None, optional
            Numerator polynomial coefficients of the transfer function. If
            None, the linear transfer is assumed (aka. [1.0]). (the default is
            None)
        denominator : `list` or None, optional
            Denominator polynomial coefficients of the transfer function. If
            None, the linear transfer is assumed (aka. [1.0]). (the default is
            None)

        Returns
        -------
        `float`
            Gain.
        coefficients : `list`
            Coefficients of the force control filter parameters.
        """

        # Note the following four constants come from the
        # Create_Filter_Params.m and I do not understand why.
        basic_gain = 0.285
        gain = basic_gain * 1.0593 * 1.0902 if is_mirror else basic_gain * 1.1885

        numerator = numerator if (numerator is not None) else [1.0]
        denominator = denominator if (denominator is not None) else [1.0]

        (
            gain_tf,
            coefficients,
        ) = MockControlClosedLoop.transfer_function_to_biquadratic_filter(
            numerator, denominator
        )

        return gain * gain_tf, coefficients

    @staticmethod
    def rigid_body_to_actuator_displacement(
        location_axial_actuator: list[list],
        location_tangent_link: list[float],
        radius: float,
        dx: float,
        dy: float,
        dz: float,
        drx: float,
        dry: float,
        drz: float,
    ) -> numpy.typing.NDArray[np.float64]:
        """Calculate the actuator displacements based on the rigid body
        position.

        Notes
        -----
        Translate the calculation from the RigidBodyToActuatorDisplacement.vi
        in ts_mtm2. For the "radius", the original developer had the
        following comment:

        An average of 5 of the 6 tangent location radius from the center of
        the M2 cell which is to be used by the rigid body to displacement
        calculation. Technically, it is not the B-ring radius and this should
        be changed in the future.

        Parameters
        ----------
        location_axial_actuator : `list [list]`
            Location of the axial actuators: (x, y) in meter. This should be a
            (NUM_ACTUATOR - NUM_TANGENT_LINK) x 2 matrix.
        location_tangent_link : `list`
            Location of the tangent links in degree. This should be a
            1 x NUM_TANGENT_LINK array.
        radius : `float`
            Radius of the cell in meter.
        dx : `float`
            Delta x position in meter.
        dy : `float`
            Delta y position in meter.
        dz : `float`
            Delta z position in meter.
        drx : `float`
            Delta x rotator in radian.
        dry : `float`
            Delta y rotator in radian.
        drz : `float`
            Delta z rotator in radian.

        Returns
        -------
        `numpy.ndarray`
            All actuator displacements in meter.
        """

        # Consider the displacement from (dx, dy, dz)

        # Transformation matrix. The raw is each actuator's displacement. The
        # column is the (x, y, z) movement.
        matrix_trans = np.zeros((NUM_ACTUATOR, 3))

        # A positive axial displacement results in a negative change in piston.
        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        matrix_trans[:num_axial_actuators, 2] = -1

        for idx, angle in enumerate(location_tangent_link):
            matrix_trans[num_axial_actuators + idx, 0] = np.cos(np.deg2rad(angle))
            matrix_trans[num_axial_actuators + idx, 1] = -np.sin(np.deg2rad(angle))

        # Translate the (dx, dy, dz) motion to 78 actuator displacement.
        disp_dxdydz = matrix_trans.dot(np.array([dx, dy, dz]))

        # Consider the displacement from (drx, dry, drz)

        location_axial = np.append(
            np.array(location_axial_actuator),
            np.zeros((num_axial_actuators, 1)),
            axis=1,
        )

        # Rotation matrix for the axial actuators
        rot_x = np.array(
            [[1, 0, 0], [0, np.cos(drx), np.sin(drx)], [0, -np.sin(drx), np.cos(drx)]]
        )
        rot_y = np.array(
            [[np.cos(dry), 0, -np.sin(dry)], [0, 1, 0], [np.sin(dry), 0, np.cos(dry)]]
        )
        rot_z = np.array(
            [[np.cos(drz), np.sin(drz), 0], [-np.sin(drz), np.cos(drz), 0], [0, 0, 1]]
        )
        rot_xyz = rot_x.dot(rot_y).dot(rot_z)

        disp_drxdrydrz_axial = rot_xyz.dot(location_axial.T)
        disp_drz_axial = disp_drxdrydrz_axial[2, :]

        # Displacement drz, note there is a "-1" here for the direction of
        # coordination system.
        disp_drz_tangent = -radius * np.sin(drz) * np.ones(len(location_tangent_link))

        disp_drz = np.append(disp_drz_axial, disp_drz_tangent)

        return disp_dxdydz + disp_drz

    @staticmethod
    def hardpoint_to_rigid_body(
        location_axial_actuator: list[list],
        location_tangent_link: list[float],
        radius: float,
        hardpoints: list[int],
        disp_hardpoint_current: list[float],
        disp_hardpoint_home: list[float],
    ) -> tuple[float, float, float, float, float, float]:
        """Calculate the rigid body position based on the hardpoint
        displacements.

        Notes
        -----
        Translate the calculation from the HardPointToRigidBody.vi in ts_mtm2.

        Parameters
        ----------
        location_axial_actuator : `list [list]`
            Location of the axial actuators: (x, y) in meter. This should be a
            (NUM_ACTUATOR - NUM_TANGENT_LINK) x 2 matrix.
        location_tangent_link : `list`
            Location of the tangent links in degree. This should be a
            1 x NUM_TANGENT_LINK array.
        radius : `float`
            Radius of the cell in meter.
        hardpoints : `list`
            Six hardpoints. The order is from low to high.
        disp_hardpoint_current : `list`
            Six current hardpoint displacements. The unit is meter.
        disp_hardpoint_home : `list`
            Six hardpoint displacements at the home position. The unit is
            meter.

        Returns
        -------
        x : `float`
            X position in meter.
        y : `float`
            Y position in meter.
        z : `float`
            Z position in meter.
        rx : `float`
            X rotator in radian.
        ry : `float`
            Y rotator in radian.
        rz : `float`
            Z rotator in radian.
        """

        # Delta of the axial hardpoint displacements
        disp_hardpoint_axial = (
            np.array(disp_hardpoint_current)[:NUM_HARDPOINTS_AXIAL]
            - np.array(disp_hardpoint_home)[:NUM_HARDPOINTS_AXIAL]
        )

        # Location of the axial hardpoints: (x_hp, y_hp, 1)
        location_axial_actuator_array = np.array(location_axial_actuator)
        location_axial_hardpoint = np.append(
            location_axial_actuator_array[hardpoints[:NUM_HARDPOINTS_AXIAL], :],
            np.ones((NUM_HARDPOINTS_AXIAL, 1)),
            axis=1,
        )

        # The displacement of axial actuators decides the (z, rx, ry) of rigid
        # body.
        dof_axial = np.linalg.inv(location_axial_hardpoint).dot(disp_hardpoint_axial)

        # Note the negative piston means the +z
        z = -dof_axial[2]

        # Calculate the (rx, ry) in radian
        mat_axial_3_points = np.array([[0, 0, 1], [1, 0, 1], [0, 1, 1]])
        z_axial_3_points = mat_axial_3_points.dot(dof_axial)

        mat_axial_3_points_updated = mat_axial_3_points.copy().astype(float)
        mat_axial_3_points_updated[:, 2] = z_axial_3_points

        r1c = mat_axial_3_points_updated[1, :] - mat_axial_3_points_updated[0, :]
        r2 = mat_axial_3_points_updated[2, :] - mat_axial_3_points_updated[0, :]
        cross_r1c_r2 = np.cross(r1c, r2)
        cross_r1c_r2_normalized = cross_r1c_r2 / np.linalg.norm(cross_r1c_r2)

        vector_rxryrz = np.cross(np.array([0, 0, 1]), cross_r1c_r2_normalized)
        norm_vector_rxryrz = np.linalg.norm(vector_rxryrz)
        if norm_vector_rxryrz == 0:
            rx = 0
            ry = 0
        else:
            rx, ry = -(
                vector_rxryrz[0:2] / norm_vector_rxryrz * np.arcsin(norm_vector_rxryrz)
            )

        # The displacement of tangent links decides the (x, y, rz) of rigid
        # body.
        num_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
        idx_tangent_hardpoint = (
            np.array(hardpoints)[NUM_HARDPOINTS_AXIAL:] - num_axial_actuators
        )
        location_tangent_hardpoint = np.array(np.deg2rad(location_tangent_link))[
            idx_tangent_hardpoint.astype(int)
        ]

        # Calculate the (x, y) in meter
        (
            x_current,
            y_current,
            delta_xy_current,
        ) = MockControlClosedLoop.calculate_rigid_body_xy(
            radius,
            disp_hardpoint_current[NUM_HARDPOINTS_AXIAL:],
            location_tangent_hardpoint,
        )

        x_home, y_home, delta_xy_home = MockControlClosedLoop.calculate_rigid_body_xy(
            radius,
            disp_hardpoint_home[NUM_HARDPOINTS_AXIAL:],
            location_tangent_hardpoint,
        )

        # The original developer comments the following in LabVIEW:
        # Need to compensate the X-Y displacements by scaling by 2 in order to
        # calculate the correct displacements as measured optically.
        x = 2 * (x_current - x_home)
        y = 2 * (y_current - y_home)

        # Calculate the rz in radian

        # The details can follow:
        # Least-Squares Rigid Motion Using SVD by Olga Sorkine-Hornung and
        # Michael Rabinovich, 2017
        # https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
        mat_u, _, mat_v = np.linalg.svd(
            delta_xy_home.T.dot(delta_xy_current), full_matrices=True
        )

        # Consider the potential reflection when calculating the rotation
        # matrix
        mat_det_vut = np.array([[1, 0], [0, numpy.linalg.det(mat_v.dot(mat_u.T))]])
        mat_r = mat_v.dot(mat_det_vut).dot(mat_u.T)

        # mat_r is the rotation matrix of rz, based on the sign of cos(rz) to
        # decide the direction of rotation.
        rz = np.arctan2(np.sign(mat_r[0, 0]) * mat_r[1, 0], abs(mat_r[0, 0]))

        return x, y, z, rx, ry, rz

    @staticmethod
    def calculate_rigid_body_xy(
        radius: float,
        tangent_hardpoint_displacement: list[float],
        tangent_hardpoint_location: list[float],
    ) -> tuple[float, float, numpy.typing.NDArray[np.float64]]:
        """Calculate the rigid body (x, y) position.

        Notes
        -----
        This is part of the calculation in HardPointToRigidBody.vi in ts_mtm2.

        Parameters
        ----------
        radius : `float`
            Radius of the cell in meter.
        tangent_hardpoint_displacement : `list`
            Displacement of the 3 tangent hardpoints in meter.
        tangent_hardpoint_location : `list`
            Location of the 3 tangent hardpoints in radian.

        Returns
        -------
        mean_x : `float`
            X position in meter.
        mean_y : `float`
            Y position in meter.
        delta_xy_tangent_hardpoint : `numpy.ndarray`
            Delta (x, y) position compared with the mean (x, y) in meter.
        """

        # d: actuator displacement
        # R: radius
        # theta, theta_0: angles of tangent links at current and home positions
        # d = R * tan(theta - theta_0)
        # => theta = theta_0 + tan^(-1)(d/R)
        theta = (
            np.arctan2(tangent_hardpoint_displacement, radius)
            + tangent_hardpoint_location
        )

        # x = R * cos(pi/2 - theta) = R * sin(theta)
        # y = R * sin(pi/2 - theta) = R * cos(theta)
        x_tangent_hardpoint = radius * np.sin(theta)
        y_tangent_hardpoint = radius * np.cos(theta)

        mean_x = np.mean(x_tangent_hardpoint)
        mean_y = np.mean(y_tangent_hardpoint)

        num = len(tangent_hardpoint_displacement)
        delta_xy_tangent_hardpoint = np.append(
            np.reshape(x_tangent_hardpoint - mean_x, (num, 1)),
            np.reshape(y_tangent_hardpoint - mean_y, (num, 1)),
            axis=1,
        )

        return mean_x, mean_y, delta_xy_tangent_hardpoint

    def is_cell_temperature_high(self) -> bool:
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

    def apply_forces(
        self,
        force_axial: list[float] | numpy.typing.NDArray[np.float64],
        force_tangent: list[float] | numpy.typing.NDArray[np.float64],
    ) -> None:
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
        self,
        applied_force_axial: numpy.typing.NDArray[np.float64] | None = None,
        applied_force_tangent: numpy.typing.NDArray[np.float64] | None = None,
        use_measured_force: bool = False,
    ) -> tuple[bool, list, list]:
        """The actuator force is out of limit or not.

        Notes
        -----
        Remove the 'use_measured_force' after we implement the forward modeling
        in the control algorithm. At that time, the measured force should be
        consistent with the demanded force (with the consideration of hardpoint
        correction).

        Parameters
        ----------
        applied_force_axial : `numpy.ndarray` or `None`, optional
            Axial forces to apply in Newton. If `None` use current setup. (the
            default is None.)
        applied_force_tangent : `numpy.ndarray` or `None`, optional
            Tangent forces to apply in Newton. If `None` use current setup. (
            the default is None.)
        use_measured_force : `bool`, optional
            Use the measured force to compare with the limit. (the default is
            False)

        Returns
        -------
        `bool`
            True if the actuator force is out of limit. Otherwise, False.
        `list`
            Triggered retracted limit switches.
        `list`
            Triggered extended limit switches.
        """

        force_to_check = (
            np.append(self.axial_forces["measured"], self.tangent_forces["measured"])
            if use_measured_force
            else self.get_demanded_force(
                applied_force_axial=applied_force_axial,
                applied_force_tangent=applied_force_tangent,
            )
        )

        return check_limit_switches(
            force_to_check,
            LIMIT_FORCE_AXIAL_CLOSED_LOOP,
            LIMIT_FORCE_TANGENT_CLOSED_LOOP,
        )

    def get_demanded_force(
        self,
        applied_force_axial: numpy.typing.NDArray[np.float64] | None = None,
        applied_force_tangent: numpy.typing.NDArray[np.float64] | None = None,
    ) -> numpy.typing.NDArray[np.float64]:
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

    def reset_force_offsets(self) -> None:
        """Reset the force offsets.

        This will put the applied force to be zero.
        """

        self.axial_forces["applied"] = np.zeros(NUM_ACTUATOR - NUM_TANGENT_LINK)
        self.tangent_forces["applied"] = np.zeros(NUM_TANGENT_LINK)

    def get_net_forces_total(self) -> dict:
        """Get the total net forces in Newton.

        Returns
        -------
        `dict`
            Total net forces in Newton.
        """
        return self._calculate_xyz_net_forces(
            self.axial_forces["measured"], self.tangent_forces["measured"]
        )

    def _calculate_xyz_net_forces(
        self,
        axial_forces: numpy.typing.NDArray[np.float64],
        tangent_forces: numpy.typing.NDArray[np.float64],
    ) -> dict:
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

        angle = np.deg2rad(self.get_actuator_location_tangent())

        fx = np.sum(np.cos(angle) * tangent_forces)
        fy = np.sum(np.sin(angle) * tangent_forces)
        fz = np.sum(axial_forces)

        return {"fx": fx, "fy": fy, "fz": fz}

    def get_net_moments_total(self) -> dict:
        """Get the total net moments in Newton * meter.

        Returns
        -------
        `dict`
            Total net moments in Newton * meter.
        """
        return self._calculate_xyz_net_moments(
            self.axial_forces["measured"], self.tangent_forces["measured"]
        )

    def _calculate_xyz_net_moments(
        self,
        axial_forces: numpy.typing.NDArray[np.float64],
        tangent_forces: numpy.typing.NDArray[np.float64],
    ) -> dict:
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

        location_axial_actuator = np.array(self.get_actuator_location_axial())
        mx = np.sum(axial_forces * location_axial_actuator[:, 1])
        my = np.sum(axial_forces * location_axial_actuator[:, 0])
        mz = np.sum(tangent_forces) * self.get_radius()

        return {"mx": mx, "my": my, "mz": mz}

    def get_force_balance(self) -> dict:
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

    def calc_look_up_forces(self, lut_angle: float) -> None:
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
            temperature_lut, np.array(self.temperature["ref"])
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

        # Use the negative sign here as requested by scientist.
        # This is different from the calculation in ts_mtm2_cell with a
        # negative sign.
        # The benefit is to make the understanding of force's details easier.
        self.axial_forces["hardpointCorrection"] = -force_hardpoint[
            :num_axial_actuators
        ]
        self.tangent_forces["hardpointCorrection"] = -force_hardpoint[
            -NUM_TANGENT_LINK:
        ]

    def _calc_look_up_forces_temperature(
        self,
        lut_temperature: numpy.typing.NDArray[np.float64],
        ref_temperature: numpy.typing.NDArray[np.float64],
    ) -> tuple[
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
    ]:
        """Calculate the temperature-related forces based on the look-up table
        (LUT) in Newton.

        Only the axial actuators have this correction.

        Parameters
        ----------
        lut_temperature : `numpy.ndarray`
            Temperature used to calculate the LUT forces of temperature
            component in degree C. The order is: [LG2-2, LG2-3, LG2-4,
            LG3-1, LG4-2, LG4-1, LG3-2, LG3-3, LG3-4, LG4-4, LG4-3, LG2-1].
        ref_temperature : `numpy.ndarray`
            Reference temperature in degree C. The order is [LG2-1, LG2-2,
            LG2-3, LG2-4, LG3-1, LG3-2, LG3-3, LG3-4, LG4-1, LG4-2, LG4-3,
            LG4-4].

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

        indexs = [11, 0, 1, 2, 3, 6, 7, 8, 5, 4, 10, 9]
        temperature = lut_temperature - ref_temperature[indexs]
        tcoef = self._lut["temp_inv"].dot(temperature.reshape(-1, 1))

        return (
            np.squeeze(tcoef[0] * self._lut["Tr"]),
            np.squeeze(tcoef[1] * self._lut["Tx"]),
            np.squeeze(tcoef[2] * self._lut["Ty"]),
            np.squeeze(tcoef[3] * self._lut["Tu"]),
        )

    def _calc_look_up_forces_gravity(
        self, lut_angle: float
    ) -> tuple[
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
    ]:
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

    def _calc_look_up_forces_hardpoint(
        self,
        force_demanded: numpy.typing.NDArray[np.float64],
        force_measured: numpy.typing.NDArray[np.float64],
    ) -> numpy.typing.NDArray[np.float64]:
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

    def handle_forces(self, force_rms: float = 0.5, force_per_cycle: float = 5) -> bool:
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

    def _force_dynamics(
        self, force_rms: float, force_per_cycle: float
    ) -> tuple[bool, numpy.typing.NDArray[np.float64]]:
        """Handle the force dynamics.

        The method works by comparing the demanded forces with the current
        forces and hardpoint correction. For each actuator, if the force
        differs by less than the amount of "force_per_cycle" then the
        current value is set to the demand value minus the hardpoint
        correction. If the difference is larger, the current value will be
        modified by the amount of "force_per_cycle".

        The target is:
        |force_demanded + hardpoints - force_measured| < force_error

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

            actuators_in_cycle_no_hardpoint_list = np.array(
                list(actuators_in_cycle_no_hardpoint), dtype=int
            )

            final_force[actuators_in_cycle_no_hardpoint_list] = (
                force_demanded[actuators_in_cycle_no_hardpoint_list]
                + force_hardpoint[actuators_in_cycle_no_hardpoint_list]
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

    def _get_force_error(
        self,
    ) -> tuple[
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
    ]:
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
        force_error = force_demanded + force_hardpoint - force_measured

        # Do not consider the force error in hardpoints.
        # This is the logic in M2 cell LabVIEW code by vendor. Need to figure
        # out why it was designed in this way in a latter time.
        force_error[self.hardpoints] = 0

        return force_error, force_demanded, force_hardpoint, force_measured
