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

__all__ = ["MockControlLoop"]

import numpy as np
import numpy.typing

from ..biquadratic_filter import BiquadraticFilter
from ..constant import NUM_ACTUATOR, NUM_HARDPOINTS_AXIAL, NUM_TANGENT_LINK
from ..simple_delay_filter import SimpleDelayFilter
from .mock_deadband_control import MockDeadbandControl
from .mock_gain_schedular import MockGainSchedular
from .mock_in_position import MockInPosition


class MockControlLoop:
    """Mock control loop that implements the force control algorithm.

    Notes
    -----
    This class is translated from vendor's original LabVIEW code:
    "Control Loop (FIFO).vi" in ts_mtm2_cell.

    Parameters
    ----------
    gain_prefilter_axial : `float`
        Gain of the prefilter for the axial actuators.
    params_prefilter_axial : `numpy.ndarray`
        Parameters of the prefilter for the axial actuators:
        [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N b2N].
    gain_prefilter_tangent : `float`
        Gain of the prefilter for the tangent actuators.
    params_prefilter_tangent : `numpy.ndarray`
        Parameters of the prefilter for the tangent actuators:
        [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N b2N].
    params_cmd_delay_axial : `list`
        Parameters of the command delay filter for the axial actuators:
        [b0, b1, b2, ..., bN].
    params_cmd_delay_tangent : `list`
        Parameters of the command delay filter for the tangent actuators:
        [b0, b1, b2, ..., bN].
    gain_control_filter_axial : `float`
        Gain of the control filter for the axial actuators.
    params_control_filter_axial : `numpy.ndarray`
        Parameters of the control filter for the axial actuators:
        [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N b2N].
    gain_control_filter_tangent : `float`
        Gain of the control filter for the tangent actuators.
    params_control_filter_tangent : `numpy.ndarray`
        Parameters of the control filter for the tangent actuators:
        [a11 a21 b11 b21 a12 a22 b12 b22 ... a1N a2N b1N b2N].
    kdc : `numpy.ndarray`
        Decoupling matrix.
    kinfl : `numpy.ndarray`
        Influence matrix.
    hd_comp : `numpy.ndarray`
        Hardpoint compensation matrix.
    thresholds_deadzone_axial : `list`
        The [lower, upper] thresholds of the axial hardpoint error in deadzone.
        The unit is Newton.
    thresholds_deadzone_tangent : `list`
        The [lower, upper] thresholds of the tangent hardpoint error in
        deadzone. The unit is Newton.
    min_gain_schedular_axial : `float`
        Minimum gain for the axial actuators in the gain schedular. The value
        should be in (0.0, 1.0).
    min_gain_schedular_tangent : `float`
        Minimum gain for the tangent actuators in the gain schedular. The value
        should be in (0.0, 1.0).
    num_sample_ramp_up : `int`
        Number of samples in the ramping up process in the gain schedular. This
        value can not be 0.
    num_sample_ramp_down : `int`
        Number of samples in the ramping down process in the gain schedular.
        This value can not be 0.
    max_sample_settle : `int`
        Maximum number of samples in the settling process in the gain
        schedular. This value can not be 0.
    step_limit_axial : `int`
        Step limit of the axial actuator in each control cycle.
    step_limit_tangent : `int`
        Step limit of the tangent actuator in each control cycle.
    in_position_window_size : `int`
        Window size in second to judge the mirror is in position or not.
    in_position_control_frequency : `float`
        Control frequency in Hz to judge the mirror is in position or not.
    in_position_threshold_axial : `float`
        Threshold of the force error of axial actuator in Newton to judge the
        mirror is in position or not.
    in_position_threshold_tangent : `float`
        Threshold of the force error of tangent actuator in Newton to judge the
        mirror is in position or not.
    is_feedforward : `bool`, optional
        The feedforward is on or not. (the default is True)
    is_feedback : `bool`, optional
        The feedback is on or not. (the default is True)

    Attributes
    ----------
    is_feedforward : `bool`
        The feedforward is on or not.
    is_feedback : `bool`
        The feedback is on or not.
    kdc : `numpy.ndarray`
        Decoupling matrix.
    kinfl : `numpy.ndarray`
        Influence matrix.
    hd_comp : `numpy.ndarray`
        Hardpoint compensation matrix.
    """

    def __init__(
        self,
        gain_prefilter_axial: float,
        params_prefilter_axial: numpy.typing.NDArray[np.float64],
        gain_prefilter_tangent: float,
        params_prefilter_tangent: numpy.typing.NDArray[np.float64],
        params_cmd_delay_axial: list[float],
        params_cmd_delay_tangent: list[float],
        gain_control_filter_axial: float,
        params_control_filter_axial: numpy.typing.NDArray[np.float64],
        gain_control_filter_tangent: float,
        params_control_filter_tangent: numpy.typing.NDArray[np.float64],
        kdc: numpy.typing.NDArray[np.float64],
        kinfl: numpy.typing.NDArray[np.float64],
        hd_comp: numpy.typing.NDArray[np.float64],
        thresholds_deadzone_axial: list[float],
        thresholds_deadzone_tangent: list[float],
        min_gain_schedular_axial: float,
        min_gain_schedular_tangent: float,
        num_sample_ramp_up: int,
        num_sample_ramp_down: int,
        max_sample_settle: int,
        step_limit_axial: int,
        step_limit_tangent: int,
        in_position_window_size: int,
        in_position_control_frequency: float,
        in_position_threshold_axial: float,
        in_position_threshold_tangent: float,
        is_feedforward: bool = True,
        is_feedback: bool = True,
    ) -> None:
        num_axial = NUM_ACTUATOR - NUM_TANGENT_LINK
        num_coeff = 4

        self._prefilter_axial = BiquadraticFilter(
            gain_prefilter_axial,
            params_prefilter_axial.reshape(-1, num_coeff),
            num_axial,
        )
        self._prefilter_tangent = BiquadraticFilter(
            gain_prefilter_tangent,
            params_prefilter_tangent.reshape(-1, num_coeff),
            NUM_TANGENT_LINK,
        )

        self._command_delay_axial = SimpleDelayFilter(params_cmd_delay_axial, num_axial)
        self._command_delay_tangent = SimpleDelayFilter(
            params_cmd_delay_tangent, NUM_TANGENT_LINK
        )

        self._control_filter_axial = BiquadraticFilter(
            gain_control_filter_axial,
            params_control_filter_axial.reshape(-1, num_coeff),
            num_axial - NUM_HARDPOINTS_AXIAL,
        )
        self._control_filter_tangent = BiquadraticFilter(
            gain_control_filter_tangent,
            params_control_filter_tangent.reshape(-1, num_coeff),
            NUM_TANGENT_LINK - NUM_HARDPOINTS_AXIAL,
        )

        # H(z) = 1 - z^(-1)
        self._feedforward_delay_filter = SimpleDelayFilter([1.0, -1.0], num_axial)

        self.kdc = kdc

        self.kinfl = kinfl

        self.hd_comp = hd_comp

        self._deadband_control_axial = MockDeadbandControl(
            thresholds_deadzone_axial[0], thresholds_deadzone_axial[1]
        )
        self._deadband_control_tangent = MockDeadbandControl(
            thresholds_deadzone_tangent[0], thresholds_deadzone_tangent[1]
        )

        self._gain_schedular = MockGainSchedular(
            min_gain_schedular_axial,
            min_gain_schedular_tangent,
            num_sample_ramp_up,
            num_sample_ramp_down,
            max_sample_settle,
        )

        self._step_limit_axial = step_limit_axial
        self._step_limit_tangent = step_limit_tangent

        self._in_position = MockInPosition(
            in_position_window_size,
            in_position_control_frequency,
            in_position_threshold_axial,
            in_position_threshold_tangent,
        )

        self.is_feedforward = is_feedforward
        self.is_feedback = is_feedback

    def reset(self, reset_all: bool = False) -> None:
        """ "Reset the history.

        Parameters
        ----------
        reset_all : `bool`, optional
            Reset all of the internal data or not. (the default is False)
        """

        self._prefilter_axial.reset()
        self._prefilter_tangent.reset()

        self._command_delay_axial.reset()
        self._command_delay_tangent.reset()

        self._control_filter_axial.reset()
        self._control_filter_tangent.reset()

        self._feedforward_delay_filter.reset()

        self._deadband_control_axial.reset(reset_all=reset_all)
        self._deadband_control_tangent.reset(reset_all=reset_all)

        self._gain_schedular.reset()

        self._in_position.reset()

    def calc_actuator_steps(
        self,
        force_demanded: numpy.typing.NDArray[np.float64],
        force_measured: numpy.typing.NDArray[np.float64],
        hardpoints: list[int],
        is_in_position: bool,
        is_deadzone_enabled_axial: bool = True,
        is_deadzone_enabled_tangent: bool = True,
    ) -> tuple[numpy.typing.NDArray[np.int64], numpy.typing.NDArray[np.float64], bool]:
        """Calculate the actuator steps to move in the next control cycle.

        Parameters
        ----------
        force_demanded : `numpy.ndarray`
            Demanded force in Newton.
        force_measured : `numpy.ndarray`
            Measured force in Newton.
        hardpoints : `list`
            Six 0-based hardpoints. The order is from low to high.
        is_in_position : `bool`
            Mirror is in position or not.
        is_deadzone_enabled_axial : `bool`, optional
            Deadzone is enabled or not for the axial hardpoints. (the default
            is True)
        is_deadzone_enabled_tangent : `bool`, optional
            Deadzone is enabled or not for the tangent hardpoints. (the default
            is True)

        Returns
        -------
        `numpy.ndarray` [`int`]
            78 actuator steps to move in the next control cycle.
        `numpy.ndarray`
            78 hardpoint correction in Newton.
        `bool`
            True if the mirror is in position. Otherwise, False.
        """

        # Filter the demanded forces
        num_axial = NUM_ACTUATOR - NUM_TANGENT_LINK

        demand_filtered_axial = self._filter_force_demanded(
            force_demanded[:num_axial], is_axail=True
        )
        demand_filtered_tangent = self._filter_force_demanded(
            force_demanded[num_axial:], is_axail=False
        )

        # Get the active and passive demanded forces
        demand_axial_passive, demand_axial_active = self._split_1d_array(
            demand_filtered_axial, hardpoints[:NUM_HARDPOINTS_AXIAL]
        )
        demand_tangent_passive, demand_tangent_active = self._split_1d_array(
            demand_filtered_tangent,
            [hardpoint - num_axial for hardpoint in hardpoints[NUM_HARDPOINTS_AXIAL:]],
        )

        # Do the feedback
        (
            feedback_axial,
            feedback_tangent,
            hardpoint_correction,
        ) = self._calc_force_feedback(
            force_measured,
            demand_axial_passive,
            demand_tangent_passive,
            hardpoints,
            is_deadzone_enabled_axial=is_deadzone_enabled_axial,
            is_deadzone_enabled_tangent=is_deadzone_enabled_tangent,
        )

        error_axial_active = (
            (demand_axial_active - feedback_axial)
            if self.is_feedback
            else demand_axial_active
        )

        error_tangent_active = (
            (demand_tangent_active - feedback_tangent)
            if self.is_feedback
            else demand_tangent_active
        )

        # Apply the control filter
        error_axial_active_filter = self._control_filter_axial.filter(
            error_axial_active
        )
        error_tangent_active_filter = self._control_filter_tangent.filter(
            error_tangent_active
        )

        # Multiply with the decoupling matrix and gains
        steps_active = self.kdc.dot(
            np.append(error_axial_active_filter, error_tangent_active_filter).reshape(
                -1, 1
            )
        ).ravel()

        gain_axial, gain_tangent = self._gain_schedular.get_gain(is_in_position)
        steps_active_axial = (
            gain_axial * steps_active[: (num_axial - NUM_HARDPOINTS_AXIAL)]
        )
        steps_active_tangent = (
            gain_tangent * steps_active[(num_axial - NUM_HARDPOINTS_AXIAL) :]
        )

        # Do the feedforward

        # Note that we are using the demanded force instead of the
        # "pre-filtered" demanded force to do the feedforward. This is
        # implemented in the ts_mtm2_cell, which is different from the
        # vendor's documentation.
        # I do not understand the reason for this inconsistency and the
        # original LabVIEW developer only commented this is just for the
        # temporary use in "Control Loop (FIFO).vi".
        steps_feedforward = self._calc_steps_feedforward(force_demanded, hardpoints)
        steps = (
            (np.append(steps_active_axial, steps_active_tangent) + steps_feedforward)
            if self.is_feedforward
            else np.append(steps_active_axial, steps_active_tangent)
        )

        # Insert the hardpoints
        steps_all = steps.copy()
        hardpoint_correction_all = hardpoint_correction.copy()
        for hardpoint in hardpoints:
            steps_all = np.insert(steps_all, hardpoint, 0.0)
            hardpoint_correction_all = np.insert(
                hardpoint_correction_all, hardpoint, 0.0
            )

        # Saturate the steps in each control cycle
        # Multiply the hardpoint_correction_all with -1 to make sure we have:
        # "force_measured = force_demanded + hardpoint_correction_all'"
        # to make the data analysis easier
        return (
            self._saturate_actuator_steps(steps_all.astype(int)),
            -hardpoint_correction_all,
            self._in_position.is_in_position(
                np.append(error_axial_active, error_tangent_active)
            ),
        )

    def _filter_force_demanded(
        self, force_demanded: numpy.typing.NDArray[np.float64], is_axail: bool = True
    ) -> numpy.typing.NDArray[np.float64]:
        """Filter the demanded force.

        Parameters
        ----------
        force_demanded : `numpy.ndarray`
            Demanded force in Newton.
        is_axail : `bool`, optional
            Is the axial actuator or not. (the default is True)

        Returns
        -------
        force_demanded_delay : `numpy.ndarray`
            Filtered demanded force.
        """

        prefilter = self._prefilter_axial if is_axail else self._prefilter_tangent
        command_delay_filter = (
            self._command_delay_axial if is_axail else self._command_delay_tangent
        )

        force_demanded_prefilter = prefilter.filter(force_demanded)
        force_demanded_delay = command_delay_filter.filter(force_demanded_prefilter)

        return force_demanded_delay

    def _split_1d_array(
        self, array: numpy.typing.NDArray[np.float64], indices: list[int]
    ) -> tuple[numpy.typing.NDArray[np.float64], numpy.typing.NDArray[np.float64]]:
        """Split the 1D array.

        Parameters
        ----------
        array : `numpy.ndarray`
            1D array
        indices : `list`
            Specific indices in array.

        Returns
        -------
        `numpy.ndarray`
            Splitted array with the specified indices.
        `numpy.ndarray`
            Splitted array without the specified indices.
        """

        indices_unspecified = [idx for idx in range(len(array)) if idx not in indices]

        return array[indices], array[indices_unspecified]

    def _calc_force_feedback(
        self,
        force_measured: numpy.typing.NDArray[np.float64],
        demand_hardpoint_axial: numpy.typing.NDArray[np.float64],
        demand_hardpoint_tangent: numpy.typing.NDArray[np.float64],
        hardpoints: list[int],
        is_deadzone_enabled_axial: bool = True,
        is_deadzone_enabled_tangent: bool = True,
    ) -> tuple[
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
        numpy.typing.NDArray[np.float64],
    ]:
        """Calculate the feedbacked force.

        Parameters
        ----------
        force_measured : `numpy.ndarray`
            Measured force in Newton.
        demand_hardpoint_axial : `numpy.ndarray`
            Demanded force of the axial hardpoints in Newton.
        demand_hardpoint_tangent : `numpy.ndarray`
            Demanded force of the tangent hardpoints in Newton.
        hardpoints : `list`
            Six 0-based hardpoints. The order is from low to high.
        is_deadzone_enabled_axial : `bool`, optional
            Deadzone is enabled or not for the axial hardpoints. (the default
            is True)
        is_deadzone_enabled_tangent : `bool`, optional
            Deadzone is enabled or not for the tangent hardpoints. (the default
            is True)

        Returns
        -------
        feedback_axial : `numpy.ndarray`
            Feedbacked force of the axial actuators in Newton.
        feedback_tangent : `numpy.ndarray`
            Feedbacked force of the tangent actuators in Newton.
        hardpoint_correction : `numpy.ndarray`
            Hardpoint correction in Newton (for the active actuators).
        """

        # Select the latched forces of hardpoints
        measured_passive, measured_active = self._split_1d_array(
            force_measured, hardpoints
        )

        latched_passive_axial = self._deadband_control_axial.select(
            demand_hardpoint_axial - measured_passive[:NUM_HARDPOINTS_AXIAL],
            is_enabled=is_deadzone_enabled_axial,
        )
        latched_passive_tangent = self._deadband_control_tangent.select(
            demand_hardpoint_tangent - measured_passive[NUM_HARDPOINTS_AXIAL:],
            is_enabled=is_deadzone_enabled_tangent,
        )

        # The feedbacked forces of active actutors need to consider the
        # hardpoint compensation.
        hardpoint_correction = self.hd_comp.dot(
            np.append(latched_passive_axial, latched_passive_tangent).reshape(-1, 1)
        ).ravel()

        feedback_axial = (
            measured_active[:-NUM_HARDPOINTS_AXIAL]
            + hardpoint_correction[:-NUM_HARDPOINTS_AXIAL]
        )
        feedback_tangent = (
            measured_active[-NUM_HARDPOINTS_AXIAL:]
            + hardpoint_correction[-NUM_HARDPOINTS_AXIAL:]
        )

        return feedback_axial, feedback_tangent, hardpoint_correction

    def _calc_steps_feedforward(
        self, force_demanded: numpy.typing.NDArray[np.float64], hardpoints: list[int]
    ) -> numpy.typing.NDArray[np.float64]:
        """Calculate the feedforward steps.

        Parameters
        ----------
        force_demanded : `numpy.ndarray`
            Demanded force in Newton.
        hardpoints : `list`
            Six 0-based hardpoints. The order is from low to high.

        Returns
        -------
        `numpy.ndarray`
            Feedforward steps.
        """

        # Calculate the feedforward matrix
        force_demanded_passive, force_demanded_active = self._split_1d_array(
            force_demanded, hardpoints
        )

        force_demanded_hd_comp = self.hd_comp.dot(
            force_demanded_passive.reshape(-1, 1)
        ).ravel()

        force_demanded_active_diff = force_demanded_active - force_demanded_hd_comp

        steps = self.kinfl.dot(force_demanded_active_diff.reshape(-1, 1)).ravel()

        # Pass the delay filter
        return self._feedforward_delay_filter.filter(steps)

    def _saturate_actuator_steps(
        self, actuator_steps: numpy.typing.NDArray[np.int64]
    ) -> numpy.typing.NDArray[np.int64]:
        """Saturate the actuator steps in each control cycle.

        Parameters
        ----------
        actuator_steps : `numpy.ndarray`
            The 78 actuator steps to move in each control cycle.

        Returns
        -------
        `numpy.ndarray`
            The saturated 78 actuator steps to move in each control cycle.
        """

        num_axial = NUM_ACTUATOR - NUM_TANGENT_LINK

        saturated_steps_axial = np.clip(
            actuator_steps[:num_axial], -self._step_limit_axial, self._step_limit_axial
        )
        saturated_steps_tangent = np.clip(
            actuator_steps[num_axial:],
            -self._step_limit_tangent,
            self._step_limit_tangent,
        )

        return np.append(saturated_steps_axial, saturated_steps_tangent).astype(int)
