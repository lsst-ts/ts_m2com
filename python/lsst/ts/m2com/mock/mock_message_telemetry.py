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

from lsst.ts.tcpip import OneClientServer

__all__ = ["MockMessageTelemetry"]


class MockMessageTelemetry:
    """Mock message of telemetry to simulate the message from real hardware.

    Parameters
    ----------
    server : `tcpip.OneClientServer` or None
        Telemetry server.

    Attributes
    ----------
    server : `tcpip.OneClientServer` or None
        Telemetry server.
    """

    def __init__(self, server: OneClientServer | None) -> None:
        self.server = server

    async def write_position(self, data: dict) -> None:
        """Write the message: M2 position by the actuator positions.

        Parameters
        ----------
        data : `dict`
            Data of M2 position.
        """

        if self.server is not None:
            msg = {
                "id": "position",
                "x": data["x"],
                "y": data["y"],
                "z": data["z"],
                "xRot": data["xRot"],
                "yRot": data["yRot"],
                "zRot": data["zRot"],
            }
            await self.server.write_json(msg)

    async def write_position_ims(self, data: dict) -> None:
        """Write the message: M2 position by the independent measurement system
        (IMS).

        Parameters
        ----------
        data : `dict`
            Data of M2 position by IMS.
        """

        if self.server is not None:
            msg = {
                "id": "positionIMS",
                "x": data["x"],
                "y": data["y"],
                "z": data["z"],
                "xRot": data["xRot"],
                "yRot": data["yRot"],
                "zRot": data["zRot"],
            }
            await self.server.write_json(msg)

    async def write_axial_force(self, data: dict) -> None:
        """Write the message: axial force in Newton.

        Parameters
        ----------
        data : `dict`
            Data of axial force.
        """

        if self.server is not None:
            msg = {
                "id": "axialForce",
                "lutGravity": list(data["lutGravity"]),
                "lutTemperature": list(data["lutTemperature"]),
                "applied": list(data["applied"]),
                "measured": list(data["measured"]),
                "hardpointCorrection": list(data["hardpointCorrection"]),
            }
            await self.server.write_json(msg)

    async def write_tangent_force(self, data: dict) -> None:
        """Write the message: tangent force in Newton.

        Parameters
        ----------
        data : `dict`
            Data of axial force.
        """

        if self.server is not None:
            msg = {
                "id": "tangentForce",
                "lutGravity": list(data["lutGravity"]),
                "lutTemperature": list(data["lutTemperature"]),
                "applied": list(data["applied"]),
                "measured": list(data["measured"]),
                "hardpointCorrection": list(data["hardpointCorrection"]),
            }
            await self.server.write_json(msg)

    async def write_force_error_tangent(self, data: dict) -> None:
        """Write the message: tangent force error.

        Parameters
        ----------
        data : `dict`
            Data of tangent force error.
        """

        if self.server is not None:
            msg = {
                "id": "forceErrorTangent",
                "force": data["force"],
                "weight": data["weight"],
                "sum": data["sum"],
            }
            await self.server.write_json(msg)

    async def write_temperature(self, data: dict) -> None:
        """Write the message: cell temperature in degree C.

        Parameters
        ----------
        data : `dict`
            Data of cell temperature.
        """

        if self.server is not None:
            msg = {
                "id": "temperature",
                "ring": list(data["ring"]),
                "intake": list(data["intake"]),
                "exhaust": list(data["exhaust"]),
            }
            await self.server.write_json(msg)

    async def write_zenith_angle(self, data: dict) -> None:
        """Write the message: zenith angle in degree.

        Parameters
        ----------
        data : `dict`
            Data of zenith angle.
        """

        if self.server is not None:
            msg = {
                "id": "zenithAngle",
                "measured": data["measured"],
                "inclinometerRaw": data["inclinometerRaw"],
                "inclinometerProcessed": data["inclinometerProcessed"],
            }
            await self.server.write_json(msg)

    async def write_inclinometer_angle_tma(self, data: dict) -> None:
        """Write the message: telescope mount assembly (TMA) inclinometer angle
        in degree.

        Parameters
        ----------
        data : `dict`
            Data of TMA angle.
        """

        if self.server is not None:
            msg = {
                "id": "inclinometerAngleTma",
                "inclinometer": data["inclinometerRaw"],
            }
            await self.server.write_json(msg)

    async def write_axial_actuator_steps(self, data: dict) -> None:
        """Write the message: axial actuator steps.

        Parameters
        ----------
        data : `dict`
            Data of axial actuator steps.
        """

        if self.server is not None:
            msg = {
                "id": "axialActuatorSteps",
                "steps": list(data["steps"]),
            }
            await self.server.write_json(msg)

    async def write_tangent_actuator_steps(self, data: dict) -> None:
        """Write the message: tangent actuator steps.

        Parameters
        ----------
        data : `dict`
            Data of tangent actuator steps.
        """

        if self.server is not None:
            msg = {
                "id": "tangentActuatorSteps",
                "steps": list(data["steps"]),
            }
            await self.server.write_json(msg)

    async def write_axial_encoder_positions(self, data: dict) -> None:
        """Write the message: axial actuator encoder positions in micron.

        Parameters
        ----------
        data : `dict`
            Data of axial actuator encoder position in millimeter.
        """

        if self.server is not None:
            # Change the unit from mm to um
            position = [value * 1e3 for value in list(data["position"])]

            msg = {
                "id": "axialEncoderPositions",
                "position": position,
            }
            await self.server.write_json(msg)

    async def write_tangent_encoder_positions(self, data: dict) -> None:
        """Write the message: tangent actuator encoder positions in micron.

        Parameters
        ----------
        data : `dict`
            Data of tangent actuator encoder position in millimeter.
        """

        if self.server is not None:
            # Change the unit from mm to um
            position = [value * 1e3 for value in list(data["position"])]

            msg = {
                "id": "tangentEncoderPositions",
                "position": position,
            }
            await self.server.write_json(msg)

    async def write_ilc_data(self, data: dict) -> None:
        """Write the message: inner-loop controller (ILC) data.

        Parameters
        ----------
        data : `dict`
            Data of ILC status.
        """

        if self.server is not None:
            msg = {
                "id": "ilcData",
                "status": list(data["status"]),
            }
            await self.server.write_json(msg)

    async def write_displacement_sensors(self, data: dict) -> None:
        """Write the message: raw measurements from displacement sensors.

        Parameters
        ----------
        data : `dict`
            Data of displacement sensors.
        """

        if self.server is not None:
            msg = {
                "id": "displacementSensors",
                "thetaZ": list(data["thetaZ"]),
                "deltaZ": list(data["deltaZ"]),
            }
            await self.server.write_json(msg)

    async def write_force_balance(self, data: dict) -> None:
        """Write the message: net forces and moments as commanded by the force
        balance system.

        Parameters
        ----------
        data : `dict`
            Data of force balance system.
        """

        if self.server is not None:
            msg = {
                "id": "forceBalance",
                "fx": data["fx"],
                "fy": data["fy"],
                "fz": data["fz"],
                "mx": data["mx"],
                "my": data["my"],
                "mz": data["mz"],
            }
            await self.server.write_json(msg)

    async def write_net_forces_total(self, data: dict) -> None:
        """Write the message: total actuator net forces in Newton.

        Parameters
        ----------
        data : `dict`
            Data of total actuator net forces.
        """

        if self.server is not None:
            msg = {
                "id": "netForcesTotal",
                "fx": data["fx"],
                "fy": data["fy"],
                "fz": data["fz"],
            }
            await self.server.write_json(msg)

    async def write_net_moments_total(self, data: dict) -> None:
        """Write the message: total actuator net moments of force in
        Newton * meter.

        Parameters
        ----------
        data : `dict`
            Data of total actuator net moments of force.
        """

        if self.server is not None:
            msg = {
                "id": "netMomentsTotal",
                "mx": data["mx"],
                "my": data["my"],
                "mz": data["mz"],
            }
            await self.server.write_json(msg)

    async def write_power_status(self, data: dict) -> None:
        """Write the message: power status.

        Parameters
        ----------
        data : `dict`
            Data of power status.
        """

        if self.server is not None:
            msg = {
                "id": "powerStatus",
                "motorVoltage": data["motorVoltage"],
                "motorCurrent": data["motorCurrent"],
                "commVoltage": data["commVoltage"],
                "commCurrent": data["commCurrent"],
            }
            await self.server.write_json(msg)

    async def write_power_status_raw(self, data: dict) -> None:
        """Write the message: raw power status.

        Parameters
        ----------
        data : `dict`
            Data of raw power status.
        """

        if self.server is not None:
            msg = {
                "id": "powerStatusRaw",
                "motorVoltage": data["motorVoltage"],
                "motorCurrent": data["motorCurrent"],
                "commVoltage": data["commVoltage"],
                "commCurrent": data["commCurrent"],
            }
            await self.server.write_json(msg)
