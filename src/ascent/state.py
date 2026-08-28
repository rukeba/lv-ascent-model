"""One sample of the flight: everything the model reports at a single instant.

Velocities are relative to the rotating Earth - that is what the atmosphere
and the pitch programme see. The inertial velocity, which is what the orbit is
built from, is carried alongside as a separate field.
"""

import math
from dataclasses import dataclass

from .constants import EARTH_RADIUS


# `slots` because one is built for every step of every flight
# See docs/performance.md
@dataclass(slots=True)
class FlightState:
    # time from lift-off, s
    t: float = 0.0
    # distance from the centre of the Earth, m
    radius: float = EARTH_RADIUS
    # geocentric angle travelled from the pad, rad
    polar_angle: float = 0.0
    # speed relative to the rotating Earth, m/s
    speed: float = 0.0
    # speed in the inertial frame, m/s
    inertial_speed: float = 0.0
    # flight-path angle above the local horizon, rad, and its rate, rad/s
    flight_path_angle: float = math.pi / 2
    flight_path_rate: float = 0.0
    mass: float = 0.0
    thrust: float = 0.0
    drag: float = 0.0
    # dynamic pressure, Pa
    dynamic_pressure: float = 0.0
    # thrust deflection the programme demands of the guidance, rad
    steering_angle: float = 0.0
    # the sine of that deflection before it is clamped. Above one no such
    # deflection exists: the thrust cannot hold the programme
    steering_demand: float = 0.0
    # velocity lost to that deflection since lift-off, m/s
    steering_loss: float = 0.0
    # the control-effort functional since lift-off, m^2/s^3. Built on the
    # demand before it is clamped, so it goes on measuring where the loss
    # above saturates
    # See docs/control-effort.md
    control_effort: float = 0.0
    # index of the stage burning at this instant
    stage: int = 0

    @property
    def altitude(self) -> float:
        return self.radius - EARTH_RADIUS

    @property
    def horizontal_speed(self) -> float:
        return self.speed * math.cos(self.flight_path_angle)

    @property
    def vertical_speed(self) -> float:
        return self.speed * math.sin(self.flight_path_angle)

    @property
    def downrange_x(self) -> float:
        """Position in the frame fixed to the pad at lift-off, m."""
        return self.radius * math.sin(self.polar_angle)

    @property
    def downrange_y(self) -> float:
        return self.radius * math.cos(self.polar_angle) - EARTH_RADIUS
