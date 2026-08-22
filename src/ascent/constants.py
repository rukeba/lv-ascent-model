"""Physical constants of the Earth model.

Fixed here rather than read from configuration: every result of the model
depends on them, so they are part of the model rather than of a run.
"""

import math

# mean radius, m
EARTH_RADIUS = 6_371_000
# gravitational constant, m^3/(kg*s^2)
GRAVITATIONAL_CONSTANT = 6.6743e-11
# mass, kg
EARTH_MASS = 5.972e24
# gravitational parameter, m^3/s^2
MU = GRAVITATIONAL_CONSTANT * EARTH_MASS
# sidereal rotation rate, rad/s
EARTH_OMEGA = 7.292115e-5
# sea-level pressure of the standard atmosphere, Pa
SEA_LEVEL_PRESSURE = 101325.0
# standard gravity, m/s^2 - the constant in the definition of specific impulse
STANDARD_GRAVITY = 9.80665


def circular_velocity(altitude: float) -> float:
    """Speed of a circular orbit at the given altitude, m/s."""
    return math.sqrt(MU / (EARTH_RADIUS + altitude))
