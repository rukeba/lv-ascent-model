"""Physical constants of the Earth model.

Fixed here rather than read from configuration: every result of the model
depends on them, so they are part of the model rather than of a run.
"""

import math

# mean radius, m
EARTH_RADIUS = 6_371_000
# gravitational parameter, m^3/s^2. Measured directly - from the motion of
# satellites, laser ranging and the perturbation of orbits - and known to some
# nine significant figures. It is not built from the two constants below: G is
# the worst measured of the fundamental constants, at a relative uncertainty
# near 2e-5, and the mass of the Earth is not measured independently at all but
# obtained as mu/G, so it carries the whole of that uncertainty. Multiplying
# rounded G and M back together throws away the precision mu was measured for.
# For a check: with G as below, the tabulated mu answers to M = 5.972168e24
# rather than 5.972e24, and the whole discrepancy sits in the fourth digit of
# the mass
MU = 3.986004418e14
# gravitational constant, m^3/(kg*s^2) - for reference; nothing here is
# computed from it
GRAVITATIONAL_CONSTANT = 6.6743e-11
# mass, kg - for reference, as above
EARTH_MASS = 5.972e24
# sidereal rotation rate, rad/s
EARTH_OMEGA = 7.292115e-5
# sea-level pressure of the standard atmosphere, Pa
SEA_LEVEL_PRESSURE = 101325.0
# standard gravity, m/s^2 - the constant in the definition of specific impulse
STANDARD_GRAVITY = 9.80665


def circular_velocity(altitude: float) -> float:
    """Speed of a circular orbit at the given altitude, m/s."""
    return math.sqrt(MU / (EARTH_RADIUS + altitude))
