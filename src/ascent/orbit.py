"""Osculating orbit recovered from a position and an inertial velocity.

The ascent model is planar, so two-body motion is fully described by the
specific energy and the specific angular momentum. Once the engines are off
and the vehicle is out of the atmosphere these parameters stop changing, and
they are what the resulting orbit is judged by.
"""

import math
from dataclasses import dataclass

from .constants import EARTH_RADIUS, MU


@dataclass(frozen=True)
class Orbit:
    # semi-major axis, m (negative on an escape trajectory)
    semi_major_axis: float
    eccentricity: float
    perigee_altitude: float
    # None when the trajectory is not closed
    apogee_altitude: float | None
    period: float | None
    # impulse that would circularise the orbit at apogee, m/s
    circularisation_dv: float | None

    @property
    def is_closed(self) -> bool:
        return self.eccentricity < 1.0

    @property
    def is_orbit(self) -> bool:
        """Closed, and with a perigee above the surface."""
        return self.is_closed and self.perigee_altitude > 0.0


def orbit_from_state(radius: float, tangential_velocity: float,
                     radial_velocity: float) -> Orbit:
    """Orbit through the given state. Velocities must be inertial, m/s."""
    energy = (tangential_velocity**2 + radial_velocity**2) / 2.0 - MU / radius
    momentum = radius * tangential_velocity

    eccentricity = math.sqrt(max(0.0, 1.0 + 2.0 * energy * momentum**2 / MU**2))
    # the semi-latus rectum gives the perigee even on an escape trajectory
    perigee = (momentum**2 / MU) / (1.0 + eccentricity)
    semi_major_axis = -MU / (2.0 * energy) if energy != 0.0 else math.inf

    if energy >= 0.0 or eccentricity >= 1.0:
        return Orbit(semi_major_axis, eccentricity, perigee - EARTH_RADIUS,
                     None, None, None)

    apogee = semi_major_axis * (1.0 + eccentricity)
    return Orbit(
        semi_major_axis=semi_major_axis,
        eccentricity=eccentricity,
        perigee_altitude=perigee - EARTH_RADIUS,
        apogee_altitude=apogee - EARTH_RADIUS,
        period=2.0 * math.pi * math.sqrt(semi_major_axis**3 / MU),
        # on the magnitude: the impulse that circularises an orbit does not
        # depend on which way round it is being flown, and a retrograde one
        # carries a negative momentum
        circularisation_dv=math.sqrt(MU / apogee) - abs(momentum) / apogee,
    )
