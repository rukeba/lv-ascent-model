"""Orbit determination against closed-form two-body results."""

import math

from ascent.constants import EARTH_RADIUS, MU, circular_velocity
from ascent.orbit import orbit_from_state


def test_circular_orbit():
    altitude = 500_000
    radius = EARTH_RADIUS + altitude
    orbit = orbit_from_state(radius, circular_velocity(altitude), 0.0)

    assert orbit.is_orbit
    assert orbit.eccentricity < 1e-12
    assert abs(orbit.perigee_altitude - altitude) < 1e-3
    assert abs(orbit.apogee_altitude - altitude) < 1e-3
    assert abs(orbit.circularisation_dv) < 1e-6
    assert math.isclose(orbit.period, 2 * math.pi * math.sqrt(radius**3 / MU))


def test_ellipse_from_its_perigee():
    perigee = EARTH_RADIUS + 400_000
    apogee = EARTH_RADIUS + 2_000_000
    semi_major_axis = (perigee + apogee) / 2
    # vis-viva at the perigee, where the velocity is purely tangential
    speed = math.sqrt(MU * (2 / perigee - 1 / semi_major_axis))

    orbit = orbit_from_state(perigee, speed, 0.0)

    assert math.isclose(orbit.semi_major_axis, semi_major_axis, rel_tol=1e-12)
    assert math.isclose(orbit.eccentricity, 1 - perigee / semi_major_axis, rel_tol=1e-9)
    assert math.isclose(orbit.apogee_altitude, apogee - EARTH_RADIUS, rel_tol=1e-9)


def test_escape_trajectory_is_not_closed():
    radius = EARTH_RADIUS + 500_000
    orbit = orbit_from_state(radius, 1.2 * math.sqrt(2 * MU / radius), 0.0)

    assert not orbit.is_closed
    assert orbit.apogee_altitude is None
    assert orbit.period is None
