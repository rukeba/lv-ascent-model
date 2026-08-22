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


def test_circularising_does_not_care_which_way_round():
    """The impulse that circularises an orbit has no direction in it.

    Energy and eccentricity are built from squares and do not know whether the
    vehicle is going east or west, and neither should this: the same ellipse
    flown backwards needs the same impulse at apogee. Taken from the signed
    angular momentum it came back as the sum of the two speeds instead of the
    difference, which for a low orbit is out by some fifteen kilometres a second.
    """
    radius = EARTH_RADIUS + 200_000
    east = orbit_from_state(radius, 0.95 * circular_velocity(200_000), 0.0)
    west = orbit_from_state(radius, -0.95 * circular_velocity(200_000), 0.0)

    assert east.apogee_altitude == west.apogee_altitude
    assert east.perigee_altitude == west.perigee_altitude
    assert east.circularisation_dv > 0.0
    assert west.circularisation_dv == east.circularisation_dv
