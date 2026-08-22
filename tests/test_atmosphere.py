"""The standard atmosphere against its own reference values."""

import math

from ascent.atmosphere import air_at, gravity
from ascent.constants import EARTH_RADIUS, STANDARD_GRAVITY


def test_sea_level():
    air = air_at(0.0)
    assert abs(air.pressure - 101325.0) < 1e-6
    assert abs(air.density - 1.225) < 1e-3
    assert abs(air.speed_of_sound - 340.29) < 0.05


def test_tropopause_is_continuous():
    below, above = air_at(10_999.0), air_at(11_001.0)
    assert abs(below.pressure - above.pressure) / below.pressure < 1e-3
    assert abs(below.density - above.density) / below.density < 1e-3


def test_density_falls_monotonically():
    densities = [air_at(h).density for h in range(0, 90_000, 1000)]
    assert all(a > b for a, b in zip(densities, densities[1:]))


def test_gravity_at_the_surface():
    # the model uses its own mu, so the surface value is close to but not
    # exactly the standard gravity
    assert abs(gravity(EARTH_RADIUS) - STANDARD_GRAVITY) < 0.02


def test_gravity_follows_the_inverse_square_law():
    assert math.isclose(gravity(2 * EARTH_RADIUS) * 4, gravity(EARTH_RADIUS))
