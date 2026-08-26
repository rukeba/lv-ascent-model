"""The standard atmosphere against its own reference values."""

import math

from ascent.atmosphere import (GAS_CONSTANT, HEAT_CAPACITY_RATIO, LAYERS,
                               air_at, gravity)
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

def test_the_layer_constants_are_the_table_walked_out():
    """The layer is found by bisection and what it fixes is worked out once.

    Both have to answer exactly what the table and the two barometric formulae
    say, so the plain form is written out here and the two are compared bit for
    bit - at the boundaries of every layer, either side of them, and across the
    whole range the model flies through.
    """
    def walked(altitude):
        height = max(0.0, altitude)
        base_height, base_temperature, base_pressure, lapse = LAYERS[-1]
        for layer, above in zip(LAYERS, LAYERS[1:]):
            if height < above[0]:
                base_height, base_temperature, base_pressure, lapse = layer
                break
        rise = height - base_height
        if lapse == 0.0:
            temperature = base_temperature
            pressure = base_pressure * math.exp(
                -STANDARD_GRAVITY * rise / (GAS_CONSTANT * base_temperature))
        else:
            temperature = base_temperature + lapse * rise
            pressure = base_pressure * (temperature / base_temperature) ** (
                -STANDARD_GRAVITY / (lapse * GAS_CONSTANT))
        return (pressure, pressure / (GAS_CONSTANT * temperature),
                math.sqrt(HEAT_CAPACITY_RATIO * GAS_CONSTANT * temperature))

    heights = [-1.0, 0.0] + [float(layer[0]) for layer in LAYERS]
    heights += [layer[0] + d for layer in LAYERS
                for d in (-1e-9, 1e-9, -1.0, 1.0)]
    heights += [h / 4.0 for h in range(0, 1_600_000, 97)]
    for height in heights:
        air = air_at(height)
        assert (air.pressure, air.density, air.speed_of_sound) == walked(height)
