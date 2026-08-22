"""ICAO standard atmosphere and the gravity field, up to 100 km.

One call returns pressure, density and the speed of sound together, because
the equations of motion need all three at the same altitude: pressure sets the
thrust and the specific impulse, the other two set the drag.
"""

import math
from dataclasses import dataclass

from .constants import MU, STANDARD_GRAVITY

# specific gas constant of air, J/(kg*K)
GAS_CONSTANT = 287.05
# ratio of specific heats
HEAT_CAPACITY_RATIO = 1.4

# (base altitude m, base temperature K, base pressure Pa, lapse rate K/m)
LAYERS = (
    (0, 288.15, 101325.0, -0.0065),
    (11_000, 216.65, 22632.06, 0.0),
    (20_000, 216.65, 5474.889, 0.001),
    (32_000, 228.65, 868.019, 0.0028),
    (47_000, 270.65, 110.906, 0.0),
    (51_000, 270.65, 66.9389, -0.0028),
    (71_000, 214.65, 3.9564, -0.002),
    (84_852, 186.946, 0.3734, 0.0),
)


@dataclass(frozen=True)
class Air:
    pressure: float
    density: float
    speed_of_sound: float


def air_at(altitude: float) -> Air:
    """State of the atmosphere at the given altitude."""
    height = max(0.0, altitude)

    base_height, base_temperature, base_pressure, lapse_rate = LAYERS[-1]
    for layer, next_layer in zip(LAYERS, LAYERS[1:]):
        if height < next_layer[0]:
            base_height, base_temperature, base_pressure, lapse_rate = layer
            break

    rise = height - base_height
    if lapse_rate == 0.0:
        temperature = base_temperature
        pressure = base_pressure * math.exp(
            -STANDARD_GRAVITY * rise / (GAS_CONSTANT * base_temperature))
    else:
        temperature = base_temperature + lapse_rate * rise
        pressure = base_pressure * (temperature / base_temperature) ** (
            -STANDARD_GRAVITY / (lapse_rate * GAS_CONSTANT))

    return Air(
        pressure=pressure,
        density=pressure / (GAS_CONSTANT * temperature),
        speed_of_sound=math.sqrt(HEAT_CAPACITY_RATIO * GAS_CONSTANT * temperature),
    )


def gravity(radius: float) -> float:
    """Gravitational acceleration at the given distance from the centre, m/s^2."""
    return MU / (radius * radius)
