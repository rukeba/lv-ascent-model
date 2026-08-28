"""ICAO standard atmosphere and the gravity field, up to 100 km.

One call returns pressure, density and the speed of sound together, because
the equations of motion need all three at the same altitude: pressure sets the
thrust and the specific impulse, the other two set the drag.

`air_values` returns the three as a tuple and `air_at` returns them named. The
equations of motion take the tuple; everything else - where the three are held,
passed on or printed - takes `Air`.

See docs/atmosphere.md
"""

import math
from bisect import bisect_right
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

# Where one layer gives way to the next, and what never changes inside one:
# the exponent of the barometric formula where the temperature falls with
# height, and the scale of the exponential where it does not. Worked out here
# rather than on each of sixteen million calls a search
# See docs/performance.md
CEILINGS = tuple(layer[0] for layer in LAYERS[1:])
SOUND_FACTOR = HEAT_CAPACITY_RATIO * GAS_CONSTANT
_LAYERS = tuple(
    (base_height, base_temperature, base_pressure, lapse_rate,
     -STANDARD_GRAVITY / (lapse_rate * GAS_CONSTANT) if lapse_rate else 0.0,
     GAS_CONSTANT * base_temperature)
    for base_height, base_temperature, base_pressure, lapse_rate in LAYERS)
_TOP = _LAYERS[-1]
_TOP_BASE = LAYERS[-1][0]


@dataclass(frozen=True, slots=True)
class Air:
    pressure: float
    density: float
    speed_of_sound: float


def air_values(altitude: float) -> tuple[float, float, float]:
    """Pressure (Pa), density (kg/m^3) and the speed of sound (m/s) up there."""
    height = altitude if altitude > 0.0 else 0.0

    base_height, base_temperature, base_pressure, lapse_rate, exponent, scale = \
        _TOP if height >= _TOP_BASE else _LAYERS[bisect_right(CEILINGS, height)]

    rise = height - base_height
    if lapse_rate == 0.0:
        temperature = base_temperature
        pressure = base_pressure * math.exp(-STANDARD_GRAVITY * rise / scale)
        density = pressure / scale
    else:
        temperature = base_temperature + lapse_rate * rise
        pressure = base_pressure * (temperature / base_temperature) ** exponent
        density = pressure / (GAS_CONSTANT * temperature)

    return pressure, density, math.sqrt(SOUND_FACTOR * temperature)


def air_at(altitude: float) -> Air:
    """State of the atmosphere at the given altitude, with the three named."""
    return Air(*air_values(altitude))


def gravity(radius: float) -> float:
    """Gravitational acceleration at the given distance from the centre, m/s^2."""
    return MU / (radius * radius)
