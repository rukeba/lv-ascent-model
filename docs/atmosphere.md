# Atmosphere and drag

[`atmosphere.py`](../src/ascent/atmosphere.py) carries the ICAO standard
atmosphere up to 100 km, and the gravity field beside it.

One call returns pressure, density and the speed of sound together, because the
equations of motion need all three at the same altitude: pressure sets the
thrust and the specific impulse, the other two set the drag.

`air_values` returns the three as a tuple and `air_at` returns them named. The
equations of motion take the tuple - see [implementation
notes](performance.md).

## The layers

Eight layers, each with a base altitude, temperature, pressure and lapse rate.
Inside one layer the barometric formula is either a power of the temperature
ratio (where the temperature falls with height) or an exponential (where it does
not), and the exponent or the scale of it never changes. Both are worked out
once at import rather than on each of some sixteen million calls in a search.

The layer is found by bisection rather than by walking the table, with the top
one - where the greater part of every ascent is flown - taken by a comparison
ahead of the search.

## Drag

The drag coefficient is a table against Mach number, interpolated linearly,
given per vehicle in the configuration file. The profiles in `config/` are
generic ones for a slender launch vehicle rather than measured data.

The frontal area is what the flow sees from each stage upwards: the widest stage
still on the vehicle. Ariane 62 and H3 are as wide as their boosters side by
side, but only until those boosters go.

Above `DRAG_CEILING` - 100 km, in [`vehicle.py`](../src/ascent/vehicle.py) - the
model takes the air as gone and the drag as zero. This is why a target altitude
inside the air is refused by the search rather than modelled.

A vehicle given no drag profile flies without drag, rather than through an
interpolation with nothing to interpolate between.
