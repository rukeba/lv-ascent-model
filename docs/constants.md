# Constants of the Earth model

Fixed in [`constants.py`](../src/ascent/constants.py) rather than read from
configuration: every result of the model depends on them, so they are part of
the model rather than of a run.

| Constant | Value | |
|---|---|---|
| `EARTH_RADIUS` | 6 371 000 m | mean radius |
| `MU` | 3.986004418e14 m^3/s^2 | gravitational parameter |
| `EARTH_OMEGA` | 7.292115e-5 rad/s | sidereal rotation rate |
| `SEA_LEVEL_PRESSURE` | 101 325 Pa | standard atmosphere |
| `STANDARD_GRAVITY` | 9.80665 m/s^2 | the constant in the definition of Isp |

`GRAVITATIONAL_CONSTANT` and `EARTH_MASS` are in the file for reference.
Nothing is computed from them.

## Why `MU` is taken as measured

`MU` is measured directly - from the motion of satellites, laser ranging and
the perturbation of orbits - and is known to some nine significant figures. It
is deliberately not built as `G * M`:

- `G` is the worst measured of the fundamental constants, at a relative
  uncertainty near 2e-5;
- the mass of the Earth is not measured independently at all. It is obtained as
  `mu / G`, so it carries the whole of that uncertainty.

Multiplying rounded `G` and `M` back together throws away the precision `mu`
was measured for. For a check: with `G` as tabulated, the measured `mu` answers
to `M = 5.972168e24` rather than `5.972e24`, and the whole discrepancy sits in
the fourth digit of the mass.

The model used to compute `MU` from `G * M`. Moving to the measured value
shifted it by 2.8e-5 relative: the required characteristic velocity rose by some
0.12 m/s at every altitude, all catalogue sets were solved again for it, and the
reference budget moved by a tenth of a metre per second in two of its three
rows.
