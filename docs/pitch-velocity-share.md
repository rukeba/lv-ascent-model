# Velocity share

`VelocityShareProgramme` in [`pitch.py`](../src/ascent/pitch.py);
`velocity-share` or `vs` on the command line.

What is prescribed is

    eta = V_vertical / V = sin(gamma)

the share of the speed that is pointed up. The angle follows as
`gamma = arcsin(eta)`.

## Where the family comes from

From launch telemetry: across Falcon 9 flights that share starts at one, falls
monotonically, and arrives at very nearly zero at cut-off, staying inside
`[0, 1]` throughout.

And from what the split buys. Taking the pair "speed magnitude and vertical
share" rather than two independent velocity components separates the energetics
of the flight from the geometry of the turn: the magnitude comes from the
vehicle's own thrust and mass through the Tsiolkovsky equation and so is
attainable by construction, while the share carries the whole of the steering.

That is what makes the altitude reached an integral of the programme,

    h = integral of eta * V dt

which can be evaluated without integrating the equations of motion - and it is
the estimate the search screens its grid with. See [the two
estimates](search-estimates.md).

## The phases

One over the vertical rise; a quartic over the turn from `t1` to `tf`; and zero
from `tf` to cut-off, where the velocity is already in the horizon.

    eta(tau) = s*tau^4 + (2 - 2s)*tau^3 + (s - 3)*tau^2 + 1,
    tau = (t - t1) / (tf - t1)

## Why a quartic, and what `s` is

The quartic is the lowest-degree polynomial that can meet the four boundary
conditions - one and zero at the ends, flat at both - and still keep a free
parameter. `s` is that parameter: it sets how full the turn is, how long the
share lingers near one, and so how much altitude the ascent accumulates.

Being flat at both ends is what makes the turn join the vertical rise and the
horizontal phase without a kink in the rate, and what makes a turn that runs all
the way to cut-off leave the vehicle on the horizon anyway: the double root
holds the share under 1e-7 at the last tabulated instant.

## Bounds

`|s| <= 3`. The quartic has an interior stationary point at `(s - 3) / 2s`,
which falls inside the turn for `|s| > 3`: beyond that the share leaves `[0, 1]`
and a clip would kink the turn, which the differenced rate reads off the grid.

The turn cannot outlast the burn, so `tf` is capped at `te`, and it has to start
after the vertical rise.

## Publication

The method behind this family is not published yet; its DOI belongs here and is
a placeholder until it is -
[doi:XX.YYYYY/ZZZZZ](https://doi.org/XX.YYYYY/ZZZZZ).
