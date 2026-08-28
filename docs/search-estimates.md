# The two estimates

Both come from the dissertation this model was written for, both are quadrature
rather than integration, and both are in
[`estimates.py`](../src/ascent/estimates.py). The search does not work without
them: a grid over every parameter of a family is large, and these are what keep
it affordable.

Neither is accurate enough to stand in for a flight. Both are accurate enough to
say which flights are worth making, and `tests/test_estimates.py` checks both
bands against every entry in the catalogue, so the constants the search relies
on cannot drift away from the data they were measured on.

## The energy-equivalent ascent time

`equivalent_time` - the instant at which the propellant has bought the orbit.

A circular orbit is an amount of energy - the speed of the orbit and the work of
lifting to it together, folded into one characteristic velocity by
`required_velocity` - and the propellant buys energy at the rate the Tsiolkovsky
equation gives. The instant at which what has been bought covers what the orbit
costs, plus everything lost on the way, is the ascent time.

The balance starts negative and rises as long as the thrust acceleration beats
the rate of loss, which on an ascent it does throughout - so the root is unique
and can be marched to rather than searched for. The dissertation puts Brent's
method on the same balance over an adaptive quadrature; marching the cumulative
integral in flight order finds the same root, and only monotony makes either of
them safe.

**What it bounds.** The window around it *is* the default range of the
`t4`/`te` axis, so the search never spends a node on a cut-off the vehicle could
not have. The same balance says before anything is flown whether the vehicle has
the propellant for the orbit at all.

**The band.** Measured against the catalogue the cut-off on file sits between
0.949 and 1.089 of the estimate, and `TIME_MARGIN_EARLY = 0.06` and
`TIME_MARGIN_LATE = 0.15` carry that band. The late end has room to spare. The
early end has 0.9 per cent, which is thinner than it looks: it is one entry -
Ariane 62's bilinear tangent at 400 km - that put it there. Widening it is not
free, since the window is what the cut-off axis spreads its values over, and it
would move every set on file.

**The late end is never past the instant the last tank runs dry.** A cut-off
after that is not a cut-off - the engines have already stopped - and the orbit
would answer to the coast instead of to the burn.

**Earth rotation.** The estimate leaves it out, as the dissertation does. It is
credited only where the answer is a yes or a no rather than a number, because a
refusal has to be made on the most generous reading there is - an eastward pad
is worth some 400 m/s, and a balance that ignores it can fail to close on an
orbit the vehicle reaches comfortably. Where the answer is the number the
rotation is left out, which is what the band was measured on.

The method is published: R. Keba and A. M. Kulabukhov, *Journal of Rocket-Space
Technology* **35**(1), 94-99 (2026),
[doi:10.15421/452567](https://doi.org/10.15421/452567).

### The loss model behind it

Coarse on purpose: the term it builds is a small correction for a vehicle with
its stages in a stack, and a term of the first order for one that flies with
boosters strapped alongside. The layered atmosphere is replaced with an
exponential one and the Mach-dependent drag coefficient with a mean over the
trajectory.

The turn is parametrised by the share of the energy already bought rather than
by time, which is what makes it computable before the length of the ascent is
known: the angle falls from the vertical to the horizon as the square of what is
left to buy.

The effective gravity is `g` less what the horizontal speed already carries.
This is the dissertation's, and it is the one deliberate difference from the
along-track equation the model itself integrates, which carries `g` alone. What
it buys is that the loss dies out exactly where the vehicle reaches orbit; the
objection to it is that a term normal to the velocity does no work along it, so
it prices the gravity loss low. Either way the estimate is calibrated as it
stands, and changing it would move the band above and every window built from
it.

## The analytic altitude integral

`analytic_altitude` - the altitude a programme reaches by the end of it.

The vertical share of the velocity is the sine of the flight-path angle, which
the programme has already tabulated; the speed is the Tsiolkovsky equation stage
by stage, less the gravity lost as the area under that same share. Their product
integrated over the programme is the altitude.

**What it screens.** Every node: a set whose integral says it cannot reach the
target is dropped without a trajectory.

**The band.** What is left out is the air, the thrust deficit at sea level and
the fall of gravity with altitude - and all three push the same way, so the
figure is always high. Against the catalogue it reads between 1.005 and 1.185
times the altitude the flight reaches.

`ALTITUDE_RATIO_LOW = 0.95` and `ALTITUDE_RATIO_HIGH = 1.40` are that band
applied backwards and widened, because it is a gate: a node it rejects is never
flown, and the measurement behind it is of three vehicles. `--no-screen` turns
it off and flies everything, which is how you check it is not hiding anything.

**Only asked of a set whose programme runs to cut-off**, which is every set with
no coast in it. A coast is powered flight the integral does not cover, so the
figure would read low by however much the vehicle climbed over it - and a screen
is turned off rather than widened by a guess.

The method behind it is not published yet; its DOI belongs here and is a
placeholder until it is -
[doi:XX.YYYYY/ZZZZZ](https://doi.org/XX.YYYYY/ZZZZZ).
