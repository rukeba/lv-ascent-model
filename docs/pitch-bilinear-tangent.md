# Bilinear tangent

`BilinearTangentProgramme` in [`pitch.py`](../src/ascent/pitch.py);
`bilinear-tangent` or `bt` on the command line.

What is prescribed is

    tan(gamma) = (a*tau + b) / (c*tau + 1),   tau = t - t1

This is the classical optimal steering law of powered flight - what the calculus
of variations returns for a flat Earth, constant gravity and no atmosphere - and
it is here as an explicit programme with its coefficients as parameters rather
than as something solved for. It has no phases: one expression covers the turn
from `t1` to `te`.

## Three things worth knowing before fitting it

**The last digits carry the terminal angle.** The numerator `a*tau + b` cancels
to almost nothing at the end of the turn, and that cancellation is what levels
the vehicle out. Round `a` and `b` and the perigee moves by a kilometre. This is
why catalogue entries write them to full precision.

**The coefficients are nearly degenerate.** Scaling `b` and `c` together leaves
almost the same turn, which is why the search grids the angles the turn passes
through instead of the coefficients - see [the grid](search-grid.md).

**It steps the angle at `t1`**, from the vertical straight to `arctan(b)`. How
far that start angle is from 90 degrees is the size of a discontinuity rather
than a free choice; the eighteen sets on file start between 84.7 and 89.2
degrees.

## Bounds

`te > t1`, and `c*(te - t1) + 1 > 0`. The denominator has a pole at
`tau = -1/c`, and a turn that runs through it comes back as a jump of pi in the
angle and as division by nothing in the rate.

## Recovering the coefficients from three angles

`bilinear_coefficients` takes the angle at `t1`, an angle at a prescribed
instant part way through, and the angle at `te`, and returns `a`, `b`, `c`.
`b` is the tangent of the first, and the other two follow from a 2x2 solve of

    a*tau - c*(y*tau) = y - b

at each of the two remaining points. This is how the search parametrises the
family, and what the catalogue writes out is `a`, `b` and `c` themselves.

## Why it is the hardest of the three to land on

It reaches the horizon linearly, so how far the cut-off falls past the end of
its turn *is* the eccentricity of the orbit. The floor of its valley in the
search ranking is a few hundredths of a second wide where the other two are a
good deal wider. See [closing in](search-refinement.md) and [what the catalogue
is missing](catalogue-gaps.md).
