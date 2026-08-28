# Tests

```sh
uv run pytest
```

## What is checked against what

**Closed forms.** The atmosphere, the orbit determination and the pitch
programmes are checked against their own reference values and against the angles
each family promises at the instants that define it.

**The rocket equation.** Horizontal drag-free flight satisfies it exactly, and
the integration is checked against it.

**Itself at half the step.** How far a result moves under refinement is its
numerical error - and a mis-timed event looks exactly like that, which is what
makes this the test that catches an event handled badly.

**The two written-out Runge-Kutta forms against the general one.**
`rk4_step` carries an explicit form for a state of four components and one of
five, and `_general` for anything else. They have to be the same scheme to the
last bit, or a trajectory would depend on how many components it happens to be
written in.

**The vehicle's precomputed tables against the plain forms they replaced.**
Three of its answers are worked out once when the vehicle is built and read back
millions of times, so each is checked against the plain form rather than against
a number written down in the test.

**The two estimates against the catalogue they bound.**
`tests/test_estimates.py` checks both bands against every entry, so the
constants the search relies on cannot drift away from the data they were
measured on. See [the two estimates](search-estimates.md).

**Every catalogue entry, flown again**, against the orbit and the losses
recorded for it - and against the tolerance the entry itself names. Both the
entries held to more than half a kilometre and the combinations the catalogue
holds at all are written out by name in `tests/test_catalogue.py` rather than
derived, so a tolerance quietly widened, or a set that stopped being found,
fails the suite instead of being absorbed. See [the catalogue](catalogue.md).

**The search machinery**, on narrow grids on purpose: what is under test is the
grid, the ranges, the ranking and the passes that close in, not how long a sweep
of a whole family takes.

## The reference budget

`tests/test_reference.py` pins the published velocity budget of the three
programmes - Falcon 9 into a 500 km circular orbit from Cape Canaveral, due
east, at 10 steps per second - to one decimal place. That ties the whole chain
down at once: atmosphere, propulsion, equations of motion, event handling and
the loss accounting.

These are not the split printed in the dissertation, which has
2326.7 / 29.3 / 579.0 in the first row. The loss projection was corrected after
that went to paper: gravity up by some 250 m/s, steering down by some 50, and
the 190 the total gains is the gap by which the old split failed to close. The
flight itself did not move - the same trajectories, read the way the equations
of motion carry them.

The three sets are flown as printed rather than solved again, because they are
on paper. So the orbits they reach are the ones those parameters give against
the gravitational parameter [as it now stands](constants.md).
