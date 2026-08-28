# Implementation notes

Why some of the code is written the way it is. None of it changes an answer; all
of it is on a path run millions of times in a search, where a trajectory is
tens of thousands of steps and a search is tens of thousands of trajectories.

Each of these is checked against the plain form it replaced rather than against
a number written into the test - see [tests](tests.md).

## The atmosphere returns a tuple

`air_values` returns pressure, density and the speed of sound as a plain tuple;
`air_at` returns them as an `Air` object. The equations of motion take the
tuple: they ask for it four times an integration step and unpack it on the spot,
and building an object to be read three times and dropped is a fifth of what the
whole atmosphere costs. Everything else - where the three are held, passed on or
printed - takes `Air`.

The barometric exponent of each layer and the scale of each exponential are
worked out once at import. The layer is found by bisection, with the top one -
where the greater part of every ascent is flown - taken by a comparison ahead of
the search.

## The pitch programme keeps plain lists

The tabulated angle, rate and acceleration are held as Python lists as well as
numpy arrays, and the ends of the tables as plain floats. The two readers are
the most-run lines in the model - four calls an integration step - and taking a
number out of a numpy array wraps it in a numpy scalar first. The arithmetic
that follows is the same IEEE double either way.

`angle_at` exists beside `sample` for the same reason: the equations of motion
ask only for the angle, and `sample` interpolates three tables to answer with
the rate and the acceleration that only the reporting wants.

## The Runge-Kutta step is written out twice

`rk4_step` carries the four- and five-component forms component by component.
This is the innermost line of every trajectory, and there a generator over `zip`
costs several times what the arithmetic inside it does. `_general` is the same
scheme for any other length.

## The vehicle precomputes three tables

Built once in `LaunchVehicle.__post_init__`, read some five times a step:

- the drag table as two sorted rows with the slope of every interval, so that
  `_interpolated` - written out because numpy interpolates an array faster than
  this and a single number slower - has nothing to work out;
- the frontal area from each stage upwards;
- the mass of the stack from each stage up, with full tanks.

`Stage.propulsion` returns thrust and flow together because the flow is the
thrust over the exhaust speed, so asking for the two separately works the thrust
out twice.

`active_stage` searches the ignition instants rather than walking them.

## The flight state has slots

One `FlightState` is built for every step of every flight and read back column by
column straight afterwards, and a slot is both quicker to fill and quicker to
read than an instance dictionary.

## A telemetry row is one generated expression

`COLUMNS` in [`telemetry.py`](../src/ascent/telemetry.py) declares the name, the
unit and how to read each column off a state. The whole row is built from that
table as a single lambda, compiled once at import with nothing in its globals
but the one constant the expressions use.

A flight is tens of thousands of rows of twenty-one columns and every one of
them is recorded, so a separate call per column costs several times what reading
the state does. Expressions rather than functions is what lets the row be one of
them.

`tests/test_telemetry.py` is what says that a column called `vertical_speed`
holds the vertical speed, since nothing else does.

## The search hands each worker its flight once

`_Flight` is handed to the worker processes when the pool starts rather than
with every node, so the vehicle and the family cross the process boundary once.
Nothing in it is written to: what a node came to comes back as a `Node` and is
counted by whoever collects it, because anything written to in a worker would be
written to a copy.

The integration step travels with the node instead, because it changes under the
workers as the passes go - see [closing in](search-refinement.md).

## Nodes already walked are recognised before they are flown

Every pass that closes in overlaps the pass before it: the node it is centred on
is one of its own, and so is every node a whole width away. Between an eighth
and a fifth of a pass has been walked already. `_key` rounds a node's coordinates
- a coordinate reached by two different routes through the arithmetic differs in
the last bit and is the same set - and the recognition happens before the
trajectory, which is what makes it a saving.

Recognised at that step and not simply at all: the same node answered at one
step a second and at ten is two answers.

## A sampled peak is interpolated

`_peak` fits a parabola through the largest row and its neighbours. The rows of
a flight are a sample of it, and the peak of the dynamic pressure falls between
two of them as often as on one; taking the largest row reads the peak low, which
at a coarse step is not nothing - and that figure is a constraint when `--max-q`
makes it one.
