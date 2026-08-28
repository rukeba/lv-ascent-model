# The velocity budget

Where the propellant went, in m/s. See
[`losses.py`](../src/ascent/losses.py).

Two of the three the trajectory pays as it flies:

- **gravity loss** - velocity spent climbing out of the gravity well;
- **aerodynamic loss** - velocity spent against drag.

What the propellant delivered, less those two, is the speed reached, to within a
metre or two. The projection here is the one the equations of motion carry, so
that the sum closes: gravity less the centrifugal term of the rotating frame,
which is what the along-track equation carries and so what the budget is spent
on.

The third is **not** in that sum:

- **steering loss** - velocity spent deflecting the thrust to fly the
  programme.

The guided phase puts the whole of the thrust along the velocity and asks
nothing of the direction, so the steering loss is the price of holding the
programme, recovered from the normal equation after the fact rather than paid by
the trajectory.

## The span

All three are integrated from lift-off to the last instant the engines were
producing thrust. A coast between two burns falls inside that span and belongs
there: gravity is lost over it as surely as under thrust, and leaving it out
would be what stopped the sum above from closing.

A flight in which nothing was ever spent has a budget of zeros. Integrating the
whole flight instead would report the gravity and the drag of a pure coast as
losses, and its last instant as a burnout that never was.

## The steering loss, recovered

The programme prescribes the flight-path angle without saying how it is
produced, so the thrust deflection that would produce it is recovered from the
normal equation of motion - see `Mission._accumulate_steering`. The normal
acceleration demanded is

    a_control = v gamma' + (g - v^2/r - omega^2 r) cos(gamma) - 2 omega v

which is gravity less the curvature of the path and the centrifugal term of the
frame, with Coriolis on the normal as `2 omega v` - which unloads the steering
for a launch to the east. The deflection follows as `asin(m a_control / T)`, and
the share of the thrust it points away from the velocity is what accumulates.

Where the demand passes one there is no such deflection: the thrust cannot hold
the programme, and the loss saturates at the whole of it. That is what the
second measure, [control effort](control-effort.md), exists to see past.

The accumulation is over the interval and not off its end alone. An interval
that begins unpowered and ends alight spans an ignition and was flown before it,
so it carries nothing: averaging across that step change would charge the new
stage for time the old one flew. The question is asked of the thrust rather than
of the rate, which is legitimately zero wherever the programme happens to be a
gravity turn.

## What it is for

The steering loss is what a pitch programme is judged by, and gravity is what it
trades against: a flatter turn steers less but climbs longer. See [what the
three cost](programme-comparison.md).
