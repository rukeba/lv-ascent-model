# Integration

Fourth-order Runge-Kutta at a fixed step, typically 10 Hz. See
[`integrators.py`](../src/ascent/integrators.py) for the step and
[`mission.py`](../src/ascent/mission.py) for the stepping that uses it.

The scheme evaluates the derivative four times and its error falls with the
fourth power of the step, so halving the step cuts the error by sixteen - far
more than the same work spent on smaller first-order steps.

## Events matter more than the order of the scheme

That rate assumes the derivative is smooth across the step. These are not:

- stage separation,
- engine cut-off,
- the end of the pitch programme,
- a tank running dry.

Each is a step change in mass, thrust or in the equations themselves. The step
is cut exactly at every one of them. At around 60 m/s^2 an event misplaced by
one step at 10 Hz is worth several m/s - far more than the error of the scheme
itself.

The first three are known before the step and are put on its bounds. The last
two - a dry tank, and a watched cut-off threshold - are what the integration is
for, so each is solved for inside the step and the piece cut there:

- **a dry tank** by regula falsi on the propellant burned, re-integrating from
  the start of the piece. A first stage empties a fraction of a second before
  separation under some 60 m/s^2, so a millisecond of error costs 0.06 m/s, and
  a single linear estimate would be first order exactly where it hurts most.
  The flow rate barely varies over a step, so two or three passes reach the
  tolerance.
- **a watched threshold** by walking the piece at eight points and then
  bisecting. Walked rather than tested at the end alone: the inertial speed can
  rise through a threshold and fall back under it inside one piece, near the top
  of a lofted ascent, and the end would then show nothing. Bisection rather than
  regula falsi, because a policy says whether it has fired and not by how much.

## The state vector

Two lengths, because the guidance changes what is integrated:

| | state |
|---|---|
| while the programme runs | `(speed, radius, polar angle, propellant burned)` |
| after it | `(radius, polar angle, vertical, horizontal, propellant burned)` |

Both are written out component by component in `rk4_step`, for the reason given
in the [implementation notes](performance.md). `_general` is the same scheme for
any other length and gives the same numbers term for term.

## What is held over a step

`rk4_step` takes a `held` argument, handed to the derivative function unchanged
and never looked into. What the derivative needs besides the instant and the
state arrives as an argument rather than out of a closure rebuilt at every step,
or - worse - off the caller between one probe and the next, which is how a
scheme that evaluates the middle of a step quietly falls to first order.

For the same reason the throttle a piece runs at is settled at the start of the
piece and never at a trial point: a trial point that overshot the tank capacity
would drop the thrust in the middle of the step, which is the step change that
cutting the step at the dry instant exists to keep out.
