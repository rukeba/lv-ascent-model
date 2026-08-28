# Five-phase turn

`FivePhaseProgramme` in [`pitch.py`](../src/ascent/pitch.py); `five-phase` or
`5f` on the command line.

What is prescribed is the pitch **rate**, as a trapezium, and the angle follows
by integrating it. Five phases:

1. a vertical rise to `t1`;
2. the rate built up from zero over the share `k2` of the turn;
3. the turn held at that rate over the share `k3`;
4. the rate arrested over what is left, so that the angle arrives at the horizon
   exactly at `t4`, where the programme ends;
5. free flight on the attitude reached, to cut-off.

## Why the rate and not the angle

A turn written as an angle leaves the rate and the angular acceleration as
derived quantities, and those are exactly what actuator authority and bending
loads are written in terms of.

A trapezium in the rate makes the rate continuous at every joint by construction
and the angular acceleration piecewise constant and bounded, so the control
moment stays finite and the programme is something an attitude loop could
actually hold.

## Why four parameters close the family

The requirement that the turn cover the whole 90 degrees closes it
analytically: the working rate `omega` follows from the angle to be covered, so
`t1`, `t4`, `k2` and `k3` are the whole of it.

`k2` and `k3` are shares rather than times, which is what lets a set of
parameters carry across vehicles of different classes and burn lengths.

## What the two levers do

They set how flat the turn is - the length of the manoeuvre and the share of it
spent at a constant rate. Both rise with the target altitude while the peak rate
falls with it: a higher orbit needs a longer burn, and a longer manoeuvre can be
made flatter, spending the propellant on horizontal speed rather than on holding
the thrust away from the horizon.

R. Keba and A. M. Kulabukhov, *Journal of Rocket-Space Technology* **34**(4),
115-122 (2025), [doi:10.15421/452553](https://doi.org/10.15421/452553).

## Bounds

`t4 > t1`, `k2 > 0`, `k3 >= 0` and `k2 + k3 < 1`. Every one of these divides
something in the construction, and the phases have to come in order: a bad set
would otherwise raise out of the arithmetic or build a turn that runs backwards.

`k2 + k3 = 1` in particular leaves the fourth phase no time to arrest the pitch
rate in, and the rate it would need to is divided by that nothing.

A search bounds `k2` away from zero for a reason of the model rather than of the
search: driving it to zero costs a model that prices only the angle nothing, and
what it buys is a phase no vehicle could fly.
