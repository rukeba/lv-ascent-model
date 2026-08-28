# Control effort

The [steering loss](velocity-budget.md) weighs propellant: how much of the
thrust went the wrong way. It says nothing about how that demand is spread over
the burn, and two programmes that cost the same can reach it very differently.

The second measure is the control-effort functional

    J = integral over the powered flight of a_control^2 dt,   m^2/s^3

where `a_control` is the normal acceleration the guidance has to produce to hold
the programme - the same quantity the steering loss is recovered from, one step
before it is turned into a deflection.

## Why the square

An abrupt stretch is charged more than an even one, so two programmes with the
same loss are still told apart by how smoothly they ask for it.

## Why it is taken before the clamp

Deliberately, and this is the point of it. The loss is built on a deflection,
and a deflection cannot exceed 90 degrees: where the demand passes one the loss
saturates at the whole of the thrust and stops separating programmes. `J` is
built on the demand itself, so it does not saturate. On H3 the demand reaches
2.9 and the steering losses of the three programmes stop being comparable, while
`J` goes on separating them.

## It is not part of the budget

It is not a velocity, and a sum with the three losses would mean nothing. It is
reported beside them and nowhere inside them.

See [`state.py`](../src/ascent/state.py) for the accumulated column and
`Mission._accumulate_steering` in
[`mission.py`](../src/ascent/mission.py) for where both are formed.
