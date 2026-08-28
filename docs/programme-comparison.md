# What the three programmes cost

Flown into the same orbit by the same vehicle, the three differ mostly in what
they spend.

## The velocity budget

The [steering loss](velocity-budget.md) - the share of the thrust that holding
the programme points away from the velocity - is what a programme is judged by,
and gravity is what it trades against: a flatter turn steers less and climbs
longer.

`examples/steering_loss_comparison.py` flies all three into a 500 km orbit from
Cape Canaveral and prints the three budgets side by side, next to the published
figures:

```sh
uv run python examples/steering_loss_comparison.py
```

```
programme                gravity  aerodynamic  steering     total   perigee   apogee
five-phase                2568.8         29.3     526.4    3124.6     499.4    501.0
velocity-share            2538.0         29.7     411.0    2978.7     500.4    506.9
bilinear-tangent          2500.0         29.6     433.0    2962.7     499.4    500.4
```

The programme with the smallest steering loss is not the one with the smallest
total: it pays for the saving in gravity losses.

## The control effort

`examples/control_effort_comparison.py` flies the same three Falcon 9 sets and
draws the two accumulations side by side - where the curves part is where the
programme swings the flight-path angle:

```sh
uv run python examples/control_effort_comparison.py    # writes out/control-effort.png
```

```
programme               steering      effort  peak demand
five-phase                 526.4       11169        0.919
velocity-share             411.0        9627        0.841
bilinear-tangent           433.0        9779        1.031
```

On these three the two measures agree on the order, which is worth knowing
rather than assuming, but not on the margins: the five-phase turn costs a fifth
more velocity than the bilinear tangent and a seventh more effort.

And the bilinear tangent is the case [the second measure](control-effort.md)
exists for - its demand peaks at 1.031 just after separation, so its loss sits
on the clamp for four seconds of the burn, where the effort is still reading the
demand itself.
