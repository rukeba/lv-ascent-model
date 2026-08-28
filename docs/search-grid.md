# The grid

## Every parameter of the turn is an axis

Nothing is held behind your back. The vertical rise, the shape of the turn, the
instant the programme ends and the instant the engines do are all coordinates of
the same grid, and `--dry-run` prints every one of them with the range and the
number of values it will be searched over.

| family | parameters on the grid, and what each is |
|---|---|
| `five-phase` | `t1` the vertical rise, 12-30 s · `k2` the share of the turn spent building the pitch rate up, 0.03-0.09 · `k3` the share spent at a constant rate, 0-0.9 · `t4` the end of the turn, over the cut-off window · `angle` the flight-path angle the turn is aimed at, 0 deg · `coast` powered flight after the programme, 0 s |
| `velocity-share` | `t1` · `turn` where the turn ends as a share of the end of the programme, 0.5-1 · `s` the fullness of the quartic, -3 to 3 · `te` the end of the programme, over the cut-off window · `coast` |
| `bilinear-tangent` | `t1` · `start` the angle the turn begins at, 80-89.6 deg · `mid` how far through the turn the middle angle is prescribed, 0.5 · `middle` that angle, 5-60 deg · `te` · `angle` the angle the turn ends at, 0 deg · `coast` |

A parameter is held by giving it a range of one node, which is all that
`angle=0`, `mid=0.5` and `coast=0` are: a circular orbit is entered along the
horizon, and every set on file ends its programme exactly at cut-off. They are
axes like the rest, and `--range coast=0:10:2` or `--range angle=-1:1:0.5` opens
either of them. The summary prints the held ones too, with the value they were
held at, so a figure that did not move is one you can see was not asked to.

`angle` is on the two families that can be aimed. The velocity share has no such
parameter and is not given an axis for one: its quartic drives the vertical
share of the speed to exactly zero at the end of the turn, so the horizon is
where it arrives by construction rather than by being asked to.

## Two reparametrisations

Neither is a parameter left out.

**The velocity share takes the end of its turn as a share** of the end of the
programme, because the two are not independent - the family refuses a turn that
outlasts the burn - so a share keeps every node of the grid inside the family
wherever the cut-off is searched to, where a pair of times would spend half the
grid on sets that do not exist.

**The bilinear tangent is gridded through the angles its turn passes through**
rather than over `a`, `b` and `c`, which are nearly degenerate: scaling `b` and
`c` together leaves almost the same turn, so a grid over them would spend most of
its nodes on programmes it had already flown. The coefficients are recovered from
the three angles. `mid` is an axis rather than the midpoint it used to be fixed
at: where along the turn the middle angle is prescribed is what decides how much
of the turn is done early, and it is no more a property of the vehicle than the
angle itself is.

Neither reparametrisation reaches the entry written out: that carries `a`, `b`
and `c` for the one and `tf` for the other, as any mission file does.

## How a grid is written

One `--range` per parameter, repeatable:

```sh
--range t1=10:25:10     # ten values from 10 to 25, one every 1.667
--range k2=0.05         # held at 0.05 - one value
```

The equals sign separates the parameter from its numbers and the colons separate
the numbers from each other: where it starts, where it ends, and how many values
to try between the two.

**A count rather than a step**, because a count is what says what the search will
cost - the grid is the product of the counts of its axes - and because both ends
of a range are then values the search actually tries. The step follows from the
three and the summary prints it alongside.

A parameter the family does not have is refused at the command line, with the
parameters it does have, rather than several minutes into a search that has
already started.

## Instants are asked for in tenths of a second

The vertical rise, the end of the programme and the cut-off are rounded there,
because that is the finest a timeline is ever issued to - and it is what the
model already works in, since a pitch programme is tabulated on a
tenth-of-a-second grid. Two values that come to the same tenth are one node of
the grid rather than two answers differing where nothing can act on the
difference.

No vehicle is commanded to shut its engines down at 502.6720 s, and none begins
its turn at 14.2676.

What this leaves the search to steer with is the shape of the turn. Those are
coefficients of a guidance law rather than instants on a timeline - `k2` and
`k3`, the fullness of a quartic, the angles a tangent passes through - and
nothing rounds them. Every family keeps exactly two of them, which is what the
two terminal conditions of a circular orbit need.

## Why the count on an axis matters, and which ones

Not for precision. Ten passes resolve any of these axes far past what the
tolerance asks, whatever the sweep gave them. What the passes cannot do is
travel: one step either side, then half a step, then a quarter - the sum of
which is two steps and no more. The whole search can move about two sweep steps
away from where the sweep pointed it.

So the count on an axis matters in proportion to how small two of its steps are
against the whole range, and that varies enormously:

| axis | values | two steps, against the range |
|---|---|---|
| `t1` | 4 | 12 s of 18 - two thirds |
| `k2` | 4 | 0.04 of 0.06 - two thirds |
| `turn`, `start` | 9 | about a quarter |
| `s`, `middle` | 9, 12 | about a fifth |
| `t4` / `te` | 41 | 2.8 s of 55 - a twentieth |

The vertical rise needs no more values than it has: the passes reach most of its
range from wherever they start. The cut-off is the opposite case, and it is also
the steepest axis there is, which is why it gets forty-one where the shape of
the turn gets four to twelve.

Even so, no count makes the cut-off safe on its own - a twentieth of the window
is still a twentieth - and what covers the rest is the [staged
recipe](search-cost.md#the-staged-way-to-use-it).
