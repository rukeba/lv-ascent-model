# What a search costs, and how to spend less

## The measurement

Falcon 9 to 500 km, at the default settings and with nothing narrowed, on
thirteen processes of a twenty-core machine:

| family | sweep | nodes walked | screened out | trajectories flown | wall clock | error |
|---|---|---|---|---|---|---|
| `five-phase` | 12,464 | 15,072 | 33 % | 10,064 | 5 min 07 s | 59 m |
| `velocity-share` | 13,284 | 15,941 | 74 % | 4,201 | 2 min 15 s | 34 m |
| `bilinear-tangent` | 17,712 | 20,244 | 51 % | 9,908 | 4 min 43 s | 199 m |

The sweep is the whole grid; the ten passes that [close
in](search-refinement.md) add five nodes an axis each, and between three hundred
and seven hundred of those turn out to have been walked already - a pass is
centred on a node of the pass before it and reaches a whole width either side -
and are skipped rather than flown twice.

All three families land on the valley floor, against a tolerance of 500 m. The
bilinear tangent is the hardest of the three, for the reason given [under the
family](pitch-bilinear-tangent.md).

## What the step ramp is worth

Measured on Falcon 9 to 400 km on the bilinear tangent, five valleys throughout:

| | nodes | flown | wall clock | found |
|---|---|---|---|---|
| no ramp, all at 10 Hz | 30,063 | 16,478 | 485 s | 131 m |
| ramp, finishing at 10 Hz | 33,955 | 19,439 | **373 s** | 131 m |
| ramp, finishing at 5 Hz | 33,955 | 19,439 | **234 s** | 131 m |

The same set to the metre in all three. The ramp is worth about a quarter of the
wall clock on its own, and finishing at 5 Hz rather than 10 is worth another
third - two independent choices, and `--steps` is the second of them.

**It does not always pay.** The saving is on the sweep, and most of a sweep's
nodes never fly at all - the altitude integral drops them. What the ramp costs
is two extra passes' worth of trajectories, once at each step-up, and a pass
costs whatever `--basins` makes it. On Ariane 62 to 600 km on the bilinear
tangent, searched with thirty valleys and the angle axes resolved several times
over, the refinement outweighs the sweep three to one:

| | nodes | flown | wall clock | found |
|---|---|---|---|---|
| no ramp, all at 5 Hz | 141,229 | 85,198 | **2,573 s** | 17,641 m |
| ramp, finishing at 5 Hz | 162,747 | 106,716 | **3,419 s** | 17,641 m |

The same set to the metre again, and a third slower. The ramp helps where the
sweep is a real share of the flying, which is where a search normally is, and
costs where a heavy `--basins` has moved the weight into the passes.

## Processes

The nodes of a pass are independent, so they are divided over two thirds of the
cores - `default_workers`. Two thirds rather than all of them because a search
is minutes long and the machine it runs on is being used for something else at
the time. It has to be processes and not threads: the work is Python arithmetic,
and threads would queue up behind the interpreter lock.

It finds exactly the same set however many: the nodes are collected in the order
of the grid, so the answer does not depend on how many processes answered it.
`--workers` says how many, and `--workers 1` searches in this process alone.

Ctrl+C ends a search where it stands, without a stack trace. Stopping one part
way through is an ordinary thing to do - the grid was wider than it needed to
be, or the progress line has already said what you wanted to know. Each worker
is made deaf to the interrupt when it starts, or one press of two keys would
print a dozen stacks from inside a process pool; the process that was asked
keeps its own handler, shuts the pool down and cancels what has not started.

## The three ways to spend less

`--coarse 0.5` lengthens the stride of every axis the family gave, leaving any
axis you wrote out yourself alone - that step was asked for.

`--steps 1` integrates at one step a second rather than ten, which barely moves
the orbit or the budget: the budget is read off the last powered row, and by
then the vehicle is level and out of the air, so all three integrands are near
zero there and the part left out is fractions of a metre per second. The entry
written out asks for ten steps a second whatever it was searched at.

`--dry-run` costs nothing at all: it prints the grid and what the passes come
to, which is worth reading before a grid you have widened yourself. A grid is
cheap to get wrong and expensive to walk.

There is also a floor under mistakes: a grid past `NODE_LIMIT` = 5,000,000 nodes
in one pass is refused before anything is integrated. A step mistyped by a factor
of a hundred is a grid a hundred times larger, and the first thing a pass does is
lay every one of its nodes out in memory.

## The staged way to use it

A sweep says where in the family the orbit lies; a second search narrowed on to
what it found is how the set itself is reached.

```sh
# the map: one sweep, no passes, every set it found written out
uv run ascent-search f9 -a 500 -p bt --refinements 0 --csv out/map.csv

# and the set: narrowed on to what the map showed, every tenth of a second of te
uv run ascent-search f9 -a 500 -p bt \
    --range t1=20 --range start=87:89:5 \
    --range middle=29:31:9 --range te=500.5:501.5:11
```

`--dry-run` on the second of those says what it will cost before it costs it.

This is worth the trouble on the bilinear tangent in particular: the same orbit
searched again on a grid narrowed to what the first search found, with every
tenth of a second in that neighbourhood offered, comes back 220 m out in 35
seconds. Not closer than the whole sweep managed - the same, for a twentieth of
the cost, and from a grid you can see the shape of.

## Against a root find

The catalogue is what this command produces, so the two columns
`examples/parameter_search.py` prints are the same set twice.

That is worth reading against how such a set used to be arrived at, because the
difference is not in the answer's quality but in what is being asked. A root
find holds four numbers where the dissertation holds them - the vertical rise at
20 s, the five-phase `k2` at 0.05, the bilinear tangent's middle angle prescribed
half way along the turn, and every turn aimed at the horizon - and solves the
rest with the cut-off free to whatever precision it likes; the five-phase set it
returns for a 500 km orbit cuts off at 502.71245 s.

This search cannot answer that and should not: it asks for the cut-off in tenths
of a second, and the nearest it will offer is 502.7. So `k2` has to be searched,
because with the cut-off on a tenth it is `k2` and `k3` that carry the two
terminal conditions between them. What comes back is a different route to the
same circle - and it is the one of the two that could be put on a timeline:
`t1` = 25.7 s, `k2` = 0.0760, `k3` = 0.4686, cut-off at 503.0 s, which is the
circle to 59 m.

The routes differ in what they cost. The root-found set spends 516.8 m/s on
steering and 3110.1 in total; this one spends 608.1 and 3213.7, dearer by
104 m/s. Nothing there is wrong: the ranking asks how close the orbit came and
nothing at all about what the route to it cost. `--csv` writes the velocity
budget of every set found beside its errors, which is where to look when the
cheapest route to the circle is wanted rather than the closest one to it.
