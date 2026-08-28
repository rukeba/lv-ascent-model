# Searching for a parameter set

`ascent-search` solves the problem [the catalogue](catalogue.md) holds the
answers to. Give it a vehicle, a circular orbit and one of the three programme
families, and it sweeps a grid over **every parameter of that family** and
returns the sets that reach the orbit, ranked by how close each came. See
[`search.py`](../src/ascent/search.py).

```sh
uv run ascent-search f9 --altitude 500
uv run ascent-search f9 --altitude 600 --programme bilinear-tangent
uv run ascent-search f9 -a 500 -p 5f --dry-run             # the grid, before it is flown
uv run ascent-search a62 --altitude 700 --yaml             # as a catalogue entry
```

The mission file supplies the vehicle and the launch site and nothing else;
`--altitude` and `--programme` say what to search for. The full option list is
in [the commands](cli.md).

## A map before it is an answer

A sweep alone cannot land on an orbit. Near a circular orbit the apogee answers
to the cut-off at some 80 km per second, and the cut-off axis spans the whole
window the estimate allows - some fifty seconds - so one step of it is worth a
hundred kilometres of apogee.

What a sweep gives is a map of where in the family the orbit lies. What lands on
it is the passes that [close in](search-refinement.md), and what makes the whole
thing affordable is [the two estimates](search-estimates.md).

## What a set is judged by

Three errors, and they are the three conditions of a circular orbit at a given
altitude:

- the **altitude** at cut-off, against the target;
- the **speed** there, against the speed of the circular orbit that was asked
  for - not against a circle through wherever the vehicle happened to be, which
  a set that levelled off twenty kilometres low would satisfy exactly while
  missing the orbit entirely. The inertial speed, because that is what the orbit
  is built from;
- the **orbit** itself: how far the apogee and the perigee each ended up from
  the circle asked for. Their sum is the ranking.

The first two are printed as a share of what was asked for - the altitude and
the speed of the orbit - and the third as a share of its radius, because an
apogee and a perigee are radii and a difference of radii over an altitude would
be a different error at every target. So the three columns each say something
about themselves and are not to be compared with one another.

### Why the third is the ranking

It is the only one of the three that is not blind to the shape of the orbit. A
set at the right altitude with the right speed but a degree off the horizon is
on an ellipse, and neither of the first two says so.

The sum of the apsidal errors is zero only when apogee = perigee = target, which
is the altitude and the circularity at once - the mean of the apsides is the
energy and the spread of them is the eccentricity - in one relative figure with
no weighting to argue over. The eccentricity is printed beside it.

### Tolerances

A set counts as reaching the orbit when all three are inside their tolerances:
`--tolerance` in kilometres for the first and the third, `--speed-tolerance` in
metres per second for the second.

The speed tolerance is loose beside the one on the orbit because it is the
weaker of the two conditions: an orbit whose apsides are both within half a
kilometre of the target is already within a metre or two per second of the right
speed, and it is there to catch the set that has the altitude and is not going
fast enough to stay there.

Ranked by the third either way, so a search that reaches nothing still says what
came closest rather than saying nothing at all - which is what tells you where
to narrow the grid to next.

## A search is a table before it is an answer

Every distinct set that closed an orbit comes back ranked, and `--top` of them
are printed - fifteen by default - with the parameters that were swept and the
errors each is judged by. The sets that meet all three tolerances are marked
`*`; the answer at the foot of the page is the best of those, or simply the best
if none of them does.

```
TOP 15 OF THE 6,271 SETS THAT REACHED AN ORBIT, BEST FIRST
55 of them meet all three tolerances, marked *

   #     t1          k2          k3     t4   cut-off   gamma     h km   h err    v m/s   v err   per km   apo km      ecc orbit err
-----------------------------------------------------------------------------------------------------------------------------------
  1*   25.7    0.076016    0.468555  503.0     503.0  -0.000   499.94 0.00012   7616.6 0.00001   499.94   499.99 0.000004  0.000010
  2*   25.7    0.076035    0.468506  503.0     503.0   0.000   499.94 0.00013   7616.6 0.00001   499.94   500.02 0.000006  0.000012
```

`--csv` writes the whole of that table to a file, not just the head of it, so a
coarse sweep can be looked at as the map it is - sorted, plotted, narrowed on.
A held parameter gets no column, because it is the same in every row and is
printed once above with the value it was held at.

Only a set that reaches the orbit is written out by `--yaml` as a catalogue
entry: one that misses is worth showing and is not worth filing.

## What the ascent is asked to pay

A flatter ascent goes faster lower down. The set found is reported with the peak
dynamic pressure it asks of the airframe, beside the figure the vehicle file
declares it is designed for, and with the peak thrust deflection it asks of the
guidance.

Neither enters the ranking unless you say so: `--max-q` puts the airframe into
the constraint, and a set that peaks above it is then not an answer however
close it came - which is where a limit on the dynamic pressure belongs in a
search of this kind and where the dissertation puts it. Without the flag the
peaks are reported and nothing more.
