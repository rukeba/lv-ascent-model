# Closing in

The sweep says where in the family the orbit lies. What lands on it is the
passes that follow.

## The passes

The best value found becomes the centre of the next grid, with one neighbour
either side of it: the range from one step below to one step above, five values
across it, which is the same two steps at half the spacing. Then the best of
*those* with half a step either side, and so on - ten times over
(`REFINEMENTS`), which takes the spacing on the cut-off from well over a second
to a couple of milliseconds.

Each pass is `REFINED_NODES` = 5 values an axis rather than the whole grid
again, so ten of them cost less than the sweep does. Five nodes span two old
steps and so halve the step; a wider grid closes in faster per pass and pays for
it as the power of its width.

`--refinements` is how many, and `--refinements 0` stops after the sweep, which
is the map on its own.

The passes are held inside the range each axis was given: a set that comes out
on a bound is reported as such - `on_edge` - rather than chased past it, because
the family may give out there or a better set may lie outside the range you
named. An axis that was held has no reach and is not closed in on.

## Several valleys at once

A pass is a local descent, and a local descent answers only the valley it
started in. Where the ranking has one broad valley that is the whole job.

The cut-off axis is where it is not. A tenth of a second of burn is some five
kilometres of apogee on a vehicle that burns for a thousand seconds, so the
ranking along it is a row of narrow valleys - each a cut-off whose turn can be
shaped to *very nearly* an orbit - and a sweep steps across several of them
between two of its own nodes.

Following only the best is how a search lands on the bottom of the wrong one,
and it has a signature: the apogee exact to ten metres and the perigee
kilometres away, or the other way about.

So each pass takes the head of the ranking *and* the best sets that are not in
the head's cell of the sweep, and closes in on all of them together, as one
pass. `--basins` is how many - five by default (`BASINS`), `--basins 1` for the
descent that follows the best set and nothing else.

### What counts as the same valley

Two sets are the same valley when they sit within one scale of each other on
every axis that was searched - and the scale is not the same on every axis. That
is the part that matters. See `_centres`.

**On a coefficient of the guidance law** it is a step of the sweep: that is the
scale the valleys were missed at, and it does not move as the passes narrow. Not
the reach of the pass, which halves each time - measuring by that would split one
valley into finer and finer pieces as the search converged, and spend the passes
on distinctions inside an answer rather than on answers it has not looked at.

**On an axis of instants** it is the tenth of a second the answer is written in.
Such an axis stops being closed in on once its window is under a tenth, which
happens after six passes or so, and from then it does not move at all. A cut-off
that locked on to the wrong tenth is a search that converges neatly on to a set
a kilometre or two out and can do nothing about it. Read at the sweep step,
every tenth within five seconds of the best would count as the same valley and
only one of them would ever be tried.

The scale used there is *half* a tenth, which is to say: the same tenth and
nothing else. A whole tenth would fold the neighbouring one in, and the
neighbouring one is precisely what has to be tried - once such an axis has
locked, the grid a pass builds on it is the single value it locked to, so a
cut-off one tenth away is covered by nothing at all. On a coefficient a whole
step is right, because there the pass reaches one step either side and does
cover its neighbours.

### What it costs

Not free and not magic. A pass is five nodes an axis whatever the grid was, so
five valleys are five of those, against a sweep that is the whole grid. Measured
on the bilinear tangent, where the difference shows:

| | one valley | five valleys |
|---|---|---|
| Falcon 9 to 400 km | misses by 860 m | **reaches, 131 m** |
| | 20,184 nodes, 7,257 flown, 218 s | 30,063 nodes, 16,478 flown, 485 s |
| H3 to 1100 km | misses by 7,328 m | **reaches, 34 m** |
| | 20,965 nodes, 10,533 flown, 442 s | 30,861 nodes, 19,984 flown, 806 s |

Half again as many nodes, a little over twice as many trajectories, a little
over twice the wall clock - the nodes grow more slowly than the valleys do
because neighbouring valleys share nodes and a search does not walk one twice.
Fifteen valleys on the first of those reaches by 128 m and costs 914 s, which is
where the returns stop being worth the wait.

Where it does not help at all: [the bilinear tangent on Ariane
62](catalogue-gaps.md).

## The step it integrates at rises as it goes

A sweep is not measuring anything. Its job is to say which cell of the family
the orbit lies in, and one step of its own on the cut-off axis is worth tens of
kilometres of apogee - so a trajectory known to within a hundred metres tells it
everything it can use. That is what one integration step a second comes to,
measured across the whole catalogue against ten; two steps a second comes to
28 m, and five to 3.

So the sweep runs at 1 Hz, the pass after it at 2 (`COARSE_STEPS`), and every
pass from the third on at the step you asked for - where the answer is being
resolved to metres and a coarse step would not be noise around it but noise
instead of it.

The ramp is laid **from the end**, so the last pass is always at the finest step
however few there are: `--refinements 0` is one pass, and that pass is the
answer. And never coarser than what was asked for: a caller who wants two steps
a second is not helped by a sweep at one.

Two things follow that look wrong until you see why.

**A search that ramps flies more trajectories than one that does not.** A node
walked at 1 Hz is walked again when the step rises, because the same set measured
two ways is two answers and the finer is the one worth having.

**The answer is drawn only from sets flown at the step you asked for** - see
`measured` and `reaches_orbit`. A coarse set can outrank a fine one on the
difference between the two rules rather than on the difference between the two
sets. Everything found is kept in the table and the CSV at whatever step it was
flown, because that is what a map is for.

A pass that steps up to a finer integration also **keeps the reach it had**
rather than halving: it is re-flying the ground the pass before it covered, and
the point of it is to see that ground plainly rather than to narrow on a coarse
reading of it. So the two step-ups of a full ramp cost two halvings, and eleven
passes resolve what nine would have. `halvings` works this out rather than
assuming one per pass, because the spacing a search reports and the edge it
decides a set sits on are both read off the reach it actually ended with.
