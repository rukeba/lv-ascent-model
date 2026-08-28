# The catalogue of solved parameter sets

Parameters that place each vehicle on a circular orbit of a given altitude - one
set per vehicle, pitch programme and altitude, 34 in all.

Kept as **one file a vehicle**: [`catalogue.f9.yaml`](../config/catalogue.f9.yaml),
[`catalogue.a62.yaml`](../config/catalogue.a62.yaml),
[`catalogue.h3.yaml`](../config/catalogue.h3.yaml). A search is run over one
vehicle at a time and a vehicle is recomputed on its own, so a file that holds
one vehicle is a file that changes when that vehicle's sets change and not
otherwise. `load_catalogue` on the directory reads all of them; on a single
file, that vehicle's sets alone.

Falcon 9 is covered from 400 to 700 km every fifty, Ariane 62 at 400, 500 and
600 km, and H3 at 1000, 1100 and 1200 km. For what is not there, see [what the
catalogue is missing](catalogue-gaps.md).

```sh
uv run python examples/programme_catalogue.py    # the whole table
uv run ascent f9 --list                          # one vehicle's file
```

## Every set was searched for, not solved for

`ascent-search` sweeps a grid over the whole of a family, closes in on the best
node ten times over, and ranks what it finds by how far the orbit reached is
from the circle asked for; an entry is the head of that ranking. See [the
search](search.md).

Nothing is held behind the search's back - not the vertical rise, not the
five-phase `k2`, not the instant the bilinear tangent's middle angle is
prescribed at - so the sets differ from what a root find with four of those
numbers pinned would return.

`examples/parameter_search.py` searches Falcon 9 to 500 km from nothing and
prints what it finds beside the file, which is the same set: the file can be
reproduced from the vehicle and the orbit alone.

## What an entry carries

An ordinary mission specification with two blocks added.

**`reached`** - the orbit it produces and what it costs.
`tests/test_catalogue.py` flies all of them to check that it still does, so any
entry can be checked without repeating the search.

**`tolerance`** - what the set was accepted under. A set counts as reaching the
orbit when the perigee, the apogee and the altitude at cut-off are all inside
`orbit_km` and the inertial speed at cut-off is inside `speed_ms`. This is why
`reached` does not read the target back twice over the way a root find would
leave it: it reads what the search actually landed on, inside whatever the
entry beside it says it was held to.

The tolerance is not a property of the file, and it is not the same on every
entry. Twenty are held to half a kilometre. Twelve - most of Falcon 9 - are
held to one, which is the box section 4.4 of the dissertation compares against,
and a tighter one closed no orbit at all on two of the three families there.
Two are on [other terms](catalogue-gaps.md): two and a half, and twenty with
fifteen metres per second beside it, because half will not close on Ariane 62's
bilinear tangent and a set that is near the circle is worth more than a blank.

`tests/test_catalogue.py` lists every entry held to anything but half a
kilometre by name, so a tolerance quietly widened to cover a set that drifted
fails the suite. An entry has to be readable as what it is without anyone
knowing how the search was run.

**`simulation`** carries the step it was flown at, for the same reason: a set is
found against a model, and the step is part of which model. Ten a second nearly
everywhere and five on the Ariane 62 bilinear sets, which halves the cost of a
search that takes hours and costs at most 3 m of apsis across the whole
catalogue.

## Precision

**Every instant is a whole tenth of a second** - the vertical rise, the end of
the programme, the cut-off - because that is the finest a timeline is ever
issued to. An answer written finer is the same answer with digits after it that
nothing can act on.

**The coefficients of the guidance law are written to full precision**, because
they have to be. Near a circular orbit the apogee answers to the cut-off at some
80 km per second, and the [bilinear tangent](pitch-bilinear-tangent.md) is more
delicate still: its numerator cancels to almost nothing at the end of the turn,
so the last digits of `a` and `b` carry the terminal angle and rounding them
moves the perigee by a kilometre.

With the cut-off held to a tenth, it is the shape of the turn that places the
orbit.
