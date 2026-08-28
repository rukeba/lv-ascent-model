# What the catalogue is missing, and on what terms

The gaps are results rather than unfinished work, and they are not all of one
kind. Each catalogue file's header names its own.

## Limits of the family and the vehicle

Four combinations reach nothing near the orbit at all.

**The five-phase turn on Ariane 62**, at 400, 500 and 600 km. Not one node of
the grid came out on an orbit - not an open trajectory that nearly closed, but
no orbit anywhere in the family. Flown again with the altitude screen off, so
that every node was integrated rather than dropped on the estimate, and the
answer was the same. Its turn would have to be one continuous manoeuvre while
the vehicle spends its last several hundred seconds on a low-thrust upper stage.

**The velocity-share quartic on Falcon 9 above 600 km.** The handful of nodes
that survive the screen come out on an ellipse with both apsides wrong; the
closest at 700 km is 1668.8 km out.

## A limit of the search: the bilinear tangent on Ariane 62

This one is on file at every altitude and is a different case. An orbit is
there - what comes back has one apsis exact to ten metres and the other
kilometres away, which is what a coordinate-wise descent looks like when it has
walked into the wrong valley - and this search does not land on it.

A tenth of a second of burn is some five kilometres of apogee on a vehicle that
burns for a thousand seconds, so the ranking along the cut-off axis is a row of
narrow valleys rather than one, and each pass is a coordinate-wise descent that
halves its reach: it travels about two sweep steps and stays in whichever valley
it started in. See [closing in](search-refinement.md).

**Following several valleys is not what is short here.** That is what recovered
the same family at 400 km on Falcon 9 and at 1100 km on H3, which were on this
list until the passes learnt to do it. Measured on Ariane 62: five valleys leave
the 400 km case 39,992 m out and twenty leave it 39,471 m at three times the
cost.

**The sweep's resolution on the two angle axes is.** `start` walks ten degrees
in nine values while every answer for a vehicle that burns for a thousand
seconds sits in the top degree of it, which puts two nodes where the whole
answer lives. At that resolution the sweep's ranking says more about the angle
being wrong than about which cell is worth descending into.

So all three were searched again with `start` at 17 values and `middle` at 23,
thirty valleys followed, and finishing at five steps a second. They came out
quite differently:

| | result | filed at |
|---|---|---|
| 500 km | **5 m** - the default grid had left this same case 114 km out | 0.5 km, as the rest |
| 400 km | **1.3 km** | 2.5 km |
| 600 km | **17.6 km**, speed 14 m/s out | 20 km and 15 m/s |

The last is not a tolerance so much as an admission: the perigee is on the
circle and the apogee is eighteen kilometres above it, which is a set a vehicle
could fly and an orbit nobody asked for.

Both of the last two are filed rather than dropped because what they fill is a
statement about this family on this vehicle that a blank line cannot make - and
neither is a set that meets what the rest of the file meets, which is exactly
why every entry carries the tolerance it was accepted at.

## Why the last stretch does not close

Where it stops, it stops because a grid is being asked to do a solver's job. At
a fixed vertical rise and a fixed cut-off, the two coefficients of the turn are
two unknowns for the two terminal conditions of a circular orbit, which is a
root to be found rather than a floor to be walked down to.
