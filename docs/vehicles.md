# The vehicles

Three are configured. Masses, thrusts and specific impulses are from published
specifications and open sources; the drag coefficient profiles are generic ones
for a slender launch vehicle rather than measured data.

| File | Vehicle | Launch site as flown here |
|---|---|---|
| [`lv.f9.yaml`](../config/lv.f9.yaml) | Falcon 9 Full Thrust | Cape Canaveral SLC-40, 28.5 N |
| [`lv.a62.yaml`](../config/lv.a62.yaml) | Ariane 62 | Kourou ELA-4, 5.2 N |
| [`lv.h3.yaml`](../config/lv.h3.yaml) | H3-22S | Tanegashima LA-Y, 30.4 N |

## Falcon 9

Three entries: first stage, second stage, payload. Nothing unusual - the stages
burn one under the next, which is the case the model is written for.

## Strap-on boosters: Ariane 62 and H3-22S

Both fly with two solid boosters alongside the core rather than under it, and
the model has no stage sequence for that. So the boosted phase is **one entry**
and the core alone continues as the next.

Because the mass is summed over everything still on the vehicle, each entry
holds only what it adds:

- the first entry is the two boosters plus the core propellant spent before
  separation, and its dry mass is the booster structure - all that is actually
  dropped at separation;
- the core itself, structure and the propellant it has left, is in the second
  entry and nowhere else;
- the diameter of the first entry is core and boosters side by side, which is
  what the flow sees until the boosters go.

The thrust of the boosted phase is the **mean** over that phase rather than the
peak of the solids: at a constant thrust it is what empties that propellant in
the time the boosters burn for - 130 s on Ariane 62, 120 s on H3. The specific
impulse is the two propellants together, weighted by flow.

## A note on vacuum and sea-level figures

A vacuum figure below the sea-level one would make the effective nozzle area
negative and the thrust rise with ambient pressure. Stages that only ever burn
in near-vacuum are given equal figures rather than an inverted pair - the H3
core after booster separation is written that way.
