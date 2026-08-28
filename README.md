# lv-ascent-model

![Falcon 9 into a 500 km circular orbit, drawn to scale over the curve of the Earth](docs/trajectory.png)

*Falcon 9 from Cape Canaveral into a 500 km circular orbit on the five-phase
turn: the ascent in the plane of the launch, drawn to scale over the curve of
the Earth. One of the ten plots of `uv run ascent f9 --report`.*

A two-dimensional model of the powered ascent of a launch vehicle, from
lift-off to orbit insertion. The vehicle flies a prescribed pitch programme;
the model integrates the trajectory that results, reports the orbit it reaches,
and accounts for the velocity spent on the way — against gravity, against the
air, and on steering the thrust away from the velocity in order to fly the
programme. That last figure is what different pitch programmes are compared by.

Three programmes are implemented — a five-phase turn, a turn parametrised by
the vertical share of the velocity, and a bilinear tangent — and three vehicles
are configured: Falcon 9, Ariane 62 and H3-22S.

**The reasoning behind all of it is in [docs/](docs/README.md).** This page is
the tour.

## Installing and running

```sh
uv sync

uv run ascent f9                            # summary of the flight on the console
uv run ascent f9 --csv out/f9.csv           # and the whole trajectory as CSV
uv run ascent f9 --report                   # an HTML report in out/f9, opened
uv run ascent config/mission.a62.yaml       # a mission file by path

uv run ascent f9 --list                     # solved parameter sets on file
uv run ascent f9 --altitude 600             # fly one of them
uv run ascent f9 -a 600 -p bt               # in short: -p is the pitch programme

uv run ascent-search f9 --altitude 500      # search for a set instead of flying one
uv run ascent-search f9 -a 500 -p 5f --dry-run   # the grid, before it is flown
```

`f9`, `a62` and `h3` are short names for `config/mission.<name>.yaml`, and the
three pitch programmes answer to `5f`, `vs` and `bt` as well as to their full
names. Every option of both commands is in [docs/cli.md](docs/cli.md); what the
console prints and what the HTML report holds is in
[docs/report.md](docs/report.md).

## The model

The flight is planar and written in polar coordinates about the centre of the
Earth, integrated in the frame that rotates with it. Thrust is interpolated by
ambient pressure, gravity is a central field on the measured gravitational
parameter, drag comes from a Mach-dependent coefficient and the ICAO standard
atmosphere. Fourth-order Runge-Kutta at a fixed step, typically 10 Hz, with the
step cut exactly at every discontinuity inside it — which matters more than the
order of the scheme.

- [docs/model.md](docs/model.md) — frame, forces, guidance, what is in the model
- [docs/constants.md](docs/constants.md) — and why `MU` is taken as measured
- [docs/atmosphere.md](docs/atmosphere.md) — ICAO layers, the drag ceiling
- [docs/integration.md](docs/integration.md) — the step, and why events matter more
- [docs/velocity-budget.md](docs/velocity-budget.md) — gravity, aerodynamic, steering
- [docs/control-effort.md](docs/control-effort.md) — the second measure, `J`

## The three pitch programmes

A pitch programme is the law that turns the vehicle from the vertical it lifts
off on to the horizontal an orbit needs — the prescribed flight-path angle
`gamma(t)`. It is not the attitude control loop: what is here is the command.

All three prescribe the same thing and disagree about what to prescribe it
*with*, which is what makes them worth comparing.

| | prescribes | |
|---|---|---|
| five-phase turn | the pitch **rate**, as a trapezium | [docs](docs/pitch-five-phase.md) |
| velocity share | `sin(gamma)`, as a quartic | [docs](docs/pitch-velocity-share.md) |
| bilinear tangent | `tan(gamma)`, as a ratio of linear functions | [docs](docs/pitch-bilinear-tangent.md) |

What they share — the tabulation, the fifth phase, the parameters each takes —
is in [docs/pitch-programmes.md](docs/pitch-programmes.md). What they cost,
flown into the same orbit by the same vehicle, is in
[docs/programme-comparison.md](docs/programme-comparison.md):

```
programme                gravity  aerodynamic  steering     total
five-phase                2568.8         29.3     526.4    3124.6
velocity-share            2538.0         29.7     411.0    2978.7
bilinear-tangent          2500.0         29.6     433.0    2962.7
```

The programme with the smallest steering loss is not the one with the smallest
total: it pays for the saving in gravity losses.

## Configuration and the catalogue

A mission file names the vehicle file beside it, the pitch programme and its
parameters, when the engines stop, and where the launch site is. A vehicle file
lists the stages in the order they burn. Both formats are in
[docs/configuration.md](docs/configuration.md), and the three vehicles — with
how the strap-on boosters are modelled — in [docs/vehicles.md](docs/vehicles.md).

The catalogue holds parameters that place each vehicle on a circular orbit of a
given altitude — one set per vehicle, pitch programme and altitude, 34 in all,
one file a vehicle. Every set was **searched for, not solved for**, and every
entry records the orbit it produces, what it costs and the tolerance it was
accepted at.

```sh
uv run python examples/programme_catalogue.py    # the whole table
uv run ascent f9 --list                          # one vehicle's file
```

- [docs/catalogue.md](docs/catalogue.md) — what an entry carries, and why
- [docs/catalogue-gaps.md](docs/catalogue-gaps.md) — what is missing, and on what terms

## Searching for a parameter set

`ascent-search` solves the problem the catalogue holds the answers to. Give it
a vehicle, a circular orbit and one of the three families, and it sweeps a grid
over **every parameter of that family** — nothing held behind your back — and
returns the sets that reach the orbit, ranked by how close each came.

A sweep alone cannot land on an orbit: near a circular orbit the apogee answers
to the cut-off at some 80 km per second. The sweep is a map of where in the
family the orbit lies; ten passes that close in on the best few valleys are what
land on it, and two quadrature estimates are what keep the whole thing
affordable.

- [docs/search.md](docs/search.md) — the command, and what a set is judged by
- [docs/search-grid.md](docs/search-grid.md) — axes, ranges, what each count buys
- [docs/search-estimates.md](docs/search-estimates.md) — the two estimates and their bands
- [docs/search-refinement.md](docs/search-refinement.md) — passes, valleys, the step ramp
- [docs/search-cost.md](docs/search-cost.md) — measured costs, and how to spend less

## Modules

| Module | What is in it |
|---|---|
| `mission.py` | the equations of motion, the stepping that solves them, the cutting of the step at events, and the steering-loss and control-effort accounting |
| `pitch.py` | the three pitch programmes and the tabulation they share |
| `vehicle.py` | stages, propulsion, mass and drag of the launch vehicle |
| `atmosphere.py` | ICAO standard atmosphere and the gravity field |
| `orbit.py` | the osculating orbit recovered from a position and an inertial velocity |
| `losses.py` | the velocity budget: gravity, aerodynamic and steering losses |
| `estimates.py` | the ascent time and the altitude reached, by quadrature rather than by integration |
| `search.py` | the grid search for the parameters of a pitch programme |
| `integrators.py` | the Runge-Kutta step, knowing nothing about rockets |
| `cutoff.py` | when the engines stop — by time, by altitude or by inertial speed |
| `state.py` | one sample of the flight |
| `telemetry.py` | the recorded flight and its CSV form |
| `summary.py` | the console summary |
| `report.py` | the HTML report: the plots, the cards and the flight log |
| `templates/` | the markup and the stylesheet of that report, rendered with Jinja |
| `config.py` | building a mission from YAML, and reading the catalogue |
| `cli.py` | the `ascent` and `ascent-search` commands |
| `constants.py` | constants of the Earth model |

Why a few of them are written the way they are:
[docs/performance.md](docs/performance.md).

## Reproducing a published result

`examples/steering_loss_comparison.py` flies Falcon 9 into a 500 km circular
orbit from Cape Canaveral three times, once per pitch programme, each with the
parameters that minimise its steering loss subject to reaching that orbit, and
prints the velocity budget next to the published figures:

```sh
uv run python examples/steering_loss_comparison.py
```

The other three examples search a set from nothing
(`parameter_search.py`), print the whole catalogue
(`programme_catalogue.py`), and draw the steering loss against the control
effort (`control_effort_comparison.py`).

## Tests

```sh
uv run pytest
```

The atmosphere, the orbit determination and the pitch programmes are checked
against closed forms; the integration against the rocket equation and against
itself at half the step; the two estimates and every catalogue entry against
the catalogue itself. `tests/test_reference.py` pins the published velocity
budget above to one decimal place, which ties the whole chain down at once.

What each test file covers, and the note on the figures the dissertation
prints, is in [docs/tests.md](docs/tests.md).

## Citing

If you use this model, please cite it — `CITATION.cff` carries the citation
metadata. The DOI of the archived release goes in there once Zenodo has minted
it, alongside the version and the release date of the tag it was cut from.

## Licence

MIT, see `LICENSE`. The vehicle data in `config/` is taken from published
specifications; the drag coefficient profiles are generic ones for a slender
launch vehicle rather than measured data.
