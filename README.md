# lv-ascent-model

A two-dimensional model of the powered ascent of a launch vehicle, from
lift-off to orbit insertion. The vehicle flies a prescribed pitch programme;
the model integrates the trajectory that results, reports the orbit it reaches,
and accounts for the velocity spent on the way — against gravity, against the
air, and on steering the thrust away from the velocity in order to fly the
programme. That last figure is what different pitch programmes are compared by.

Three programmes are implemented — a five-phase turn, a turn parametrised by
the vertical share of the velocity, and a bilinear tangent — and three vehicles
are configured: Falcon 9, Ariane 62 and H3-22S.

## Installing and running

```sh
uv sync

uv run ascent f9                            # summary of the flight on the console
uv run ascent f9 --csv out/f9.csv           # and the whole trajectory as CSV
uv run ascent f9 --report out/f9            # and an HTML report with plots
uv run ascent config/mission.a62.yaml       # a mission file by path

uv run ascent f9 --list                     # solved parameter sets on file
uv run ascent f9 --altitude 650             # fly one of them
uv run ascent f9 --altitude 650 --programme bilinear-tangent
```

`f9`, `a62` and `h3` are short names for `config/mission.<name>.yaml`.

The console summary lists the set-up, the notable instants of the flight, the
state at engine cut-off, the orbit reached and the velocity budget. The HTML
report shows the same figures plus plots and a flight log.

## The model

The flight is planar and written in polar coordinates about the centre of the
Earth: the distance `r` and the angle `psi` travelled from the pad.

**Two velocities.** The state is integrated in the frame that rotates with the
Earth, so the speed the model carries is the speed relative to the atmosphere
and to the launch site. The inertial speed adds the rotation the pad already
had, and it is the one the orbit is built from. Earth rotation is never added
as an initial jump: the vehicle stands still on the pad while the equations
carry the centrifugal and Coriolis terms of the rotating frame, so the two
velocities stay consistent throughout. Launching due north (`azimuth: 0`) makes
them equal.

**Forces.** Thrust interpolated between the sea-level and vacuum figures by
ambient pressure; gravity as a central field; drag from a Mach-dependent
coefficient and the ICAO standard atmosphere, taken as zero above 100 km.
Thrust and drag act along the axis of the vehicle, which is the zero-lift
assumption of a vehicle flying at a small angle of attack.

**Guidance.** While the pitch programme runs it fixes the direction of the
velocity and only the magnitude is integrated; the thrust deflection that would
be needed to hold that direction is recovered from the normal equation of
motion, and the share of thrust it points away from the velocity is accumulated
as the steering loss. When the programme ends the vehicle holds the attitude it
reached and both velocity components are integrated to the end of the flight.

**Integration.** Fourth-order Runge-Kutta at a fixed step, typically 10 Hz. The
step is cut exactly at every discontinuity inside it — stage separation, engine
cut-off, the end of the programme — and a tank running dry is solved for by
regula falsi rather than estimated. This matters more than the order of the
scheme: at around 60 m/s² an event misplaced by one step at 10 Hz is worth
several m/s, far more than the error of the scheme itself.

## Modules

| Module | What is in it |
|---|---|
| `mission.py` | the equations of motion, the stepping that solves them, the cutting of the step at events, and the steering-loss accounting |
| `pitch.py` | the three pitch programmes and the tabulation they share |
| `vehicle.py` | stages, propulsion, mass and drag of the launch vehicle |
| `atmosphere.py` | ICAO standard atmosphere and the gravity field |
| `orbit.py` | the osculating orbit recovered from a position and an inertial velocity |
| `losses.py` | the velocity budget: gravity, aerodynamic and steering losses |
| `integrators.py` | the Runge-Kutta step, knowing nothing about rockets |
| `cutoff.py` | when the engines stop — by time, by altitude or by inertial speed |
| `state.py` | one sample of the flight |
| `telemetry.py` | the recorded flight and its CSV form |
| `summary.py` | the console summary |
| `report.py` | the HTML report with plots |
| `config.py` | building a mission from YAML, and reading the catalogue |
| `cli.py` | the `ascent` command |
| `constants.py` | constants of the Earth model |

## Configuration

A mission file names the vehicle file beside it, the pitch programme and its
parameters, when the engines stop, and where the launch site is:

```yaml
vehicle: lv.f9
target_altitude: 500_000       # altitude of the circular orbit aimed for, m
launch_site:
  latitude: 28.5
  azimuth: 90                  # degrees from north: 90 is due east
pitch_programme:
  type: five-phase             # or velocity-share, or bilinear-tangent
  t1: 20.0
  t4: 502.8
  k2: 0.056178
  k3: 0.522859
cutoff:
  type: time                   # or altitude, or inertial-speed
  time: 502.8
simulation:
  duration: 600
  steps_per_second: 10
```

A vehicle file lists the stages in the order they burn, each taking over at its
own `ignition_time`, along with the drag coefficient against Mach number. The
mass of the vehicle is the sum over the stages still on it, so an entry holds
only what it adds to the stack - which for the two vehicles with strap-on
boosters means the boosted phase carries the boosters and the core propellant
it spends, and the core itself belongs to the entry that flies on after
separation. The last stage carries no propellant: it is the payload. See
`config/lv.f9.yaml` for the simple case and `config/lv.a62.yaml` for the other.

The three programmes take these parameters:

| Programme | Parameters |
|---|---|
| `five-phase` | `t1` end of the vertical rise, `t4` end of the programme, `k2` and `k3` the shares of the turn spent building up and holding the pitch rate |
| `velocity-share` | `t1` end of the vertical rise, `tf` end of the turn, `te` end of the burn, `s` how much of the turn is done early, between -3 and 3 |
| `bilinear-tangent` | `t1` start of the turn, `a`, `b`, `c` of `tan(gamma) = (a*tau + b) / (c*tau + 1)`, `te` end of the programme |

## Catalogue of solved parameter sets

`config/catalogue.yaml` holds parameters that place each vehicle on a circular
orbit of a given altitude - one set per vehicle, pitch programme and altitude,
42 in all. Falcon 9 is covered from 400 to 700 km, Ariane 62 from 400 to
900 km and H3 from 1000 to 1200 km.

Each set is defined by its terminal condition: perigee and apogee both at the
target. The vertical rise `t1` is held at 20 s and the programme ends at
cut-off. The five-phase turn holds `k2` at 0.05 as well: minimising the loss
drives that share of the turn to zero, which is a step in pitch rate and
defeats the point of the phase, so it is a design choice rather than something
the terminal condition can settle - which leaves `k3` and the cut-off time,
two unknowns for two conditions. The other two programmes keep a third
parameter, and among the sets meeting the condition the search preferred the
smaller steering loss. Every entry records the orbit it produces and what it
costs, and `tests/test_catalogue.py` flies all of them to check that it still
does.

```sh
uv run python examples/programme_catalogue.py    # the whole table
```

Two things about the file are worth knowing. The numbers are written to full
precision because near a circular orbit the apogee answers to the cut-off time
at some 80 km per second, and the bilinear tangent is more delicate still - its
numerator cancels to almost nothing at the end of the turn, which is what makes
the vehicle level out, so the last digits of `a` and `b` carry the terminal
angle. And the table has gaps, which are properties of the programmes and of
the vehicles rather than unfinished work: Ariane 62 gives out a little above
900 km, having all but emptied its upper stage by cut-off; the velocity-share
quartic gives out a hundred kilometres earlier still, as it does at 600 km on
Falcon 9; and the five-phase turn covers only the middle of each range, having
no freedom left to stretch to the ends once `k2` is fixed. It places no orbit
at all for Ariane 62, whose turn would have to be one continuous manoeuvre
while the vehicle spends its last several hundred seconds on a low-thrust upper
stage.

## Reproducing a published result

`examples/steering_loss_comparison.py` flies Falcon 9 into a 500 km circular
orbit from Cape Canaveral three times, once per pitch programme, each with the
parameters that minimise its steering loss subject to reaching that orbit, and
prints the velocity budget next to the published figures:

```sh
uv run python examples/steering_loss_comparison.py
```

```
programme                gravity  aerodynamic  steering     total   perigee   apogee
five-phase                2326.7         29.3     578.7    2934.7     500.1    501.0
velocity-share            2290.9         29.7     460.0    2780.6     500.4    507.6
bilinear-tangent          2242.1         29.6     485.2    2757.0     500.1    500.5

largest deviation from the published figures: 0.04 m/s
```

The programme with the smallest steering loss is not the one with the smallest
total: it pays for the saving in gravity losses.

## Tests

```sh
uv run pytest
```

The atmosphere, the orbit determination and the pitch programmes are checked
against closed forms. The integration is checked against the rocket equation,
which horizontal drag-free flight satisfies exactly, and against itself at half
the step — how far a result moves under refinement is its numerical error, and
a mis-timed event looks exactly like that. `tests/test_reference.py` pins the
published velocity budget above to one decimal place, which ties the whole
chain down at once, and `tests/test_catalogue.py` flies every catalogue entry
against the orbit and the losses recorded for it.

## Citing

If you use this model, please cite it — `CITATION.cff` carries the citation
metadata. The DOI of the archived release goes in there once Zenodo has minted
it, alongside the version and the release date of the tag it was cut from.

## Licence

MIT, see `LICENSE`. The vehicle data in `config/` is taken from published
specifications; the drag coefficient profiles are generic ones for a slender
launch vehicle rather than measured data.
