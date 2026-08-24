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

## Installing and running

```sh
uv sync

uv run ascent f9                            # summary of the flight on the console
uv run ascent f9 --csv out/f9.csv           # and the whole trajectory as CSV
uv run ascent f9 --report                   # an HTML report in out/f9, opened
uv run ascent f9 --report out/run-12        # or wherever it is wanted
uv run ascent config/mission.a62.yaml       # a mission file by path

uv run ascent f9 --list                     # solved parameter sets on file
uv run ascent f9 --altitude 650             # fly one of them
uv run ascent f9 --altitude 650 --programme bilinear-tangent
uv run ascent f9 -a 650 -p bt               # the same, in short

uv run ascent-search f9 --altitude 500      # solve for a set instead of flying one
uv run ascent-search f9 -a 650 -p bt --yaml
```

`f9`, `a62` and `h3` are short names for `config/mission.<name>.yaml`, and the
three pitch programmes answer to `5f`, `vs` and `bt` as well as to their full
names. `--altitude` is `-a` and `--programme` is `-p` on both commands.

The console summary lists the set-up, the notable instants of the flight, the
state at engine cut-off, the orbit reached and the velocity budget.

`--report` writes those same figures as a page — laid out as cards, with the
velocity budget drawn to scale — and adds ten plots and the trajectory
tabulated every five seconds. The plots are PNG files beside the page, drawn
well above screen resolution so that they stay sharp when opened full size or
printed; the styles are inlined, so the page can be sent on with the images
next to it. It is opened in a browser as soon as it is written, which
`--no-open` suppresses. Given no directory it writes to `out/` and the name of
the vehicle file, so `ascent f9 --report` lands in `out/f9`. The page carries
the command that produced it, so a report found months later says how to make
it again.

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
ambient pressure; gravity as a central field, less the centrifugal term of the
rotating frame; drag from a Mach-dependent coefficient and the ICAO standard
atmosphere, taken as zero above 100 km. The velocity budget is projected the
same way the equations of motion are: what the propellant delivered, less the
gravity and aerodynamic losses, is the speed reached, to within a metre or two.
The steering loss is not in that sum - it is the price of holding the
programme, recovered afterwards rather than paid by the trajectory.
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

## The three pitch programmes

A pitch programme is the law that turns the vehicle from the vertical it lifts
off on to the horizontal an orbit needs — the prescribed flight-path angle
`gamma(t)`, and with it the share of the engine's work that goes into altitude
rather than into speed along the horizon. It is not the attitude control loop:
that is the inner loop which holds the vehicle on the commanded direction
against disturbances, and this model has none of it. What is here is the
command.

All three families below prescribe the same thing and disagree about what to
prescribe it *with*, which is what makes them worth comparing. Each is
tabulated on a tenth-of-a-second grid at construction and read back by
interpolation, so the shape of a programme never enters the equations of motion
— only its value at an instant does. Each ends before the engines do; after its
last instant the vehicle holds the attitude it reached and flies on it to
cut-off.

### Five-phase turn

What is prescribed is the pitch **rate**, as a trapezium, and the angle follows
by integrating it. Five phases: a vertical rise to `t1`; the rate built up from
zero over the share `k2` of the turn; the turn held at that rate over the share
`k3`; the rate arrested over what is left, so that the angle arrives at the
horizon exactly at `t4`; and then the fifth phase, free flight on the attitude
reached, to cut-off.

Prescribing the rate rather than the angle is the point of the family. A turn
written as an angle leaves the rate and the angular acceleration as derived
quantities, and those are exactly what actuator authority and bending loads are
written in terms of; a trapezium in the rate makes the rate continuous at every
joint by construction and the angular acceleration piecewise constant and
bounded, so the control moment stays finite and the programme is something an
attitude loop could actually hold. The requirement that the turn cover the
whole 90 degrees closes the family analytically: the working rate follows from
the angle to be covered, so `t1`, `t4`, `k2` and `k3` are the whole of it.

`k2` and `k3` are shares rather than times, which is what lets a set of
parameters carry across vehicles of different classes and burn lengths. Two
levers set how flat the turn is — the length of the manoeuvre and the share of
it spent at a constant rate — and both rise with the target altitude while the
peak rate falls with it — a higher orbit needs a longer burn, and a longer manoeuvre can be
made flatter, spending the propellant on horizontal speed rather than on
holding the thrust away from the horizon (R. Keba and A. M. Kulabukhov,
*Journal of Rocket-Space Technology* **34**(4), 115–122 (2025),
[doi:10.15421/452553](https://doi.org/10.15421/452553)).

### Velocity share

What is prescribed is `eta = V_vertical / V = sin(gamma)`: the share of the
speed that is pointed up. The family comes out of launch telemetry — across
Falcon 9 flights that share starts at one, falls monotonically, and arrives at
very nearly zero at cut-off, staying inside `[0, 1]` throughout — and out of
what that split buys. Taking the pair "speed magnitude and vertical share"
rather than two independent velocity components separates the energetics of the
flight from the geometry of the turn: the magnitude comes from the vehicle's
own thrust and mass through the Tsiolkovsky equation and so is attainable by
construction, while the share carries the whole of the steering. That is what
makes the altitude reached an integral of the programme, `h = integral of
eta*V dt`, which can be evaluated without integrating the equations of motion —
and it is the estimate the search screens its grid with.

The share is prescribed by phases: one over the vertical rise, a quartic over
the turn from `t1` to `tf`, and zero from `tf` to cut-off, where the velocity
is already in the horizon. The quartic is the lowest-degree polynomial that can
meet the four boundary conditions — one and zero at the ends, flat at both — and
still keep a free parameter, and `s` is that parameter: it sets how full the
turn is, how long the share lingers near one, and so how much altitude the
ascent accumulates. Outside `|s| <= 3` the quartic leaves `[0, 1]` and stops
being a turn, which is why the family refuses it. Being flat at both ends is
what makes the turn join the vertical rise and the horizontal phase without a
kink in the rate, and what makes a turn that runs all the way to cut-off leave
the vehicle on the horizon anyway: the double root holds the share under `1e-7`
at the last tabulated instant.

The method behind this family is not published yet; its DOI belongs here and
is a placeholder until it is —
[doi:XX.YYYYY/ZZZZZ](https://doi.org/XX.YYYYY/ZZZZZ).

### Bilinear tangent

What is prescribed is `tan(gamma) = (a*tau + b) / (c*tau + 1)`, with `tau`
counted from the end of the vertical rise. This is the classical optimal
steering law of powered flight — what the calculus of variations returns for a
flat Earth, constant gravity and no atmosphere — and it is here as an explicit
programme with its coefficients as parameters rather than as something solved
for. It has no phases: one expression covers the turn from `t1` to `te`.

Three things about it are worth knowing before fitting it. Its numerator
cancels to almost nothing at the end of the turn, and that cancellation is what
levels the vehicle out, so the last digits of `a` and `b` carry the terminal
angle — round them and the perigee moves by a kilometre. Its coefficients are
nearly degenerate, in that scaling `b` and `c` together leaves almost the same
turn, which is why the search grids the angles the turn passes through instead.
And it steps the angle at `t1`, from the vertical straight to `arctan(b)`,
so how far that start angle is from 90 degrees is the size of a discontinuity
rather than a free choice; the eighteen sets on file start between 84.7 and
89.2 degrees.

### What they cost

Flown into the same orbit by the same vehicle, the three differ mostly in what
they spend. The steering loss — the share of the thrust that holding the
programme points away from the velocity — is what a programme is judged by, and
gravity is what it trades against: a flatter turn steers less and climbs
longer. `examples/steering_loss_comparison.py` flies all three into a 500 km
orbit and prints the three budgets side by side; the programme with the
smallest steering loss is not the one with the smallest total.

## Modules

| Module | What is in it |
|---|---|
| `mission.py` | the equations of motion, the stepping that solves them, the cutting of the step at events, and the steering-loss accounting |
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

## Configuration

A mission file names the vehicle file beside it, the pitch programme and its
parameters, when the engines stop, and where the launch site is:

```yaml
vehicle: lv.f9
target_altitude: 500_000       # altitude of the circular orbit aimed for, m
launch_site:
  name: Cape Canaveral SLC-40, Florida   # reported, and nothing more
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

The three programmes take these parameters — what each of them means is above,
under [The three pitch programmes](#the-three-pitch-programmes):

| Programme | Parameters |
|---|---|
| `five-phase`, `5f` | `t1` end of the vertical rise, `t4` end of the programme, `k2` and `k3` the shares of the turn spent building up and holding the pitch rate |
| `velocity-share`, `vs` | `t1` end of the vertical rise, `tf` end of the turn, `te` end of the burn, `s` how full the turn is, between -3 and 3 |
| `bilinear-tangent`, `bt` | `t1` start of the turn, `a`, `b`, `c` of `tan(gamma) = (a*tau + b) / (c*tau + 1)`, `te` end of the programme |

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

## Searching for a parameter set

`ascent-search` solves the problem the catalogue holds the answers to. Give it
a vehicle, a circular orbit and one of the three programme families, and it
returns the parameters that reach that orbit — and, among the sets that do, the
ones that reach it soonest.

```sh
uv run ascent-search f9 --altitude 500
uv run ascent-search f9 --altitude 650 --programme bilinear-tangent
uv run ascent-search a62 --altitude 700 --yaml            # as a catalogue entry
uv run ascent-search f9 --altitude 500 --report           # fly it and report it
uv run ascent-search h3 --altitude 1100 --coarse 0.5      # a quicker, rougher look
uv run ascent-search f9 --altitude 500 --workers 1        # in this process alone
```

The mission file supplies the vehicle and the launch site and nothing else;
`--altitude` and `--programme` — or `-a` and `-p` — say what to search for. `--yaml` prints the set
found as a catalogue entry, ready to paste; `--report` turns it into that same
entry, flies it and writes the page `ascent --report` writes, so a set that was
searched for and one that was filed give the same report. A set that misses the
orbit is not an entry: it is printed, and it is not flown. A search prints its progress as it
runs — which pass it is on, how many trajectories it has integrated and roughly
how much longer it will take.

**What is gridded and what is solved.** The grid runs over the shape of the
turn, and only over the shape: one number for the five-phase family, two for
each of the others. The cut-off is not one of its axes. The two conditions of a
circular orbit divide between the parameters the way the dissertation this
model was written for divides them — the condition on the speed fixes the end
of the powered flight, the condition on the altitude fixes the shape of the
turn — and this follows that division. It has to: near a circular orbit the apogee answers to the
cut-off time at some 80 km per second, so a grid fine enough to resolve it
along that axis would be enormous and one coarse enough to afford would resolve
nothing. So at every node the cut-off is solved for instead, by driving the
semi-major axis to the radius at cut-off, which is what makes an orbit
circular. Every node the search flies therefore sits on a circular orbit
already, at its own altitude; the grid is looking for the shape whose altitude
is the one asked for.

**The two estimates.** Both come from the dissertation, both are quadrature
rather than integration, and both are in `estimates.py`:

- the **energy-equivalent ascent time** is the instant at which the propellant
  has bought the orbit — the characteristic velocity accumulated less what the
  orbit costs in energy less everything lost on the way. It bounds the cut-off:
  the search brackets its solve inside a window around the estimate, and the
  same balance says before anything is flown whether the vehicle has the
  propellant for the orbit at all. Measured against the catalogue it sits
  between 4.8 per cent high and 9.1 per cent low, and the window carries that
  band with something to spare. The method is published: R. Keba and A. M.
  Kulabukhov, *Journal of Rocket-Space Technology* **35**(1), 94–99 (2026),
  [doi:10.15421/452567](https://doi.org/10.15421/452567).
- the **analytic altitude integral** is the altitude a programme reaches,
  taken as the integral of the vertical component of the velocity with the
  speed from the Tsiolkovsky equation stage by stage and the flight-path angle
  read off the programme itself. It screens the grid: the altitude a shape
  would reach at either end of the cut-off window bounds what it can reach
  anywhere inside it, and a shape that cannot reach the target is dropped
  without a trajectory. It reads between 1.005 and 1.185 times the altitude the
  flight reaches — never low, because the air, the thrust deficit at sea level
  and the fall of gravity with altitude all push the same way — and the screen
  is that band applied backwards, widened to 0.95–1.40 because it is a gate: a
  node it rejects is never flown, and the measurement behind it is of three
  vehicles. The method behind it is not published yet; its DOI belongs here and
  is a placeholder until it is —
  [doi:XX.YYYYY/ZZZZZ](https://doi.org/XX.YYYYY/ZZZZZ).

Neither is accurate enough to stand in for a flight. Both are accurate enough
to say which flights are worth making, and `tests/test_estimates.py` checks
both bands against every entry in the catalogue, so the constants the search
relies on cannot drift away from the data they were measured on.

**Which node a pass closes in on.** Not simply the quickest one that reached
the orbit. At the resolution of an early pass, whether a node lands on the
orbit at all is largely luck, and a set half a kilometre out but two seconds
quicker is the better thing to look near. What the passes follow instead is the
cut-off each node would need to reach the target, read off the line its own
pass draws between the altitude reached and the instant of cut-off — a line
because the two are readings of the same energy. Where that leads into a corner
of the family from which the orbit cannot be reached, which happens on a
vehicle near its limit, the grid is run a second time for the orbit alone and
the better of the two answers is reported. Falcon 9 to 700 km on the bilinear
tangent is the case that needs it, and the summary says when it has happened.

**What it costs.** Eleven passes: the first over the whole range of the family,
then ten closing in, each one grid step wide about the best node of the pass
before and halving the step. A five-phase search integrates some seven hundred
trajectories, one of the two-axis families some three thousand, and twice that
where the grid has to be run again.

Not of wall-clock, though. The nodes of a pass are independent — each is its own
cut-off solved over its own handful of trajectories — so they are divided over
two thirds of the cores, which is seven times faster on a machine with fourteen
of them and turns several minutes into under one. It finds exactly the same
set: the nodes are collected in the order of the grid, so the answer does not
depend on how many processes answered it. `--workers` says how many, and
`--workers 1` searches in this process alone.

What the screen saves is almost all on the first pass, the only one that covers
the whole range of a family: of a Falcon 9 first pass to 500 km it drops four
fifths of the velocity-share nodes unflown, half of the bilinear-tangent ones
and a tenth of the five-phase ones, which have a single axis and less to
reject. On the passes after it, already gathered about an answer, it drops
nothing, and it is not meant to. `--steps` and `--coarse` are there for a
quicker look: the orbit a set reaches is the same to within a few metres at one
step a second as at ten, and so is the velocity budget: it is read off the last
powered row rather than off the cut-off itself, but by then the vehicle is
level and out of the air, so all three integrands are near zero there and the
part left out is fractions of a metre per second. The entry the search writes
out asks for ten steps a second whatever it was searched at.

**What the quicker ascent is paid for.** A quicker ascent is a flatter one,
and a flatter one goes faster lower down. The set found is reported with the
peak dynamic pressure it asks of the airframe, beside the figure the vehicle
file declares it is designed for, and with the peak thrust deflection it asks
of the guidance — and the first of those is not free. Searching Falcon 9 to
500 km on the bilinear tangent returns a set that cuts off at 499.202 s rather
than the 500.910 on file, and it peaks at 37.8 kPa against a design figure of
35. Neither figure enters the ranking unless you say so: `--max-q` puts the
airframe into the constraint, and a set that peaks above it is then not an
answer however quick it is, which is where a limit on the dynamic pressure
belongs in a search of this kind and where the dissertation puts it. Without
the flag the peaks are reported and nothing more, which is how the rest of the
model treats them.

```sh
uv run python examples/parameter_search.py       # all three families, side by side
```

**It does not reproduce the catalogue, and should not.** The catalogue
preferred the smallest steering loss among the sets that reach the orbit; this
prefers the earliest cut-off. For the five-phase family there is nothing to
prefer — two conditions and two unknowns leave no freedom — and the two agree
to the figures the grid resolves: searching Falcon 9 to 500 km returns a
cut-off of 502.693 s against the 502.707 on file, and the same velocity budget
to within a metre per second, as it does at 650 km and as H3 does at 1100. The
other two families keep a parameter to spend, and the search spends it
differently: Falcon 9 to 500 km on the velocity share cuts off at 501.696 s
rather than 502.188, for 2916.6 m/s of losses rather than 2996.4.

The second of those figures need not follow the first, and on H3 it does not.
An earlier cut-off is always less propellant burned — the burn is shorter — but
only the gravity and the aerodynamic terms are paid by the trajectory. The
steering loss is the price of holding the programme, recovered from the normal
equation after the fact rather than spent on the way, so a set can cut off
sooner and still be charged more of it. On H3 it is not even comparable: the
thrust cannot hold any of these programmes over part of the burn, the demand
saturates, and the figure stops measuring anything — which the search reports,
along with the peak, for exactly that reason.

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
five-phase                2568.7         29.3     526.5    3124.6     500.1    501.0
velocity-share            2537.9         29.7     411.1    2978.7     500.4    507.6
bilinear-tangent          2500.0         29.6     433.1    2962.7     500.1    500.5

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
