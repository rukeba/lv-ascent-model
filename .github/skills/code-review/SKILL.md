---
name: code-review
description: How to review this repository - a numerical model of the powered ascent of a launch vehicle. Explains what correctness means here, which invariants a change must not break, and which apparent oddities are deliberate.
---

# Reviewing lv-ascent-model

This is a numerical physics model, not an application. A change that reads well
and returns plausible numbers can still be wrong by several metres per second,
and nothing will fail. Weight the review accordingly: correctness of the
equations, of the frames they are written in, and of the integration that
solves them comes first. Style comes a distant last.

Every published figure in this repository was produced by this code. A change
that moves those figures is a change to the results, whether or not it was
meant as one.

## What correctness means here

**Two velocities, never interchangeable.** The state is integrated in the frame
that rotates with the Earth. `FlightState.speed` is relative to that frame -
relative to the atmosphere and to the launch pad - and it is what the
aerodynamics, the dynamic pressure, the Mach number and the pitch programme
must use. `FlightState.inertial_speed` adds the rotation the pad already had,
`hypot(horizontal + omega * radius, vertical)`, and it is what the orbit, the
insertion criterion and the centripetal balance must use. Mixing them is the
most likely error in any change to `mission.py`, `losses.py` or `summary.py`,
and it is worth roughly 400 m/s at Cape Canaveral. Flag any use of one where
the other belongs.

`omega` is the Earth's rotation projected on to the launch plane, so it is zero
for a launch due north and negative to the west. Earth rotation is never added
as an initial velocity: the vehicle stands still on the pad while the equations
carry the centrifugal and Coriolis terms. A change that adds a starting
velocity instead would double-count it.

**Events, not order of accuracy.** Staging, engine cut-off, the end of the
pitch programme and a tank running dry are step changes in mass, thrust or in
the equations themselves. `Mission._segment_bounds` cuts the step at each of
them and `Mission._solve_exhaustion` locates a dry tank by regula falsi rather
than estimating it. At around 60 m/s^2 an event misplaced by one step at 10 Hz
costs several m/s - far more than the integration scheme itself. Any change
that lets a step span an event, or that replaces the solve with a linear
estimate, is a correctness bug even though every test of the scheme still
passes.

**The derivative function must stay pure.** `_guided_rates` and `_free_rates`
are evaluated four times per step at trial points, including the middle of the
step and points that are never reached. They must not mutate anything. This is
why the propellant burned travels in the state vector rather than on the stage.
Any assignment to `self`, to a stage or to a state object inside them - or any
memoisation keyed on time - silently breaks RK4 down to first order while still
looking about right. The same applies to `PitchProgramme.sample`, which is
called at trial times.

**Units.** SI everywhere inside the model: metres, m/s, kilograms, newtons,
radians, seconds. Degrees appear in exactly three places - configuration files,
`Telemetry` columns, and anything printed. A conversion anywhere else is a bug.
`losses.py` reads angles back out of telemetry, so it converts from degrees on
purpose.

**Invariants worth checking a change against.** A launch due north must make
the two velocities identical. A circular orbit must stay circular. Horizontal
drag-free flight must satisfy the rocket equation exactly. Halving the step
must barely move the answer.

## Deliberate choices - please do not "fix" these

- **`constants.py` computes `MU` as `G * MASS` from its own values**, which
  differs from the standard 3.986004418e14 in the fourth digit. Every recorded
  result depends on it. If you think it should change, say so once as a
  modelling question, not as a defect.
- **Full-precision literals in the `config/catalogue.*.yaml` files.** Near a circular orbit
  the apogee answers to the cut-off time at some 80 km per second, and the
  bilinear tangent's numerator `a*tau + b` cancels to almost nothing at the end
  of the turn - which is what levels the vehicle out. Rounding those numbers
  moves the perigee by a kilometre. They are not noise.
- **Gaps in the catalogue.** Not every programme reaches every orbit; the file
  header says which and why. Absent combinations are results, not omissions.
- **British spelling** (`programme`, `minimise`) and the module-level docstring
  on every file are house style.
- **`euler` is absent on purpose.** There is one integration scheme.

## Things that would be genuinely useful to hear about

- An equation of motion that is wrong, misprojected, or missing a term of the
  rotating frame.
- A quantity taken from the wrong end of a step, or a state read before it is
  written, in `Mission._collect` and `_accumulate_steering`.
- Division by a quantity that can reach zero: a radius, a mass, a speed of
  sound, a specific impulse, a thrust, a denominator in `pitch.py`.
- An index or interpolation in `PitchProgramme.sample` or `Telemetry.at` that
  can run off the end of its array.
- A stage sequence, throttle or propellant bookkeeping that can double-count or
  lose mass across staging.
- Silent failure: a path that returns a number where it should raise, or that
  lets a non-finite value propagate.
- A test that would still pass if the physics it covers were broken.

## Things not worth a comment

Line length, import order, f-string versus format, type-annotation
completeness, docstring wording, and suggestions to add abstraction for its own
sake. If the only issue in a file is stylistic, say the file is fine.
