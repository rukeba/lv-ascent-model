# The model

A two-dimensional model of the powered ascent, from lift-off to orbit
insertion. The vehicle flies a prescribed pitch programme; the model integrates
the trajectory that results, reports the orbit reached, and accounts for the
velocity spent on the way.

The flight is planar and written in polar coordinates about the centre of the
Earth: the distance `r` and the angle `psi` travelled from the pad. See
[`mission.py`](../src/ascent/mission.py).

## Two velocities

The state is integrated in the frame that rotates with the Earth, so the speed
the model carries is the speed relative to the atmosphere and to the launch
site. The inertial speed adds the rotation the pad already had, and it is the
one the orbit is built from.

Earth rotation is never added as an initial jump: the vehicle stands still on
the pad while the equations carry the centrifugal and Coriolis terms of the
rotating frame, so the two velocities stay consistent throughout. Launching due
north (`azimuth: 0`) makes them equal.

## Forces

- **Thrust** interpolated between the sea-level and vacuum figures by ambient
  pressure.
- **Gravity** as a central field on the measured gravitational parameter - see
  [constants](constants.md) - less the centrifugal term of the rotating frame.
- **Drag** from a Mach-dependent coefficient and the ICAO standard atmosphere,
  taken as zero above 100 km. See [atmosphere](atmosphere.md).

Thrust and drag act along the axis of the vehicle, which is the zero-lift
assumption of a vehicle flying at a small angle of attack.

## Guidance

While the pitch programme runs it fixes the direction of the velocity and only
the magnitude is integrated. The thrust deflection that would be needed to hold
that direction is recovered from the normal equation of motion, and the share of
thrust it points away from the velocity is accumulated as the steering loss -
see [the velocity budget](velocity-budget.md).

When the programme ends the vehicle holds the attitude it reached and both
velocity components are integrated to the end of the flight.

This is not the attitude control loop: that is the inner loop which holds the
vehicle on the commanded direction against disturbances, and the model has none
of it. What is here is the command.

## The orbit

The ascent is planar, so two-body motion is fully described by the specific
energy and the specific angular momentum. Once the engines are off and the
vehicle is out of the atmosphere those stop changing, and they are what the
resulting orbit is judged by. See [`orbit.py`](../src/ascent/orbit.py).

The orbit is built from the inertial velocity at the last recorded instant.
