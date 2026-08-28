# Pitch programmes

A pitch programme is the law that turns the vehicle from the vertical it lifts
off on to the horizontal an orbit needs - the prescribed flight-path angle
`gamma(t)`, and with it the share of the engine's work that goes into altitude
rather than into speed along the horizon.

It is not the attitude control loop. What is here is the command; the inner loop
that holds the vehicle on it is not modelled.

Three families are implemented, and it is their parameters that are being
optimised. See [`pitch.py`](../src/ascent/pitch.py).

- [Five-phase turn](pitch-five-phase.md) - prescribes the pitch **rate**
- [Velocity share](pitch-velocity-share.md) - prescribes `sin(gamma)`
- [Bilinear tangent](pitch-bilinear-tangent.md) - prescribes `tan(gamma)`

All three prescribe the same thing and disagree about what to prescribe it
*with*, which is what makes them worth comparing. See [what the three
cost](programme-comparison.md).

## What they share

**Tabulation.** Each is tabulated on a uniform tenth-of-a-second grid once, at
construction, and read back by interpolation. So the shape of a programme never
enters the equations of motion - only its value at an instant does. The grid
step, `GRID_STEP`, is also the finest a timeline is ever issued to, which is why
the search rounds every instant to it.

Interpolated rather than snapped to the nearest tabulated point: a multi-stage
integrator probes the middle of a step, and a staircase there caps the order of
accuracy whatever the scheme.

**A fifth phase.** Each ends before the engines do. After its last instant the
vehicle holds the attitude it reached and flies on it to cut-off.

**Refusal.** Each refuses parameters that are not a turn - phases out of order,
a share that leaves `[0, 1]`, a tangent whose denominator passes through zero
inside the turn. A search reads that refusal as a node of the grid that is not
in the family, rather than as an error.

## The parameters

| Programme | Parameters |
|---|---|
| `five-phase`, `5f` | `t1` end of the vertical rise, `t4` end of the programme, `k2` and `k3` the shares of the turn spent building up and holding the pitch rate |
| `velocity-share`, `vs` | `t1` end of the vertical rise, `tf` end of the turn, `te` end of the burn, `s` how full the turn is, between -3 and 3 |
| `bilinear-tangent`, `bt` | `t1` start of the turn, `a`, `b`, `c` of `tan(gamma) = (a*tau + b) / (c*tau + 1)`, `te` end of the programme |

Both families that can be aimed - the five-phase turn and the bilinear tangent -
take a final angle as well, zero by default because a circular orbit is entered
along the horizon.
