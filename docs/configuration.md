# Configuration files

Everything in [`config/`](../config). YAML, read by
[`config.py`](../src/ascent/config.py). A configuration file names a model -
`five-phase`, `time` - rather than an import path.

## A mission file

`config/mission.<name>.yaml`. Names the vehicle file beside it, the pitch
programme and its parameters, when the engines stop, and where the launch site
is:

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

The parameters each programme takes are in [pitch
programmes](pitch-programmes.md).

`f9`, `a62` and `h3` are short names for `config/mission.<name>.yaml`. A mission
given by path brings its own vehicle with it, since a mission names its vehicle
file as a neighbour rather than borrowing one from the configuration directory.

## A cut-off policy

| `type` | fires |
|---|---|
| `time` | at a stated instant |
| `altitude` | the first time the altitude reaches a threshold |
| `inertial-speed` | the first time the inertial speed does |

The first knows its own instant, so the integration step is cut exactly there.
The other two have to watch the flight, and the instant is solved for inside the
step - see [integration](integration.md).

Both watched thresholds are crossed on the way up and fall again afterwards -
the inertial speed as soon as the vehicle coasts uphill, the altitude past
apogee. Reading the condition afresh each time would relight the engines there,
on a stage that has propellant left, which is not what a cut-off means. So the
first crossing is latched.

## A vehicle file

`config/lv.<name>.yaml`. Lists the stages in the order they burn, each taking
over at its own `ignition_time`, along with the drag coefficient against Mach
number and the dynamic pressure the airframe is designed for - reported, not
enforced.

**The mass of the vehicle is the sum over the stages still on it**, so an entry
holds only what it *adds* to the stack. For the two vehicles with strap-on
boosters that means the boosted phase carries the boosters and the core
propellant it spends, and the core itself belongs to the entry that flies on
after separation. See [the vehicles](vehicles.md).

**The last stage carries no propellant**: it is the payload.

`config/lv.f9.yaml` is the simple case and `config/lv.a62.yaml` the other.

## The catalogue

`config/catalogue.<name>.yaml`, one file a vehicle. An ordinary mission
specification with two blocks added. See [the catalogue](catalogue.md).
