"""Building a mission from YAML.

A mission file names the vehicle file next to it, the pitch programme and its
parameters, when the engines stop and where the launch site is. Programme and
cut-off types are looked up in the tables below, so a configuration file names
a model rather than an import path.

The catalogue holds the same kind of specification many times over - one per
vehicle, programme and target altitude - so a solved parameter set can be
flown without writing a mission file for it. It is kept as one file a vehicle,
and `load_catalogue` on the directory reads the lot of them.

See docs/configuration.md
"""

from pathlib import Path
from typing import Any

import yaml

from .cutoff import Cutoff, CutoffAtAltitude, CutoffAtInertialSpeed, CutoffAtTime
from .mission import Mission
from .pitch import (BilinearTangentProgramme, FivePhaseProgramme, PitchProgramme,
                    VelocityShareProgramme)
from .vehicle import LaunchVehicle, Stage

PITCH_PROGRAMMES = {
    'five-phase': FivePhaseProgramme,
    'velocity-share': VelocityShareProgramme,
    'bilinear-tangent': BilinearTangentProgramme,
}

# short names for the programmes above, accepted on the command line
PROGRAMME_ALIASES = {
    '5f': 'five-phase',
    'vs': 'velocity-share',
    'bt': 'bilinear-tangent',
}

CUTOFFS = {
    'time': CutoffAtTime,
    'altitude': CutoffAtAltitude,
    'inertial-speed': CutoffAtInertialSpeed,
}


def programme_name(name: str) -> str:
    """The full name of a pitch programme, from a short one or from itself.

    Anything else is handed back untouched, so that an unknown name is
    reported by whatever was going to look it up rather than here.
    """
    return PROGRAMME_ALIASES.get(name, name)


def load_mission(path: str | Path) -> Mission:
    """Read a mission file and everything it refers to."""
    path = Path(path)
    return mission_from_spec(read_spec(path), path.parent)


def mission_from_spec(spec: dict[str, Any], directory: str | Path) -> Mission:
    """Build a mission from an already-read specification.

    `directory` is where the vehicle file it names is looked for.
    """
    site = spec.get('launch_site', {})
    simulation = spec.get('simulation', {})

    return Mission(
        vehicle=load_vehicle(Path(directory) / f"{spec['vehicle']}.yaml"),
        pitch_programme=_build(PITCH_PROGRAMMES, spec['pitch_programme'], 'pitch programme'),
        cutoff=_build(CUTOFFS, spec['cutoff'], 'cut-off'),
        target_altitude=spec['target_altitude'],
        duration=simulation.get('duration', 600.0),
        steps_per_second=simulation.get('steps_per_second', 10),
        latitude_deg=site.get('latitude', 0.0),
        azimuth_deg=site.get('azimuth', 90.0),
        site_name=site.get('name', ''),
    )


# What a catalogue file is called: the word, the vehicle it holds and nothing
# else. One file a vehicle, because a recomputed vehicle should touch its own
# file only
CATALOGUE_FILES = 'catalogue.*.yaml'


def load_catalogue(path: str | Path = 'config') -> list[dict[str, Any]]:
    """The solved parameter sets, as a list of mission specifications.

    A directory is every catalogue file in it - `catalogue.f9.yaml` and its
    neighbours, one a vehicle - concatenated in the order their names sort in.
    A file is that file alone, which is the sets of the one vehicle.
    """
    path = Path(path)
    if not path.is_dir():
        return read_spec(path)['missions']

    files = sorted(path.glob(CATALOGUE_FILES))
    if not files:
        raise FileNotFoundError(
            f'no catalogue in {path}: it is kept as one file a vehicle, named '
            f'{CATALOGUE_FILES}, and there is none there')
    return [spec for file in files for spec in read_spec(file)['missions']]


def find_in_catalogue(catalogue: list[dict[str, Any]], vehicle: str,
                      target_altitude: float, programme: str) -> dict[str, Any]:
    """The entry for one vehicle, target altitude and pitch programme."""
    for spec in catalogue:
        if (spec['vehicle'] == vehicle
                and spec['target_altitude'] == target_altitude
                and spec['pitch_programme']['type'] == programme):
            return spec

    available = sorted({entry['target_altitude'] / 1000 for entry in catalogue
                        if entry['vehicle'] == vehicle
                        and entry['pitch_programme']['type'] == programme})
    raise LookupError(
        f'no {programme} entry for {vehicle} at {target_altitude / 1000:g} km; '
        + (f'altitudes available: {", ".join(f"{a:g}" for a in available)} km'
           if available else f'no {programme} entries for {vehicle} at all'))


def load_vehicle(path: str | Path) -> LaunchVehicle:
    spec = read_spec(Path(path))
    return LaunchVehicle(
        name=spec['name'],
        stages=[Stage(**stage) for stage in spec['stages']],
        drag_coefficient={float(mach): value
                          for mach, value in spec['drag_coefficient'].items()},
        design_dynamic_pressure=spec.get('design_dynamic_pressure'),
    )


def resolve(name: str, directory: str | Path = 'config') -> Path:
    """Accept either a path to a mission file or a short name such as `f9`."""
    path = Path(name)
    if path.suffix in ('.yaml', '.yml'):
        return path
    return Path(directory) / f'mission.{name}.yaml'


def read_spec(path: Path) -> dict[str, Any]:
    """Read one YAML file, with a clear message when it is not there."""
    if not path.exists():
        raise FileNotFoundError(f'configuration file not found: {path}')
    with open(path, 'rb') as handle:
        return yaml.safe_load(handle)


def _build(registry: dict, spec: dict, what: str) -> PitchProgramme | Cutoff:
    arguments = dict(spec)
    kind = arguments.pop('type')
    if kind not in registry:
        raise ValueError(
            f'unknown {what} {kind!r}, expected one of {sorted(registry)}')
    return registry[kind](**arguments)
