"""Building a mission from YAML.

A mission file names the vehicle file next to it, the pitch programme and its
parameters, when the engines stop and where the launch site is. Programme and
cut-off types are looked up in the tables below, so a configuration file names
a model rather than an import path.
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

CUTOFFS = {
    'time': CutoffAtTime,
    'altitude': CutoffAtAltitude,
    'inertial-speed': CutoffAtInertialSpeed,
}


def load_mission(path: str | Path) -> Mission:
    """Read a mission file and everything it refers to."""
    path = Path(path)
    spec = _read(path)
    vehicle = load_vehicle(path.parent / f"{spec['vehicle']}.yaml")
    site = spec.get('launch_site', {})
    simulation = spec.get('simulation', {})

    return Mission(
        vehicle=vehicle,
        pitch_programme=_build(PITCH_PROGRAMMES, spec['pitch_programme'], 'pitch programme'),
        cutoff=_build(CUTOFFS, spec['cutoff'], 'cut-off'),
        target_altitude=spec['target_altitude'],
        duration=simulation.get('duration', 600.0),
        steps_per_second=simulation.get('steps_per_second', 10),
        latitude_deg=site.get('latitude', 0.0),
        azimuth_deg=site.get('azimuth', 90.0),
    )


def load_vehicle(path: str | Path) -> LaunchVehicle:
    spec = _read(Path(path))
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


def _read(path: Path) -> dict[str, Any]:
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
