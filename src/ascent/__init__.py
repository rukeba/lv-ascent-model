"""A two-dimensional model of the powered ascent of a launch vehicle.

The vehicle flies a prescribed pitch programme from the pad to orbit insertion;
the model integrates the resulting trajectory and reports the orbit it reaches
and the velocity it spent on the way.
"""

from .config import load_mission, load_vehicle
from .cutoff import CutoffAtAltitude, CutoffAtInertialSpeed, CutoffAtTime
from .losses import VelocityBudget, velocity_budget
from .mission import Mission
from .orbit import Orbit, orbit_from_state
from .pitch import (BilinearTangentProgramme, FivePhaseProgramme,
                    VelocityShareProgramme, bilinear_coefficients)
from .summary import summarise
from .telemetry import Telemetry
from .vehicle import LaunchVehicle, Stage

__all__ = [
    'BilinearTangentProgramme', 'CutoffAtAltitude', 'CutoffAtInertialSpeed',
    'CutoffAtTime', 'FivePhaseProgramme', 'LaunchVehicle', 'Mission', 'Orbit',
    'Stage', 'Telemetry', 'VelocityBudget', 'VelocityShareProgramme',
    'bilinear_coefficients', 'load_mission', 'load_vehicle', 'orbit_from_state',
    'summarise', 'velocity_budget',
]
