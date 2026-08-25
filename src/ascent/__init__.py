"""A two-dimensional model of the powered ascent of a launch vehicle.

The vehicle flies a prescribed pitch programme from the pad to orbit insertion;
the model integrates the resulting trajectory and reports the orbit it reaches
and the velocity it spent on the way.
"""

from .config import load_mission, load_vehicle
from .cutoff import CutoffAtAltitude, CutoffAtInertialSpeed, CutoffAtTime
from .estimates import (analytic_altitude, equivalent_time, required_velocity,
                        vacuum_time)
from .losses import VelocityBudget, velocity_budget
from .mission import Mission
from .orbit import Orbit, orbit_from_state
from .pitch import (BilinearTangentProgramme, FivePhaseProgramme,
                    VelocityShareProgramme, bilinear_coefficients)
from .search import Candidate, Range, SearchResult, plan, search
from .summary import (summarise, summarise_found, summarise_plan,
                      summarise_search)
from .telemetry import Telemetry
from .vehicle import LaunchVehicle, Stage

__all__ = [
    'BilinearTangentProgramme', 'Candidate', 'CutoffAtAltitude',
    'CutoffAtInertialSpeed', 'CutoffAtTime', 'FivePhaseProgramme',
    'LaunchVehicle', 'Mission', 'Orbit', 'Range', 'SearchResult', 'Stage',
    'Telemetry', 'VelocityBudget', 'VelocityShareProgramme',
    'analytic_altitude', 'bilinear_coefficients', 'equivalent_time',
    'load_mission', 'load_vehicle', 'orbit_from_state', 'plan',
    'required_velocity', 'search', 'summarise', 'summarise_found',
    'summarise_plan', 'summarise_search', 'vacuum_time', 'velocity_budget',
]
