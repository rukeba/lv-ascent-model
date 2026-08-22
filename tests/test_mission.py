"""The integration itself: against a closed form, and against a finer step."""

import math

import numpy as np

from ascent import CutoffAtTime, LaunchVehicle, Mission, Stage, load_mission
from ascent.constants import EARTH_RADIUS, STANDARD_GRAVITY
from ascent.pitch import PitchProgramme


class Horizontal(PitchProgramme):
    """Flat programme: the velocity is horizontal for the whole flight."""

    def __init__(self, end_time: float) -> None:
        t = self._grid(end_time)
        zero = np.zeros_like(t)
        self._tabulate(t, zero, zero, zero)

    def describe(self) -> str:
        return 'horizontal'


def vacuum_vehicle(dry=1_000.0, propellant=9_000.0, thrust=100_000.0, isp=300.0):
    """A single stage with no drag and no pressure dependence."""
    return LaunchVehicle(
        name='test article',
        stages=[Stage(name='only stage', ignition_time=0, dry_mass=dry,
                      propellant_mass=propellant, thrust_vacuum=thrust,
                      thrust_sea_level=thrust, isp_vacuum=isp, diameter=1.0)],
        drag_coefficient={0.0: 0.0, 10.0: 0.0},
    )


def test_horizontal_vacuum_flight_matches_tsiolkovsky():
    """Horizontal, drag-free flight is the rocket equation exactly.

    Gravity is perpendicular to the velocity and the radius cannot change, so
    nothing but the thrust acts along the motion and the closed form applies.
    """
    dry, propellant, isp = 1_000.0, 9_000.0, 300.0
    vehicle = vacuum_vehicle(dry=dry, propellant=propellant, isp=isp)
    mission = Mission(vehicle, Horizontal(300.0), CutoffAtTime(400.0),
                      target_altitude=0.0, duration=300.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)  # due north: no rotation
    telemetry = mission.run()

    expected = isp * STANDARD_GRAVITY * math.log((dry + propellant) / dry)
    assert abs(mission.final_state.speed - expected) < 0.01
    # the radius is untouched, so the altitude never leaves zero
    assert np.max(np.abs(telemetry.altitude)) < 1e-6


def test_tank_running_dry_is_located_exactly():
    """The burn ends when the propellant does, not at the end of a step."""
    vehicle = vacuum_vehicle()
    mission = Mission(vehicle, Horizontal(300.0), CutoffAtTime(400.0),
                      target_altitude=0.0, duration=300.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)
    telemetry = mission.run()

    stage = vehicle.stages[0]
    exhausted = stage.propellant_mass / stage.mass_flow(0.0, 1.0)
    burning = telemetry.t[telemetry.thrust > 0.0]
    assert abs(burning[-1] - exhausted) <= 1 / mission.steps_per_second


def test_halving_the_step_barely_moves_the_answer():
    """How much a result moves under refinement is its numerical error."""
    coarse = load_mission('config/mission.f9.yaml')
    coarse.steps_per_second = 10
    coarse.run()

    fine = load_mission('config/mission.f9.yaml')
    fine.steps_per_second = 20
    fine.run()

    assert abs(coarse.final_state.inertial_speed - fine.final_state.inertial_speed) < 0.05
    assert abs(coarse.orbit.apogee_altitude - fine.orbit.apogee_altitude) < 100.0


def test_launching_east_gains_the_rotation_of_the_earth():
    east = load_mission('config/mission.f9.yaml')
    east.azimuth_deg = 90.0
    east.run()

    north = load_mission('config/mission.f9.yaml')
    north.azimuth_deg = 0.0
    north.run()

    # due north the two velocities coincide; due east the pad is already moving
    assert abs(north.final_state.inertial_speed - north.final_state.speed) < 1e-9
    assert east.final_state.inertial_speed > north.final_state.inertial_speed


def test_configuration_files_load():
    for name in ('f9', 'a62', 'h3'):
        mission = load_mission(f'config/mission.{name}.yaml')
        assert mission.vehicle.stages
        assert mission.pitch_programme.end_time > 0
        assert mission.target_altitude > 0
