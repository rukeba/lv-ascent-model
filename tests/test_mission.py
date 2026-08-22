"""The integration itself: against a closed form, and against a finer step."""

import math

import numpy as np
import pytest

from ascent import (CutoffAtInertialSpeed, CutoffAtTime, LaunchVehicle, Mission,
                    Stage, load_mission, velocity_budget)
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


def test_tank_running_dry_is_located_inside_the_step():
    """The burn ends when the propellant does, not at the end of a step.

    The tank is sized to empty nine tenths of the way through a step - inside
    it, and in the part of it where a trial point of the scheme overshoots the
    capacity while the weighted result of the whole step does not. A crossing
    test read off that result alone misses this and carries the burn into the
    next step, which puts the last powered instant a step late and the event
    itself some fifty milliseconds late.

    Both halves are checked: the instant, to within the step it belongs to, and
    the total, which the rocket equation fixes to a hundredth of a metre per
    second - a thousandth of a step of the acceleration at burnout.
    """
    step, dry, isp = 0.1, 1_000.0, 300.0
    flow = vacuum_vehicle(dry=dry, isp=isp).stages[0].mass_flow(0.0, 1.0)
    exhausted = 90.0 + 0.9 * step
    propellant = flow * exhausted

    vehicle = vacuum_vehicle(dry=dry, propellant=propellant, isp=isp)
    mission = Mission(vehicle, Horizontal(300.0), CutoffAtTime(400.0),
                      target_altitude=0.0, duration=300.0,
                      steps_per_second=1 / step,
                      latitude_deg=0.0, azimuth_deg=0.0)
    telemetry = mission.run()

    # the last row still under thrust is the one before the tank runs dry, and
    # the row after it is already empty
    burning = telemetry.t[telemetry.thrust > 0.0]
    assert burning[-1] == pytest.approx(exhausted - 0.9 * step, abs=1e-9)

    expected = isp * STANDARD_GRAVITY * math.log((dry + propellant) / dry)
    assert abs(mission.final_state.speed - expected) < 0.01


def test_a_stage_still_burning_at_separation_keeps_its_own_mass():
    """The mass over a step belongs to the stage that flies it.

    The step is cut at the separation, so the last point of the piece below it
    falls exactly on the ignition above. Read the mass by that instant and the
    final evaluation of the scheme weighs the vehicle without the stage that is
    still pushing it - here a third of the stack - which is a step change in
    mass inside a step that was cut precisely to keep step changes out.

    Two burns in a vacuum, the first cut short by the separation rather than by
    an empty tank, so the closed form is the rocket equation applied twice.
    """
    separation, isp = 60.0, 300.0
    lower = Stage(name='lower', ignition_time=0, dry_mass=2_000.0,
                  propellant_mass=30_000.0, thrust_vacuum=400_000.0,
                  thrust_sea_level=400_000.0, isp_vacuum=isp, diameter=1.0)
    upper = Stage(name='upper', ignition_time=separation, dry_mass=1_000.0,
                  propellant_mass=6_000.0, thrust_vacuum=100_000.0,
                  thrust_sea_level=100_000.0, isp_vacuum=isp, diameter=1.0)
    vehicle = LaunchVehicle(name='two stages', stages=[lower, upper],
                            drag_coefficient={0.0: 0.0, 10.0: 0.0})
    mission = Mission(vehicle, Horizontal(300.0), CutoffAtTime(400.0),
                      target_altitude=0.0, duration=300.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)
    telemetry = mission.run()

    # the lower stage is dropped with propellant still in it, and the upper one
    # empties its own tank well inside the flight, so the closed form applies
    spent = lower.mass_flow(0.0, 1.0) * separation
    assert spent < lower.propellant_mass
    assert separation + upper.propellant_mass / upper.mass_flow(0.0, 1.0) < 300.0

    stack = vehicle.lift_off_mass
    above = upper.dry_mass + upper.propellant_mass
    expected = isp * STANDARD_GRAVITY * (
        math.log(stack / (stack - spent)) + math.log(above / upper.dry_mass))
    assert abs(mission.final_state.speed - expected) < 0.01
    # and no mass appears or disappears while the lower stage is still flying
    before = telemetry.mass[telemetry.t < separation]
    assert before[-1] == pytest.approx(stack - spent + lower.mass_flow(0.0, 1.0) * 0.1)


def test_a_speed_cut_off_fires_once_and_stays_fired():
    """Cut-off is an event, not a condition re-read every step.

    The inertial speed is the quantity that fixes the orbit, but it falls again
    as soon as the vehicle coasts uphill. A threshold that is compared afresh
    each step and never remembered hands the throttle back there and relights a
    stage that still has propellant in it.
    """
    threshold = 1_000.0
    vehicle = vacuum_vehicle(propellant=9_000.0, thrust=300_000.0)
    mission = Mission(vehicle, Vertical(200.0), CutoffAtInertialSpeed(threshold),
                      target_altitude=0.0, duration=120.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)
    telemetry = mission.run()

    # the flight has to fall back through the threshold, or the test would pass
    # on a vehicle that never gets the chance to relight, and it has to have
    # propellant left to relight on
    assert telemetry.inertial_speed[-1] < threshold - 100.0
    assert telemetry.mass[-1] > vehicle.stages[0].dry_mass

    powered = np.flatnonzero(telemetry.thrust > 0.0)
    assert len(powered)
    assert np.array_equal(powered, np.arange(powered[0], powered[-1] + 1))


def test_a_speed_cut_off_is_placed_at_the_crossing():
    """A watched threshold is an event too, and cannot be put on the bounds.

    Where the speed crosses is not known until the step has been taken, so the
    step cannot be cut there in advance the way it is for a scheduled cut-off.
    Solved for instead: the burn ends at the crossing rather than at the next
    step of the grid, which under 20 m/s^2 is worth some 2 m/s of overshoot.
    """
    threshold = 1_000.0
    vehicle = vacuum_vehicle(propellant=9_000.0, thrust=300_000.0)
    mission = Mission(vehicle, Vertical(200.0), CutoffAtInertialSpeed(threshold),
                      target_altitude=0.0, duration=120.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)
    telemetry = mission.run()

    # never past the threshold - the burn stops exactly on it
    assert telemetry.inertial_speed.max() <= threshold + 1e-6
    # and not short of it by more than the gravity of the part of a step left
    # over, which is what says the crossing was solved for and not rounded down
    assert telemetry.inertial_speed.max() > threshold - 1.0


def test_a_flight_that_never_lights_an_engine_spends_nothing():
    """A coast is not a loss.

    The budget accounts for the velocity the propellant paid for, and a flight
    that burns none of it has nothing to account for. Integrating the whole
    flight instead would report the gravity and the drag of the coast as
    losses, and its last instant as a burnout that never happened.
    """
    mission = Mission(vacuum_vehicle(), Horizontal(300.0), CutoffAtTime(0.0),
                      target_altitude=0.0, duration=300.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)
    telemetry = mission.run()
    budget = velocity_budget(telemetry, mission.omega)

    assert not telemetry.thrust.any()
    assert (budget.gravity, budget.aerodynamic, budget.steering) == (0.0, 0.0, 0.0)
    assert budget.burnout_time == 0.0


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


def test_the_velocity_budget_adds_back_up():
    """What the propellant delivered is what the flight spent.

    The along-track equation loses `(g - omega^2 r) sin gamma` to gravity and
    `D/m` to the air, and nothing else - the guided phase puts the whole of the
    thrust along the velocity, and the steering loss is a price recovered
    afterwards rather than something the trajectory pays. So the ideal velocity
    delivered over the powered flight, less those two, is the speed reached.

    It only closes if both loss integrands are projected in the frame the
    equations are written in. Taking the centripetal term from the inertial
    speed instead leaves Falcon 9 out by some 240 m/s and H3 by 820.
    """
    for name in ('f9', 'a62', 'h3'):
        mission = load_mission(f'config/mission.{name}.yaml')
        telemetry = mission.run()
        budget = velocity_budget(telemetry, mission.omega)

        end = int(np.flatnonzero(telemetry.thrust > 0.0)[-1]) + 1
        delivered = float(np.trapezoid(telemetry.thrust[:end] / telemetry.mass[:end],
                                       telemetry.t[:end]))
        spent = budget.gravity + budget.aerodynamic + telemetry.speed[end - 1]
        assert abs(delivered - spent) < 5.0, name


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


class Vertical(PitchProgramme):
    """Points the velocity straight up for the whole flight."""

    def __init__(self, end_time: float) -> None:
        t = self._grid(end_time)
        angle = np.full_like(t, math.pi / 2)
        zero = np.zeros_like(t)
        self._tabulate(t, angle, zero, zero)

    def describe(self) -> str:
        return 'vertical'


class StraightDown(PitchProgramme):
    """Points the velocity at the ground for the whole flight."""

    def __init__(self, end_time: float) -> None:
        t = self._grid(end_time)
        angle = np.full_like(t, -math.pi / 2)
        zero = np.zeros_like(t)
        self._tabulate(t, angle, zero, zero)

    def describe(self) -> str:
        return 'straight down'


def test_a_vehicle_that_cannot_hold_its_programme_raises():
    """A guided phase that runs out of speed has to fail loudly.

    What is integrated while the programme runs is the magnitude of the
    velocity, and a magnitude has no sign to turn round. A vehicle too heavy to
    climb the programme it is given drives it negative; reported as a zero it
    would read as a vehicle at rest while its radius went on falling, and the
    orbit at the end would be built out of that.
    """
    vehicle = vacuum_vehicle(thrust=50_000.0)     # half of its own weight
    mission = Mission(vehicle, Vertical(200.0), CutoffAtTime(400.0),
                      target_altitude=0.0, duration=200.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)

    with pytest.raises(ValueError, match='run out of speed'):
        mission.run()


def test_a_trajectory_that_leaves_the_model_raises():
    """Flying into the ground has to fail loudly rather than return numbers.

    Once the radius passes through zero everything downstream is meaningless,
    and a comparison against a NaN is false, so the run would otherwise carry
    on quietly.
    """
    vehicle = vacuum_vehicle(dry=1_000.0, propellant=200_000.0, thrust=5_000_000.0)
    mission = Mission(vehicle, StraightDown(400.0), CutoffAtTime(400.0),
                      target_altitude=0.0, duration=400.0, steps_per_second=10,
                      latitude_deg=0.0, azimuth_deg=0.0)

    with pytest.raises(ValueError, match='left the model'):
        mission.run()
