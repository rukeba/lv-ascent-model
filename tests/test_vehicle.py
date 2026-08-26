"""The vehicle: what it weighs, which stage is burning, and what the air costs.

Three of these answers are worked out once when the vehicle is built and read
back millions of times, so each is checked against the plain form it replaced
rather than against a number written down here.
"""

import numpy as np
import pytest

from ascent.atmosphere import air_at
from ascent.config import load_vehicle
from ascent.vehicle import LaunchVehicle, Stage, _interpolated

CATALOGUE = ('lv.f9', 'lv.a62', 'lv.h3')


def vehicles(directory='config'):
    from pathlib import Path
    return [load_vehicle(Path(directory) / f'{name}.yaml') for name in CATALOGUE]


def test_drag_coefficient_is_what_numpy_interpolates():
    """The table is read here rather than by `np.interp`, and has to agree.

    Over the whole Mach range, on the tabulated points themselves, either side
    of them, and off both ends - and on a Mach that is not a number, which a
    diverging trajectory can produce and which numpy carries through.
    """
    for vehicle in vehicles():
        points = np.array(vehicle._mach_points)
        values = np.array(vehicle._cd_values)
        trials = [float(m) for m in points]
        trials += [m + d for m in trials for d in (-1e-12, 1e-12, -0.05, 0.05)]
        trials += list(np.linspace(-1.0, 8.0, 4000))
        trials += [float('inf'), float('-inf')]
        for mach in trials:
            mine = _interpolated(mach, vehicle._mach_points,
                                 vehicle._cd_values, vehicle._cd_slopes)
            assert mine == float(np.interp(mach, points, values)), mach
        assert np.isnan(_interpolated(float('nan'), vehicle._mach_points,
                                      vehicle._cd_values, vehicle._cd_slopes))


def test_a_drag_table_of_one_point_is_that_number_everywhere():
    """A table of one point has no interval to interpolate over, and numpy
    answers it with that one number for every Mach - a Mach that is not a
    number included, which is the one place a NaN does not carry through. That
    reads like an oversight and is not one: it is what `np.interp` does, and
    the assertion below is against numpy rather than against a rule.
    """
    table = ((1.5,), (0.42,), ())
    for mach in (0.0, 9.0, float('nan'), float('inf'), float('-inf'), 1.5):
        assert _interpolated(mach, *table) \
            == float(np.interp(mach, np.array([1.5]), np.array([0.42])))


def test_the_active_stage_is_the_last_one_lit():
    for vehicle in vehicles():
        for t in np.linspace(-10.0, 1500.0, 5000):
            expected = 0
            for i, stage in enumerate(vehicle.stages):
                if t >= stage.ignition_time:
                    expected = i
            assert vehicle.active_stage(float(t))[0] == expected


def test_two_stages_lighting_together_hand_over_to_the_upper_one():
    stages = [Stage(name=f'stage {i}', ignition_time=time, dry_mass=100.0,
                    propellant_mass=1000.0, thrust_vacuum=1e5,
                    thrust_sea_level=1e5, isp_vacuum=300.0, diameter=1.0)
              for i, time in enumerate((0.0, 100.0, 100.0))]
    vehicle = LaunchVehicle(name='shared instant', stages=stages)

    assert vehicle.active_stage(99.9)[0] == 0
    assert vehicle.active_stage(100.0)[0] == 2
    assert vehicle.active_stage(100.1)[0] == 2


def test_the_stack_mass_is_the_stages_above_it():
    for vehicle in vehicles():
        for index, stage in enumerate(vehicle.stages):
            full = sum(s.dry_mass + s.propellant_mass
                       for s in vehicle.stages[index:])
            assert vehicle.mass_on(index, 0.0) == full
            assert vehicle.mass_on(index, stage.propellant_mass) \
                == full - stage.propellant_mass
            # burning past the tank takes nothing further off the vehicle
            assert vehicle.mass_on(index, 2 * stage.propellant_mass + 1.0) \
                == full - stage.propellant_mass


def test_thrust_and_flow_together_are_the_two_taken_apart():
    """Asked of the stages that burn: the payload has no impulse to divide by,
    and the equations of motion never light it - a stage with an empty tank is
    flown at no power at all."""
    for vehicle in vehicles():
        for stage in vehicle.stages:
            if stage.propellant_mass <= 0.0 or stage.isp_vacuum <= 0.0:
                continue
            for altitude in (0.0, 5_000.0, 40_000.0, 200_000.0):
                pressure = air_at(altitude).pressure
                for throttle in (1.0, 0.7, 0.25):
                    thrust, flow = stage.propulsion(pressure, throttle)
                    assert thrust == stage.thrust(pressure) * throttle
                    assert flow == stage.mass_flow(pressure, throttle)
