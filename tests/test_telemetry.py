"""What each column of a recorded flight holds.

A row is one generated expression built from `COLUMNS` at import, so nothing
but this says that a column called `vertical_speed` holds the vertical speed.
"""

import math

import pytest

from ascent.constants import EARTH_RADIUS
from ascent.state import FlightState
from ascent.telemetry import COLUMNS, Telemetry

ANGLE = math.pi / 6


def sample_state():
    return FlightState(
        t=12.5, radius=EARTH_RADIUS + 40_000.0, polar_angle=0.02, speed=900.0,
        inertial_speed=1305.0, flight_path_angle=ANGLE, flight_path_rate=-0.003,
        mass=310_000.0, thrust=7.4e6, drag=12_500.0, dynamic_pressure=31_000.0,
        steering_angle=0.05, steering_demand=0.049, steering_loss=18.0,
        control_effort=440.0, stage=0)


def test_every_column_holds_what_its_name_says():
    telemetry = Telemetry()
    state = sample_state()
    telemetry.record(state)

    degrees = 180.0 / math.pi
    expected = {
        't': 12.5,
        'altitude': 40_000.0,
        'radius': EARTH_RADIUS + 40_000.0,
        'polar_angle': 0.02 * degrees,
        'downrange_x': (EARTH_RADIUS + 40_000.0) * math.sin(0.02),
        'downrange_y': (EARTH_RADIUS + 40_000.0) * math.cos(0.02) - EARTH_RADIUS,
        'speed': 900.0,
        'inertial_speed': 1305.0,
        'horizontal_speed': 900.0 * math.cos(ANGLE),
        'vertical_speed': 900.0 * math.sin(ANGLE),
        'flight_path_angle': 30.0,
        'flight_path_rate': -0.003 * degrees,
        'mass': 310_000.0,
        'thrust': 7.4e6,
        'drag': 12_500.0,
        'dynamic_pressure': 31_000.0,
        'steering_angle': 0.05 * degrees,
        'steering_demand': 0.049,
        'steering_loss': 18.0,
        'control_effort': 440.0,
        'stage': 0,
    }
    # every column is accounted for, so a new one cannot slip past unchecked
    assert [name for name, _, _ in COLUMNS] == list(expected)
    for name, value in expected.items():
        assert getattr(telemetry, name)[0] == pytest.approx(value), name


def test_a_column_it_does_not_have_is_not_an_array_of_nothing():
    telemetry = Telemetry()
    telemetry.record(sample_state())
    with pytest.raises(AttributeError):
        telemetry.apogee
