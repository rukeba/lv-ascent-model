"""The recorded flight: one row per integration step, and its CSV form.

Columns are declared once, here, so the CSV file, the console summary and the
report all describe the same run in the same terms.
"""

import csv
import math

import numpy as np

from .state import FlightState

DEGREES = 180.0 / math.pi

# name, unit, and how to read it off a state `s`, as an expression
COLUMNS = (
    ('t', 's', 's.t'),
    ('altitude', 'm', 's.altitude'),
    ('radius', 'm', 's.radius'),
    ('polar_angle', 'deg', 's.polar_angle * DEGREES'),
    ('downrange_x', 'm', 's.downrange_x'),
    ('downrange_y', 'm', 's.downrange_y'),
    ('speed', 'm/s', 's.speed'),
    ('inertial_speed', 'm/s', 's.inertial_speed'),
    ('horizontal_speed', 'm/s', 's.horizontal_speed'),
    ('vertical_speed', 'm/s', 's.vertical_speed'),
    ('flight_path_angle', 'deg', 's.flight_path_angle * DEGREES'),
    ('flight_path_rate', 'deg/s', 's.flight_path_rate * DEGREES'),
    ('mass', 'kg', 's.mass'),
    ('thrust', 'N', 's.thrust'),
    ('drag', 'N', 's.drag'),
    ('dynamic_pressure', 'Pa', 's.dynamic_pressure'),
    ('steering_angle', 'deg', 's.steering_angle * DEGREES'),
    ('steering_demand', '', 's.steering_demand'),
    ('steering_loss', 'm/s', 's.steering_loss'),
    ('control_effort', 'm^2/s^3', 's.control_effort'),
    ('stage', '', 's.stage'),
)

# The whole row in one expression, built from the table above once at import.
# A flight is tens of thousands of rows of twenty-one columns and every one of
# them is recorded, so a separate call per column costs several times what
# reading the state does. Expressions rather than functions is what lets the
# row be one of them.
_row_of = eval('lambda s: (' + ', '.join(read for _, _, read in COLUMNS) + ',)',
               {'DEGREES': DEGREES})


class Telemetry:
    """Columns are reachable as attributes: `telemetry.altitude` is an array."""

    def __init__(self) -> None:
        self._rows: list[tuple[float, ...]] = []
        self._arrays: dict[str, np.ndarray] | None = None

    def record(self, state: FlightState) -> None:
        self._rows.append(_row_of(state))
        self._arrays = None

    def __len__(self) -> int:
        return len(self._rows)

    def __getattr__(self, name: str) -> np.ndarray:
        arrays = self.__dict__.get('_arrays')
        if arrays is None:
            arrays = self.__dict__['_arrays'] = {
                key: np.array([row[i] for row in self._rows])
                for i, (key, _, _) in enumerate(COLUMNS)}
        if name not in arrays:
            raise AttributeError(name)
        return arrays[name]

    def at(self, t: float) -> int:
        """Index of the last row recorded at or before the given instant."""
        index = int(np.searchsorted(self.t, t + 1e-9, side='right') - 1)
        if index < 0:
            # an index of -1 is a valid row and the wrong one: the last of the
            # flight, handed back for an instant before its first
            raise ValueError(
                f'nothing was recorded at or before t = {t:g} s'
                + (f'; the flight starts at {self.t[0]:g} s' if len(self) else ''))
        return index

    def write_csv(self, path: str) -> None:
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(f'{name}[{unit}]' if unit else name
                            for name, unit, _ in COLUMNS)
            for row in self._rows:
                writer.writerow(f'{value:.6g}' for value in row)
