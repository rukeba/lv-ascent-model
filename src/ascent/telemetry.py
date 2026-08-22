"""The recorded flight: one row per integration step, and its CSV form.

Columns are declared once, here, so the CSV file, the console summary and the
report all describe the same run in the same terms.
"""

import csv
import math

import numpy as np

from .state import FlightState

DEGREES = 180.0 / math.pi

# name, unit, how to read it off a state
COLUMNS = (
    ('t', 's', lambda s: s.t),
    ('altitude', 'm', lambda s: s.altitude),
    ('radius', 'm', lambda s: s.radius),
    ('polar_angle', 'deg', lambda s: s.polar_angle * DEGREES),
    ('downrange_x', 'm', lambda s: s.downrange_x),
    ('downrange_y', 'm', lambda s: s.downrange_y),
    ('speed', 'm/s', lambda s: s.speed),
    ('inertial_speed', 'm/s', lambda s: s.inertial_speed),
    ('horizontal_speed', 'm/s', lambda s: s.horizontal_speed),
    ('vertical_speed', 'm/s', lambda s: s.vertical_speed),
    ('flight_path_angle', 'deg', lambda s: s.flight_path_angle * DEGREES),
    ('flight_path_rate', 'deg/s', lambda s: s.flight_path_rate * DEGREES),
    ('mass', 'kg', lambda s: s.mass),
    ('thrust', 'N', lambda s: s.thrust),
    ('drag', 'N', lambda s: s.drag),
    ('dynamic_pressure', 'Pa', lambda s: s.dynamic_pressure),
    ('steering_angle', 'deg', lambda s: s.steering_angle * DEGREES),
    ('steering_demand', '', lambda s: s.steering_demand),
    ('steering_loss', 'm/s', lambda s: s.steering_loss),
    ('stage', '', lambda s: s.stage),
)


class Telemetry:
    """Columns are reachable as attributes: `telemetry.altitude` is an array."""

    def __init__(self) -> None:
        self._rows: list[tuple[float, ...]] = []
        self._arrays: dict[str, np.ndarray] | None = None

    def record(self, state: FlightState) -> None:
        self._rows.append(tuple(read(state) for _, _, read in COLUMNS))
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
