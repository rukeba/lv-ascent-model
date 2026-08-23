"""Search for a pitch programme, and print it beside the set already on file.

Falcon 9 into a 500 km circular orbit from Cape Canaveral, once per programme
family. Nothing is taken from the catalogue but the row to compare against: the
search is given the vehicle, the orbit and the family, and works from there.

The five-phase family has no freedom left once the orbit is fixed - two
terminal conditions and two unknowns - so it returns the set on file. The other
two keep a parameter, and the catalogue spent it on the smallest steering loss
where this spends it on the earliest cut-off, so those two rows differ, and the
search rows are the quicker ascents.

    uv run python examples/parameter_search.py

Several minutes: it integrates a few thousand trajectories. Pass a coarser
integration step to see it sooner - the orbit a set reaches is the same to
within a few metres either way.

    uv run python examples/parameter_search.py 2
"""

import sys

from ascent import search
from ascent.config import find_in_catalogue, load_catalogue, load_vehicle

TARGET_ALTITUDE = 500_000
LATITUDE, AZIMUTH = 28.5, 90.0
PROGRAMMES = ('five-phase', 'velocity-share', 'bilinear-tangent')


def main(steps_per_second: float = 10) -> None:
    vehicle = load_vehicle('config/lv.f9.yaml')
    catalogue = load_catalogue('config/catalogue.yaml')

    print(f'{"programme":<18}{"source":>10}{"cut-off":>10}{"perigee":>9}'
          f'{"apogee":>9}{"steering":>10}{"total":>9}')
    for programme in PROGRAMMES:
        on_file = find_in_catalogue(catalogue, 'lv.f9', TARGET_ALTITUDE, programme)
        reached = on_file['reached']
        _row(programme, 'catalogue', on_file['cutoff']['time'],
             reached['perigee_km'], reached['apogee_km'],
             reached['steering_loss'], reached['total_loss'])

        result = search(vehicle, TARGET_ALTITUDE, programme,
                        latitude_deg=LATITUDE, azimuth_deg=AZIMUTH,
                        steps_per_second=steps_per_second,
                        report=_progress)
        found = result.best
        _row('', 'search', found.cutoff_time,
             found.orbit.perigee_altitude / 1000, found.orbit.apogee_altitude / 1000,
             found.steering_loss, found.total_loss)

    print('\nlosses in m/s; the search minimises the cut-off, the catalogue '
          'minimised the steering loss')


def _row(programme, source, cut_off, perigee, apogee, steering, total) -> None:
    print(f'{programme:<18}{source:>10}{cut_off:>9.3f}s{perigee:>9.2f}'
          f'{apogee:>9.2f}{steering:>10.1f}{total:>9.1f}')


def _progress(result) -> None:
    """A dot a pass, wiped once the row those dots were waiting for is ready."""
    if result.pass_node != result.pass_nodes:
        return
    if result.pass_number < result.passes:
        print('.', end='', flush=True)
    else:
        print('\r' + ' ' * result.passes + '\r', end='', flush=True)


if __name__ == '__main__':
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 10)
