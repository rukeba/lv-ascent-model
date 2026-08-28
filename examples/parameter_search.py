"""Search for a pitch programme, and print it beside the set already on file.

Falcon 9 into a 500 km circular orbit from Cape Canaveral, once per programme
family. Nothing is taken from the catalogue but the row to compare against: the
search is given the vehicle, the orbit and the family, and works from there.

The two columns should agree, and they do - because the catalogue is what this
search returned. Running it again is a check that the file can be reproduced
from the vehicle and the orbit alone, rather than a comparison of two methods.

What they are worth watching for is the day they stop agreeing: a change
anywhere under the search moves the answer, and this is where that shows.

    uv run python examples/parameter_search.py

About a quarter of an hour: some twenty-four thousand trajectories, divided
over two thirds of the cores. Pass a coarser integration step to see it sooner
- the orbit a set reaches is the same to within a few metres either way, though
a coarse step will not reproduce the file exactly.

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
    catalogue = load_catalogue('config')

    print(f'{"programme":<18}{"source":>10}{"t1":>7}{"cut-off":>10}'
          f'{"perigee":>9}{"apogee":>9}{"steering":>10}{"total":>9}')
    for programme in PROGRAMMES:
        on_file = find_in_catalogue(catalogue, 'lv.f9', TARGET_ALTITUDE, programme)
        reached = on_file['reached']
        _row(programme, 'catalogue', on_file['pitch_programme']['t1'],
             on_file['cutoff']['time'], reached['perigee_km'],
             reached['apogee_km'], reached['steering_loss'],
             reached['total_loss'])

        result = search(vehicle, TARGET_ALTITUDE, programme,
                        latitude_deg=LATITUDE, azimuth_deg=AZIMUTH,
                        steps_per_second=steps_per_second,
                        report=_progress)
        found = result.best
        _row('', 'search', found.values['t1'], found.cutoff_time,
             found.orbit.perigee_altitude / 1000,
             found.orbit.apogee_altitude / 1000,
             found.steering_loss, found.total_loss)
        if not result.reaches_orbit:
            print(f'{"":<28}closest of {len(result.found):,} sets found, '
                  f'{found.miss:.0f} m out')

    print('\nlosses in m/s; every parameter of each family was searched, and '
          'the catalogue is what that search returned')


def _row(programme, source, rise, cut_off, perigee, apogee, steering, total) -> None:
    print(f'{programme:<18}{source:>10}{rise:>6.2f}s{cut_off:>9.3f}s'
          f'{perigee:>9.2f}{apogee:>9.2f}{steering:>10.1f}{total:>9.1f}')


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
