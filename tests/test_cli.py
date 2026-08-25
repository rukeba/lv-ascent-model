"""The command line: how a mission is found, and what it is flown with."""

import csv
from pathlib import Path

import pytest
import yaml

from ascent.cli import main, quietly, search_main
from ascent.config import PROGRAMME_ALIASES, programme_name
from ascent.search import FAMILIES


def test_mission_named_by_short_name_flies():
    assert main(['f9']) == 0


def test_mission_given_by_path_brings_its_own_vehicle(tmp_path):
    """A mission file names its vehicle as a neighbour.

    Given a path, the vehicle beside it is the one meant - not whatever file
    of that name happens to sit in the configuration directory.
    """
    (tmp_path / 'lv.demo.yaml').write_text(Path('config/lv.f9.yaml').read_text())
    mission = tmp_path / 'mission.demo.yaml'
    mission.write_text(Path('config/mission.f9.yaml').read_text()
                       .replace('vehicle: lv.f9', 'vehicle: lv.demo'))

    assert main([str(mission)]) == 0


def test_catalogue_entry_is_flown_when_asked_for():
    assert main(['f9', '--altitude', '650', '--programme', 'bilinear-tangent']) == 0


def test_a_catalogue_entry_is_flown_from_the_pad_the_mission_file_names(capsys):
    """The catalogue holds a site as two numbers; the file gives it a name."""
    assert main(['f9', '--altitude', '650']) == 0
    assert 'Cape Canaveral SLC-40, Florida' in capsys.readouterr().out


def test_a_catalogue_entry_can_be_asked_for_in_short():
    """`-a` is the altitude and `-p` the programme, by its short name."""
    assert main(['f9', '-a', '650', '-p', 'bt']) == 0


def test_every_short_name_stands_for_a_family_that_can_be_searched():
    for short, full in PROGRAMME_ALIASES.items():
        assert programme_name(short) == full
        assert full in FAMILIES


def test_a_name_that_is_not_short_for_anything_is_left_alone():
    """Whatever is going to look the name up reports it, not the alias table."""
    assert programme_name('five-phase') == 'five-phase'
    assert programme_name('parabolic') == 'parabolic'


def test_altitude_absent_from_the_catalogue_is_reported():
    with pytest.raises(LookupError, match='altitudes available'):
        main(['f9', '--altitude', '999'])


def test_listing_the_catalogue(capsys):
    assert main(['f9', '--list']) == 0
    assert 'bilinear-tangent' in capsys.readouterr().out


# A search narrow enough for a test: the five-phase family with the vertical
# rise and k2 held where the catalogue holds them, k3 and the end of the turn
# swept over four nodes each, at one integration step a second and in this
# process. What the tests below are of is the plumbing of the command - the
# units the arguments are read in, the vehicle found beside the mission file,
# the entry written out and the exit status - and not how well the grid was
# swept, which is `tests/test_search.py`.
NARROW = ['--range', 't1=20', '--range', 'k2=0.03:0.07:3',
          '--range', 'k3=0.50:0.56:3', '--range', 't4=502:503:3',
          '--refinements', '4', '--steps', '1', '--workers', '1',
          '--tolerance', '2']


def test_the_search_writes_out_the_set_it_found(capsys):
    """`ascent-search --yaml` writes an entry the catalogue reader can read.

    Nothing but the entry goes to the output, so that a redirect of it is a
    file rather than a file with a summary printed across the top.
    """
    assert search_main(['f9', '--altitude', '500', '--programme', 'five-phase',
                        *NARROW, '--yaml']) == 0

    written = capsys.readouterr()
    assert 'Falcon 9 to 500 km' in written.err
    entry = yaml.safe_load(written.out)['missions'][0]
    assert entry['vehicle'] == 'lv.f9'
    assert entry['target_altitude'] == 500_000
    assert entry['launch_site'] == {'latitude': 28.5, 'azimuth': 90}
    assert entry['pitch_programme']['type'] == 'five-phase'
    assert entry['pitch_programme']['t1'] == 20.0
    # an instant a vehicle could be given, and not a hundredth past it
    assert entry['cutoff']['time'] == pytest.approx(502.7, abs=0.5)
    assert entry['cutoff']['time'] == round(entry['cutoff']['time'], 1)


def test_a_search_that_reaches_nothing_writes_nothing_out(capsys):
    """A set that misses the orbit is shown, and is not filed as an entry."""
    assert search_main(['f9', '--altitude', '500', '--programme', 'five-phase',
                        '--range', 't1=20', '--range', 'k2=0.03:0.07:3',
                        '--range', 'k3=0.50:0.56:3',
                        '--range', 't4=502:503:3',
                        '--tolerance', '0.001', '--refinements', '0',
                        '--steps', '1', '--workers', '1', '--yaml']) == 1
    written = capsys.readouterr()
    assert 'reaches the orbit       no' in written.err
    assert written.out.strip() == ''


def test_the_search_summary_is_the_output_without_yaml(capsys):
    """Without `--yaml` there is nothing to redirect, so it reads normally."""
    assert search_main(['f9', '--altitude', '500', '--programme', 'five-phase',
                        *NARROW]) == 0
    written = capsys.readouterr()
    assert 'Falcon 9 to 500 km' in written.out
    assert 'missions:' not in written.out


def test_the_search_prints_the_sets_it_found_as_a_table(capsys):
    """The best of them, with the errors each is judged by, and the axes swept.

    A held parameter has no column - it is the same in every row - and is
    printed once above with the range it was held at, so the table says what
    was searched and the grid above it says what was not.
    """
    assert search_main(['f9', '--altitude', '500', '--programme', '5f',
                        *NARROW, '--top', '4']) == 0

    written = capsys.readouterr().out
    assert 'TOP 4 OF THE' in written
    assert 'orbit err' in written
    # the three axes swept have columns; the ones held do not
    header = next(line for line in written.splitlines() if 'orbit err' in line)
    assert 'k2' in header and 'k3' in header and 't4' in header
    assert 't1' not in header
    assert '  t1                      20, held' in written


def test_the_grid_is_printed_before_it_is_walked(capsys):
    """What is about to be searched, while there is still time to stop it.

    A search is minutes of integration, so the grid comes first and the results
    come after it - and `nodes planned` belongs to the first half only, because
    the second half has `nodes visited` and does not want the two side by side.
    """
    assert search_main(['f9', '--altitude', '500', '--programme', '5f',
                        *NARROW, '--top', '3']) == 0

    written = capsys.readouterr().out
    assert written.index('GRID') < written.index('COST')
    assert written.index('nodes planned') < written.index('nodes visited')
    assert written.count('nodes planned') == 1


def test_an_interrupted_command_ends_without_a_stack(capsys):
    """Ctrl+C is an ordinary way to stop a search, and not an error.

    What the interpreter would print instead is a stack from the middle of a
    process pool, once for the process that was asked and once for every worker
    it had. The newline is what closes off the progress line the search was
    writing in place.
    """
    def interrupted(argv):
        raise KeyboardInterrupt

    assert quietly(interrupted, []) == 130
    written = capsys.readouterr()
    assert written.err == '\nstopped\n'
    assert 'Traceback' not in written.err and 'Traceback' not in written.out


def test_the_grid_can_be_looked_at_before_it_is_walked(capsys):
    """`--dry-run` prints every axis and what the passes come to, and stops."""
    assert search_main(['f9', '--altitude', '500', '--programme', '5f',
                        '--range', 't1=10:25:10', '--dry-run']) == 0

    written = capsys.readouterr().out
    assert '10 to 25, 10 values, step 1.66667' in written
    assert 'nodes planned' in written
    # nothing was flown, so there is nothing found to report
    assert 'TOP' not in written and 'FOUND' not in written


def test_a_parameter_the_family_does_not_have_is_refused_at_the_command_line():
    """Answered before the search starts, with the parameters it does have."""
    with pytest.raises(SystemExit):
        search_main(['f9', '--altitude', '500', '--programme', '5f',
                     '--range', 'kick=1:2:0.5', '--dry-run'])


@pytest.mark.parametrize('argument', ['--range=t1', '--range=t1:10:30:2',
                                      '--range=t1=10:30'])
def test_a_range_that_is_not_one_is_refused_at_the_command_line(argument):
    with pytest.raises(SystemExit):
        search_main(['f9', '--altitude', '500', '--programme', '5f',
                     argument, '--dry-run'])


def test_the_search_is_refused_an_orbit_out_of_reach():
    with pytest.raises(ValueError, match='does not reach a circular orbit'):
        search_main(['f9', '--altitude', '20000'])


@pytest.mark.parametrize('argument', ['--steps=0', '--tolerance=0',
                                      '--speed-tolerance=0', '--top=0',
                                      '--coarse=-1', '--refinements=-1',
                                      '--max-q=0'])
def test_a_setting_that_cannot_be_searched_with_is_refused(argument):
    """Told at the command line rather than met as an arithmetic error."""
    with pytest.raises(SystemExit):
        search_main(['f9', '--altitude', '500', argument])


def test_every_set_found_can_be_written_out(tmp_path, capsys):
    """`--csv` is the whole table, where the summary prints the best few."""
    path = tmp_path / 'found.csv'
    assert search_main(['f9', '--altitude', '500', '--programme', '5f',
                        *NARROW, '--top', '3', '--csv', str(path)]) == 0

    rows = list(csv.DictReader(path.read_text(encoding='utf-8').splitlines()))
    assert len(rows) > 3, 'the file holds more than the summary printed'
    assert {'t1', 'k2', 'k3', 't4', 'coast'} <= set(rows[0])
    assert rows[0]['reaches_orbit'] == 'yes'
    errors = [float(row['orbit_error']) for row in rows]
    assert errors == sorted(errors)
    assert str(path) in capsys.readouterr().out


def test_the_search_reports_the_set_it_found(tmp_path, monkeypatch):
    """`--report` flies what was found and writes the page `ascent` writes."""
    opened = []
    monkeypatch.setattr('webbrowser.open', lambda url: opened.append(url))
    directory = tmp_path / 'found'

    assert search_main(['f9', '--altitude', '500', '--programme', '5f',
                        *NARROW, '--report', str(directory)]) == 0

    page = (directory / 'index.html').read_text(encoding='utf-8')
    assert 'Falcon 9 to 500 km' in page
    assert 'Cape Canaveral SLC-40, Florida' in page
    assert 'ascent-search f9 --altitude 500' in page
    assert (directory / 'attitude.png').exists()
    assert opened


def test_a_search_that_reaches_nothing_writes_no_report(tmp_path, capsys):
    """There is no flight to report: a set that misses is not an entry."""
    directory = tmp_path / 'nothing'

    assert search_main(['f9', '--altitude', '500', '--programme', '5f',
                        '--range', 't1=20', '--range', 'k2=0.03:0.07:3',
                        '--range', 'k3=0.50:0.56:3',
                        '--range', 't4=502:503:3',
                        '--tolerance', '0.001', '--refinements', '0',
                        '--steps', '1', '--workers', '1',
                        '--report', str(directory)]) == 1

    assert not directory.exists()
    assert 'no report' in capsys.readouterr().out
