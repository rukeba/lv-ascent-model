"""The command line: how a mission is found, and what it is flown with."""

from pathlib import Path

import pytest
import yaml

from ascent.cli import main, search_main


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


def test_altitude_absent_from_the_catalogue_is_reported():
    with pytest.raises(LookupError, match='altitudes available'):
        main(['f9', '--altitude', '999'])


def test_listing_the_catalogue(capsys):
    assert main(['f9', '--list']) == 0
    assert 'bilinear-tangent' in capsys.readouterr().out


def test_the_search_writes_out_the_set_it_found(capsys):
    """`ascent-search --yaml` writes an entry the catalogue reader can read.

    Coarse and shallow on purpose - what is under test is the plumbing of the
    command, not the search: the units the arguments are read in, the vehicle
    found beside the mission file, the set written out and the exit status. The
    tolerance is opened wide so that a grid this rough reaches it.

    Nothing but the entry goes to the output, so that a redirect of it is a
    file rather than a file with a summary printed across the top.
    """
    assert search_main(['f9', '--altitude', '500', '--programme', 'five-phase',
                        '--tolerance', '20', '--coarse', '0.3',
                        '--refinements', '1', '--steps', '1', '--yaml']) == 0

    written = capsys.readouterr()
    assert 'Falcon 9 to 500 km' in written.err
    entry = yaml.safe_load(written.out)['missions'][0]
    assert entry['vehicle'] == 'lv.f9'
    assert entry['target_altitude'] == 500_000
    assert entry['launch_site'] == {'latitude': 28.5, 'azimuth': 90}
    assert entry['pitch_programme']['type'] == 'five-phase'
    assert entry['cutoff']['time'] == pytest.approx(502.7, abs=5.0)


def test_a_search_that_reaches_nothing_writes_nothing_out(capsys):
    """A set that misses the orbit is shown, and is not filed as an entry."""
    assert search_main(['f9', '--altitude', '500', '--programme', 'five-phase',
                        '--tolerance', '0.001', '--coarse', '0.3',
                        '--refinements', '0', '--steps', '1', '--yaml']) == 1
    written = capsys.readouterr()
    assert 'reaches the orbit       no' in written.err
    assert written.out.strip() == ''


def test_the_search_summary_is_the_output_without_yaml(capsys):
    """Without `--yaml` there is nothing to redirect, so it reads normally."""
    assert search_main(['f9', '--altitude', '500', '--programme', 'five-phase',
                        '--tolerance', '20', '--coarse', '0.3',
                        '--refinements', '1', '--steps', '1']) == 0
    written = capsys.readouterr()
    assert 'Falcon 9 to 500 km' in written.out
    assert 'missions:' not in written.out


def test_the_search_is_refused_an_orbit_out_of_reach():
    with pytest.raises(ValueError, match='does not have the propellant'):
        search_main(['f9', '--altitude', '20000'])


@pytest.mark.parametrize('argument', ['--steps=0', '--tolerance=0',
                                      '--coarse=-1', '--refinements=-1',
                                      '--max-q=0'])
def test_a_setting_that_cannot_be_searched_with_is_refused(argument):
    """Told at the command line rather than met as an arithmetic error."""
    with pytest.raises(SystemExit):
        search_main(['f9', '--altitude', '500', argument])
