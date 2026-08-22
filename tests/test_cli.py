"""The command line: how a mission is found, and what it is flown with."""

from pathlib import Path

import pytest

from ascent.cli import main


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
