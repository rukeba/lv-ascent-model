"""The HTML report: what it is made of, and that it says what was flown."""

import re
from html import escape

import pytest

from ascent import load_mission
from ascent.cli import main
from ascent.report import LOG_INTERVAL, write_report
from ascent.summary import LABEL_WIDTH, summarise


@pytest.fixture(scope='module')
def report(tmp_path_factory):
    """One flight, reported once: the plots are slow to draw."""
    mission = load_mission('config/mission.f9.yaml')
    telemetry = mission.run()
    directory = tmp_path_factory.mktemp('report')
    path = write_report(mission, telemetry, directory)
    return path, path.read_text(encoding='utf-8'), summarise(mission, telemetry)


def test_the_page_and_every_plot_are_written(report):
    path, page, _ = report
    assert path.name == 'index.html'
    for name in ('attitude', 'altitude', 'speed', 'velocity-components',
                 'propulsion', 'acceleration', 'dynamic-pressure', 'steering',
                 'speed-altitude', 'trajectory'):
        assert (path.parent / f'{name}.png').exists()
        assert f'{name}.png' in page


def test_the_styles_are_inlined_rather_than_linked(report):
    """The page has to survive being sent on its own, images aside."""
    _, page, _ = report
    assert '<style>' in page and '.tiles' in page
    assert '<link' not in page


def test_the_page_and_the_console_agree(report):
    """Both are laid out from the same blocks, so every figure is in both."""
    _, page, console = report
    rows = [line for line in console.splitlines() if line.startswith('  ')]
    assert len(rows) > 20

    for row in rows:
        label, value = row[2:LABEL_WIDTH + 2].strip(), row[LABEL_WIDTH + 2:]
        assert escape(value) in page, f'{label}: {value}'


def test_the_log_is_sampled_at_the_interval_to_the_end_of_the_flight(report):
    _, page, _ = report
    body = page[page.index('<tbody>'):page.index('</tbody>')]
    instants = [float(row) for row in re.findall(r'<tr[^>]*>\s*<td>(\d+)</td>', body)]

    assert instants[0] == 0.0
    assert instants[1] - instants[0] == LOG_INTERVAL
    # the flight is 600 s long and the last row is its last instant
    assert instants[-1] == 600.0
    assert len(instants) == 1 + 600 / LOG_INTERVAL


def test_the_report_is_opened_unless_it_is_asked_not_to_be(tmp_path, monkeypatch):
    opened = []
    monkeypatch.setattr('webbrowser.open', lambda url: opened.append(url))

    assert main(['f9', '--report', str(tmp_path / 'shown')]) == 0
    assert opened and opened[0].endswith('/shown/index.html')

    assert main(['f9', '--report', str(tmp_path / 'quiet'), '--no-open']) == 0
    assert len(opened) == 1
