import json
from unittest.mock import patch
import pytest
import run_mission
import run_detect_nav
import run_visual_nav


def test_cli_consent_rejected_before_runner():
    with patch('run_mission.MissionRunner') as runner:
        with pytest.raises(SystemExit) as err:
            run_mission.main(['--dispatch', '--dry-run'])
        assert err.value.code == 2
        runner.assert_not_called()


def test_cli_confirmation_without_dispatch(capsys):
    assert run_mission.main(['--dry-run', '--confirm']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['phase'] == 'CONFIRM'


def test_detection_mock_dispatch_stays_offline(capsys):
    with patch('roslibpy.Ros', side_effect=AssertionError('network')):
        assert run_detect_nav.main(['--mock', '--confirm', '--dispatch']) == 0
    assert json.loads(capsys.readouterr().out)['phase'] == 'SUCCEEDED'


def test_qr_demo_does_not_load_live_provider(tmp_path, capsys):
    with patch('x3_nav.bridge.robonix_navigation.RobonixNavigation', side_effect=AssertionError('live')):
        assert run_visual_nav.main(['--demo', '--output-dir', str(tmp_path)]) == 0
    data = json.loads(next(tmp_path.glob('mission-*.json')).read_text(encoding='utf-8'))
    assert data['snapshot']['phase'] == 'PREPARE'
    assert not data['physical_arrival_verified']


def test_demo_and_live_provider_cannot_mix():
    with pytest.raises(SystemExit):
        run_visual_nav.main(['--demo', '--provider-dir', 'dummy'])
