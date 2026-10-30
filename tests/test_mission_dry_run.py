import json
from unittest.mock import Mock
import pytest
from x3_nav.config import load_config
from x3_nav.safety import CandidateGoal
from x3_nav.bridge.offline import OfflineBridge, SimClock
from x3_nav.mission.runner import MissionRunner


@pytest.mark.parametrize('confirm,dispatch,preempt,phase,goals,cancels', [
    (False, False, False, 'PREPARE', 0, 0), (True, False, False, 'CONFIRM', 0, 0),
    (True, True, False, 'SUCCEEDED', 1, 0), (True, True, True, 'PREEMPTED', 1, 1)])
def test_full_offline_chain(tmp_path, confirm, dispatch, preempt, phase, goals, cancels):
    runner = MissionRunner(log_root=tmp_path)
    result = runner.dry_run(CandidateGoal(.3, 0), human_confirm=confirm, dispatch=dispatch, auto_preempt=preempt)
    assert result.ok and result.phase == phase
    msgs = runner.last_bridge.publications
    assert sum(m['topic'].endswith('/goal') for m in msgs) == goals
    assert sum(m['topic'].endswith('/cancel') for m in msgs) == cancels
    assert runner.last_bridge.closed
    data = json.loads(open(result.log_path, encoding='utf-8').read())
    assert data['extra']['mode'] == 'dry-run'
    assert not data['extra']['physical_motion_verified']
    assert data['log_path'] == result.log_path
    assert 'token' not in data['extra']
    with pytest.raises(RuntimeError):
        runner.run(human_confirm=True, dispatch=True)


def test_unconfirmed_dispatch_never_connects():
    factory = Mock()
    with pytest.raises(RuntimeError):
        MissionRunner(bridge_factory=factory).run(dispatch=True)
    factory.assert_not_called()


@pytest.mark.parametrize('missing', ['scan', 'odom', 'amcl_pose'])
def test_missing_topics_never_prepare(missing):
    cfg = load_config()
    clock = SimClock()
    bridge = OfflineBridge(cfg, clock, missing=[missing])
    runner = MissionRunner(cfg, bridge_factory=lambda: bridge, clock=clock, sleep=bridge.sleep)
    result = runner.run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True, save_log=False)
    assert result.phase == 'FAILED'
    assert not bridge.publications
    assert not result.snapshot['has_token']
    assert bridge.closed


def test_timeout_cancels_and_is_failure(tmp_path):
    runner = MissionRunner(log_root=tmp_path)
    result = runner.dry_run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True,
                            timeout_sec=.2, outcome=None)
    assert result.phase == 'FAILED'
    assert runner.last_bridge.publications[-1]['topic'] == '/move_base/cancel'
    assert runner.extra['cancel_requested']


@pytest.mark.parametrize('code', [4, 5, 9])
def test_failed_status_cancels(tmp_path, code):
    runner = MissionRunner(log_root=tmp_path)
    result = runner.dry_run(CandidateGoal(.1, 0), human_confirm=True, dispatch=True, outcome=code)
    assert not result.ok and result.phase == 'FAILED'
    assert runner.extra['cancel_requested']


def test_wrong_frame_and_overdistance_rejected():
    for goal in (CandidateGoal(.3, 0, frame_id='odom'), CandidateGoal(.81, 0)):
        runner = MissionRunner()
        result = runner.dry_run(goal, human_confirm=True, dispatch=True, save_log=False)
        assert not result.ok
        assert not runner.last_bridge.publications
