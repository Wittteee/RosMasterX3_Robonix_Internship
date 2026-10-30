from unittest.mock import patch
import pytest
from x3_nav.bridge.offline import OfflineBridge, SimClock
from x3_nav.config import load_config
from x3_nav.mission.runner import MissionRunner
from x3_nav.safety import CandidateGoal


def setup(bridge_class=OfflineBridge):
    cfg = load_config()
    clock = SimClock()
    bridge = bridge_class(cfg, clock)
    runner = MissionRunner(cfg, bridge_factory=lambda: bridge, clock=clock, sleep=bridge.sleep)
    return runner, bridge


def test_preview_movement_prevents_token():
    class MovingDuringPreview(OfflineBridge):
        def sleep(self, dt):
            self.xy = (.1, 0)
            super().sleep(dt)
    r, b = setup(MovingDuringPreview)
    result = r.run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True, save_log=False)
    assert result.phase == 'FAILED'
    assert not b.publications and not result.snapshot['has_token']


def test_publish_exception_attempts_cancel_once():
    class BrokenPublish(OfflineBridge):
        def publish(self, topic, *args, **kwargs):
            if topic.endswith('/goal'):
                raise RuntimeError('publish outcome unknown')
            super().publish(topic, *args, **kwargs)
    r, b = setup(BrokenPublish)
    result = r.run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True, save_log=False)
    assert result.phase == 'FAILED'
    assert len(b.publications) == 1 and b.publications[0]['topic'].endswith('/cancel')
    assert not r.extra['cancel_terminal_observed']


def test_cancel_failure_is_not_preempted():
    class BrokenCancel(OfflineBridge):
        def publish(self, topic, *args, **kwargs):
            if topic.endswith('/cancel'):
                raise RuntimeError('link lost')
            super().publish(topic, *args, **kwargs)
    r, b = setup(BrokenCancel)
    result = r.run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True, auto_preempt=True, save_log=False)
    assert result.phase == 'FAILED'
    assert r.extra['cancel_error'] == 'link lost'


def test_interrupt_requests_cancel_and_observes_preempted():
    class Interrupting(OfflineBridge):
        def sleep(self, dt):
            if self.code == 1:
                raise KeyboardInterrupt()
            super().sleep(dt)
    r, b = setup(Interrupting)
    result = r.run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True, save_log=False)
    assert result.phase == 'PREEMPTED'
    assert r.extra['cancel_terminal_observed']


def test_no_socket_created_in_dry_run():
    with patch('roslibpy.Ros', side_effect=AssertionError('network access')):
        result = MissionRunner().dry_run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True, save_log=False)
    assert result.phase == 'SUCCEEDED'


def test_stale_localization_after_dispatch_cancels():
    class StalePose(OfflineBridge):
        def publish(self, topic, *args, **kwargs):
            super().publish(topic, *args, **kwargs)
            if topic.endswith('/goal'):
                self.missing.add('amcl_pose')
                self.outcome = None
    r, b = setup(StalePose)
    result = r.run(CandidateGoal(.3, 0), human_confirm=True, dispatch=True, save_log=False)
    assert result.phase == 'FAILED'
    assert 'stale' in result.detail.lower()
    assert r.extra['cancel_requested']
