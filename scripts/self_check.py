"""Offline checks: actual forbidden publish attempt, authorization and cancellation."""
import json
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from x3_nav.bridge import RosBridge, ForbiddenPublishError
from x3_nav.config import load_config
from x3_nav.mission.runner import MissionRunner
from x3_nav.mission.detection import run_detection
from x3_nav.safety import CandidateGoal
from x3_nav.vision.detect import Detection


def main():
    cfg = load_config()
    checks = []
    bridge = RosBridge(cfg)
    for topic in ('cmd_vel', '/cmd_vel', '/cmd_vel/', '/robot/cmd_vel'):
        try:
            bridge.publish(topic, 'geometry_msgs/Twist', {})
            raise AssertionError('Velocity publication was not blocked')
        except ForbiddenPublishError:
            checks.append('PASS forbidden publication: ' + topic)
    for confirm, dispatch, preempt, phase in (
            (False, False, False, 'PREPARE'), (True, False, False, 'CONFIRM'),
            (True, True, False, 'SUCCEEDED'), (True, True, True, 'PREEMPTED')):
        result = MissionRunner(cfg).dry_run(CandidateGoal(.3, 0), human_confirm=confirm,
                                           dispatch=dispatch, auto_preempt=preempt)
        assert result.ok and result.phase == phase, result
        checks.append('PASS dry-run: ' + phase)
    plan, result = run_detection(cfg, Detection('person', .9, .15), human_confirm=True, dispatch=True)
    assert result.ok and result.phase == 'SUCCEEDED'
    checks.append('PASS detection proposal -> shared mission')
    for check in checks:
        print(check)
    print(json.dumps(dict(ok=True, mode='dry-run', checks=len(checks), physical_motion_verified=False)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
