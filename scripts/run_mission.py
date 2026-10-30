"""Manual/relative short-step navigation, with independent consent and dispatch."""
import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from x3_nav.config import load_config
from x3_nav.mission.runner import MissionRunner, offset_goal_from_robot
from x3_nav.safety import CandidateGoal


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--x', type=float, help='Absolute goal x in configured frame')
    parser.add_argument('--y', type=float, default=0.)
    parser.add_argument('--yaw', type=float, default=0., help='Goal yaw in radians')
    parser.add_argument('--distance', type=float, help='Forward step from current pose in metres')
    parser.add_argument('--confirm', action='store_true')
    parser.add_argument('--dispatch', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--auto-preempt', action='store_true', help='Cancellation test, incompatible with arrival experiment')
    parser.add_argument('--timeout', type=float)
    args = parser.parse_args(argv)
    if args.dispatch and not args.confirm:
        parser.error('--dispatch requires --confirm')
    if args.x is not None and args.distance is not None:
        parser.error('Use either absolute --x/--y or relative --distance')
    try:
        cfg = load_config(args.config)
        goal = None
        if args.x is not None:
            goal = CandidateGoal(args.x, args.y, args.yaw, cfg['mission']['frame_id'])
        elif args.distance is not None:
            if not 0 < args.distance <= cfg['safety']['candidate_distance_m']:
                raise ValueError('Distance must be positive and within candidate limit')
            goal = lambda pose: offset_goal_from_robot(pose.xy, args.distance, pose.yaw, pose.frame_id)
        runner = MissionRunner(cfg)
        run = runner.dry_run if args.dry_run else runner.run
        result = run(goal, human_confirm=args.confirm, dispatch=args.dispatch,
                     auto_preempt=args.auto_preempt, timeout_sec=args.timeout)
        print(json.dumps(dict(asdict(result), mode=runner.mode), ensure_ascii=False, indent=2))
        return 0 if result.ok else 2
    except (ValueError, RuntimeError, KeyError, OSError) as exc:
        print(json.dumps(dict(ok=False, error=str(exc)), ensure_ascii=False))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
