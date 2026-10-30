"""Detection proposal -> common authorization chain; default is offline."""
import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from x3_nav.config import load_config
from x3_nav.vision.detect import Detection
from x3_nav.mission.detection import run_detection


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--mock', action='store_true')
    mode.add_argument('--live', action='store_true', help='Use CLI-injected observation on real robot')
    parser.add_argument('--label')
    parser.add_argument('--offset-x', type=float, default=.15)
    parser.add_argument('--size', type=float, default=.22)
    parser.add_argument('--confidence', type=float, default=.86)
    parser.add_argument('--confirm', action='store_true')
    parser.add_argument('--dispatch', action='store_true')
    parser.add_argument('--auto-preempt', action='store_true')
    parser.add_argument('--timeout', type=float)
    args = parser.parse_args(argv)
    if args.dispatch and not args.confirm:
        parser.error('--dispatch requires --confirm')
    try:
        cfg = load_config(args.config)
        det = Detection(args.label or cfg['vision']['target_label'], args.confidence,
                        args.offset_x, args.size, 'cli-injection' if args.live else 'mock')
        plan, result = run_detection(cfg, det, dry_run=not args.live, human_confirm=args.confirm,
            dispatch=args.dispatch, auto_preempt=args.auto_preempt, timeout_sec=args.timeout)
        print(json.dumps(dict(asdict(result), plan=plan.to_dict() if plan else None,
                             mode='live-cli-injection' if args.live else 'dry-run'), ensure_ascii=False, indent=2))
        return 0 if result.ok else 2
    except (ValueError, RuntimeError, KeyError, OSError) as exc:
        print(json.dumps(dict(ok=False, error=str(exc)), ensure_ascii=False))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
