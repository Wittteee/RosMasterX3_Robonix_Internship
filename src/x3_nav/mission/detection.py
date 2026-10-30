"""Route detection proposals through the same MissionRunner as manual goals."""
from x3_nav.vision.pipeline import DetectNavPipeline
from x3_nav.mission.runner import MissionRunner


def run_detection(config, detection, *, dry_run=True, human_confirm=False,
                  dispatch=False, auto_preempt=False, timeout_sec=None, log_root=None):
    if not dry_run and detection.source == 'mock':
        raise ValueError('Mock detections may not be executed on a live robot')
    pipeline = DetectNavPipeline(config)
    plans = []
    def propose(pose):
        plan = pipeline.plan_from_detection(detection, robot_xy=pose.xy, robot_yaw=pose.yaw)
        plans.append(plan)
        return plan.goal
    runner = MissionRunner(config, log_root=log_root)
    method = runner.dry_run if dry_run else runner.run
    result = method(propose, human_confirm=human_confirm, dispatch=dispatch,
                    auto_preempt=auto_preempt, timeout_sec=timeout_sec)
    return (plans[0] if plans else None), result
