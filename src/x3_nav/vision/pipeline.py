"""Detection proposals only. Authorization and I/O belong to mission/bridge."""
from __future__ import annotations
from dataclasses import dataclass, field, asdict
from x3_nav.config import validate_config
from x3_nav.safety import CandidateGoal
from x3_nav.vision.detect import Detection, detection_to_goal


@dataclass
class DetectNavPlan:
    detection: Detection
    goal: CandidateGoal
    notes: list[str] = field(default_factory=list)

    def to_dict(self):
        return asdict(self)


class DetectNavPipeline:
    def __init__(self, config):
        self.config = validate_config(config)
        v = self.config['vision']
        self.target_label = v['target_label']
        self.min_confidence = v['min_confidence']
        self.max_distance_m = v['max_approach_m']
        self.min_distance_m = v['min_approach_m']
        self.max_bearing_deg = v['max_yaw_deg']
        self.frame_id = self.config['mission']['frame_id']

    def plan_from_detection(self, det, *, robot_xy=(0., 0.), robot_yaw=0.):
        goal = detection_to_goal(det, robot_xy=robot_xy, robot_yaw=robot_yaw,
            max_distance_m=self.max_distance_m, min_distance_m=self.min_distance_m,
            max_bearing_deg=self.max_bearing_deg, frame_id=self.frame_id,
            target_label=self.target_label, min_confidence=self.min_confidence)
        return DetectNavPlan(det, goal, [
            'Heuristic short-step proposal, not metric object localization',
            'Proposal distance: {:.3f} m'.format(goal.distance_from(*robot_xy))])

    def run_live(self, *args, **kwargs):
        raise RuntimeError('Vision cannot execute; use x3_nav.mission.detection.run_detection')
