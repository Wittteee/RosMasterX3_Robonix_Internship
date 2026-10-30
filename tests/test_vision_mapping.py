import math
import pytest
from x3_nav.vision.detect import Detection, detection_to_goal
from x3_nav.config import load_config
from x3_nav.mission.detection import run_detection
from x3_nav.safety import SafetyController


@pytest.mark.parametrize('offset,size,distance,yaw', [(5, -2, .8, -30), (-5, 2, .2, 30), (0, .5, .5, 0)])
def test_clamps_and_rightward_sign(offset, size, distance, yaw):
    goal = detection_to_goal(Detection('person', .9, offset, size), robot_xy=(2, 3), robot_yaw=1)
    assert goal.distance_from(2, 3) == pytest.approx(distance)
    assert goal.yaw == pytest.approx(1 + math.radians(yaw))


@pytest.mark.parametrize('det', [Detection('other', .9, 0), Detection('person', .1, 0),
                                 Detection('person', float('nan'), 0), Detection('person', .9, float('inf'))])
def test_bad_detection_rejected(det):
    with pytest.raises(ValueError):
        detection_to_goal(det, target_label='person')


def test_preview_is_independent_distance_gate():
    goal = detection_to_goal(Detection('person', .9, 0, 0))
    with pytest.raises(ValueError):
        SafetyController(max_candidate_distance_m=.3).start_preview(goal, robot_xy=(0, 0))


def test_detection_full_authorization(tmp_path):
    plan, result = run_detection(load_config(), Detection('person', .9, .2),
        human_confirm=True, dispatch=True, log_root=tmp_path)
    assert plan and result.phase == 'SUCCEEDED'
    with pytest.raises(ValueError):
        run_detection(load_config(), Detection('person', .9, .2), dry_run=False, dispatch=True, human_confirm=True)
