from unittest.mock import Mock
import numpy as np
from preview_visual_target import demo_frame
from run_visual_nav import SimulationProvider
from x3_nav.vision.targets import detect_qr_targets, recommend_option
from x3_nav.mission.visual import VisualNavigation


def test_exact_text_and_duplicate_targets():
    frame = demo_frame('X3_TARGET_A')
    assert not detect_qr_targets(frame, 'x3_target_a')
    duplicate = np.concatenate([frame, frame], axis=1)
    assert len(detect_qr_targets(duplicate, 'X3_TARGET_A')) == 2


def test_no_match_or_provider_failure_never_executes():
    for preview in ({'ok': False}, {'ok': True, 'session_id': 's', 'options': []}):
        api = Mock(wraps=SimulationProvider())
        api.preview.return_value = preview
        result = VisualNavigation(api, lambda: 0).run(execute=True, confirm=lambda _: '确认执行')
        assert not result['ok']
        api.prepare.assert_not_called()
        api.execute.assert_not_called()


def test_qr_uses_shared_safety_and_no_preview_dispatch():
    api = Mock(wraps=SimulationProvider())
    mission = VisualNavigation(api, lambda: 20)
    result = mission.run()
    assert result['snapshot']['phase'] == 'PREPARE'
    api.execute.assert_not_called()
    selected = recommend_option(api.preview(), 20)
    assert selected['preview_only'] and not selected['navigation_goal_sent']
