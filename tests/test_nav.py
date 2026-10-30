from unittest.mock import Mock
from x3_nav.config import load_config
from x3_nav.nav import NavClient, GoalStatusMonitor
from x3_nav.nav.pose import PoseStore
from x3_nav.safety import CandidateGoal


def status(gid, code):
    return dict(status_list=[dict(goal_id=dict(id=gid), status=code)])


def test_stale_status_and_new_goal_binding():
    m = GoalStatusMonitor()
    m.callback(status('old', 3))
    m.arm()
    m.callback(status('old', 3))
    m.callback(status('new', 3))
    assert m.last_status_code is None
    m.callback(status('new', 1))
    m.callback(status('old', 3))
    assert m.last_status_code == 1
    m.callback(status('new', 3))
    assert m.is_terminal()


def test_concurrent_goals_are_ambiguous():
    m = GoalStatusMonitor()
    m.arm()
    m.callback(status('a', 1))
    m.callback(status('b', 1))
    assert m.snapshot()['ambiguous']


def test_nav_only_encodes_goal_and_cancel():
    b = Mock(config=load_config())
    nav = NavClient(b)
    nav.publish_goal(CandidateGoal(.3, .1, 1))
    nav.cancel()
    calls = b.publish.call_args_list
    assert [c.args[0] for c in calls] == ['/move_base_simple/goal', '/move_base/cancel']
    assert calls[0].args[2]['pose']['position']['z'] == 0
    assert calls[1].args[2]['id'] == ''


def test_pose_frame_priority_and_stale_data():
    now = [0]
    store = PoseStore(lambda: now[0])
    def message(frame, x):
        return dict(header=dict(frame_id=frame), pose=dict(pose=dict(
            position=dict(x=x, y=0), orientation=dict(x=0, y=0, z=0, w=1))))
    store.update('amcl', message('map', 1))
    store.update('odom', message('odom', 100))
    assert store.latest.x == 100
    assert store.get('map', 2).x == 1
    store.update('amcl', {})
    assert store.get('map', 2).x == 1
    now[0] = 3
    assert store.get('map', 2) is None
