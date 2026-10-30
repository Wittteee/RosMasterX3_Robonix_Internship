"""One-shot authorization and navigation orchestration, live or in-memory."""
from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import Any
from x3_nav.config import load_config, validate_config, project_root, finite_number
from x3_nav.bridge.client import RosBridge
from x3_nav.logutil import write_json_log
from x3_nav.nav.client import NavClient
from x3_nav.nav.pose import PoseStore
from x3_nav.nav.status import GoalStatusMonitor
from x3_nav.safety import CandidateGoal, SafetyController, Phase
from x3_nav.validation import check_topic_rates, summarize_health


@dataclass
class MissionResult:
    ok: bool
    phase: str
    status_name: str | None = None
    detail: str = ''
    snapshot: dict[str, Any] = field(default_factory=dict)
    log_path: str | None = None


class MissionRunner:
    def __init__(self, config=None, *, bridge_factory=None, clock=time.monotonic,
                 sleep=time.sleep, log_root=None):
        self.config = validate_config(config) if config is not None else load_config()
        self.clock, self.sleep = clock, sleep
        self.bridge_factory = bridge_factory or (lambda: RosBridge(self.config))
        self.log_root = log_root or project_root()
        self.ctrl = SafetyController(self.config['mission']['token_ttl_sec'],
                                     self.config['safety']['candidate_distance_m'], clock=clock)
        self.used = False
        self.last_bridge = None
        self.extra = {}
        self.mode = 'live'
        self.attempted = False
        self.nav = None
        self.monitor = GoalStatusMonitor()

    def dry_run(self, goal=None, *, human_confirm=False, dispatch=False,
                auto_preempt=False, timeout_sec=None, save_log=True, outcome=3):
        if self.used:
            raise RuntimeError('Use a new MissionRunner for each task')
        from x3_nav.bridge.offline import OfflineBridge, SimClock
        self.clock = SimClock()
        bridge = OfflineBridge(self.config, self.clock, outcome=outcome)
        self.bridge_factory = lambda: bridge
        self.sleep = bridge.sleep
        self.ctrl = SafetyController(self.config['mission']['token_ttl_sec'],
                                     self.config['safety']['candidate_distance_m'], clock=self.clock)
        self.mode = 'dry-run'
        return self.run(goal, human_confirm=human_confirm, dispatch=dispatch,
                        auto_preempt=auto_preempt, timeout_sec=timeout_sec, save_log=save_log)

    def run(self, goal=None, *, human_confirm=False, dispatch=False, auto_preempt=False,
            timeout_sec=None, health_duration_sec=None, save_log=True):
        if dispatch and human_confirm is not True:
            raise RuntimeError('未人工确认时禁止 dispatch（尚未连接）')
        if self.used:
            raise RuntimeError('One runner handles exactly one task; no token replay')
        self.used = True
        cfg = self.config['mission']
        timeout = finite_number(timeout_sec if timeout_sec is not None else cfg['default_timeout_sec'], 'timeout', positive=True)
        duration = finite_number(health_duration_sec if health_duration_sec is not None else cfg['health_duration_sec'], 'health duration', positive=True)
        self.extra = dict(mode=self.mode, physical_motion_verified=False, goal_publish_attempted=False,
                          cancel_requested=False, cancel_terminal_observed=False)
        ok, detail = False, ''
        try:
            self.last_bridge = self.bridge_factory()
            with self.last_bridge as bridge:
                self.nav = NavClient(bridge)
                poses = PoseStore(self.clock)
                subs = []
                try:
                    subs.append(self.nav.subscribe_status(self.monitor.callback))
                    subs.append(self.nav.subscribe_odom(lambda msg: poses.update('odom', msg)))
                    subs.append(self.nav.subscribe_amcl(lambda msg: poses.update('amcl', msg)))
                    deadline = self.clock() + duration
                    pose = poses.get(cfg['frame_id'], cfg['pose_max_age_sec'])
                    while pose is None and self.clock() < deadline:
                        self.sleep(cfg['poll_interval_sec'])
                        pose = poses.get(cfg['frame_id'], cfg['pose_max_age_sec'])
                    if pose is None:
                        raise RuntimeError('No fresh pose in mission frame; refusing origin=(0,0) fallback')
                    if self.monitor.has_active_goal:
                        raise RuntimeError('Another navigation task is active')
                    candidate = goal(pose) if callable(goal) else goal
                    if candidate is None:
                        candidate = offset_goal_from_robot(pose.xy, self.config['safety']['candidate_distance_m'], pose.yaw, cfg['frame_id'])
                    if candidate.frame_id != cfg['frame_id']:
                        raise ValueError('Candidate and localization frames differ')
                    self.ctrl.start_preview(candidate, robot_xy=pose.xy)
                    self.extra['health'] = summarize_health(check_topic_rates(bridge, duration, sleep=self.sleep))
                    if not self.extra['health']['ok']:
                        raise RuntimeError('Missing required topic data: ' + str(self.extra['health']['missing']))
                    current = self._validate_pose(poses, candidate)
                    if self.monitor.has_active_goal:
                        raise RuntimeError('Another navigation task became active during preview')
                    token = self.ctrl.prepare(True, 'short-window sensor data present')
                    if not human_confirm:
                        ok, detail = True, 'Prepared; no human confirmation, no goal sent'
                    else:
                        self.ctrl.begin_confirmation()
                        self.ctrl.confirm(token, human_ok=human_confirm)
                        if not dispatch:
                            ok, detail = True, 'Confirmed only; dispatch disabled'
                        else:
                            self._validate_pose(poses, candidate)
                            ready = self.ctrl.mark_executing()
                            self.monitor.arm()
                            self.attempted = True
                            self.extra['goal_publish_attempted'] = True
                            self.nav.publish_goal(ready)
                            ok, detail = self._monitor(poses, auto_preempt, timeout)
                except (Exception, KeyboardInterrupt) as exc:
                    detail = 'interrupted' if isinstance(exc, KeyboardInterrupt) else str(exc)
                    if self.attempted:
                        self.extra['status_before_cancel'] = self.monitor.snapshot()
                        stopped = self._cancel_and_wait()
                        if isinstance(exc, KeyboardInterrupt) and stopped and self.monitor.last_status_code in {2, 8}:
                            self.ctrl.mark_preempted('human interrupt, terminal cancellation observed')
                            ok = True
                    if self.ctrl.session.phase not in {Phase.SUCCEEDED, Phase.PREEMPTED, Phase.FAILED}:
                        self.ctrl.mark_failed(detail)
                finally:
                    for sub in subs:
                        try:
                            sub.unsubscribe()
                        except Exception:
                            pass
        except (Exception, KeyboardInterrupt) as exc:
            detail = str(exc) or 'interrupted during connection'
            if self.ctrl.session.phase not in {Phase.SUCCEEDED, Phase.PREEMPTED, Phase.FAILED}:
                self.ctrl.mark_failed(detail)
            ok = False
        self.extra['status'] = self.monitor.snapshot()
        return self._finish(ok, detail, save_log=save_log)

    def _validate_pose(self, poses, candidate):
        cfg = self.config['mission']
        pose = poses.get(cfg['frame_id'], cfg['pose_max_age_sec'])
        if pose is None:
            raise RuntimeError('Localization missing or stale')
        if math.dist(pose.xy, self.ctrl.session.start_xy) > cfg['preview_motion_tolerance_m']:
            raise RuntimeError('Robot moved during preview; obtain a new preview')
        if candidate.distance_from(*pose.xy) > self.config['safety']['candidate_distance_m'] + 1e-6:
            raise RuntimeError('Candidate now exceeds distance limit')
        return pose

    def _monitor(self, poses, auto_preempt, timeout):
        cfg = self.config['mission']
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            status = self.monitor.snapshot()
            if status['ambiguous']:
                raise RuntimeError('Multiple new goals observed; cannot correlate this task')
            code = status['status_code']
            if code == 3:
                self.ctrl.mark_succeeded()
                return True, 'SUCCEEDED (reported by navigation, not a physical measurement)'
            if code in {2, 8}:
                self.ctrl.mark_preempted('terminal cancellation observed')
                self.extra['cancel_terminal_observed'] = True
                return True, 'PREEMPTED'
            if code in {4, 5, 9}:
                raise RuntimeError(status['status_name'])
            pose = poses.get(cfg['frame_id'], cfg['pose_max_age_sec'])
            if pose is None:
                raise RuntimeError('Localization became stale during execution')
            self.extra['last_pose_xy'] = pose.xy
            if auto_preempt and self.ctrl.should_preempt_by_displacement(
                    pose.xy, self.config['safety']['preempt_trigger_displacement_m']):
                if self._cancel_and_wait() and self.monitor.last_status_code in {2, 8}:
                    self.ctrl.mark_preempted('displacement protection; terminal cancellation observed')
                    return True, 'PREEMPTED by displacement protection'
                raise RuntimeError('Cancel requested but PREEMPTED was not observed')
            self.sleep(cfg['poll_interval_sec'])
        raise TimeoutError('Navigation monitoring timeout')

    def _cancel_and_wait(self):
        if self.extra['cancel_requested']:
            return self.extra['cancel_terminal_observed']
        self.extra['cancel_requested'] = True
        try:
            self.nav.cancel()
            deadline = self.clock() + self.config['mission']['cancel_timeout_sec']
            while self.clock() < deadline:
                if self.monitor.is_terminal() and not self.monitor.ambiguous:
                    self.extra['cancel_terminal_observed'] = self.monitor.last_status_code in {2, 8}
                    return True
                self.sleep(self.config['mission']['poll_interval_sec'])
        except Exception as exc:
            self.extra['cancel_error'] = str(exc)
        return False

    def _finish(self, ok, detail, *, save_log):
        snapshot = self.ctrl.snapshot()
        status = self.monitor.snapshot()['status_name']
        result = MissionResult(ok, snapshot['phase'], status, detail, snapshot)
        if save_log:
            payload = dict(ok=ok, phase=result.phase, status_name=status, detail=detail,
                           snapshot=snapshot, extra=self.extra)
            result.log_path = str(write_json_log(self.log_root, 'mission', payload))
        return result


def offset_goal_from_robot(robot_xy, distance_m, yaw=0.0, frame_id='map'):
    return CandidateGoal(robot_xy[0] + distance_m*math.cos(yaw),
                         robot_xy[1] + distance_m*math.sin(yaw), yaw, frame_id)
