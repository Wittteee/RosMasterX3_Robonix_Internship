import pytest
from x3_nav.safety import CandidateGoal, SafetyController


def prepared(clock=lambda: 0):
    c = SafetyController(clock=clock)
    c.start_preview(CandidateGoal(.3, 0), robot_xy=(0, 0))
    return c, c.prepare(True)


def test_full_chain_and_single_use():
    c, token = prepared()
    assert c.snapshot()["phase"] == "PREPARE"
    with pytest.raises(RuntimeError):
        c.confirm(token, True)
    c.begin_confirmation()
    c.confirm(token, True)
    assert c.snapshot()["has_token"]
    c.mark_executing()
    assert not c.snapshot()["has_token"]
    assert token not in str(c.snapshot())
    with pytest.raises(RuntimeError):
        c.mark_executing()
    c.mark_succeeded()
    for method in (c.mark_succeeded, lambda: c.mark_failed("late"),
                   lambda: c.start_preview(CandidateGoal(.2, 0), robot_xy=(0, 0))):
        with pytest.raises(RuntimeError):
            method()


@pytest.mark.parametrize("operation", [lambda c: c.prepare(True), lambda c: c.confirm("t", True),
    lambda c: c.begin_confirmation(), lambda c: c.mark_executing(), lambda c: c.mark_succeeded(),
    lambda c: c.mark_preempted()])
def test_jump_rejected(operation):
    with pytest.raises(RuntimeError):
        operation(SafetyController())


def test_expiry_before_and_after_confirmation():
    for before in (True, False):
        now = [0.0]
        c, token = prepared(lambda: now[0])
        c.begin_confirmation()
        if not before:
            c.confirm(token, True)
        now[0] = 60.0
        with pytest.raises(RuntimeError):
            c.confirm(token, True) if before else c.mark_executing()


def test_wrong_token_and_no_consent():
    c, token = prepared()
    c.begin_confirmation()
    for t, consent in ((token, False), ("wrong", True), (token, 1)):
        with pytest.raises(RuntimeError):
            c.confirm(t, consent)


def test_distance_health_and_protection():
    c = SafetyController()
    with pytest.raises(ValueError):
        c.start_preview(CandidateGoal(.6, .6), robot_xy=(0, 0))
    assert c.snapshot()["phase"] == "IDLE"
    with pytest.raises(ValueError):
        c.start_preview(CandidateGoal(.2, 0))
    c.start_preview(CandidateGoal(.3, 0), robot_xy=(0, 0))
    with pytest.raises(RuntimeError):
        c.prepare(False)
    token = c.prepare(True)
    c.begin_confirmation()
    c.confirm(token, True)
    c.mark_executing()
    assert not c.should_preempt_by_displacement((.079, 0), .08)
    assert c.should_preempt_by_displacement((.08, 0), .08)
    c.mark_preempted()


@pytest.mark.parametrize("value", [float("nan"), float("inf"), True])
def test_invalid_goal(value):
    with pytest.raises(ValueError):
        CandidateGoal(value, 0)


def test_concurrent_consumers_cannot_reuse_token():
    from concurrent.futures import ThreadPoolExecutor
    import time
    c, token = prepared()
    c.begin_confirmation()
    c.confirm(token, True)
    check = c._check_token
    def slow_check():
        check()
        time.sleep(.02)
    c._check_token = slow_check
    def consume(_):
        try:
            c.mark_executing()
            return True
        except RuntimeError:
            return False
    with ThreadPoolExecutor(max_workers=4) as executor:
        assert sum(executor.map(consume, range(4))) == 1
