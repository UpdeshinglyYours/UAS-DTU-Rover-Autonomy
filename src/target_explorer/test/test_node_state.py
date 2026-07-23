from types import SimpleNamespace

from action_msgs.msg import GoalStatus

from target_explorer.exploration_policy import make_candidate
from target_explorer.target_explorer_node import (
    ActionState,
    TargetExplorerNode,
)


class FakeFuture:

    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result


class FakeLogger:

    def info(self, *_args, **_kwargs):
        pass

    def warning(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def make_node(state):
    node = object.__new__(TargetExplorerNode)
    node.action_state = state
    node._goal_request_id = 3
    node._active_goal_handle = object()
    node._active_candidate = make_candidate(
        1.0,
        2.0,
        0.0,
        0.0,
        0.0,
        0.0,
        10.0,
        1.0,
        1.0,
        0.1,
    )
    node._active_goal_pose = object()
    node._active_safety_unavailable_since = None
    node._last_active_safety_check = 0.0
    node._next_selection_time = 0.0
    node.get_logger = lambda: FakeLogger()
    return node


def test_execution_state_has_only_requested_states():
    assert {state.value for state in ActionState} == {
        'IDLE',
        'SENDING',
        'ACTIVE',
        'CANCELING',
    }


def test_accepted_cancel_waits_for_terminal_action_result():
    node = make_node(ActionState.CANCELING)
    cancel_response = SimpleNamespace(goals_canceling=[object()])

    node._navigation_cancel_response(FakeFuture(cancel_response), 3)

    assert node.action_state == ActionState.CANCELING
    assert node._active_candidate is not None


def test_terminal_cancel_result_returns_to_idle():
    node = make_node(ActionState.CANCELING)
    result = SimpleNamespace(status=GoalStatus.STATUS_CANCELED)

    node._navigation_result(FakeFuture(result), 3)

    assert node.action_state == ActionState.IDLE
    assert node._active_candidate is None


def test_rejected_cancel_keeps_the_same_goal_active():
    node = make_node(ActionState.CANCELING)
    cancel_response = SimpleNamespace(goals_canceling=[])

    node._navigation_cancel_response(FakeFuture(cancel_response), 3)

    assert node.action_state == ActionState.ACTIVE
    assert node._active_candidate is not None
