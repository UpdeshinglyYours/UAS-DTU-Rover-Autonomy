import math

from target_explorer.exploration_policy import (
    make_candidate,
    select_candidate,
    update_score,
)


def candidate(x, y, target_x=0.0, target_y=10.0):
    return make_candidate(
        x,
        y,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        target_x=target_x,
        target_y=target_y,
        progress_weight=1.0,
        alignment_weight=1.0,
        path_cost_weight=0.1,
    )


def test_progress_is_reduction_in_final_target_distance():
    result = candidate(0.0, 4.0)

    assert math.isclose(result.progress, 4.0)
    assert math.isclose(result.alignment, 1.0)


def test_alignment_uses_robot_to_target_not_positive_y():
    result = candidate(4.0, 0.0, target_x=10.0, target_y=0.0)

    assert math.isclose(result.progress, 4.0)
    assert math.isclose(result.alignment, 1.0)


def test_score_uses_only_progress_alignment_and_path_length():
    result = candidate(0.0, 4.0)
    result.path_length = 6.0

    score = update_score(
        result,
        progress_weight=2.0,
        alignment_weight=3.0,
        path_cost_weight=0.5,
    )

    assert math.isclose(score, 8.0)


def test_forward_candidate_wins_before_negative_recovery():
    forward = candidate(0.0, 0.1)
    backward = candidate(0.0, -1.0)
    backward.score = 1000.0

    selected = select_candidate(
        [backward, forward],
        minimum_target_progress_m=0.0,
        allow_negative_progress_recovery=True,
    )

    assert selected is forward


def test_negative_candidate_is_allowed_only_when_recovery_enabled():
    backward = candidate(0.0, -1.0)

    assert select_candidate(
        [backward],
        minimum_target_progress_m=0.0,
        allow_negative_progress_recovery=True,
    ) is backward
    assert select_candidate(
        [backward],
        minimum_target_progress_m=0.0,
        allow_negative_progress_recovery=False,
    ) is None
