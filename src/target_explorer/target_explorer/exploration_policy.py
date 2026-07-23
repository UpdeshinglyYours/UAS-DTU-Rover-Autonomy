"""Simple target-directed frontier scoring."""

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional


@dataclass
class Candidate:
    """One possible NavigateToPose endpoint."""

    x: float
    y: float
    theta: float
    progress: float
    alignment: float
    distance_from_robot: float
    score: float
    path_length: Optional[float] = None
    is_final_target: bool = False


def make_candidate(
    x: float,
    y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    target_x: float,
    target_y: float,
    progress_weight: float,
    alignment_weight: float,
    path_cost_weight: float,
    is_final_target: bool = False,
) -> Candidate:
    """Create a candidate using the final target as the only heuristic."""
    robot_target_distance = math.hypot(
        target_x - robot_x,
        target_y - robot_y,
    )
    candidate_target_distance = math.hypot(target_x - x, target_y - y)
    progress = robot_target_distance - candidate_target_distance

    candidate_distance = math.hypot(x - robot_x, y - robot_y)
    alignment = 0.0
    if robot_target_distance > 1e-9 and candidate_distance > 1e-9:
        alignment = (
            (target_x - robot_x) * (x - robot_x)
            + (target_y - robot_y) * (y - robot_y)
        ) / (robot_target_distance * candidate_distance)
        alignment = max(-1.0, min(1.0, alignment))

    if candidate_target_distance > 1e-9:
        theta = math.atan2(target_y - y, target_x - x)
    elif robot_target_distance > 1e-9:
        theta = math.atan2(target_y - robot_y, target_x - robot_x)
    else:
        theta = robot_yaw

    candidate = Candidate(
        x=x,
        y=y,
        theta=theta,
        progress=progress,
        alignment=alignment,
        distance_from_robot=candidate_distance,
        score=0.0,
        is_final_target=is_final_target,
    )
    update_score(
        candidate,
        progress_weight,
        alignment_weight,
        path_cost_weight,
    )
    return candidate


def update_score(
    candidate: Candidate,
    progress_weight: float,
    alignment_weight: float,
    path_cost_weight: float,
) -> float:
    """Apply the deliberately small, unnormalized scoring formula."""
    path_length = (
        candidate.path_length
        if candidate.path_length is not None
        else 0.0
    )
    candidate.score = (
        progress_weight * candidate.progress
        + alignment_weight * candidate.alignment
        - path_cost_weight * path_length
    )
    return candidate.score


def select_candidate(
    candidates: Iterable[Candidate],
    minimum_target_progress_m: float,
    allow_negative_progress_recovery: bool,
) -> Optional[Candidate]:
    """
    Prefer forward progress and use non-forward candidates only as recovery.

    Reachability is expected to have been checked before this function is
    called.
    """
    candidate_list: List[Candidate] = list(candidates)
    forward = [
        candidate
        for candidate in candidate_list
        if candidate.progress >= minimum_target_progress_m
    ]
    if forward:
        return max(forward, key=lambda candidate: candidate.score)
    if allow_negative_progress_recovery and candidate_list:
        return max(candidate_list, key=lambda candidate: candidate.score)
    return None
