"""Target-directed frontier selection with one NavigateToPose goal."""

import math
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .endpoint_safety import ClearanceResult, combined_clearance
from .exploration_policy import (
    Candidate,
    make_candidate,
    select_candidate,
    update_score,
)
from .frontier_search import FrontierSearch
from .grid_snapshot import GridSnapshot


class ActionState(str, Enum):
    """The complete NavigateToPose execution state."""

    IDLE = 'IDLE'
    SENDING = 'SENDING'
    ACTIVE = 'ACTIVE'
    CANCELING = 'CANCELING'


class TargetExplorerNode(Node):
    """Choose where to go; leave route generation and driving to Nav2."""

    _MANAGER_PERIOD_SEC = 0.1
    _SELECTION_RETRY_PERIOD_SEC = 1.0

    def __init__(self) -> None:
        super().__init__('target_explorer_node')

        self.goal_mode = self._declare_string('goal_mode', 'local').lower()
        if self.goal_mode != 'local':
            raise ValueError(
                'target_explorer supports goal_mode="local"; '
                f'received {self.goal_mode!r}'
            )

        self.global_frame = self._declare_string('global_frame', 'map')
        self.robot_base_frame = self._declare_string(
            'robot_base_frame',
            'base_footprint',
        )
        self.map_topic = self._declare_string('map_topic', '/map')
        self.global_costmap_topic = self._declare_string(
            'global_costmap_topic',
            '/global_costmap/costmap',
        )
        self.navigate_to_pose_action = self._declare_string(
            'navigate_to_pose_action',
            '/navigate_to_pose',
        )
        self.compute_path_to_pose_action = self._declare_string(
            'compute_path_to_pose_action',
            '/compute_path_to_pose',
        )

        self.target_x = self._declare_float('target_x', 0.0)
        self.target_y = self._declare_float('target_y', 40.0)
        self.target_tolerance = self._declare_float(
            'target_tolerance',
            1.0,
        )

        self.goal_progress_weight = self._declare_float(
            'goal_progress_weight',
            1.0,
        )
        self.target_alignment_weight = self._declare_float(
            'target_alignment_weight',
            1.0,
        )
        self.path_cost_weight = self._declare_float(
            'path_cost_weight',
            0.10,
        )
        self.minimum_target_progress_m = self._declare_float(
            'minimum_target_progress_m',
            0.0,
        )
        self.allow_negative_progress_recovery = self._declare_bool(
            'allow_negative_progress_recovery',
            True,
        )

        self.goal_obstacle_clearance_m = self._declare_float(
            'goal_obstacle_clearance_m',
            1.7,
        )
        self.map_occupied_threshold = self._declare_int(
            'map_occupied_threshold',
            65,
        )
        self.costmap_lethal_threshold = self._declare_int(
            'costmap_lethal_threshold',
            100,
        )
        self.active_goal_safety_check_period_sec = self._declare_float(
            'active_goal_safety_check_period_sec',
            0.5,
        )

        self.minimum_frontier_distance_from_robot = self._declare_float(
            'minimum_frontier_distance_from_robot',
            0.5,
        )
        self.minimum_frontier_size_m = self._declare_float(
            'minimum_frontier_size_m',
            0.5,
        )
        self.maximum_goal_samples_per_frontier = self._declare_int(
            'maximum_goal_samples_per_frontier',
            12,
        )
        self.planner_timeout_sec = self._declare_float(
            'planner_timeout_sec',
            2.0,
        )
        self.tf_timeout_sec = self._declare_float('tf_timeout_sec', 0.3)
        self.persistent_tf_failure_timeout_sec = self._declare_float(
            'persistent_tf_failure_timeout_sec',
            3.0,
        )

        self._frontier_search = FrontierSearch(
            minimum_frontier_size_m=self.minimum_frontier_size_m,
            maximum_goal_samples_per_frontier=(
                self.maximum_goal_samples_per_frontier
            ),
        )

        self._map_grid: Optional[GridSnapshot] = None
        self._costmap_grid: Optional[GridSnapshot] = None
        self._received_first_map = False
        self._received_first_costmap = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action,
        )
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            self.compute_path_to_pose_action,
        )

        self.selected_goal_pub = self.create_publisher(
            PoseStamped,
            '/target_explorer/selected_goal',
            10,
        )
        self.candidate_marker_pub = self.create_publisher(
            MarkerArray,
            '/target_explorer/candidate_goals',
            10,
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            self._costmap_callback,
            10,
        )

        self.action_state = ActionState.IDLE
        self._goal_request_id = 0
        self._active_goal_handle = None
        self._active_candidate: Optional[Candidate] = None
        self._active_goal_pose: Optional[PoseStamped] = None
        self._last_active_safety_check = 0.0
        self._active_safety_unavailable_since: Optional[float] = None

        self._selection_active = False
        self._selection_robot_pose: Optional[
            Tuple[float, float, float]
        ] = None
        self._planner_queue: List[Candidate] = []
        self._reachable_candidates: List[Candidate] = []
        self._planner_current: Optional[Candidate] = None
        self._planner_goal_handle = None
        self._planner_request_token = 0
        self._planner_deadline = 0.0
        self._planner_timeout_expired = False
        self._planner_cancel_in_progress = False

        self._target_reached = False
        self._next_selection_time = 0.0
        self._last_safety_error = ''
        self._log_times: Dict[str, float] = {}

        self.manager_timer = self.create_timer(
            self._MANAGER_PERIOD_SEC,
            self._manager_callback,
        )

        self.get_logger().info(
            'target_explorer started: final target=({:.2f}, {:.2f}) '
            'frame={} action={}'.format(
                self.target_x,
                self.target_y,
                self.global_frame,
                self.navigate_to_pose_action,
            )
        )

    def _declare_string(self, name: str, default_value: str) -> str:
        return str(self.declare_parameter(name, default_value).value)

    def _declare_float(self, name: str, default_value: float) -> float:
        return float(self.declare_parameter(name, default_value).value)

    def _declare_int(self, name: str, default_value: int) -> int:
        return int(self.declare_parameter(name, default_value).value)

    def _declare_bool(self, name: str, default_value: bool) -> bool:
        return bool(self.declare_parameter(name, default_value).value)

    def _map_callback(self, message: OccupancyGrid) -> None:
        try:
            self._map_grid = GridSnapshot.from_occupancy_grid(message)
        except (TypeError, ValueError) as error:
            self.get_logger().error(f'Ignoring invalid map: {error}')
            return
        if not self._received_first_map:
            self._received_first_map = True
            self.get_logger().info(
                'Received {}: {}x{} cells at {:.3f} m/cell'.format(
                    self.map_topic,
                    self._map_grid.width,
                    self._map_grid.height,
                    self._map_grid.resolution,
                )
            )

    def _costmap_callback(self, message: OccupancyGrid) -> None:
        try:
            self._costmap_grid = GridSnapshot.from_occupancy_grid(message)
        except (TypeError, ValueError) as error:
            self.get_logger().error(f'Ignoring invalid costmap: {error}')
            return
        if not self._received_first_costmap:
            self._received_first_costmap = True
            self.get_logger().info(
                'Received {}: {}x{} cells at {:.3f} m/cell'.format(
                    self.global_costmap_topic,
                    self._costmap_grid.width,
                    self._costmap_grid.height,
                    self._costmap_grid.resolution,
                )
            )

    def _manager_callback(self) -> None:
        now = time.monotonic()
        if self.action_state == ActionState.ACTIVE:
            self._check_active_goal_safety(now)
            return
        if self.action_state != ActionState.IDLE:
            return
        if self._selection_active:
            self._check_planner_timeout(now)
            return
        if not self._target_reached and now >= self._next_selection_time:
            self._begin_selection()

    def _begin_selection(self) -> None:
        if self.action_state != ActionState.IDLE or self._selection_active:
            return
        if self._map_grid is None:
            self._log_throttled(
                'waiting_map',
                5.0,
                'info',
                f'Waiting for {self.map_topic}.',
            )
            self._defer_selection()
            return
        if not self.planner_client.server_is_ready():
            self._log_throttled(
                'waiting_planner',
                5.0,
                'info',
                'Waiting for ComputePathToPose action server.',
            )
            self._defer_selection()
            return
        if not self.nav_client.server_is_ready():
            self._log_throttled(
                'waiting_navigation',
                5.0,
                'info',
                'Waiting for NavigateToPose action server.',
            )
            self._defer_selection()
            return

        robot_pose = self._robot_pose()
        if robot_pose is None:
            self._defer_selection()
            return
        robot_x, robot_y, robot_yaw = robot_pose
        target_distance = math.hypot(
            self.target_x - robot_x,
            self.target_y - robot_y,
        )
        if target_distance <= self.target_tolerance:
            self._target_reached = True
            self.get_logger().info(
                'Final target reached: robot is {:.2f} m from '
                '({:.2f}, {:.2f}).'.format(
                    target_distance,
                    self.target_x,
                    self.target_y,
                )
            )
            return

        map_robot_point = self._transform_point(
            robot_x,
            robot_y,
            self.global_frame,
            self._map_grid.frame_id,
        )
        if map_robot_point is None:
            self._defer_selection()
            return

        self._selection_active = True
        self._selection_robot_pose = robot_pose
        self._reachable_candidates = []
        candidates: List[Candidate] = []

        final_candidate = make_candidate(
            self.target_x,
            self.target_y,
            robot_x,
            robot_y,
            robot_yaw,
            self.target_x,
            self.target_y,
            self.goal_progress_weight,
            self.target_alignment_weight,
            self.path_cost_weight,
            is_final_target=True,
        )
        final_clearance = self._endpoint_clearance(
            final_candidate.x,
            final_candidate.y,
        )
        if final_clearance.is_safe(self.goal_obstacle_clearance_m):
            candidates.append(final_candidate)
        elif final_clearance.available:
            self._log_endpoint_rejection(
                final_candidate,
                final_clearance,
                'final target',
            )

        search_result = self._frontier_search.search(
            self._map_grid,
            map_robot_point[0],
            map_robot_point[1],
        )
        if search_result.error:
            self._log_throttled(
                'frontier_search_error',
                2.0,
                'warning',
                f'Frontier search unavailable: {search_result.error}.',
            )
        else:
            for frontier in search_result.frontiers:
                option_candidates = []
                for map_x, map_y in frontier.goal_options:
                    point = self._transform_point(
                        map_x,
                        map_y,
                        self._map_grid.frame_id,
                        self.global_frame,
                    )
                    if point is None:
                        continue
                    candidate = make_candidate(
                        point[0],
                        point[1],
                        robot_x,
                        robot_y,
                        robot_yaw,
                        self.target_x,
                        self.target_y,
                        self.goal_progress_weight,
                        self.target_alignment_weight,
                        self.path_cost_weight,
                    )
                    if (
                        candidate.distance_from_robot
                        < self.minimum_frontier_distance_from_robot
                    ):
                        continue
                    clearance = self._endpoint_clearance(
                        candidate.x,
                        candidate.y,
                    )
                    if not clearance.available:
                        continue
                    if not clearance.is_safe(
                        self.goal_obstacle_clearance_m
                    ):
                        self._log_endpoint_rejection(
                            candidate,
                            clearance,
                            'frontier',
                        )
                        continue
                    option_candidates.append(candidate)
                if option_candidates:
                    candidates.append(max(
                        option_candidates,
                        key=lambda candidate: candidate.score,
                    ))

        frontier_candidates = sorted(
            (
                candidate
                for candidate in candidates
                if not candidate.is_final_target
            ),
            key=lambda candidate: candidate.score,
            reverse=True,
        )
        final_candidates = [
            candidate
            for candidate in candidates
            if candidate.is_final_target
        ]
        self._planner_queue = final_candidates + frontier_candidates
        self._publish_candidate_markers(candidates)

        if not self._planner_queue:
            self.get_logger().info(
                'No safe frontier endpoint is currently available.'
            )
            self._finish_selection_without_goal()
            return
        self._plan_next_candidate()

    def _plan_next_candidate(self) -> None:
        if not self._selection_active:
            return
        if not self._planner_queue:
            self._select_reachable_candidate()
            return

        candidate = self._planner_queue.pop(0)
        self._planner_current = candidate
        self._planner_goal_handle = None
        self._planner_timeout_expired = False
        self._planner_cancel_in_progress = False
        self._planner_request_token += 1
        token = self._planner_request_token
        self._planner_deadline = (
            time.monotonic() + max(0.1, self.planner_timeout_sec)
        )

        goal = ComputePathToPose.Goal()
        goal.goal = self._pose_for_candidate(candidate)
        goal.use_start = False
        try:
            future = self.planner_client.send_goal_async(goal)
            future.add_done_callback(
                lambda completed, request_token=token:
                self._planner_goal_response(completed, request_token)
            )
        except Exception as error:  # ROS middleware errors are not uniform.
            self.get_logger().warning(
                'Could not request a path to ({:.2f}, {:.2f}): {}'
                .format(candidate.x, candidate.y, error)
            )
            self._clear_planner_request()
            self._plan_next_candidate()

    def _planner_goal_response(self, future, token: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            if token == self._planner_request_token:
                self.get_logger().warning(
                    f'ComputePathToPose request failed: {error}'
                )
                self._clear_planner_request()
                self._plan_next_candidate()
            return

        if (
            token != self._planner_request_token
            or not self._selection_active
        ):
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._clear_planner_request()
            self._plan_next_candidate()
            return

        self._planner_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, request_token=token:
            self._planner_result(completed, request_token)
        )
        if self._planner_timeout_expired:
            self._request_planner_cancel(token)

    def _planner_result(self, future, token: int) -> None:
        if (
            token != self._planner_request_token
            or not self._selection_active
        ):
            return
        candidate = self._planner_current
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(
                f'ComputePathToPose result failed: {error}'
            )
            response = None

        reachable = False
        path_length = 0.0
        if response is not None and candidate is not None:
            result = response.result
            error_code = int(getattr(result, 'error_code', 0))
            poses = list(result.path.poses)
            reachable = (
                response.status == GoalStatus.STATUS_SUCCEEDED
                and error_code == 0
                and bool(poses)
            )
            if reachable:
                path_length = self._path_length(poses)

        self._clear_planner_request()
        if reachable and candidate is not None:
            candidate.path_length = path_length
            update_score(
                candidate,
                self.goal_progress_weight,
                self.target_alignment_weight,
                self.path_cost_weight,
            )
            if candidate.is_final_target:
                clearance = self._endpoint_clearance(
                    candidate.x,
                    candidate.y,
                )
                if clearance.is_safe(self.goal_obstacle_clearance_m):
                    self.get_logger().info(
                        'Final target is reachable and safe; sending it '
                        'directly.'
                    )
                    self._dispatch_selected_candidate(candidate)
                    return
                if clearance.available:
                    self._log_endpoint_rejection(
                        candidate,
                        clearance,
                        'final target',
                    )
            else:
                self._reachable_candidates.append(candidate)
        self._plan_next_candidate()

    def _check_planner_timeout(self, now: float) -> None:
        if (
            self._planner_current is None
            or now < self._planner_deadline
            or self._planner_timeout_expired
        ):
            return

        self._planner_timeout_expired = True
        candidate = self._planner_current
        self.get_logger().warning(
            'ComputePathToPose timed out for ({:.2f}, {:.2f}) after '
            '{:.2f} s.'.format(
                candidate.x,
                candidate.y,
                self.planner_timeout_sec,
            )
        )
        if self._planner_goal_handle is None:
            return
        self._request_planner_cancel(self._planner_request_token)

    def _request_planner_cancel(self, token: int) -> None:
        if (
            token != self._planner_request_token
            or self._planner_goal_handle is None
            or self._planner_cancel_in_progress
        ):
            return
        self._planner_cancel_in_progress = True
        try:
            future = self._planner_goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed, request_token=token:
                self._planner_cancel_complete(completed, request_token)
            )
        except Exception as error:
            self._planner_cancel_in_progress = False
            self.get_logger().warning(
                f'ComputePathToPose cancellation request failed: {error}'
            )

    def _planner_cancel_complete(self, future, token: int) -> None:
        if (
            token != self._planner_request_token
            or not self._selection_active
        ):
            return
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as error:
            self.get_logger().warning(
                f'ComputePathToPose cancellation response failed: {error}'
            )
            accepted = False
        if not accepted:
            self.get_logger().warning(
                'Nav2 did not accept the timed-out path cancellation; '
                'waiting for its terminal result before continuing.'
            )

    def _select_reachable_candidate(self) -> None:
        remaining = list(self._reachable_candidates)
        while remaining:
            candidate = select_candidate(
                remaining,
                self.minimum_target_progress_m,
                self.allow_negative_progress_recovery,
            )
            if candidate is None:
                break
            clearance = self._endpoint_clearance(
                candidate.x,
                candidate.y,
            )
            if clearance.is_safe(self.goal_obstacle_clearance_m):
                if candidate.progress < self.minimum_target_progress_m:
                    self.get_logger().info(
                        'No forward-progress frontier is reachable; '
                        'using recovery frontier ({:.2f}, {:.2f}).'
                        .format(candidate.x, candidate.y)
                    )
                self._dispatch_selected_candidate(candidate)
                return
            if clearance.available:
                self._log_endpoint_rejection(
                    candidate,
                    clearance,
                    'frontier',
                )
            remaining.remove(candidate)

        self.get_logger().info(
            'Nav2 found no reachable safe frontier candidate.'
        )
        self._finish_selection_without_goal()

    def _dispatch_selected_candidate(self, candidate: Candidate) -> None:
        if self.action_state != ActionState.IDLE:
            self._finish_selection_without_goal()
            return
        clearance = self._endpoint_clearance(candidate.x, candidate.y)
        if not clearance.is_safe(self.goal_obstacle_clearance_m):
            if clearance.available:
                self._log_endpoint_rejection(
                    candidate,
                    clearance,
                    'final target' if candidate.is_final_target
                    else 'frontier',
                )
            self._finish_selection_without_goal()
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warning(
                'NavigateToPose became unavailable before goal dispatch.'
            )
            self._finish_selection_without_goal()
            return

        self._selection_active = False
        self._planner_queue = []
        self._reachable_candidates = []
        self._selection_robot_pose = None
        self._planner_request_token += 1
        self._clear_planner_request()

        pose = self._pose_for_candidate(candidate)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._goal_request_id += 1
        request_id = self._goal_request_id
        self._active_candidate = candidate
        self._active_goal_pose = pose
        self._active_goal_handle = None
        self._active_safety_unavailable_since = None
        self.action_state = ActionState.SENDING

        self.get_logger().info(
            'Sending NavigateToPose goal ({:.2f}, {:.2f}): '
            'progress={:.2f} m alignment={:.3f} path={:.2f} m '
            'score={:.3f}.'.format(
                candidate.x,
                candidate.y,
                candidate.progress,
                candidate.alignment,
                candidate.path_length or 0.0,
                candidate.score,
            )
        )
        self.selected_goal_pub.publish(pose)
        try:
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(
                lambda completed, goal_id=request_id:
                self._navigation_goal_response(completed, goal_id)
            )
        except Exception as error:
            self.get_logger().error(
                f'NavigateToPose send failed: {error}'
            )
            self._return_to_idle()

    def _navigation_goal_response(self, future, request_id: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            if request_id == self._goal_request_id:
                self.get_logger().error(
                    f'NavigateToPose response failed: {error}'
                )
                self._return_to_idle()
            return

        if (
            request_id != self._goal_request_id
            or self.action_state != ActionState.SENDING
        ):
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if goal_handle is None or not goal_handle.accepted:
            candidate = self._active_candidate
            if candidate is not None:
                self.get_logger().warning(
                    'Nav2 rejected goal ({:.2f}, {:.2f}).'.format(
                        candidate.x,
                        candidate.y,
                    )
                )
            self._return_to_idle()
            return

        self._active_goal_handle = goal_handle
        self.action_state = ActionState.ACTIVE
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed, goal_id=request_id:
            self._navigation_result(completed, goal_id)
        )
        candidate = self._active_candidate
        if candidate is not None:
            self.get_logger().info(
                'Nav2 accepted goal ({:.2f}, {:.2f}).'.format(
                    candidate.x,
                    candidate.y,
                )
            )
        self._check_active_goal_safety(time.monotonic(), force=True)

    def _navigation_result(self, future, request_id: int) -> None:
        if request_id != self._goal_request_id:
            return
        try:
            response = future.result()
            status = response.status
        except Exception as error:
            self.get_logger().error(
                f'NavigateToPose result failed: {error}'
            )
            status = GoalStatus.STATUS_UNKNOWN

        candidate = self._active_candidate
        coordinates = (
            'unknown endpoint'
            if candidate is None
            else '({:.2f}, {:.2f})'.format(candidate.x, candidate.y)
        )
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'NavigateToPose succeeded for {coordinates}.'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(
                f'NavigateToPose cancellation completed for {coordinates}.'
            )
        else:
            self.get_logger().warning(
                'NavigateToPose finished for {} with status {}.'.format(
                    coordinates,
                    status,
                )
            )
        self._return_to_idle()

    def _check_active_goal_safety(
        self,
        now: float,
        force: bool = False,
    ) -> None:
        if (
            self.action_state != ActionState.ACTIVE
            or self._active_candidate is None
        ):
            return
        if (
            not force
            and now - self._last_active_safety_check
            < max(0.05, self.active_goal_safety_check_period_sec)
        ):
            return
        self._last_active_safety_check = now
        candidate = self._active_candidate
        clearance = self._endpoint_clearance(candidate.x, candidate.y)
        if not clearance.available:
            if self._active_safety_unavailable_since is None:
                self._active_safety_unavailable_since = now
            elapsed = now - self._active_safety_unavailable_since
            self._log_throttled(
                'active_safety_unavailable',
                1.0,
                'warning',
                'Cannot evaluate active-goal endpoint safety: {} '
                '(unavailable for {:.1f} s).'.format(
                    self._last_safety_error or 'no covering obstacle grid',
                    elapsed,
                ),
            )
            if elapsed >= self.persistent_tf_failure_timeout_sec:
                self.get_logger().warning(
                    'Canceling active goal ({:.2f}, {:.2f}): endpoint '
                    'safety has been unavailable for {:.2f} s.'
                    .format(candidate.x, candidate.y, elapsed)
                )
                self._cancel_active_goal()
            return

        self._active_safety_unavailable_since = None
        if clearance.distance <= self.goal_obstacle_clearance_m:
            self.get_logger().warning(
                'Canceling active goal ({:.2f}, {:.2f}): latest map '
                'places an obstacle {:.2f} m from the endpoint; '
                'required clearance is {:.2f} m.'.format(
                    candidate.x,
                    candidate.y,
                    clearance.distance,
                    self.goal_obstacle_clearance_m,
                )
            )
            self._cancel_active_goal()

    def _cancel_active_goal(self) -> None:
        if (
            self.action_state != ActionState.ACTIVE
            or self._active_goal_handle is None
        ):
            return
        self.action_state = ActionState.CANCELING
        request_id = self._goal_request_id
        try:
            future = self._active_goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda completed, goal_id=request_id:
                self._navigation_cancel_response(completed, goal_id)
            )
        except Exception as error:
            self.get_logger().error(
                f'NavigateToPose cancellation request failed: {error}'
            )
            self.action_state = ActionState.ACTIVE

    def _navigation_cancel_response(self, future, request_id: int) -> None:
        if (
            request_id != self._goal_request_id
            or self.action_state != ActionState.CANCELING
        ):
            return
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as error:
            self.get_logger().error(
                f'NavigateToPose cancellation response failed: {error}'
            )
            accepted = False
        if accepted:
            self.get_logger().info(
                'Nav2 accepted the cancellation; waiting for the action '
                'result before selecting another goal.'
            )
            return

        self.get_logger().warning(
            'Nav2 did not accept the cancellation; the current goal '
            'remains active.'
        )
        self.action_state = ActionState.ACTIVE
        self._last_active_safety_check = 0.0

    def _endpoint_clearance(self, x: float, y: float) -> ClearanceResult:
        checks = []
        errors = []
        for source, grid, threshold in (
            (
                self.map_topic,
                self._map_grid,
                self.map_occupied_threshold,
            ),
            (
                self.global_costmap_topic,
                self._costmap_grid,
                self.costmap_lethal_threshold,
            ),
        ):
            if grid is None:
                continue
            point = self._transform_point(
                x,
                y,
                self.global_frame,
                grid.frame_id,
                log_error=False,
            )
            if point is None:
                errors.append(
                    f'no transform {self.global_frame}->{grid.frame_id}'
                )
                continue
            checks.append((
                source,
                grid,
                point[0],
                point[1],
                threshold,
            ))
        result = combined_clearance(
            checks,
            self.goal_obstacle_clearance_m,
        )
        if result.available:
            self._last_safety_error = ''
        elif errors:
            self._last_safety_error = '; '.join(errors)
        else:
            self._last_safety_error = 'no obstacle grid covers the endpoint'
        return result

    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=max(0.0, self.tf_timeout_sec)),
            )
        except TransformException as error:
            self._log_throttled(
                'robot_tf',
                1.0,
                'warning',
                'Cannot look up robot pose {} -> {}: {}.'.format(
                    self.global_frame,
                    self.robot_base_frame,
                    error,
                ),
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            self._yaw_from_quaternion(rotation),
        )

    def _transform_point(
        self,
        x: float,
        y: float,
        source_frame: str,
        target_frame: str,
        log_error: bool = True,
    ) -> Optional[Tuple[float, float]]:
        source = source_frame.lstrip('/')
        target = target_frame.lstrip('/')
        if not source or not target or source == target:
            return x, y
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=max(0.0, self.tf_timeout_sec)),
            )
        except TransformException as error:
            if log_error:
                self._log_throttled(
                    f'point_tf_{source}_{target}',
                    1.0,
                    'warning',
                    'Cannot transform point {} -> {}: {}.'.format(
                        source_frame,
                        target_frame,
                        error,
                    ),
                )
            return None
        translation = transform.transform.translation
        yaw = self._yaw_from_quaternion(
            transform.transform.rotation
        )
        return (
            float(translation.x) + math.cos(yaw) * x - math.sin(yaw) * y,
            float(translation.y) + math.sin(yaw) * x + math.cos(yaw) * y,
        )

    def _pose_for_candidate(self, candidate: Candidate) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = candidate.x
        pose.pose.position.y = candidate.y
        pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(candidate.theta * 0.5),
            w=math.cos(candidate.theta * 0.5),
        )
        return pose

    def _publish_candidate_markers(
        self,
        candidates: List[Candidate],
    ) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for marker_id, candidate in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = stamp
            marker.ns = 'target_explorer_candidates'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = candidate.x
            marker.pose.position.y = candidate.y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.30
            marker.scale.y = 0.30
            marker.scale.z = 0.30
            marker.color.a = 0.9
            if candidate.is_final_target:
                marker.color.b = 1.0
            elif candidate.progress >= self.minimum_target_progress_m:
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.65
            marker.lifetime = Duration(seconds=2.0).to_msg()
            markers.markers.append(marker)
        self.candidate_marker_pub.publish(markers)

    def _log_endpoint_rejection(
        self,
        candidate: Candidate,
        clearance: ClearanceResult,
        label: str,
    ) -> None:
        key = '{}_{:.1f}_{:.1f}'.format(
            label.replace(' ', '_'),
            candidate.x,
            candidate.y,
        )
        self._log_throttled(
            key,
            2.0,
            'warning',
            'Rejected {} ({:.2f}, {:.2f}): endpoint clearance '
            '{:.2f} m is below required {:.2f} m.'.format(
                label,
                candidate.x,
                candidate.y,
                clearance.distance,
                self.goal_obstacle_clearance_m,
            ),
        )

    def _finish_selection_without_goal(self) -> None:
        self._selection_active = False
        self._selection_robot_pose = None
        self._planner_queue = []
        self._reachable_candidates = []
        self._planner_request_token += 1
        self._clear_planner_request()
        self._defer_selection()

    def _clear_planner_request(self) -> None:
        self._planner_current = None
        self._planner_goal_handle = None
        self._planner_deadline = 0.0
        self._planner_timeout_expired = False
        self._planner_cancel_in_progress = False

    def _return_to_idle(self) -> None:
        self.action_state = ActionState.IDLE
        self._active_goal_handle = None
        self._active_candidate = None
        self._active_goal_pose = None
        self._active_safety_unavailable_since = None
        self._last_active_safety_check = 0.0
        self._next_selection_time = time.monotonic()

    def _defer_selection(self) -> None:
        self._next_selection_time = (
            time.monotonic() + self._SELECTION_RETRY_PERIOD_SEC
        )

    def _log_throttled(
        self,
        key: str,
        period_sec: float,
        level: str,
        message: str,
    ) -> None:
        now = time.monotonic()
        if now - self._log_times.get(key, float('-inf')) < period_sec:
            return
        self._log_times[key] = now
        logger_method = getattr(self.get_logger(), level)
        logger_method(message)

    @staticmethod
    def _path_length(poses) -> float:
        return sum(
            math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
            for first, second in zip(poses, poses[1:])
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
