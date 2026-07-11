import math
from collections import Counter
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class Candidate:
    x: float
    y: float
    theta: float
    requested_distance: float
    key: Tuple[float, float]
    source: str = 'fixed_sample'
    valid: bool = False
    reason: str = ''
    score: float = float('-inf')
    progress: float = 0.0
    alignment: float = 0.0
    step_distance: float = 0.0


@dataclass
class Breadcrumb:
    x: float
    y: float
    sequence: int
    target_distance: float
    selected_goal_key: Tuple[float, float]
    source: str = ''
    progress: float = 0.0
    alignment: float = 0.0
    score: float = 0.0
    recovery_attempted: bool = False
    failed: bool = False


@dataclass
class MemoryCandidate:
    x: float
    y: float
    key: Tuple[float, float]
    source: str
    origin_x: float
    origin_y: float
    origin_sequence: int
    progress_from_origin: float
    alignment_from_origin: float
    score_from_origin: float
    attempted: bool = False
    failed: bool = False
    reached: bool = False


@dataclass
class RayBackoffStats:
    angles_checked: int = 0
    hit_unknown: int = 0
    hit_outside_map: int = 0
    hit_occupied: int = 0
    no_safe_free_cell: int = 0


class TargetExplorerNode(Node):
    """Select safe intermediate map-frame goals and send them to Nav2."""

    def __init__(self):
        super().__init__('target_explorer_node')

        self.global_frame = self._declare_string('global_frame', 'map')
        self.robot_base_frame = self._declare_string(
            'robot_base_frame',
            'base_footprint',
        )
        self.target_x = self._declare_float('target_x', 0.0)
        self.target_y = self._declare_float('target_y', 10.0)
        self.target_tolerance = self._declare_float('target_tolerance', 1.0)

        self.step_min = self._declare_float('step_min', 1.0)
        self.step_max = self._declare_float('step_max', 4.0)
        self.step_resolution = self._declare_float('step_resolution', 0.5)
        self.fan_angle_deg = self._declare_float('fan_angle_deg', 70.0)
        self.fan_angle_step_deg = self._declare_float(
            'fan_angle_step_deg',
            10.0,
        )
        self.use_ray_backoff_goal_generation = self._declare_bool(
            'use_ray_backoff_goal_generation',
            True,
        )
        self.ray_step_resolution = self._declare_float(
            'ray_step_resolution',
            0.05,
        )
        self.min_frontier_goal_distance = self._declare_float(
            'min_frontier_goal_distance',
            0.5,
        )
        self.frontier_backoff_distance = self._declare_float(
            'frontier_backoff_distance',
            0.30,
        )

        self.robot_radius = self._declare_float('robot_radius', 0.60)
        self.safety_margin = self._declare_float('safety_margin', 0.20)
        self.use_global_costmap_check = self._declare_bool(
            'use_global_costmap_check',
            False,
        )
        self.require_positive_progress = self._declare_bool(
            'require_positive_progress',
            True,
        )
        self.goal_reached_distance = self._declare_float(
            'goal_reached_distance',
            0.75,
        )
        self.nav_goal_timeout_sec = self._declare_float(
            'nav_goal_timeout_sec',
            45.0,
        )
        self.enable_backtracking = self._declare_bool(
            'enable_backtracking',
            True,
        )
        self.stuck_attempt_threshold = max(1, self._declare_int(
            'stuck_attempt_threshold',
            3,
        ))
        self.nav_failure_threshold = max(1, self._declare_int(
            'nav_failure_threshold',
            2,
        ))
        self.memory_candidate_min_separation = self._declare_float(
            'memory_candidate_min_separation',
            0.75,
        )
        self.max_memory_candidates = max(0, self._declare_int(
            'max_memory_candidates',
            100,
        ))
        self.max_breadcrumbs = max(0, self._declare_int(
            'max_breadcrumbs',
            100,
        ))
        self.max_branch_candidates_per_cycle = max(0, self._declare_int(
            'max_branch_candidates_per_cycle',
            5,
        ))
        self.allow_negative_progress_in_recovery = self._declare_bool(
            'allow_negative_progress_in_recovery',
            True,
        )
        self.recovery_target_progress_weight = self._declare_float(
            'recovery_target_progress_weight',
            2.0,
        )
        self.recovery_distance_penalty_weight = self._declare_float(
            'recovery_distance_penalty_weight',
            0.5,
        )
        self.recovery_alignment_weight = self._declare_float(
            'recovery_alignment_weight',
            0.5,
        )
        self.recovery_recency_weight = self._declare_float(
            'recovery_recency_weight',
            0.1,
        )
        self.min_recovery_goal_distance = self._declare_float(
            'min_recovery_goal_distance',
            1.0,
        )
        self.visited_goal_radius = self._declare_float(
            'visited_goal_radius',
            0.75,
        )
        self.loop_rate_hz = self._declare_float('loop_rate_hz', 1.0)

        self.map_msg: Optional[OccupancyGrid] = None
        self.costmap_msg: Optional[OccupancyGrid] = None
        self._received_first_map = False
        self._received_first_costmap = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
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
            '/map',
            self._map_callback,
            map_qos,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._costmap_callback,
            10,
        )
        self.odometry_sub = self.create_subscription(
            Odometry,
            '/genz/odometry',
            self._odometry_callback,
            qos_profile_sensor_data,
        )

        self.blacklisted_goals = set()
        self._navigation_active = False
        self._goal_handle = None
        self._active_goal_id: Optional[int] = None
        self._active_goal_key: Optional[Tuple[float, float]] = None
        self._active_goal_pose: Optional[PoseStamped] = None
        self._active_goal_started: Optional[Time] = None
        self._active_goal_progress = 0.0
        self._active_goal_alignment = 0.0
        self._active_goal_score = 0.0
        self._active_goal_source = ''
        self._active_goal_is_recovery = False
        self._active_memory_candidate_key: Optional[Tuple[float, float]] = None
        self._active_breadcrumb_sequence: Optional[int] = None
        self._goal_sequence = 0
        self._memory_sequence = 0
        self._breadcrumb_sequence = 0
        self._consecutive_no_goal_cycles = 0
        self._consecutive_nav_failures = 0
        self._recovery_mode = False
        self._target_reached = False
        self.memory_candidates: List[MemoryCandidate] = []
        self.breadcrumbs: List[Breadcrumb] = []

        timer_period = 1.0 / max(self.loop_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            'target_explorer_node started: target=({:.2f}, {:.2f}) frame={}'
            .format(self.target_x, self.target_y, self.global_frame)
        )

    def _declare_string(self, name: str, default_value: str) -> str:
        return str(self.declare_parameter(name, default_value).value)

    def _declare_float(self, name: str, default_value: float) -> float:
        return float(self.declare_parameter(name, default_value).value)

    def _declare_int(self, name: str, default_value: int) -> int:
        return int(self.declare_parameter(name, default_value).value)

    def _declare_bool(self, name: str, default_value: bool) -> bool:
        return bool(self.declare_parameter(name, default_value).value)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        if not self._received_first_map:
            self._received_first_map = True
            self.get_logger().info(
                'Received /map: {}x{} cells, resolution {:.3f} m/cell'
                .format(msg.info.width, msg.info.height, msg.info.resolution)
            )

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        self.costmap_msg = msg
        if not self._received_first_costmap:
            self._received_first_costmap = True
            self.get_logger().info(
                'Received /global_costmap/costmap: {}x{} cells'
                .format(msg.info.width, msg.info.height)
            )

    def _odometry_callback(self, _msg: Odometry) -> None:
        """Advance from an intermediate goal as soon as the robot is near it."""
        if (
            not self._navigation_active
            or self._active_goal_pose is None
            or self.goal_reached_distance <= 0.0
        ):
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y = robot_pose
        goal_position = self._active_goal_pose.pose.position
        goal_distance = self._distance(
            robot_x,
            robot_y,
            goal_position.x,
            goal_position.y,
        )
        if goal_distance > self.goal_reached_distance:
            return

        was_recovery = self._active_goal_is_recovery
        self._store_breadcrumb_from_active_goal()
        self._consecutive_nav_failures = 0
        if was_recovery:
            self._mark_active_recovery_reached()
            self._consecutive_no_goal_cycles = 0
            self._recovery_mode = False

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        self.get_logger().info(
            'Robot is {:.2f} m from intermediate goal (threshold {:.2f} m); '
            'selecting the next goal early.'
            .format(goal_distance, self.goal_reached_distance)
        )
        self._clear_active_navigation()
        self._timer_callback()

    def _timer_callback(self) -> None:
        if self._target_reached:
            return

        if self.map_msg is None:
            self.get_logger().warn(
                'Waiting for /map before selecting exploration goals.',
                throttle_duration_sec=5.0,
            )
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            return

        robot_x, robot_y = robot_pose
        target_distance = self._distance(robot_x, robot_y,
                                         self.target_x, self.target_y)

        if target_distance < self.target_tolerance:
            self._handle_target_reached(robot_x, robot_y, target_distance)
            return

        if self._navigation_active:
            self._check_navigation_timeout()
            return

        candidates, ray_stats = self._evaluate_candidates(
            robot_x,
            robot_y,
            self.fan_angle_deg,
        )
        selected = self._select_best_candidate(candidates)

        expanded = False
        if selected is None and self.fan_angle_deg < 120.0:
            expanded = True
            candidates, ray_stats = self._evaluate_candidates(
                robot_x,
                robot_y,
                120.0,
            )
            selected = self._select_best_candidate(candidates)

        if selected is None:
            self._consecutive_no_goal_cycles += 1
            reason_summary = self._summarize_rejections(candidates)
            expansion_text = ' after expanding fan to 120 deg' if expanded else ''
            if self.use_ray_backoff_goal_generation:
                ray_summary = self._summarize_ray_stats(ray_stats)
                self.get_logger().warn(
                    'No valid intermediate goal{}; retrying. Ray backoff: {}. '
                    'Rejections: {}. no_goal_cycles={}'
                    .format(
                        expansion_text,
                        ray_summary,
                        reason_summary,
                        self._consecutive_no_goal_cycles,
                    ),
                    throttle_duration_sec=5.0,
                )
            else:
                self.get_logger().warn(
                    'No valid intermediate goal{}; retrying. Rejections: {}. '
                    'no_goal_cycles={}'
                    .format(
                        expansion_text,
                        reason_summary,
                        self._consecutive_no_goal_cycles,
                    ),
                    throttle_duration_sec=5.0,
                )
            self._maybe_enter_recovery_mode()
            recovery = self._select_recovery_candidate(robot_x, robot_y)
            if self._recovery_mode and recovery is not None:
                recovery_candidate, memory_candidate, breadcrumb = recovery
                self._publish_candidate_markers(
                    candidates,
                    None,
                    recovery_candidate,
                )
                goal_pose = self._candidate_to_pose(recovery_candidate)
                self.selected_goal_pub.publish(goal_pose)
                self.get_logger().info(
                    'Recovery selected: C=({:.2f}, {:.2f}), source={}, '
                    'dist_from_robot={:.2f}, target_progress={:.2f}, '
                    'alignment={:.2f}, score={:.2f}'
                    .format(
                        recovery_candidate.x,
                        recovery_candidate.y,
                        recovery_candidate.source,
                        recovery_candidate.step_distance,
                        recovery_candidate.progress,
                        recovery_candidate.alignment,
                        recovery_candidate.score,
                    )
                )
                sent = self._send_nav_goal(
                    goal_pose,
                    recovery_candidate,
                    is_recovery=True,
                    memory_candidate_key=(
                        memory_candidate.key
                        if memory_candidate is not None else None
                    ),
                    breadcrumb_sequence=(
                        breadcrumb.sequence
                        if breadcrumb is not None else None
                    ),
                )
                if sent:
                    if memory_candidate is not None:
                        memory_candidate.attempted = True
                    if breadcrumb is not None:
                        breadcrumb.recovery_attempted = True
                return

            self._publish_candidate_markers(candidates, None)
            if self._recovery_mode:
                self.get_logger().warn(
                    'Stuck and no valid recovery candidates available. '
                    'Continuing to retry local search.',
                    throttle_duration_sec=5.0,
                )
            return

        if self._recovery_mode:
            self.get_logger().info(
                'Normal local goal available again; returning to '
                'target-directed exploration.'
            )
            self._recovery_mode = False

        self._consecutive_no_goal_cycles = 0
        self._store_branch_candidates(candidates, selected, robot_x, robot_y)
        self._publish_candidate_markers(candidates, selected)

        goal_pose = self._candidate_to_pose(selected)
        self.selected_goal_pub.publish(goal_pose)

        self.get_logger().info(
            'Selected goal: R=({:.2f}, {:.2f}), T=({:.2f}, {:.2f}), '
            'C=({:.2f}, {:.2f}), step={:.2f}, progress={:.2f}, '
            'alignment={:.2f}, score={:.2f}, source={}, '
            'global_costmap_check={}'
            .format(
                robot_x,
                robot_y,
                self.target_x,
                self.target_y,
                selected.x,
                selected.y,
                selected.step_distance,
                selected.progress,
                selected.alignment,
                selected.score,
                selected.source,
                str(self.use_global_costmap_check).lower(),
            )
        )

        self._send_nav_goal(goal_pose, selected)

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warn(
                'Waiting for TF {} -> {}: {}'
                .format(self.global_frame, self.robot_base_frame, exc),
                throttle_duration_sec=5.0,
            )
            return None

        translation = transform.transform.translation
        return translation.x, translation.y

    def _handle_target_reached(
        self,
        robot_x: float,
        robot_y: float,
        target_distance: float,
    ) -> None:
        if self._navigation_active and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._clear_active_navigation()
        self._target_reached = True
        self.get_logger().info(
            'Final target reached: robot=({:.2f}, {:.2f}) target=({:.2f}, '
            '{:.2f}) distance={:.2f} m'
            .format(robot_x, robot_y, self.target_x, self.target_y,
                    target_distance)
        )

    def _check_navigation_timeout(self) -> None:
        if self._active_goal_started is None or self.nav_goal_timeout_sec <= 0.0:
            return

        elapsed = self.get_clock().now() - self._active_goal_started
        if elapsed.nanoseconds <= self.nav_goal_timeout_sec * 1e9:
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._handle_active_navigation_failure('timed out')

    def _evaluate_candidates(
        self,
        robot_x: float,
        robot_y: float,
        fan_angle_deg: float,
    ) -> Tuple[List[Candidate], RayBackoffStats]:
        candidates: List[Candidate] = []
        ray_stats = RayBackoffStats()

        if self.use_ray_backoff_goal_generation:
            ray_candidates, ray_stats = self._evaluate_ray_backoff_candidates(
                robot_x,
                robot_y,
                fan_angle_deg,
            )
            candidates.extend(ray_candidates)
            if self._select_best_candidate(ray_candidates) is not None:
                return candidates, ray_stats

        candidates.extend(
            self._evaluate_fixed_sample_candidates(
                robot_x,
                robot_y,
                fan_angle_deg,
            )
        )
        return candidates, ray_stats

    def _evaluate_fixed_sample_candidates(
        self,
        robot_x: float,
        robot_y: float,
        fan_angle_deg: float,
    ) -> List[Candidate]:
        target_distance = self._distance(robot_x, robot_y,
                                         self.target_x, self.target_y)
        theta_target = math.atan2(self.target_y - robot_y,
                                  self.target_x - robot_x)
        target_unit_x = math.cos(theta_target)
        target_unit_y = math.sin(theta_target)
        clearance_radius = self.robot_radius + self.safety_margin

        distances = self._sample_range(
            max(0.0, self.step_min),
            max(self.step_min, self.step_max),
            abs(self.step_resolution),
        )

        candidates: List[Candidate] = []
        for offset_deg in self._sample_angle_offsets(fan_angle_deg):
            theta = theta_target + math.radians(offset_deg)
            for requested_distance in distances:
                x = robot_x + requested_distance * math.cos(theta)
                y = robot_y + requested_distance * math.sin(theta)
                candidates.append(self._make_candidate(
                    x=x,
                    y=y,
                    theta=theta,
                    requested_distance=requested_distance,
                    source='fixed_sample',
                    robot_x=robot_x,
                    robot_y=robot_y,
                    target_distance=target_distance,
                    target_unit_x=target_unit_x,
                    target_unit_y=target_unit_y,
                    clearance_radius=clearance_radius,
                ))

        return candidates

    def _evaluate_ray_backoff_candidates(
        self,
        robot_x: float,
        robot_y: float,
        fan_angle_deg: float,
    ) -> Tuple[List[Candidate], RayBackoffStats]:
        target_distance = self._distance(robot_x, robot_y,
                                         self.target_x, self.target_y)
        theta_target = math.atan2(self.target_y - robot_y,
                                  self.target_x - robot_x)
        target_unit_x = math.cos(theta_target)
        target_unit_y = math.sin(theta_target)
        clearance_radius = self.robot_radius + self.safety_margin
        ray_step = self._effective_ray_step_resolution()
        max_ray_distance = max(self.step_max, self.min_frontier_goal_distance)
        stats = RayBackoffStats()
        candidates: List[Candidate] = []

        for offset_deg in self._sample_angle_offsets(fan_angle_deg):
            stats.angles_checked += 1
            theta = theta_target + math.radians(offset_deg)
            stop_distance: Optional[float] = None
            last_safe_distance: Optional[float] = None
            distance = ray_step

            while distance <= max_ray_distance + ray_step * 1e-6:
                x = robot_x + distance * math.cos(theta)
                y = robot_y + distance * math.sin(theta)
                cell_state = self._map_cell_state(x, y)

                if cell_state == 'outside_map':
                    stats.hit_outside_map += 1
                    stop_distance = distance
                    break
                if cell_state == 'unknown':
                    stats.hit_unknown += 1
                    stop_distance = distance
                    break
                if cell_state == 'occupied':
                    stats.hit_occupied += 1
                    stop_distance = distance
                    break

                if not self.is_occupied_nearby(x, y, clearance_radius):
                    last_safe_distance = distance

                distance += ray_step

            candidate = self._candidate_from_ray_backoff(
                robot_x=robot_x,
                robot_y=robot_y,
                theta=theta,
                stop_distance=stop_distance,
                last_safe_distance=last_safe_distance,
                ray_step=ray_step,
                target_distance=target_distance,
                target_unit_x=target_unit_x,
                target_unit_y=target_unit_y,
                clearance_radius=clearance_radius,
            )
            if candidate is None:
                stats.no_safe_free_cell += 1
                continue

            candidates.append(candidate)

        return candidates, stats

    def _candidate_from_ray_backoff(
        self,
        robot_x: float,
        robot_y: float,
        theta: float,
        stop_distance: Optional[float],
        last_safe_distance: Optional[float],
        ray_step: float,
        target_distance: float,
        target_unit_x: float,
        target_unit_y: float,
        clearance_radius: float,
    ) -> Optional[Candidate]:
        if last_safe_distance is None:
            return None

        candidate_distance = last_safe_distance
        if stop_distance is not None:
            backoff_distance = max(0.0, self.frontier_backoff_distance)
            candidate_distance = min(
                last_safe_distance,
                max(0.0, stop_distance - backoff_distance),
            )

        if candidate_distance < self.min_frontier_goal_distance:
            x = robot_x + last_safe_distance * math.cos(theta)
            y = robot_y + last_safe_distance * math.sin(theta)
            candidate = self._make_candidate(
                x=x,
                y=y,
                theta=theta,
                requested_distance=last_safe_distance,
                source='ray_backoff',
                robot_x=robot_x,
                robot_y=robot_y,
                target_distance=target_distance,
                target_unit_x=target_unit_x,
                target_unit_y=target_unit_y,
                clearance_radius=clearance_radius,
            )
            candidate.valid = False
            candidate.reason = 'too_close'
            candidate.score = float('-inf')
            return candidate

        best_invalid: Optional[Candidate] = None
        search_distance = min(candidate_distance, last_safe_distance)
        while search_distance >= self.min_frontier_goal_distance - ray_step * 1e-6:
            x = robot_x + search_distance * math.cos(theta)
            y = robot_y + search_distance * math.sin(theta)
            candidate = self._make_candidate(
                x=x,
                y=y,
                theta=theta,
                requested_distance=search_distance,
                source='ray_backoff',
                robot_x=robot_x,
                robot_y=robot_y,
                target_distance=target_distance,
                target_unit_x=target_unit_x,
                target_unit_y=target_unit_y,
                clearance_radius=clearance_radius,
            )
            if candidate.valid:
                return candidate
            if best_invalid is None:
                best_invalid = candidate
            if candidate.reason == 'no_progress':
                return candidate
            search_distance -= ray_step

        return best_invalid

    def _make_candidate(
        self,
        x: float,
        y: float,
        theta: float,
        requested_distance: float,
        source: str,
        robot_x: float,
        robot_y: float,
        target_distance: float,
        target_unit_x: float,
        target_unit_y: float,
        clearance_radius: float,
        require_positive_progress: Optional[bool] = None,
    ) -> Candidate:
        candidate = Candidate(
            x=x,
            y=y,
            theta=theta,
            requested_distance=requested_distance,
            key=self._candidate_key(x, y),
            source=source,
        )

        candidate.step_distance = self._distance(robot_x, robot_y, x, y)
        candidate.progress = (
            target_distance
            - self._distance(x, y, self.target_x, self.target_y)
        )
        if candidate.step_distance > 1e-6 and target_distance > 1e-6:
            move_unit_x = (x - robot_x) / candidate.step_distance
            move_unit_y = (y - robot_y) / candidate.step_distance
            candidate.alignment = (
                move_unit_x * target_unit_x
                + move_unit_y * target_unit_y
            )

        reject_reason = self._candidate_reject_reason(
            candidate,
            clearance_radius,
            require_positive_progress=require_positive_progress,
        )
        if reject_reason is None:
            candidate.valid = True
            candidate.score = (
                3.0 * candidate.progress
                + 1.0 * candidate.alignment
                + 0.2 * candidate.step_distance
            )
        else:
            candidate.reason = reject_reason

        return candidate

    def _candidate_reject_reason(
        self,
        candidate: Candidate,
        clearance_radius: float,
        require_positive_progress: Optional[bool] = None,
    ) -> Optional[str]:
        if require_positive_progress is None:
            require_positive_progress = self.require_positive_progress

        if self.world_to_map(candidate.x, candidate.y) is None:
            return 'outside_map'

        if not self.is_known_free(candidate.x, candidate.y):
            return 'not_known_free'

        if self.is_occupied_nearby(
            candidate.x,
            candidate.y,
            clearance_radius,
        ):
            return 'occupied_clearance'

        if (self.use_global_costmap_check
                and not self.global_costmap_allows(candidate.x, candidate.y)):
            return 'global_costmap_reject'

        if require_positive_progress and candidate.progress <= 0.0:
            return 'no_progress'

        if candidate.key in self.blacklisted_goals:
            return 'blacklisted'

        return None

    def _select_best_candidate(
        self,
        candidates: List[Candidate],
    ) -> Optional[Candidate]:
        valid_candidates = [candidate for candidate in candidates
                            if candidate.valid]
        if not valid_candidates:
            return None
        return max(valid_candidates, key=lambda candidate: candidate.score)

    def _store_branch_candidates(
        self,
        candidates: List[Candidate],
        selected: Candidate,
        robot_x: float,
        robot_y: float,
    ) -> None:
        if (not self.enable_backtracking
                or self.max_memory_candidates <= 0
                or self.max_branch_candidates_per_cycle <= 0):
            return

        valid_unselected = [
            candidate for candidate in candidates
            if candidate.valid and candidate.key != selected.key
        ]
        valid_unselected.sort(
            key=lambda candidate: candidate.score,
            reverse=True,
        )

        stored = 0
        for candidate in valid_unselected:
            if stored >= self.max_branch_candidates_per_cycle:
                break
            if not self._should_store_memory_candidate(candidate, selected):
                continue

            self._memory_sequence += 1
            self.memory_candidates.append(MemoryCandidate(
                x=candidate.x,
                y=candidate.y,
                key=candidate.key,
                source=candidate.source,
                origin_x=robot_x,
                origin_y=robot_y,
                origin_sequence=self._memory_sequence,
                progress_from_origin=candidate.progress,
                alignment_from_origin=candidate.alignment,
                score_from_origin=candidate.score,
            ))
            stored += 1

        if stored > 0:
            self._trim_memory_candidates()
            self.get_logger().info(
                'Stored {} branch candidates from R=({:.2f}, {:.2f}). '
                'memory_size={}'
                .format(stored, robot_x, robot_y, len(self.memory_candidates))
            )

    def _should_store_memory_candidate(
        self,
        candidate: Candidate,
        selected: Candidate,
    ) -> bool:
        min_separation = max(0.0, self.memory_candidate_min_separation)

        if self._distance(candidate.x, candidate.y,
                          selected.x, selected.y) < min_separation:
            return False
        if candidate.key in self.blacklisted_goals:
            return False
        if self._is_near_blacklisted_goal(candidate.x, candidate.y):
            return False
        if self._is_near_existing_memory(candidate.x, candidate.y):
            return False
        if self._is_near_visited_breadcrumb(candidate.x, candidate.y):
            return False

        return True

    def _is_near_existing_memory(self, x: float, y: float) -> bool:
        min_separation = max(0.0, self.memory_candidate_min_separation)
        for memory_candidate in self.memory_candidates:
            if self._distance(x, y, memory_candidate.x,
                              memory_candidate.y) < min_separation:
                return True
        return False

    def _is_near_blacklisted_goal(self, x: float, y: float) -> bool:
        min_separation = max(0.0, self.memory_candidate_min_separation)
        for goal_x, goal_y in self.blacklisted_goals:
            if self._distance(x, y, goal_x, goal_y) < min_separation:
                return True
        return False

    def _is_near_visited_breadcrumb(self, x: float, y: float) -> bool:
        radius = max(0.0, self.visited_goal_radius)
        for breadcrumb in self.breadcrumbs:
            if self._distance(x, y, breadcrumb.x, breadcrumb.y) < radius:
                return True
            goal_x, goal_y = breadcrumb.selected_goal_key
            if self._distance(x, y, goal_x, goal_y) < radius:
                return True
        return False

    def _trim_memory_candidates(self) -> None:
        if self.max_memory_candidates <= 0:
            self.memory_candidates.clear()
            return
        overflow = len(self.memory_candidates) - self.max_memory_candidates
        if overflow > 0:
            del self.memory_candidates[:overflow]

    def _trim_breadcrumbs(self) -> None:
        if self.max_breadcrumbs <= 0:
            self.breadcrumbs.clear()
            return
        overflow = len(self.breadcrumbs) - self.max_breadcrumbs
        if overflow > 0:
            del self.breadcrumbs[:overflow]

    def _maybe_enter_recovery_mode(self) -> bool:
        if not self.enable_backtracking:
            return False

        should_recover = (
            self._consecutive_no_goal_cycles >= self.stuck_attempt_threshold
            or self._consecutive_nav_failures >= self.nav_failure_threshold
        )
        if not should_recover:
            return False

        if not self._recovery_mode:
            self.get_logger().warn(
                'Entering recovery mode: no_goal_cycles={}, nav_failures={}, '
                'memory_candidates={}, breadcrumbs={}'
                .format(
                    self._consecutive_no_goal_cycles,
                    self._consecutive_nav_failures,
                    len(self.memory_candidates),
                    len(self.breadcrumbs),
                )
            )
        self._recovery_mode = True
        return True

    def _select_recovery_candidate(
        self,
        robot_x: float,
        robot_y: float,
    ) -> Optional[Tuple[Candidate, Optional[MemoryCandidate],
                       Optional[Breadcrumb]]]:
        if not self.enable_backtracking or not self._recovery_mode:
            return None

        options: List[Tuple[Candidate, Optional[MemoryCandidate],
                            Optional[Breadcrumb]]] = []

        for memory_candidate in self.memory_candidates:
            candidate = self._memory_candidate_to_candidate(
                memory_candidate,
                robot_x,
                robot_y,
            )
            if candidate is None or not candidate.valid:
                continue
            options.append((candidate, memory_candidate, None))

        for breadcrumb in self.breadcrumbs:
            candidate = self._breadcrumb_to_candidate(
                breadcrumb,
                robot_x,
                robot_y,
            )
            if candidate is None or not candidate.valid:
                continue
            options.append((candidate, None, breadcrumb))

        if not options:
            return None

        return max(options, key=lambda option: option[0].score)

    def _memory_candidate_to_candidate(
        self,
        memory_candidate: MemoryCandidate,
        robot_x: float,
        robot_y: float,
    ) -> Optional[Candidate]:
        if (memory_candidate.failed
                or memory_candidate.attempted
                or memory_candidate.reached):
            return None
        if self._is_near_visited_breadcrumb(
                memory_candidate.x,
                memory_candidate.y):
            return None

        candidate = self._recovery_pose_to_candidate(
            x=memory_candidate.x,
            y=memory_candidate.y,
            source='memory_{}'.format(memory_candidate.source),
            robot_x=robot_x,
            robot_y=robot_y,
            recency_sequence=memory_candidate.origin_sequence,
        )
        return candidate

    def _breadcrumb_to_candidate(
        self,
        breadcrumb: Breadcrumb,
        robot_x: float,
        robot_y: float,
    ) -> Optional[Candidate]:
        if breadcrumb.failed or breadcrumb.recovery_attempted:
            return None

        return self._recovery_pose_to_candidate(
            x=breadcrumb.x,
            y=breadcrumb.y,
            source='breadcrumb',
            robot_x=robot_x,
            robot_y=robot_y,
            recency_sequence=breadcrumb.sequence,
        )

    def _recovery_pose_to_candidate(
        self,
        x: float,
        y: float,
        source: str,
        robot_x: float,
        robot_y: float,
        recency_sequence: int,
    ) -> Optional[Candidate]:
        if self._is_near_blacklisted_goal(x, y):
            return None

        target_distance = self._distance(robot_x, robot_y,
                                         self.target_x, self.target_y)
        theta = math.atan2(y - robot_y, x - robot_x)
        target_theta = math.atan2(self.target_y - robot_y,
                                  self.target_x - robot_x)
        target_unit_x = math.cos(target_theta)
        target_unit_y = math.sin(target_theta)
        clearance_radius = self.robot_radius + self.safety_margin
        require_positive_progress = (
            not self.allow_negative_progress_in_recovery
        )

        candidate = self._make_candidate(
            x=x,
            y=y,
            theta=theta,
            requested_distance=self._distance(robot_x, robot_y, x, y),
            source=source,
            robot_x=robot_x,
            robot_y=robot_y,
            target_distance=target_distance,
            target_unit_x=target_unit_x,
            target_unit_y=target_unit_y,
            clearance_radius=clearance_radius,
            require_positive_progress=require_positive_progress,
        )
        if not candidate.valid:
            return candidate

        if candidate.step_distance < max(0.0, self.min_recovery_goal_distance):
            candidate.valid = False
            candidate.reason = 'too_close_recovery'
            candidate.score = float('-inf')
            return candidate

        candidate.score = self._recovery_score(candidate, recency_sequence)
        return candidate

    def _recovery_score(
        self,
        candidate: Candidate,
        recency_sequence: int,
    ) -> float:
        max_sequence = max(self._memory_sequence, self._breadcrumb_sequence, 1)
        recency = max(0.0, min(1.0, recency_sequence / max_sequence))
        return (
            self.recovery_target_progress_weight * candidate.progress
            - self.recovery_distance_penalty_weight * candidate.step_distance
            + self.recovery_alignment_weight * candidate.alignment
            + self.recovery_recency_weight * recency
        )

    def _send_nav_goal(
        self,
        goal_pose: PoseStamped,
        candidate: Candidate,
        is_recovery: bool = False,
        memory_candidate_key: Optional[Tuple[float, float]] = None,
        breadcrumb_sequence: Optional[int] = None,
    ) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                'Waiting for /navigate_to_pose action server.',
                throttle_duration_sec=5.0,
            )
            return False

        nav_goal_pose = self._make_nav_goal_pose(goal_pose)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = nav_goal_pose

        self.get_logger().info(
            'Sending Nav2 goal with stamp=0/latest frame={} x={:.2f} y={:.2f}'
            .format(
                nav_goal_pose.header.frame_id,
                nav_goal_pose.pose.position.x,
                nav_goal_pose.pose.position.y,
            )
        )

        self._goal_sequence += 1
        goal_id = self._goal_sequence
        self._active_goal_id = goal_id
        self._active_goal_key = candidate.key
        self._active_goal_pose = nav_goal_pose
        self._active_goal_started = self.get_clock().now()
        self._active_goal_progress = candidate.progress
        self._active_goal_alignment = candidate.alignment
        self._active_goal_score = candidate.score
        self._active_goal_source = candidate.source
        self._active_goal_is_recovery = is_recovery
        self._active_memory_candidate_key = memory_candidate_key
        self._active_breadcrumb_sequence = breadcrumb_sequence
        self._navigation_active = True
        self._goal_handle = None

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done_future: self._goal_response_callback(
                done_future,
                goal_id,
            )
        )
        return True

    def _goal_response_callback(self, future, goal_id: int) -> None:
        if goal_id != self._active_goal_id or not self._navigation_active:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                'Failed to send NavigateToPose goal: {}'.format(exc)
            )
            self._handle_active_navigation_failure('send failed')
            return

        if not goal_handle.accepted:
            self._handle_active_navigation_failure('rejected by Nav2')
            return

        self._goal_handle = goal_handle
        self.get_logger().info(
            'Nav2 accepted intermediate goal ({:.2f}, {:.2f}).'
            .format(
                self._active_goal_pose.pose.position.x,
                self._active_goal_pose.pose.position.y,
            )
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self._nav_result_callback(
                done_future,
                goal_id,
            )
        )

    def _nav_result_callback(self, future, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            return

        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                'NavigateToPose result failed: {}'.format(exc)
            )
            self._handle_active_navigation_failure('result failed')
            return

        status = wrapped_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            was_recovery = self._active_goal_is_recovery
            self._store_breadcrumb_from_active_goal()
            self._consecutive_nav_failures = 0
            if was_recovery:
                self._mark_active_recovery_reached()
                self._consecutive_no_goal_cycles = 0
                self._recovery_mode = False
                self.get_logger().info(
                    'Recovery goal reached; returning to normal '
                    'target-directed exploration.'
                )
            self.get_logger().info(
                'Nav2 reached intermediate goal; selecting next goal.'
            )
            self._clear_active_navigation()
            return

        status_name = self._goal_status_name(status)
        self.get_logger().warn(
            'Nav2 did not reach intermediate goal: status={}.'.format(
                status_name,
            )
        )
        self._handle_active_navigation_failure(
            'Nav2 status {}'.format(status_name)
        )

    def _handle_active_navigation_failure(self, reason: str) -> None:
        self._consecutive_nav_failures += 1
        self._blacklist_active_goal(reason)
        self._maybe_enter_recovery_mode()
        self._clear_active_navigation()

    def _store_breadcrumb_from_active_goal(self) -> None:
        if self.max_breadcrumbs <= 0 or self._active_goal_pose is None:
            return

        robot_pose = self._lookup_robot_pose()
        if robot_pose is None:
            position = self._active_goal_pose.pose.position
            robot_x = position.x
            robot_y = position.y
        else:
            robot_x, robot_y = robot_pose

        self._breadcrumb_sequence += 1
        target_distance = self._distance(
            robot_x,
            robot_y,
            self.target_x,
            self.target_y,
        )
        selected_goal_key = (
            self._active_goal_key
            if self._active_goal_key is not None
            else self._candidate_key(robot_x, robot_y)
        )

        self.breadcrumbs.append(Breadcrumb(
            x=robot_x,
            y=robot_y,
            sequence=self._breadcrumb_sequence,
            target_distance=target_distance,
            selected_goal_key=selected_goal_key,
            source=self._active_goal_source,
            progress=self._active_goal_progress,
            alignment=self._active_goal_alignment,
            score=self._active_goal_score,
        ))
        self._trim_breadcrumbs()

    def _mark_active_recovery_reached(self) -> None:
        if self._active_memory_candidate_key is not None:
            for memory_candidate in self.memory_candidates:
                if memory_candidate.key == self._active_memory_candidate_key:
                    memory_candidate.reached = True
                    break

    def _mark_memory_candidate_failed_by_key(
        self,
        key: Tuple[float, float],
    ) -> None:
        for memory_candidate in self.memory_candidates:
            if memory_candidate.key != key or memory_candidate.failed:
                continue
            memory_candidate.failed = True
            self.get_logger().warn(
                'Recovery candidate failed; marked failed and blacklisted: '
                'C=({:.2f}, {:.2f})'
                .format(memory_candidate.x, memory_candidate.y)
            )
            break

    def _mark_active_recovery_failed(self) -> None:
        if self._active_memory_candidate_key is not None:
            self._mark_memory_candidate_failed_by_key(
                self._active_memory_candidate_key
            )

        if self._active_breadcrumb_sequence is not None:
            for breadcrumb in self.breadcrumbs:
                if breadcrumb.sequence == self._active_breadcrumb_sequence:
                    breadcrumb.failed = True
                    break

    def _blacklist_active_goal(self, reason: str) -> None:
        if self._active_goal_key is not None:
            self.blacklisted_goals.add(self._active_goal_key)
            self._mark_memory_candidate_failed_by_key(self._active_goal_key)
        if self._active_goal_is_recovery:
            self._mark_active_recovery_failed()

        if self._active_goal_pose is None:
            self.get_logger().warn(
                'Blacklisted active goal because it {}.'.format(reason)
            )
            return

        position = self._active_goal_pose.pose.position
        self.get_logger().warn(
            'Blacklisted intermediate goal ({:.2f}, {:.2f}) because it {}.'
            .format(position.x, position.y, reason)
        )

    def _clear_active_navigation(self) -> None:
        self._navigation_active = False
        self._goal_handle = None
        self._active_goal_id = None
        self._active_goal_key = None
        self._active_goal_pose = None
        self._active_goal_started = None
        self._active_goal_progress = 0.0
        self._active_goal_alignment = 0.0
        self._active_goal_score = 0.0
        self._active_goal_source = ''
        self._active_goal_is_recovery = False
        self._active_memory_candidate_key = None
        self._active_breadcrumb_sequence = None

    def world_to_map(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        return self._world_to_grid(self.map_msg, x, y)

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        if self.map_msg is None:
            raise RuntimeError('map_to_world called before /map was received')
        return self._grid_to_world(self.map_msg, mx, my)

    def is_known_free(self, x: float, y: float) -> bool:
        coords = self.world_to_map(x, y)
        if coords is None or self.map_msg is None:
            return False
        value = self._grid_value(self.map_msg, coords[0], coords[1])
        return value == 0

    def _map_cell_state(self, x: float, y: float) -> str:
        coords = self.world_to_map(x, y)
        if coords is None or self.map_msg is None:
            return 'outside_map'

        value = self._grid_value(self.map_msg, coords[0], coords[1])
        if value < 0:
            return 'unknown'
        if value == 0:
            return 'free'
        return 'occupied'

    def is_occupied_nearby(
        self,
        x: float,
        y: float,
        radius: float,
    ) -> bool:
        if self.map_msg is None:
            return True

        coords = self.world_to_map(x, y)
        if coords is None:
            return True

        mx, my = coords
        resolution = self.map_msg.info.resolution
        radius_cells = int(math.ceil(radius / resolution))

        for iy in range(my - radius_cells, my + radius_cells + 1):
            for ix in range(mx - radius_cells, mx + radius_cells + 1):
                if not self._grid_contains(self.map_msg, ix, iy):
                    continue
                cell_x, cell_y = self._grid_to_world(self.map_msg, ix, iy)
                if self._distance(x, y, cell_x, cell_y) > radius:
                    continue
                if self._grid_value(self.map_msg, ix, iy) > 0:
                    return True

        return False

    def global_costmap_allows(self, x: float, y: float) -> bool:
        if not self.use_global_costmap_check:
            return True

        if self.costmap_msg is None:
            return True

        coords = self._world_to_grid(self.costmap_msg, x, y)
        if coords is None:
            return False

        value = self._grid_value(self.costmap_msg, coords[0], coords[1])
        return 0 <= value < 80

    def _world_to_grid(
        self,
        grid: Optional[OccupancyGrid],
        x: float,
        y: float,
    ) -> Optional[Tuple[int, int]]:
        if grid is None or grid.info.resolution <= 0.0:
            return None

        origin = grid.info.origin
        yaw = self._quaternion_to_yaw(origin.orientation)
        dx = x - origin.position.x
        dy = y - origin.position.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        mx = math.floor(local_x / grid.info.resolution)
        my = math.floor(local_y / grid.info.resolution)

        if not self._grid_contains(grid, mx, my):
            return None

        return int(mx), int(my)

    def _grid_to_world(
        self,
        grid: OccupancyGrid,
        mx: int,
        my: int,
    ) -> Tuple[float, float]:
        origin = grid.info.origin
        yaw = self._quaternion_to_yaw(origin.orientation)
        local_x = (mx + 0.5) * grid.info.resolution
        local_y = (my + 0.5) * grid.info.resolution
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        world_x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
        world_y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
        return world_x, world_y

    def _grid_contains(self, grid: OccupancyGrid, mx: int, my: int) -> bool:
        return 0 <= mx < grid.info.width and 0 <= my < grid.info.height

    def _grid_value(self, grid: OccupancyGrid, mx: int, my: int) -> int:
        index = my * grid.info.width + mx
        if index < 0 or index >= len(grid.data):
            return -1
        return int(grid.data[index])

    def _candidate_key(self, x: float, y: float) -> Tuple[float, float]:
        return round(x, 1), round(y, 1)

    def _candidate_to_pose(self, candidate: Candidate) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = candidate.x
        pose.pose.position.y = candidate.y
        pose.pose.position.z = 0.0

        yaw = math.atan2(
            self.target_y - candidate.y,
            self.target_x - candidate.x,
        )
        pose.pose.orientation = self._yaw_to_quaternion(yaw)
        return pose

    def _make_nav_goal_pose(self, goal_pose: PoseStamped) -> PoseStamped:
        nav_goal_pose = PoseStamped()
        nav_goal_pose.header.frame_id = self.global_frame
        nav_goal_pose.header.stamp = Time().to_msg()
        nav_goal_pose.pose.position.x = goal_pose.pose.position.x
        nav_goal_pose.pose.position.y = goal_pose.pose.position.y
        nav_goal_pose.pose.position.z = goal_pose.pose.position.z
        nav_goal_pose.pose.orientation.x = goal_pose.pose.orientation.x
        nav_goal_pose.pose.orientation.y = goal_pose.pose.orientation.y
        nav_goal_pose.pose.orientation.z = goal_pose.pose.orientation.z
        nav_goal_pose.pose.orientation.w = goal_pose.pose.orientation.w
        return nav_goal_pose

    def _publish_candidate_markers(
        self,
        candidates: List[Candidate],
        selected: Optional[Candidate],
        recovery_selected: Optional[Candidate] = None,
    ) -> None:
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for index, candidate in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = now
            marker.ns = 'valid_candidates' if candidate.valid else 'rejected_candidates'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = candidate.x
            marker.pose.position.y = candidate.y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.18 if candidate.valid else 0.10
            marker.scale.y = marker.scale.x
            marker.scale.z = marker.scale.x

            if candidate.valid:
                marker.color.r = 0.10
                marker.color.g = 0.80
                marker.color.b = 0.20
                marker.color.a = 0.80
            else:
                marker.color.r = 0.90
                marker.color.g = 0.10
                marker.color.b = 0.10
                marker.color.a = 0.35

            marker_array.markers.append(marker)

        for index, memory_candidate in enumerate(self.memory_candidates):
            if (memory_candidate.failed
                    or memory_candidate.attempted
                    or memory_candidate.reached):
                continue
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = now
            marker.ns = 'memory_candidates'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = memory_candidate.x
            marker.pose.position.y = memory_candidate.y
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.16
            marker.scale.y = 0.16
            marker.scale.z = 0.16
            marker.color.r = 1.00
            marker.color.g = 0.78
            marker.color.b = 0.05
            marker.color.a = 0.85
            marker_array.markers.append(marker)

        for index, breadcrumb in enumerate(self.breadcrumbs):
            if breadcrumb.failed or breadcrumb.recovery_attempted:
                continue
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = now
            marker.ns = 'breadcrumbs'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = breadcrumb.x
            marker.pose.position.y = breadcrumb.y
            marker.pose.position.z = 0.06
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.14
            marker.scale.y = 0.14
            marker.scale.z = 0.14
            marker.color.r = 1.00
            marker.color.g = 0.45
            marker.color.b = 0.05
            marker.color.a = 0.70
            marker_array.markers.append(marker)

        if selected is not None:
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = now
            marker.ns = 'selected_goal'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = selected.x
            marker.pose.position.y = selected.y
            marker.pose.position.z = 0.12
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.36
            marker.scale.y = 0.36
            marker.scale.z = 0.36
            marker.color.r = 0.10
            marker.color.g = 0.35
            marker.color.b = 1.00
            marker.color.a = 1.00
            marker_array.markers.append(marker)

        if recovery_selected is not None:
            marker = Marker()
            marker.header.frame_id = self.global_frame
            marker.header.stamp = now
            marker.ns = 'selected_recovery_goal'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = recovery_selected.x
            marker.pose.position.y = recovery_selected.y
            marker.pose.position.z = 0.16
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.42
            marker.scale.y = 0.42
            marker.scale.z = 0.42
            marker.color.r = 0.65
            marker.color.g = 0.20
            marker.color.b = 1.00
            marker.color.a = 1.00
            marker_array.markers.append(marker)

        self.candidate_marker_pub.publish(marker_array)

    def _sample_range(
        self,
        start: float,
        stop: float,
        step: float,
    ) -> List[float]:
        if step <= 0.0:
            return [start, stop] if start != stop else [start]

        if stop < start:
            start, stop = stop, start

        values = []
        value = start
        epsilon = step * 1e-6
        while value <= stop + epsilon:
            values.append(value)
            value += step

        if values and values[-1] < stop - epsilon:
            values.append(stop)
        elif not values:
            values.append(start)

        return values

    def _sample_angle_offsets(self, fan_angle_deg: float) -> List[float]:
        angle_offsets = self._sample_range(
            -abs(fan_angle_deg),
            abs(fan_angle_deg),
            abs(self.fan_angle_step_deg),
        )
        if 0.0 not in angle_offsets:
            angle_offsets.append(0.0)
            angle_offsets.sort()
        return angle_offsets

    def _effective_ray_step_resolution(self) -> float:
        if self.ray_step_resolution > 0.0:
            return self.ray_step_resolution
        if self.map_msg is not None and self.map_msg.info.resolution > 0.0:
            return self.map_msg.info.resolution
        return 0.05

    def _summarize_rejections(self, candidates: List[Candidate]) -> str:
        if not candidates:
            return 'no candidates generated'

        counts = Counter(candidate.reason or 'valid' for candidate in candidates)
        reason_order = [
            'outside_map',
            'not_known_free',
            'occupied_clearance',
            'global_costmap_reject',
            'no_progress',
            'blacklisted',
            'too_close',
        ]
        parts = [
            '{}={}'.format(reason, counts.get(reason, 0))
            for reason in reason_order
        ]
        extras = sorted(
            reason for reason in counts
            if reason not in reason_order and reason != 'valid'
        )
        parts.extend(
            '{}={}'.format(reason, counts[reason])
            for reason in extras
        )
        if counts.get('valid', 0) > 0:
            parts.append('valid={}'.format(counts['valid']))
        return ', '.join(parts)

    def _summarize_ray_stats(self, stats: RayBackoffStats) -> str:
        return (
            'angles_checked={}, hit_unknown={}, hit_outside_map={}, '
            'hit_occupied={}, no_safe_free_cell={}'
            .format(
                stats.angles_checked,
                stats.hit_unknown,
                stats.hit_outside_map,
                stats.hit_occupied,
                stats.no_safe_free_cell,
            )
        )

    def _distance(self, ax: float, ay: float, bx: float, by: float) -> float:
        return math.hypot(ax - bx, ay - by)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw * 0.5)
        quaternion.w = math.cos(yaw * 0.5)
        return quaternion

    def _quaternion_to_yaw(self, quaternion: Quaternion) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _goal_status_name(self, status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, str(status))


def main(args=None):
    rclpy.init(args=args)
    node = TargetExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
