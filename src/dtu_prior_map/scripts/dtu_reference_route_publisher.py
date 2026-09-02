#!/usr/bin/env python3
"""Publish an obstacle-independent route over the DTU road-centre graph."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import json
import math
from pathlib import Path
from typing import Iterable

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


Point = tuple[float, float]


@dataclass(frozen=True)
class Edge:
    edge_id: int
    source: int
    target: int
    length: float
    geometry: tuple[Point, ...]
    component: int
    kind: str


@dataclass(frozen=True)
class Snap:
    edge: Edge
    point: Point
    along: float
    distance: float


def polyline_length(points: Iterable[Point]) -> float:
    points = list(points)
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def interpolate(a: Point, b: Point, fraction: float) -> Point:
    return (
        a[0] + fraction * (b[0] - a[0]),
        a[1] + fraction * (b[1] - a[1]),
    )


def project_to_polyline(point: Point, geometry: tuple[Point, ...]) -> tuple[Point, float, float]:
    best_distance = math.inf
    best_point = geometry[0]
    best_along = 0.0
    traversed = 0.0
    for start, end in zip(geometry, geometry[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        fraction = 0.0 if length_squared == 0.0 else max(
            0.0,
            min(
                1.0,
                ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                / length_squared,
            ),
        )
        projected = interpolate(start, end, fraction)
        distance = math.dist(point, projected)
        segment_length = math.sqrt(length_squared)
        if distance < best_distance:
            best_distance = distance
            best_point = projected
            best_along = traversed + fraction * segment_length
        traversed += segment_length
    return best_point, best_along, best_distance


def point_at_distance(geometry: tuple[Point, ...], distance: float) -> Point:
    remaining = max(0.0, distance)
    for start, end in zip(geometry, geometry[1:]):
        length = math.dist(start, end)
        if remaining <= length or length == 0.0:
            return interpolate(start, end, 0.0 if length == 0.0 else remaining / length)
        remaining -= length
    return geometry[-1]


def slice_polyline(
    geometry: tuple[Point, ...], start_distance: float, end_distance: float
) -> list[Point]:
    if end_distance < start_distance:
        return list(reversed(slice_polyline(geometry, end_distance, start_distance)))
    total = polyline_length(geometry)
    start_distance = max(0.0, min(total, start_distance))
    end_distance = max(0.0, min(total, end_distance))
    output = [point_at_distance(geometry, start_distance)]
    traversed = 0.0
    for index, (start, end) in enumerate(zip(geometry, geometry[1:])):
        traversed += math.dist(start, end)
        if start_distance < traversed < end_distance:
            output.append(geometry[index + 1])
    output.append(point_at_distance(geometry, end_distance))
    return deduplicate_points(output)


def deduplicate_points(points: Iterable[Point], epsilon: float = 1.0e-6) -> list[Point]:
    output: list[Point] = []
    for point in points:
        if not output or math.dist(point, output[-1]) > epsilon:
            output.append(point)
    return output


def densify(points: list[Point], spacing: float) -> list[Point]:
    if len(points) < 2:
        return points
    output = [points[0]]
    for start, end in zip(points, points[1:]):
        length = math.dist(start, end)
        steps = max(1, math.ceil(length / spacing))
        for step in range(1, steps + 1):
            output.append(interpolate(start, end, step / steps))
    return deduplicate_points(output)


class RoadGraph:
    def __init__(self, path: Path):
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("schema_version") != 1 or data.get("frame_id") != "map":
            raise RuntimeError(f"Unsupported road graph schema in {path}")
        self.nodes = {
            int(node["id"]): (float(node["x"]), float(node["y"]))
            for node in data["nodes"]
        }
        self.edges = {
            int(edge["id"]): Edge(
                edge_id=int(edge["id"]),
                source=int(edge["source"]),
                target=int(edge["target"]),
                length=float(edge["length"]),
                geometry=tuple((float(point[0]), float(point[1])) for point in edge["geometry"]),
                component=int(edge["component"]),
                kind=str(edge.get("kind", "road")),
            )
            for edge in data["edges"]
        }
        self.road_edges = [edge for edge in self.edges.values() if edge.kind == "road"]
        self.adjacency: dict[int, list[tuple[int, Edge]]] = {
            node_id: [] for node_id in self.nodes
        }
        for edge in self.edges.values():
            self.adjacency[edge.source].append((edge.target, edge))
            self.adjacency[edge.target].append((edge.source, edge))

    def snaps_by_component(self, point: Point) -> dict[int, Snap]:
        result: dict[int, Snap] = {}
        for edge in self.road_edges:
            projected, along, distance = project_to_polyline(point, edge.geometry)
            candidate = Snap(edge, projected, along, distance)
            previous = result.get(edge.component)
            if previous is None or candidate.distance < previous.distance:
                result[edge.component] = candidate
        return result

    def shortest_node_route(
        self, start: int, goal: int, component: int
    ) -> tuple[float, list[tuple[Edge, int, int]]] | None:
        queue: list[tuple[float, int]] = [(0.0, start)]
        distances = {start: 0.0}
        parents: dict[int, tuple[int, Edge]] = {}
        while queue:
            distance, node = heapq.heappop(queue)
            if distance != distances.get(node):
                continue
            if node == goal:
                break
            for neighbour, edge in self.adjacency[node]:
                if edge.component != component:
                    continue
                new_distance = distance + edge.length
                if new_distance < distances.get(neighbour, math.inf):
                    distances[neighbour] = new_distance
                    parents[neighbour] = (node, edge)
                    heapq.heappush(queue, (new_distance, neighbour))
        if goal not in distances:
            return None
        steps: list[tuple[Edge, int, int]] = []
        node = goal
        while node != start:
            previous, edge = parents[node]
            steps.append((edge, previous, node))
            node = previous
        steps.reverse()
        return distances[goal], steps

    def route(self, start: Point, goal: Point) -> tuple[list[Point], dict]:
        start_by_component = self.snaps_by_component(start)
        goal_by_component = self.snaps_by_component(goal)
        common_components = sorted(set(start_by_component) & set(goal_by_component))
        if not common_components:
            raise RuntimeError("Start and goal cannot be connected through the road graph")
        component = min(
            common_components,
            key=lambda component_id: (
                start_by_component[component_id].distance
                + goal_by_component[component_id].distance
            ),
        )
        start_snap = start_by_component[component]
        goal_snap = goal_by_component[component]

        candidates: list[tuple[float, list[Point]]] = []
        if start_snap.edge.edge_id == goal_snap.edge.edge_id:
            direct = slice_polyline(
                start_snap.edge.geometry, start_snap.along, goal_snap.along
            )
            candidates.append((abs(goal_snap.along - start_snap.along), direct))

        start_options = [
            (
                start_snap.edge.source,
                start_snap.along,
                list(reversed(slice_polyline(start_snap.edge.geometry, 0.0, start_snap.along))),
            ),
            (
                start_snap.edge.target,
                start_snap.edge.length - start_snap.along,
                slice_polyline(start_snap.edge.geometry, start_snap.along, start_snap.edge.length),
            ),
        ]
        goal_options = [
            (
                goal_snap.edge.source,
                goal_snap.along,
                slice_polyline(goal_snap.edge.geometry, 0.0, goal_snap.along),
            ),
            (
                goal_snap.edge.target,
                goal_snap.edge.length - goal_snap.along,
                list(reversed(slice_polyline(goal_snap.edge.geometry, goal_snap.along, goal_snap.edge.length))),
            ),
        ]
        for start_node, start_cost, start_geometry in start_options:
            for goal_node, goal_cost, goal_geometry in goal_options:
                middle = self.shortest_node_route(start_node, goal_node, component)
                if middle is None:
                    continue
                middle_cost, steps = middle
                geometry = list(start_geometry)
                for edge, from_node, to_node in steps:
                    oriented = (
                        list(edge.geometry)
                        if edge.source == from_node and edge.target == to_node
                        else list(reversed(edge.geometry))
                    )
                    geometry.extend(oriented)
                geometry.extend(goal_geometry)
                candidates.append(
                    (start_cost + middle_cost + goal_cost, deduplicate_points(geometry))
                )
        if not candidates:
            raise RuntimeError("No connected centreline route was found")
        graph_cost, graph_geometry = min(candidates, key=lambda candidate: candidate[0])
        full_geometry = deduplicate_points(
            [start, start_snap.point, *graph_geometry, goal_snap.point, goal]
        )
        return full_geometry, {
            "component": component,
            "start_connector_length": start_snap.distance,
            "goal_connector_length": goal_snap.distance,
            "graph_length": graph_cost,
            "total_length": polyline_length(full_geometry),
        }


class ReferenceRoutePublisher(Node):
    def __init__(self) -> None:
        super().__init__("dtu_reference_route_publisher")
        default_graph = str(
            Path(get_package_share_directory("dtu_prior_map"))
            / "maps"
            / "dtu_road_graph.json"
        )
        self.declare_parameter("graph_path", default_graph)
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("reference_topic", "/dtu_reference_route")
        self.declare_parameter("densify_spacing", 0.5)
        self.declare_parameter("goal_change_tolerance", 0.25)
        graph_path = Path(self.get_parameter("graph_path").value)
        self.graph = RoadGraph(graph_path)
        self.spacing = float(self.get_parameter("densify_spacing").value)
        self.goal_change_tolerance = float(
            self.get_parameter("goal_change_tolerance").value
        )
        if self.spacing <= 0.0:
            raise ValueError("densify_spacing must be positive")

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PathMessage, str(self.get_parameter("reference_topic").value), latched
        )
        self.subscription = self.create_subscription(
            PathMessage,
            str(self.get_parameter("plan_topic").value),
            self.on_plan,
            10,
        )
        self.last_goal: Point | None = None
        self.get_logger().info(
            f"Loaded {len(self.graph.nodes)} road nodes and {len(self.graph.edges)} edges"
        )

    def on_plan(self, plan: PathMessage) -> None:
        if len(plan.poses) < 2:
            return
        frame = plan.header.frame_id or plan.poses[0].header.frame_id
        if frame != "map":
            self.get_logger().error(f"Ignoring plan in frame '{frame}', expected 'map'")
            return
        first = plan.poses[0].pose.position
        last = plan.poses[-1].pose.position
        start = (first.x, first.y)
        goal = (last.x, last.y)
        # Nav2 republishes a collision-aware plan at 1 Hz.  Holding the route
        # for an unchanged goal prevents temporary obstacle detours from feeding
        # back into the nominal reference line.
        if self.last_goal is not None and math.dist(goal, self.last_goal) <= self.goal_change_tolerance:
            return
        try:
            points, metadata = self.graph.route(start, goal)
        except RuntimeError as error:
            self.get_logger().error(str(error))
            return
        points = densify(points, self.spacing)
        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            if index + 1 < len(points):
                dx = points[index + 1][0] - point[0]
                dy = points[index + 1][1] - point[1]
            else:
                dx = point[0] - points[index - 1][0]
                dy = point[1] - points[index - 1][1]
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            message.poses.append(pose)
        self.publisher.publish(message)
        self.last_goal = goal
        self.get_logger().info(
            "Published %d reference poses, %.2f m total (start connector %.2f m, "
            "goal connector %.2f m, component %d)"
            % (
                len(points),
                metadata["total_length"],
                metadata["start_connector_length"],
                metadata["goal_connector_length"],
                metadata["component"],
            )
        )


def main() -> None:
    rclpy.init()
    node = ReferenceRoutePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
