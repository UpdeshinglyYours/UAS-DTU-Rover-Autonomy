#!/usr/bin/env python3
"""Validate geometry and navigability of flat_100m_obstacle_field.sdf."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from math import hypot, isclose
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


GROUND_NAME = "flat_ground_100m"
WORLD_HALF_EXTENT = 50.0
OBSTACLE_EDGE_LIMIT = 48.0
MIN_MODEL_CLEARANCE = 3.0
MIN_SPAWN_CLEARANCE = 2.0
PREFERRED_SPAWN_CLEARANCE = 4.0
TOLERANCE = 1.0e-8


@dataclass(frozen=True)
class Component:
    model: str
    link: str
    kind: str
    cx: float
    cy: float
    bottom: float
    top: float
    xmin: float = 0.0
    xmax: float = 0.0
    ymin: float = 0.0
    ymax: float = 0.0
    radius: float = 0.0


def fail(message: str) -> None:
    raise ValueError(message)


def floats(text: str | None, count: int, label: str) -> tuple[float, ...]:
    if text is None:
        fail(f"Missing {label}")
    values = tuple(float(value) for value in text.split())
    if len(values) != count:
        fail(f"{label} must contain {count} values, got {values}")
    return values


def pose(element: ET.Element, label: str) -> tuple[float, ...]:
    pose_element = element.find("pose")
    if pose_element is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = floats(pose_element.text, 6, f"{label} pose")
    if any(abs(value) > TOLERANCE for value in values[3:]):
        fail(f"{label} has unsupported rotation {values[3:]}")
    return values


def geometry_signature(geometry: ET.Element, label: str) -> tuple:
    children = list(geometry)
    if len(children) != 1:
        fail(f"{label} must have exactly one geometry primitive")
    primitive = children[0]
    if primitive.tag == "box":
        size = floats(primitive.findtext("size"), 3, f"{label} box size")
        if any(value <= 0.0 for value in size):
            fail(f"{label} box size must be positive")
        return ("box", *size)
    if primitive.tag == "cylinder":
        radius = float(primitive.findtext("radius", "0"))
        length = float(primitive.findtext("length", "0"))
        if radius <= 0.0 or length <= 0.0:
            fail(f"{label} cylinder dimensions must be positive")
        return ("cylinder", radius, length)
    if primitive.tag == "plane":
        normal = floats(
            primitive.findtext("normal"), 3, f"{label} plane normal"
        )
        size = floats(primitive.findtext("size"), 2, f"{label} plane size")
        return ("plane", *normal, *size)
    fail(f"{label} uses forbidden/non-primitive geometry {primitive.tag}")


def obstacle_component(
    model: ET.Element,
    link: ET.Element,
    collision: ET.Element,
) -> Component:
    model_name = model.attrib["name"]
    link_name = link.attrib["name"]
    model_pose = pose(model, model_name)
    link_pose = pose(link, link_name)
    collision_pose = pose(collision, collision.attrib["name"])
    x = model_pose[0] + link_pose[0] + collision_pose[0]
    y = model_pose[1] + link_pose[1] + collision_pose[1]
    z = model_pose[2] + link_pose[2] + collision_pose[2]
    geometry = collision.find("geometry")
    if geometry is None:
        fail(f"{collision.attrib['name']} has no geometry")
    signature = geometry_signature(geometry, collision.attrib["name"])
    if signature[0] == "box":
        _, sx, sy, sz = signature
        return Component(
            model=model_name,
            link=link_name,
            kind="box",
            cx=x,
            cy=y,
            bottom=z - sz / 2.0,
            top=z + sz / 2.0,
            xmin=x - sx / 2.0,
            xmax=x + sx / 2.0,
            ymin=y - sy / 2.0,
            ymax=y + sy / 2.0,
        )
    if signature[0] == "cylinder":
        _, radius, length = signature
        return Component(
            model=model_name,
            link=link_name,
            kind="cylinder",
            cx=x,
            cy=y,
            bottom=z - length / 2.0,
            top=z + length / 2.0,
            radius=radius,
        )
    fail(f"Obstacle {model_name} must use only box or cylinder geometry")


def component_distance(first: Component, second: Component) -> float:
    if first.kind == "box" and second.kind == "box":
        dx = max(
            first.xmin - second.xmax,
            second.xmin - first.xmax,
            0.0,
        )
        dy = max(
            first.ymin - second.ymax,
            second.ymin - first.ymax,
            0.0,
        )
        return hypot(dx, dy)
    if first.kind == "cylinder" and second.kind == "cylinder":
        return max(
            0.0,
            hypot(first.cx - second.cx, first.cy - second.cy)
            - first.radius
            - second.radius,
        )
    box, cylinder = (
        (first, second) if first.kind == "box" else (second, first)
    )
    dx = max(box.xmin - cylinder.cx, cylinder.cx - box.xmax, 0.0)
    dy = max(box.ymin - cylinder.cy, cylinder.cy - box.ymax, 0.0)
    return max(0.0, hypot(dx, dy) - cylinder.radius)


def origin_distance(component: Component) -> float:
    if component.kind == "cylinder":
        return max(0.0, hypot(component.cx, component.cy) - component.radius)
    dx = max(component.xmin, -component.xmax, 0.0)
    dy = max(component.ymin, -component.ymax, 0.0)
    return hypot(dx, dy)


def is_blocked(
    x: float,
    y: float,
    components: list[Component],
    inflation: float,
) -> bool:
    for component in components:
        if component.kind == "box":
            if (
                component.xmin - inflation <= x <= component.xmax + inflation
                and component.ymin - inflation <= y <= component.ymax + inflation
            ):
                return True
        elif hypot(x - component.cx, y - component.cy) <= (
            component.radius + inflation
        ):
            return True
    return False


def validate_connectivity(components: list[Component]) -> tuple[int, int]:
    # Inflate by more than BCR Bot's half-width and use a conservative
    # cardinal grid. Reaching all four boundary corridors demonstrates that
    # buildings have not split the field into disconnected regions.
    resolution = 0.5
    lower = -48.0
    upper = 48.0
    count = int(round((upper - lower) / resolution)) + 1
    free = {
        (ix, iy)
        for ix in range(count)
        for iy in range(count)
        if not is_blocked(
            lower + ix * resolution,
            lower + iy * resolution,
            components,
            inflation=0.6,
        )
    }
    spawn = (
        int(round((0.0 - lower) / resolution)),
        int(round((0.0 - lower) / resolution)),
    )
    if spawn not in free:
        fail("Inflated obstacle geometry blocks the rover spawn")
    reached = {spawn}
    queue = deque([spawn])
    while queue:
        ix, iy = queue.popleft()
        for neighbor in ((ix - 1, iy), (ix + 1, iy), (ix, iy - 1), (ix, iy + 1)):
            if neighbor in free and neighbor not in reached:
                reached.add(neighbor)
                queue.append(neighbor)

    def grid_point(x: float, y: float) -> tuple[int, int]:
        return (
            int(round((x - lower) / resolution)),
            int(round((y - lower) / resolution)),
        )

    exits = {
        "north": grid_point(0.0, 47.0),
        "south": grid_point(0.0, -47.0),
        "east": grid_point(47.0, 0.0),
        "west": grid_point(-47.0, 0.0),
    }
    missing = [name for name, point in exits.items() if point not in reached]
    if missing:
        fail(f"Spawn cannot reach boundary corridors: {', '.join(missing)}")
    if len(reached) / len(free) < 0.99:
        fail(
            "Inflated obstacle layout disconnects more than 1% of free grid cells"
        )
    return len(reached), len(free)


def validate(path: Path) -> None:
    tree = ET.parse(path)
    root = tree.getroot()
    if root.tag != "sdf":
        fail("Root element must be sdf")
    world = root.find("world")
    if world is None or world.attrib.get("name") != "default":
        fail("Expected one world named default")
    if len(root.findall("world")) != 1:
        fail("Expected exactly one world")

    forbidden = ("include", "uri", "mesh", "heightmap")
    for tag in forbidden:
        if world.findall(f".//{tag}"):
            fail(f"World contains forbidden external/complex element <{tag}>")
    if world.findall("plugin"):
        fail("World-level plugins must remain implicit, matching the working world")

    models = world.findall("model")
    model_names = [model.attrib.get("name", "") for model in models]
    if len(model_names) != len(set(model_names)):
        fail("Model names are not unique")
    if GROUND_NAME not in model_names:
        fail(f"Missing {GROUND_NAME}")

    ground = next(model for model in models if model.attrib["name"] == GROUND_NAME)
    if ground.findtext("static") != "true":
        fail("Ground must be static")
    ground_pose = pose(ground, GROUND_NAME)
    if any(abs(value) > TOLERANCE for value in ground_pose):
        fail("Ground model pose must be exactly zero")
    ground_links = ground.findall("link")
    if len(ground_links) != 1:
        fail("Ground must have exactly one link")
    ground_link = ground_links[0]
    ground_collision = ground_link.find("collision")
    ground_visual = ground_link.find("visual")
    if ground_collision is None or ground_visual is None:
        fail("Ground requires collision and visual geometry")
    ground_collision_signature = geometry_signature(
        ground_collision.find("geometry"), "ground collision"
    )
    ground_visual_signature = geometry_signature(
        ground_visual.find("geometry"), "ground visual"
    )
    expected_ground = ("plane", 0.0, 0.0, 1.0, 100.0, 100.0)
    if ground_collision_signature != expected_ground:
        fail(f"Ground collision is not an exact 100 x 100 m flat plane: {ground_collision_signature}")
    if ground_visual_signature != expected_ground:
        fail(f"Ground visual is not an exact 100 x 100 m flat plane: {ground_visual_signature}")

    obstacle_models = [
        model for model in models if model.attrib["name"] != GROUND_NAME
    ]
    if not 25 <= len(obstacle_models) <= 35:
        fail(f"Expected 25-35 obstacles, found {len(obstacle_models)}")
    buildings = [
        model for model in obstacle_models if model.attrib["name"].startswith("building_")
    ]
    if not 6 <= len(buildings) <= 10:
        fail(f"Expected 6-10 buildings, found {len(buildings)}")

    all_names: list[str] = []
    components: list[Component] = []
    for model in models:
        model_name = model.attrib.get("name", "")
        if not model_name:
            fail("A model has no name")
        all_names.append(model_name)
        if model.findtext("static") != "true":
            fail(f"{model_name} is not static")
        if model is ground:
            links_to_check = [ground_link]
        else:
            links_to_check = model.findall("link")
            if not links_to_check:
                fail(f"{model_name} has no links")
        for link in links_to_check:
            link_name = link.attrib.get("name", "")
            if not link_name:
                fail(f"A link in {model_name} has no name")
            all_names.append(link_name)
            collisions = link.findall("collision")
            visuals = link.findall("visual")
            if len(collisions) != 1 or len(visuals) != 1:
                fail(f"{link_name} must have exactly one collision and one visual")
            collision = collisions[0]
            visual = visuals[0]
            collision_name = collision.attrib.get("name", "")
            visual_name = visual.attrib.get("name", "")
            if not collision_name or not visual_name:
                fail(f"{link_name} has an unnamed collision or visual")
            all_names.extend((collision_name, visual_name))
            collision_geometry = collision.find("geometry")
            visual_geometry = visual.find("geometry")
            if collision_geometry is None or visual_geometry is None:
                fail(f"{link_name} lacks collision or visual geometry")
            collision_signature = geometry_signature(
                collision_geometry, collision_name
            )
            visual_signature = geometry_signature(visual_geometry, visual_name)
            if collision_signature != visual_signature:
                fail(f"{link_name} visual and collision geometries differ")
            if model is not ground:
                component = obstacle_component(model, link, collision)
                if not isclose(component.bottom, 0.0, abs_tol=TOLERANCE):
                    fail(
                        f"{component.link} does not rest on the ground; bottom={component.bottom}"
                    )
                components.append(component)

    if len(all_names) != len(set(all_names)):
        duplicates = sorted(
            name for name in set(all_names) if all_names.count(name) > 1
        )
        fail(f"Element names are not globally unique: {duplicates}")

    for building in buildings:
        links = building.findall("link")
        if len(links) != 1:
            fail(f"Building {building.attrib['name']} must be one solid box")
        signature = geometry_signature(
            links[0].find("collision/geometry"), building.attrib["name"]
        )
        if signature[0] != "box":
            fail(f"Building {building.attrib['name']} is not a box")
        _, sx, sy, sz = signature
        if not (6.0 <= sx <= 15.0 and 6.0 <= sy <= 15.0 and 4.0 <= sz <= 12.0):
            fail(f"Building {building.attrib['name']} dimensions out of range: {signature[1:]}")

    for component in components:
        if component.kind == "box":
            extent = (
                component.xmin,
                component.xmax,
                component.ymin,
                component.ymax,
            )
            if min(extent) < -OBSTACLE_EDGE_LIMIT - TOLERANCE or max(extent) > OBSTACLE_EDGE_LIMIT + TOLERANCE:
                fail(f"{component.link} violates the 2 m boundary margin: {extent}")
        else:
            extent = (
                component.cx - component.radius,
                component.cx + component.radius,
                component.cy - component.radius,
                component.cy + component.radius,
            )
            if min(extent) < -OBSTACLE_EDGE_LIMIT - TOLERANCE or max(extent) > OBSTACLE_EDGE_LIMIT + TOLERANCE:
                fail(f"{component.link} violates the 2 m boundary margin: {extent}")

    by_model: dict[str, list[Component]] = {}
    for component in components:
        by_model.setdefault(component.model, []).append(component)
    closest_pair = (float("inf"), "", "")
    names = sorted(by_model)
    for index, first_name in enumerate(names):
        for second_name in names[index + 1 :]:
            distance = min(
                component_distance(first, second)
                for first in by_model[first_name]
                for second in by_model[second_name]
            )
            if distance < closest_pair[0]:
                closest_pair = (distance, first_name, second_name)
            if distance + TOLERANCE < MIN_MODEL_CLEARANCE:
                fail(
                    f"{first_name} and {second_name} have only {distance:.3f} m footprint clearance"
                )

    spawn_clearance = min(origin_distance(component) for component in components)
    if spawn_clearance + TOLERANCE < MIN_SPAWN_CLEARANCE:
        fail(f"Spawn clearance is only {spawn_clearance:.3f} m")
    if spawn_clearance + TOLERANCE < PREFERRED_SPAWN_CLEARANCE:
        fail(
            f"Spawn clearance {spawn_clearance:.3f} m is below preferred 4 m"
        )

    reached, free = validate_connectivity(components)
    print(f"PASS: {path}")
    print(f"  ground: exact flat plane, 100.000 x 100.000 m")
    print(
        f"  obstacles: {len(obstacle_models)} groups "
        f"({len(buildings)} buildings, {len(obstacle_models) - len(buildings)} smaller groups)"
    )
    print(f"  primitive collision components: {len(components)}")
    print(f"  spawn surface clearance: {spawn_clearance:.3f} m")
    print(
        f"  closest separate models: {closest_pair[0]:.3f} m "
        f"({closest_pair[1]} / {closest_pair[2]})"
    )
    print(f"  boundary margin: at least {WORLD_HALF_EXTENT - OBSTACLE_EDGE_LIMIT:.3f} m")
    print(
        f"  connected inflated free grid: {reached}/{free} cells; "
        "north/south/east/west corridors reachable"
    )
    print("  external models, meshes, heightmaps and world sensor plugins: none")


def main() -> int:
    default_world = Path(__file__).resolve().parents[1] / "worlds" / (
        "flat_100m_obstacle_field.sdf"
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("world", nargs="?", type=Path, default=default_world)
    args = parser.parse_args()
    try:
        validate(args.world.resolve())
    except (ET.ParseError, OSError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
