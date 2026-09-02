#!/usr/bin/env python3
"""Generate the DTU centreline preference raster and routable road graph.

The source of truth is the projected geometry in dtu_map.gpkg.  The raster
uses the existing DTU Nav2 grid, while the graph stores map-frame metres.
Only GDAL/OGR, NumPy and PyYAML are required; no raster skeletonization is
used.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np
from osgeo import gdal, ogr, osr
import yaml


EXPECTED_EPSG = 32643
DEFAULT_EDGE_COST = 45
DEFAULT_OFFROAD_COST = 70


@dataclass
class RoadPart:
    feature_id: int
    part_id: int
    points: list[tuple[float, float]]


@dataclass
class Segment:
    segment_id: int
    feature_id: int
    part_id: int
    segment_index: int
    start: tuple[float, float]
    end: tuple[float, float]
    splits: list[tuple[float, tuple[float, float]]] = field(default_factory=list)

    @property
    def length(self) -> float:
        return math.dist(self.start, self.end)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("gpkg", type=Path, help="Input dtu_map.gpkg")
    parser.add_argument("output", type=Path, help="dtu_prior_map package root")
    parser.add_argument(
        "--map-origin",
        type=Path,
        help="Existing dtu_map_origin.yaml (defaults to OUTPUT/maps)",
    )
    parser.add_argument("--edge-cost", type=int, default=DEFAULT_EDGE_COST)
    parser.add_argument("--offroad-cost", type=int, default=DEFAULT_OFFROAD_COST)
    parser.add_argument("--snap-tolerance", type=float, default=1.0)
    return parser.parse_args()


def validate_spatial_reference(layer: ogr.Layer, name: str) -> None:
    srs = layer.GetSpatialRef()
    if srs is None:
        raise RuntimeError(f"Layer '{name}' has no CRS")
    srs = srs.Clone()
    srs.AutoIdentifyEPSG()
    code = srs.GetAuthorityCode(None) or srs.GetAuthorityCode("PROJCS")
    if code is None or int(code) != EXPECTED_EPSG:
        raise RuntimeError(f"Layer '{name}' must be EPSG:{EXPECTED_EPSG}")


def read_layer(
    dataset: ogr.DataSource, name: str
) -> tuple[ogr.osr.SpatialReference, list[tuple[int, ogr.Geometry]]]:
    layer = dataset.GetLayerByName(name)
    if layer is None:
        raise RuntimeError(f"GeoPackage is missing layer '{name}'")
    validate_spatial_reference(layer, name)
    rows: list[tuple[int, ogr.Geometry]] = []
    for feature in layer:
        geometry = feature.GetGeometryRef()
        if geometry is None or geometry.IsEmpty():
            raise RuntimeError(f"Layer '{name}' has empty geometry at FID {feature.GetFID()}")
        clone = geometry.Clone()
        if not clone.IsValid():
            clone = clone.MakeValid()
        rows.append((feature.GetFID(), clone))
    if not rows:
        raise RuntimeError(f"Layer '{name}' is empty")
    rows.sort(key=lambda item: item[0])
    return layer.GetSpatialRef().Clone(), rows


def union_all(rows: Iterable[tuple[int, ogr.Geometry]]) -> ogr.Geometry:
    iterator = iter(rows)
    try:
        _, first = next(iterator)
    except StopIteration as exc:
        raise RuntimeError("Cannot union an empty geometry collection") from exc
    merged = first.Clone()
    for _, geometry in iterator:
        merged = merged.Union(geometry)
    return merged


def rasterize_geometries(
    geometries: Iterable[ogr.Geometry],
    *,
    width: int,
    height: int,
    geotransform: tuple[float, float, float, float, float, float],
    projection: str,
    all_touched: bool,
) -> np.ndarray:
    raster = gdal.GetDriverByName("MEM").Create("", width, height, 1, gdal.GDT_Byte)
    raster.SetGeoTransform(geotransform)
    raster.SetProjection(projection)
    raster.GetRasterBand(1).Fill(0)

    vector = ogr.GetDriverByName("Memory").CreateDataSource("")
    srs = osr.SpatialReference()
    srs.ImportFromWkt(projection)
    layer = vector.CreateLayer("features", srs=srs, geom_type=ogr.wkbUnknown)
    definition = layer.GetLayerDefn()
    for geometry in geometries:
        feature = ogr.Feature(definition)
        feature.SetGeometry(geometry)
        if layer.CreateFeature(feature) != ogr.OGRERR_NONE:
            raise RuntimeError("Failed to stage a geometry for rasterization")

    options = ["ALL_TOUCHED=TRUE"] if all_touched else []
    result = gdal.RasterizeLayer(
        raster, [1], layer, burn_values=[1], options=options
    )
    if result != gdal.CE_None:
        raise RuntimeError("GDAL rasterization failed")
    raw = raster.GetRasterBand(1).ReadRaster(
        0, 0, width, height, buf_xsize=width, buf_ysize=height,
        buf_type=gdal.GDT_Byte,
    )
    return np.frombuffer(raw, dtype=np.uint8).reshape((height, width)).copy()


def proximity(mask: np.ndarray) -> np.ndarray:
    height, width = mask.shape
    source = gdal.GetDriverByName("MEM").Create("", width, height, 1, gdal.GDT_Byte)
    source.GetRasterBand(1).WriteRaster(
        0, 0, width, height, mask.tobytes(order="C"),
        buf_xsize=width, buf_ysize=height, buf_type=gdal.GDT_Byte,
    )
    target = gdal.GetDriverByName("MEM").Create("", width, height, 1, gdal.GDT_Float32)
    result = gdal.ComputeProximity(
        source.GetRasterBand(1),
        target.GetRasterBand(1),
        ["VALUES=1", "DISTUNITS=PIXEL"],
    )
    if result != gdal.CE_None:
        raise RuntimeError("GDAL proximity calculation failed")
    raw = target.GetRasterBand(1).ReadRaster(
        0, 0, width, height, buf_xsize=width, buf_ysize=height,
        buf_type=gdal.GDT_Float32,
    )
    return np.frombuffer(raw, dtype=np.float32).reshape((height, width)).copy()


def write_pgm(path: Path, array: np.ndarray, comment: str) -> None:
    if array.dtype != np.uint8:
        raise TypeError("PGM array must be uint8")
    height, width = array.shape
    with path.open("wb") as stream:
        stream.write(f"P5\n# {comment}\n{width} {height}\n255\n".encode("ascii"))
        stream.write(array.tobytes(order="C"))


def write_map_yaml(path: Path, image_name: str, resolution: float) -> None:
    path.write_text(
        "\n".join(
            [
                f"image: {image_name}",
                "mode: raw",
                f"resolution: {resolution:.6f}",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        ),
        encoding="utf-8",
    )


def flatten_lines(geometry: ogr.Geometry) -> list[ogr.Geometry]:
    kind = ogr.GT_Flatten(geometry.GetGeometryType())
    if kind == ogr.wkbLineString:
        return [geometry.Clone()]
    if kind in (ogr.wkbMultiLineString, ogr.wkbGeometryCollection):
        lines: list[ogr.Geometry] = []
        for index in range(geometry.GetGeometryCount()):
            lines.extend(flatten_lines(geometry.GetGeometryRef(index)))
        return lines
    return []


def cross(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


def subtract(
    a: tuple[float, float], b: tuple[float, float]
) -> tuple[float, float]:
    return (a[0] - b[0], a[1] - b[1])


def interpolate(
    a: tuple[float, float], b: tuple[float, float], t: float
) -> tuple[float, float]:
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))


def projection_parameter(
    point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]
) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return 0.0
    return max(
        0.0,
        min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator),
    )


def segment_intersections(
    first: Segment, second: Segment, epsilon: float = 1.0e-9
) -> list[tuple[float, float, tuple[float, float]]]:
    p = first.start
    q = second.start
    r = subtract(first.end, first.start)
    s = subtract(second.end, second.start)
    r_cross_s = cross(r, s)
    q_minus_p = subtract(q, p)
    output: list[tuple[float, float, tuple[float, float]]] = []

    if abs(r_cross_s) > epsilon:
        t = cross(q_minus_p, s) / r_cross_s
        u = cross(q_minus_p, r) / r_cross_s
        if -epsilon <= t <= 1.0 + epsilon and -epsilon <= u <= 1.0 + epsilon:
            t = max(0.0, min(1.0, t))
            u = max(0.0, min(1.0, u))
            output.append((t, u, interpolate(first.start, first.end, t)))
        return output

    if abs(cross(q_minus_p, r)) > epsilon:
        return output

    # Collinear overlap: split at every endpoint that lies on the other segment.
    candidates = [
        (first.start, 0.0, None),
        (first.end, 1.0, None),
        (second.start, None, 0.0),
        (second.end, None, 1.0),
    ]
    for point, known_t, known_u in candidates:
        t = projection_parameter(point, first.start, first.end)
        u = projection_parameter(point, second.start, second.end)
        on_first = math.dist(point, interpolate(first.start, first.end, t)) <= epsilon
        on_second = math.dist(point, interpolate(second.start, second.end, u)) <= epsilon
        if on_first and on_second:
            output.append((known_t if known_t is not None else t, known_u if known_u is not None else u, point))
    return output


def add_split(segment: Segment, t: float, point: tuple[float, float]) -> None:
    if all(abs(t - old_t) > 1.0e-8 for old_t, _ in segment.splits):
        segment.splits.append((t, point))


def build_graph(
    road_rows: list[tuple[int, ogr.Geometry]],
    boundary: ogr.Geometry,
    *,
    origin_e: float,
    origin_n: float,
    snap_tolerance: float,
) -> dict:
    parts: list[RoadPart] = []
    for feature_id, geometry in road_rows:
        clipped = geometry.Intersection(boundary)
        for part_index, line in enumerate(flatten_lines(clipped)):
            points = [
                (line.GetX(i) - origin_e, line.GetY(i) - origin_n)
                for i in range(line.GetPointCount())
            ]
            clean = [points[0]] if points else []
            for point in points[1:]:
                if math.dist(point, clean[-1]) > 1.0e-6:
                    clean.append(point)
            if len(clean) >= 2:
                parts.append(RoadPart(feature_id, part_index, clean))

    segments: list[Segment] = []
    for part_id, part in enumerate(parts):
        for segment_index, (start, end) in enumerate(zip(part.points, part.points[1:])):
            segment = Segment(
                len(segments), part.feature_id, part_id, segment_index, start, end
            )
            if segment.length > 1.0e-6:
                segment.splits = [(0.0, start), (1.0, end)]
                segments.append(segment)

    # Spatial buckets keep exact segment-intersection testing tractable.
    bucket_size = 25.0
    buckets: dict[tuple[int, int], list[int]] = {}
    for segment in segments:
        min_x = min(segment.start[0], segment.end[0])
        max_x = max(segment.start[0], segment.end[0])
        min_y = min(segment.start[1], segment.end[1])
        max_y = max(segment.start[1], segment.end[1])
        for bx in range(math.floor(min_x / bucket_size), math.floor(max_x / bucket_size) + 1):
            for by in range(math.floor(min_y / bucket_size), math.floor(max_y / bucket_size) + 1):
                buckets.setdefault((bx, by), []).append(segment.segment_id)

    tested_pairs: set[tuple[int, int]] = set()
    intersection_points: set[tuple[float, float]] = set()
    for members in buckets.values():
        for left_index in range(len(members)):
            for right_index in range(left_index + 1, len(members)):
                pair = tuple(sorted((members[left_index], members[right_index])))
                if pair in tested_pairs:
                    continue
                tested_pairs.add(pair)
                first, second = segments[pair[0]], segments[pair[1]]
                for t, u, point in segment_intersections(first, second):
                    add_split(first, t, point)
                    add_split(second, u, point)
                    if 1.0e-8 < t < 1.0 - 1.0e-8 or 1.0e-8 < u < 1.0 - 1.0e-8:
                        intersection_points.add((round(point[0], 6), round(point[1], 6)))

    # Join dangling line endpoints to the nearest other road segment when close.
    endpoint_records: list[tuple[int, tuple[float, float]]] = []
    for part_id, part in enumerate(parts):
        endpoint_records.append((part_id, part.points[0]))
        endpoint_records.append((part_id, part.points[-1]))

    snap_connectors: list[
        tuple[tuple[float, float], tuple[float, float], int, int]
    ] = []
    for part_id, endpoint in endpoint_records:
        best: tuple[float, Segment, float, tuple[float, float]] | None = None
        for target in segments:
            if target.part_id == part_id:
                continue
            t = projection_parameter(endpoint, target.start, target.end)
            projected = interpolate(target.start, target.end, t)
            distance = math.dist(endpoint, projected)
            if best is None or distance < best[0]:
                best = (distance, target, t, projected)
        if best is None or best[0] > snap_tolerance or best[0] <= 1.0e-5:
            continue
        _, target, t, projected = best
        add_split(target, t, projected)
        source_feature = parts[part_id].feature_id
        snap_connectors.append((endpoint, projected, source_feature, target.feature_id))

    node_lookup: dict[tuple[float, float], int] = {}
    nodes: list[dict] = []

    def node_for(point: tuple[float, float]) -> int:
        key = (round(point[0], 6), round(point[1], 6))
        if key not in node_lookup:
            node_lookup[key] = len(nodes)
            nodes.append({"id": len(nodes), "x": key[0], "y": key[1]})
        return node_lookup[key]

    edge_by_nodes: dict[tuple[int, int], dict] = {}

    def add_edge(
        start: tuple[float, float],
        end: tuple[float, float],
        *,
        kind: str,
        source_feature: int,
    ) -> None:
        length = math.dist(start, end)
        if length <= 1.0e-5:
            return
        source = node_for(start)
        target = node_for(end)
        if source == target:
            return
        key = tuple(sorted((source, target)))
        candidate = {
            "source": source,
            "target": target,
            "length": round(length, 6),
            "geometry": [
                [round(start[0], 6), round(start[1], 6)],
                [round(end[0], 6), round(end[1], 6)],
            ],
            "kind": kind,
            "source_feature": source_feature,
        }
        previous = edge_by_nodes.get(key)
        if previous is None or candidate["length"] < previous["length"]:
            edge_by_nodes[key] = candidate

    for segment in segments:
        ordered = sorted(segment.splits, key=lambda item: item[0])
        for (_, start), (_, end) in zip(ordered, ordered[1:]):
            add_edge(start, end, kind="road", source_feature=segment.feature_id)

    for start, end, source_feature, _ in snap_connectors:
        add_edge(start, end, kind="snap_connector", source_feature=source_feature)

    edges = sorted(
        edge_by_nodes.values(),
        key=lambda edge: (edge["source"], edge["target"], edge["kind"]),
    )
    for edge_id, edge in enumerate(edges):
        edge["id"] = edge_id

    adjacency: list[list[int]] = [[] for _ in nodes]
    for edge in edges:
        adjacency[edge["source"]].append(edge["target"])
        adjacency[edge["target"]].append(edge["source"])

    components: list[list[int]] = []
    unseen = set(range(len(nodes)))
    while unseen:
        root = min(unseen)
        stack = [root]
        unseen.remove(root)
        component: list[int] = []
        while stack:
            node = stack.pop()
            component.append(node)
            for neighbour in adjacency[node]:
                if neighbour in unseen:
                    unseen.remove(neighbour)
                    stack.append(neighbour)
        components.append(sorted(component))
    components.sort(key=lambda component: (-len(component), component[0]))
    component_for_node: dict[int, int] = {}
    for component_id, component in enumerate(components):
        for node_id in component:
            component_for_node[node_id] = component_id
            nodes[node_id]["component"] = component_id
    for edge in edges:
        edge["component"] = component_for_node[edge["source"]]

    return {
        "schema_version": 1,
        "frame_id": "map",
        "crs": f"EPSG:{EXPECTED_EPSG}",
        "map_origin_utm": {"easting": origin_e, "northing": origin_n},
        "snap_tolerance": snap_tolerance,
        "nodes": nodes,
        "edges": edges,
        "components": [
            {
                "id": component_id,
                "node_count": len(component),
                "edge_count": sum(
                    edge["component"] == component_id for edge in edges
                ),
            }
            for component_id, component in enumerate(components)
        ],
        "statistics": {
            "source_road_features": len(road_rows),
            "clipped_line_parts": len(parts),
            "source_segments": len(segments),
            "geometric_intersections_split": len(intersection_points),
            "snap_connectors": len(snap_connectors),
            "duplicate_or_zero_edges_removed": (
                sum(max(0, len(segment.splits) - 1) for segment in segments)
                + len(snap_connectors)
                - len(edges)
            ),
        },
    }


def main() -> None:
    args = parse_args()
    if not 1 <= args.edge_cost < args.offroad_cost < 100:
        raise ValueError("Require 1 <= edge-cost < offroad-cost < 100")
    if args.snap_tolerance < 0.0:
        raise ValueError("snap-tolerance cannot be negative")

    package_root = args.output.resolve()
    maps_dir = package_root / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)
    origin_path = args.map_origin or maps_dir / "dtu_map_origin.yaml"
    origin_data = yaml.safe_load(origin_path.read_text(encoding="utf-8"))
    origin_e = float(origin_data["utm_origin_easting"])
    origin_n = float(origin_data["utm_origin_northing"])
    resolution = float(origin_data["resolution"])
    width = int(origin_data["width_pixels"])
    height = int(origin_data["height_pixels"])

    dataset = ogr.Open(str(args.gpkg), 0)
    if dataset is None:
        raise RuntimeError(f"Cannot open {args.gpkg}")
    srs, roads = read_layer(dataset, "roads")
    _, corridors = read_layer(dataset, "road_corridors")
    _, buildings = read_layer(dataset, "buildings")
    _, boundaries = read_layer(dataset, "operational_boundary")
    boundary_geometry = union_all(boundaries)

    top_northing = origin_n + height * resolution
    geotransform = (origin_e, resolution, 0.0, top_northing, 0.0, -resolution)
    projection = srs.ExportToWkt()
    shape = (height, width)
    boundary_mask = rasterize_geometries(
        (geometry for _, geometry in boundaries),
        width=width,
        height=height,
        geotransform=geotransform,
        projection=projection,
        all_touched=False,
    )
    corridor_mask = rasterize_geometries(
        (geometry for _, geometry in corridors),
        width=width,
        height=height,
        geotransform=geotransform,
        projection=projection,
        all_touched=False,
    )
    building_mask = rasterize_geometries(
        (geometry for _, geometry in buildings),
        width=width,
        height=height,
        geotransform=geotransform,
        projection=projection,
        all_touched=True,
    )
    centreline_mask = rasterize_geometries(
        (geometry for _, geometry in roads),
        width=width,
        height=height,
        geotransform=geotransform,
        projection=projection,
        all_touched=True,
    )
    corridor_mask &= boundary_mask

    centre_distance = proximity(centreline_mask)
    outside_corridor = (corridor_mask == 0).astype(np.uint8)
    edge_distance = proximity(outside_corridor)
    # Pixel-centre proximity to the first outside cell overstates the remaining
    # half-width by half a cell. Correcting it makes the final inside row land
    # near the requested edge cost while preserving a smooth normalized ramp.
    edge_distance = np.maximum(edge_distance - 0.5, 0.25)

    output = np.full(shape, 100, dtype=np.uint8)
    traversable = boundary_mask == 1
    output[traversable] = args.offroad_cost
    road = corridor_mask == 1
    ratio = centre_distance[road] / (centre_distance[road] + edge_distance[road])
    output[road] = np.rint(args.edge_cost * ratio).astype(np.uint8)
    output[building_mask == 1] = 100

    pgm_path = maps_dir / "dtu_centerline_cost.pgm"
    yaml_path = maps_dir / "dtu_centerline_cost.yaml"
    graph_path = maps_dir / "dtu_road_graph.json"
    write_pgm(
        pgm_path,
        output,
        (
            f"normalized centreline preference: 0 centre, {args.edge_cost} edge, "
            f"{args.offroad_cost} off-road, 100 blocked"
        ),
    )
    write_map_yaml(yaml_path, pgm_path.name, resolution)

    graph = build_graph(
        roads,
        boundary_geometry,
        origin_e=origin_e,
        origin_n=origin_n,
        snap_tolerance=args.snap_tolerance,
    )
    graph_path.write_text(
        json.dumps(graph, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    unique, counts = np.unique(output, return_counts=True)
    print(f"Generated {width} x {height} centreline mask at {resolution:.3f} m/cell")
    print("Mask counts:", dict(zip(unique.tolist(), counts.tolist())))
    print(
        "Road graph:",
        f"{len(graph['nodes'])} nodes, {len(graph['edges'])} edges,",
        f"{len(graph['components'])} components",
    )
    print("Graph statistics:", graph["statistics"])
    print(f"Output: {pgm_path}, {yaml_path}, {graph_path}")


if __name__ == "__main__":
    main()
