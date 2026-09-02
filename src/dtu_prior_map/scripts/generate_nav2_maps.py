#!/usr/bin/env python3
"""Convert the DTU GeoPackage prior into Nav2-compatible raster maps.

Inputs expected in the GeoPackage:
  - operational_boundary (Polygon/MultiPolygon)
  - road_corridors (Polygon/MultiPolygon)
  - buildings (Polygon/MultiPolygon)
  - other_features (optional; natural=water is treated as occupied)

Outputs:
  - dtu_static.pgm/yaml: hard occupancy (buildings, water, outside boundary)
  - dtu_road_mask.pgm/yaml: preferred roads with weighted off-road terrain
  - dtu_map_origin.yaml: exact projected and geographic origin metadata
  - dtu_map_statistics.yaml: validation and pixel counts
  - preview/dtu_road_mask_preview.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import geopandas as gpd
import numpy as np
from PIL import Image, ImageDraw
from pyproj import Proj, Transformer
from rasterio.features import rasterize
from rasterio.transform import from_origin


EXPECTED_EPSG = 32643


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("gpkg", type=Path, help="Input dtu_map.gpkg")
    parser.add_argument("output", type=Path, help="Package root/output directory")
    parser.add_argument("--resolution", type=float, default=0.25)
    parser.add_argument("--offroad-cost", type=int, default=70)
    return parser.parse_args()


def read_layer(path: Path, layer: str) -> gpd.GeoDataFrame:
    frame = gpd.read_file(path, layer=layer)
    if frame.empty:
        raise RuntimeError(f"Layer '{layer}' is empty")
    if frame.crs is None or frame.crs.to_epsg() != EXPECTED_EPSG:
        raise RuntimeError(
            f"Layer '{layer}' must be EPSG:{EXPECTED_EPSG}; got {frame.crs}"
        )
    if frame.geometry.isna().any() or frame.geometry.is_empty.any():
        raise RuntimeError(f"Layer '{layer}' contains null or empty geometry")
    if not frame.geometry.is_valid.all():
        frame = frame.copy()
        frame.geometry = frame.geometry.make_valid()
    return frame


def mask_for(geometries, shape: tuple[int, int], transform, *, all_touched: bool):
    items = [(geom, 1) for geom in geometries if geom is not None and not geom.is_empty]
    if not items:
        return np.zeros(shape, dtype=np.uint8)
    return rasterize(
        items,
        out_shape=shape,
        transform=transform,
        fill=0,
        default_value=1,
        all_touched=all_touched,
        dtype=np.uint8,
    )


def write_pgm(path: Path, array: np.ndarray, comment: str) -> None:
    if array.dtype != np.uint8:
        raise TypeError("PGM data must be uint8")
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


def write_preview(
    path: Path,
    boundary_mask: np.ndarray,
    road_mask: np.ndarray,
    hard_mask: np.ndarray,
) -> None:
    # Exact semantic preview: gray=outside, tan=off-road, green=road, black=hard obstacle.
    rgb = np.zeros((*boundary_mask.shape, 3), dtype=np.uint8)
    rgb[:] = (78, 78, 78)
    rgb[boundary_mask == 1] = (222, 201, 151)
    rgb[(boundary_mask == 1) & (road_mask == 1)] = (85, 176, 92)
    rgb[hard_mask == 1] = (20, 20, 20)

    image = Image.fromarray(rgb, mode="RGB")
    max_side = 1600
    scale = min(1.0, max_side / max(image.size))
    if scale < 1.0:
        image = image.resize(
            (round(image.width * scale), round(image.height * scale)),
            Image.Resampling.NEAREST,
        )

    legend_height = 58
    canvas = Image.new("RGB", (image.width, image.height + legend_height), "white")
    canvas.paste(image, (0, 0))
    draw = ImageDraw.Draw(canvas)
    entries = [
        ((85, 176, 92), "road: 0"),
        ((222, 201, 151), "off-road: 70"),
        ((20, 20, 20), "building/water: 100"),
        ((78, 78, 78), "outside boundary: 100"),
    ]
    x = 14
    y = image.height + 18
    for colour, label in entries:
        draw.rectangle((x, y, x + 20, y + 20), fill=colour, outline="black")
        draw.text((x + 27, y + 3), label, fill="black")
        x += max(145, 37 + 7 * len(label))
    canvas.save(path, optimize=True)


def main() -> None:
    args = parse_args()
    if args.resolution <= 0:
        raise ValueError("resolution must be positive")
    if not 1 <= args.offroad_cost <= 99:
        raise ValueError("offroad cost must be between 1 and 99")

    package_root = args.output.resolve()
    maps_dir = package_root / "maps"
    preview_dir = package_root / "preview"
    maps_dir.mkdir(parents=True, exist_ok=True)
    preview_dir.mkdir(parents=True, exist_ok=True)

    boundary = read_layer(args.gpkg, "operational_boundary")
    roads = read_layer(args.gpkg, "road_corridors")
    buildings = read_layer(args.gpkg, "buildings")

    try:
        others = read_layer(args.gpkg, "other_features")
        water = others[others.get("natural").eq("water")]
    except (ValueError, RuntimeError):
        water = gpd.GeoDataFrame(geometry=[], crs=f"EPSG:{EXPECTED_EPSG}")

    min_x, min_y, max_x, max_y = boundary.total_bounds
    resolution = args.resolution
    origin_e = math.floor(min_x / resolution) * resolution
    origin_n = math.floor(min_y / resolution) * resolution
    raster_max_e = math.ceil(max_x / resolution) * resolution
    raster_max_n = math.ceil(max_y / resolution) * resolution
    width = int(round((raster_max_e - origin_e) / resolution))
    height = int(round((raster_max_n - origin_n) / resolution))
    shape = (height, width)
    transform = from_origin(origin_e, raster_max_n, resolution, resolution)

    boundary_mask = mask_for(boundary.geometry, shape, transform, all_touched=False)
    road_mask = mask_for(roads.geometry, shape, transform, all_touched=False)
    building_mask = mask_for(buildings.geometry, shape, transform, all_touched=True)
    water_mask = mask_for(water.geometry, shape, transform, all_touched=True)
    hard_mask = np.maximum(building_mask, water_mask)

    static_map = np.full(shape, 100, dtype=np.uint8)
    static_map[boundary_mask == 1] = 0
    static_map[hard_mask == 1] = 100

    road_preference = np.full(shape, 100, dtype=np.uint8)
    road_preference[boundary_mask == 1] = args.offroad_cost
    road_preference[(boundary_mask == 1) & (road_mask == 1)] = 0
    road_preference[hard_mask == 1] = 100

    write_pgm(
        maps_dir / "dtu_static.pgm",
        static_map,
        "Nav2 raw occupancy: 0 free, 100 occupied",
    )
    write_pgm(
        maps_dir / "dtu_road_mask.pgm",
        road_preference,
        f"Nav2 raw weighted mask: 0 road, {args.offroad_cost} off-road, 100 blocked",
    )
    write_map_yaml(maps_dir / "dtu_static.yaml", "dtu_static.pgm", resolution)
    write_map_yaml(maps_dir / "dtu_road_mask.yaml", "dtu_road_mask.pgm", resolution)

    to_wgs84 = Transformer.from_crs(EXPECTED_EPSG, 4326, always_xy=True)
    datum_lon, datum_lat = to_wgs84.transform(origin_e, origin_n)
    grid_convergence_degrees = Proj(
        f"EPSG:{EXPECTED_EPSG}"
    ).get_factors(datum_lon, datum_lat).meridian_convergence
    grid_convergence_radians = math.radians(grid_convergence_degrees)
    datum_yaw_radians = -grid_convergence_radians

    origin_text = f"""# Geographic origin of Nav2 map coordinate (0, 0).
map_frame: map
projected_crs: EPSG:{EXPECTED_EPSG}
utm_zone: 43N
utm_origin_easting: {origin_e:.3f}
utm_origin_northing: {origin_n:.3f}
datum_latitude: {datum_lat:.10f}
datum_longitude: {datum_lon:.10f}
grid_convergence_radians: {grid_convergence_radians:.10f}
datum_yaw_radians: {datum_yaw_radians:.10f}
resolution: {resolution:.6f}
width_pixels: {width}
height_pixels: {height}
width_meters: {width * resolution:.3f}
height_meters: {height * resolution:.3f}
"""
    (maps_dir / "dtu_map_origin.yaml").write_text(origin_text, encoding="utf-8")

    unique_static, counts_static = np.unique(static_map, return_counts=True)
    unique_road, counts_road = np.unique(road_preference, return_counts=True)
    stats = {
        "boundary_area_m2": float(boundary.geometry.area.sum()),
        "road_area_inside_boundary_m2": float(
            roads.geometry.union_all().intersection(boundary.geometry.union_all()).area
        ),
        "buildings_intersecting_boundary": int(
            buildings.intersects(boundary.geometry.union_all()).sum()
        ),
        "water_features_intersecting_boundary": int(
            water.intersects(boundary.geometry.union_all()).sum()
        ),
        "static_pixel_counts": dict(zip(unique_static.tolist(), counts_static.tolist())),
        "road_mask_pixel_counts": dict(zip(unique_road.tolist(), counts_road.tolist())),
    }
    stats_lines = [
        "# Generated validation statistics",
        f"boundary_area_m2: {stats['boundary_area_m2']:.3f}",
        f"road_area_inside_boundary_m2: {stats['road_area_inside_boundary_m2']:.3f}",
        f"buildings_intersecting_boundary: {stats['buildings_intersecting_boundary']}",
        f"water_features_intersecting_boundary: {stats['water_features_intersecting_boundary']}",
        "static_pixel_counts:",
    ]
    stats_lines.extend(f"  {key}: {value}" for key, value in stats["static_pixel_counts"].items())
    stats_lines.append("road_mask_pixel_counts:")
    stats_lines.extend(f"  {key}: {value}" for key, value in stats["road_mask_pixel_counts"].items())
    stats_lines.append("")
    (maps_dir / "dtu_map_statistics.yaml").write_text(
        "\n".join(stats_lines), encoding="utf-8"
    )

    write_preview(
        preview_dir / "dtu_road_mask_preview.png",
        boundary_mask,
        road_mask,
        hard_mask,
    )

    print(f"Generated {width} x {height} cells at {resolution:.3f} m/cell")
    print(f"Map datum: {datum_lat:.10f}, {datum_lon:.10f}")
    print(f"Grid convergence: {grid_convergence_radians:.10f} rad")
    print(f"Datum yaw: {datum_yaw_radians:.10f} rad")
    print(f"Output: {package_root}")


if __name__ == "__main__":
    main()
