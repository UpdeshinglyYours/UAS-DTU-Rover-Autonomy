# DTU prior map for Nav2 (ROS 2 Humble)

This package turns the supplied DTU OpenStreetMap/QGIS data into aligned Nav2
maps and a routable centreline graph:

- `dtu_static`: free ground inside the selected boundary, with buildings and
  the outside area occupied.
- `dtu_road_mask`: roads cost `0`, non-road terrain inside the boundary costs
  `70`, and buildings/outside cost `100`.
- `dtu_centerline_cost`: source-road centres cost `0`, the normalized road-edge
  cost is about `45`, off-road terrain costs `70`, and hard areas cost `100`.
- `dtu_road_graph.json`: metric `map`-frame graph built from the original road
  LineStrings, including their intermediate vertices.

The static map and road mask are deliberately separate. The static map supplies
hard geometry; the weighted KeepoutFilter makes off-road travel possible but
undesirable. Live LiDAR obstacle/voxel layers must remain enabled and take
priority over this prior.

## Generated map facts

- Source CRS: `EPSG:32643` (WGS 84 / UTM zone 43N)
- Resolution: `0.25 m/cell`
- Size: `4218 x 4195` cells (`1054.5 x 1048.75 m`)
- Selected operational area: `557,832.905 m²`
- Road corridor area inside it: `67,415.218 m²`
- Buildings intersecting it: `72`
- Nav2 map `(0, 0)`:
  - UTM: `E 706231.750, N 3181578.500`
  - WGS84: `28.7451419213, 77.1119513181`
  - UTM grid convergence: `+0.0177330185 rad`
  - `navsat_transform_node` datum yaw: `-0.0177330185 rad`

The earlier HERE4 fix `28.747980800, 77.117433400` converts to approximately:

```text
map x = 529.862 m
map y = 324.136 m
```

It lands inside the selected boundary, on a road corridor, with static and road
cost both `0`. This validates the offline coordinate conversion. It does not by
itself validate the live IMU heading or TF configuration.

## Install and build

Copy this directory into any ROS 2 workspace:

```bash
cp -r dtu_prior_map <your_ws>/src/
cd <your_ws>
source /opt/ros/humble/setup.bash
colcon build --packages-select dtu_prior_map
source install/setup.bash
```

## Launch and inspect the two maps

```bash
ros2 launch dtu_prior_map dtu_prior_map.launch.py
```

The launch file starts and activates:

```text
/dtu_static_map
/dtu_road_mask
/dtu_road_filter_info
```

Check that both map messages arrive:

```bash
ros2 topic echo /dtu_static_map --once
ros2 topic echo /dtu_road_mask --once
```

In RViz, use fixed frame `map` and add two Map displays, one per map topic.

## Connect it to Nav2

Merge `config/nav2_prior_snippet.yaml` into your existing Nav2 configuration.
Do not replace the rest of your costmap configuration. In particular, keep the
live `/scan`, curb, and `/cloud/nonground` obstacle/voxel sources.

The intended stack is:

```text
dtu_static_map (hard prior) --+
dtu_road_mask (soft bias) ----+--> global_costmap --> SmacPlanner2D
live LiDAR obstacles ---------+
```

Use the `RoadAware` planner ID when sending a Nav2 goal. Tune
`cost_travel_multiplier` and the off-road mask cost only after viewing the
resulting global path.

## GPS/map alignment

`config/navsat_datum.yaml` contains the exact geographic coordinate for map
`(0,0)`. Its yaw is the negative UTM grid convergence, which cancels the
convergence added by `navsat_transform_node` and makes `/odometry/gps` equal to
the direct EPSG:32643 projection minus the recorded map origin. The recommended
TF ownership is:

```text
map -> odom       global robot_localization EKF using transformed RTK position
odom -> base_link local EKF using GenZ odometry + IMU
```

Important:

- Only one node may publish `map -> odom`.
- If the global EKF owns that transform, disable RTAB-Map's TF publication.
- Do not assume the heading is correct merely because GPS position is correct.
  The map follows the UTM grid; validate the IMU's ENU earth-referenced heading
  against a straight driven course before retaining yaw fusion.
- The rover's `base_link -> base_footprint` adapter rotates its legacy
  +Y-forward body convention by `+pi/2` into REP-103. The
  measured `base_footprint -> gps_antenna` lever arm is
  `[0.10, 0.05, 0.40] m` (forward, left, up), allowing
  `navsat_transform_node` to compensate for it through the TF chain.

The GPS/EKF fragment cannot safely replace your current EKF YAML without first
checking which pose, velocity, and yaw fields your existing topics publish.

## Regenerate after editing the QGIS map

The source GeoPackage and QGIS project are in `source/`. Install the generator's
offline dependencies if needed:

```bash
sudo apt install python3-geopandas python3-rasterio python3-pyproj python3-pil
```

Then run from the package directory:

```bash
python3 scripts/generate_nav2_maps.py \
  source/dtu_map.gpkg . \
  --resolution 0.25 \
  --offroad-cost 70
```

The generator validates the source CRS/geometries, rebuilds both maps, computes
and records `grid_convergence_radians` and its negative `datum_yaw_radians`, and
recreates the semantic preview. Copy the generated datum yaw into
`config/navsat_datum.yaml`; never replace it with zero.

Regenerate the centreline mask and road graph from the repository root with
GDAL, NumPy, and PyYAML installed:

```bash
sudo apt install python3-gdal python3-numpy python3-yaml
python3 -B src/dtu_prior_map/scripts/generate_centerline_navigation.py \
  src/dtu_prior_map/source/dtu_map.gpkg \
  src/dtu_prior_map \
  --snap-tolerance 1.0
```

This generator reads `roads`, `road_corridors`, `buildings`, and
`operational_boundary`, uses the source LineStrings rather than a raster
skeleton, and preserves the existing map grid.

## Files

```text
maps/dtu_static.pgm              hard occupancy map
maps/dtu_static.yaml             Nav2 map metadata
maps/dtu_road_mask.pgm           weighted road-preference mask
maps/dtu_road_mask.yaml          Nav2 mask metadata
maps/dtu_centerline_cost.pgm     normalized centreline preference
maps/dtu_centerline_cost.yaml    Nav2 centreline-mask metadata
maps/dtu_road_graph.json         routable source-road graph
maps/dtu_map_origin.yaml         exact GPS/UTM origin
maps/dtu_map_statistics.yaml     generation checks
config/nav2_prior_snippet.yaml   Nav2 merge fragment
config/navsat_datum.yaml         GPS datum fragment
launch/dtu_prior_map.launch.py   map/filter servers
preview/dtu_road_mask_preview.png
source/dtu_map.gpkg
source/dtu_map_project.qgz
```
