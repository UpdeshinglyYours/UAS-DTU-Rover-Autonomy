# DTU physical barricade course

This optional scenario adds four physical Gazebo barricades to the existing
clear-road centreline test. It does not alter `dtu_static.pgm`,
`dtu_centerline_cost.pgm`, `dtu_road_graph.json`, or
`/dtu_reference_route`. LiDAR observations and the existing live obstacle
layers are the only way the barricades enter Nav2's costmaps.

## Model and measured placement

Every entity uses the same static SDF box. Visual and collision geometry are
identical:

```text
thickness X = 0.16 m
width Y     = 1.60 m
height Z    = 2.00 m
centre Z    = 1.00 m
colour      = safety orange
```

The thin model X axis is aligned with the local road tangent, placing its long
Y axis across the road. Positions were derived at measured arc lengths along
the reference route from start `(487.055800, 393.518522)` to road goal
`(517.625, 359.625)`. Each centre is offset 0.65 m alternately left and right
of the original road LineString.

| Entity | Route distance | Offset | Map X | Map Y | Yaw (rad) | Yaw (deg) |
| --- | ---: | --- | ---: | ---: | ---: | ---: |
| `dtu_barricade_1` | 22 m | left | 475.674465 | 378.623476 | -1.895832 | -108.62 |
| `dtu_barricade_2` | 40 m | right | 478.468571 | 366.197206 | -0.252160 | -14.45 |
| `dtu_barricade_3` | 56 m | left | 494.286929 | 363.464159 | -0.252160 | -14.45 |
| `dtu_barricade_4` | 70 m | right | 507.777965 | 359.540692 | -0.053098 | -3.04 |

The four complete box footprints were sampled against the checked-in static
and road masks; every sample has static cost 0 and road-mask cost 0. Local road
half-width is approximately 3.49-3.50 m. The alternating placements leave the
following physical corridor widths around each box:

| Entity | Left passage | Right passage | Nearest static lethal cell |
| --- | ---: | ---: | ---: |
| `dtu_barricade_1` | 2.05 m | 3.34 m | 21.58 m |
| `dtu_barricade_2` | 3.34 m | 2.05 m | 26.58 m |
| `dtu_barricade_3` | 2.04 m | 3.35 m | 18.35 m |
| `dtu_barricade_4` | 3.34 m | 2.05 m | 19.83 m |

The BCR Nav2 footprint is 1.00 by 0.72 m. Even the narrower physical passage
is 1.32 m wider than the full robot width, and the opposite side is wider
still. The barricades are separated by 12.74, 16.05, and 14.05 m in Euclidean
space, leaving clear road between manoeuvres. They are 18.75 m from the start
and at least 9.85 m from the goal. The route remains traversable off-road as a
fallback, and no building or operational-boundary geometry is nearby.

## Build

From the autonomy repository root:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/q_ws/install/bcr_bot/share/bcr_bot/package.bash"
colcon build --packages-select dtu_prior_map uas_dtu_gz_sim --symlink-install
source install/setup.bash
```

## Normal RViz launch

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  enable_centerline_tether:=true \
  cost_travel_multiplier:=3.0 \
  spawn_barricade_course:=true
```

## Headless validation launch

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  headless:=true use_rviz:=false \
  enable_centerline_tether:=true \
  cost_travel_multiplier:=3.0 \
  spawn_barricade_course:=true
```

Send the documented road goal through the entire course:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 517.625, y: 359.625, z: 0.0}, orientation: {w: 1.0}}}}"
```

To disable the scenario, omit the argument or set it false:

```bash
ros2 launch uas_dtu_gz_sim bcr_bot_dtu_sim.launch.py \
  spawn_barricade_course:=false
```

The existing `spawn_test_obstacle` switch is independent and unchanged.

## Recorded headless validation

Validation on 2026-08-26 used the exact headless launch and road goal above.
All required lifecycle nodes reached `active`, `map -> base_link` resolved,
and `/tf` and `/tf_static` had their expected disjoint bridge,
robot-state-publisher, and static-map-transform owners. Gazebo listed all four
unique barricade entities. `/bcr_bot/scan` ran at 20 Hz and produced finite
returns from the course; each obstacle reached occupancy 100 in both global
and local live costmaps.

The first monitored forward traversal generated 104 `/plan` updates while the
reference route had one update and one unique geometry (79.998 m). The rover
reached `SUCCEEDED` without a recovery and without crossing a lethal static
cell. It passed every barricade with positive oriented-footprint clearance and
then returned toward the reference:

| Entity | Max `/plan` deviation | Max rover deviation | Min footprint clearance | Closest return to reference |
| --- | ---: | ---: | ---: | ---: |
| `dtu_barricade_1` | 1.136 m | 1.025 m | 0.480 m | 0.012 m |
| `dtu_barricade_2` | 0.794 m | 0.775 m | 0.251 m | 0.000 m |
| `dtu_barricade_3` | 1.054 m | 0.963 m | 0.438 m | 0.001 m |
| `dtu_barricade_4` | 0.924 m | 0.824 m | 0.286 m | 0.003 m |

The measured rover trajectory was 77.271 m: 55.07% lay within 0.5 m of the
reference, 98.43% within 1.0 m, and 100% within 2.0 m. It remained entirely on
the road mask and crossed zero lethal static cells.

A repeat forward goal also reached `SUCCEEDED`. The existing
`dtu_path_metrics.py` tool captured its live `/plan` at 75.405 m versus a
79.446 m reference: 86.42%, 97.71%, and 100% were within 0.5, 1.0, and 2.0 m,
respectively. Its minimum live-costmap clearance was 0.293 m, with zero live
or static lethal cells crossed. Compact topic-derived evidence is checked in
at `validation/barricade_course.json`; no map or road-graph artifact was
modified.
