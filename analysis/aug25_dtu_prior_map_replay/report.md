# August 25 bag over the DTU prior map

## Outcome

The current August 27 `dtu_rtk` pipeline does **not** place this bag correctly
on the DTU prior map. The prior map itself appears broadly georeferenced
correctly; most of the large error is introduced by the localization pipeline.

The raw WGS84 fixes, converted directly into the map's native EPSG:32643 UTM
coordinates and offset by the raster origin, follow the mapped road centreline
with a median distance of `0.965 m` and a 95th-percentile distance of `4.214 m`.
All 112 fixes land in free cells of the hard static map.

In contrast, `navsat_transform_node` rotates those same coordinates by exactly
`-1.016027 degrees` about the map origin. That is the local UTM meridian
convergence reported by the node (`0.017733 rad`), and it creates a
`9.759-10.392 m` offset at the recorded route. The rotation fit has less than
`0.000002 m` maximum residual, so this is a coordinate-frame convention error,
not random GPS drift.

The global EKF also starts at map `(0, 0)` while the direct GPS position is
`(446.802, 379.244) m`, initially putting the estimate `586.053 m` away. It
takes roughly one minute to approach the recorded route and never removes the
coordinate rotation error. During the final 60 seconds, the global EKF remains
`11.043 m` from direct UTM GPS on average; during the final 10 seconds it is
`10.988 m` away on average. Its final centreline distance is `5.734 m`.
For the first `17.50 s`, the false convergence trajectory also passes through
cells that the hard prior map marks occupied/outside; Nav2 was inactive, so no
planning or motion command was produced from that invalid pose.

## Interpretation

This replay does not support abandoning the DTU prior map. The direct GPS/map
overlay is much better than the live localization result. Two localization
issues should be resolved first:

1. The prior raster is already expressed in UTM grid coordinates, while the
   present `navsat_transform_node` output applies the UTM meridian-convergence
   rotation. `/odometry/gps` must be made numerically consistent with
   `UTM(fix) - map_origin` before it is fused.
2. The global EKF publishes an origin-based estimate before the absolute GPS
   correction has initialized it, then converges very slowly from a 586 m
   innovation. The global filter/TF should be seeded or gated on a valid global
   position with an appropriate initial covariance before Nav2 can activate.

The bag does not contain the MAVROS RTK-status topic, so this run cannot prove
that the receiver was RTK-fixed. The recorded `NavSatFix` starts with about
`3.18 m^2` X/Y variance (about `1.78 m` standard deviation). IMU heading
calibration also still needs a separate check before any autonomous run.

## Runtime procedure

Scope was real-rover read-only bag replay. No physical connection, actuation,
RC override, navigation goal, or Nav2 activation was used.

The source overlay was built and tested with:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dtu_prior_map lirovo --symlink-install
source install/setup.bash
colcon test --packages-select dtu_prior_map lirovo
colcon test-result --verbose
```

Result: both packages built; 51 tests passed, 0 failures, 6 skipped.

The timing-correct replay used an isolated ROS domain and the current launch:

```bash
export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lirovo lirovo.launch.py \
  use_sim_time:=true \
  global_localization:=dtu_rtk \
  dtu_nav2_autostart:=false \
  enable_centerline_tether:=true \
  cost_travel_multiplier:=3.0
```

Only original sensor inputs were replayed. Recorded `/tf` and
`/odometry/filtered` were intentionally excluded so they could not compete
with or bypass the current EKFs:

```bash
ros2 bag play /home/aryaman/Desktop/aug25.bag \
  --clock 100 --rate 1.0 \
  --topics \
    /genz/odometry \
    /mavros/global_position/raw/fix \
    /mavros/imu/data \
  --disable-keyboard-controls
```

The timing-correct output contains 112 `/odometry/gps` messages, 3,351 global
EKF messages, and 4,467 local EKF messages. An earlier 2x exploratory replay is
also retained, and it produced materially identical coordinate and trajectory
results.

Lifecycle checks during replay showed:

- `/dtu_static_map_server`: active
- `/dtu_centerline_cost_server`: active
- `/planner_server`, `/controller_server`, `/bt_navigator`: unconfigured
- `/tf` publishers: local EKF and global EKF only, owning different TF edges

All launched processes were stopped with SIGINT and exited cleanly.

## Evidence files

- `aug25_dtu_overlay.png`: full relocation, route zoom, and error timeline
- `summary.json`: machine-readable metrics
- `gps_samples.csv`: direct UTM and `navsat_transform` coordinates per fix
- `global_ekf_samples.csv`: global EKF track and error per output sample
- `current_pipeline_output_1x/`: timing-correct replay output bag
- `current_pipeline_output/`: initial 2x comparison output bag
- `analyze_replay.py`: reproducible analysis and plotting script

No rover source/configuration file was changed. The analysis artifacts above
are the only files created by this task. Navigation behavior, planning, LiDAR
obstacle response, and physical motion were deliberately not tested because
the request was to determine where the recorded localization lands on the map.
