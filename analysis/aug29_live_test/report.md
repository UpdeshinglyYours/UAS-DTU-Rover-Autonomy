# August 29 DTU live rover test — offline bag analysis

## 1. Executive summary

The dominant obstacle-avoidance failure is upstream of Nav2 tuning: `/scan`, the sole configured obstacle source for both costmaps, recorded **zero messages**. The cloud-to-scan launch defaults to `/nonground`, but that topic is absent; `/genz/non_planar_points` exists at only 6.67 Hz and was not consumed by Nav2. The controller also lacks an explicit `odom_topic`; ROS 2 Humble Nav2 defaults it to `/odom`, and this bag has no `/odom` publisher. These two wiring gaps are stronger explanations than MPPI critic weights, footprint, inflation, or the centreline tether.

The rover's global pose was a median **0.492 m** from the mapped road centreline (p95 **2.208 m**). The collision-aware plan itself was much more centred: 93.0% of recorded plan poses were within 0.5 m. Deviations were not consistently on one side; the longest interval above 2 m was 10.13 s (t=87.40–97.54 s), during prolonged hard steering and RTK-float operation.

Restoring EKF output to 30 Hz is **not justified**: `/odometry/filtered` was 40.00 Hz and `/odometry/global` was 30.00 Hz with no gaps above 50 ms. The raw GenZ rate was lower than expected at 6.68 Hz, but the EKFs predicted at healthy rates.

## 2. Bag filtering result

- Original: 8.848 GiB.
- Filtered: 82.333 MiB.
- Reduction: 99.09%.
- Filtered duration: 126.423504 s.
- Start/end: `2026-08-29T11:39:48.505532+00:00` / `2026-08-29T11:41:54.929036+00:00`.
- Retained topic records: 27 (23 non-empty); messages: 86019.
- `/genz/non_planar_points` is retained only because `/scan` is empty; it is the smallest recorded non-planar proxy cloud. Both ~3.5 GiB global costmap streams and duplicate raw costmaps were excluded.

| Topic | Type | Messages |
|---|---|---:|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 1914 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | 1896 |
| `/dtu_centerline_filter_info` | `nav2_msgs/msg/CostmapFilterInfo` | 1 |
| `/dtu_reference_route` | `nav_msgs/msg/Path` | 2 |
| `/genz/non_planar_points` | `sensor_msgs/msg/PointCloud2` | 843 |
| `/genz/odometry` | `nav_msgs/msg/Odometry` | 843 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 0 |
| `/gps/filtered` | `sensor_msgs/msg/NavSatFix` | 3382 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 1055 |
| `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 3160 |
| `/mavros/global_position/compass_hdg` | `std_msgs/msg/Float64` | 27140 |
| `/mavros/global_position/raw/fix` | `sensor_msgs/msg/NavSatFix` | 127 |
| `/mavros/global_position/raw/gps_vel` | `geometry_msgs/msg/TwistStamped` | 127 |
| `/mavros/global_position/raw/satellites` | `std_msgs/msg/UInt32` | 127 |
| `/mavros/gpsstatus/gps1/raw` | `mavros_msgs/msg/GPSRAW` | 127 |
| `/mavros/gpsstatus/gps1/rtk` | `mavros_msgs/msg/GPSRTK` | 0 |
| `/mavros/gpsstatus/gps2/raw` | `mavros_msgs/msg/GPSRAW` | 127 |
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | 27134 |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 5054 |
| `/odometry/global` | `nav_msgs/msg/Odometry` | 3792 |
| `/odometry/gps` | `nav_msgs/msg/Odometry` | 127 |
| `/plan` | `nav_msgs/msg/Path` | 93 |
| `/plan_smoothed` | `nav_msgs/msg/Path` | 0 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 0 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 8850 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 5 |
| `/unsmoothed_plan` | `nav_msgs/msg/Path` | 93 |

## 3. RTK/global localization quality

`/odometry/global` vs `/odometry/gps`: median 0.229 m, mean 0.322 m, p95 0.954 m, max 1.391 m. The receiver reported RTK fixed for only 35.4% of samples and float for 64.6%. During fixed periods the global-vs-GPS median was 0.077 m; during float it was 0.291 m.

The global EKF made 34 consecutive position steps above 0.5 m at a nominal 30 Hz; the largest was 1.348 m. 33 of those 34 steps occurred within 0.1 s of a GPS-odometry update, supporting delayed/discrete GPS correction rather than physical motion over ~33 ms. GPS's own 1 Hz position step had p95/max 1.154/1.663 m. Heading was much smoother (maximum consecutive change 4.37°).

Reported global XY sigma did not grow monotonically: first/median/p95/last were 0.271/0.234/0.318/0.112 m. GPS1 reported a median horizontal-accuracy field of 20.0 mm and 31 satellites, but the prolonged float status and correction steps mean global localization was not consistently smooth enough to call fully acceptable.

The bag provides no independent ground truth. Consequently, it cannot uniquely divide every centreline deviation into physical tracking error versus GNSS/global-estimator error. The strong status correlation is evidence of a localization contribution, not proof that all float-period error was GNSS error.

## 4. Centerline tracking quality

The actual tether geometry comes from `src/dtu_prior_map/maps/dtu_road_graph.json`, which generates the recorded centreline cost mask. `/dtu_reference_route` is also recorded, but it is a goal-specific obstacle-independent route and includes short start/goal connectors, so the required nearest-centreline statistics use all mapped road graph segments.

- Median: **0.492 m**; mean: **0.810 m**; p95: **2.208 m**; max: **3.524 m**.
- Within 0.5/1.0/1.5/2.0 m: 50.7% / 70.2% / 82.5% / 88.5%.
- RTK fixed centreline median/p95: 0.239/0.945 m; RTK float: 0.742/2.513 m.
- Longest sustained >2 m: t=87.40–97.54 s, 10.13 s, mean/max 2.316/3.392 m.
- Direction: nearest graph-edge sign was positive for 60.5% with signed median 0.204 m. Edge directions are not lane directions, and the recorded goal-route sign also alternates; the data does not support a systematic one-side bias.
- Turn correlation was weak overall (Pearson correlation with `|cmd_vel.angular.z|` 0.027), though the longest high-error interval coincided with sustained maximum steering. Replanning was steady at 0.971 Hz; the plan remained near the mapped road, so large rover-pose errors were not primarily caused by the global planner choosing an off-centre route.

Plots: `centerline_tracking.png`, `centerline_error_vs_time.png`.

## 5. Odometry and sensor rates

| Topic | Messages | Mean Hz | Median dt | p95 dt | Worst gap | >0.05 s | >0.10 s | >0.20 s | >0.50 s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `/genz/odometry` | 843 | 6.675 | 0.1484 | 0.1737 | 0.1970 | 842 | 842 | 0 | 0 |
| `/odometry/filtered` | 5054 | 39.999 | 0.0250 | 0.0259 | 0.0346 | 0 | 0 | 0 | 0 |
| `/odometry/gps` | 127 | 1.000 | 1.0000 | 1.0336 | 1.0351 | 126 | 126 | 126 | 126 |
| `/odometry/global` | 3792 | 30.000 | 0.0333 | 0.0342 | 0.0432 | 0 | 0 | 0 | 0 |
| `/mavros/imu/data` | 27134 | 214.699 | 0.0042 | 0.0078 | 0.0293 | 0 | 0 | 0 | 0 |
| `/scan` | 0 | 0.000 | n/a | n/a | n/a | 0 | 0 | 0 | 0 |
| `/cmd_vel_nav` | 1896 | 20.010 | 0.0497 | 0.0627 | 0.0752 | 946 | 0 | 0 | 0 |
| `/cmd_vel` | 1914 | 20.000 | 0.0500 | 0.0507 | 0.0581 | 940 | 0 | 0 | 0 |

The prior expectation was GenZ ~11 Hz, IMU 400+ Hz, filtered odometry 25–30 Hz. This run instead recorded GenZ 6.68 Hz, IMU 214.70 Hz, local filtered odometry 40.00 Hz, and global odometry 30.00 Hz. The EKF outputs were not slow.

Odometry consumers from the repository configuration:

- `bt_navigator`: `/odometry/filtered`.
- `velocity_smoother`: `/odometry/filtered`, but configured `OPEN_LOOP`.
- local costmap: pose via TF `odom -> base_link`, published by the 40 Hz local EKF.
- `controller_server`: no `odom_topic` configured, therefore Humble default `/odom`; no such publisher/topic exists in the bag. MPPI therefore lacked recorded velocity feedback even though TF pose updates were healthy.

## 6. Obstacle avoidance findings

Both costmap obstacle layers are configured exclusively for `/scan`, with marking and clearing enabled. `/scan` has zero messages. Thus obstacle entry time, true forward range, steering latency, braking latency, scan age, and scan-to-costmap latency cannot be reconstructed from the intended perception stream.

The local occupancy grid published at 8.34 Hz versus a configured 10 Hz publication rate (internal update configured 25 Hz). This topic rate does not prove the internal update loop was slow; with no scan, there was no live obstacle observation to update. The only proxy point within 4 m occurred at t=72.50 s, range 2.24 m. The command was already 0.25 m/s and -0.90 rad/s; its local-costmap cell was 0.0. Because it was a single isolated sample, steering and speed-reduction latency are not measurable.

Current relevant settings: controller 20 Hz; MPPI horizon 2.80 s (56 × 0.05 s); max 0.9 m/s and 0.9 rad/s; footprint `[[0.50, 0.45], [-0.50, 0.45], [-0.50, -0.45], [0.50, -0.45]]`; inflation 0.80 m; collision margin 0.25 m; local transform tolerance 0.30 s; deceleration limits [-4.0, 0.0, -6.0].

The centreline tether acts in the **global planner** through the global costmap filter and `cost_travel_multiplier=3.0`. MPPI's local obstacle critic acts in the **local controller**. Because no scan reached either obstacle layer, this bag cannot show the tether overpowering a correctly marked obstacle. The plan being 93% within 0.5 m is expected when the planner is blind to live obstacles, not evidence that the weight is too strong.

## 7. Ranked likely causes of poor avoidance

1. **HIGH** — No /scan messages reached the recorded Nav2 obstacle input, leaving live obstacle marking and clearing without observations.
2. **HIGH** — controller_server omitted odom_topic, so Humble defaulted to /odom; the bag contains no /odom publisher, while /odometry/filtered was only configured for BT navigator and velocity smoother.
3. **MEDIUM** — Global localization was discontinuous during mostly RTK-float operation: 34 global EKF position steps exceeded 0.5 m and centerline error was substantially worse in float periods.

The 6.67 Hz upstream cloud/GenZ cadence is a secondary rate concern: at 0.9 m/s the rover moves about 0.135 m per cloud interval. It is not the main failure because Nav2 received no scan at any rate.

## 8. Recommended changes for the next rover test

### HIGH CONFIDENCE

- Make the point-cloud-to-LaserScan input match the actually published nonground/obstacle cloud; gate Nav2 activation on a nonzero, healthy /scan rate and verify live obstacle cells in the local costmap.
- Set controller_server.ros__parameters.odom_topic to /odometry/filtered and verify the controller subscription before motion.
- Record /scan explicitly with compatible sensor-data QoS plus the compact obstacle cloud in the next diagnostic bag.

### MEDIUM CONFIDENCE

- Investigate why GenZ/cloud cadence was 6.67 Hz rather than the previously observed ~11 Hz; retest stopping/steering margin at the actual maximum rover speed.
- Gate or reduce navigation speed when HERE4 is RTK float, and tune global GPS fusion only after checking time synchronization/covariance because the global EKF made repeated sub-second correction jumps.

### DO NOT CHANGE YET

- Do not reduce centerline cost_travel_multiplier solely from this run; the planner had no recorded obstacle input and 93.0% of plan samples stayed within 0.5 m of a mapped centerline.
- Do not change footprint, inflation, MPPI obstacle weights, or deceleration limits until a run with a validated /scan stream shows inadequate clearance despite correctly marked obstacles.
- Do not lower or restore EKF output rates: /odometry/filtered was 40.0 Hz and /odometry/global was 30.0 Hz, already above/at the requested 25-30 Hz range.

## 9. Things NOT justified by the evidence

- Restoring `/odometry/filtered` to ~30 Hz: it already ran at 40.0 Hz, while `/odometry/global` ran at 30.0 Hz.
- Blaming slow controller execution: `/cmd_vel_nav` and `/cmd_vel` were both ~20 Hz, matching configuration.
- Blaming small inflation, an undersized footprint, weak obstacle critic, or insufficient configured deceleration before validating that obstacles reach the costmap.
- Blaming the centreline tether for unsafe clearance: no recorded scan allowed the global planner or MPPI obstacle critic to compare centreline preference against live obstacles.
- Claiming a measured obstacle-response latency or physical GPS ground-truth error from this bag.

## 10. Exact commands/scripts used

```bash
source /opt/ros/humble/setup.bash
ros2 bag info aug29.bag
python3 analysis/scripts/filter_rosbag.py aug29.bag aug29_filtered.bag
ros2 bag info aug29_filtered.bag
python3 analysis/aug29_live_test/analyze_aug29.py
```

Filtering is implemented in `analysis/scripts/filter_rosbag.py`; analysis and plotting are implemented in this directory's `analyze_aug29.py`. The original bag was opened read-only and was not altered. No ROS nodes, live launch, navigation goals, or motion commands were started.
