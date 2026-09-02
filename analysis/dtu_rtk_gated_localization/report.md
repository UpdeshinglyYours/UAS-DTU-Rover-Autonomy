# DTU gated global localization report

Date: 2026-08-31

Scope: real-rover read-only implementation and offline/synthetic validation.
No navigation goal, motion command, RC override, arming command, parameter
write to hardware, or physical actuation was performed.

## 1. Final architecture

```text
/initialpose (map -> base_link clicked pose)
  -> manual_global_alignment
  -> /ekf_filter_node_map/set_pose
  -> global EKF (only map -> odom owner)

/genz/odometry + /mavros/imu/data
  -> local EKF -> /odometry/filtered + odom -> base_link
  -> global EKF (differential X/Y + yaw/yaw rate)

/mavros/gpsstatus/gps1/raw + /mavros/global_position/raw/fix
  -> rtk_fix_gate
  -> /mavros/global_position/rtk_fixed_only
  -> navsat_transform
  -> /odometry/gps
  -> global EKF
```

The gate and manual alignment nodes do not publish TF. The local EKF remains
the only `odom -> base_link` owner and the global EKF remains the only
`map -> odom` owner. The zero-yaw `base_link -> base_footprint` transform and
the `[0.10, 0.05, 0.40] m` `base_footprint -> gps_antenna` transform are
unchanged.

## 2. robot_localization reset/set-pose API

The installed ROS 2 Humble `robot_localization` version is 3.5.4. Inspection
of the installed interface and source found both supported mechanisms:

- relative topic `set_pose`, type
  `geometry_msgs/msg/PoseWithCovarianceStamped`;
- relative service `set_pose`, type `robot_localization/srv/SetPose`.

`SetPose.srv` is exactly:

```text
geometry_msgs/PoseWithCovarianceStamped pose
---
```

The filter callback clears its global measurement queues/history, transforms
the supplied pose into `world_frame`, sets the state/covariance, and ignores
sensor messages at or before the reset stamp. The service was selected because
its future confirms that the synchronous callback completed. Since two EKFs
otherwise expose the same root-relative name, the global EKF remaps only its
interface to `/ekf_filter_node_map/set_pose`; the local EKF retains `/set_pose`.
There is no direct TF workaround and the local filter is never reset.

## 3. Manual alignment math

The RViz click is interpreted as the desired current `T_map_base`. The node
looks up the current `T_odom_base` and computes the alignment for validation
and logging:

```text
T_map_odom = T_map_base_clicked * inverse(T_odom_base_current)
```

It sends the clicked `map -> base_link` pose, including its quaternion and
covariance, to the global filter's supported set-pose service. A runtime test
clicked `(110, 210, yaw=0.3 rad)` while the local pose was approximately
`(22.48, 1.0)`. The global pose moved to the clicked location and yaw while
the local pose continued unchanged. A click with yaw different from the
absolute IMU was accepted intact, then the configured absolute IMU naturally
corrected the global yaw; therefore the live IMU heading must agree with the
physically observed clicked direction.

Messages with a zero, older-than-2-second, or more-than-0.5-second future
timestamp are rejected. A non-`map` frame, missing TF, unavailable service, or
already-pending request is also rejected with a clear log.

## 4. RViz workflow

1. Start MAVROS, Blickfeld, GenZ, and `lirovo` in `dtu_rtk` mode with
   `dtu_nav2_autostart:=false`.
2. Open RViz and set Fixed Frame to `map`.
3. Choose **2D Pose Estimate**.
4. Click the rover's actual physical map position and drag the arrow in the
   rover's actual forward direction.
5. Release and wait for `MANUAL GLOBAL ALIGNMENT ACCEPTED`.
6. Inspect `/dtu_global_localization/status`, `/odometry/global`,
   `map -> odom`, `odom -> base_link`, and `map -> base_link`.
7. Activate Nav2 only after localization, heading, TF ownership, maps, and
   sensor data are verified.

Inspect the state with:

```bash
ros2 topic echo /dtu_global_localization/status --once --full-length
```

The transient-local JSON string includes state, fix type, consecutive count,
gate state, both quality values, manual initialization, re-entry distance, and
the rejection reason.

## 5. RTK gate conditions

The verified installed `mavros_msgs/msg/GPSRAW` constants are RTK Float `5`
and RTK Fixed `6`. GPS is republished only when:

- the latest fresh GPSRAW status is Fixed;
- at least 10 consecutive Fixed/quality-passing GPSRAW samples were received;
- GPSRAW `h_acc`, when valid, satisfies
  `(h_acc_mm / 1000)^2 <= 0.25 m²`;
- the current NavSatFix has known finite covariance and
  `max(covariance_x, covariance_y) <= 0.25 m²`;
- status and fix arrival/header ages are within 1.5 seconds;
- re-entry is sane when an anchor already exists.

All limits and topic/service/frame names are parameters in
`config/dtu_rtk_gate.yaml`.

## 6. Re-entry sanity behavior

After a manual or previous RTK anchor, the first otherwise-eligible Fixed fix
is converted to map coordinates with `/navsat_transform/fromLL`. The gate reads
the existing `base_link -> gps_antenna` TF and removes the rotated lever arm,
then compares candidate base X/Y with `/odometry/global`. More than 2 m keeps
the gate closed and requires manual realignment or a future deliberate policy.
No interpolation, snap, filter reset, or automatic reinitialization is used.

Synthetic integration rejected a 229.873 m candidate and accepted a 0.125 m
candidate only after 10 Fixed samples. (The latter test was run before the
lever-arm precision refinement; the final pure-function test verifies exact
lever-arm removal.) A transition back to Float immediately closed the gate,
and the global pose continued by 0.100 m in the next second from GenZ/IMU.

## 7. navsat_transform odometry input decision

The input remains `/odometry/global`. The installed Humble 3.5.4
`dual_ekf_navsat_example.launch.py` explicitly remaps navsat's
`odometry/filtered` input to the global EKF output. With the configured manual
datum, `setManualDatum()` internally fixes the datum at world `(0,0,0)`, so
the datum/map conversion does not inherit a pre-click global pose. Preserving
the installed reference pattern also preserves the validated August 25 datum
and filtered-GPS behavior. Changing to local odometry was therefore not
justified by the apparent data-flow loop alone.

## 8. Global EKF with GPS absent

The global filter still runs at 30 Hz and retains:

- differential X/Y from `/genz/odometry`;
- absolute yaw and yaw rate from `/mavros/imu/data`;
- optional absolute X/Y from `/odometry/gps`.

When the gate is closed, the optional input simply stops. Synthetic and bag
tests both observed continued `/odometry/global` publication and motion with
zero gated fixes and zero new `/odometry/gps` messages.

## 9. Offline `30aug1.bag` result

Read-only SQL/message inspection of the full bag found 168 GPSRAW/NavSatFix
pairs: 163 RTK Float and 5 DGPS, with no RTK Fixed. GPSRAW `h_acc` ranged from
0.042 to 0.379 m; NavSatFix horizontal variance ranged from 0.001764 to
0.143641 m². Fix quality was numerically good, but every sample was rejected
because none was Fixed.

A 20-second, selected-topic replay starting 30 seconds into the bag used
`--clock 100 --rate 1.0` and replayed only GenZ, IMU, raw fix, and GPSRAW.
Recorded `/tf`, `/tf_static`, `/odometry/filtered`, `/odometry/global`,
`/odometry/gps`, `/cmd_vel`, and actuator topics were not replayed. The monitor
observed:

- 13 GPSRAW samples, all Float;
- 0 gated NavSatFix and 0 `/odometry/gps` outputs;
- 539 local EKF and 405 global EKF outputs;
- a test `/initialpose` accepted during replay;
- final state `MANUAL_INITIALIZED`, gate closed;
- local pose changed from `(7.706, -4.163)` to `(29.102, -9.700)` and global
  pose continued after manual alignment.

Because the bag has no Fixed interval, Fixed-count, covariance, 229.873 m
rejection, 0.125 m acceptance, and Float-drop behavior were exercised with a
non-actuating synthetic ROS stream.

## 10. Files changed

- `src/lirovo/lirovo/manual_global_alignment.py` (new)
- `src/lirovo/lirovo/rtk_fix_gate.py` (new)
- `src/lirovo/config/dtu_rtk_gate.yaml` (new)
- `src/lirovo/launch/lirovo.launch.py`
- `src/lirovo/setup.py`
- `src/lirovo/package.xml`
- `src/lirovo/test/test_gated_global_localization.py` (new)
- `src/lirovo/README.md`
- `analysis/dtu_rtk_gated_localization/report.md` (new)

`dtu_prior_map`, Nav2 obstacle layers, Smac2D, MPPI, EKF frequencies,
`base_link -> base_footprint`, and the GPS lever arm were deliberately left
unchanged.

## 11. Tests run

```text
python3 -m flake8 <new/touched Python files>          PASS
python3 -m pytest test_gated_global_localization.py  12 passed
colcon build --packages-select lirovo dtu_prior_map
  --symlink-install                                  PASS (2 packages)
colcon test --packages-select lirovo dtu_prior_map   PASS
colcon test-result --verbose                         51 tests, 0 failures
```

The exact requested real-clock launch started both filters, gate, manual node,
navsat, map servers, and inactive Nav2. Runtime evidence showed:

- `/ekf_filter_node_map/set_pose` has the exact SetPose type;
- `/navsat_transform/fromLL` has the exact FromLL type;
- planner and controller were both `unconfigured`;
- `/tf` had only the local and global EKF publishers;
- the gated topic had one gate publisher and one navsat subscriber with
  compatible best-effort QoS;
- both managed map stacks were active;
- SIGINT shutdown completed cleanly after the shutdown guard fix.

## 12. Remaining risks before a live test

- No RTK-Fixed sample exists in `30aug1.bag`; live HERE4 Fixed acquisition and
  Fixed-to-Float transitions still need read-only observation.
- The absolute MAVROS IMU yaw must be ENU, earth-referenced, calibrated, and
  consistent with the RViz arrow. Otherwise it will correct the clicked yaw.
- GPSRAW and NavSatFix are paired by current freshness/receiver stream rather
  than exact-time synchronization; validate their live stamps and rates.
- Validate `h_acc` semantics and NavSatFix covariance against current HERE4
  firmware/MAVROS in Fixed mode before changing the 0.25 m² limit.
- The workspace overlay prints pre-existing missing-package setup warnings;
  they did not prevent this build or launch but should be cleaned separately.
- No physical motion, Nav2 activation, autonomous goal, obstacle detour, or
  actuator-path test was authorized or performed.
