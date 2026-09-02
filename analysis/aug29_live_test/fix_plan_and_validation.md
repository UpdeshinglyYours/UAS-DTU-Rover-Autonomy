# August 29 obstacle-pipeline fix and validation

Date: 2026-08-29  
Scope: real-rover, read-only validation. No navigation goal, actuator command,
automatic Nav2 activation, flight-controller change, or EKF retuning was
performed.

## 1. Root causes confirmed

1. The recorded `/scan` topic has zero messages. Both DTU obstacle layers use
   `/scan`, so neither costmap received live obstacles.
2. Before this change, the DTU launch defaulted `input_cloud_topic` to
   `/nonground`. Patchwork++ was not launched and `/nonground` was not present.
   The scan converter therefore had no input.
3. Before this change, `controller_server` did not set `odom_topic`. The
   installed Humble implementation reads `odom_topic` in
   `/opt/ros/humble/include/nav_2d_utils/odom_subscriber.hpp` and defaults it to
   `odom`. The bag contains no `/odom` topic.
4. The August 29 run does not provide valid evidence for MPPI obstacle tuning:
   MPPI and both obstacle layers were blind because `/scan` was empty.

The 40 Hz local EKF and 30 Hz global EKF rates were healthy. Their output
frequency was not a cause of the missing obstacle observations and was not
changed.

## 2. Files changed

- `src/lirovo/launch/lirovo.launch.py`: restored the raw Blickfeld-to-scan
  defaults and corrected scan geometry; retained identity base-frame adapter.
- `src/lirovo/config/nav2_dtu_rtk_params.yaml`: restored known-working scan
  ranges and set controller odometry explicitly.
- `src/lirovo/lirovo/dtu_nav2_readiness.py`: added a read-only, fail-closed DTU
  readiness command.
- `src/lirovo/test/test_dtu_rtk_config.py`: added/updated focused regression
  coverage.
- `src/lirovo/setup.py`, `src/lirovo/package.xml`: installed the readiness
  command and declared its runtime dependencies.
- This report.

All unrelated tracked changes, deletions, untracked packages, map data, GenZ
configuration, and RTK localization work were left unchanged.

## 3. Exact old-versus-new scan configuration

Here “old” means the DTU configuration immediately before this task; “new” is
the restored physical-rover baseline.

| Setting | Old DTU value | New value |
|---|---:|---:|
| input topic default | `/nonground` | `/bf_lidar/point_cloud_out` |
| output topic default | `/scan` | `/scan` |
| target frame | `base_link` | `base_link` |
| transform tolerance | 0.05 s | 0.05 s |
| minimum height | 0.0 m | 0.2 m |
| maximum height | 2.0 m | 1.0 m |
| minimum angle | -3.14159 rad | -0.6293 rad |
| maximum angle | 3.14159 rad | 0.6293 rad |
| angle increment | 0.00872665 rad | 0.00872665 rad |
| scan time | 0.3 s | 0.1 s |
| minimum range | 0.43 m | 1.5 m |
| maximum range | 35.0 m | 41.0 m |
| use infinity | true | true |
| infinity epsilon | 1.0 | 1.0 |
| queue size | 50 | 50 |

The configured forward sector is approximately +/-36 degrees and produces
approximately 145 beams (`ceil((0.6293 - -0.6293) / 0.00872665)`). Patchwork++
and `/nonground` can still be selected through the launch argument, but neither
is required by default. The launch contains one `/scan` producer.

Costmap restoration:

| Setting | Old DTU local | New local | Old DTU global | New global |
|---|---:|---:|---:|---:|
| obstacle minimum | 0.43 m | 1.5 m | 0.43 m | 1.5 m |
| obstacle maximum | 20.0 m | 20.0 m | 30.0 m | 30.0 m |
| raytrace minimum | 0.43 m | 1.5 m | 0.43 m | 1.5 m |
| raytrace maximum | 20.0 m | 15.0 m | 30.0 m | 30.0 m |

Both layers retain `/scan`, `LaserScan`, marking, clearing, infinity-valid,
and the 0.0--1.0 m obstacle-height limits. Local costmap frequency, dimensions,
resolution, footprint, inflation radius, and inflation scaling are unchanged.

Safety note: the deliberately restored 1.5 m obstacle/raytrace minimum creates
a close-range observation limitation. Obstacles inside 1.5 m require careful
stationary and low-speed validation before any higher-speed physical run. This
task did not tune that value.

## 4. MPPI differences found

No difference was found between the current DTU `FollowPath` configuration and
the complete supplied known-working MPPI baseline. Plugin, horizon, batch,
noise, velocity bounds, temperature, gamma, motion model, all nine critics,
and all listed critic values match exactly. Classification: no A/B/C items;
no MPPI value was changed.

Runtime configure-only validation loaded
`nav2_mppi_controller::MPPIController` and all nine expected critics. The
controller remained inactive.

## 5. Smac2D differences found

| Historical baseline | Current DTU value | Class | Decision |
|---|---:|---|---|
| `SmacPlanner2D` | `SmacPlanner2D` | same | preserve |
| frequency 5.0 Hz | 5.0 Hz | same | preserve |
| tolerance 0.5 m | 0.5 m | same | preserve |
| `allow_unknown: true` | `false` | A | preserve hard, bounded prior-map behavior |
| downsample true | false | A | preserve DTU map/filter resolution |
| factor 2 | 1 | A | preserve DTU map/filter resolution |
| travel multiplier 4.0 | 3.0 | A | preserve centreline launch tuning |
| final approach orientation true | true | same | preserve |

The DTU static map and centreline mask are already 0.25 m/cell. Avoiding an
additional factor-two downsample preserves prior-map and road-mask detail.
`track_unknown_space` and `allow_unknown` remain false so the authoritative DTU
map defines the operational area. The 3.0 travel multiplier remains
launch-rewritable, and `enable_centerline_tether` still rewrites the global
road-preference filter. These settings are DTU-specific and were not reverted.

Additional current Smac limits/smoother values were not part of the supplied
historical comparison list. They were classified C (uncertain) and left
unchanged.

## 6. Controller odometry fix

Added:

```yaml
controller_server:
  ros__parameters:
    odom_topic: /odometry/filtered
```

This is the correct Humble parameter name and node scope. During safe launch
validation, only `controller_server` was manually configured. It transitioned
to `inactive`, subscribed to `/odometry/filtered`, had no `/odom` subscription,
and published no command message. No remapping workaround was added.

## 7. TF adapter and ownership result

- `base_link -> base_footprint`: translation `[0, 0, 0]`, identity quaternion,
  yaw 0.0 degrees. The obsolete +90 degree adapter was not restored.
- `base_footprint -> gps_antenna`: `[0.10, 0.05, 0.40]`, identity rotation.
- No static `map -> odom` transform exists in the launch.
- Local EKF remains configured with `world_frame: odom` and publishes
  `odom -> base_link`.
- Global EKF remains configured with `world_frame: map` and publishes
  `map -> odom`.

The bag contains only the expected dynamic edges: `map -> odom` at 30.0 Hz and
`odom -> base_link` at 40.0 Hz. The safe launch graph contained exactly the two
dynamic TF publisher endpoints, `ekf_filter_node_map` and `ekf_filter_node`.
Static TF runtime output confirmed zero yaw and the lever arm.

## 8. GenZ rate investigation

The filtered bag has 843 `/genz/odometry` messages and 843
`/genz/non_planar_points` messages. Both header-stamp cadences are approximately
6.675 Hz. Median storage-minus-header latency is 26.8 ms for odometry and
45.1 ms for the debug cloud. This confirms one common processed-cloud cadence,
but does not reveal the raw input cadence.

Neither `/bf_lidar/point_cloud_out` nor the deskewed cloud exists in the
filtered bag, and the original `aug29.bag` is not present in the workspace.
Therefore the Blickfeld rate, input-to-output drop ratio, and whether raw input
was also about 6.7 Hz cannot be measured from the available evidence.

Static evidence:

- No explicit GenZ rate throttle was found.
- The GenZ point-cloud subscription uses reliable `KeepLast(1)`, so a
  compute-bound callback can replace/drop queued frames rather than build an
  unbounded backlog.
- The user-owned current `rover.yaml` differs from tracked history by raising
  `desired_num_voxelized_points` from 2500 to 5500. The repository tuning guide
  explicitly warns that a value that is too large can overload the CPU.
- `max_num_iterations` is 100. DTU Nav2 concurrently ran MPPI at 20 Hz, a local
  costmap update target of 25 Hz, two EKFs, map/filter servers, and navsat.
- The recorded IMU worst gap was 29.3 ms, below the configured 60 ms GenZ IMU
  gap, so the bag does not support missing IMU samples as the primary rate
  cause.

Conclusion: the cause is unresolved because the decisive raw-cloud topic and
CPU telemetry were not recorded. Increased point count plus a depth-one input
queue is a concrete compute-load candidate, not proof. No GenZ parameter was
changed. The next run should record raw and deskewed cloud headers, GenZ timing
diagnostics, per-process CPU, and dropped-cloud counters.

## 9. RTK/global EKF investigation

No concrete fusion configuration error was found:

- GPS is fused once, through `/odometry/gps`; the global EKF does not also fuse
  raw NavSatFix.
- GenZ contributes differential X/Y motion; the GPS odometry contributes
  absolute X/Y; IMU contributes absolute yaw/yaw rate.
- All 127 raw GPS fixes and `/odometry/gps` messages have identical header
  stamps and identical X/Y covariance. Raw-fix receive latency is about 0.1 ms
  median; navsat output arrives about 117.7 ms after that preserved stamp.
- `/odometry/global` and `map -> odom` have approximately 3 ms median
  storage-minus-header latency, with maxima below 19 ms.
- Bag TF has one effective edge for each required dynamic transform.

The receiver was float for 82/127 samples and fixed for 45/127. Reported median
horizontal sigma was 2.4 cm during float and 1.4 cm during fixed. The float
covariance remains tight relative to the observed metre-scale corrections, so
covariance calibration/receiver semantics should be verified operationally,
but this is not enough evidence to invent an EKF change. Fusion parameters were
left unchanged. The next physical test should begin only after sustained RTK
Fixed and stable timestamps/covariance are observed.

## 10. Readiness utility

Run after sourcing the workspace:

```bash
ros2 run lirovo dtu_nav2_readiness
```

It samples for 5 seconds and prints PASS/FAIL for `/scan`, local/global
odometry, IMU, GPS, the four required direct TF edges, controller config, both
costmap scan sources, and the live controller odometry subscription. It reports
GPSRAW RTK Float/Fixed when available. It also reports TF publisher endpoint
counts and explains when node-to-edge ownership must be correlated manually.

It never publishes, calls lifecycle transitions, activates Nav2, or sends a
goal. It exits nonzero on missing/stale/slow required data. With no live rover
drivers after shutdown, the smoke test correctly returned FAIL while its three
static configuration checks passed.

## 11. Offline `/scan` replay result

`aug29_filtered.bag` contains `/scan` with zero messages and does not contain
`/bf_lidar/point_cloud_out`. The original `aug29.bag` is not present. Per the
requested decision rule, no substitute perception architecture was replayed.
Offline geometry and launch/configuration were validated statically and through
the safe live launch. Frame ID, finite-return fraction, and actual scan rate
cannot honestly be reported without the raw cloud.

## 12. Build and test results

| Command/check | Result |
|---|---|
| `colcon build --packages-select dtu_prior_map lirovo --symlink-install` | PASS, 2 packages |
| focused `test_dtu_rtk_config.py` | PASS, 9 tests |
| focused `ament_flake8` on touched Python/launch/test files | PASS, 3 files |
| `colcon test --packages-select dtu_prior_map lirovo` | command PASS; selected Python package runner reported 0 discovered tests |
| package-wide direct pytest | 9 pass, 1 skip, 2 legacy lint-wrapper failures |
| safe DTU RTK launch | PASS; Nav2 did not autostart |
| configure-only controller inspection | PASS; inactive, correct odom subscription, MPPI loaded |
| offline raw-cloud replay | NOT RUN; raw topic/original bag unavailable |

The legacy `test_flake8.py` and `test_pep257.py` call their linters with an empty
argument list from the repository root, so they scan the entire dirty workspace,
including generated and unrelated third-party files. Their failures were not
caused by the touched files; focused lint is clean. Direct pytest also required
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` because a user-site `anyio` plugin expects a
newer pytest than Ubuntu 22.04 provides.

Safe launch command:

```bash
ros2 launch lirovo lirovo.launch.py \
  use_sim_time:=false \
  global_localization:=dtu_rtk \
  dtu_nav2_autostart:=false
```

All Nav2 nodes initially remained `unconfigured`. `controller_server` alone was
configured for subscription inspection and remained `inactive`; planner and BT
navigator stayed `unconfigured`. No `/cmd_vel_nav` message was emitted in the
observation window, `/cmd_vel` did not exist, and shutdown via Ctrl+C completed
cleanly.

## 13. Remaining risks before the next physical test

1. Do not activate Nav2 until the readiness command passes with a sustained,
   fresh `/scan` rate and all dynamic TF/odometry/GPS checks.
2. Validate the intentional 1.5 m near-field blind region at stationary and
   low speed before higher-speed operation.
3. Begin under sustained RTK Fixed; float-period jumps remain an operational
   blocker.
4. Record raw Blickfeld and deskewed cloud topics plus CPU/drop diagnostics to
   resolve the 6.7 Hz GenZ cadence.
5. Perform authorized physical clear-road, obstacle-detour, and reachable
   off-road tests only with an operator, bounded area, emergency stop, speed
   limit, and explicit stop condition. None was authorized or run here.

## Focused worktree status

The full worktree was dirty before this task and remains so. Task-touched paths:

```text
 M src/lirovo/launch/lirovo.launch.py
 M src/lirovo/package.xml
 M src/lirovo/setup.py
?? analysis/aug29_live_test/fix_plan_and_validation.md
?? src/lirovo/config/nav2_dtu_rtk_params.yaml
?? src/lirovo/lirovo/dtu_nav2_readiness.py
?? src/lirovo/test/test_dtu_rtk_config.py
```

The three `??` DTU config/test paths already existed as user-owned untracked
work before this task except for the new readiness utility; their task changes
are the focused scan/controller/test additions described above. No files were
staged, committed, reset, restored, cleaned, or deleted.

## Concise terminal summary

```text
SCAN INPUT: /bf_lidar/point_cloud_out
SCAN OUTPUT: /scan
SCAN RATE: August 29 /scan = 0 Hz; corrected replay unavailable (raw bag topic absent)
CONTROLLER ODOM: /odometry/filtered (runtime subscription verified; no /odom)
LOCAL EKF RATE TARGET: 40.0 Hz
GLOBAL EKF RATE TARGET: 30.0 Hz
MPPI CONFIG: known-working baseline, unchanged; runtime load verified
SMAC2D CONFIG: preserved DTU-specific 0.25 m/no-downsample, multiplier 3.0
BASE_LINK -> BASE_FOOTPRINT YAW: 0.0 degrees
MAP -> ODOM OWNER: ekf_filter_node_map (global EKF)
ODOM -> BASE_LINK OWNER: ekf_filter_node (local EKF)
GENZ RATE CAUSE: unresolved; raw rate absent, 5500-point compute load is a candidate
RTK ISSUE: mostly Float with GPS-correlated jumps; no concrete EKF config bug found
TESTS: build PASS; focused config 9/9 PASS; focused lint PASS; safe launch PASS
NEXT LIVE TEST BLOCKERS: readiness PASS, sustained RTK Fixed, close-range/low-speed obstacle validation, GenZ/raw-cloud rate evidence
```
