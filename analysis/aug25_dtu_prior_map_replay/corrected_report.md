# August 25 DTU RTK correction replay

The corrected pipeline passes every offline acceptance criterion. This was a
1× replay with a 100 Hz bag clock. Only `/genz/odometry`,
`/mavros/global_position/raw/fix`, and `/mavros/imu/data` were replayed;
recorded `/tf` and `/odometry/filtered` were excluded.

| Metric | Original | Corrected | Required |
|---|---:|---:|---:|
| `/odometry/gps` max error vs direct EPSG:32643 | 10.392 m | 0.000003 m | < 0.01 m |
| Time for global EKF to reach < 2 m | never | 0.396 s | <= 1 s |
| Global-to-GPS median after corrected initialization | — | 1.413 m | <= 2 m |
| Global-to-GPS p95 after corrected initialization | — | 3.672 m | <= 4 m |
| Centreline median after corrected initialization | — | 1.845 m | <= 2.5 m |
| Centreline p95 after corrected initialization | — | 3.960 m | <= 5 m |

The corrected datum produced a `navsat_transform_node` heading factor of
`-2.44452e-11 rad`. Runtime inspection showed exactly two `/tf` publishers:
`ekf_filter_node` owned `odom -> base_link` and `ekf_filter_node_map` owned
`map -> odom`. Both DTU map servers were `active`; the planner and controller
remained `unconfigured` before and after playback.

The first GPS can initialize position, but the global EKF still follows the
fused GenZ and MAVROS-yaw motion between GPS fixes. The remaining 1–4 m error
is therefore localization/sensor disagreement, not the prior map's geographic
origin. The direct GPS route itself has centreline median
0.965 m and p95
4.214 m.

Live rover activation remains blocked on the measured GPS antenna lever arm,
ten consecutive RTK-fixed samples, covariance/readiness gates, and the required
20 m MAVROS yaw validation. No goal or physical motion command was sent.
