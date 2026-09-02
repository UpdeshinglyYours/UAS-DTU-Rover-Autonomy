# Recorded ROS topic evidence

These JSON snapshots were produced by `dtu_path_metrics.py` from `/plan`,
`/dtu_reference_route`, `/dtu_static_map`, `/dtu_road_mask`, and
`/global_costmap/costmap` during the headless Gazebo validation described in
`../CENTERLINE_NAVIGATION.md`.

The files intentionally contain only compact derived evidence, not a large
rosbag. Repeat the documented commands whenever parameters or source map
geometry change.

`barricade_course.json` additionally records the physical four-barricade
headless run described in `../BARRICADE_COURSE.md`. Its per-barricade entries
combine live global/local costmap marking, LaserScan, `/plan`, odometry, and
oriented BCR-footprint clearance observations. The `forward_plan_snapshot`
object is the direct output of the existing `dtu_path_metrics.py` tool.
