# Flat 100 m Obstacle Field

`flat_100m_obstacle_field.sdf` is a lightweight, primitive-only Gazebo
Harmonic world for LiDAR, mapping, exploration, and navigation tests. It is
independent of the AWS warehouse models.

## Layout

- The exactly flat native SDF plane is 100 x 100 m, covering approximately
  `x = -50..50` and `y = -50..50`.
- BCR Bot spawns at `(0, 0, 0.28)` with zero yaw.
- There are 31 static obstacle groups: 8 solid buildings and 23 smaller box,
  cylinder, barrier, tower, L-wall, U-enclosure, and dead-end groups.
- The closest obstacle surface is 7.25 m from the spawn point.
- Separate obstacle groups have at least 3 m horizontal footprint-to-footprint
  clearance. Components of the same compound wall group may touch.
- Every obstacle footprint has at least 2 m clearance from the world edge.
- A conservative grid check with obstacles inflated by 0.6 m confirms that
  the spawn remains connected to north, south, east, and west boundary
  corridors.

## Buildings

| Name | Center `(x, y)` m | Size `(x, y, z)` m |
| --- | ---: | ---: |
| `building_northwest_slate` | `(-38, 38)` | `(10, 10, 8)` |
| `building_north_umber` | `(-12, 38)` | `(12, 8, 6)` |
| `building_northeast_steel` | `(24, 38)` | `(14, 10, 10)` |
| `building_east_olive` | `(40, 17)` | `(10, 12, 7)` |
| `building_southeast_clay` | `(36, -36)` | `(12, 10, 9)` |
| `building_south_sandstone` | `(8, -40)` | `(10, 12, 5)` |
| `building_southwest_graphite` | `(-36, -35)` | `(14, 10, 11)` |
| `building_west_bluegray` | `(-40, 5)` | `(10, 12, 6)` |

## Build and launch

```bash
source /opt/ros/humble/setup.bash
source "$HOME/q_ws/install/setup.bash"
cd "$HOME/UAS-DTU-Rover-Autonomy"
colcon build --packages-select uas_dtu_gz_sim
source install/setup.bash
ros2 launch uas_dtu_gz_sim flat_100m_obstacle_field.launch.py
```

Run the source-tree geometry checks with:

```bash
python3 src/uas_dtu_gz_sim/scripts/validate_flat_obstacle_layout.py
```
