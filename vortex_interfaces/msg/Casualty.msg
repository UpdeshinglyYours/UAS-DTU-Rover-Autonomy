# One casualty known to casualty_manager.

uint8 PENDING=0      # added, not visited yet
uint8 ACTIVE=1       # rover is travelling to it
uint8 STANDBY=2      # rover parked at it, waiting for start_orbit
uint8 ORBITING=3     # orbit in progress
uint8 DONE=4         # visited; covered_deg says how much of the circle was swept
uint8 SKIPPED=5      # not visited (approach failed / operator skip); see reason

uint32 id
geometry_msgs/PoseStamped pose   # casualty position, map frame
float64 lat                      # NaN if the casualty was given in map coordinates
float64 lon
uint8 status
float32 covered_deg              # degrees of the orbit circle actually swept (0..360)
string source                    # "latlon", "rviz", "gui", "cli", ...
string reason                    # why SKIPPED, or the orbit outcome for DONE
