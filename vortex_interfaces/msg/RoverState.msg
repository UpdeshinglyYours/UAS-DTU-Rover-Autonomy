# Mission phase announced by casualty_manager on /rover_state (latched).
# The model script waits for STANDBY, does its work, then calls
# /casualty_manager/start_orbit with the same casualty_id.

uint8 TRAVEL=0    # driving to a casualty (or away from one)
uint8 STANDBY=1   # parked at casualty_id, facing it, waiting for start_orbit
uint8 ORBIT=2     # orbiting casualty_id
uint8 IDLE=3      # nothing left to do

std_msgs/Header header
uint8 state
uint32 casualty_id
geometry_msgs/PoseStamped casualty_pose
