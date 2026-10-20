# Final result of one OrbitCasualty run, published latched on /orbit_casualty/report.
# (Humble's TimedBehavior can only fill total_elapsed_time in the action result,
# so the real outcome travels here.)

uint8 FULL=0           # swept the requested angle (normally 360 deg)
uint8 PARTIAL=1        # swept every reachable part of the circle, but not all of it
uint8 BLOCKED=2        # no part of the circle reachable from where the rover was; no motion
uint8 KEEPOUT_GUARD=3  # stopped because the body was predicted to get too close
uint8 TIMEOUT=4        # took too long
uint8 CANCELED=5       # action was cancelled
uint8 ERROR=6          # no costmap / no pose / bad goal

std_msgs/Header header
uint32 casualty_id
uint8 outcome
float32 covered_deg
float32[] sector_start_deg   # covered sectors, angles of the rover around the casualty (map frame, deg)
float32[] sector_end_deg
float32 min_body_distance    # closest any part of the (real) footprint came to the casualty, m
string message
