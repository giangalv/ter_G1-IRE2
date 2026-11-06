# Failure event raised by failure_detector_node (heuristic, realtime CPU) and
# optionally confirmed by vlm_failure_detector_node (offboard VLM).

uint8 STUCK=0             # |velocity| < eps, not at goal, t > 3 s
uint8 WRONG_GRASP=1       # DynaMem object pose delta > 5 cm after grasp attempt
uint8 COLLISION=2         # joint torque spike + velocity drop
uint8 STAGNATION=3        # same SubtaskToken.text for > 5 s
uint8 LOOP=4              # near-identical ActionChunk repeated > 3 times
uint8 TACTILE_SLIP=5      # tactile pressure drop > 40 % mid-grasp (U6 specific)
uint8 FORCE_OVERLOAD=6    # per-finger normal force above safety limit (U6 specific)
uint8 INCOMPLETE_GRASP=7  # grasp subtask active but < 3 fingers in contact after 2 s

std_msgs/Header header
uint8 type
string description
sensor_msgs/Image[] frames    # last 3 RGB frames at failure time
float32[] joint_torques       # 41-value effort snapshot at failure
