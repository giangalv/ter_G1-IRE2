# Command for one Inspire RH56DFQ hand (consumed by inspire_hand_bridge_node).
# The RH56DFQ exposes 6 actuated DOF per hand:
#   [little_flex, ring_flex, middle_flex, index_flex, thumb_flex, thumb_rotation]

std_msgs/Header header
float32[] position           # 6 values, normalized [0..1] open->closed (Inspire convention)
float32[] force_limit        # 5 values: per-finger max force (N), safety-clamped by bridge
bool use_tactile_feedback    # enable 100 Hz force-position hybrid loop on TactileArray
