# Dual-path action chunk emitted by vla_inference_node (@ 8-10 Hz).
#
# positions / velocities are row-major flattened [chunk_length, 53]:
#   53 = 41 body joints (U6 full body, primary-encoder order)
#      + 12 hand joints (6 actuated DOF per Inspire RH56DFQ hand, left then right)
# hand_force is row-major flattened [chunk_length, 10]:
#   10 = per-fingertip force targets in Newtons (5 left fingers, then 5 right)
#
# Body slice [0:14] of each 53-vector is the arm_sdk mapping
# (left arm indices 0-6, right arm 7-13). See g1_bringup/config for joint order.

uint8 MODE_FAST=0      # flow matching, 3 ODE steps
uint8 MODE_PRECISE=1   # DDIM, 10 steps

std_msgs/Header header
uint32 chunk_length       # number of timesteps in this chunk (typ. 16)
float64[] positions       # chunk_length * 53
float64[] velocities      # chunk_length * 53
float64[] hand_force      # chunk_length * 10
float64 timestamp         # generation time (epoch seconds, offboard clock)
uint8 mode                # MODE_FAST | MODE_PRECISE
float32 action_entropy    # raw entropy from the VLA action head; consumed by
                          # risk_estimator_node and attc_node
