# Direct joint-position command for both G1 arms via arm_sdk (EDU only).
# Self-contained stand-in for the unitree arm_sdk command so this workspace
# builds without the vendor tree; a thin relay in unitree_ros2 maps it onto the
# native SDK message (see README "arm_sdk bridging").
#
# Joint order (matches unitree_sdk2 arm_sdk 14-joint layout):
#   0-6  : left  shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch,
#          wrist_roll, wrist_pitch, wrist_yaw
#   7-13 : right shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch,
#          wrist_roll, wrist_pitch, wrist_yaw
# Waist and leg joints are NOT part of arm_sdk - they belong to the Unitree
# locomotion controller.

std_msgs/Header header
float64[14] q        # joint position targets (rad)
float64[14] dq       # joint velocity feedforward (rad/s)
float64[14] tau_ff   # joint torque feedforward (N*m), zeros if unused
float64 weight       # arm_sdk authority blend [0..1]; 0 releases control smoothly
