"""Shared QoS profiles per CONVENTIONS.md.

* Sensor streams (images, tactile, joint/hand states, lidar, imu, odom):
  ``qos_profile_sensor_data`` (BEST_EFFORT, shallow depth).
* Commands & decisions (ActionChunk, ArmCmd, HandCmd, Twist, RiskScore,
  failures, subtasks): RELIABLE, depth 10.
* Latched events (/g1/estop, /g1/memory/dynamem/chunk_invalid,
  /g1/diagnostics/encoder_fault, /g1/hand/*/available): RELIABLE,
  TRANSIENT_LOCAL, depth 1.
"""

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

SENSOR_QOS: QoSProfile = qos_profile_sensor_data

COMMAND_QOS: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

LATCHED_QOS: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
