"""QoS profiles per CONVENTIONS.md (local copy: this package depends on no g1_* package).

* Image streams: ``qos_profile_sensor_data`` (BEST_EFFORT, shallow depth).
* Latched availability flag: RELIABLE, TRANSIENT_LOCAL, depth 1 (same profile
  as ``/g1/hand/*/available``).
"""

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

SENSOR_QOS: QoSProfile = qos_profile_sensor_data

LATCHED_QOS: QoSProfile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)
