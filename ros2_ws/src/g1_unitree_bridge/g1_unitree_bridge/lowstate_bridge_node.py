"""Read-only bridge: Unitree ``LowState`` -> ``/g1/joint_states`` (BODY-41) + ``/g1/imu/data``.

The robot publishes its state on the DDS topics ``/lowstate`` (~1 kHz) and ``/lf/lowstate``
(~20 Hz, meant for slow links such as WiFi). This node only SUBSCRIBES to one of them; it
never publishes towards the robot, so it is safe to run next to the locomotion controller.

Run it anywhere the robot's DDS (domain 0) is visible: onboard, or on the viewing PC (then
``/lf/lowstate`` keeps the radio load at a few hundred kbit/s).
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from unitree_hg.msg import LowState

from g1_unitree_bridge.bridge_core import BODY41_NAMES, quat_wxyz_to_xyzw, to_body41


class LowStateBridge(Node):
    """Republish Unitree LowState as workspace joint-state and IMU topics."""

    def __init__(self) -> None:
        super().__init__("lowstate_bridge")
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("max_rate_hz", 500.0)  # 0 = republish every message
        self.declare_parameter("imu_frame", "imu_link")

        topic = str(self.get_parameter("lowstate_topic").value)
        max_rate = float(self.get_parameter("max_rate_hz").value)
        self._min_period_ns = int(1e9 / max_rate) if max_rate > 0 else 0
        self._imu_frame = str(self.get_parameter("imu_frame").value)
        self._last_ns = 0

        self._joint_pub = self.create_publisher(JointState, "/g1/joint_states", qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(Imu, "/g1/imu/data", qos_profile_sensor_data)
        self.create_subscription(LowState, topic, self._on_lowstate, qos_profile_sensor_data)
        self._warned_mode: Optional[int] = None
        self.get_logger().info(
            f"lowstate_bridge up: {topic} -> /g1/joint_states, /g1/imu/data "
            f"(max {max_rate:g} Hz, read-only)"
        )

    def _on_lowstate(self, msg: LowState) -> None:
        now = self.get_clock().now()
        if self._min_period_ns and now.nanoseconds - self._last_ns < self._min_period_ns:
            return
        self._last_ns = now.nanoseconds
        stamp = now.to_msg()

        motors = msg.motor_state
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = BODY41_NAMES
        joints.position = to_body41([m.q for m in motors])
        joints.velocity = to_body41([m.dq for m in motors])
        joints.effort = to_body41([m.tau_est for m in motors])
        self._joint_pub.publish(joints)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._imu_frame
        x, y, z, w = quat_wxyz_to_xyzw(msg.imu_state.quaternion)
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = x, y, z, w
        # array fields come out of rclpy as numpy.float32, which message fields reject
        gyro: List[float] = [float(v) for v in msg.imu_state.gyroscope]
        acc: List[float] = [float(v) for v in msg.imu_state.accelerometer]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acc
        self._imu_pub.publish(imu)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LowStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
