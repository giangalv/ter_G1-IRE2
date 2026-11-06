"""Relay node: BODY-41 / hand joint vectors -> URDF-named /joint_states + TF.

Feeds ``robot_state_publisher`` (and therefore RViz's RobotModel display) from
the VLA-stack topics:

* subscribes ``/g1/joint_states`` (flat BODY-41 vector, primary encoders) and
  optionally ``/g1/hand/{left,right}/joint_states`` (6 DOF each);
* republishes a merged, URDF-named ``sensor_msgs/JointState`` on
  ``/joint_states`` at ``publish_rate_hz``;
* optionally broadcasts the ``odom -> base_link`` TF from ``/g1/odom`` so the
  robot model moves in the world (nothing else in the stack broadcasts it).

Runs fine on a remote PC: it only consumes DDS topics.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from g1_viz.relay_core import (
    DEFAULT_BODY_JOINT_NAMES,
    DEFAULT_HAND_JOINT_NAMES_LEFT,
    DEFAULT_HAND_JOINT_CLOSED_RAD,
    DEFAULT_HAND_JOINT_NAMES_RIGHT,
    hand_to_angles,
    map_joint_vector,
    merge_joint_states,
)


class JointStateRelay(Node):
    """Republish VLA-stack joint vectors as URDF-named /joint_states."""

    def __init__(self) -> None:
        super().__init__("joint_state_relay")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("body_joint_names", DEFAULT_BODY_JOINT_NAMES)
        self.declare_parameter("hand_joint_names_left", DEFAULT_HAND_JOINT_NAMES_LEFT)
        self.declare_parameter("hand_joint_names_right", DEFAULT_HAND_JOINT_NAMES_RIGHT)
        self.declare_parameter("hand_joint_closed_rad", DEFAULT_HAND_JOINT_CLOSED_RAD)

        self._body_names: List[str] = list(
            self.get_parameter("body_joint_names").value or []
        )
        self._hand_names_left: List[str] = list(
            self.get_parameter("hand_joint_names_left").value or []
        )
        self._hand_names_right: List[str] = list(
            self.get_parameter("hand_joint_names_right").value or []
        )
        self._hand_closed_rad: List[float] = [
            float(v) for v in self.get_parameter("hand_joint_closed_rad").value
        ]
        self._odom_frame: str = str(self.get_parameter("odom_frame").value)
        self._base_frame: str = str(self.get_parameter("base_frame").value)

        self._latest_body: Optional[JointState] = None
        self._latest_hand_left: Optional[JointState] = None
        self._latest_hand_right: Optional[JointState] = None

        self.create_subscription(
            JointState, "/g1/joint_states", self._on_body, qos_profile_sensor_data
        )
        if self._hand_names_left:
            self.create_subscription(
                JointState,
                "/g1/hand/left/joint_states",
                self._on_hand_left,
                qos_profile_sensor_data,
            )
        if self._hand_names_right:
            self.create_subscription(
                JointState,
                "/g1/hand/right/joint_states",
                self._on_hand_right,
                qos_profile_sensor_data,
            )

        self._pub = self.create_publisher(JointState, "/joint_states", 10)

        self._tf_broadcaster: Optional[TransformBroadcaster] = None
        if bool(self.get_parameter("publish_odom_tf").value):
            self._tf_broadcaster = TransformBroadcaster(self)
            self.create_subscription(
                Odometry, "/g1/odom", self._on_odom, qos_profile_sensor_data
            )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            f"joint_state_relay up: {len(self._body_names)} body joints, "
            f"hands L:{len(self._hand_names_left)} R:{len(self._hand_names_right)}, "
            f"odom TF {'on' if self._tf_broadcaster else 'off'}"
        )

    def _on_body(self, msg: JointState) -> None:
        self._latest_body = msg

    def _on_hand_left(self, msg: JointState) -> None:
        self._latest_hand_left = msg

    def _on_hand_right(self, msg: JointState) -> None:
        self._latest_hand_right = msg

    def _on_odom(self, msg: Odometry) -> None:
        if self._tf_broadcaster is None:
            return
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    def _publish(self) -> None:
        if self._latest_body is None:
            return
        groups = [
            map_joint_vector(
                self._body_names,
                self._latest_body.position,
                self._latest_body.velocity,
            )
        ]
        for names, latest in (
            (self._hand_names_left, self._latest_hand_left),
            (self._hand_names_right, self._latest_hand_right),
        ):
            if not names:
                continue
            # No hand message (hand bridge not running): publish "open" so that
            # robot_state_publisher still emits a transform for every finger link.
            raw = list(latest.position) if latest is not None else []
            angles = hand_to_angles(raw, self._hand_closed_rad)
            groups.append((list(names), angles, [0.0] * len(names)))
        names, positions, velocities = merge_joint_states(*groups)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = positions
        out.velocity = velocities
        self._pub.publish(out)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
