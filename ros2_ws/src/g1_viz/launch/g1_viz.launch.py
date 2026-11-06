"""RViz2 visualization of the G1 U6 from any PC on the robot LAN.

Starts: joint_state_relay (BODY-41 -> URDF names + odom TF),
robot_state_publisher (if a URDF is available) and RViz2 with the bundled
config. Everything is passive — this launch only *subscribes* to robot topics,
so it is safe to run and stop at any time.

    ros2 launch g1_viz g1_viz.launch.py                          # auto-find URDF
    ros2 launch g1_viz g1_viz.launch.py urdf:=/path/to/g1.urdf   # explicit
    ros2 launch g1_viz g1_viz.launch.py use_rviz:=false          # relay only

    # real robot, no VLA stack running: read /lowstate straight from the robot's DDS
    ros2 launch g1_viz g1_viz.launch.py lowstate_bridge:=true lowstate_topic:=/lf/lowstate \\
        fixed_base:=true hands:=true rviz_config:=<share>/config/g1_real.rviz

URDF lookup order: the ``urdf`` argument, then the ``g1_description`` package (``fetch_assets.sh``
downloads Unitree's model there), trying its common 29-DOF file names.
Without a URDF the launch still runs (sensors + TF, no robot mesh).

``fixed_base``: nothing broadcasts odom -> base when the robot is on its stand and no odometry
topic is running, so a static transform holds the pelvis ``base_height`` metres above the floor.
"""
from __future__ import annotations

import os
from typing import List, Optional

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_G1_DESCRIPTION_CANDIDATES = [
    "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf",
    "g1_29dof_with_hand_rev_1_0.urdf",
    "g1_29dof_rev_1_0.urdf",
    "g1_29dof.urdf",
    "g1.urdf",
]


def _find_urdf(explicit: str) -> Optional[str]:
    """Resolve the URDF path: explicit argument first, then g1_description."""
    if explicit:
        return explicit if os.path.isfile(os.path.expanduser(explicit)) else None
    try:
        share = get_package_share_directory("g1_description")
    except PackageNotFoundError:
        return None
    for sub in ("urdf", "."):
        for name in _G1_DESCRIPTION_CANDIDATES:
            path = os.path.join(share, sub, name)
            if os.path.isfile(path):
                return path
    return None


def _load_robot_description(path: str) -> str:
    """Read a URDF, running xacro first when the file requires it."""
    if path.endswith(".xacro"):
        import xacro  # ros-humble-xacro

        return xacro.process_file(path).toxml()
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def _setup(context) -> List:
    explicit = LaunchConfiguration("urdf").perform(context)
    urdf_path = _find_urdf(explicit)
    if urdf_path is None:
        return [
            LogInfo(
                msg=(
                    "[g1_viz] no URDF found (argument 'urdf' empty and package "
                    "'g1_description' not installed) - starting WITHOUT the robot "
                    "model. Run src/g1_description/fetch_assets.sh and rebuild."
                )
            )
        ]
    return [
        LogInfo(msg=f"[g1_viz] robot model: {urdf_path}"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": _load_robot_description(urdf_path)}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    rviz_config = os.path.join(
        get_package_share_directory("g1_viz"), "config", "g1_u6.rviz"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf",
                default_value="",
                description="Path to the G1 URDF/xacro (empty = auto-find g1_description)",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rs_color_compressed",
                default_value="false",
                description="true = RealSense colour from the Jetson's JPEG stream instead of raw (robot Wi-Fi)",
            ),
            DeclareLaunchArgument(
                "rs_depth_compressed",
                default_value="false",
                description="true = RealSense depth from the Jetson's lossless PNG stream instead of raw (robot Wi-Fi)",
            ),
            DeclareLaunchArgument(
                "hand_tcp_timeout_s",
                default_value="0.02",
                description="Modbus TCP transaction timeout of the hand bridge (0.02 cable, ~0.15 Wi-Fi)",
            ),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            DeclareLaunchArgument(
                "base_frame",
                default_value="pelvis",
                description="Root link of the URDF (Unitree's G1 URDFs start at 'pelvis')",
            ),
            DeclareLaunchArgument(
                "fixed_base",
                default_value="false",
                description="true = hold odom->base_frame still (robot on its stand, no /g1/odom)",
            ),
            DeclareLaunchArgument("base_height", default_value="0.79"),
            DeclareLaunchArgument(
                "lowstate_bridge",
                default_value="false",
                description="true = also run g1_unitree_bridge (reads the robot's native DDS state)",
            ),
            DeclareLaunchArgument(
                "lowstate_topic",
                default_value="/lowstate",
                description="/lowstate (~1 kHz, cable) or /lf/lowstate (~20 Hz, WiFi)",
            ),
            Node(
                package="g1_viz",
                executable="joint_state_relay",
                name="joint_state_relay",
                output="screen",
                parameters=[
                    {
                        "base_frame": LaunchConfiguration("base_frame"),
                        "publish_odom_tf": ParameterValue(
                            PythonExpression(
                                ["'", LaunchConfiguration("fixed_base"), "' != 'true'"]
                            ),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="fixed_base_tf",
                arguments=[
                    "--z", LaunchConfiguration("base_height"),
                    "--frame-id", "odom",
                    "--child-frame-id", LaunchConfiguration("base_frame"),
                ],
                condition=IfCondition(LaunchConfiguration("fixed_base")),
            ),
            DeclareLaunchArgument(
                "hands",
                default_value="false",
                description="true = read the Inspire hands over Modbus TCP (read-only) and draw the tactile cells",
            ),
            DeclareLaunchArgument(
                "head_camera",
                default_value="false",
                description="true = decode Unitree's multicast head video to /g1/camera/color/image",
            ),
            DeclareLaunchArgument(
                "realsense",
                default_value="false",
                description="true = relay the D435i depth from the RealSense domain (Jetson driver) and make a cloud",
            ),
            Node(
                package="g1_unitree_bridge",
                executable="realsense_relay",
                name="realsense_relay",
                output="screen",
                parameters=[
                    {
                        # true over the robot's Wi-Fi: colour as JPEG and depth as PNG made on the Jetson
                        # (tools/realsense_encoder_node.py in the base repo)
                        "color_compressed": ParameterValue(LaunchConfiguration("rs_color_compressed"), value_type=bool),
                        "depth_compressed": ParameterValue(LaunchConfiguration("rs_depth_compressed"), value_type=bool),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("realsense")),
            ),
            DeclareLaunchArgument(
                "hand_cameras",
                default_value="false",
                description="true = relay the palm cameras (JPEG) from the Jetson's DDS domain (tools/hand_cameras_ros2.sh)",
            ),
            Node(
                package="g1_unitree_bridge",
                executable="hand_camera_relay",
                name="hand_camera_relay",
                output="screen",
                condition=IfCondition(LaunchConfiguration("hand_cameras")),
            ),
            DeclareLaunchArgument(
                "hand_2d",
                default_value="false",
                description="true = open the 2D window: both hands laid flat with the touched cells and the palm cameras",
            ),
            Node(
                package="g1_viz",
                executable="hand_2d",
                name="hand_2d",
                output="screen",
                condition=IfCondition(LaunchConfiguration("hand_2d")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="d435_optical_tf",
                # URDF d435_link is a body frame (x forward, z up); the optical frame is x right, y down, z forward
                arguments=[
                    "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
                    "--frame-id", "d435_link", "--child-frame-id", "d435_depth_optical_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="d435_color_optical_tf",
                # same orientation as the depth optical frame; the ~15 mm offset of the colour sensor is ignored
                arguments=[
                    "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
                    "--frame-id", "d435_link", "--child-frame-id", "d435_color_optical_frame",
                ],
            ),
            DeclareLaunchArgument(
                "head_video_iface",
                default_value="",
                description="NIC facing the robot LAN, for the multicast video (empty = default route)",
            ),
            Node(
                package="g1_unitree_bridge",
                executable="head_video",
                name="head_video",
                output="screen",
                parameters=[{"iface": LaunchConfiguration("head_video_iface")}],
                condition=IfCondition(LaunchConfiguration("head_camera")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="livox_frame_tf",
                # The Livox driver names its frame livox_frame; Unitree's URDF calls the sensor
                # mid360_link. Orientation checked with the LiDAR IMU's gravity vector (~0.5 degrees);
                # the position is the URDF's, not measured.
                arguments=["--frame-id", "mid360_link", "--child-frame-id", "livox_frame"],
            ),
            Node(
                package="g1_action",
                executable="inspire_hand_bridge_node",
                name="inspire_hand_bridge_node",
                output="screen",
                parameters=[
                    {
                        "read_only": True,  # commands are dropped, only reads
                        # 20 ms is right for the cable (RTT ~1 ms); over the robot's Wi-Fi use ~0.15 (viz_real.sh wifi does)
                        "tcp_timeout_s": ParameterValue(
                            LaunchConfiguration("hand_tcp_timeout_s"), value_type=float
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("hands")),
            ),
            Node(
                package="g1_viz",
                executable="tactile_markers",
                name="tactile_markers",
                output="screen",
                condition=IfCondition(LaunchConfiguration("hands")),
            ),
            Node(
                package="g1_unitree_bridge",
                executable="lowstate_bridge",
                name="lowstate_bridge",
                output="screen",
                parameters=[
                    {
                        "lowstate_topic": LaunchConfiguration("lowstate_topic"),
                        # /lf/lowstate is already slow; /lowstate is decimated to 50 Hz for RViz
                        "max_rate_hz": 50.0,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("lowstate_bridge")),
            ),
            OpaqueFunction(function=_setup),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
