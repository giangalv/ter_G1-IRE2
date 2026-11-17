"""Setup for g1_unitree_bridge: native Unitree DDS state -> workspace topics (read-only)."""
from setuptools import setup

package_name = "g1_unitree_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Gianluca Galvagni",
    maintainer_email="gianluca.galvagni99@gmail.com",
    description="Read-only bridge from Unitree LowState to /g1/joint_states and /g1/imu/data.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lowstate_bridge = g1_unitree_bridge.lowstate_bridge_node:main",
            "head_video = g1_unitree_bridge.head_video_node:main",
            "realsense_relay = g1_unitree_bridge.realsense_relay_node:main",
            "hand_camera_relay = g1_unitree_bridge.hand_camera_relay_node:main",
            "lidar_restamp = g1_unitree_bridge.lidar_restamp_node:main",
        ],
    },
)
