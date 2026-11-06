"""Setup for g1_viz: RViz2 visualization support for the G1 U6 VLA system."""
from glob import glob

from setuptools import setup

package_name = "g1_viz"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valerio Belcamino",
    maintainer_email="v.belcamino@fusionailabs.it",
    description="RViz2 visualization for the G1 U6 VLA system.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_state_relay = g1_viz.joint_state_relay:main",
            "tactile_markers = g1_viz.tactile_markers_node:main",
            "hand_2d = g1_viz.hand_2d_node:main",
        ],
    },
)
