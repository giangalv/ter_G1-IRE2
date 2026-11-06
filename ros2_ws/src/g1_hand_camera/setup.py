"""Setup for the g1_hand_camera ament_python package."""

from setuptools import find_packages, setup

package_name = "g1_hand_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Valerio Belcamino",
    maintainer_email="v.belcamino@fusionailabs.it",
    description=(
        "Palm cameras of the Unitree G1 U6 Inspire hands as ROS 2 image "
        "streams (raw + JPEG + camera_info + availability)."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hand_camera_node = "
            "g1_hand_camera.hand_camera_node:main",
        ],
    },
)
