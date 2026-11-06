"""Setup for the g1_action ament_python package."""

from setuptools import find_packages, setup

package_name = "g1_action"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Valerio Belcamino",
    maintainer_email="v.belcamino@fusionailabs.it",
    description=(
        "Safety-critical realtime execution package for the G1 U6 hybrid VLA system: "
        "action chunk execution, risk estimation and Inspire hand bridging."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "action_executor_node = g1_action.action_executor_node:main",
            "risk_estimator_node = g1_action.risk_estimator_node:main",
            "inspire_hand_bridge_node = g1_action.inspire_hand_bridge_node:main",
        ],
    },
)
