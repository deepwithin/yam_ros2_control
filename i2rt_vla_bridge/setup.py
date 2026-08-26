import os
from glob import glob

from setuptools import find_packages, setup

package_name = "i2rt_vla_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Deep Patel",
    maintainer_email="deep@futurhandrobotics.com",
    description=(
        "Live OpenVLA -> IK -> YAM arm bridge: camera + instruction -> OpenVLA server -> "
        "delta gripper_tip pose -> /compute_ik -> joint_trajectory_controller/gripper_controller, "
        "with an RViz ghost of the next IK-solved step alongside the live arm."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vla_bridge_node = i2rt_vla_bridge.vla_bridge_node:main",
        ],
    },
)
