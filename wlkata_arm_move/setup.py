import os
from glob import glob
from setuptools import find_packages, setup

package_name = "wlkata_arm_move"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=[
        "setuptools",
        "pyserial",
    ],
    zip_safe=True,
    maintainer="wlkata",
    maintainer_email="wlkata@todo.todo",
    description=(
        "ROS 2 Humble driver: MoveIt2 FollowJointTrajectory -> G-code -> Mirobot serial"
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "mirobot_moveit_move = wlkata_arm_move.mirobot_moveit_move:main",
        ],
    },
)
