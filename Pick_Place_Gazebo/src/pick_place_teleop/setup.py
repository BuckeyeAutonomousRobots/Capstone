from glob import glob
import os

from setuptools import setup

package_name = "pick_place_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Keyboard teleoperation and bridge nodes for Gazebo pick-and-place.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "keyboard_target_twist_teleop = pick_place_teleop.keyboard_target_twist_teleop:main",
            "target_twist_to_servo_cmd = pick_place_teleop.target_twist_to_servo_cmd:main",
            "target_twist_to_gripper_cmd = pick_place_teleop.target_twist_to_gripper_cmd:main",
            "target_twist_reset_manager = pick_place_teleop.target_twist_reset_manager:main",
        ],
    },
)
