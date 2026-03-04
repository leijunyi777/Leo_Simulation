from setuptools import setup
import os
from glob import glob

package_name = "robot_control_system"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name, ["robot_control_system/best.pt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@todo.todo",
    description="Robot control system including FSM, Camera, and Manipulator nodes",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_fsm = robot_control_system.robot_fsm:main",
            "camera_node = robot_control_system.camera_node:main",
            "manipulator_node = robot_control_system.manipulator_node:main",
            "nav_node = robot_control_system.nav_node:main",
            "tf_sim_node = robot_control_system.tf_sim_node:main",
        ],
    },
)
