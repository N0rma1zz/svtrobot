from glob import glob

from setuptools import find_packages, setup


package_name = "openarm_vla"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="normal",
    maintainer_email="normal@example.com",
    description="OpenArm VLA ROS2 integration package for LeRobot recording and camera topic publishing.",
    license="CC-BY-NC-SA-4.0",
    tests_require=["pytest"],
    entry_points={
        "lerobot.robots": [
            "openarm_follower_ros2 = openarm_vla.robot_config_ros2:OpenArmRos2Config",
        ],
        "lerobot.teleoperators": [
            "openarm_leader_ros2 = openarm_vla.teleop_config_ros2:OpenArmRos2TeleopConfig",
        ],
    },
)