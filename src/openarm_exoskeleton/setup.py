from glob import glob

from setuptools import find_packages, setup


package_name = "openarm_exoskeleton"


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
    description="Exoskeleton teleoperation package for OpenArm.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_zero_node = openarm_exoskeleton.servo_zero_node:main",
        ],
    },
)