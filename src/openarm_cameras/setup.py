from glob import glob

from setuptools import find_packages, setup


package_name = "openarm_cameras"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="normal",
    maintainer_email="normal@example.com",
    description="USB camera startup package for OpenArm collection workflows.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "opencv_camera_pub = openarm_cameras.opencv_camera_pub:main",
        ],
    },
)