from setuptools import find_packages, setup
import os
from glob import glob

package_name = "biguasim_examples"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="biguauser",
    maintainer_email="marckquiroz8@gmail.com",
    description="Example teleoperation/visualization nodes for BiguaSim (ported from holoocean_examples).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_biguasim = biguasim_examples.joy_biguasim:main",
            "waypoint_follower = biguasim_examples.waypoint_follower:main",
            "camera_hud = biguasim_examples.camera_hud:main",
        ],
    },
)
