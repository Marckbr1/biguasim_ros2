from setuptools import find_packages, setup
import os
from glob import glob

package_name = "biguasim_main"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="biguauser",
    maintainer_email="marckquiroz8@gmail.com",
    description="Core simulation bridge for BiguaSim (Node/Interface/Encoders, ported from holoocean_main).",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "biguasim_node = biguasim_main.biguasim_node:main",
        ],
    },
)
