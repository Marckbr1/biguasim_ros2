from setuptools import find_packages, setup

package_name = "biguasim_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/biguasim_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="biguauser",
    maintainer_email="marckquiroz8@gmail.com",
    description="Normalize BiguaSim custom/polar sensor messages into standard ROS 2 types.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_bridge_node = biguasim_bridge.odom_bridge_node:main",
            "sonar_bridge_node = biguasim_bridge.sonar_bridge_node:main",
        ],
    },
)
