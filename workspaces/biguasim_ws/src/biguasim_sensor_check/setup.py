from setuptools import find_packages, setup

package_name = "biguasim_sensor_check"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sensor_check.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="biguauser",
    maintainer_email="marckquiroz8@gmail.com",
    description="Verify BiguaSim sensor topics are flowing over ROS 2.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensor_check_node = biguasim_sensor_check.sensor_check_node:main",
        ],
    },
)
