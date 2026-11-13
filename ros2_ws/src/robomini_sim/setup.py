import os
from glob import glob

from setuptools import setup

package_name = "robomini_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2-sim",
    maintainer_email="ros2-sim@robomini.local",
    description="ROS 2 bridge config, rune referee state machine, launcher wrapper and launch files "
                "for the Robomini auto-aim simulation platform.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rune_referee = robomini_sim.rune_referee:main",
            "launcher_node = robomini_sim.launcher_node:main",
        ],
    },
)
