from glob import glob

from setuptools import setup

package_name = "tomato_harvest_robot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],  # Python コードは PYTHONPATH=/workspace/tomato-harvest/src 経由でロード
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="atsushi",
    maintainer_email="kodamaatsushi428@gmail.com",
    description="ROS2 robot node for tomato harvest simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tomato_harvest_robot_node = tomato_harvest_sim.robot.robot_node:main",
        ],
    },
)
