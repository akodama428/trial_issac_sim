from setuptools import setup

package_name = "tomato_harvest_simulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],  # Python コードは PYTHONPATH=/workspace/tomato-harvest/src 経由でロード
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="atsushi",
    maintainer_email="kodamaatsushi428@gmail.com",
    description="ROS2 simulator node for tomato harvest simulation (runs inside Isaac Sim)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tomato_harvest_simulator_node = tomato_harvest_sim.simulator.simulator_node:main",
        ],
    },
)
