from setuptools import find_packages, setup

package_name = "zed_perception_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="falcon team",
    maintainer_email="user@example.com",
    description=(
        "Bridge PerceptionFrame v2 (JSON channel) to DepthObservation V1 "
        "for the netgun interception chain."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = zed_perception_bridge.bridge_node:main",
        ],
    },
)
