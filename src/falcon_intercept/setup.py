from glob import glob
import os

from setuptools import find_packages, setup


package_name = "falcon_intercept"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "tools"),
            glob("tools/field_terminal_*.sh"),
        ),
        (
            os.path.join("share", package_name, "tools"),
            glob("tools/field_terminal_*.py"),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config", "camera"),
            glob("config/camera/*.yaml") + glob("config/camera/*.csv"),
        ),
        (os.path.join("share", package_name, "models"), glob("models/*.pt")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="falcon team",
    maintainer_email="user@example.com",
    description="Jetson ROS2 V0 single-UAV visual intercept pipeline.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "intercept_fsm_node = falcon_intercept.intercept_fsm_node:main",
            "px4_adapter_node = falcon_intercept.px4_adapter_node:main",
            "px4_upstream_adapter_node = falcon_intercept.px4_upstream_adapter_node:main",
            "replay_px4_state_node = falcon_intercept.replay_px4_state_node:main",
            "synthetic_engagement_node = falcon_intercept.synthetic_engagement_node:main",
            "field_monitor_node = falcon_intercept.field_monitor_node:main",
            "target_kf_node = falcon_intercept.target_kf_node:main",
            "stereo_target_solver_node = falcon_intercept.stereo_target_solver:main",
            "search_bearing_node = falcon_intercept.search_bearing_node:main",
            "perception_bridge_node = falcon_intercept.perception_bridge_node:main",
            "upstream_gate_node = falcon_intercept.upstream_gate_node:main",
            "search_servo_node = falcon_intercept.search_servo_node:main",
            "stereo_roi_node = falcon_intercept.stereo_roi_node:main",
            "v4l2_camera_node = falcon_intercept.v4l2_camera_node:main",
            "jetson_camera_node = falcon_intercept.jetson_camera_node:main",
            "detector_node = falcon_intercept.detector_node:main",
            "real_aircraft_virtual_target = falcon_intercept.virtual_target_node:main",
            "virtual_shot_referee = falcon_intercept.virtual_referee:main",
        ],
    },
)
