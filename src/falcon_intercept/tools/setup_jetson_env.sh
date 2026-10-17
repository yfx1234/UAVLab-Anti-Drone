#!/usr/bin/env bash
set -eo pipefail

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-numpy \
  python3-opencv \
  python3-yaml \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-image-tools \
  ros-humble-sensor-msgs \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-default-plugins

# Keep Jetson's apt OpenCV build so GStreamer/Argus integration is preserved.
# Do not install opencv-python from PyPI on the board.

echo "Jetson base ROS2 environment dependencies installed."
echo "YOLO runtime is intentionally not pinned here; choose PyTorch/Ultralytics/TensorRT after Jetson model path is confirmed."
