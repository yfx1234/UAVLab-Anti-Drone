#!/usr/bin/env bash
# Flight ROS 2 discovery baseline for the local PX4 bridge.
#
# Keep this file limited to environment variables so it is safe to source from
# interactive shells. The Micro XRCE-DDS Agent and the ROS 2 CLI daemon must
# use the same discovery settings.

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
