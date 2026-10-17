#!/usr/bin/env bash
# Flight ROS 2 discovery baseline for the local PX4 bridge.
#
# Keep this file limited to environment variables so it is safe to source from
# interactive shells. The Micro XRCE-DDS Agent and the ROS 2 CLI daemon must
# use the same discovery settings.

export ROS_DOMAIN_ID=0
# N-1（2026-10-17）：与拦截链/感知统一为 0。原 1 仅影响登录 shell 的
# CLI 视图，会让 ros2 topic hz /fmu/out 误显有数据；真实连通以拦截链
# 日志 attmiss/fcu 为准。若日后飞控 PTCFG=1（仅本机），此处再改回 1。
export ROS_LOCALHOST_ONLY=0
