#!/usr/bin/env python3
"""Read-only ROS2 observation inspection; subscribes to no control/flight topics."""
import argparse
import json
import math
import time


def check_frame(frame):
    errors=[]
    if frame.schema_version!=2:errors.append('unsupported_schema')
    if not frame.run_id or not frame.boot_id:errors.append('missing_run_identity')
    if frame.header.frame_id!='zed_left_camera_optical_frame':errors.append('unexpected_coordinate_frame')
    latency=frame.processing_latency_s
    if not math.isfinite(latency) or latency<0:errors.append('invalid_processing_latency')
    if frame.observations_fresh and (not frame.detector_enabled or not 0<=latency<1):errors.append('invalid_freshness_flag')
    if (not frame.detector_enabled or not frame.observations_fresh) and frame.targets:errors.append('inactive_frame_contains_targets')
    for target in frame.targets:
        if target.mode not in ('depth','size_estimate','bearing_only'):errors.append('unknown_position_mode')
        if target.depth_measured_valid!=(target.mode=='depth'):errors.append('depth_mode_flag_mismatch')
        if target.position_valid!=(target.mode!='bearing_only'):errors.append('position_mode_flag_mismatch')
        if target.position_valid:
            values=[target.position_camera.x,target.position_camera.y,target.position_camera.z,target.distance_m]
            if not all(math.isfinite(v) for v in values) or target.position_camera.z<=0 or target.distance_m<=0:errors.append('invalid_position_values')
    return errors


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--seconds',type=float,default=20)
    args=parser.parse_args()
    if not 1<=args.seconds<=300:parser.error('seconds must be 1..300')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from zed_perception_interfaces.msg import PerceptionFrame
    rclpy.init();node=rclpy.create_node('uavlab_readonly_observation_inspector')
    result={'frames':0,'fresh_frames':0,'inactive_frames':0,'sequence_gaps':0,'errors':{},'modes':{},'maximum_processing_latency_s':0}
    previous=None
    def receive(frame):
        nonlocal previous
        result['frames']+=1;result['fresh_frames']+=int(frame.observations_fresh);result['inactive_frames']+=int(not frame.detector_enabled)
        for error in check_frame(frame):result['errors'][error]=result['errors'].get(error,0)+1
        identity=(frame.boot_id,frame.run_id)
        if previous and previous[0]==identity:
            if frame.sequence<=previous[1]:result['errors']['nonincreasing_sequence']=result['errors'].get('nonincreasing_sequence',0)+1
            else:result['sequence_gaps']+=frame.sequence-previous[1]-1
        previous=(identity,frame.sequence)
        if math.isfinite(frame.processing_latency_s):result['maximum_processing_latency_s']=max(result['maximum_processing_latency_s'],frame.processing_latency_s)
        for t in frame.targets:result['modes'][t.mode]=result['modes'].get(t.mode,0)+1
    node.create_subscription(PerceptionFrame,'/perception/observations',receive,qos_profile_sensor_data)
    started=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-started<args.seconds:rclpy.spin_once(node,timeout_sec=.1)
    finally:node.destroy_node();rclpy.shutdown()
    elapsed=time.monotonic()-started;result['seconds']=elapsed;result['received_hz']=result['frames']/elapsed
    print(json.dumps(result,allow_nan=False))
    if not result['frames'] or result['errors']:raise SystemExit(1)


if __name__=='__main__':main()
