"""Opt-in regression: SDK verification must preserve existing export mappings."""
import pathlib,sys,tempfile
def run():
    sys.path.insert(0,'/home/uavlab/Jetson_ROS2_V0/ros2_ws/src/zed_tracking/tools')
    from perception_supervisor import RECORDS,WS,run_sdk_tool
    source=sorted(RECORDS.glob('session_7abd*/segments/*.svo2'))[0]
    with tempfile.TemporaryDirectory(prefix='perception_mapping_test_') as folder:
        mapping=pathlib.Path(folder)/'mapping.jsonl';mapping.write_text('existing mapping must survive\n')
        expected=mapping.read_bytes()
        result=run_sdk_tool([WS/'install_perception/zed_tracking/lib/zed_tracking/export_svo_rgb',source,folder,2,'verify'],timeout=120)
        assert result.returncode==0,result.stderr
        assert mapping.read_bytes()==expected,'verification destroyed existing mapping'
    print('PASS: verify preserves exported frame mappings')
    
if __name__=='__main__':run()
