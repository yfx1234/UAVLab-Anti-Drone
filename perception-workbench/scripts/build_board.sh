#!/usr/bin/env bash
set -eo pipefail
task_project=${1:?Usage: bash scripts/build_board.sh /home/uavlab/Jetson_ROS2_V0}
task_workspace="$task_project/ros2_ws"
test -d "$task_workspace/src/zed_tracking"
python3 - <<'PY'
import json, pathlib
p = pathlib.Path('/tmp/zed_perception/supervisor.json')
if p.is_file():
    pid = json.loads(p.read_text()).get('pid')
    if isinstance(pid, int) and pathlib.Path('/proc', str(pid)).exists():
        raise SystemExit('Release camera before building to avoid loading the current Nano during capture')
PY
source /opt/ros/humble/setup.bash
set -u
cd "$task_workspace"
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon --log-base log_perception build --base-paths src/zed_perception_interfaces src/zed_tracking \
  --build-base build_perception --install-base install_perception --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DUAVLAB_BUILD_LEGACY_TRACKING=OFF
ctest --test-dir build_perception/zed_tracking --output-on-failure
echo 'Built and checked perception packages. Camera has not been started.'
