"""run_interceptor_stack.sh（2026-10-23）：start 建复盘记录（bag/参数/安装参数/运行信息）并随链起
常驻发射通道；stop 收尾（录制器 SIGINT 写 metadata、拷入运行日志）。

测试副本不 source ROS；ros2 为 Python 桩（像真 launch/录制器那样自行处理 SIGINT——
nohup … & 启动的后台命令在非交互 bash 里 SIGINT 被忽略，bash 桩收不到）；pgrep 桩
保证兜底清理绝不碰本机其他进程（板上带生产链跑也安全）。"""
import contextlib
import importlib.util
import io
import json
import os
import pathlib
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'deploy' / 'run_interceptor_stack.sh'
spec = importlib.util.spec_from_file_location('netgun_fire_button_stack_test', ROOT / 'deploy' / 'netgun_fire_button.py')
fb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fb)

ROS2_STUB = r'''#!/usr/bin/env python3
import os, signal, sys, time
args = sys.argv[1:]
out = None
if args[:1] == ['launch']:
    with open(os.environ['ARGS_FILE'], 'w') as f:
        f.write('\n'.join(args) + '\n')
elif args[:2] == ['bag', 'record']:
    with open(os.environ['ARGS_FILE'] + '.bag', 'w') as f:
        f.write('\n'.join(args) + '\n')
    out = args[args.index('-o') + 1]
    os.makedirs(out)
    print("[INFO] [1.0] [rosbag2_recorder]: Subscribed to topic '/debug/strike_state'", flush=True)
elif args[:2] == ['param', 'dump']:
    print('%s:\n  ros__parameters:\n    use_sim_time: false' % args[2].lstrip('/'))
    sys.exit(0)
else:
    sys.exit(3)

def stop(*_):
    if out:
        with open(os.path.join(out, 'metadata.yaml'), 'w') as f:
            f.write('rosbag2_bagfile_information: {}\n')
    sys.exit(0)

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while True:
    time.sleep(0.1)
'''


def stack_script_copy(root):
    script = SCRIPT.read_text(encoding='utf-8')
    assert script.count('env_setup() {\n') == 1
    assert script.count('sleep 15; run_start_check "$log_file" "$pid"') == 1
    script = script.replace('env_setup() {\n', 'env_setup() {\n  return 0\n', 1)
    script = script.replace('sleep 15; run_start_check "$log_file" "$pid"', 'true', 1)
    path = root / 'run_interceptor_stack.sh'
    path.write_text(script, encoding='utf-8')
    return path


def stub_bin(root):
    bin_dir = root / 'bin'
    bin_dir.mkdir()
    (bin_dir / 'ros2').write_text(ROS2_STUB, encoding='utf-8')
    (bin_dir / 'pgrep').write_text('#!/bin/sh\nexit 1\n', encoding='utf-8')
    for name in ('ros2', 'pgrep'):
        (bin_dir / name).chmod(0o755)
    return bin_dir


def kill_groups(logs):
    for name in ('interceptor_stack.pid', 'interceptor_bag.pid', 'netgun_fire_server.pid'):
        try:
            os.killpg(int((logs / name).read_text().strip()), signal.SIGKILL)
        except (OSError, ValueError):
            pass


@unittest.skipIf(os.name == 'nt' or not shutil.which('bash') or not shutil.which('setsid'), 'POSIX bash required')
class StackReviewTests(unittest.TestCase):
    def setUp(self):
        self.root = pathlib.Path(tempfile.mkdtemp(prefix='interceptor_ws_'))
        self.workspace = self.root / 'ws'
        (self.workspace / 'install').mkdir(parents=True)
        (self.workspace / 'deploy').mkdir()
        for name in ('mounting_geometry.py', 'netgun_fire_button.py'):
            shutil.copy(ROOT / 'deploy' / name, self.workspace / 'deploy' / name)
        self.config = self.root / 'config'
        self.logs = self.root / 'logs'
        self.args_file = self.root / 'ros2_args.txt'
        bin_dir = stub_bin(self.root)
        self.script = stack_script_copy(self.root)
        self.env = dict(os.environ, INTERCEPTOR_WS=str(self.workspace), NETGUN_LOG_DIR=str(self.logs),
                        NETGUN_CONFIG_DIR=str(self.config), ARGS_FILE=str(self.args_file), ROS_DOMAIN_ID='199',
                        PATH=str(bin_dir) + os.pathsep + os.environ.get('PATH', ''))
        self.env.pop('NETGUN_FIRE_SOCKET', None)
        self.env.pop('NETGUN_REVIEW_MIN_FREE_MB', None)
        self.socket = self.logs / 'netgun_fire_d199.sock'

    def tearDown(self):
        kill_groups(self.logs)
        shutil.rmtree(self.root, ignore_errors=True)

    def run_script(self, *args, env=None):
        return subprocess.run(['bash', str(self.script), *args], env=env or self.env, capture_output=True, text=True, timeout=60)

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return predicate()

    def review_dir(self):
        dirs = sorted(self.logs.glob('review_*'))
        self.assertEqual(len(dirs), 1, dirs)
        return dirs[0]

    def test_start_records_review_and_starts_fire_channel(self):
        self.config.mkdir()
        (self.config / 'mounting_geometry.json').write_text('{}', encoding='utf-8')
        geometry = subprocess.run(['python3', str(self.workspace / 'deploy' / 'mounting_geometry.py'), 'launch-args'],
                                  env=self.env, capture_output=True, text=True)
        if geometry.returncode:
            (self.config / 'mounting_geometry.json').unlink()
        started = time.monotonic()
        result = self.run_script('start', 'fire_mode:=auto_manual')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertLess(time.monotonic() - started, 10, 'start must not wait for the 15 s background check')
        review = self.review_dir()
        self.assertIn('复盘录制 %s' % review, result.stdout)
        self.assertIn('常驻发射通道随链启动', result.stdout)
        info = (review / 'run_info.txt').read_text(encoding='utf-8')
        log_name = next(self.logs.glob('interceptor_stack_*.log')).name
        self.assertIn('stack_log=%s\n' % log_name, info)
        self.assertRegex(info, r'started=\d{4}-\d\d-\d\d ')
        self.assertIn('commit=', info)
        self.assertRegex(info, r'launch_args=.*fire_mode:=auto_manual')
        self.assertEqual(review.name, 'review_' + log_name[len('interceptor_stack_'):-len('.log')])
        if (self.config / 'mounting_geometry.json').exists():
            self.assertTrue((review / 'mounting_geometry.json').exists())
        self.assertTrue(self.wait_for(lambda: pathlib.Path(str(self.args_file) + '.bag').exists()))
        bag_args = pathlib.Path(str(self.args_file) + '.bag').read_text().splitlines()
        self.assertEqual(bag_args[:4], ['bag', 'record', '-o', str(review / 'bag')])
        self.assertEqual(bag_args[4], '--regex')
        self.assertEqual(bag_args[6:], ['--storage-preset-profile', 'resilient'])
        topics = re.compile(bag_args[5])
        for topic in ('/fmu/out/vehicle_status', '/fmu/in/offboard_control_mode', '/debug/strike_state',
                      '/tracking/virtual_shot_event', '/netgun/fire_command', '/netgun/blocked/vehicle_command',
                      '/perception/observations_json', '/perception/depth/observation', '/rosout',
                      '/simulation/pursuer_takeoff_ready'):
            self.assertTrue(topics.match(topic), topic)
        for topic in ('/zed/zed_node/rgb/image_rect_color', '/perception/depth/image', '/tf', '/fmu/outx'):
            self.assertFalse(topics.match(topic), topic)
        log = (self.logs / log_name).read_text(encoding='utf-8')
        self.assertIn('NETGUN_REVIEW %s\n' % review, log)
        self.assertEqual((self.logs / 'interceptor_review.current').read_text().strip(), str(review))
        # 常驻发射通道：本域套接字可 ping，未见火控节点时停在等待阶段
        self.assertTrue(self.wait_for(self.socket.exists))
        reply = fb.channel_request('ping', str(self.socket))
        self.assertEqual(reply['link']['phase'], '等待火控节点')
        self.assertTrue(self.wait_for(lambda: 'netgun_fire_server' in (self.logs / log_name).read_text(encoding='utf-8')))
        # 节点参数导出（测试副本不等 15s）
        dumps = [review / 'params' / (node + '.yaml') for node in
                 ('zed_perception_bridge', 'relative_estimator', 'netgun_guidance_node', 'netgun_strike_node')]
        self.assertTrue(self.wait_for(lambda: all(p.exists() and 'ros__parameters' in p.read_text() for p in dumps)))
        status = self.run_script('status')
        self.assertIn('常驻发射通道：运行中', status.stdout)
        self.assertIn('复盘录制：录制中', status.stdout)
        listing = self.run_script('review')
        self.assertIn(review.name, listing.stdout)
        self.assertIn('bag 录制中', listing.stdout)
        self.assertIn('scp -r uavlab@192.168.50.100:~/netgun_logs/', listing.stdout)

    def test_stop_finalizes_review_and_stops_channel(self):
        self.assertEqual(self.run_script('start').returncode, 0)
        review = self.review_dir()
        self.assertTrue(self.wait_for(self.socket.exists))
        self.assertTrue(self.wait_for(lambda: (self.logs / 'interceptor_bag.pid').exists()))
        bag_pid = int((self.logs / 'interceptor_bag.pid').read_text())
        result = self.run_script('stop')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertRegex(result.stdout, r'已停止；复盘记录 %s（.+，bag 已收尾）' % review.name)
        self.assertTrue((review / 'bag' / 'metadata.yaml').exists(), 'recorder must get SIGINT and close the bag')
        info = (review / 'run_info.txt').read_text(encoding='utf-8')
        self.assertRegex(info, r'stopped=\d{4}-')
        self.assertIn('bag=已收尾', info)
        stack_log = next(self.logs.glob('interceptor_stack_*.log'))
        self.assertEqual((review / 'stack.log').read_text(encoding='utf-8'), stack_log.read_text(encoding='utf-8'))
        for name in ('interceptor_review.current', 'interceptor_bag.pid', 'netgun_fire_server.pid', 'interceptor_stack.pid'):
            self.assertFalse((self.logs / name).exists(), name)
        self.assertTrue(self.wait_for(lambda: not self.socket.exists()))
        with self.assertRaises(ProcessLookupError):
            os.kill(bag_pid, 0)
        listing = self.run_script('review')
        self.assertIn('bag 已收尾', listing.stdout)
        again = self.run_script('stop')
        self.assertEqual(again.stdout.strip(), '拦截链未运行')

    def test_low_disk_keeps_logs_and_params_without_bag(self):
        env = dict(self.env, NETGUN_REVIEW_MIN_FREE_MB='999999999')
        result = self.run_script('start', env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        review = self.review_dir()
        self.assertFalse(pathlib.Path(str(self.args_file) + '.bag').exists())
        self.assertIn('本次不录复盘 bag', (review / 'run_info.txt').read_text(encoding='utf-8'))
        self.assertIn('本次不录复盘 bag', next(self.logs.glob('interceptor_stack_*.log')).read_text(encoding='utf-8'))
        stop = self.run_script('stop', env=env)
        self.assertIn('bag 未录制', stop.stdout)
        self.assertTrue((review / 'stack.log').exists())

    def test_reused_pid_files_are_not_trusted(self):
        self.logs.mkdir()
        bystander = subprocess.Popen(['sleep', '30'], start_new_session=True)
        try:
            for name in ('interceptor_stack.pid', 'interceptor_bag.pid', 'netgun_fire_server.pid'):
                (self.logs / name).write_text('%d\n' % bystander.pid)
            self.assertIn('未运行', self.run_script('status').stdout.splitlines()[0])
            result = self.run_script('stop')
            self.assertEqual(result.stdout.strip(), '拦截链未运行')
            self.assertIsNone(bystander.poll(), 'an unrelated process reusing a stale pid must survive stop')
        finally:
            bystander.kill()
            bystander.wait()



class StatusScriptTests(unittest.TestCase):
    """deploy/interceptor_status.py 的 review / fire_link 字段（面板 2s 轮询）。"""

    def setUp(self):
        spec = importlib.util.spec_from_file_location('interceptor_status_under_test', ROOT / 'deploy' / 'interceptor_status.py')
        self.status = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.status)
        self.root = pathlib.Path(tempfile.mkdtemp(prefix='netgun_logs_'))
        self.addCleanup(shutil.rmtree, self.root, True)
        for name, value in (('LOG_ROOT', str(self.root)), ('PID_FILE', str(self.root / 'interceptor_stack.pid')),
                            ('REVIEW_FILE', str(self.root / 'interceptor_review.current')),
                            ('BAG_PID_FILE', str(self.root / 'interceptor_bag.pid'))):
            setattr(self.status, name, value)

    @unittest.skipUnless(os.path.isdir('/proc'), 'Linux /proc required (board script)')
    def test_review_recording_size_topics_and_notes(self):
        self.assertNotIn('dir', self.status.review_status())
        old = self.root / 'review_20261022_100000'
        old.mkdir()
        self.assertEqual(self.status.review_status()['last'], old.name)
        review = self.root / 'review_20261023_170000'
        (review / 'bag').mkdir(parents=True)
        (review / 'bag' / 'bag_0.db3').write_bytes(b'x' * 1048576)
        (review / 'bag_record.log').write_text(
            "[INFO] [1.0] [rosbag2_recorder]: Subscribed to topic '/debug/strike_state'\n"
            "[INFO] [1.1] [rosbag2_recorder]: Subscribed to topic '/fmu/out/vehicle_status'\n"
            "[INFO] [1.2] [rosbag2_recorder]: Subscribed to topic '/debug/strike_state'\n", encoding='utf-8')
        (review / 'run_info.txt').write_text('started=x\n', encoding='utf-8')
        (self.root / 'interceptor_review.current').write_text(str(review) + '\n')
        recorder = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)', 'bag', 'record'])
        self.addCleanup(lambda: recorder.poll() is None and recorder.kill())
        (self.root / 'interceptor_bag.pid').write_text('%d\n' % recorder.pid)
        info = self.status.review_status()
        self.assertEqual(info['dir'], review.name)
        self.assertTrue(info['recording'])
        self.assertEqual(info['topics'], 2)
        self.assertAlmostEqual(info['size_mb'], 1.0, delta=0.05)
        self.assertIsInstance(info['free_mb'], int)
        self.assertNotIn('note', info)
        recorder.kill()
        recorder.wait()
        with open(review / 'bag_record.log', 'a', encoding='utf-8') as f:
            f.write('[ERROR] [2.0] [rosbag2_storage]: disk full\n')
        info = self.status.review_status()
        self.assertFalse(info['recording'])
        self.assertIn('disk full', info['note'])
        (review / 'run_info.txt').write_text('WARN：磁盘剩余 100 MB（< 5120 MB），本次不录复盘 bag\n', encoding='utf-8')
        self.assertIn('不录复盘', self.status.review_status()['note'])

    def test_fire_link_reports_missing_channel(self):
        missing = str(self.root / 'none.sock')
        with mock.patch.dict(os.environ, {'NETGUN_FIRE_SOCKET': missing}):
            info = self.status.fire_link_status()
        self.assertFalse(info['ok'])
        self.assertIn('未运行', info['error'])

    @unittest.skipUnless(hasattr(socket, 'AF_UNIX'), 'Unix sockets required')
    def test_fire_link_ping(self):
        path = str(self.root / 'fire.sock')
        channel = fb.FireChannel(log=lambda *a, **k: None)
        sock = channel.bind(path)
        import threading
        thread = threading.Thread(target=channel.serve_forever, args=(sock,), daemon=True)
        thread.start()
        try:
            with mock.patch.dict(os.environ, {'NETGUN_FIRE_SOCKET': path}):
                info = self.status.fire_link_status()
        finally:
            channel.stop.set()
            thread.join(2)
            sock.close()
        self.assertTrue(info['ok'])
        self.assertEqual(info['link']['phase'], '等待火控节点')
        self.assertNotIn('state', info)

    def test_main_reports_review_and_hides_marker_lines(self):
        log = self.root / 'interceptor_stack_20261023_170000.log'
        log.write_text('NETGUN_REVIEW /x/review_20261023_170000\n[INFO] [1792729673.6] [netgun_fire_server]: 常驻发射通道就绪\n',
                       encoding='utf-8')
        out = io.StringIO()
        with mock.patch.object(self.status, 'stack_pid', return_value=(True, 42)), \
                mock.patch.object(self.status, 'fire_link_status', return_value={'ok': True, 'link': {'ready': True}}), \
                contextlib.redirect_stdout(out):
            self.status.main()
        result = json.loads(out.getvalue())
        self.assertEqual(result['fire_link'], {'ok': True, 'link': {'ready': True}})
        self.assertIn('review', result)
        self.assertEqual(result['tail'], ['[INFO] [1792729673.6] [netgun_fire_server]: 常驻发射通道就绪'])
        out = io.StringIO()
        with mock.patch.object(self.status, 'stack_pid', return_value=(False, None)), \
                mock.patch.object(self.status, 'fire_link_status') as ping, contextlib.redirect_stdout(out):
            self.status.main()
        ping.assert_not_called()
        self.assertNotIn('fire_link', json.loads(out.getvalue()))


if __name__ == '__main__':
    unittest.main()
