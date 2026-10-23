"""run_interceptor_stack.sh（2026-10-23）：start 建复盘记录（bag/参数/安装参数/运行信息）并随链起
常驻发射通道；stop 收尾（录制器 SIGINT 写 metadata、拷入运行日志）。2026-10-23：start 默认装订
实物击发 trigger_command（netgun_pwm_link.py fire），NETGUN_TRIGGER_COMMAND/命令行可覆盖。

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
import threading
import time
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'deploy' / 'run_interceptor_stack.sh'
spec = importlib.util.spec_from_file_location('netgun_fire_button_stack_test', ROOT / 'deploy' / 'netgun_fire_button.py')
fb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fb)
spec = importlib.util.spec_from_file_location('netgun_pwm_link_stack_test', ROOT / 'deploy' / 'netgun_pwm_link.py')
pl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pl)


class StubFcLink:
    """PwmService.status() 所需的最小飞控链路桩（不需要 pymavlink）。"""

    def __init__(self, params=None, alive=True, armed=False):
        self.params = {'PWM_AUX_FUNC5': 301, 'PWM_AUX_DIS5': 0, 'PWM_AUX_FAIL5': 0, 'PWM_AUX_MIN5': 1000,
                       'PWM_AUX_MAX5': 2000, 'PWM_AUX_TIM1': 50, 'COM_MOT_TEST_EN': 1} if params is None else params
        self.alive, self.armed = alive, armed

    def snapshot(self):
        return {'alive': self.alive, 'fc': '2/1' if self.alive else None, 'armed': self.armed,
                'heartbeat_age_s': 0.4 if self.alive else None}

    def param(self, name):
        return (self.params[name], time.monotonic()) if name in self.params else None

    def servo_now(self):
        return {}

ROS2_STUB = r'''#!/usr/bin/env python3
import os, signal, sys, time
args = sys.argv[1:]
out = None
if args[:1] == ['launch']:
    with open(os.environ['ARGS_FILE'], 'w') as f:
        f.write('\n'.join(args) + '\n')
    with open(os.environ['ARGS_FILE'] + '.env', 'w') as f:
        f.write(os.environ.get('NETGUN_REVIEW_DIR', '') + '\n')
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
        for name in ('mounting_geometry.py', 'netgun_fire_button.py', 'netgun_pwm_link.py'):
            shutil.copy(ROOT / 'deploy' / name, self.workspace / 'deploy' / name)
        self.config = self.root / 'config'
        self.logs = self.root / 'logs'
        self.args_file = self.root / 'ros2_args.txt'
        bin_dir = stub_bin(self.root)
        self.script = stack_script_copy(self.root)
        self.env = dict(os.environ, INTERCEPTOR_WS=str(self.workspace), NETGUN_LOG_DIR=str(self.logs),
                        NETGUN_CONFIG_DIR=str(self.config), ARGS_FILE=str(self.args_file), ROS_DOMAIN_ID='199',
                        PATH=str(bin_dir) + os.pathsep + os.environ.get('PATH', ''))
        for name in ('NETGUN_FIRE_SOCKET', 'NETGUN_REVIEW_MIN_FREE_MB', 'NETGUN_PWM_SOCKET', 'NETGUN_TRIGGER_COMMAND'):
            self.env.pop(name, None)
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
        # 实物击发：默认装订常驻击发服务客户端；拦截链进程带本次复盘目录
        trigger = 'python3 %s fire' % (self.workspace / 'deploy' / 'netgun_pwm_link.py')
        launch_args = self.args_file.read_text(encoding='utf-8').splitlines()
        self.assertIn('trigger_command:=' + trigger, launch_args)
        self.assertLess(launch_args.index('trigger_command:=' + trigger), launch_args.index('fire_mode:=auto_manual'),
                        'explicit command-line arguments must come last')
        self.assertIn('NETGUN_TRIGGER %s\n' % trigger, log)
        self.assertIn('实物击发：已装订 netgun_pwm_link；不可用：击发服务未运行', result.stdout)
        self.assertIn('实物击发：已装订 netgun_pwm_link；不可用', log)
        self.assertEqual(pathlib.Path(str(self.args_file) + '.env').read_text().strip(), str(review))
        self.assertIn('trigger_command:=' + trigger, info)
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
        # 三节点在 /pursuer 命名空间下（不带命名空间 ros2 param dump 报 Node not found）
        for p in dumps[1:]:
            self.assertTrue(p.read_text().startswith('pursuer/' + p.stem + ':'), p.read_text())
        self.assertTrue(dumps[0].read_text().startswith('zed_perception_bridge:'))
        # errors.log 在导出循环中先建后删（无错误才删），等循环收尾再判
        self.assertTrue(self.wait_for(lambda: not (review / 'params' / 'errors.log').exists()))
        status = self.run_script('status')
        self.assertIn('常驻发射通道：运行中', status.stdout)
        self.assertIn('实物击发装订：' + trigger, status.stdout)
        self.assertIn('实物击发链路：不可用：击发服务未运行', status.stdout)
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

    def test_trigger_binding_can_be_cleared_or_overridden(self):
        env = dict(self.env, NETGUN_TRIGGER_COMMAND='')
        result = self.run_script('start', env=env)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(any(a.startswith('trigger_command:=') for a in self.args_file.read_text().splitlines()))
        log = next(self.logs.glob('interceptor_stack_*.log'))
        self.assertIn('NETGUN_TRIGGER \n', log.read_text(encoding='utf-8'))
        self.assertIn('实物击发：未装订（仅虚拟开火事件，不驱动网枪）', result.stdout)
        self.assertIn('实物击发装订：未装订', self.run_script('status', env=env).stdout)
        self.assertEqual(self.run_script('stop', env=env).returncode, 0)
        time.sleep(1.1)  # 日志名按秒取时间戳
        result = self.run_script('start', 'trigger_command:=bash /tmp/hook.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.args_file.read_text().splitlines()
        self.assertEqual(args[-1], 'trigger_command:=bash /tmp/hook.sh', 'the explicit argument is last and wins')
        newest = max(self.logs.glob('interceptor_stack_*.log'), key=lambda path: path.name)
        self.assertIn('NETGUN_TRIGGER bash /tmp/hook.sh\n', newest.read_text(encoding='utf-8'))
        self.assertIn('实物击发：已装订自定义命令（bash /tmp/hook.sh）', result.stdout)

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
        log.write_text('NETGUN_REVIEW /x/review_20261023_170000\nNETGUN_TRIGGER python3 /ws/deploy/netgun_pwm_link.py fire\n'
                       '[INFO] [1792729673.6] [netgun_fire_server]: 常驻发射通道就绪\n', encoding='utf-8')
        out = io.StringIO()
        with mock.patch.object(self.status, 'stack_pid', return_value=(True, 42)), \
                mock.patch.object(self.status, 'fire_link_status', return_value={'ok': True, 'link': {'ready': True}}), \
                mock.patch.dict(os.environ, {'NETGUN_PWM_SOCKET': str(self.root / 'none.sock')}), \
                contextlib.redirect_stdout(out):
            self.status.main()
        result = json.loads(out.getvalue())
        self.assertEqual(result['fire_link'], {'ok': True, 'link': {'ready': True}})
        self.assertIn('review', result)
        self.assertEqual(result['tail'], ['[INFO] [1792729673.6] [netgun_fire_server]: 常驻发射通道就绪'])
        self.assertEqual(result['trigger']['bound'], 'python3 /ws/deploy/netgun_pwm_link.py fire')
        self.assertFalse(result['trigger']['ok'])
        self.assertIn('击发服务未运行', result['trigger']['error'])
        out = io.StringIO()
        with mock.patch.object(self.status, 'stack_pid', return_value=(False, None)), \
                mock.patch.object(self.status, 'fire_link_status') as ping, \
                mock.patch.object(self.status, 'trigger_status') as trigger, contextlib.redirect_stdout(out):
            self.status.main()
        ping.assert_not_called()
        trigger.assert_not_called()
        self.assertNotIn('fire_link', json.loads(out.getvalue()))
        self.assertNotIn('trigger', json.loads(out.getvalue()))

    @unittest.skipUnless(hasattr(socket, 'AF_UNIX'), 'Unix sockets required')
    def test_trigger_status_reads_pwm_service(self):
        self.assertEqual(self.status.trigger_status('x\n')['bound'], None, 'old logs have no NETGUN_TRIGGER line')
        self.assertEqual(self.status.trigger_status('NETGUN_TRIGGER \n')['bound'], '')
        path = str(self.root / 'pwm.sock')
        service = pl.PwmService(StubFcLink(), -1.0, 1.0, 1.0, events_path=str(self.root / 'events.jsonl'),
                                log=lambda *a, **k: None)
        sock = service.bind(path)
        thread = threading.Thread(target=service.serve_forever, args=(sock,), daemon=True)
        thread.start()
        try:
            (self.root / 'interceptor_stack.pid').write_text('42\n')
            service.last_fire = {'ok': True, 'mode': 'bench', 'time': time.time() - 3600, 'ack_ms': 20.0}
            with mock.patch.dict(os.environ, {'NETGUN_PWM_SOCKET': path}):
                info = self.status.trigger_status('NETGUN_TRIGGER python3 x fire\n')
                self.assertTrue(info['ok'])
                self.assertTrue(info['ready'])
                self.assertEqual(info['link']['fc'], '2/1')
                self.assertEqual(info['config']['ok'], True)
                self.assertEqual(info['options']['fire_us'], 1000)
                self.assertNotIn('last_fire', info, 'a fire from before this start is not shown')
                service.last_fire = {'ok': False, 'refused': 'DISARMED', 'time': time.time() + 1, 'servo': {'0': {}}}
                info = self.status.trigger_status('NETGUN_TRIGGER python3 x fire\n')
                self.assertEqual(info['last_fire']['refused'], 'DISARMED')
                self.assertNotIn('servo', info['last_fire'])
                service.link.params['PWM_AUX_DIS5'] = 1000
                info = self.status.trigger_status('NETGUN_TRIGGER python3 x fire\n')
                self.assertFalse(info['ready'])
                self.assertEqual(info['config']['problems'], ['PWM_AUX_DIS5=1000（应为 0）'])
        finally:
            service.stop.set()
            thread.join(2)
            sock.close()


if __name__ == '__main__':
    unittest.main()
