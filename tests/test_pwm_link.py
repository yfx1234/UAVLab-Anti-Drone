"""deploy/netgun_pwm_link.py（2026-10-23）：实物击发链路——假飞控（UDP，扮 MAVProxy 转发端）
覆盖 live/bench 指令与参数、解锁门、ACK 重试、拒绝、参数核对、复位、客户端退出码；
preflight_check.py 10b 击发链路检查（rclpy 等以桩模块代替）。"""
import contextlib
import importlib.util
import io
import json
import math
import os
import pathlib
import shutil
import socket
import struct
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('netgun_pwm_link_under_test', ROOT / 'deploy' / 'netgun_pwm_link.py')
pl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pl)

try:
    from pymavlink.dialects.v20 import common as mavlink
except ImportError:  # 板上/开发机未装 pymavlink 时只跑客户端部分
    mavlink = None

GOOD_PARAMS = {'PWM_AUX_FUNC5': 301, 'PWM_AUX_DIS5': 0, 'PWM_AUX_FAIL5': 0, 'PWM_AUX_MIN5': 1000,
               'PWM_AUX_MAX5': 2000, 'PWM_AUX_TIM1': 50, 'COM_MOT_TEST_EN': 1, 'COM_PREARM_MODE': 0,
               'PWM_MAIN_FUNC1': 0, 'PWM_MAIN_FUNC5': 1, 'PWM_AUX_FUNC1': 103, 'PWM_AUX_FUNC2': 102,
               'PWM_AUX_FUNC3': 101, 'PWM_AUX_FUNC4': 104}


class FakeFc:
    """扮演 MAVProxy 转发端 + PX4：发心跳/舵机输出，回 PARAM_VALUE 与 COMMAND_ACK。"""

    def __init__(self, port, params=None, armed=False, ack_result=0, drop_acks=0, heartbeat=True):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.05)
        self.dest = ('127.0.0.1', port)
        self.mav = mavlink.MAVLink(None, srcSystem=2, srcComponent=1)
        self.qgc = mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
        self.params = dict(GOOD_PARAMS if params is None else params)
        self.armed, self.ack_result, self.drop_acks, self.heartbeat = armed, ack_result, drop_acks, heartbeat
        self.commands, self.param_requests, self.aux5 = [], [], 0
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.threads = [threading.Thread(target=self._rx, daemon=True), threading.Thread(target=self._tx, daemon=True)]
        for thread in self.threads:
            thread.start()

    def send(self, msg, mav=None):
        self.sock.sendto(msg.pack(mav or self.mav), self.dest)

    def _tx(self):
        while not self.stop.is_set():
            if self.heartbeat:
                # QGC 心跳（type=GCS）也会被转发——不能拿来判断解锁
                self.send(self.qgc.heartbeat_encode(6, 8, 128 if not self.armed else 0, 0, 4), self.qgc)
                self.send(self.mav.heartbeat_encode(2, 12, 128 | 1 if self.armed else 1, 0, 3))
                self.send(self.mav.attitude_encode(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                self.send(self.mav.servo_output_raw_encode(0, 0, 0, 0, 0, 0, self.aux5, 0, 0, 0))
            self.stop.wait(0.1)

    def _rx(self):
        while not self.stop.is_set():
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            msg = self.mav.decode(bytearray(data))
            name = msg.get_type()
            if name == 'PARAM_REQUEST_READ':
                pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
                with self.lock:
                    self.param_requests.append(pid)
                if pid in self.params:
                    raw = struct.unpack('<f', struct.pack('<i', self.params[pid]))[0]
                    self.send(self.mav.param_value_encode(pid.encode(), raw, 6, 1000, 1))
            elif name == 'COMMAND_LONG':
                with self.lock:
                    self.commands.append(msg)
                if msg.command == 310:
                    self.aux5 = 0 if msg.param2 <= 0 else int(1500 + 500 * msg.param1)
                elif msg.command == 187 and self.armed:
                    self.aux5 = int(1500 + 500 * msg.param1)
                if self.drop_acks > 0:
                    self.drop_acks -= 1
                    continue
                self.send(self.mav.command_ack_encode(msg.command, self.ack_result, 0, 0,
                                                      msg.get_srcSystem(), msg.get_srcComponent()))

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(1)
        self.sock.close()


@unittest.skipIf(mavlink is None, 'pymavlink required')
class PwmLinkTests(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix='pwm_link_'))
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.fc = None
        self.logs = []

    def start(self, safe_value=1.0, hold_s=0.2, **fc):
        self.link = pl.FcLink(mavlink, ('127.0.0.1', 0), log=lambda *a, **k: self.logs.append(a))
        self.service = pl.PwmService(self.link, -1.0, safe_value, hold_s, events_path=str(self.tmp / 'events.jsonl'),
                                     log=lambda *a, **k: self.logs.append(a))
        self.threads = [threading.Thread(target=self.link.run, args=(self.service.stop,), daemon=True),
                        threading.Thread(target=self._maintain, daemon=True)]
        for thread in self.threads:
            thread.start()
        self.addCleanup(self.stop)
        self.fc = FakeFc(self.link.listen[1], **fc)
        return self.service

    def _maintain(self):
        while not self.service.stop.is_set():
            self.service.maintain()
            self.service.stop.wait(0.05)

    def stop(self):
        self.service.stop.set()
        if self.service.busy.acquire(timeout=5):
            self.service.busy.release()
        for thread in self.threads:
            thread.join(2)
        if self.fc:
            self.fc.close()
        self.link.close()

    def wait_for(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        return predicate()

    def wait_ready(self):
        self.assertTrue(self.wait_for(lambda: self.service.status()['ready']), self.service.status())

    def wait_idle(self):
        self.assertTrue(self.wait_for(lambda: not self.service.busy.locked()))

    def events(self, path=None):
        path = path or self.tmp / 'events.jsonl'
        return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]

    def commands(self):
        with self.fc.lock:
            return list(self.fc.commands)

    def test_live_armed_fires_then_resets_to_safe_value(self):
        self.start(armed=True)
        self.wait_ready()
        status = self.service.status()
        self.assertEqual(status['link']['fc'], '2/1', 'the QGC heartbeat must not be taken for the autopilot')
        self.assertTrue(status['link']['armed'])
        review = self.tmp / 'review_x'
        review.mkdir()
        reply = self.service.fire({'mode': 'live', 'shot_id': 1, 'reason': 'MANUAL', 'review_dir': str(review)})
        self.assertTrue(reply['ok'], reply)
        self.assertEqual((reply['result_name'], reply['attempts'], reply['pwm_us']), ('ACCEPTED', 1, 1000))
        self.assertLess(reply['ack_ms'], 250)
        self.assertNotIn('servo_before', reply)
        first = self.commands()[0]
        self.assertEqual((first.command, first.target_system, first.target_component, first.confirmation), (187, 2, 1, 0))
        self.assertEqual((first.get_srcSystem(), first.get_srcComponent()), (245, 191))
        self.assertEqual(first.param1, -1.0)
        self.assertTrue(all(math.isnan(v) for v in (first.param2, first.param3, first.param4, first.param5, first.param6)))
        self.assertEqual(first.param7, 0.0)
        self.assertTrue(self.service.busy.locked(), 'hold/reset runs after the reply')
        self.wait_idle()
        reset = self.commands()[1]
        self.assertEqual((reset.command, reset.param1), (187, 1.0))
        self.assertEqual(len(self.commands()), 2)
        record = self.events()[-1]
        self.assertEqual(record['reset']['result_name'], 'ACCEPTED')
        self.assertEqual(record['reset']['pwm_us'], 2000)
        self.assertEqual(record['shot_id'], 1)
        self.assertIn('1000', json.dumps(record['servo']), 'servo read-back during the hold is recorded')
        self.assertEqual(self.events(review / 'pwm_events.jsonl'), [record])
        self.assertFalse(self.service.status()['last_fire'].get('pending'))

    def test_live_disarmed_is_refused_without_sending(self):
        self.start(armed=False)
        self.wait_ready()
        reply = self.service.fire({'mode': 'live', 'shot_id': 1})
        self.assertFalse(reply['ok'])
        self.assertEqual(reply['refused'], 'DISARMED')
        self.assertIn('锁存', reply['reason_text'])
        time.sleep(0.3)
        self.assertEqual(self.commands(), [], 'a disarmed live trigger would latch in PX4 and fire at arming')
        self.assertEqual(self.events()[-1]['refused'], 'DISARMED')

    def test_bench_uses_actuator_test_then_safe_then_release(self):
        with mock.patch.object(pl, 'BENCH_SAFE_S', 0.05):
            self.start(armed=False)
            self.wait_ready()
            reply = self.service.fire({'mode': 'bench'})
            self.assertTrue(reply['ok'], reply)
            self.wait_idle()
        test, safe, release = self.commands()
        self.assertEqual((test.command, test.param1, test.param5), (310, -1.0, 1301.0))
        self.assertAlmostEqual(test.param2, 0.25, places=5)
        self.assertEqual((safe.command, safe.param1, safe.param5), (310, 1.0, 1301.0))
        self.assertGreater(safe.param2, 0)
        self.assertEqual((release.command, release.param2, release.param5), (310, 0.0, 1301.0))
        record = self.events()[-1]
        self.assertEqual((record['reset']['result_name'], record['reset']['release']), ('ACCEPTED', 'ACCEPTED'))

    def test_bench_refused_when_armed_and_without_motor_test(self):
        self.start(armed=True)
        self.wait_ready()
        self.assertEqual(self.service.fire({'mode': 'bench'})['refused'], 'ARMED')
        self.fc.armed = False
        self.fc.params['COM_MOT_TEST_EN'] = 0
        self.service.param_sent = None
        self.assertTrue(self.wait_for(lambda: self.service.config()['values'].get('COM_MOT_TEST_EN') == 0
                                      and self.service.link.snapshot()['armed'] is False))
        self.assertIn('台架模式不可用', '；'.join(self.service.config()['warnings']))
        self.assertEqual(self.service.fire({'mode': 'bench'})['refused'], 'MOT_TEST_DISABLED')
        self.assertEqual(self.commands(), [])

    def test_no_heartbeat_refuses_no_link(self):
        self.start(heartbeat=False)
        time.sleep(0.2)
        status = self.service.status()
        self.assertFalse(status['ready'])
        self.assertIn('飞控无心跳', pl.brief(status))
        reply = self.service.fire({'mode': 'live'})
        self.assertEqual(reply['refused'], 'NO_LINK')

    def test_lost_ack_is_retried_with_confirmation(self):
        self.start(armed=True, drop_acks=1)
        self.wait_ready()
        reply = self.service.fire({'mode': 'live', 'shot_id': 1})
        self.assertTrue(reply['ok'], reply)
        self.assertEqual(reply['attempts'], 2)
        self.assertGreaterEqual(reply['ack_ms'], 250)
        commands = self.commands()
        self.assertEqual([c.confirmation for c in commands[:2]], [0, 1])
        self.wait_idle()

    def test_rejection_is_reported_and_reset_still_sent(self):
        self.start(armed=True, ack_result=2)
        self.wait_ready()
        reply = self.service.fire({'mode': 'live'})
        self.assertFalse(reply['ok'])
        self.assertEqual(reply['result_name'], 'DENIED')
        self.assertIn('DENIED', reply['reason_text'])
        self.wait_idle()
        self.assertEqual([c.param1 for c in self.commands()], [-1.0, 1.0])

    def test_no_ack_reports_after_three_attempts(self):
        self.start(armed=True, drop_acks=99)
        self.wait_ready()
        reply = self.service.fire({'mode': 'live'})
        self.assertFalse(reply['ok'])
        self.assertEqual((reply['result_name'], reply['attempts']), ('NO_ACK', 3))
        self.wait_idle()

    def test_configuration_mismatch_or_duplicate_output_refuses(self):
        params = dict(GOOD_PARAMS, PWM_AUX_DIS5=1000)
        self.start(armed=True, params=params)
        self.assertTrue(self.wait_for(lambda: self.service.config()['ok'] is False))
        status = self.service.status()
        self.assertFalse(status['ready'])
        self.assertIn('PWM_AUX_DIS5=1000（应为 0）', pl.brief(status))
        reply = self.service.fire({'mode': 'live'})
        self.assertEqual(reply['refused'], 'CONFIG')
        self.fc.params.update(PWM_AUX_DIS5=0, PWM_MAIN_FUNC1=301)
        self.service.param_sent = None
        self.assertTrue(self.wait_for(lambda: any('PWM_MAIN_FUNC1=301' in p for p in self.service.config()['problems'])))
        self.assertEqual(self.service.fire({'mode': 'live'})['refused'], 'CONFIG')
        self.assertEqual(self.commands(), [])

    def test_missing_parameters_keep_link_not_ready(self):
        params = dict(GOOD_PARAMS)
        del params['PWM_AUX_TIM1']
        self.start(armed=True, params=params)
        self.assertTrue(self.wait_for(lambda: self.service.config()['missing'] == ['PWM_AUX_TIM1']))
        self.assertIsNone(self.service.config()['ok'])
        self.assertIn('参数核对中', pl.brief(self.service.status()))
        self.assertEqual(self.service.fire({'mode': 'live'})['refused'], 'CONFIG')

    def test_safe_none_keeps_fire_value_and_busy_blocks_second_request(self):
        self.start(armed=True, safe_value=None, hold_s=0.3)
        self.wait_ready()
        self.assertIn('不复位', pl.brief(self.service.status()))
        self.assertTrue(self.service.fire({'mode': 'live'})['ok'])
        self.assertEqual(self.service.fire({'mode': 'live'})['refused'], 'BUSY')
        self.wait_idle()
        self.assertEqual(len(self.commands()), 1)
        self.assertIn('skipped', self.events()[-1]['reset'])

    def test_unix_socket_client_fire_and_status(self):
        service = self.start(armed=True)
        self.wait_ready()
        path = str(self.tmp / 'pwm.sock')
        sock = service.bind(path)
        self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)
        server = threading.Thread(target=service.serve_forever, args=(sock,), daemon=True)
        server.start()
        self.addCleanup(sock.close)
        env = {'NETGUN_PWM_SOCKET': path, 'NETGUN_SHOT_ID': '1', 'NETGUN_SHOT_REASON': 'MANUAL',
               'NETGUN_REVIEW_DIR': str(self.tmp)}
        out = io.StringIO()
        with mock.patch.dict(os.environ, env), contextlib.redirect_stdout(out):
            code = pl.main(['x', 'fire'])
        self.assertEqual(code, 0, out.getvalue())
        line = out.getvalue().strip()
        self.assertTrue(line.startswith('NETGUN_PWM_RESULT {'), line)
        self.assertNotIn('NETGUN FIRE', line, 'status counters count NETGUN FIRE lines')
        reply = json.loads(line.split(' ', 1)[1])
        self.assertEqual((reply['ok'], reply['mode'], reply['shot_id'], reply['reason'], reply['source']),
                         (True, 'live', 1, 'MANUAL', 'trigger_command'))
        self.assertIsInstance(reply['client_ms'], float)
        self.wait_idle()
        self.assertTrue((self.tmp / 'pwm_events.jsonl').exists())
        with mock.patch.dict(os.environ, {'NETGUN_PWM_SOCKET': path}):
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(pl.main(['x', 'status', '--brief']), 0)
            self.assertRegex(out.getvalue(), r'^实物击发链路：就绪（飞控 2/1 · 已解锁 · AUX5 -1→1000 µs 保持 0\.2 s → \+1→2000 µs）')
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(pl.main(['x', 'fire', '--bench']), 1)
            self.assertEqual(json.loads(out.getvalue().split(' ', 1)[1])['refused'], 'ARMED')
        service.stop.set()
        server.join(2)
        service.unlink()
        self.assertFalse(os.path.exists(path))


class StubFcLink:
    """PwmService.status() 所需的最小飞控链路桩（不需要 pymavlink）。"""

    def __init__(self):
        self.params = {name: value for name, value in pl.EXPECTED}
        self.params['COM_MOT_TEST_EN'] = 1

    def snapshot(self):
        return {'alive': True, 'fc': '2/1', 'armed': False, 'heartbeat_age_s': 0.3}

    def param(self, name):
        return (self.params[name], time.monotonic()) if name in self.params else None

    def servo_now(self):
        return {}


@unittest.skipUnless(hasattr(socket, 'AF_UNIX'), 'Unix sockets required')
class PreflightPwmCheckTests(unittest.TestCase):
    def setUp(self):
        stubs = {name: types.ModuleType(name) for name in (
            'rclpy', 'rclpy.node', 'rclpy.qos', 'rcl_interfaces', 'rcl_interfaces.srv', 'px4_msgs', 'px4_msgs.msg',
            'std_msgs', 'std_msgs.msg')}
        stubs['rclpy.node'].Node = object
        stubs['rclpy.qos'].qos_profile_sensor_data = None
        stubs['rcl_interfaces.srv'].GetParameters = object
        stubs['std_msgs.msg'].String = object
        for name in ('VehicleAttitude', 'VehicleLocalPosition', 'VehicleStatus'):
            setattr(stubs['px4_msgs.msg'], name, object)
        with mock.patch.dict(sys.modules, stubs):
            spec = importlib.util.spec_from_file_location('preflight_under_test', ROOT / 'deploy' / 'preflight_check.py')
            self.preflight = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.preflight)
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix='pwm_preflight_'))
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def test_pwm_link_check_outcomes(self):
        path = str(self.tmp / 'pwm.sock')
        with mock.patch.dict(os.environ, {'NETGUN_PWM_SOCKET': path}):
            name, result, detail = self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire')
            self.assertEqual(result, 'FAIL')
            self.assertIn('击发服务未运行', detail)
            service = pl.PwmService(StubFcLink(), -1.0, 1.0, 1.0, events_path=str(self.tmp / 'e.jsonl'),
                                    log=lambda *a, **k: None)
            sock = service.bind(path)
            thread = threading.Thread(target=service.serve_forever, args=(sock,), daemon=True)
            thread.start()
            try:
                name, result, detail = self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire')
                self.assertEqual(result, 'PASS', detail)
                self.assertIn('FMU PWM OUT 5', name)
                self.assertIn('就绪（飞控 2/1 · 未解锁', detail)
                self.assertEqual(self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire --bench')[1], 'WARN')
                service.link.params['COM_PREARM_MODE'] = 1
                result, detail = self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire')[1:]
                self.assertEqual(result, 'WARN')
                self.assertIn('COM_PREARM_MODE', detail)
                service.link.params['PWM_AUX_FUNC5'] = 0
                result, detail = self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire')[1:]
                self.assertEqual(result, 'FAIL')
                self.assertIn('PWM_AUX_FUNC5=0（应为 301）', detail)
            finally:
                service.stop.set()
                thread.join(2)
                sock.close()


class ClientOnlyTests(unittest.TestCase):
    def test_unreachable_service_exit_codes(self):
        missing = os.path.join(tempfile.gettempdir(), 'no_such_pwm_%d.sock' % os.getpid())
        with mock.patch.dict(os.environ, {'NETGUN_PWM_SOCKET': missing}):
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(pl.main(['x', 'fire']), 2)
            reply = json.loads(out.getvalue().split(' ', 1)[1])
            self.assertEqual(reply['refused'], 'UNAVAILABLE')
            self.assertIn('未发出击发指令', reply['reason_text'])
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(pl.main(['x', 'status', '--brief']), 2)
            self.assertIn('实物击发链路：不可用：击发服务未运行', out.getvalue())
            self.assertFalse(pl.status()['ok'])

    def test_default_socket_follows_log_dir(self):
        with mock.patch.dict(os.environ, {'NETGUN_LOG_DIR': '/x/logs'}):
            os.environ.pop('NETGUN_PWM_SOCKET', None)
            self.assertEqual(pl.socket_path(), '/x/logs/netgun_pwm.sock')
        self.assertEqual((pl.pwm_us(-1.0), pl.pwm_us(1.0), pl.pwm_us(0.0), pl.pwm_us(None)), (1000, 2000, 1500, None))

    def test_param_number_decodes_px4_int_bytewise(self):
        class Msg:
            param_type = 6

            def __init__(self, value):
                self.param_value = struct.unpack('<f', struct.pack('<i', value))[0]

            def get_msgbuf(self):
                raise RuntimeError('no buffer')
        for value in (0, 50, 301, 1000, 2000, -1):
            self.assertEqual(pl.param_number(Msg(value)), value)


if __name__ == '__main__':
    unittest.main()
