"""deploy/netgun_pwm_link.py：实物击发链路——假飞控（UDP，扮 MAVProxy 转发端）。
2026-10-23 建链；2026-10-24 按网枪触发模块说明改为"低位待命 → 跳高位击发"：服务启动即锁存
待命低位、live 高位→保持→回低位、解锁后先等低位、bench 低位→高位→低位→释放、击发参数
（文件/套接字/命令行/服务未运行时直接写文件）、链路中断后重新锁存；preflight_check.py 10b
击发链路检查（rclpy 等以桩模块代替）。"""
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
               'PWM_AUX_REV': 0, 'PWM_MAIN_FUNC1': 0, 'PWM_MAIN_FUNC5': 1, 'PWM_AUX_FUNC1': 103,
               'PWM_AUX_FUNC2': 102, 'PWM_AUX_FUNC3': 101, 'PWM_AUX_FUNC4': 104}
FAST = {'low_us': 1000, 'high_us': 2000, 'hold_s': 0.3, 'low_s': 0.2}


class FakeFc:
    """扮演 MAVProxy 转发端 + PX4：发心跳/舵机输出，回 PARAM_VALUE 与 COMMAND_ACK。
    AUX5：执行器测试中 = 测试值；否则已解锁 = Actuator Set 锁存值，未解锁 = 无脉冲（DIS5=0）。"""

    def __init__(self, port, params=None, armed=False, ack_result=0, drop_acks=0, heartbeat=True):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.05)
        self.dest = ('127.0.0.1', port)
        self.mav = mavlink.MAVLink(None, srcSystem=2, srcComponent=1)
        self.qgc = mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
        self.params = dict(GOOD_PARAMS if params is None else params)
        self.armed, self.ack_result, self.drop_acks, self.heartbeat = armed, ack_result, drop_acks, heartbeat
        self.commands, self.times, self.param_requests = [], [], []
        self.latched, self.test_value = None, None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.threads = [threading.Thread(target=self._rx, daemon=True), threading.Thread(target=self._tx, daemon=True)]
        for thread in self.threads:
            thread.start()

    def aux5(self):
        value = self.test_value if self.test_value is not None else (self.latched if self.armed else None)
        return 0 if value is None else int(round(1500 + 500 * value))

    def send(self, msg, mav=None):
        self.sock.sendto(msg.pack(mav or self.mav), self.dest)

    def _tx(self):
        while not self.stop.is_set():
            if self.heartbeat:
                # QGC 心跳（type=GCS）也会被转发——不能拿来判断解锁
                self.send(self.qgc.heartbeat_encode(6, 8, 128 if not self.armed else 0, 0, 4), self.qgc)
                self.send(self.mav.heartbeat_encode(2, 12, 128 | 1 if self.armed else 1, 0, 3))
                self.send(self.mav.attitude_encode(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                self.send(self.mav.servo_output_raw_encode(0, 0, 0, 0, 0, 0, self.aux5(), 0, 0, 0))
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
                    self.times.append(time.monotonic())
                if msg.command == 310:
                    self.test_value = msg.param1 if msg.param2 > 0 else None
                elif msg.command == 187 and math.isfinite(msg.param1):
                    self.latched = msg.param1  # PX4 未解锁时也接受并锁存
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

    def start(self, params=FAST, params_file=None, fc_params=None, **fc):
        self.link = pl.FcLink(mavlink, ('127.0.0.1', 0), log=lambda *a, **k: self.logs.append(a))
        self.service = pl.PwmService(self.link, params, events_path=str(self.tmp / 'events.jsonl'),
                                     params_file=str(params_file or self.tmp / 'netgun_pwm.json'),
                                     log=lambda *a, **k: self.logs.append(a))
        self.threads = [threading.Thread(target=self.link.run, args=(self.service.stop,), daemon=True),
                        threading.Thread(target=self._maintain, daemon=True)]
        for thread in self.threads:
            thread.start()
        self.addCleanup(self.stop)
        self.fc = FakeFc(self.link.listen[1], params=fc_params, **fc)
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

    def wait_armed(self, armed=True):
        self.assertTrue(self.wait_for(lambda: self.link.snapshot()['armed'] is armed))

    def events(self, path=None):
        path = path or self.tmp / 'events.jsonl'
        return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]

    def commands(self, since=0):
        with self.fc.lock:
            return list(self.fc.commands[since:])

    def count(self):
        with self.fc.lock:
            return len(self.fc.commands)

    def test_low_is_latched_at_start_and_live_fire_goes_high_then_back_low(self):
        self.start(armed=True)
        self.wait_ready()
        status = self.service.status()
        self.assertEqual(status['link']['fc'], '2/1', 'the QGC heartbeat must not be taken for the autopilot')
        self.assertTrue(status['link']['armed'])
        self.assertEqual((status['idle']['latched'], status['idle']['low_us']), (True, 1000))
        latch = self.commands()[0]
        self.assertEqual((latch.command, latch.target_system, latch.target_component, latch.confirmation), (187, 2, 1, 0))
        self.assertEqual((latch.get_srcSystem(), latch.get_srcComponent()), (245, 191))
        self.assertEqual(latch.param1, -1.0, 'idle LOW 1000 µs is latched before any fire')
        self.assertTrue(all(math.isnan(v) for v in (latch.param2, latch.param3, latch.param4, latch.param5, latch.param6)))
        self.assertEqual(latch.param7, 0.0)
        self.assertTrue(self.wait_for(lambda: self.fc.aux5() == 1000))
        time.sleep(0.8)  # 解锁斜坡 + 低位确认已过：不再等待
        before = self.count()
        review = self.tmp / 'review_x'
        review.mkdir()
        reply = self.service.fire({'mode': 'live', 'shot_id': 1, 'reason': 'MANUAL', 'review_dir': str(review)})
        self.assertTrue(reply['ok'], reply)
        self.assertEqual((reply['result_name'], reply['attempts'], reply['pwm_us']), ('ACCEPTED', 1, 2000))
        self.assertEqual((reply['low_us'], reply['high_us'], reply['hold_s'], reply['low_s']), (1000, 2000, 0.3, 0.2))
        self.assertNotIn('low_wait_ms', reply)
        self.assertNotIn('pre_low', reply)
        self.assertLess(reply['ack_ms'], 250)
        self.assertNotIn('servo_before', reply)
        high = self.commands(before)[0]
        self.assertEqual((high.command, high.param1, high.param7), (187, 1.0, 0.0))
        self.assertEqual(self.fc.aux5(), 2000)
        self.assertTrue(self.service.busy.locked(), 'hold and the return to LOW run after the reply')
        self.wait_idle()
        sent = self.commands(before)
        self.assertEqual([(c.command, c.param1) for c in sent], [(187, 1.0), (187, -1.0)])
        with self.fc.lock:
            held = self.fc.times[before + 1] - self.fc.times[before]
        self.assertGreaterEqual(held, 0.28, 'HIGH is held for hold_s')
        self.assertEqual(self.fc.aux5(), 1000)
        record = self.events()[-1]
        self.assertEqual((record['reset']['result_name'], record['reset']['pwm_us']), ('ACCEPTED', 1000))
        self.assertEqual(record['shot_id'], 1)
        self.assertIn('2000', json.dumps(record['servo']), 'servo read-back during the hold is recorded')
        self.assertEqual(self.events(review / 'pwm_events.jsonl'), [record])
        self.assertFalse(self.service.status()['last_fire'].get('pending'))
        self.assertTrue(self.service.status()['ready'], 'the LOW latch is confirmed again after the fire')

    def test_live_fire_right_after_arming_waits_for_the_low_level(self):
        self.start(armed=False)
        self.wait_ready()
        self.assertEqual(self.fc.aux5(), 0, 'disarmed: no pulse even with LOW latched')
        before = self.count()
        self.fc.armed = True
        armed_at = time.monotonic()
        self.wait_armed()
        reply = self.service.fire({'mode': 'live', 'shot_id': 1})
        self.assertTrue(reply['ok'], reply)
        self.assertGreater(reply['low_wait_ms'], 300)
        with self.fc.lock:
            high_at = self.fc.times[before]
        self.assertGreaterEqual(high_at - armed_at, pl.ARM_RAMP_S + FAST['low_s'] - 0.02,
                                'arming ramp 0.5 s + low_s before the HIGH step')
        self.wait_idle()

    def test_disarmed_live_is_refused_and_only_the_low_latch_was_sent(self):
        self.start(armed=False)
        self.wait_ready()
        reply = self.service.fire({'mode': 'live', 'shot_id': 1})
        self.assertFalse(reply['ok'])
        self.assertEqual(reply['refused'], 'DISARMED')
        self.assertIn('锁存', reply['reason_text'])
        time.sleep(0.3)
        self.assertEqual([(c.command, c.param1) for c in self.commands()], [(187, -1.0)],
                         'a HIGH sent while disarmed would latch in PX4 and fire at arming')
        self.assertEqual(self.events()[-1]['refused'], 'DISARMED')

    def test_bench_runs_low_high_low_then_release(self):
        self.start(armed=False)
        self.wait_ready()
        before = self.count()
        reply = self.service.fire({'mode': 'bench'})
        self.assertTrue(reply['ok'], reply)
        self.assertEqual(reply['low_wait_ms'], 700.0)
        self.assertEqual(reply['pre_low']['result_name'], 'ACCEPTED')
        self.assertEqual(self.fc.aux5(), 2000)
        self.wait_idle()
        low, high, back, release = self.commands(before)
        self.assertEqual((low.command, low.param1, low.param5), (310, -1.0, 1301.0))
        self.assertAlmostEqual(low.param2, 1.2, places=5)
        self.assertEqual((high.command, high.param1, high.param5), (310, 1.0, 1301.0))
        self.assertAlmostEqual(high.param2, 0.8, places=5)
        self.assertEqual((back.command, back.param1), (310, -1.0))
        self.assertAlmostEqual(back.param2, 0.7, places=5)
        self.assertEqual((release.command, release.param2, release.param5), (310, 0.0, 1301.0))
        with self.fc.lock:
            times = self.fc.times[before:]
        self.assertGreaterEqual(times[1] - times[0], 0.68, 'LOW for ramp + low_s before HIGH')
        self.assertGreaterEqual(times[2] - times[1], 0.28, 'HIGH for hold_s')
        self.assertGreaterEqual(times[3] - times[2], 0.18, 'LOW for low_s before release')
        self.assertEqual(self.fc.aux5(), 0)
        record = self.events()[-1]
        self.assertEqual((record['reset']['result_name'], record['reset']['release']), ('ACCEPTED', 'ACCEPTED'))

    def test_bench_refused_when_armed_and_without_motor_test(self):
        self.start(armed=True)
        self.wait_ready()
        before = self.count()
        self.assertEqual(self.service.fire({'mode': 'bench'})['refused'], 'ARMED')
        self.fc.armed = False
        self.fc.params['COM_MOT_TEST_EN'] = 0
        self.service.param_sent = None
        self.assertTrue(self.wait_for(lambda: self.service.config()['values'].get('COM_MOT_TEST_EN') == 0
                                      and self.service.link.snapshot()['armed'] is False))
        self.assertIn('台架模式不可用', '；'.join(self.service.config()['warnings']))
        self.assertEqual(self.service.fire({'mode': 'bench'})['refused'], 'MOT_TEST_DISABLED')
        self.assertEqual(self.commands(before), [])

    def test_no_heartbeat_refuses_no_link(self):
        self.start(heartbeat=False)
        time.sleep(0.2)
        status = self.service.status()
        self.assertFalse(status['ready'])
        self.assertIn('飞控无心跳', pl.brief(status))
        reply = self.service.fire({'mode': 'live'})
        self.assertEqual(reply['refused'], 'NO_LINK')
        self.assertEqual(self.commands(), [])

    def test_lost_ack_is_retried_with_confirmation(self):
        self.start(armed=True)
        self.wait_ready()
        time.sleep(0.8)
        before = self.count()
        self.fc.drop_acks = 1
        reply = self.service.fire({'mode': 'live', 'shot_id': 1})
        self.assertTrue(reply['ok'], reply)
        self.assertEqual(reply['attempts'], 2)
        self.assertGreaterEqual(reply['ack_ms'], 250)
        self.assertEqual([(c.param1, c.confirmation) for c in self.commands(before)[:2]], [(1.0, 0), (1.0, 1)])
        self.wait_idle()

    def test_rejection_is_reported_and_low_is_still_sent_then_retried(self):
        self.start(armed=True)
        self.wait_ready()
        before = self.count()
        self.fc.ack_result = 2
        reply = self.service.fire({'mode': 'live'})
        self.assertFalse(reply['ok'])
        self.assertEqual(reply['result_name'], 'DENIED')
        self.assertIn('DENIED', reply['reason_text'])
        self.wait_idle()
        values = [c.param1 for c in self.commands(before)]
        self.assertEqual(values[:2], [1.0, -1.0])
        self.assertTrue(all(v == -1.0 for v in values[1:]))
        self.assertFalse(self.service.status()['ready'], 'a refused LOW is not counted as latched')
        self.fc.ack_result = 0
        self.wait_ready()  # 每秒补发低位，飞控恢复接受后重新就绪

    def test_no_ack_reports_after_three_attempts(self):
        self.start(armed=True)
        self.wait_ready()
        time.sleep(0.8)
        self.fc.drop_acks = 99
        reply = self.service.fire({'mode': 'live'})
        self.assertFalse(reply['ok'])
        self.assertEqual((reply['result_name'], reply['attempts']), ('NO_ACK', 3))
        self.fc.drop_acks = 0
        self.wait_idle()

    def test_unconfirmed_low_refuses_without_sending_high(self):
        self.start(armed=True, ack_result=2)
        self.assertTrue(self.wait_for(lambda: self.service.idle()['error'] is not None))
        status = self.service.status()
        self.assertFalse(status['ready'])
        self.assertEqual(pl.brief(status), '待命低位未设好（飞控未确认低位 1000 µs（DENIED））')
        reply = self.service.fire({'mode': 'live', 'shot_id': 1})
        self.assertEqual(reply['refused'], 'LOW_FAILED')
        self.assertEqual(reply['pre_low']['result_name'], 'DENIED')
        self.assertTrue(all(c.param1 == -1.0 for c in self.commands()), 'no HIGH without a confirmed LOW')

    def test_configuration_mismatch_duplicate_output_or_reversed_aux5_refuses(self):
        params = dict(GOOD_PARAMS, PWM_AUX_DIS5=1500)
        self.start(armed=True, fc_params=params)
        self.assertTrue(self.wait_for(lambda: self.service.config()['ok'] is False))
        status = self.service.status()
        self.assertFalse(status['ready'])
        self.assertIn('PWM_AUX_DIS5=1500（应为 0 或 800–1250）', pl.brief(status))
        self.assertEqual(self.service.fire({'mode': 'live'})['refused'], 'CONFIG')
        self.fc.params.update(PWM_AUX_DIS5=1000, PWM_MAIN_FUNC1=301, PWM_AUX_REV=16)
        self.service.param_sent = None
        # DIS5=1000（未解锁也给低位）可接受；另一路 301、AUX5 反向不行
        expected = ['PWM_MAIN_FUNC1=301（Actuator Set 1 同时驱动另一路输出）',
                    'PWM_AUX_REV=16 含 AUX5（输出反向：待命低位会变成高位）']
        self.assertTrue(self.wait_for(lambda: self.service.config()['problems'] == expected), self.service.config())
        self.assertEqual(self.service.fire({'mode': 'live'})['refused'], 'CONFIG')
        self.assertEqual(self.commands(), [], 'nothing is latched while the configuration is wrong')

    def test_missing_parameters_keep_link_not_ready(self):
        params = dict(GOOD_PARAMS)
        del params['PWM_AUX_TIM1']
        self.start(armed=True, fc_params=params)
        self.assertTrue(self.wait_for(lambda: self.service.config()['missing'] == ['PWM_AUX_TIM1']))
        self.assertIsNone(self.service.config()['ok'])
        self.assertIn('参数核对中', pl.brief(self.service.status()))
        self.assertEqual(self.service.fire({'mode': 'live'})['refused'], 'CONFIG')

    def test_busy_blocks_a_second_request(self):
        self.start(armed=True)
        self.wait_ready()
        time.sleep(0.8)
        before = self.count()
        self.assertTrue(self.service.fire({'mode': 'live'})['ok'])
        self.assertEqual(self.service.fire({'mode': 'live'})['refused'], 'BUSY')
        self.wait_idle()
        self.assertEqual([c.param1 for c in self.commands(before)], [1.0, -1.0])

    def test_link_loss_forgets_the_latch_and_relatches_after_recovery(self):
        with mock.patch.object(pl, 'HEARTBEAT_STALE_S', 0.4):
            self.start(armed=False)
            self.wait_ready()
            self.fc.heartbeat = False
            self.assertTrue(self.wait_for(lambda: not self.service.idle()['latched']))
            self.assertFalse(self.service.status()['ready'])
            self.fc.heartbeat = True
            self.wait_ready()
        self.assertEqual([(c.command, c.param1) for c in self.commands()], [(187, -1.0), (187, -1.0)])

    def test_invalid_params_file_refuses_and_latches_nothing(self):
        path = self.tmp / 'netgun_pwm.json'
        path.write_text(json.dumps({'low_us': 1000, 'high_us': 1500, 'hold_s': 1.0, 'low_s': 0.3}), encoding='utf-8')
        self.start(params=None, params_file=path, armed=True)
        self.assertTrue(self.wait_for(lambda: self.service.config()['ok'] is True))
        time.sleep(0.2)
        status = self.service.status()
        self.assertFalse(status['ready'])
        self.assertIsNone(status['options']['high_us'])
        self.assertIn('击发高位 1500 µs 超出范围 1750–2000 µs', status['options']['error'])
        self.assertIn('击发参数无效：', pl.brief(status))
        reply = self.service.fire({'mode': 'live'})
        self.assertEqual(reply['refused'], 'PARAMS')
        self.assertEqual(self.commands(), [])

    def test_params_via_socket_cli_and_offline(self):
        service = self.start(params=None, armed=False)
        self.wait_ready()
        self.assertEqual(service.options()['source'], 'default')
        path = str(self.tmp / 'pwm.sock')
        sock = service.bind(path)
        server = threading.Thread(target=service.serve_forever, args=(sock,), daemon=True)
        server.start()
        self.addCleanup(sock.close)
        env = {'NETGUN_PWM_SOCKET': path, 'NETGUN_CONFIG_DIR': str(self.tmp / 'offline_cfg')}

        def cli(*args, stdin=None):
            out = io.StringIO()
            with mock.patch.dict(os.environ, env), contextlib.redirect_stdout(out), \
                    mock.patch.object(sys, 'stdin', io.StringIO(stdin or '')):
                code = pl.main(['x', 'params', *args])
            return code, json.loads(out.getvalue().strip().splitlines()[-1])

        code, shown = cli()
        self.assertEqual((code, shown['ok'], shown['service']), (0, True, True))
        self.assertEqual((shown['options']['low_us'], shown['options']['limits']['high_us']), (1000, [1750, 2000]))
        before = self.count()
        code, reply = cli('--high-us', '1900', '--hold-s', '0.8')
        self.assertEqual((code, reply['applied']), (0, True), reply)
        self.assertEqual((reply['options']['high_us'], reply['options']['hold_s'], reply['options']['source']), (1900, 0.8, 'file'))
        saved = json.loads((self.tmp / 'netgun_pwm.json').read_text(encoding='utf-8'))
        self.assertEqual((saved['high_us'], saved['hold_s'], saved['low_us'], saved['schema_version']), (1900, 0.8, 1000, 1))
        self.assertRegex(saved['saved_at'], r'^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d[+-]\d\d:\d\d$')
        self.assertEqual(self.count(), before, 'LOW unchanged: no re-latch')
        code, reply = cli('--low-us', '1100')
        self.assertEqual(code, 0, reply)
        self.assertEqual((reply['idle']['latched'], reply['idle']['low_us']), (True, 1100))
        relatch = self.commands(before)[-1]
        self.assertEqual(relatch.command, 187)
        self.assertAlmostEqual(relatch.param1, -0.8, places=5)
        code, reply = cli('--high-us', '1650')
        self.assertEqual(code, 1)
        self.assertIn('击发高位 1650 µs 超出范围 1750–2000 µs', reply['error'])
        self.assertEqual(service.params['high_us'], 1900)
        self.assertEqual(cli('--high-us')[0], 2)
        self.assertEqual(cli('--bogus', '1')[0], 2)
        self.fc.armed = True
        self.wait_armed()
        code, reply = cli('--hold-s', '1.5')
        self.assertEqual(code, 1)
        self.assertIn('飞控已解锁', reply['error'])
        self.fc.armed = False
        self.wait_armed(False)
        code, reply = cli('save', stdin=json.dumps({'config': {'low_us': 1000, 'high_us': 2000, 'hold_s': 1.2, 'low_s': 0.5}}))
        self.assertEqual(code, 0, reply)
        self.assertEqual(service.params, {'low_us': 1000, 'high_us': 2000, 'hold_s': 1.2, 'low_s': 0.5})
        self.assertEqual(service.idle()['low_us'], 1000)
        self.assertEqual(cli('save', stdin='{"low_us": 1000}')[0], 2)
        code, reply = cli('--reset')
        self.assertEqual((code, reply['options']['source'], reply['options']['hold_s']), (0, 'default', 1.0))
        self.assertFalse((self.tmp / 'netgun_pwm.json').exists())
        self.assertIn('参数', '；'.join(str(a[0]) for a in self.logs))
        service.stop.set()
        server.join(2)
        service.unlink()
        code, reply = cli('--hold-s', '0.5')
        self.assertEqual((code, reply['service'], reply['applied']), (0, False, False), reply)
        self.assertIn('服务启动后生效', reply['note'])
        code, shown = cli()
        self.assertEqual((code, shown['service'], shown['options']['hold_s'], shown['options']['source']), (0, False, 0.5, 'file'))

    def test_unix_socket_client_fire_and_status(self):
        service = self.start(armed=True)
        self.wait_ready()
        time.sleep(0.8)
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
            self.assertEqual(out.getvalue().strip(),
                             '实物击发链路：就绪（飞控 2/1 · 已解锁 · AUX5 待命 1000 µs → 击发 2000 µs 保持 0.3 s）')
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


def stub_service(tmp):
    service = pl.PwmService(StubFcLink(), events_path=str(tmp / 'e.jsonl'), params_file=str(tmp / 'netgun_pwm.json'),
                            log=lambda *a, **k: None)
    service.latch = {'low_us': 1000, 't': time.monotonic(), 'why': 'test'}  # 桩链路没有 maintain 线程
    return service


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
            service = stub_service(self.tmp)
            sock = service.bind(path)
            thread = threading.Thread(target=service.serve_forever, args=(sock,), daemon=True)
            thread.start()
            try:
                name, result, detail = self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire')
                self.assertEqual(result, 'PASS', detail)
                self.assertIn('FMU PWM OUT 5', name)
                self.assertIn('就绪（飞控 2/1 · 未解锁 · AUX5 待命 1000 µs → 击发 2000 µs 保持 1.0 s · 默认参数）', detail)
                self.assertEqual(self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire --bench')[1], 'WARN')
                service.latch = None
                result, detail = self.preflight.pwm_link_check('python3 x/netgun_pwm_link.py fire')[1:]
                self.assertEqual(result, 'FAIL')
                self.assertIn('待命低位未设好', detail)
                service.latch = {'low_us': 1000, 't': time.monotonic(), 'why': 'test'}
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
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix='pwm_client_'))
        self.addCleanup(shutil.rmtree, self.tmp, True)

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

    def test_default_paths_and_pulse_conversion(self):
        with mock.patch.dict(os.environ, {'NETGUN_LOG_DIR': '/x/logs', 'NETGUN_CONFIG_DIR': '/x/cfg'}):
            os.environ.pop('NETGUN_PWM_SOCKET', None)
            self.assertEqual(pl.socket_path(), '/x/logs/netgun_pwm.sock')
            self.assertEqual(pl.params_path(), '/x/cfg/netgun_pwm.json')
        self.assertEqual((pl.pwm_us(-1.0), pl.pwm_us(1.0), pl.pwm_us(0.0), pl.pwm_us(None)), (1000, 2000, 1500, None))
        self.assertEqual((pl.unit_value(1000), pl.unit_value(2000), pl.unit_value(1500)), (-1.0, 1.0, 0.0))
        for us in (1000, 1100, 1250, 1750, 1900, 2000):
            self.assertEqual(pl.pwm_us(pl.unit_value(us)), us)

    def test_params_validation_and_file_round_trip(self):
        good = pl.validate_params({'low_us': 1100.0, 'high_us': 1900, 'hold_s': 0.8, 'low_s': 0.25})
        self.assertEqual(good, {'low_us': 1100, 'high_us': 1900, 'hold_s': 0.8, 'low_s': 0.25})
        self.assertIsInstance(good['low_us'], int)
        bad = [({'low_us': 1300}, '待命低位 1300 µs 超出范围 1000–1250 µs'), ({'high_us': 1700}, '1750–2000'),
               ({'hold_s': 0.1}, '高位保持'), ({'low_s': 3}, '低位确认'), ({'low_us': 1000.5}, '需为整数'),
               ({'hold_s': True}, '需为数字'), ({'hold_s': float('nan')}, '需为数字'), ({'extra': 1}, '未知参数')]
        for change, text in bad:
            with self.assertRaises(pl.ParamsError) as caught:
                pl.validate_params(dict(pl.DEFAULT_PARAMS, **change))
            self.assertIn(text, str(caught.exception))
        with self.assertRaisesRegex(pl.ParamsError, '缺少击发高位'):
            pl.validate_params({'low_us': 1000, 'hold_s': 1, 'low_s': 0.3})
        path = str(self.tmp / 'cfg' / 'netgun_pwm.json')
        params, meta = pl.load_params(path)
        self.assertEqual((params, meta['source'], meta['error']), (pl.DEFAULT_PARAMS, 'default', None))
        meta = pl.save_params(good, path)
        self.assertEqual(meta['source'], 'file')
        self.assertEqual(pl.load_params(path)[0], good)
        self.assertEqual(os.listdir(os.path.dirname(path)), ['netgun_pwm.json'], 'no temporary file is left')
        with open(path, 'w', encoding='utf-8') as f:
            f.write('{"low_us": 1000')
        params, meta = pl.load_params(path)
        self.assertIsNone(params)
        self.assertIn('击发参数文件无效', meta['error'])
        info = pl.params_info(path)
        self.assertEqual((info['low_us'], info['limits']['low_us'], info['defaults']['hold_s']), (None, [1000, 1250], 1.0))
        pl.remove_params(path)
        pl.remove_params(path)
        self.assertEqual(pl.load_params(path)[1]['source'], 'default')

    def test_param_number_decodes_px4_int_bytewise(self):
        class Msg:
            param_type = 6

            def __init__(self, value):
                self.param_value = struct.unpack('<f', struct.pack('<i', value))[0]

            def get_msgbuf(self):
                raise RuntimeError('no buffer')
        for value in (0, 50, 301, 1000, 2000, -1):
            self.assertEqual(pl.param_number(Msg(value)), value)

    def test_result_line_survives_non_utf8_stdout(self):
        # 火控节点 stdout 若不是 UTF-8，结果行退回 ASCII 转义，不能因中文原因崩掉
        raw = io.BytesIO()
        out = io.TextIOWrapper(raw, encoding='ascii')
        with mock.patch.object(sys, 'stdout', out):
            pl.emit({'ok': False, 'reason_text': '飞控未解锁'}, 'NETGUN_PWM_RESULT')
        out.flush()
        line = raw.getvalue().decode('ascii')
        self.assertTrue(line.startswith('NETGUN_PWM_RESULT {') and line.endswith('\n'), line)
        self.assertEqual(json.loads(line.split(' ', 1)[1])['reason_text'], '飞控未解锁')

    def test_brief_texts(self):
        base = {'ok': True, 'link': {'alive': True, 'fc': '2/1', 'armed': False},
                'config': {'ok': True, 'problems': [], 'missing': [], 'warnings': []},
                'options': dict(pl.DEFAULT_PARAMS, source='file', error=None),
                'idle': {'latched': True, 'low_us': 1000}}
        self.assertEqual(pl.brief(base), '就绪（飞控 2/1 · 未解锁 · AUX5 待命 1000 µs → 击发 2000 µs 保持 1.0 s）')
        base['options']['hold_s'] = 0.75
        self.assertIn('保持 0.75 s', pl.brief(base))
        self.assertEqual(pl.brief(dict(base, idle={'latched': False, 'error': None})),
                         '待命低位未设好（等待飞控确认 1000 µs）')
        self.assertEqual(pl.brief(dict(base, options={'fire_value': -1, 'fire_us': 1000})),
                         '击发服务是旧版本：板上重跑 install_netgun_pwm_link.sh')
        self.assertEqual(pl.brief(dict(base, options={'high_us': None, 'error': '击发参数文件无效（x）：y'})),
                         '击发参数无效：击发参数文件无效（x）：y')


if __name__ == '__main__':
    unittest.main()
