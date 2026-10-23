"""常驻发射通道（deploy/netgun_fire_button.py，2026-10-23）：不依赖 ROS 的门控、确认、
套接字协议与客户端回退语义。ROS 侧（RosLink）由板上 fullchain --fire manual 覆盖。"""
import contextlib
import importlib.util
import io
import json
import os
import pathlib
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
TOOL = ROOT / 'deploy' / 'netgun_fire_button.py'
spec = importlib.util.spec_from_file_location('netgun_fire_button_under_test', TOOL)
fb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fb)

READY = 'fmode=auto_manual fready=1 mblk=NONE auth=0 est=1 range=6.20 tilt=3.0 shots=0'
NOT_ARMED = 'fmode=auto_manual fready=0 mblk=NOT_ARMED auth=0 est=1 range=6.20 tilt=3.0 shots=0'


class FakeLink:
    def __init__(self, channel=None, matched=True, on_publish=None, fail=None):
        self.channel, self.matched, self.on_publish, self.fail = channel, matched, on_publish, fail
        self.published = 0

    def status(self):
        return {'matched': self.matched, 'phase': '就绪' if self.matched else '等待匹配', 'shot_event': True}

    def publish_fire(self):
        if self.fail:
            raise RuntimeError(self.fail)
        self.published += 1
        if self.on_publish:
            self.on_publish(self.published)


def channel_with(link_kwargs=None, state=READY, clock=None):
    logs = []
    channel = fb.FireChannel(clock=clock or time.monotonic, log=lambda text, level='INFO': logs.append((level, text)))
    link = FakeLink(channel, **(link_kwargs or {}))
    channel.link = link
    if state is not None:
        channel.on_state(state)
    return channel, link, logs


def shot_later(channel, delay=0.05):
    return lambda n: threading.Timer(delay, channel.on_shot, args=(n,)).start()


class FireChannelTests(unittest.TestCase):
    def test_no_or_stale_state_falls_back_without_publishing(self):
        channel, link, _ = channel_with(state=None)
        reply = channel.handle('fire')
        self.assertTrue(reply['fallback'])
        self.assertIn('尚未收到', reply['reason'])
        now = [100.0]
        channel, link, _ = channel_with(clock=lambda: now[0])
        now[0] += fb.STATE_FRESH_S + 0.1
        reply = channel.handle('fire')
        self.assertTrue(reply['fallback'])
        self.assertIn('未更新', reply['reason'])
        self.assertTrue(channel.handle('status')['fallback'])
        self.assertEqual(link.published, 0)

    def test_ready_and_matched_publishes_once_and_confirms_by_shot_event(self):
        channel, link, logs = channel_with()
        link.on_publish = shot_later(channel)
        reply = channel.handle('fire')
        self.assertEqual(link.published, 1)
        self.assertTrue(reply['ok'])
        self.assertTrue(reply['fired'])
        self.assertEqual(reply['path'], 'server')
        self.assertEqual(reply['confirmed_by'], 'shot_event')
        self.assertEqual(reply['shot_id'], 1)
        self.assertTrue(reply['latched'])
        self.assertEqual(reply['shots'], 1)
        self.assertLess(reply['confirm_ms'], 1000)
        self.assertIn('publish_ms', reply)
        self.assertTrue(any('已发布' in text for _, text in logs))
        for _, text in logs:
            self.assertNotIn('NETGUN FIRE', text, 'status script counts shots by this keyword')
            self.assertNotIn('netgun diag:', text)

    def test_confirmation_by_state_shots_increase(self):
        channel, link, _ = channel_with()
        link.on_publish = lambda n: threading.Timer(0.05, channel.on_state, args=(READY.replace('shots=0', 'shots=1'),)).start()
        reply = channel.handle('fire')
        self.assertTrue(reply['fired'])
        self.assertEqual(reply['confirmed_by'], 'strike_state')

    def test_gate_rejections_never_publish(self):
        cases = [
            (NOT_ARMED, '未解锁'),
            (READY.replace('shots=0', 'shots=1').replace('mblk=NONE', 'mblk=LATCHED').replace('fready=1', 'fready=0'), '已发射过'),
            (READY.replace('fmode=auto_manual', 'fmode=auto'), 'fire_mode=auto'),
        ]
        for text, expected in cases:
            with self.subTest(expected=expected):
                channel, link, logs = channel_with(state=text)
                reply = channel.handle('fire')
                self.assertFalse(reply['ok'])
                self.assertIn(expected, reply['reason'])
                self.assertEqual(reply['path'], 'server')
                self.assertNotIn('fallback', reply)
                self.assertEqual(link.published, 0)
                self.assertTrue(any('被拦下' in text for _, text in logs))

    def test_unmatched_link_or_publish_failure_falls_back(self):
        channel, link, _ = channel_with({'matched': False})
        reply = channel.handle('fire')
        self.assertTrue(reply['fallback'])
        self.assertIn('尚未与火控匹配', reply['reason'])
        self.assertEqual(link.published, 0)
        channel, link, _ = channel_with({'fail': 'boom'})
        reply = channel.handle('fire')
        self.assertTrue(reply['fallback'])
        self.assertIn('boom', reply['reason'])
        self.assertIsNone(channel.pending)

    def test_unconfirmed_publish_reports_without_fallback(self):
        channel, link, logs = channel_with()
        with mock.patch.object(fb, 'CONFIRM_SECONDS', 0.2):
            reply = channel.handle('fire')
        self.assertEqual(link.published, 1)
        self.assertFalse(reply['ok'])
        self.assertFalse(reply['fired'])
        self.assertNotIn('fallback', reply, 'a published command must never be retried by the client')
        self.assertIn('未确认', reply['reason'])
        self.assertEqual(logs[-1][0], 'WARN')

    def test_concurrent_presses_publish_once(self):
        channel, link, _ = channel_with()
        link.on_publish = shot_later(channel, 0.1)
        replies = []
        threads = [threading.Thread(target=lambda: replies.append(channel.handle('fire'))) for _ in range(3)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(5)
        self.assertEqual(link.published, 1, replies)
        self.assertEqual(sum(1 for r in replies if r.get('fired')), 1)
        for reply in replies:
            if not reply.get('fired'):
                self.assertIn('已发射过', reply['reason'])
        # 确认后、下一条 2Hz 状态前：ping/status 已报已发射
        self.assertTrue(channel.handle('status')['latched'])
        self.assertTrue(channel.handle('ping')['state']['latched'])

    def test_ping_reports_link_and_fresh_state(self):
        channel, link, _ = channel_with()
        reply = channel.handle('ping')
        self.assertTrue(reply['ok'])
        self.assertTrue(reply['link']['ready'])
        self.assertTrue(reply['link']['state_fresh'])
        self.assertTrue(reply['state']['ready'])
        self.assertEqual(reply['state']['reason'], '自动开火待命；手动兜底就绪，可发射')
        waiting = fb.FireChannel(log=lambda *a, **k: None)
        reply = waiting.handle('ping')
        self.assertFalse(reply['link']['ready'])
        self.assertEqual(reply['link']['phase'], '等待火控节点')
        self.assertNotIn('state', reply)


@unittest.skipUnless(hasattr(socket, 'AF_UNIX'), 'Unix sockets required')
class SocketProtocolTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix='fire_')
        self.path = os.path.join(self.dir, 'fire.sock')

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)

    def serve(self, channel):
        sock = channel.bind(self.path)
        thread = threading.Thread(target=channel.serve_forever, args=(sock,), daemon=True)
        thread.start()

        def stop():
            channel.stop.set()
            thread.join(2)
            sock.close()
            channel.unlink()
        self.addCleanup(stop)
        return sock

    def raw(self, payload):
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.settimeout(2)
        client.connect(self.path)
        client.sendall(payload)
        data = client.makefile('rb').readline()
        client.close()
        return json.loads(data)

    def test_request_roundtrip_permissions_and_unknown_command(self):
        channel, link, _ = channel_with()
        link.on_publish = shot_later(channel)
        self.serve(channel)
        self.assertEqual(os.stat(self.path).st_mode & 0o777, 0o600)
        self.assertTrue(fb.channel_request('ping', self.path)['link']['ready'])
        self.assertEqual(fb.channel_request('status', self.path)['path'], 'server')
        started = time.monotonic()
        reply = fb.channel_request('fire', self.path)
        self.assertTrue(reply['fired'])
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertEqual(self.raw(b'{"cmd": "launch"}\n'), {'ack': False, 'reason': '未知请求'})
        self.assertEqual(link.published, 1)

    def test_missing_stale_and_live_socket_handling(self):
        with self.assertRaises(fb.ChannelError) as missing:
            fb.channel_request('fire', self.path)
        self.assertIn('未运行', str(missing.exception))
        stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        stale.bind(self.path)
        stale.close()  # 上次异常退出留下的套接字文件
        with self.assertRaises(fb.ChannelError) as refused:
            fb.channel_request('status', self.path)
        self.assertIn('残留', str(refused.exception))
        channel, _, _ = channel_with()
        self.serve(channel)
        with self.assertRaises(RuntimeError):
            fb.FireChannel(log=lambda *a, **k: None).bind(self.path)
        self.assertTrue(fb.channel_request('ping', self.path)['ok'])

    def test_stale_state_reply_falls_back(self):
        now = [10.0]
        channel, link, _ = channel_with(clock=lambda: now[0])
        self.serve(channel)
        now[0] += 5
        with self.assertRaises(fb.ChannelError) as stale:
            fb.channel_request('fire', self.path)
        self.assertIn('未更新', str(stale.exception))
        self.assertEqual(link.published, 0)

    def silent_server(self, ack):
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(self.path)
        server.listen(1)
        held = []

        def accept():
            conn, _ = server.accept()
            held.append(conn)
            conn.makefile('rb').readline()
            if ack:
                conn.sendall(b'{"ack": true}\n')
        threading.Thread(target=accept, daemon=True).start()
        self.addCleanup(lambda: [c.close() for c in held] + [server.close()])

    def test_no_ack_is_a_safe_fallback(self):
        self.silent_server(ack=False)
        with mock.patch.object(fb, 'ACK_TIMEOUT_S', 0.2):
            with self.assertRaises(fb.ChannelError) as silent:
                fb.channel_request('fire', self.path)
        self.assertIn('无应答', str(silent.exception))

    def test_accepted_fire_without_reply_is_not_retried(self):
        self.silent_server(ack=True)
        with mock.patch.object(fb, 'REPLY_MARGIN_S', 0.1):
            reply = fb.channel_request('fire', self.path, confirm_s=0.1)
        self.assertFalse(reply['ok'])
        self.assertEqual(reply['path'], 'server')
        self.assertIn('以火控状态为准', reply['reason'])


class ClientMainTests(unittest.TestCase):
    def run_main(self, argv, env):
        out = io.StringIO()
        with mock.patch.dict(os.environ, env), contextlib.redirect_stdout(out):
            code = fb.main(argv)
        return code, json.loads(out.getvalue().strip().splitlines()[-1])

    def test_production_domain_without_stack_answers_fast(self):
        with tempfile.TemporaryDirectory() as folder, \
                mock.patch.object(fb, 'stack_running', return_value=False), \
                mock.patch.object(fb, 'reexec_with_ros') as reexec:
            started = time.monotonic()
            code, reply = self.run_main(['x', 'fire'], {'ROS_DOMAIN_ID': '0', 'NETGUN_FIRE_SOCKET': os.path.join(folder, 'none.sock'),
                                                        'NETGUN_FIRE_REEXEC': ''})
        self.assertEqual(code, 1)
        self.assertEqual(reply['reason'], '拦截链未运行')
        self.assertEqual(reply['path'], 'none')
        self.assertIn('未运行', reply['fallback_reason'])
        self.assertLess(time.monotonic() - started, 2.0)
        reexec.assert_not_called()

    @unittest.skipUnless(hasattr(socket, 'AF_UNIX'), 'Unix sockets required')
    def test_running_channel_is_used_first(self):
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, 'fire.sock')
            channel, link, _ = channel_with(state=NOT_ARMED)
            sock = channel.bind(path)
            thread = threading.Thread(target=channel.serve_forever, args=(sock,), daemon=True)
            thread.start()
            try:
                code, reply = self.run_main(['x', 'fire'], {'ROS_DOMAIN_ID': '0', 'NETGUN_FIRE_SOCKET': path, 'NETGUN_FIRE_REEXEC': ''})
            finally:
                channel.stop.set()
                thread.join(2)
                sock.close()
        self.assertEqual(code, 1)
        self.assertEqual(reply['path'], 'server')
        self.assertIn('未解锁', reply['reason'])
        self.assertIn('elapsed_ms', reply)
        self.assertEqual(link.published, 0)


@unittest.skipIf(os.name == 'nt' or not shutil.which('pgrep'), 'POSIX process tools required')
class ServeProcessTests(unittest.TestCase):
    def test_server_waits_for_strike_and_exits_with_parent(self):
        folder = tempfile.mkdtemp(prefix='fire_serve_')
        self.addCleanup(shutil.rmtree, folder, True)
        bin_dir = pathlib.Path(folder) / 'bin'
        bin_dir.mkdir()
        # pgrep 桩：看不到任何火控节点（也绝不碰本机真实进程），通道停在等待阶段、不加载 ROS
        (bin_dir / 'pgrep').write_text('#!/bin/sh\nexit 1\n', encoding='utf-8')
        (bin_dir / 'pgrep').chmod(0o755)
        path = os.path.join(folder, 'fire.sock')
        parent = subprocess.Popen(['sleep', '30'])
        self.addCleanup(lambda: parent.poll() is None and parent.kill())
        env = dict(os.environ, NETGUN_FIRE_SOCKET=path, ROS_DOMAIN_ID='199',
                   PATH=str(bin_dir) + os.pathsep + os.environ.get('PATH', ''))
        server = subprocess.Popen([sys.executable, '-u', str(TOOL), 'serve', '--parent-pid', str(parent.pid)],
                                  env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.addCleanup(lambda: server.poll() is None and server.kill())
        for _ in range(50):
            if os.path.exists(path):
                break
            time.sleep(0.1)
        reply = fb.channel_request('ping', path)
        self.assertFalse(reply['link']['ready'])
        self.assertEqual(reply['link']['phase'], '等待火控节点')
        with self.assertRaises(fb.ChannelError):
            fb.channel_request('fire', path)
        parent.kill()
        parent.wait()
        self.assertEqual(server.wait(timeout=5), 0)
        output = server.stdout.read()
        server.stdout.close()
        self.assertIn('[netgun_fire_server]: 常驻发射通道启动', output)
        self.assertIn('随之停止', output)
        self.assertFalse(os.path.exists(path), 'socket removed on exit')

    def test_sigterm_stops_and_unlinks(self):
        folder = tempfile.mkdtemp(prefix='fire_serve_')
        self.addCleanup(shutil.rmtree, folder, True)
        bin_dir = pathlib.Path(folder) / 'bin'
        bin_dir.mkdir()
        (bin_dir / 'pgrep').write_text('#!/bin/sh\nexit 1\n', encoding='utf-8')
        (bin_dir / 'pgrep').chmod(0o755)
        path = os.path.join(folder, 'fire.sock')
        env = dict(os.environ, NETGUN_FIRE_SOCKET=path, ROS_DOMAIN_ID='199',
                   PATH=str(bin_dir) + os.pathsep + os.environ.get('PATH', ''))
        server = subprocess.Popen([sys.executable, '-u', str(TOOL), 'serve'], env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.addCleanup(lambda: server.poll() is None and server.kill())
        for _ in range(50):
            if os.path.exists(path):
                break
            time.sleep(0.1)
        self.assertTrue(fb.channel_request('ping', path)['ok'])
        server.send_signal(signal.SIGTERM)
        self.assertEqual(server.wait(timeout=5), 0)
        self.assertFalse(os.path.exists(path))


if __name__ == '__main__':
    unittest.main()
