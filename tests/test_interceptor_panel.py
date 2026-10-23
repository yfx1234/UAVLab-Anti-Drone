"""面板侧拦截链（2026-10-23）：缓存 SSH 连接执行（状态轮询/发射按钮）、发射任务不排队、
发射命令走常驻通道助手；前端显示发射通道/复盘记录并优先用 2Hz 火控状态。"""
import io
import json
import pathlib
import shutil
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'panel'))
import board_client  # noqa: E402
import dashboard  # noqa: E402
from job_manager import JobManager  # noqa: E402

HTML = (ROOT / 'panel' / 'index.html').read_text(encoding='utf-8')


class FakeChannel:
    def __init__(self, client):
        self.client = client
        self.closed = False

    def settimeout(self, value):
        self.timeout = value

    def exec_command(self, command):
        self.client.commands.append(command)
        if self.client.exec_error:
            raise self.client.exec_error

    def makefile(self, mode):
        if self.client.read_error:
            error = self.client.read_error

            class Broken:
                def read(self):
                    raise error
            return Broken()
        if self.client.gate:
            self.client.gate.wait(2)
        return io.BytesIO(self.client.output)

    def makefile_stderr(self, mode):
        return io.BytesIO(b'')

    def close(self):
        self.closed = True


class FakeClient:
    def __init__(self, output=b'{"ok": true}\n', open_error=None, exec_error=None, read_error=None, gate=None):
        self.output, self.open_error, self.exec_error, self.read_error, self.gate = output, open_error, exec_error, read_error, gate
        self.active, self.closed, self.commands, self.channels = True, False, [], []

    def get_transport(self):
        return self

    def is_active(self):
        return self.active and not self.closed

    def open_session(self, timeout=None):
        if self.open_error:
            raise self.open_error
        channel = FakeChannel(self)
        self.channels.append(channel)
        return channel

    def close(self):
        self.closed = True


class SharedExecTests(unittest.TestCase):
    def board(self, *clients):
        board = board_client.BoardClient()
        queue = list(clients)
        board.connection = lambda: queue.pop(0)
        return board

    def test_reuses_one_connection_and_closes_each_channel(self):
        client = FakeClient()
        board = self.board(client)
        self.assertEqual(board.shared_exec('status'), '{"ok": true}\n')
        self.assertEqual(board.shared_exec('fire'), '{"ok": true}\n')
        self.assertEqual(client.commands, ['status', 'fire'])
        self.assertTrue(all(channel.closed for channel in client.channels))
        self.assertFalse(client.closed)
        board.close_shared()
        self.assertTrue(client.closed)

    def test_dead_or_unusable_transport_is_replaced_once(self):
        stale, fresh = FakeClient(open_error=EOFError('gone')), FakeClient()
        board = self.board(stale, fresh)
        self.assertEqual(board.shared_exec('fire'), '{"ok": true}\n')
        self.assertTrue(stale.closed)
        self.assertEqual(fresh.commands, ['fire'])
        inactive, replacement = FakeClient(), FakeClient()
        board = self.board(inactive, replacement)
        board.shared_exec('status')
        inactive.active = False
        board.shared_exec('status')
        self.assertTrue(inactive.closed)
        self.assertEqual(replacement.commands, ['status'])

    def test_exec_request_failure_retries_once_then_raises(self):
        first, second = FakeClient(exec_error=EOFError('a')), FakeClient(exec_error=EOFError('b'))
        board = self.board(first, second)
        with self.assertRaises(EOFError):
            board.shared_exec('fire')
        self.assertTrue(first.closed and second.closed)
        self.assertEqual(first.commands + second.commands, ['fire', 'fire'])

    def test_failure_after_command_started_is_not_retried(self):
        client = FakeClient(read_error=socket.timeout('slow'))
        board = self.board(client, FakeClient())
        with self.assertRaises(socket.timeout):
            board.shared_exec('fire')
        self.assertEqual(client.commands, ['fire'], 'a command that may have run must not be re-sent')
        self.assertTrue(client.closed)
        self.assertIsNone(board.shared)

    def test_empty_output_keeps_the_connection(self):
        client = FakeClient(output=b'')
        board = self.board(client)
        with self.assertRaisesRegex(ValueError, '无输出'):
            board.shared_exec('status')
        self.assertIs(board.shared, client)
        self.assertFalse(client.closed)

    def test_close_waits_for_in_flight_command(self):
        gate = threading.Event()
        client = FakeClient(gate=gate)
        board = self.board(client)
        result = []
        worker = threading.Thread(target=lambda: result.append(board.shared_exec('fire')))
        worker.start()
        for _ in range(100):
            if client.commands:
                break
            time.sleep(0.01)
        board.close_shared()
        self.assertFalse(client.closed, 'in-flight fire must finish before the connection closes')
        gate.set()
        worker.join(2)
        self.assertEqual(result, ['{"ok": true}\n'])
        self.assertTrue(client.closed)


class FireJobTests(unittest.TestCase):
    def test_dedicated_job_runs_while_pool_is_busy(self):
        jobs = JobManager()
        gate = threading.Event()
        for index in range(5):
            jobs.submit('long%d' % index, lambda: gate.wait(5))
        jid = jobs.submit('fire', lambda: {'fired': True}, dedicated=True)
        try:
            for _ in range(100):
                if jobs.get(jid)['state'] == 'complete':
                    break
                time.sleep(0.01)
            self.assertEqual(jobs.get(jid).get('result'), {'fired': True})
        finally:
            gate.set()
            jobs.pool.shutdown()

    def test_fire_action_uses_channel_helper_on_cached_connection(self):
        self.assertEqual(dashboard.INTERCEPTOR_FIRE_CMD, 'python3 /home/uavlab/interceptor_ws/deploy/netgun_fire_button.py fire')
        calls = []

        class Board:
            def shared_exec(self, command, timeout=45):
                calls.append((command, timeout))
                return 'noise\n{"ok": true, "fired": true, "path": "server", "elapsed_ms": 61.0}\n'

            def exec_command(self, command, timeout=45):
                raise AssertionError('fire must use the cached connection')
        panel = dashboard.Panel.__new__(dashboard.Panel)
        panel.lock = threading.RLock()
        panel.connected = True
        panel.jobs = JobManager()
        panel.board = Board()
        panel.progress, panel.cancelled, panel.scene_jobs = {}, set(), {}
        with mock.patch.object(panel.jobs, 'submit', wraps=panel.jobs.submit) as submit:
            jid = panel.submit({'action': 'interceptor_fire', 'request_id': 'fire_test', 'params': {}})
        self.assertTrue(submit.call_args.kwargs.get('dedicated'))
        for _ in range(100):
            job = panel.jobs.get(jid)
            if job['state'] in ('complete', 'error'):
                break
            time.sleep(0.01)
        panel.jobs.pool.shutdown()
        self.assertEqual(job['state'], 'complete', job)
        self.assertTrue(job['result']['fired'])
        self.assertEqual(job['result']['path'], 'server')
        self.assertIsInstance(job['result']['panel_ms'], int)
        self.assertEqual(calls, [(dashboard.INTERCEPTOR_FIRE_CMD, 20)])


@unittest.skipUnless(shutil.which('node'), 'Node required')
class FrontendFireLinkTests(unittest.TestCase):
    def run_node(self, assertions):
        block = 'const interceptorStageNames=' + HTML.split('const interceptorStageNames=', 1)[1].split('async function action(', 1)[0]
        timing = 'function fireTiming(' + HTML.split('function fireTiming(', 1)[1].split("byId('confirmFire').onclick", 1)[0]
        harness = r"""
const assert=require('node:assert/strict'),elements={};
const byId=id=>elements[id]||(elements[id]={id,textContent:'',title:'',hidden:false,disabled:false,open:false,children:[],classList:{names:new Set(),toggle(n,on){if(on)this.names.add(n);else this.names.delete(n);},contains(n){return this.names.has(n);}}});
const number=(v,d=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—';
const disable=(id,condition)=>{byId(id).disabled=!!condition;return byId(id).disabled;};
const base=()=>({connected:true,error:null,interceptor_error:null,interceptor_age_s:1,state:{camera_state:'running',pipeline:{detector_state:2}},
 interceptor:{running:true,pid:5,shots:0,strike:{fire_mode:'auto_manual',fire_ready:0,manual_block:'NOT_ARMED',arm:0},estimator:{},guidance:{},tail:[],
  fire_link:{ok:true,link:{ready:true,phase:'就绪'},state:{fire_mode:'auto_manual',ready:true,latched:false,block:'NONE',shots:0}},
  review:{dir:'review_20261023_170000',recording:true,size_mb:12.3,topics:31,free_mb:40960}}});
let state=base();
""" + block + timing
        script = harness + "\n" + assertions
        result = subprocess.run(['node', '-e', script], capture_output=True, text=True, encoding='utf-8', timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_fire_link_review_and_live_fire_state(self):
        self.run_node(r"""
renderInterceptor(true);
assert.equal(byId('chkFireLink').textContent,'就绪 · 直达火控');assert.ok(byId('chkFireLink').classList.contains('good'));
assert.equal(byId('chkReview').textContent,'录制中 · 12 MB');assert.ok(byId('chkReview').classList.contains('good'));
assert.match(byId('chkReview').title,/review_20261023_170000；31 个话题；剩余 40\.0 GB/);
assert.equal(byId('netgunFireState').textContent,'自动待命 · 手动可发射','2 Hz channel state wins over the 5 s diag line');
assert.equal(byId('netgunFire').disabled,false);assert.ok(byId('netgunFire').classList.contains('danger-fill'));
state=base();state.interceptor.fire_link={ok:false,error:'常驻发射通道未运行'};state.interceptor.review={dir:'review_x',recording:false,size_mb:1,note:'WARN：磁盘剩余 100 MB'};
renderInterceptor(true);
assert.equal(byId('chkFireLink').textContent,'未运行 · 按键直连');assert.ok(byId('chkFireLink').classList.contains('warn'));
assert.match(byId('chkFireLink').title,/约 2–3 s/);
assert.equal(byId('netgunFireState').textContent,'手动未就绪：未解锁','without the channel the diag line is used');
assert.equal(byId('chkReview').textContent,'未录制');assert.match(byId('chkReview').title,/磁盘剩余 100 MB.*review_x/);
state=base();state.interceptor.fire_link.link={ready:false,phase:'等待匹配'};state.interceptor.fire_link.state={fire_mode:'auto_manual',ready:false,latched:true,block:'LATCHED',shots:1};
renderInterceptor(true);
assert.equal(byId('chkFireLink').textContent,'等待匹配');assert.equal(byId('netgunFireState').textContent,'已发射 1 发');assert.equal(byId('netgunFire').disabled,true);
state=base();delete state.interceptor.fire_link;delete state.interceptor.review;renderInterceptor(true);
assert.equal(byId('chkFireLink').textContent,'—');assert.equal(byId('chkReview').textContent,'未录制');
state=base();state.interceptor.running=false;renderInterceptor(false);
assert.equal(byId('chkFireLink').textContent,'—');assert.equal(byId('chkReview').textContent,'—');assert.ok(byId('chkReview').classList.contains('idle'));
assert.equal(fireTiming({path:'server'},213),' · 0.21 s（常驻通道）');assert.equal(fireTiming({path:'direct'},2480),' · 2.48 s（直连）');assert.equal(fireTiming({},50),' · 0.05 s');
""")

    def test_fire_job_polls_fast_and_rows_exist(self):
        self.assertIn("const pollMs=actionName==='interceptor_fire'?100:400;", HTML)
        for row in ('<dt>发射通道</dt><dd id="chkFireLink"', '<dt>复盘记录</dt><dd id="chkReview"'):
            self.assertIn(row, HTML)
        self.assertIn('fireDialog', HTML, 'the confirmation dialog stays')


if __name__ == '__main__':
    unittest.main()
