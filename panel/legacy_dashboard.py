"""Loopback-only board diagnostics. No remote source writes or flight commands."""
import base64
import hashlib
import hmac
import http.server
import json
import logging
import os
import pathlib
import re
import secrets
import socket
import subprocess
import sys
import threading
import time
import urllib.request
import warnings

with warnings.catch_warnings():
    warnings.simplefilter('ignore')
    import paramiko

logging.getLogger('paramiko').setLevel(logging.ERROR)
BASE = pathlib.Path(__file__).resolve().parents[2]
HOST = '192.168.50.100'
LOCAL_IP = '192.168.50.101'
HOST_KEY = 'CHnInkACqORSkPoeTv8qWJtjHUOjRIPHPvVIdPHLdeI'
RELAY = 'http://127.0.0.1:8766'
COMMANDS = {
    'link': ('连接检查', 'hostname; uptime; ip -br addr'),
    'system': ('系统概况', "uname -a; cat /etc/nv_tegra_release; free -m; df -h /"),
    'camera': ('相机与服务', 'systemctl is-active zed_x_daemon IMU_Daemon nvargus-daemon; ls -l /dev/video*; ps -eo pid,comm,args --width 240 | grep -Ei "zed|argus|camera|deepstream" | grep -v grep'),
    'network': ('网络检查', 'ip -br addr; ip route; ss -lnt | head -25'),
    'ros': ('ROS 图像话题', "bash -lc 'source /opt/ros/humble/setup.bash; printf \"ROS_DOMAIN_ID=\"; printenv ROS_DOMAIN_ID; timeout 8 ros2 topic list --no-daemon'"),
    'logs': ('相机服务日志', 'journalctl -u nvargus-daemon -u zed_x_daemon -u IMU_Daemon -n 45 --no-pager'),
    'boot': ('启动顺序', 'efibootmgr -v'),
}

# ZED 跟踪链路的固定控制动作(板上白名单脚本,不含自由命令)。
CHAIN_SCRIPT = ('bash /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/zed_tracking/'
                'tools/field_flight_zed.sh')
WATCHER = ('/home/uavlab/Jetson_ROS2_V0/ros2_ws/src/zed_tracking/tools/'
           'flight_status_watcher.py')
BOARD_STATUS = ('bash /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/zed_tracking/'
                'tools/board_flight_status.sh')
CHAIN_ACTIONS = {
    'chain_check': ('链路检查(只读)', f'{CHAIN_SCRIPT} check', False),
    'chain_start': ('启动链路(影子)', f'FALCON_CHECK_SKIP_FC=1 FALCON_ADAPTER_MODE=shadow setsid nohup {CHAIN_SCRIPT} start > /tmp/panel_chain.log 2>&1 < /dev/null & echo 已后台启动(影子,允许无飞控),用链路日志查看进度', False),
    'chain_start_live': ('启动链路(实控)', f'FALCON_ADAPTER_MODE=live setsid nohup {CHAIN_SCRIPT} start > /tmp/panel_chain.log 2>&1 < /dev/null & echo 已后台启动(实控模式)', False),
    'chain_stop': ('停止链路', f'setsid nohup {CHAIN_SCRIPT} stop > /tmp/panel_chain.log 2>&1 < /dev/null & echo 已后台执行停止', False),
    'chain_log': ('链路日志', 'tail -n 60 /tmp/panel_chain.log 2>/dev/null || echo 尚无链路日志', False),
    'watcher_start': ('启动状态监视', 'pgrep -f flight_status_watcher >/dev/null && echo 监视器已在运行 || (setsid nohup bash -c "source /opt/ros/humble/setup.bash && source /home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash && exec python3 ' + WATCHER + '" > /tmp/panel_watcher.log 2>&1 < /dev/null & echo 监视器已启动)', False),
    'watcher_stop': ('停止状态监视', 'pkill -f "[f]light_status_watcher"; echo 已请求停止', False),
}

def command_for(action):
    if not isinstance(action, str) or action not in COMMANDS:
        raise ValueError('不支持的诊断动作')
    return COMMANDS[action][1]

def preview_blockers(process_rows, argus_client_pids):
    blockers = []
    for pid, name, args in process_rows:
        if name in ('nvargus-daemon', 'zed_preview', 'ZEDX_Daemon', 'ZED_X_Daemon', 'IMU_Daemon'):
            continue
        known = re.search(r'zed_perception|zed_wrapper|zed_camera|ZED_Explorer|ZED_Depth_Viewer|deepstream|argus_camera_node|usb_mjpeg_camera_node|gst-launch', args, re.I)
        if pid in argus_client_pids or known:
            blockers.append(pid)
    return blockers

def valid_request(host, origin, supplied_token, expected_token):
    return (host in ('127.0.0.1:8767', 'localhost:8767') and
            origin == 'http://' + host and isinstance(supplied_token,str) and
            hmac.compare_digest(supplied_token, expected_token))

def argus_clients(raw):
    clients=set();socket_ids=set()
    for line in raw.splitlines():
        if '/tmp/argus_socket' in line:
            fields=line.split()
            if len(fields)>7:socket_ids.update((fields[5],fields[7]))
            clients.update(int(x) for x in re.findall(r'pid=(\d+)',line))
    socket_ids.difference_update({'0','*'})
    for line in raw.splitlines():
        fields=line.split()
        if len(fields)>7 and (fields[5] in socket_ids or fields[7] in socket_ids):
            clients.update(int(x) for x in re.findall(r'pid=(\d+)',line))
    return clients

class PinnedHost(paramiko.MissingHostKeyPolicy):
    def missing_host_key(self, client, hostname, key):
        actual = base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
        if not hmac.compare_digest(actual, HOST_KEY):
            raise paramiko.SSHException('SSH 主机身份与已核实的板子不一致，请人工核对')

class Panel:
    def __init__(self):
        self.lock = threading.RLock()
        self.client = None
        self.password = os.environ.pop('UAVLAB_SSH_PASSWORD', None)
        self.token = secrets.token_urlsafe(32)
        self.events = []
        self.preview = {'available':False,'running':False}
        self.preview_owned = False
        self.last_poll = 0
        self.last_check = None
        self.flight = {'available':False}
        self.flight_poll = 0
        threading.Thread(target=self._flight_loop,daemon=True).start()

    def _flight_loop(self):
        # 连接期间每 3 秒刷新一次板上飞行状态(单次只读命令)。
        # 必须持面板锁:paramiko 会话非线程安全,与用户动作并发使用
        # 会在链路半死时互相卡死(2026-10-09 实测);失败后退避 30 秒。
        failures = 0
        while True:
            time.sleep(3 if failures < 2 else 30)
            if time.monotonic()-self.flight_poll < 2.5:continue
            if not self.connected():continue
            self.flight_poll=time.monotonic()
            try:
                with self.lock:
                    raw,_,code=self.run(BOARD_STATUS,timeout=14)
                data=json.loads(raw[raw.find('{'):].strip() or '{}')
                if code==0 and isinstance(data,dict):
                    self.flight={'available':True,**data}
                    failures=0
            except Exception:
                self.flight={'available':False}
                failures+=1

    def event(self, title, text, ok=True):
        self.events.insert(0, {'time':time.strftime('%H:%M:%S'), 'title':title, 'text':text,'ok':ok})
        self.events = self.events[:25]

    def connected(self):
        transport = self.client.get_transport() if self.client else None
        return bool(transport and transport.is_active() and transport.is_authenticated())

    def connect(self, password=None):
        if self.connected():return
        if password is not None:
            if not isinstance(password,str) or len(password)>256:raise ValueError('密码格式无效')
            self.password = password
        if self.password is None:raise ValueError('请输入板子的 SSH 密码；空格也是密码字符')
        if self.client:self.client.close()
        self.client = None
        sock = socket.socket();sock.settimeout(10)
        client = paramiko.SSHClient();client.set_missing_host_key_policy(PinnedHost())
        try:
            sock.bind((LOCAL_IP,0));sock.connect((HOST,22))
            client.connect(HOST,sock=sock,username='uavlab',password=self.password,
                           look_for_keys=False,allow_agent=False,timeout=10,banner_timeout=10,auth_timeout=10)
            client.get_transport().set_keepalive(15)
            self.client = client
        except Exception:
            client.close();sock.close();raise
        self.event('SSH 已连接', f'{LOCAL_IP} → uavlab@{HOST}；已核对主机身份')

    def run(self, command, sudo=False, timeout=16):
        if not self.connected():raise ConnectionError('请先连接板子')
        stdin,out,err = self.client.exec_command(command,timeout=timeout)
        if sudo:
            stdin.write(self.password+'\n');stdin.flush()
        stdin.channel.shutdown_write()
        channel=out.channel; stdout=bytearray();stderr=bytearray();deadline=time.monotonic()+timeout+2
        try:
            while True:
                if channel.recv_ready():stdout.extend(channel.recv(8192))
                if channel.recv_stderr_ready():stderr.extend(channel.recv_stderr(8192))
                if len(stdout)+len(stderr)>160000:raise RuntimeError('诊断输出超过限制')
                if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():break
                if time.monotonic()>deadline:raise TimeoutError('诊断超时，已关闭本次查询通道')
                time.sleep(.015)
            return stdout.decode(errors='replace'),stderr.decode(errors='replace'),channel.recv_exit_status()
        finally:channel.close()

    def inspect_owners(self):
        raw,error,code=self.run('ps -eo pid,comm,args --width 240')
        if code:raise RuntimeError('未能读取相机进程状态')
        rows=[]
        for line in raw.splitlines()[1:]:
            parts=line.strip().split(None,2)
            if len(parts)==3 and parts[0].isdigit():rows.append((int(parts[0]),parts[1],parts[2]))
        raw,error,code=self.run("sudo -S -p '' sh -c \"ss -xapH | sed -E 's/[[:space:]]+$//'\"",sudo=True)
        if code:raise RuntimeError('相机占用检查需要 sudo 读取 socket；当前检查未完成，拒绝启动新采集')
        clients=argus_clients(raw)
        owners,error,code=self.run("sudo -S -p '' sh -c 'fuser /dev/video0 /dev/video1 2>/dev/null; test $? -le 1'",sudo=True)
        if code:raise RuntimeError('无法核实视频设备占用，未启动新采集')
        clients.update(int(x) for x in re.findall(r'\b\d+\b',owners))
        ids=preview_blockers(rows,clients)
        return [dict(pid=pid,name=name,args=args) for pid,name,args in rows if pid in ids]

    def relay_request(self, endpoint, post=False):
        request=urllib.request.Request(RELAY+endpoint,data=b'' if post else None,
                                      headers={'Origin':RELAY},method='POST' if post else 'GET')
        with urllib.request.urlopen(request,timeout=3) as response:return response.read(2000000)

    def poll_preview(self):
        if time.monotonic()-self.last_poll<(10 if self.preview.get('error') else 1):return
        self.last_poll=time.monotonic()
        try:self.preview=json.loads(self.relay_request('/status'));self.preview['available']=True
        except Exception:self.preview={'available':False,'running':False,'error':'本机预览服务未启动'}

    def ensure_relay(self):
        self.poll_preview()
        if self.preview.get('available'):return
        startup=subprocess.STARTUPINFO();startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW
        logs=BASE/'work'
        with (logs/'zed_live_stdout.log').open('ab') as out,(logs/'zed_live_stderr.log').open('ab') as err:
            subprocess.Popen([sys.executable,'-u',str(BASE/'work/zed_live_preview.py')],cwd=str(BASE),
                             stdout=out,stderr=err,startupinfo=startup,creationflags=subprocess.CREATE_NO_WINDOW)
        for _ in range(20):
            try:self.relay_request('/status');return
            except Exception:time.sleep(.1)
        raise RuntimeError('本机预览服务未能启动')

    def dispatch(self, action, password=None):
        with self.lock:
            if action=='connect':self.connect(password)
            elif action=='disconnect':
                if self.client:self.client.close()
                self.client=None;self.password=None
                self.event('SSH 已断开','已清除面板会话凭据；相机预览可单独停止')
            elif action=='preview_start':
                if not self.connected():raise ConnectionError('请先连接板子')
                blockers=self.inspect_owners()
                if blockers:raise RuntimeError('相机可能被其他程序占用，未启动采集：'+', '.join(f"{x['name']} (PID {x['pid']})" for x in blockers))
                self.ensure_relay()
                state=json.loads(self.relay_request('/status'))
                if not state.get('running'):
                    self.relay_request('/start',True);self.preview_owned=True
                    self.event('独立预览启动','复用已部署的低带宽采集程序；不会启动飞行程序')
                else:self.event('复用已有预览','已在运行，无需重新打开相机')
                self.last_poll=0
            elif action=='preview_stop':
                self.relay_request('/stop',True);self.preview_owned=False;self.last_poll=0
                self.event('独立预览已停止','仅停止 zed_preview 的数据连接；请等待相机释放后再运行其他程序')
            elif action in CHAIN_ACTIONS:
                title,command,_live = CHAIN_ACTIONS[action]
                out,err,code=self.run(command,timeout=20)
                self.last_check=time.strftime('%H:%M:%S')
                self.event(title,(out+('\n'+err if err else '')).strip() or '(无输出)',code==0)
            elif action=='flight_status':
                out,err,code=self.run(BOARD_STATUS,timeout=14)
                if code:raise RuntimeError('飞行状态读取失败')
                data=json.loads(out[out.find('{'):].strip() or '{}')
                self.flight={'available':True,**data}
                self.event('飞行状态','链路进程与飞控状态已刷新（卡片数据为自动轮询）')
            else:
                command=command_for(action)
                out,err,code=self.run(command)
                self.last_check=time.strftime('%H:%M:%S')
                self.event(COMMANDS[action][0],(out+('\n'+err if err else '')).strip() or '(无输出)',code==0)
            return self.snapshot()

    def snapshot(self):
        # Reading local telemetry does not issue additional remote diagnostic commands.
        with self.lock:
            self.poll_preview()
            return {'connected':self.connected(),'credential_ready':self.password is not None,
                    'host':HOST,'local_ip':LOCAL_IP,'last_check':self.last_check,
                    'preview':self.preview,'events':list(self.events)}

def make_handler(panel):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self, body, kind='application/json; charset=utf-8', code=200):
            self.send_response(code);self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Referrer-Policy','no-referrer');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def js(self, body, code=200):self.send(json.dumps(body,ensure_ascii=False).encode(),code=code)
        def do_GET(self):
            if self.headers.get('Host') not in ('127.0.0.1:8767','localhost:8767'):
                self.js({'error':'仅接受本机访问'},403);return
            path=self.path.split('?',1)[0]
            if path=='/':
                page=(pathlib.Path(__file__).parent/'index.html').read_text(encoding='utf-8')
                self.send(page.replace('__TOKEN__',panel.token).encode(),'text/html; charset=utf-8')
            elif path=='/api/status':self.js(panel.snapshot())
            elif path=='/api/flight':
                with panel.lock:
                    self.js(panel.flight)
            elif path=='/api/frame.jpg':
                with panel.lock:
                    panel.poll_preview();state=panel.preview
                age=state.get('frame_age_s')
                if not state.get('running') or age is None or age>3:
                    self.js({'error':'当前无新鲜图像'},503);return
                try:self.send(panel.relay_request('/snapshot.jpg'),'image/jpeg')
                except Exception:self.js({'error':'图像读取失败'},503)
            elif path=='/api/export':self.js(panel.snapshot())
            else:self.js({'error':'未找到'},404)
        def do_POST(self):
            if not valid_request(self.headers.get('Host'),self.headers.get('Origin'),self.headers.get('X-Panel-Token'),panel.token):
                self.js({'error':'请求来源或会话校验失败'},403);return
            try:
                if self.path!='/api/action':self.js({'error':'未找到'},404);return
                if self.headers.get('Content-Type')!='application/json':raise ValueError('需要 JSON 请求')
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<4096:raise ValueError('请求长度无效')
                data=json.loads(self.rfile.read(size))
                if not isinstance(data,dict) or set(data)-{'action','password'}:raise ValueError('请求字段无效')
                if 'password' in data and data.get('action')!='connect':raise ValueError('此动作不接受密码')
                result=panel.dispatch(data.get('action'),data.get('password'));self.js(result)
            except Exception as error:
                # Do not log request bodies or passwords.
                message=str(error)
                with panel.lock:panel.event('操作未完成',message,False)
                self.js({'error':message},400)
    return Handler

if __name__=='__main__':
    panel=Panel()
    server=http.server.ThreadingHTTPServer(('127.0.0.1',8767),make_handler(panel))
    print('PANEL_URL http://127.0.0.1:8767/',flush=True)
    try:server.serve_forever()
    finally:
        if panel.client:panel.client.close()
        server.server_close()
