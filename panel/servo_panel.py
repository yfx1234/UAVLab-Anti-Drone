#!/usr/bin/env python3
"""舵机控制面板（独立小面板，端口 8768）。

本机 ──SSH──> 板上 deploy/servo_mavlink.py ──UDP/MAVProxy──> 飞控 PWM ──> 舵机。
板上零配置依赖：只要 servo_mavlink.py 在板 + MAVProxy 含
--out=udp:127.0.0.1:14560 输出口（install_servo_mavlink.sh 自动处理）。

用法：python servo_panel.py  →  浏览器 http://127.0.0.1:8768/
"""
import json
import os
import shlex
import threading
import http.server
import urllib.parse

try:
    import paramiko
except ImportError:
    paramiko = None

HOST, PORT = "127.0.0.1", 8768
BOARD_CLI = "python3 $HOME/interceptor_ws/deploy/servo_cmd.py"

_LOCK = threading.Lock()
_SSH = None          # paramiko.SSHClient
_CFG = {"host": "192.168.50.100", "user": "uavlab", "password": ""}


def _ssh_exec(cmd, timeout_s=15.0):
    """在板子上执行命令，返回 (exit_code, stdout)。抛错=SSH 未连接。"""
    global _SSH
    with _LOCK:
        if _SSH is None:
            raise RuntimeError("未连接板子")
        try:
            _, out, err = _SSH.exec_command(cmd, timeout=timeout_s)
            code = out.channel.recv_exit_status()
            text = out.read().decode("utf-8", "replace")
            if code != 0:
                text += err.read().decode("utf-8", "replace")
            return code, text
        except Exception as exc:
            # 连接可能已断：丢弃会话，要求重连
            try:
                _SSH.close()
            except Exception:
                pass
            _SSH = None
            raise RuntimeError("SSH 连接断开：%s（请重新连接）" % exc)


def _ssh_connect(host, user, password):
    global _SSH, _CFG
    if paramiko is None:
        raise RuntimeError("本机未安装 paramiko：python -m pip install paramiko")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(host, username=user, password=password,
                   timeout=10, banner_timeout=10, auth_timeout=10,
                   allow_agent=False, look_for_keys=False)
    with _LOCK:
        if _SSH is not None:
            try:
                _SSH.close()
            except Exception:
                pass
        _SSH = client
        _CFG = {"host": host, "user": user, "password": password}


def _ssh_connect_capped(host, user, password, cap_s=15.0):
    """连接硬超时：图传链路闪断时 paramiko 可能挂死，超过 cap_s 强制报错返回。"""
    result = {"done": False, "error": None}

    def _work():
        try:
            _ssh_connect(host, user, password)
        except Exception as exc:
            result["error"] = str(exc) or type(exc).__name__
        result["done"] = True

    worker = threading.Thread(target=_work, daemon=True)
    worker.start()
    worker.join(cap_s)
    if not result["done"]:
        raise RuntimeError("连接超时（%ds）：图传链路闪断，稍等几秒重试" % int(cap_s))
    if result["error"]:
        raise RuntimeError(result["error"])


def _servo(payload):
    """通过 SSH 调板上 servo CLI，解析 SERVO_RESULT JSON。"""
    op = payload["op"]
    if op == "check":
        code, text = _ssh_exec("echo ok && ls $HOME/interceptor_ws/deploy/servo_cmd.py")
        ok = code == 0 and "ok" in text
        return {"ok": ok, "error": None if ok else "脚本不在板上：deploy/servo_cmd.py 缺失"}
    if op == "move":
        cmd = "%s move --slot %d --norm %.4f --ack 1" % (
            BOARD_CLI, int(payload["slot"]), float(payload["norm"]))
        timeout = 12.0
    elif op == "seq":
        pts = json.dumps(payload["points"], ensure_ascii=False)
        cmd = "%s seq --points %s" % (BOARD_CLI, shlex.quote(pts))
        timeout = 20.0
    elif op == "stop":
        cmd = "%s stop" % BOARD_CLI
        timeout = 8.0
    elif op == "state":
        cmd = "%s state" % BOARD_CLI
        timeout = 8.0
    else:
        return {"ok": False, "error": "未知 op"}
    code, text = _ssh_exec(cmd, timeout)
    for line in text.splitlines():
        if line.startswith("SERVO_RESULT "):
            doc = json.loads(line[len("SERVO_RESULT "):])
            doc.setdefault("ok", doc.get("result") == 0 or doc.get("ok", False))
            return doc
    return {"ok": False, "error": "板上无响应（exit %d）：%s" % (code, text[-200:])}


PAGE = r"""<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>舵机控制面板</title>
<style>
:root{--bg:#0f1420;--card:#1a2233;--line:#2a3650;--fg:#e6ebf5;--dim:#8b98b3;--acc:#4f8cff;--ok:#3ecf8e;--bad:#ff6b6b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.6 system-ui,"Microsoft YaHei";padding:20px}
.wrap{max-width:980px;margin:0 auto}
h1{font-size:20px;margin:0 0 4px}.sub{color:var(--dim);font-size:12px;margin-bottom:16px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:16px;margin-bottom:14px}
h2{font-size:14px;margin:0 0 10px;color:var(--acc)}
label{font-size:12px;color:var(--dim)}
input,button,select{background:#121a2b;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 10px;font:inherit}
input:focus{outline:1px solid var(--acc)}
button{cursor:pointer}button:hover{border-color:var(--acc)}
button.pri{background:var(--acc);border-color:var(--acc);color:#fff}
button.dan{background:#7a2727;border-color:#a33;color:#fff}
button.gh{background:transparent}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.row>*{flex:0 0 auto}
.stat{font-size:12px;padding:3px 10px;border-radius:10px;background:#0d1119;border:1px solid var(--line)}
.stat.ok{color:var(--ok)}.stat.bad{color:var(--bad)}
#sliderWrap{margin:14px 0}
input[type=range]{width:100%;height:34px;accent-color:var(--acc)}
.big{font-size:26px;font-weight:600;text-align:center;padding:6px}
.seqtab{width:100%;border-collapse:collapse;font-size:13px}
.seqtab th,.seqtab td{border-bottom:1px solid var(--line);padding:5px 6px;text-align:center}
.seqtab input{width:70px;text-align:center}
.msg{font-size:12px;color:var(--dim);min-height:18px;margin-top:8px;white-space:pre-wrap}
.err{color:var(--bad)}
kbd{background:#0d1119;border:1px solid var(--line);border-radius:4px;padding:0 5px;font-size:11px}
</style></head><body><div class="wrap">
<h1>舵机控制面板</h1>
<div class="sub">本机 → SSH → 板上 servo_mavlink.py → MAVProxy → 飞控 PWM → 舵机 ｜ 无任何使用限制，点击即发</div>

<div class="card">
 <h2>连接</h2>
 <div class="row">
  <label>板子IP <input id="host" value="192.168.50.100" size="14"></label>
  <label>用户 <input id="user" value="uavlab" size="8"></label>
  <label>密码 <input id="pass" type="password" size="10"></label>
  <label><input type="checkbox" id="remember"> 记住</label>
  <button class="pri" id="btnConn">连接板子</button>
  <span class="stat" id="connStat">未连接</span>
 </div>
 <div class="msg" id="connMsg">首次使用：装依赖 pip install paramiko；密码为板子 SSH 密码（三个空格）。</div>
</div>

<div class="card">
 <h2>通道配置（保存到本机）</h2>
 <div class="row">
  <label>槽位(功能301+槽)
   <select id="slot"><option value="1">1 (FUNC301，网枪未装时舵机用这个)</option><option value="2">2 (FUNC302)</option><option value="3">3 (FUNC303)</option><option value="4">4 (FUNC304)</option><option value="5">5 (FUNC305)</option><option value="6">6 (FUNC306)</option></select></label>
  <label>行程下限µs <input id="usMin" value="1000" size="6"></label>
  <label>行程上限µs <input id="usMax" value="2000" size="6"></label>
  <label>角度行程° <input id="deg" value="180" size="5"></label>
  <span class="stat">物理口：<b id="phys">?</b></span>
 </div>
 <div class="msg">QGC 对应参数：PWM_MAIN/AUX_FUNC<b>通道号</b>=<span id="funcHint">302</span>、MIN/MAX=行程上下限。槽≠物理通道：槽是 PX4 内部 Actuator Set 编号，物理通道由 FUNC 参数绑定。</div>
</div>

<div class="card">
 <h2>手动控制</h2>
 <div id="sliderWrap"><input type="range" id="slider" min="0" max="1000" value="500"></div>
 <div class="big" id="angleShow">90.0°</div>
 <div class="row" style="justify-content:center">
  <label>精确µs <input id="usIn" size="7"></label>
  <button class="pri" id="btnGo">转到</button>
  <span class="stat" id="lastStat">—</span>
 </div>
</div>

<div class="card">
 <h2>动作序列（从哪转到哪再转到哪）</h2>
 <table class="seqtab"><thead><tr><th>#</th><th>角度°</th><th>转到用时ms</th><th>保持ms</th><th></th></tr></thead>
 <tbody id="seqBody"></tbody></table>
 <div class="row" style="margin-top:10px">
  <button id="btnAdd">+ 加一点</button>
  <button class="pri" id="btnRun">▶ 执行序列</button>
  <button class="dan" id="btnStop">■ 停止</button>
  <span class="stat" id="seqStat">空闲</span>
 </div>
 <div class="msg">插值时间=从上一个位置转到本位置的时长（板端 20ms 步进平滑）。</div>
</div>

<div class="card">
 <h2>使用说明</h2>
 <div class="msg" style="min-height:0">
 1. QGC 一次性设置（例：舵机接 I/O 座 6 号通道）：PWM_MAIN_FUNC6=<span class="funcEcho">302</span>、
 PWM_MAIN_MIN6/MAX6=行程脉宽；想未解锁也能动舵机就设 PWM_MAIN_DIS6=中位脉宽(如1500)。<br>
 2. 槽位与 FUNC 的关系：槽 N ⇔ 功能号 300+N ⇔ DO_SET_ACTUATOR param N。槽1 通常留给网枪(301)。<br>
 3. 所有命令立即下发，无解锁/高度等任何限制。
 </div>
</div>
</div>

<script>
const $=id=>document.getElementById(id);
function cfg(){const mn=parseFloat($('usMin').value)||1000,mx=parseFloat($('usMax').value)||2000;
 return {slot:parseInt($('slot').value),mn,mx,deg:parseFloat($('deg').value)||180,
         ch:parseInt($('slot').dataset.ch||'6')};}
function us2norm(us){const c=cfg(),mid=(c.mn+c.mx)/2;return (us-mid)/((c.mx-c.mn)/2);}
function norm2us(n){const c=cfg(),mid=(c.mn+c.mx)/2;return mid+n*((c.mx-c.mn)/2);}
function ang2us(a){const c=cfg();return c.mn+(a/c.deg)*(c.mx-c.mn);}
function us2ang(us){const c=cfg();return (us-c.mn)/(c.mx-c.mn)*c.deg;}
// localStorage 恢复
(function(){try{
 const saved=JSON.parse(localStorage.getItem('servoCfg')||'{}');
 if(saved.slot)$('slot').value=saved.slot;
 if(saved.usMin)$('usMin').value=saved.usMin;
 if(saved.usMax)$('usMax').value=saved.usMax;
 if(saved.deg)$('deg').value=saved.deg;
 const p=JSON.parse(localStorage.getItem('servoConn')||'{}');
 if(p.host)$('host').value=p.host; if(p.user)$('user').value=p.user;
 if(p.remember&&p.pass){$('pass').value=p.pass;$('remember').checked=true;}
}catch(e){}
 const seq=JSON.parse(localStorage.getItem('servoSeq')||'null');
 if(seq&&seq.length)seq.forEach(p=>addRow(p));
 else{addRow({a:45,h:500,i:600});addRow({a:135,h:500,i:600});addRow({a:90,h:1000,i:600});}
 refreshPhys();
})();
function saveCfg(){localStorage.setItem('servoCfg',JSON.stringify(cfg()));
 localStorage.setItem('servoConn',JSON.stringify({host:$('host').value,user:$('user').value,
   remember:$('remember').checked,pass:$('remember').checked?$('pass').value:''}));
 localStorage.setItem('servoSeq',JSON.stringify(seqPts()));refreshPhys();}
function refreshPhys(){const s=parseInt($('slot').value),func=300+s;
 const isMain=localStorage.getItem('servoDomain')!=='aux';
 $('phys').textContent=isMain?('MAIN'+localStorage.getItem('servoCh')):'AUX'+localStorage.getItem('servoCh');
 $('funcHint').textContent=func;document.querySelectorAll('.funcEcho').forEach(e=>e.textContent=func);}
['slot','usMin','usMax','deg'].forEach(id=>$(id).onchange=saveCfg);
// ---- 滑条 ----
let sliderTimer=null;
$('slider').oninput=()=>{const v=parseInt($('slider').value);const ang=v/1000*cfg().deg;
 $('angleShow').textContent=ang.toFixed(1)+'°';$('usIn').value=Math.round(ang2us(ang));
 clearTimeout(sliderTimer);sliderTimer=setTimeout(()=>sendMove(ang2us(ang)),150);};
$('btnGo').onclick=()=>{const us=parseFloat($('usIn').value);if(!isNaN(us)){sendMove(us);
 $('angleShow').textContent=us2ang(us).toFixed(1)+'°';}};
function sendMove(us){const c=cfg();
 api({op:'move',slot:c.slot,norm:us2norm(us)},d=>{$('lastStat').textContent=d.ok?
   ('已下发 '+Math.round(us)+'µs · ACK '+ (d.result_name||'')):'';showErr('connMsg',d);});}
// ---- 序列 ----
function addRow(p){p=p||{a:90,h:500,i:600};const tb=$('seqBody');const tr=document.createElement('tr');
 tr.innerHTML='<td></td><td><input class="a" value="'+(p.a!==undefined?p.a:'')+'"></td>'+
  '<td><input class="i" value="'+(p.i||600)+'"></td><td><input class="h" value="'+(p.h||500)+'"></td>'+
  '<td><button class="gh del">删</button></td>';
 tr.querySelector('.del').onclick=()=>{tr.remove();renum();saveCfg();};
 tb.appendChild(tr);renum();}
function renum(){[...$('seqBody').children].forEach((tr,i)=>tr.children[0].textContent=i+1);}
function seqPts(){return [...$('seqBody').children].map(tr=>({
 a:parseFloat(tr.querySelector('.a').value),i:parseFloat(tr.querySelector('.i').value)||0,
 h:parseFloat(tr.querySelector('.h').value)||0}));}
$('btnAdd').onclick=()=>{addRow({a:90,h:500,i:600});saveCfg();};
$('btnRun').onclick=()=>{const c=cfg();const pts=seqPts();if(!pts.length)return message('序列为空');
 // 角度→µs→norm
 const out=pts.map(p=>({norm:us2norm(ang2us(p.a)),interp_ms:p.i||0,hold_ms:p.h||0}));
 api({op:'seq',points:out},d=>{$('seqStat').textContent=d.ok?'执行中('+out.length+'点)':'失败';
  $('seqStat').className='stat '+(d.ok?'ok':'bad');if(d.ok)pollSeq();showErr('seqStat',d);});};
$('btnStop').onclick=()=>api({op:'stop'},d=>{$('seqStat').textContent='已停止';});
function pollSeq(){const t=setInterval(()=>api({op:'state'},d=>{
 if(!d.seq_running){clearInterval(t);$('seqStat').textContent='空闲';$('seqStat').className='stat';}
 else{$('seqStat').textContent='执行中…';}},800),800);}
// ---- API ----
async function api(payload,cb){try{
 const r=await fetch('/api',{method:'POST',headers:{'Content-Type':'application/json'},
   body:JSON.stringify(payload)});
 const d=await r.json();cb&&cb(d);
}catch(e){cb&&cb({ok:false,error:'本机面板异常：'+e});}}
function message(t){$('connMsg').textContent=t;}
function showErr(id,d){if(d&&d.ok===false&&d.error){const el=$(id);el.textContent=
 (el.textContent?el.textContent+'\n':'')+d.error;el.classList.add('err');
 setTimeout(()=>el.classList.remove('err'),4000);}}
// ---- 连接 ----
$('btnConn').onclick=async()=>{saveCfg();
 $('connStat').textContent='连接中…';$('connStat').className='stat';
 const d=await new Promise(res=>fetch('/api',{method:'POST',
  headers:{'Content-Type':'application/json'},
  body:JSON.stringify({op:'connect',host:$('host').value,user:$('user').value,
   pass:$('pass').value})}).then(r=>r.json()).catch(e=>({ok:false,error:''+e})));
 if(d.ok){$('connStat').textContent='已连接 '+$('host').value;$('connStat').className='stat ok';
  message('连接成功');refreshPhys();
  api({op:'state'},s=>{if(s.alive!==undefined)$('lastStat').textContent=
   '链路'+(s.alive?'正常':'等待MAVProxy流')+(s.last?(' · 最近槽值 '+JSON.stringify(s.last)):'');});
 }else{$('connStat').textContent='失败';$('connStat').className='stat bad';
  message('连接失败：'+(d.error||''));}};
</script></body></html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _json(self, doc, code=200):
        body = json.dumps(doc, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _page(self):
        body = PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if urllib.parse.urlparse(self.path).path in ("/", "/index.html"):
            self._page()
        else:
            self.send_error(404)

    def do_POST(self):
        if urllib.parse.urlparse(self.path).path != "/api":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length") or 0)
            payload = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            self._json({"ok": False, "error": "请求 JSON 无效"})
            return
        op = payload.get("op")
        try:
            if op == "connect":
                _ssh_connect_capped(str(payload["host"]).strip(),
                                    str(payload["user"]).strip(),
                                    str(payload.get("pass", "")))
                self._json({"ok": True})
            elif _SSH is None:
                self._json({"ok": False, "error": "未连接板子（先点连接板子）"})
            else:
                self._json(_servo(payload))
        except Exception as exc:
            self._json({"ok": False, "error": str(exc)})


def main():
    try:
        server = http.server.ThreadingHTTPServer((HOST, PORT), Handler)
    except OSError as exc:
        print("启动失败：端口 %d 被占用（%s）。" % (PORT, exc))
        print("可能已有一个面板实例在运行：先关掉旧的命令行窗口，或重启电脑后重试。")
        input("按回车退出...")
        return 1
    print("舵机面板 http://%s:%d/  （Ctrl+C 停止）" % (HOST, PORT), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
