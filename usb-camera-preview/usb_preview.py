#!/usr/bin/env python3
"""USB-only preview for Ubuntu/Jetson. One capture worker serves every browser."""
import argparse
from collections import OrderedDict, deque
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
import os
from pathlib import Path
import re
from socketserver import ThreadingMixIn
import sys
import threading
import time
from urllib.parse import urlsplit
import uuid


class Camera:
    def __init__(self, options, cv2):
        self.options, self.cv2 = options, cv2
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.frames = OrderedDict()
        self.stamps = deque(maxlen=60)
        self.prefix = uuid.uuid4().hex[:12]
        self.sequence = 0
        self.error = '正在打开摄像头'
        self.width = self.height = 0
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        if self.thread:
            self.thread.join(timeout=3)

    def _failed(self, message):
        with self.lock:
            self.error = message
            self.frames.clear()
            self.stamps.clear()

    def _publish(self, frame):
        ok, encoded = self.cv2.imencode(
            '.jpg', frame, [self.cv2.IMWRITE_JPEG_QUALITY, self.options.quality])
        if not ok:
            raise RuntimeError('JPEG 编码失败')
        height, width = frame.shape[:2]
        with self.lock:
            self.sequence += 1
            key = '{}_{}'.format(self.prefix, self.sequence)
            self.frames[key] = (encoded.tobytes(), time.monotonic())
            while len(self.frames) > 6:
                self.frames.popitem(last=False)
            self.stamps.append(time.monotonic())
            self.width, self.height, self.error = width, height, None

    def _loop(self):
        cv = self.cv2
        while not self.closed.is_set():
            capture = None
            try:
                # All capture calls, including release, belong to this worker.
                capture = cv.VideoCapture()
                if not capture.open(self.options.device, cv.CAP_V4L2):
                    raise RuntimeError('无法打开 {}；请检查设备路径、权限和占用情况'.format(self.options.device))
                capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*self.options.fourcc))
                capture.set(cv.CAP_PROP_FRAME_WIDTH, self.options.width)
                capture.set(cv.CAP_PROP_FRAME_HEIGHT, self.options.height)
                capture.set(cv.CAP_PROP_FPS, self.options.fps)
                capture.set(cv.CAP_PROP_BUFFERSIZE, 1)
                next_encode = 0.0
                while not self.closed.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise RuntimeError('读取摄像头失败，正在重连')
                    now = time.monotonic()
                    if now >= next_encode:
                        self._publish(frame)
                        next_encode = max(next_encode + 1.0 / self.options.fps, now)
            except Exception as exc:
                self._failed(str(exc))
            finally:
                if capture is not None:
                    capture.release()
            self.closed.wait(1)

    def snapshot(self):
        with self.lock:
            latest = next(reversed(self.frames.values())) if self.frames else None
            age = time.monotonic() - latest[1] if latest else None
            online = latest is not None and age <= 3
            elapsed = self.stamps[-1] - self.stamps[0] if len(self.stamps) > 1 else 0
            fps = (len(self.stamps) - 1) / elapsed if elapsed > 0 and online else 0
            return dict(online=online, device=self.options.device,
                        width=self.width, height=self.height,
                        requested_width=self.options.width, requested_height=self.options.height,
                        requested_fps=self.options.fps, preview_fps=round(fps, 1),
                        frame_age_s=round(age, 3) if age is not None else None,
                        frames=self.sequence,
                        error=self.error or (None if online else '画面已过期，等待摄像头恢复'))

    def latest(self):
        with self.lock:
            if not self.frames:
                return None
            key = next(reversed(self.frames))
            jpeg, stamp = self.frames[key]
            if time.monotonic() - stamp > 3:
                return None
            return key, jpeg, stamp

    def image(self, key):
        with self.lock:
            value = self.frames.get(key)
            if value and time.monotonic() - value[1] <= 3:
                return value[0]
            return None


def make_handler(camera, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, body, content_type, status=200, download=False):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
            if download:
                self.send_header('Content-Disposition', 'attachment; filename="usb-camera-{}.jpg"'.format(time.strftime('%Y%m%d-%H%M%S')))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def js(self, value, status=200):
            self.send(json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8'),
                      'application/json; charset=utf-8', status)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/':
                self.send(page, 'text/html; charset=utf-8')
            elif path == '/api/status':
                self.js(camera.snapshot())
            elif path == '/api/frame/meta':
                latest = camera.latest()
                if latest is None:
                    self.js(dict(error='暂无新鲜画面'), 503)
                else:
                    key, jpeg, stamp = latest
                    self.js(dict(frame_key=key, image_url='/api/frame/{}.jpg'.format(key)))
            elif re.fullmatch(r'/api/frame/[a-zA-Z0-9_]+\.jpg', path):
                key = path.rsplit('/', 1)[1][:-4]
                jpeg = camera.image(key)
                if jpeg is None:
                    self.js(dict(error='帧已过期，请获取最新画面'), 410)
                else:
                    self.send(jpeg, 'image/jpeg')
            elif path == '/snapshot.jpg':
                latest = camera.latest()
                if latest is None:
                    self.js(dict(error='暂无新鲜画面'), 503)
                else:
                    self.send(latest[1], 'image/jpeg', download=True)
            else:
                self.js(dict(error='未找到'), 404)
    return Handler


class PreviewServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def get_request(self):
        sock, address = super().get_request()
        sock.settimeout(10)
        return sock, address


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Jetson / Ubuntu USB 摄像头网页预览')
    parser.add_argument('--device', default='/dev/video0', help='USB 摄像头 V4L2 节点或 /dev/v4l/by-id/ 路径')
    parser.add_argument('--host', default='0.0.0.0', help='监听地址，默认局域网可访问')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=15)
    parser.add_argument('--fourcc', choices=['MJPG', 'YUYV'], default='MJPG')
    parser.add_argument('--quality', type=int, default=80)
    options = parser.parse_args(argv)
    if not (1 <= options.port <= 65535 and 16 <= options.width <= 7680 and
            16 <= options.height <= 4320 and math.isfinite(options.fps) and
            1 <= options.fps <= 60 and 20 <= options.quality <= 95):
        parser.error('请检查端口、尺寸、帧率（1–60）和 JPEG 质量（20–95）')
    return options


def main(argv=None):
    options = parse_args(argv)
    if sys.platform != 'linux':
        print('请在插有 USB 摄像头的 Jetson / Ubuntu 上运行；Windows 只需打开浏览器。', file=sys.stderr)
        return 1
    try:
        import cv2
    except ImportError:
        print('缺少 OpenCV。请运行：sudo apt install python3-opencv', file=sys.stderr)
        return 1
    import fcntl
    import hashlib
    # Prevent two copies of this program from opening the same physical node.
    identity = hashlib.sha256(os.path.realpath(options.device).encode()).hexdigest()[:20]
    lock_path = '/tmp/uavlab-usb-preview-{}.lock'.format(identity)
    lock_file = open(lock_path, 'a')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        print('该摄像头已有 USB 预览进程；请使用原网页，或先结束原进程。', file=sys.stderr)
        return 1
    camera = Camera(options, cv2)
    server = None
    try:
        page = Path(__file__).with_name('usb_index.html').read_bytes()
        server = PreviewServer((options.host, options.port), make_handler(camera, page))
        camera.start()
        print('USB 相机监看已启动。Windows 打开：http://<Jetson的IP>:{}'.format(options.port), flush=True)
        print('设备：{}；请求 {}x{} @ {} fps；Ctrl+C 结束。'.format(
            options.device, options.width, options.height, options.fps), flush=True)
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        print('启动失败：{}'.format(exc), file=sys.stderr)
        return 1
    finally:
        if server is not None:
            server.server_close()
        camera.close()
        lock_file.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
