"""One board stream shared by every browser; immutable images indexed by frame key."""
from collections import OrderedDict
from dataclasses import dataclass
import json
import socket
import struct
import threading
import time

@dataclass(frozen=True)
class PreviewPacket:
    metadata: dict
    jpeg: bytes
    received_at: float

class FrameCache:
    def __init__(self,capacity=4):
        self.capacity=capacity;self.lock=threading.Lock();self.frames=OrderedDict()
    def put(self,metadata,jpeg):
        key=metadata.get('frame_key')
        if not isinstance(key,str) or not key or len(key)>100:raise ValueError('invalid frame key')
        with self.lock:
            if key in self.frames:return self.frames[key]
            packet=PreviewPacket(metadata.copy(),bytes(jpeg),time.monotonic())
            self.frames[key]=packet
            while len(self.frames)>self.capacity:self.frames.popitem(last=False)
            return packet
    def latest(self):
        with self.lock:return next(reversed(self.frames.values()),None) if self.frames else None
    def get(self,key):
        with self.lock:return self.frames.get(key)
    def clear(self):
        with self.lock:self.frames.clear()

def _read_exact(sock,n):
    parts=[]
    while n:
        data=sock.recv(n)
        if not data:raise EOFError('stream closed inside packet')
        parts.append(data);n-=len(data)
    return b''.join(parts)

def read_packet(sock):
    jn,in_=struct.unpack('!II',_read_exact(sock,8))
    if not 0<jn<=65536 or not 0<in_<=4*1024*1024:raise ValueError('packet exceeds limit')
    meta=json.loads(_read_exact(sock,jn))
    if not isinstance(meta,dict):raise ValueError('metadata must be object')
    data=_read_exact(sock,in_)
    if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):raise ValueError('invalid JPEG')
    return meta,data

class FrameReceiver:
    def __init__(self,host='192.168.50.100',local_ip='192.168.50.101',port=49543):
        self.host=host;self.local_ip=local_ip;self.port=port;self.cache=FrameCache()
        self.enabled=threading.Event();self.closed=threading.Event();self.sock=None
        self.error=None;self.frames=0;self.bytes=0;self.started=time.monotonic()
        self.listener=None;self.thread=threading.Thread(target=self._loop,daemon=True);self.thread.start()
    def set_enabled(self,enabled):
        if enabled:self.enabled.set()
        else:
            self.enabled.clear();self.cache.clear()
            if self.sock:
                try:self.sock.shutdown(socket.SHUT_RDWR)
                except OSError:pass
    def close(self):self.closed.set();self.set_enabled(False)
    def _loop(self):
        while not self.closed.is_set():
            if not self.enabled.wait(.25):continue
            try:
                with socket.socket() as s:
                    self.sock=s;s.settimeout(4);s.bind((self.local_ip,0));s.connect((self.host,self.port))
                    self.error=None
                    while self.enabled.is_set() and not self.closed.is_set():
                        meta,jpeg=read_packet(s);old=self.cache.latest()
                        packet=self.cache.put(meta,jpeg)
                        if old is None or old.metadata['frame_key']!=meta['frame_key']:
                            self.frames+=1;self.bytes+=len(jpeg)
                            if self.listener:self.listener(packet)
            except (OSError,ValueError,EOFError) as e:self.error=str(e)
            finally:self.sock=None
            self.closed.wait(.5)
    def snapshot(self):
        p=self.cache.latest();elapsed=max(.001,time.monotonic()-self.started)
        return {'enabled':self.enabled.is_set(),'frame_age_s':None if p is None else time.monotonic()-p.received_at,
                'received_frames':self.frames,'average_mbps':self.bytes*8/elapsed/1e6,'error':self.error}
