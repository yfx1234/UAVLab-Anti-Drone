"""SSH identity check and fixed stdin JSON RPC, with optional transport reuse."""
import base64
import hashlib
import hmac
import json
import logging
import os
import socket
import warnings
with warnings.catch_warnings():
    warnings.simplefilter('ignore');import paramiko
logging.getLogger('paramiko').setLevel(logging.ERROR)
HOST='192.168.50.100';LOCAL_IP='192.168.50.101'
RPC='python3 /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/zed_tracking/tools/perception_supervisor.py'
FINGERPRINT='CHnInkACqORSkPoeTv8qWJtjHUOjRIPHPvVIdPHLdeI'

class PinnedHost(paramiko.MissingHostKeyPolicy):
    def missing_host_key(self,client,hostname,key):
        actual=base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip('=')
        if not hmac.compare_digest(actual,FINGERPRINT):raise paramiko.SSHException('板子 SSH 主机身份不符')

class BoardClient:
    def __init__(self):self.password=None
    def connection(self):
        sock=socket.socket();sock.settimeout(8);sock.bind((LOCAL_IP,0))
        client=paramiko.SSHClient();client.set_missing_host_key_policy(PinnedHost())
        try:
            sock.connect((HOST,22))
            client.connect(HOST,username='uavlab',password=self.password,sock=sock,timeout=8,
                           banner_timeout=8,auth_timeout=8,look_for_keys=True,allow_agent=True)
            return client
        except Exception:client.close();sock.close();raise
    def call(self,action,params=None,request=None,timeout=50):
        with self.connection() as client:
            return self.call_on(client,action,params,request,timeout)
    def exec_command(self,command,timeout=45):
        """Run a fixed read/control command on the board (interceptor stack ops).

        Same pinned-connection and output limits as the RPC channel; callers
        must only pass constant command strings, never user input.
        """
        with self.connection() as client:
            return self.exec_on(client,command,timeout)
    def exec_on(self,client,command,timeout=45):
        stdin,stdout,stderr=client.exec_command(command,timeout=timeout)
        try:
            raw=stdout.read();error=stderr.read().decode(errors='replace')
            text=raw.decode(errors='replace')
            if len(raw)>512*1024:raise ValueError('板上命令输出过大')
            if not text.strip():raise ValueError('板上命令无输出'+('：'+error[-400:] if error.strip() else ''))
            return text
        finally:
            stdin.close();stdout.close();stderr.close()
    def fetch_file(self,remote,local,timeout=1800):
        """SFTP download of a fixed board artifact (annotated preview videos)."""
        with self.connection() as client:
            sftp=client.open_sftp()
            try:sftp.get(remote,local)
            finally:sftp.close()
    def call_on(self,client,action,params=None,request=None,timeout=50):
        """Use a new RPC channel on an already pinned/authenticated connection."""
        data=dict(request or {});data.update(action=action,params=params or {})
        payload=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
        if len(payload)>65536:raise ValueError('RPC 内容过大')
        stdin,stdout,stderr=client.exec_command(RPC,timeout=timeout)
        try:
            stdin.write(payload);stdin.flush();stdin.channel.shutdown_write()
            raw=stdout.read();error=stderr.read().decode(errors='replace')
            if len(raw)>8*1024*1024:raise ValueError('RPC 响应过大')
            try:result=json.loads(raw)
            except ValueError:raise ValueError('板上 RPC 响应无效：'+error[-800:])
            if isinstance(result,dict) and result.get('ok') is False:raise ValueError(result.get('error','板上操作失败'))
            return result
        finally:
            stdin.close();stdout.close();stderr.close()
