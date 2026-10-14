#!/usr/bin/python3
"""Fixed root-owned helper: PC UTC -> Linux/RTC, only with no camera owner."""
import fcntl
import json
import math
import os
import pathlib
import pwd
import re
import stat
import subprocess
import sys
import time

def parse_target(data):
    if not isinstance(data,dict) or set(data)!={'pc_unix_s','boot_id'}:raise ValueError('invalid clock fields')
    value=data['pc_unix_s']
    if type(value) not in (float,int) or not math.isfinite(value) or not 1704067200<=value<4102444800:raise ValueError('invalid UTC value')
    if data['boot_id']!=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip():raise ValueError('boot changed')
    return value

def camera_active():
    names=(b'zed_perception_standalone',b'ZED_Explorer',b'ZED_Depth_Viewer',b'zed_perception_node',b'zed_preview')
    for process in pathlib.Path('/proc').iterdir():
        if not process.name.isdigit():continue
        try:
            command=(process/'cmdline').read_bytes()
            if any(name in command for name in names):return True
            for descriptor in (process/'fd').iterdir():
                try:device=os.readlink(descriptor)
                except OSError:continue
                if re.match(r'^/dev/(video\d|media\d|camchar|nvhost-vi|nvhost-nvcsi)',device):return True
        except OSError:continue
    return False

def camera_lock():
    root=pathlib.Path('/tmp/zed_perception')
    owner=pwd.getpwnam('uavlab')
    # A stable directory fd prevents ancestor replacement from redirecting root.
    directory=os.open(root,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW)
    try:
        if os.fstat(directory).st_uid!=owner.pw_uid:raise ValueError('unexpected camera directory owner')
        try:
            fd=os.open('camera.lock',os.O_RDWR|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW|os.O_NONBLOCK,0o600,dir_fd=directory)
            os.fchown(fd,owner.pw_uid,owner.pw_gid)
        except FileExistsError:fd=os.open('camera.lock',os.O_RDWR|os.O_NOFOLLOW|os.O_NONBLOCK,dir_fd=directory)
        info=os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid!=owner.pw_uid or info.st_nlink!=1:os.close(fd);raise ValueError('unsafe camera lock file')
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except Exception:os.close(fd);raise ValueError('camera lock is owned; release camera first')
        return fd
    finally:os.close(directory)

def main():
    if os.geteuid()!=0:raise ValueError('root-owned helper requires sudo')
    raw=sys.stdin.buffer.read(2049)
    if len(raw)>2048:raise ValueError('clock request too large')
    target=parse_target(json.loads(raw))
    with open('/run/zed-clock-sync.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        fd=camera_lock()
        try:
            if camera_active():raise ValueError('release camera before clock adjustment')
            time.clock_settime(time.CLOCK_REALTIME,target)
            try:
                result=subprocess.run(['/sbin/hwclock','--systohc','--utc','--rtc','/dev/rtc0'],capture_output=True,text=True,timeout=8)
                rtc_written=result.returncode==0;rtc_error=result.stderr[-500:] if result.returncode else None
            except (OSError,subprocess.TimeoutExpired) as error:rtc_written=False;rtc_error=str(error)
            print(json.dumps({'synced':True,'board_unix_s':time.time(),'rtc_written':rtc_written,'rtc_error':rtc_error}))
        finally:os.close(fd)

if __name__=='__main__':
    try:main()
    except Exception as error:print(json.dumps({'synced':False,'error':str(error)}));sys.exit(1)
