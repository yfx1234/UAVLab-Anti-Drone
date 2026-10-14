"""Low-frequency, read-only hardware telemetry. No SDK, GPU work or shell commands."""
import json
import math
import os
import pathlib
import re
import socket
import struct
import time
import uuid


def read_hardware(root=pathlib.Path('/'), disk_root=None):
    root=pathlib.Path(root)
    def text(path):
        try:return path.read_text().strip()
        except OSError:return ''
    def number(path, minimum=0, maximum=float('inf')):
        try:
            value=float(text(path))
            return value if math.isfinite(value) and minimum<=value<=maximum else None
        except ValueError:return None
    temperatures={'cpu':None,'gpu':None,'max':None};values=[]
    for zone in (root/'sys/class/thermal').glob('thermal_zone*'):
        milli=number(zone/'temp',1000,150000)
        if milli is None:continue
        value=milli/1000;values.append(value);name=text(zone/'type')
        if name=='cpu-thermal':temperatures['cpu']=value
        if name=='gpu-thermal':temperatures['gpu']=value
    if values:temperatures['max']=max(values)
    voltage=power=None
    # Discover by rail label; hwmon indices can change across boots.
    for label in (root/'sys/bus/i2c/drivers/ina3221').glob('*/hwmon/hwmon*/in*_label'):
        if text(label)!='VDD_IN':continue
        channel=label.name.removeprefix('in').removesuffix('_label')
        mv=number(label.with_name('in'+channel+'_input'),1,100000)
        ma=number(label.with_name('curr'+channel+'_input'),0,100000)
        voltage=None if mv is None else mv/1000
        power=None if mv is None or ma is None else mv*ma/1000000
        break
    available=None
    match=re.search(r'^MemAvailable:\s+(\d+)\s+kB',text(root/'proc/meminfo'),re.M)
    if match:available=int(match[1])
    network={'interface':None,'carrier':None,'speed_mbps':None}
    target=struct.unpack('<I',socket.inet_aton('192.168.50.101'))[0];routes=[]
    for line in text(root/'proc/net/route').splitlines()[1:]:
        try:
            fields=line.split();destination=int(fields[1],16);mask=int(fields[7],16)
            if int(fields[3],16)&1 and target&mask==destination&mask:routes.append((mask.bit_count(),fields[0]))
        except (ValueError,IndexError):continue
    if routes:
        interface=max(routes)[1];base=root/'sys/class/net'/interface
        carrier=text(base/'carrier');speed=number(base/'speed',1,100000)
        network={'interface':interface,'carrier':None if carrier not in ('0','1') else carrier=='1',
                 'speed_mbps':speed}
    try:
        disk=os.statvfs(disk_root or root);free=disk.f_bavail*disk.f_frsize
    except (OSError,AttributeError):free=None
    return {'temperatures_c':temperatures,'module_input_voltage_v':voltage,'module_power_w':power,
            'power_rail':'VDD_IN' if voltage is not None else None,'mem_available_kib':available,
            'disk_free_bytes':free,'network':network,'radio_signal_dbm':None,
            'radio_signal_status':'not_provided','source':'linux_sysfs_read_only'}


def hardware_snapshot(boot_id,cache_file,disk_root=None,now=None):
    now=time.monotonic() if now is None else now;cache_file=pathlib.Path(cache_file)
    try:
        cached=json.loads(cache_file.read_text())
        age=now-cached['sample_monotonic_s']
        if cached.get('schema_version')==1 and cached.get('boot_id')==boot_id and 0<=age<2:
            return dict(cached,sample_age_s=age)
    except (OSError,ValueError,KeyError,TypeError):pass
    started=time.perf_counter()
    result=dict(read_hardware(disk_root=disk_root),schema_version=1,boot_id=boot_id,
                sample_monotonic_s=now,sample_age_s=0)
    result['collection_ms']=(time.perf_counter()-started)*1000
    temporary=cache_file.with_name(cache_file.name+'.'+uuid.uuid4().hex+'.tmp')
    try:
        cache_file.parent.mkdir(parents=True,exist_ok=True)
        temporary.write_text(json.dumps(result,allow_nan=False),encoding='utf8');temporary.replace(cache_file)
    except OSError:pass # Telemetry cache failure must not block camera controls.
    finally:
        try:temporary.unlink(missing_ok=True)
        except OSError:pass
    return result
