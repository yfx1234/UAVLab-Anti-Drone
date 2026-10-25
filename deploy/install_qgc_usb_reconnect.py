#!/usr/bin/env python3
"""Wrap the existing QGC forwarder without dropping its configured outputs."""
import argparse
from pathlib import Path
import os
import shlex
import shutil
import subprocess
import tempfile
import time


def render_unit(text: str, supervisor: Path) -> str:
    lines = text.splitlines(keepends=True)
    commands = [line for line in lines if line.startswith('ExecStart=')]
    if len(commands) != 1:
        raise ValueError('Expected one single-line ExecStart; unit left unchanged')
    line = commands[0]
    command = line.removeprefix('ExecStart=').rstrip('\r\n')
    if any(char in command for char in ('$', '%', '\\')):
        raise ValueError('Expanded or continued ExecStart is unsupported')
    tokens = shlex.split(command)
    path = supervisor.as_posix()
    if not path.startswith('/') or any(char in path for char in ' \t\r\n$%"\\'):
        raise ValueError('Supervisor needs an absolute path without systemd quoting')
    if len(tokens) > 1 and tokens[1] == path:
        return text
    if len(tokens) < 4 or tokens[:3] != ['/usr/bin/python3', '-m', 'MAVProxy.mavproxy']:
        raise ValueError('Only the known direct MAVProxy Python command is supported')
    masters = [arg.split('=', 1)[1] for arg in tokens if arg.startswith('--master=')]
    if len(masters) != 1 or not masters[0].startswith('/dev/serial/by-id/'):
        raise ValueError('Expected one stable by-id USB master')
    device = masters[0]
    if any(char in device for char in ' \t\r\n"\\'):
        raise ValueError('Unsupported device path')
    wrapped = 'ExecStart=/usr/bin/python3 %s --device=%s -- %s\n' % (path, device, command)
    return text.replace(line, wrapped, 1)


def atomic_write(path: Path, data: bytes):
    mode = path.stat().st_mode & 0o777
    descriptor, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(name, mode)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def install_unit(unit: Path, supervisor: Path):
    original = unit.read_bytes()
    updated = render_unit(original.decode('utf-8'), supervisor).encode('utf-8')
    if updated == original:
        return None
    backup = unit.with_name(unit.name + '.bak_' + str(time.time_ns()))
    shutil.copy2(unit, backup)
    atomic_write(unit, updated)
    return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--unit', type=Path,
                        default=Path.home() / '.config/systemd/user/qgc-mavlink-forward.service')
    args = parser.parse_args()
    supervisor = Path(__file__).resolve().with_name('supervise_mavlink_usb.py')
    if not supervisor.is_file():
        parser.error('Supervisor file is missing')
    backup = install_unit(args.unit, supervisor)
    if backup is None:
        print('USB supervision already installed; service unchanged')
        return 0
    print('Original service backup: %s' % backup, flush=True)
    try:
        subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
        subprocess.run(['systemctl', '--user', 'restart', args.unit.name], check=True)
    except subprocess.CalledProcessError:
        atomic_write(args.unit, backup.read_bytes())
        subprocess.run(['systemctl', '--user', 'daemon-reload'], check=False)
        subprocess.run(['systemctl', '--user', 'restart', args.unit.name], check=False)
        raise
    print('USB supervision installed; all original forwarding arguments preserved')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
