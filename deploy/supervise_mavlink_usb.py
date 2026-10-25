#!/usr/bin/env python3
"""Retire stale MAVProxy sessions when a USB serial device is re-enumerated.

The supervisor only stats the device. MAVProxy remains its sole serial reader.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def device_identity(device: Path):
    try:
        info = device.stat()
    except FileNotFoundError:
        return None
    return info.st_dev, info.st_ino, info.st_rdev


def stop_child(child: subprocess.Popen, timeout: float):
    """Bound shutdown and reap the child before another process can own USB."""
    try:
        os.killpg(child.pid, signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait()


def supervise(device: Path, command: list[str], poll: float = .25,
              settle: float = .75, stop_timeout: float = 3.0) -> int:
    stopping = threading.Event()
    previous_handlers = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, lambda *_: stopping.set())
    candidate = None
    stable_since = 0.0
    child = None
    try:
        while not stopping.is_set():
            current = device_identity(device)
            if current != candidate:
                candidate = current
                stable_since = time.monotonic()
            if current is None or time.monotonic() - stable_since < settle:
                stopping.wait(poll)
                continue
            opened_identity = current
            print('USB ready; starting MAVProxy', flush=True)
            child = subprocess.Popen(command, start_new_session=True)
            try:
                while not stopping.wait(poll):
                    if device_identity(device) != opened_identity:
                        print('USB removed or replaced; retiring old MAVProxy', flush=True)
                        break
                    result = child.poll()
                    if result is not None:
                        print('MAVProxy exited: %s' % result, flush=True)
                        return result
            finally:
                stop_child(child, stop_timeout)
                child = None
            candidate = None
        return 0
    finally:
        if child is not None:
            stop_child(child, stop_timeout)
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', type=Path, required=True)
    parser.add_argument('--poll', type=float, default=.25)
    parser.add_argument('--settle', type=float, default=.75)
    parser.add_argument('--stop-timeout', type=float, default=3.0)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or args.poll <= 0 or args.settle < 0 or args.stop_timeout <= 0:
        parser.error('command and valid positive timing bounds are required')
    return supervise(args.device, command, args.poll, args.settle, args.stop_timeout)


if __name__ == '__main__':
    raise SystemExit(main())
