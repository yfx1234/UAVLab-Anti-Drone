"""Regression boundaries for USB re-enumeration and lossless service installation."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR = ROOT / 'deploy' / 'supervise_mavlink_usb.py'
INSTALLER = ROOT / 'deploy' / 'install_qgc_usb_reconnect.py'
COMMAND = ('/usr/bin/python3 -m MAVProxy.mavproxy '
           '--master=/dev/serial/by-id/flight-controller --default-modules= '
           '--out=udp:192.168.50.101:14550 --out=udp:192.168.50.101:14551 '
           '--out=udp:127.0.0.1:14555 --heartbeat-rate=0 --streamrate=-1 --nodtr')
UNIT = '[Service]\nEnvironment=PYTHONPATH=/vendor\nExecStart=' + COMMAND + '\nRestart=always\n'


def load_installer():
    spec = importlib.util.spec_from_file_location('qgc_installer', INSTALLER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class InstallerTests(unittest.TestCase):
    def test_preserves_original_mavproxy_command_and_environment(self):
        self.assertTrue(INSTALLER.exists(), 'service installer has not been implemented')
        output = load_installer().render_unit(UNIT, Path('/workspace/deploy/supervise_mavlink_usb.py'))
        self.assertIn(' -- ' + COMMAND + '\n', output)
        self.assertIn('Environment=PYTHONPATH=/vendor\n', output)
        self.assertIn('--device=/dev/serial/by-id/flight-controller', output)

    def test_second_install_is_a_noop(self):
        self.assertTrue(INSTALLER.exists(), 'service installer has not been implemented')
        module = load_installer()
        path = Path('/workspace/deploy/supervise_mavlink_usb.py')
        first = module.render_unit(UNIT, path)
        self.assertEqual(module.render_unit(first, path), first)

    def test_rejects_ambiguous_or_expanded_systemd_commands(self):
        self.assertTrue(INSTALLER.exists(), 'service installer has not been implemented')
        module = load_installer()
        for unit in [UNIT + 'ExecStart=/other\n', UNIT.replace(COMMAND, '/bin/bash -c run'),
                     UNIT.replace('--nodtr', '--nodtr $EXTRA'),
                     UNIT.replace('--nodtr', '--nodtr %i'),
                     UNIT.replace('--nodtr', '--nodtr \\\n --more')]:
            with self.subTest(unit=unit), self.assertRaises(ValueError):
                module.render_unit(unit, Path('/workspace/supervise_mavlink_usb.py'))

    def test_install_keeps_original_backup_and_rerun_changes_nothing(self):
        self.assertTrue(INSTALLER.exists(), 'service installer has not been implemented')
        module = load_installer()
        with tempfile.TemporaryDirectory() as tmp:
            unit = Path(tmp) / 'qgc.service'
            unit.write_text(UNIT)
            supervisor = Path('/workspace/supervise_mavlink_usb.py')
            backup = module.install_unit(unit, supervisor)
            self.assertEqual(backup.read_text(), UNIT)
            installed = unit.read_text()
            self.assertIsNone(module.install_unit(unit, supervisor))
            self.assertEqual(unit.read_text(), installed)
            self.assertEqual(len(list(Path(tmp).glob('*.bak_*'))), 1)


@unittest.skipUnless(os.name == 'posix', 'Linux subprocess and signal integration')
class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SUPERVISOR.exists(), 'USB supervisor has not been implemented')
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.device = self.root / 'usb-device'
        self.events = self.root / 'child-events.jsonl'
        self.child = self.root / 'child.py'
        self.child.write_text('''import os,sys,time,json,signal
from pathlib import Path
device,events=sys.argv[1:3]
if '--stubborn' in sys.argv: signal.signal(signal.SIGINT,signal.SIG_IGN)
with open(device,'rb') as fd:
 with open(events,'a') as log:
  log.write(json.dumps({'pid':os.getpid(),'inode':os.fstat(fd.fileno()).st_ino})+'\\n');log.flush()
 while True:time.sleep(.05)
''')

    def records(self):
        if not self.events.exists():
            return []
        return [json.loads(x) for x in self.events.read_text().splitlines() if x]

    def wait_for(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.025)
        self.fail('Expected lifecycle transition did not occur')

    def start(self, *extra):
        self.output = (self.root / 'supervisor-output.txt').open('w')
        self.addCleanup(self.output.close)
        self.proc = subprocess.Popen([sys.executable, str(SUPERVISOR), '--device', str(self.device),
                                     '--poll', '.04', '--settle', '.08', '--stop-timeout', '.25', '--',
                                     sys.executable, str(self.child), str(self.device), str(self.events), *extra],
                                    stdout=self.output, stderr=subprocess.STDOUT)
        self.addCleanup(self.stop)

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            self.proc.wait(timeout=3)

    def pid_alive(self, pid):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        return True

    def test_waits_for_absent_device_then_starts_once(self):
        self.start()
        time.sleep(.25)
        self.assertEqual(self.records(), [])
        self.device.write_bytes(b'device')
        self.wait_for(lambda: len(self.records()) == 1)
        time.sleep(.2)
        self.assertEqual(len(self.records()), 1)

    def test_same_path_new_inode_replaces_child_without_orphan(self):
        self.device.write_bytes(b'old')
        self.start()
        self.wait_for(lambda: len(self.records()) == 1)
        original = self.records()[0]
        replacement = self.root / 'new-device'
        replacement.write_bytes(b'new')
        replacement.replace(self.device)
        self.wait_for(lambda: len(self.records()) == 2)
        current = self.records()[1]
        self.assertNotEqual(current['inode'], original['inode'])
        self.assertEqual(current['inode'], self.device.stat().st_ino)
        self.assertFalse(self.pid_alive(original['pid']))
        self.stop()
        self.assertFalse(self.pid_alive(current['pid']))

    def test_removal_stops_child_and_waits_until_reappearance(self):
        self.device.write_bytes(b'old')
        self.start()
        self.wait_for(lambda: len(self.records()) == 1)
        original = self.records()[0]
        self.device.unlink()
        self.wait_for(lambda: not self.pid_alive(original['pid']))
        time.sleep(.2)
        self.assertEqual(len(self.records()), 1)
        self.device.write_bytes(b'new')
        self.wait_for(lambda: len(self.records()) == 2)

    def test_sigterm_escalates_stubborn_child_and_exits_cleanly(self):
        self.device.write_bytes(b'device')
        self.start('--stubborn')
        self.wait_for(lambda: len(self.records()) == 1)
        child_pid = self.records()[0]['pid']
        self.stop()
        self.assertEqual(self.proc.returncode, 0)
        self.assertFalse(self.pid_alive(child_pid))

    def test_sigterm_while_waiting_does_not_start_a_child(self):
        self.start()
        time.sleep(.15)
        self.stop()
        self.assertEqual(self.proc.returncode, 0)
        self.assertEqual(self.records(), [])


if __name__ == '__main__':
    unittest.main()
