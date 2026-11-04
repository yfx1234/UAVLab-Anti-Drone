"""Motion modes use a fake link: these tests cannot move physical hardware."""
import json
import threading
import time
from unittest import mock

import pytest

from test_standalone_servo import panel, servo


class FakeLink:
    def __init__(self):
        self.calls = []
        self.cond = threading.Condition()
        self.acks = []
        self.fc = (1, 1)
        self.armed = False
        self.outputs = {"0": {"5": 1500}}
        self.output_t = time.monotonic()
        self.hb_t = time.monotonic()
        self.rx_pkts = self.rx_msgs = 1
        self.rx_bad = 0
        self.dest = ("127.0.0.1", 9999)
        self.clock = time.monotonic

    def alive(self):
        return True

    def send_test(self, slot, value, timeout_s=1):
        self.calls.append((slot, value, timeout_s))
        return {"result": 0, "result_name": "ACCEPTED", "ack_ms": 1}

    def read_parameters(self, names):
        return {name: 302 if "FUNC5" in name else 0 for name in names}


def wait_for(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate()


@pytest.fixture
def service():
    instance = servo.ServoService(link=FakeLink(), lease_timeout_s=0.3)
    yield instance
    instance.handle({"action": "stop"})


def test_pulse_duration_is_configurable_and_releases_afterwards(service):
    result = service.move(2, 0.1, True, mode="pulse", hold_s=0.2)
    assert result["ok"]
    assert result["lease_id"]
    assert service.state()["mode"] == "pulse"
    wait_for(lambda: service.state()["mode"] == "idle")
    assert service.link.calls[-1] == (2, 0, 0)


def test_hold_needs_the_owning_browser_heartbeat(service):
    result = service.move(2, 0.1, True, mode="hold")
    for _ in range(4):
        time.sleep(0.1)
        reply = service.handle({"action": "state", "lease_id": result["lease_id"]})
        assert reply["mode"] == "hold"
    assert all(call[2] > 0 for call in service.link.calls)
    wait_for(lambda: service.state()["mode"] == "idle")
    assert service.link.calls[-1][2] == 0
    assert "连接" in service.state()["motion_error"]


def test_old_or_wrong_browser_heartbeat_cannot_keep_new_motion_alive(service):
    first = service.move(2, 0.1, True, mode="hold")
    second = service.move(2, -0.1, True, mode="hold")
    assert first["lease_id"] != second["lease_id"]
    service.handle({"action": "state", "lease_id": first["lease_id"]})
    wait_for(lambda: service.state()["mode"] == "idle")
    assert service.link.calls[-1][2] == 0


@pytest.mark.parametrize("mode,duration", [("unknown", 1), ("pulse", 0),
                                          ("pulse", 31), ("pulse", float("nan"))])
def test_invalid_behavior_never_sends_a_packet(service, mode, duration):
    result = service.move(2, 0, True, mode=mode, hold_s=duration)
    assert result["ok"] is False
    assert service.link.calls == []


def test_stop_ends_hold_and_releases_output(service):
    service.move(2, 0.1, True, mode="hold")
    assert service.handle({"action": "stop"})["ok"]
    assert service.state()["mode"] == "idle"
    assert service.link.calls[-1] == (2, 0, 0)


def test_sequence_reports_progress_and_can_keep_last_position(service):
    result = service.seq_start([{"slot": 2, "norm": 0.1,
                                 "interp_ms": 0, "hold_ms": 50}], after="hold")
    assert result["ok"]
    wait_for(lambda: service.state()["mode"] == "hold")
    assert service.state()["seq_index"] == 1
    assert service.state()["seq_total"] == 1
    assert service.link.calls[0][:2] == (2, 0.1)


def test_inspection_reads_the_selected_physical_output(service):
    result = service.handle({"action": "inspect", "bank": "MAIN", "channel": 5})
    assert result["ok"]
    assert result["bank"] == "MAIN" and result["channel"] == 5
    assert result["params"]["PWM_MAIN_FUNC5"] == 302
    assert not service.link.calls


def test_panel_passes_behavior_and_browser_lease_to_board():
    with mock.patch.object(panel, "_ssh_exec", return_value=(0, 'SERVO_RESULT {"ok":true}')) as execute:
        panel._servo({"op": "move", "slot": 2, "norm": 0.1, "mode": "hold", "hold_s": 2.5})
        command = execute.call_args.args[0]
        assert "--mode hold" in command and "--hold-s 2.500" in command
        panel._servo({"op": "state", "lease_id": "browser123"})
        assert "--lease-id browser123" in execute.call_args.args[0]


def test_sequence_cli_preserves_end_behavior():
    request = {}
    with mock.patch.object(servo, "_control_request", side_effect=lambda data, **kw: request.update(data) or {"ok": True}):
        servo.main(["seq", "--points", json.dumps([{"norm": 0}]), "--after", "hold"])
    assert request["after"] == "hold"
