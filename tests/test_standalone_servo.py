"""Independent servo regressions. All actuator packets stay on local test sockets."""
import http.client
import importlib.util
import json
from pathlib import Path
import socket
import threading
import time
from unittest import mock

import pytest
from pymavlink.dialects.v20 import common as mavlink


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


panel = load_module("standalone_servo_panel", "panel/servo_panel.py")
servo = load_module("standalone_servo_link", "deploy/servo_mavlink.py")


def test_http_buttons_return_json_when_disconnected(tmp_path):
    server = panel.http.server.ThreadingHTTPServer(("127.0.0.1", 0), panel.Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        with mock.patch.object(panel, "__file__", str(tmp_path / "servo_panel.py")), \
                mock.patch.object(panel, "_SSH", None):
            conn = http.client.HTTPConnection(*server.server_address, timeout=2)
            conn.request("POST", "/api", json.dumps({"op": "state"}),
                         {"Content-Type": "application/json"})
            try:
                response = conn.getresponse()
                result = json.loads(response.read())
            except http.client.RemoteDisconnected:
                pytest.fail("Button POST dropped the connection instead of returning JSON")
            finally:
                conn.close()
            assert response.status == 200
            assert result["ok"] is False
            assert "未连接" in result["error"]
    finally:
        server.shutdown()
        server.server_close()
        worker.join(2)


def test_panel_uses_the_independent_servo_service():
    with mock.patch.object(panel, "_ssh_exec", return_value=(0, 'SERVO_RESULT {"ok":true}')) as execute:
        assert panel._servo({"op": "state"})["ok"]
        assert "deploy/servo_mavlink.py state" in execute.call_args.args[0]


def test_ssh_can_use_the_existing_computer_key():
    with mock.patch.object(panel.paramiko, "SSHClient") as client_class, \
            mock.patch.object(panel, "_SSH", None):
        panel._ssh_connect("192.0.2.1", "test", "")
        assert client_class.return_value.connect.call_args.kwargs["look_for_keys"] is True


@pytest.mark.parametrize("norm", [float("nan"), float("inf"), -1.1, 1.1])
def test_invalid_positions_do_not_send_any_actuator_packet(norm):
    service = servo.ServoService()
    try:
        with mock.patch.object(service.link, "send_test") as send:
            result = service.move(2, norm, True)
            assert result["ok"] is False
            send.assert_not_called()
    finally:
        service.link.sock.close()


@pytest.mark.parametrize("version,signed", [(1, False), (2, False), (2, True)])
def test_receiver_decodes_heartbeat_and_ack_across_datagrams(version, signed):
    link = servo.ServoLink(listen=("127.0.0.1", 0))
    stop = threading.Event()
    worker = threading.Thread(target=link.run, args=(stop,), daemon=True)
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx = mavlink.MAVLink(None, srcSystem=7, srcComponent=1)
    if signed:
        tx.signing.secret_key = bytes(32)
        tx.signing.sign_outgoing = True
    heartbeat = tx.heartbeat_encode(2, 12, 0, 0, 3)
    raw = heartbeat.pack(tx, force_mavlink1=version == 1)
    raw += tx.command_ack_encode(187, 0).pack(tx, force_mavlink1=version == 1)
    worker.start()
    try:
        peer.sendto(raw[:5], link.listen)
        peer.sendto(raw[5:], link.listen)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and not link.acks:
            time.sleep(0.01)
        assert link.fc == (7, 1), "valid MAVLink frames must identify the test autopilot"
        assert link.hb_t is not None
        assert link.alive()
        assert link.acks[0][2:4] == (187, 0)
        assert link.rx_bad == 0
    finally:
        stop.set()
        link.sock.close()
        peer.close()
        worker.join(2)


def test_sender_writes_only_the_selected_servo_slot_to_a_local_peer():
    link = servo.ServoLink(listen=("127.0.0.1", 0))
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    peer.settimeout(1)
    link.dest, link.fc = peer.getsockname(), (7, 1)
    link.last_rx_t = link.clock()
    try:
        with mock.patch.object(link, "_wait_ack", return_value=(1, link.clock(), 187, 0, 7, 1)):
            result = link.send_command(2, 0.1, attempts=1)
        assert result["result"] == 0
        message = mavlink.MAVLink(None).decode(bytearray(peer.recv(1024)))
        assert message.get_type() == "COMMAND_LONG"
        assert message.target_system == 7
        assert message.param2 == pytest.approx(0.1)
        assert message.param1 != message.param1  # NaN leaves other outputs unchanged.
        assert message.param7 == 0
    finally:
        peer.close()
        link.sock.close()


def test_inspection_requests_parameters_without_sending_actuator_commands():
    link = servo.ServoLink(listen=("127.0.0.1", 0))
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    peer.settimeout(1)
    link.dest, link.fc = peer.getsockname(), (7, 1)
    link.last_rx_t = link.clock()
    stop = threading.Event()
    receiver = threading.Thread(target=link.run, args=(stop,), daemon=True)
    receiver.start()

    def respond():
        try:
            data, address = peer.recvfrom(1024)
        except OSError:
            return
        request = mavlink.MAVLink(None).decode(bytearray(data))
        assert request.get_type() == "PARAM_REQUEST_READ"
        tx = mavlink.MAVLink(None, srcSystem=7, srcComponent=1)
        value = servo.struct.unpack("<f", servo.struct.pack("<i", 302))[0]
        response = tx.param_value_encode(b"PWM_MAIN_FUNC5", value,
                                        mavlink.MAV_PARAM_TYPE_INT32, 1, 0)
        peer.sendto(response.pack(tx), address)

    responder = threading.Thread(target=respond, daemon=True)
    responder.start()
    try:
        result = link.read_parameters(["PWM_MAIN_FUNC5"], timeout_s=1)
        assert result == {"PWM_MAIN_FUNC5": 302}
    finally:
        stop.set()
        link.sock.close()
        peer.close()
        receiver.join(2)
        responder.join(2)


def test_bench_test_addresses_one_peripheral_and_always_has_a_timeout():
    link = servo.ServoLink(listen=("127.0.0.1", 0))
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    peer.settimeout(1)
    link.dest, link.fc = peer.getsockname(), (7, 1)
    link.armed = False
    link.hb_t = link.last_rx_t = link.clock()
    try:
        with mock.patch.object(link, "_wait_ack", return_value=(1, link.clock(), 310, 0, 7, 1)):
            result = link.send_test(2, 0.05)
        assert result["result"] == 0
        message = mavlink.MAVLink(None).decode(bytearray(peer.recv(1024)))
        assert message.command == 310
        assert message.param1 == pytest.approx(0.05)
        assert 0 < message.param2 <= 1
        assert message.param5 == 1302  # PX4 peripheral function 302, not a motor.
    finally:
        peer.close()
        link.sock.close()


@pytest.mark.parametrize("armed", [None, True])
def test_bench_test_rejects_armed_or_unknown_state_without_transmitting(armed):
    link = servo.ServoLink(listen=("127.0.0.1", 0))
    link.armed = armed
    try:
        with mock.patch.object(link, "tx") as tx:
            result = link.send_test(2, 0.05)
            assert result.get("error")
            tx.assert_not_called()
    finally:
        link.sock.close()
