import unittest
import http.server, json, threading, urllib.request, urllib.error
import dashboard
from dashboard import command_for, preview_blockers, valid_request

class ProtectionTests(unittest.TestCase):
    # A missing action allowlist would permit shell input to reach the board.
    def test_unknown_and_shell_actions_are_rejected(self):
        for action in ['arm', 'heartbeat', 'uptime; reboot', '', 'preview_start']:
            with self.subTest(action=action), self.assertRaises(ValueError):
                command_for(action)

    # Missing ownership checks would open a second camera while another producer owns Argus.
    def test_preview_refuses_unknown_argus_camera_client(self):
        self.assertEqual(preview_blockers([(42,'python3','python3 custom_camera.py')],{42}),[42])

    def test_background_daemons_and_existing_preview_are_exempt(self):
        self.assertEqual(preview_blockers([(7,'nvargus-daemon','/usr/sbin/nvargus-daemon'),(9,'zed_preview','/home/uavlab/zed_preview/zed_preview')],{7,9}),[])

    def test_known_camera_producer_blocks_even_without_visible_argus_pid(self):
        self.assertEqual(preview_blockers([(15,'zed_perception_','/opt/zed_perception_node')],set()),[15])

    # Removing any of these checks lets an unrelated web page send control requests.
    def test_control_requires_local_host_origin_and_session_token(self):
        for host,origin,token in [('evil.test:8767','http://127.0.0.1:8767','secret'),('127.0.0.1:8767','https://evil.test','secret'),('127.0.0.1:8767',None,'secret'),('127.0.0.1:8767','http://127.0.0.1:8767','wrong')]:
            with self.subTest(host=host,origin=origin,token=token):
                self.assertFalse(valid_request(host,origin,token,'secret'))
        self.assertTrue(valid_request('127.0.0.1:8767','http://127.0.0.1:8767','secret','secret'))

    def test_unconnected_socket_sentinel_does_not_mark_all_processes_as_camera_owners(self):
        raw='u_str ESTAB 0 0 /tmp/argus_socket 123 * 0 users:(("nvargus-daemon",pid=7,fd=12))\n'
        raw+='u_str ESTAB 0 0 * 456 * 0 users:(("python3",pid=42,fd=9))\n'
        self.assertEqual(preview_blockers([(7,'nvargus-daemon','argus'),(42,'python3','python unrelated.py')],dashboard.argus_clients(raw)),[])

class HttpProtectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.panel=dashboard.Panel()
        cls.panel.password='test secret never exported'
        cls.server=http.server.ThreadingHTTPServer(('127.0.0.1',0),dashboard.make_handler(cls.panel))
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True);cls.thread.start()
    @classmethod
    def tearDownClass(cls):cls.server.shutdown();cls.server.server_close();cls.thread.join()
    def request(self,payload=None,token=None,origin='http://127.0.0.1:8767',path='/api/action'):
        headers={'Host':'127.0.0.1:8767','Origin':origin,'Content-Type':'application/json'}
        if token:headers['X-Panel-Token']=token
        req=urllib.request.Request('http://127.0.0.1:'+str(self.server.server_port)+path,data=json.dumps(payload).encode() if payload is not None else None,headers=headers)
        try:
            with urllib.request.urlopen(req,timeout=4) as r:return r.status,r.read()
        except urllib.error.HTTPError as error:return error.code,error.read()
    def test_action_api_rejects_cross_origin_before_running_commands(self):
        code,body=self.request({'action':'system'},self.panel.token,'https://evil.test')
        self.assertEqual(code,403)
    def test_connected_command_cannot_run_while_disconnected(self):
        code,body=self.request({'action':'system'},self.panel.token)
        self.assertEqual(code,400);self.assertIn('请先连接板子',body.decode())
    def test_action_api_rejects_arbitrary_command_fields(self):
        code,body=self.request({'action':'system','command':'reboot'},self.panel.token)
        self.assertEqual(code,400)
    def test_export_excludes_credentials(self):
        code,body=self.request(path='/api/export')
        self.assertEqual(code,200);self.assertNotIn(b'test secret never exported',body)

if __name__ == '__main__':unittest.main()
