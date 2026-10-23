import pathlib
import tempfile
import time
import unittest
import dashboard
from dashboard import valid_request,Panel

class DashboardTests(unittest.TestCase):
    def test_remote_origin_rejected(self):
        self.assertFalse(valid_request('127.0.0.1:8767','https://example.com','token','token'))
        self.assertFalse(valid_request('127.0.0.1:8767','http://127.0.0.1:8767','bad','token'))
        self.assertTrue(valid_request('127.0.0.1:8767','http://127.0.0.1:8767','token','token'))
    def test_flight_action_rejected_before_any_network_job(self):
        panel=Panel()
        try:
            for action in ('chain_start_live','chain_start','chain_stop','watcher_start'):
                with self.assertRaises(ValueError):panel.submit({'action':action})
            self.assertFalse(panel.snapshot()['connected'])
        finally:panel.close()
    @unittest.skipIf(dashboard.geometry is None,'需要仓库 deploy/mounting_geometry.py')
    def test_geometry_save_is_validated_before_a_constant_board_command(self):
        panel=Panel()
        try:
            default=dashboard.geometry.DEFAULT_CONFIG
            for params in ({},{'config':{}},{'config':dict(default,camera_top_m=[0,0.9,0])},{'config':dict(default,gun_axis_tilt_deg=[0,0])},{'config':default,'command':'rm -rf ~'}):
                with self.assertRaises(ValueError):panel.submit({'action':'interceptor_geometry_save','params':params})
            calls=[]
            class Board:
                def exec_json(self,command,payload,timeout=30):
                    calls.append((command,payload));return {'ok':True,'source':'file','config':payload['config']}
            panel.board=Board();panel.connected=True
            jid=panel.submit({'action':'interceptor_geometry_save','request_id':'geometry1','params':{'config':dict(default,flight_controller_m=[0,0,0.18004])}})
            for _ in range(100):
                job=panel.jobs.get(jid)
                if job['state'] in ('complete','error'):break
                time.sleep(0.02)
            self.assertEqual(job['state'],'complete',job.get('error'))
            self.assertEqual(calls,[(dashboard.INTERCEPTOR_GEOMETRY_SAVE_CMD,{'config':dict(default,flight_controller_m=[0.0,0.0,0.18])})])
            self.assertEqual(dashboard.INTERCEPTOR_GEOMETRY_SAVE_CMD,'python3 /home/uavlab/interceptor_ws/deploy/mounting_geometry.py save')
        finally:panel.close()
    def test_panel_deployed_without_repository_tools_leaves_geometry_validation_to_board(self):
        # 面板目录单独复制部署（旁边没有仓库 deploy/）时仍能启动；安装参数交板上脚本完整校验
        original_base=dashboard.BASE
        with tempfile.TemporaryDirectory() as root:
            dashboard.BASE=pathlib.Path(root)/'jetson_dashboard'
            try:self.assertIsNone(dashboard._load_geometry_module())
            finally:dashboard.BASE=original_base
        original=dashboard.geometry;dashboard.geometry=None;panel=Panel()
        try:
            for params in ({},{'config':'x'},{'config':{},'command':'rm -rf ~'}):
                with self.assertRaises(ValueError):panel.submit({'action':'interceptor_geometry_save','params':params})
            calls=[]
            class Board:
                def exec_json(self,command,payload,timeout=30):
                    calls.append((command,payload));raise ValueError('缺少安装参数：RTK 天线')
            panel.board=Board();panel.connected=True
            jid=panel.submit({'action':'interceptor_geometry_save','request_id':'geometry2','params':{'config':{'camera_right_eye':'forward'}}})
            for _ in range(100):
                job=panel.jobs.get(jid)
                if job['state'] in ('complete','error'):break
                time.sleep(0.02)
            self.assertEqual(job['state'],'error')
            self.assertIn('缺少安装参数',job.get('error') or '')
            self.assertEqual(calls,[(dashboard.INTERCEPTOR_GEOMETRY_SAVE_CMD,{'config':{'camera_right_eye':'forward'}})])
        finally:dashboard.geometry=original;panel.close()
    def test_trigger_params_save_is_format_checked_and_board_errors_surface(self):
        # 击发参数：面板只查格式（四个已知键、有限数字），范围与解锁/击发中拒绝由板上击发服务负责
        panel=Panel()
        try:
            self.assertEqual(panel.snapshot()['features'],['trigger_params'],'frontend gates saving on this flag')
            good={'low_us':1000,'high_us':2000,'hold_s':1.0,'low_s':0.3}
            for params in ({},{'config':{}},{'config':'x'},{'config':dict(good,fire_us=1000)},{'config':dict(good,hold_s=True)},
                           {'config':dict(good,low_us='1000')},{'config':dict(good,hold_s=float('nan'))},{'config':good,'command':'rm -rf ~'}):
                with self.assertRaises(ValueError):panel.submit({'action':'interceptor_trigger_save','params':params})
            calls=[];replies=[{'ok':True,'applied':True,'options':dict(good,source='file')},ValueError('飞控已解锁：上锁后再改击发参数')]
            class Board:
                def exec_json(self,command,payload,timeout=30):
                    calls.append((command,payload,timeout));reply=replies.pop(0)
                    if isinstance(reply,Exception):raise reply
                    return reply
            panel.board=Board();panel.connected=True
            def run(rid,config):
                jid=panel.submit({'action':'interceptor_trigger_save','request_id':rid,'params':{'config':config}})
                for _ in range(100):
                    job=panel.jobs.get(jid)
                    if job['state'] in ('complete','error'):return job
                    time.sleep(0.02)
                return job
            job=run('trigger1',good)
            self.assertEqual(job['state'],'complete',job.get('error'));self.assertTrue(job['result']['applied'])
            job=run('trigger2',{'hold_s':1.5})
            self.assertEqual(job['state'],'error');self.assertIn('飞控已解锁',job.get('error') or '')
            self.assertEqual(calls,[(dashboard.INTERCEPTOR_TRIGGER_SAVE_CMD,{'config':good},20),(dashboard.INTERCEPTOR_TRIGGER_SAVE_CMD,{'config':{'hold_s':1.5}},20)])
            self.assertEqual(dashboard.INTERCEPTOR_TRIGGER_SAVE_CMD,'python3 /home/uavlab/interceptor_ws/deploy/netgun_pwm_link.py params save')
        finally:panel.close()
if __name__=='__main__':unittest.main()
