import sys,pathlib,unittest,threading,time
from types import SimpleNamespace
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'panel'))
from dashboard import Panel
from job_manager import JobManager
class CachedWorkflowTests(unittest.TestCase):
    def test_cached_rgb_bypasses_camera_export_and_transfers(self):
        calls=[]
        class Board:
            def call(self,action,params=None):
                calls.append(action)
                if action=='ready_media':return {'ready':True}
                raise AssertionError('ready RGB must not invoke idle-only export')
        panel=Panel.__new__(Panel);panel.lock=threading.RLock();panel.connected=True;panel.jobs=JobManager();panel.board=Board();panel.progress={};panel.cancelled=set()
        panel.sessions=SimpleNamespace(select_files=lambda files,kind:None,download=lambda *args:{'folder':'fixture','kind':'rgb','files':[]})
        panel.media=SimpleNamespace(prepare=lambda result,cancelled:result)
        jid=panel.submit({'action':'download_asset','request_id':'cached_test','params':{'session_id':'fixture','kind':'rgb'}})
        for _ in range(50):
            job=panel.jobs.get(jid)
            if job['state'] in ('complete','error'):break
            time.sleep(.01)
        self.assertEqual(job['state'],'complete',job);self.assertEqual(calls,['ready_media']);panel.jobs.pool.shutdown()
if __name__=='__main__':unittest.main()
