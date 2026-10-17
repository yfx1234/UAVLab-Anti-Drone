import sys,pathlib,threading,time,unittest
from types import SimpleNamespace
R=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'panel'))
from dashboard import Panel
from job_manager import JobManager
class SceneBusyTests(unittest.TestCase):
 def test_delete_rejected_during_download_and_idempotent_request_reuses_job(self):
  gate=threading.Event()
  p=Panel.__new__(Panel);p.lock=threading.RLock();p.connected=True;p.jobs=JobManager();p.scene_jobs={};p.progress={};p.cancelled=set()
  p.board=SimpleNamespace(call=lambda *a,**k:None)
  p.sessions=SimpleNamespace(select_files=lambda *a:None,download=lambda *a:(gate.wait(2) or {'files':[],'folder':'fixture'}))
  try:
   data={'action':'download_asset','request_id':'download1','params':{'session_id':'fixture','kind':'stereo'}}
   jid=p.submit(data);self.assertEqual(p.submit(data),jid)
   with self.assertRaisesRegex(ValueError,'正在'):p.submit({'action':'delete_session','request_id':'delete1','params':{'session_id':'fixture','confirmation':'fixture'}})
  finally:gate.set();p.jobs.pool.shutdown()
  self.assertEqual(p.scene_jobs,{})
if __name__=='__main__':unittest.main()
