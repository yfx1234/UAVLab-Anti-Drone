import pathlib,sys,unittest,tempfile,hashlib,stat,contextlib
from types import SimpleNamespace
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'panel'))
from session_manager import SessionManager
class TransferRegression(unittest.TestCase):
    def fixture(self):
        data=b'0123456789abcdef'*131072;self.polls=0
        class File:
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def seek(self,n):pass
            def readv(self,reads):return (data[n:n+length] for n,length in reads)
        class FTP:
            def lstat(self,path):return SimpleNamespace(st_mode=stat.S_IFREG)
            def open(self,*args):return File()
        class Board:
            @contextlib.contextmanager
            def connection(self):yield SimpleNamespace(open_sftp=lambda:FTP())
            def call(board,action,params=None):
                if action=='list_sessions':return [{'session_id':'fixture','summary':{'state':'complete','tail_metadata_may_arrive':False},'files':[{'name':'exports/segment_1/rgb_video.mp4','bytes':len(data)}]}]
                if action=='session_checksum':return {'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
                if action=='status':
                    self.polls+=1
                    return {'pid':None,'camera_state':'stopped'} if self.polls==1 else {'pid':123,'camera_state':'running'}
                raise AssertionError(action)
        return Board(),data
    def test_status_transition_then_cancel_resume_verified_selected_video(self):
        board,data=self.fixture();updates=[];sleeps=[];clock=iter(range(10000))
        with tempfile.TemporaryDirectory() as tmp,patch('session_manager.time.monotonic',lambda:next(clock)),patch('session_manager.time.sleep',sleeps.append):
            manager=SessionManager(board,tmp)
            with self.assertRaisesRegex(ValueError,'取消'):
                manager.download('fixture',updates.append,lambda:bool(updates and updates[-1]['downloaded_bytes']>=700000),'rgb')
            self.assertIn('停止后高速',[p['speed_mode'] for p in updates]);self.assertIn('采集中限速',[p['speed_mode'] for p in updates])
            part=next(pathlib.Path(tmp).rglob('*.part'));offset=part.stat().st_size;self.assertGreater(offset,0)
            result=manager.download('fixture',kind='rgb');self.assertEqual(result['kind'],'rgb')
            self.assertEqual(next(pathlib.Path(tmp).rglob('*.mp4')).read_bytes(),data);self.assertFalse(list(pathlib.Path(tmp).rglob('*.part')))
    def test_insufficient_local_disk_rejects_without_partial_video(self):
        board,data=self.fixture()
        with tempfile.TemporaryDirectory() as tmp,patch('session_manager.shutil.disk_usage',return_value=SimpleNamespace(free=512)):
            with self.assertRaisesRegex(ValueError,'空间不足'):SessionManager(board,tmp).download('fixture',kind='rgb')
            self.assertFalse(list(pathlib.Path(tmp).rglob('*.part')))
if __name__=='__main__':unittest.main()
