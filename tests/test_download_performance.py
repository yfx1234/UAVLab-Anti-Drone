"""Transfer behavior at slow external I/O boundaries, with real local files/hashes."""
import contextlib
import hashlib
import pathlib
import stat
import sys
import tempfile
import threading
import unittest
import copy
from types import SimpleNamespace
from unittest.mock import patch

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from session_manager import SessionManager
from dashboard import Panel

class Clock:
    def __init__(self):self.now=10.0
    def monotonic(self):return self.now
    def sleep(self,seconds):self.now+=seconds

def fixture(clock=None,source_hash=None):
    data=b'0123456789abcdef'*4096
    class File:
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def seek(self,n):pass
        def readv(self,reads):
            for n,length in reads:
                if clock:clock.now+=.1
                yield data[n:n+length]
    class FTP:
        def lstat(self,path):return SimpleNamespace(st_mode=stat.S_IFREG)
        def open(self,*args):return File()
    class Board:
        def __init__(self):self.calls=[];self.channels=[]
        @contextlib.contextmanager
        def connection(self):yield SimpleNamespace(open_sftp=lambda:FTP())
        def call(self,action,params=None):
            self.calls.append(action)
            if action=='list_sessions':return [{'session_id':'fixture','display_name':'fixture',
                'summary':{'state':'complete','tail_metadata_may_arrive':False},
                'files':[{'name':'exports/segment_1/rgb_video.mp4','bytes':len(data)}]}]
            if action=='session_checksum':return {'sha256':source_hash or hashlib.sha256(data).hexdigest(),'bytes':len(data)}
            if action=='status':return {'pid':123,'camera_state':'running'}
            raise AssertionError(action)
        def call_on(self,client,action,params=None):
            self.channels.append(action);return self.call(action,params)
    return Board(),data

class DownloadPerformanceTests(unittest.TestCase):
    def healthy_preview(self):
        return {'pid':123,'camera_state':'running','profile':'rgb','last_status_age_s':.2,'pipeline':{'profile':'rgb',
            'depth_enabled':False,'detector_enabled':False,'record_state':'idle','capture_hz':15,
            'mem_available_kib':4*1024*1024,'max_temperature_c':56,'last_frame_age_s':.1}}

    def test_preview_acceleration_requires_every_health_and_idle_condition(self):
        good=self.healthy_preview()
        self.assertEqual(SessionManager.transfer_limit(good),768*1024)
        cases=[('profile','perception'),('camera_state','unresponsive'),('pid',None),
            ('last_status_age_s',1.5001),('last_status_age_s',-1),('last_status_age_s',float('nan')),
            ('last_status_age_s',float('inf')),
            ('pipeline.profile','perception'),('pipeline.depth_enabled',True),('pipeline.detector_enabled',True),
            ('pipeline.record_state','recording'),('pipeline.record_state','finalizing'),
            ('pipeline.capture_hz',12.9),('pipeline.mem_available_kib',2*1024*1024-1),
            ('pipeline.max_temperature_c',65),('pipeline.last_frame_age_s',1)]
        for key,value in cases:
            state=copy.deepcopy(good)
            if key.startswith('pipeline.'):state['pipeline'][key.split('.')[1]]=value
            else:state[key]=value
            with self.subTest(key=key,value=value):self.assertEqual(SessionManager.transfer_limit(state),256*1024)
        incomplete=copy.deepcopy(good);del incomplete['pipeline']['depth_enabled']
        self.assertEqual(SessionManager.transfer_limit(incomplete),256*1024)
        no_node_age=copy.deepcopy(good);del no_node_age['last_status_age_s']
        self.assertEqual(SessionManager.transfer_limit(no_node_age),256*1024)

    def test_cached_node_age_increases_even_when_rpc_snapshot_has_not_expired(self):
        panel=Panel.__new__(Panel);panel.lock=threading.RLock();panel.connected=True;panel.error=None
        panel.state=self.healthy_preview();panel.state_observed_at=10
        with patch('dashboard.time.monotonic',return_value=11.4):
            self.assertEqual(SessionManager.transfer_limit(panel.transfer_state()),256*1024)
        self.assertEqual(panel.state['last_status_age_s'],.2,'reading cache must not mutate the stored board snapshot')

    def test_starting_detection_drops_preview_acceleration_before_next_chunk(self):
        clock=Clock();board,data=fixture(clock);updates=[];states=[]
        good=self.healthy_preview();detecting=copy.deepcopy(good);detecting['pipeline']['detector_enabled']=True
        states.extend([good,detecting])
        with tempfile.TemporaryDirectory() as tmp,patch('session_manager.time.monotonic',clock.monotonic),patch('session_manager.time.sleep',clock.sleep):
            manager=SessionManager(board,tmp,status_provider=lambda:states.pop(0) if states else detecting)
            manager.download('fixture',updates.append,kind='rgb')
        transfers=[p for p in updates if p.get('phase_code')=='transfer']
        self.assertEqual([p['limit_bytes_s'] for p in transfers],[786432,262144])
        self.assertEqual([p['speed_mode'] for p in transfers],['预览中加速','采集中限速'])

    def test_cache_avoids_duplicate_status_connections_and_new_file_double_hash(self):
        board,data=fixture();updates=[]
        with tempfile.TemporaryDirectory() as tmp,patch('session_manager.time.sleep'):
            manager=SessionManager(board,tmp)
            manager.status_provider=lambda:{'pid':123,'camera_state':'running'}
            result=manager.download('fixture',updates.append,kind='rgb')
            self.assertEqual(next(pathlib.Path(tmp).rglob('*.mp4')).read_bytes(),data)
        self.assertEqual(board.calls.count('status'),0,'fresh Panel state already polls the board')
        self.assertEqual(board.calls.count('session_checksum'),1,'first download needs final source hash')
        self.assertIn('session_checksum',board.channels,'hash should share existing SSH transport')
        self.assertIn('transfer_s',result['timings_s'])
        self.assertIn('校验',[p.get('stage') for p in updates])

    def test_limit_counts_time_spent_receiving_bytes_instead_of_adding_full_sleep(self):
        clock=Clock();board,data=fixture(clock);updates=[]
        with tempfile.TemporaryDirectory() as tmp,patch('session_manager.time.monotonic',clock.monotonic),patch('session_manager.time.sleep',clock.sleep):
            result=SessionManager(board,tmp).download('fixture',updates.append,kind='rgb')
        # Two 32 KiB reads each spend 0.1s in the network. At 256 KiB/s,
        # each should finish at 0.125s, not 0.225s (read + full sleep).
        self.assertAlmostEqual(result['elapsed_s'],.25,places=6)
        self.assertEqual(updates[-2]['speed_mode'],'采集中限速')

    def test_cache_failure_keeps_active_limit_and_corrupted_partial_unpublished(self):
        clock=Clock();board,data=fixture(clock,source_hash='0'*64)
        with tempfile.TemporaryDirectory() as tmp,patch('session_manager.time.monotonic',clock.monotonic),patch('session_manager.time.sleep',clock.sleep):
            manager=SessionManager(board,tmp)
            def unavailable():raise OSError('poll unavailable')
            manager.status_provider=unavailable
            with self.assertRaisesRegex(ValueError,'校验失败'):manager.download('fixture',kind='rgb')
            self.assertFalse(list(pathlib.Path(tmp).rglob('*.mp4')))
            self.assertEqual(next(pathlib.Path(tmp).rglob('*.part')).read_bytes(),data)
            self.assertAlmostEqual(clock.now-10,.25)

    def test_panel_stale_disconnected_or_failed_state_never_enables_unlimited_transfer(self):
        panel=Panel.__new__(Panel);panel.lock=threading.RLock();panel.connected=True;panel.error=None
        panel.state={'pid':None,'camera_state':'stopped'};panel.state_observed_at=10
        with patch('dashboard.time.monotonic',return_value=11):
            self.assertEqual(SessionManager.transfer_limit(panel.transfer_state()),0)
        with patch('dashboard.time.monotonic',return_value=14):
            self.assertEqual(SessionManager.transfer_limit(panel.transfer_state()),256*1024)
        panel.state_observed_at=14;panel.error='SSH unavailable'
        with patch('dashboard.time.monotonic',return_value=14):
            self.assertEqual(SessionManager.transfer_limit(panel.transfer_state()),256*1024)
        panel.error=None;panel.connected=False
        with patch('dashboard.time.monotonic',return_value=14):
            self.assertEqual(SessionManager.transfer_limit(panel.transfer_state()),256*1024)

if __name__=='__main__':unittest.main()
