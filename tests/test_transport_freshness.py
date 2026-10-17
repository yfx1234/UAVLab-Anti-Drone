import pathlib,sys,threading,unittest
from unittest.mock import patch
from types import SimpleNamespace
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from frame_receiver import FrameReceiver
from dashboard import Panel
sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
import perception_supervisor as supervisor
import contextlib

class TransportFreshnessTests(unittest.TestCase):
    def test_replayed_action_keeps_result_but_refreshes_state(self):
        old={'ok':True,'result':{'did_work':True},'state':{'sample_monotonic_s':100}}
        current={'sample_monotonic_s':110,'boot_id':'boot'}
        with patch.object(supervisor,'state',return_value=current),patch.object(supervisor,'read_json',return_value={'repeat':old}),patch.object(supervisor,'state_lock',return_value=contextlib.nullcontext()):
            reply=supervisor.dispatch({'action':'stop_capture','request_id':'repeat'})
        self.assertEqual(reply['state'],current)
        self.assertIs(reply['result'],old['result'])
        self.assertEqual(old['state']['sample_monotonic_s'],100)
    def test_tcp_backlog_age_uses_board_clock_not_receipt(self):
        receiver=FrameReceiver.__new__(FrameReceiver)
        receiver.clock_lock=threading.Lock();receiver.clock_reference=None
        receiver.observe_board_clock({'boot_id':'boot','sample_monotonic_s':100},10)
        packet={'boot_id':'boot','monotonic_s':99.8,'source_age_s':.02}
        with patch('frame_receiver.time.monotonic',return_value=15):
            aged=receiver.age_metadata(packet)
        self.assertAlmostEqual(aged['source_age_s'],5.2)
        self.assertEqual(packet['source_age_s'],.02)
        with patch('frame_receiver.time.monotonic',return_value=15):
            self.assertIsNone(receiver.age_metadata(dict(packet,boot_id='old')))

    def test_slow_rpc_ages_hardware_node_and_transfer_state(self):
        p=Panel.__new__(Panel);p.lock=threading.RLock();p.connected=True;p.error=None
        p.receiver=SimpleNamespace(observe_board_clock=lambda *args:None)
        state={'last_status_age_s':.1,'hardware':{'sample_age_s':1},'pid':None}
        with patch('dashboard.time.monotonic',return_value=17):
            p.observe_state(state,10)
            self.assertAlmostEqual(p.transfer_state()['last_status_age_s'],7.1)
        self.assertEqual(p.state['hardware']['sample_age_s'],8)
        self.assertEqual(state['hardware']['sample_age_s'],1)

if __name__=='__main__':unittest.main()
