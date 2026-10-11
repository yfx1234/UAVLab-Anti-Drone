import importlib.util
import json
import pathlib
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'panel'),str(ROOT/'board/zed_tracking/tools')]

class PlatformTests(unittest.TestCase):
    def test_frame_rollover_does_not_pair_old_metadata_with_new_image(self):
        from frame_receiver import FrameCache
        cache=FrameCache(capacity=4)
        cache.put({'frame_key':'run_1','targets':[{'class_id':0}]},b'first')
        cache.put({'frame_key':'run_2','targets':[]},b'second')
        self.assertEqual(cache.get('run_1').jpeg,b'first')
        self.assertEqual(cache.latest().metadata['frame_key'],'run_2')
        for n in range(3,8):cache.put({'frame_key':f'run_{n}','targets':[]},b'new')
        self.assertIsNone(cache.get('run_1'))

    def test_duplicate_frame_does_not_refresh_freshness(self):
        from frame_receiver import FrameCache
        cache=FrameCache();cache.put({'frame_key':'run_1'},b'first')
        original=cache.latest().received_at
        time.sleep(.02);cache.put({'frame_key':'run_1'},b'first')
        self.assertEqual(cache.latest().received_at,original)

    def test_wire_reader_rejects_oversized_and_truncated_packets(self):
        from frame_receiver import read_packet
        for blob in [struct.pack('!II',65537,1),struct.pack('!II',2,3)+b'{}x']:
            a,b=socket.socketpair()
            try:
                a.sendall(blob);a.shutdown(socket.SHUT_WR)
                with self.assertRaises((ValueError,EOFError)):read_packet(b)
            finally:a.close();b.close()

    def test_old_run_request_cannot_stop_new_owner(self):
        from perception_supervisor import validate_expectation,Conflict
        state={'boot_id':'boot','run_id':'new','revision':8}
        with self.assertRaises(Conflict):
            validate_expectation(state,{'expected_boot_id':'boot','expected_run_id':'old','expected_revision':8})
        validate_expectation(state,{'expected_boot_id':'boot','expected_run_id':'new','expected_revision':8})

    def test_model_version_changes_when_same_filename_gets_new_weights(self):
        import perception_supervisor as supervisor
        with tempfile.TemporaryDirectory() as folder:
            root=pathlib.Path(folder);workspace=root/'ros2_ws';workspace.mkdir()
            weights=root/'weights/candidates/test';weights.mkdir(parents=True);engine=weights/'exp_opt3.engine'
            original=supervisor.WS;supervisor.WS=workspace
            try:
                engine.write_bytes(b'first_engine');first=supervisor.models()[0]['model_id']
                engine.write_bytes(b'second_engine');second=supervisor.models()[0]['model_id']
                self.assertNotEqual(first,second)
            finally:supervisor.WS=original

    def test_unsafe_paths_and_flight_actions_are_rejected(self):
        from perception_supervisor import session_path,validate_action
        with tempfile.TemporaryDirectory() as folder:
            for name in ['../other','/tmp/escape','x/../y']:
                with self.assertRaises(ValueError):session_path(pathlib.Path(folder),name)
        with self.assertRaises(ValueError):validate_action('chain_start_live',{})
        with self.assertRaises(ValueError):validate_action('start_capture',{'profile':'rgb','command':'rm'})

    def test_jobs_do_not_block_status_and_repeat_id_runs_once(self):
        from job_manager import JobManager
        jobs=JobManager();calls=[];gate=threading.Event()
        def work():calls.append(1);gate.wait(1);return {'done':True}
        a=jobs.submit('one',work);b=jobs.submit('one',work)
        self.assertEqual(a,b)
        self.assertIn(jobs.get(a)['state'],('queued','running'))
        gate.set()
        for _ in range(100):
            if jobs.get(a)['state']=='complete':break
            time.sleep(.005)
        self.assertEqual(jobs.get(a)['result'],{'done':True});self.assertEqual(calls,[1])

    def test_group_split_keeps_session_together(self):
        from export_session import grouped_split
        groups={f'session{i}':list(range(10)) for i in range(10)}
        splits=grouped_split(groups)
        flattened=[v for values in splits.values() for v in values]
        self.assertEqual(set(flattened),set(groups));self.assertEqual(len(flattened),10)
        self.assertTrue(splits['validation'] and splits['test'])

    def test_dataset_index_keeps_unknown_annotations_and_session_groups(self):
        from export_session import prepare_dataset_index
        with tempfile.TemporaryDirectory() as folder:
            root=pathlib.Path(folder)
            for i in range(3):
                session=root/f'session_{i}';out=session/'exports/segment_1';out.mkdir(parents=True)
                (session/'session_summary.json').write_text(json.dumps({'state':'complete'}))
                image=out/'frame_0.png';image.write_bytes(b'image fixture')
                (out/'mapping.jsonl').write_text(json.dumps({'file':str(image),'stamp_ns':100+i,'svo_position':0})+'\n')
            index=prepare_dataset_index(root)
            self.assertEqual(len(index['images']),3)
            self.assertEqual({r['annotation_status'] for r in index['images']},{'unlabeled'})
            self.assertEqual(len(index['splits']['train']),1)
            self.assertEqual(len(index['splits']['validation']),1)
            self.assertEqual(len(index['splits']['test']),1)

    def test_preview_recording_finishes_readable_video_and_fixed_view(self):
        from preview_recorder import PreviewRecorder
        from frame_receiver import PreviewPacket
        from PIL import Image
        import io
        image=Image.new('RGB',(64,36),(20,30,40));data=io.BytesIO();image.save(data,'JPEG')
        with tempfile.TemporaryDirectory() as folder:
            rec=PreviewRecorder(pathlib.Path(folder));rec.start('rgb')
            for i in range(8):
                rec.consume(PreviewPacket({'frame_key':f'r_{i}','image_width':64,'image_height':36,'targets':[]},data.getvalue(),time.monotonic()))
                time.sleep(.02)
            result=rec.stop()
            self.assertEqual(result['view_mode'],'rgb')
            self.assertGreater(result['frames'],0)
            self.assertTrue(pathlib.Path(result['files'][0]).is_file())
            import imageio_ffmpeg
            reader=imageio_ffmpeg.read_frames(result['files'][0]);header=next(reader)
            self.assertEqual(header['size'],(64,36));next(reader);reader.close()

    def test_preview_recording_stops_before_local_disk_fills(self):
        from preview_recorder import PreviewRecorder
        from frame_receiver import PreviewPacket
        from unittest.mock import patch
        from PIL import Image
        import io
        import collections
        image=Image.new('RGB',(64,36));data=io.BytesIO();image.save(data,'JPEG')
        usage=collections.namedtuple('usage','total used free')
        with tempfile.TemporaryDirectory() as folder:
            rec=PreviewRecorder(folder);rec.start('rgb')
            with patch('shutil.disk_usage',return_value=usage(2*1024**3,2*1024**3,512)):
                rec.consume(PreviewPacket({'frame_key':'disk_1','targets':[]},data.getvalue(),time.monotonic()))
                for _ in range(50):
                    if rec.error:break
                    time.sleep(.02)
                rec.stop()
            self.assertIn('磁盘',rec.error or '')
            self.assertEqual(rec.frames,0)

    def test_http_rejects_flight_action_and_serves_keyed_old_frame(self):
        import dashboard
        import http.server
        import urllib.request
        import urllib.error
        panel=dashboard.Panel()
        server=http.server.ThreadingHTTPServer(('127.0.0.1',0),dashboard.make_handler(panel))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        url=f'http://127.0.0.1:{server.server_port}'
        try:
            request=urllib.request.Request(url+'/api/action',data=json.dumps({'action':'chain_start_live'}).encode(),
                headers={'Host':'127.0.0.1:8767','Origin':'http://127.0.0.1:8767','X-Panel-Token':panel.token,'Content-Type':'application/json'})
            with self.assertRaises(urllib.error.HTTPError) as error:urllib.request.urlopen(request)
            self.assertEqual(error.exception.code,400)
            panel.receiver.cache.put({'frame_key':'r_1'},b'first')
            panel.receiver.cache.put({'frame_key':'r_2'},b'second')
            req=urllib.request.Request(url+'/api/frame/r_1.jpg',headers={'Host':'127.0.0.1:8767'})
            self.assertEqual(urllib.request.urlopen(req).read(),b'first')
        finally:server.shutdown();server.server_close();panel.close()

    def test_session_checksum_rejects_symlink_outside_session(self):
        import perception_supervisor as supervisor
        if sys.platform=='win32':self.skipTest('symlink permission varies; run on the Linux board')
        with tempfile.TemporaryDirectory() as folder:
            root=pathlib.Path(folder);session=root/'session';session.mkdir();(root/'private').write_text('secret')
            (session/'link').symlink_to(root/'private')
            prior=supervisor.RECORDS;supervisor.RECORDS=root
            try:
                with self.assertRaises(ValueError):supervisor.dispatch({'action':'session_checksum','params':{'session_id':'session','file':'link'}})
            finally:supervisor.RECORDS=prior

if __name__=='__main__':unittest.main()
