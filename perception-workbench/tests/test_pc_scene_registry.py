import contextlib
import hashlib
import json
import pathlib
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'panel'))
from media_library import MediaLibrary


class PcSceneRegistryTests(unittest.TestCase):
    def scene(self, root, sid, stamp, name=None):
        folder = root / sid
        video = folder / 'exports/segment_1/rgb_video.view.mp4'
        video.parent.mkdir(parents=True)
        video.write_bytes(b'original-video-' + sid.encode())
        (folder / 'manifest.json').write_text(json.dumps({'created_at_unix_s': stamp, 'created_time_source': 'pc_reference_monotonic'}))
        return {'session_id': sid, 'display_name': name or sid, 'created_at_unix_s': stamp,
                'created_time_uncertain': False}, video

    def test_real_folder_rename_preserves_content_and_time_order_after_reload(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            old, first = self.scene(root, 'session_old', 1800000000, 'Z较早')
            new, second = self.scene(root, 'session_new', 1800000100, 'A较新')
            before = hashlib.sha256(first.read_bytes()).hexdigest()
            library = MediaLibrary(root)
            library.scene_names([old, new])
            self.assertEqual([r.get('session_id') for r in library.list()], ['session_new', 'session_old'])
            result = library.rename_scene('session_old', '中文时间 2026-10-12 10:20:30')
            renamed = pathlib.Path(result['folder'])
            self.assertIn('中文时间 2026-10-12 10-20-30', renamed.name)
            self.assertFalse((root / 'session_old').exists())
            self.assertEqual(hashlib.sha256((renamed / 'exports/segment_1/rgb_video.view.mp4').read_bytes()).hexdigest(), before)
            from scene_registry import SceneRegistry
            rows = MediaLibrary(root,registry=SceneRegistry(root)).list()
            self.assertEqual([r['session_id'] for r in rows], ['session_new', 'session_old'])
            self.assertEqual(rows[1]['display_name'], '中文时间 2026-10-12 10:20:30')
            self.assertEqual(rows[1]['created_at_unix_s'], 1800000000)

    def test_duplicate_and_windows_reserved_names_are_safe_and_unique(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp); library = MediaLibrary(root)
            a, _ = self.scene(root, 'session_a', 1800000000)
            b, _ = self.scene(root, 'session_b', 1800000100)
            library.scene_names([a, b])
            p1 = pathlib.Path(library.rename_scene('session_a', 'CON')['folder'])
            p2 = pathlib.Path(library.rename_scene('session_b', 'CON')['folder'])
            self.assertNotEqual(p1, p2)
            self.assertNotEqual(p1.stem.upper(), 'CON')
            safe = pathlib.Path(library.rename_scene('session_a', '../中文:*?\\..')['folder'])
            self.assertEqual(safe.parent, root.resolve())
            self.assertFalse(any(c in safe.name for c in '<>:"/\\|?*'))
            with self.assertRaises(ValueError): library.rename_scene('../outside', 'name')

    def test_existing_destination_is_not_overwritten(self):
        from scene_registry import SceneRegistry
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp); row,_=self.scene(root,'session_a',1800000000,'before')
            library=MediaLibrary(root);library.scene_names([row]);old=library.registry.folder('session_a')
            target=root/SceneRegistry.safe_folder('after','session_a');target.mkdir();(target/'keep.txt').write_text('do not overwrite')
            with self.assertRaisesRegex(ValueError,'已存在'):library.rename_scene('session_a','after')
            self.assertTrue((old/'exports/segment_1/rgb_video.view.mp4').is_file())
            self.assertEqual((target/'keep.txt').read_text(),'do not overwrite')

    def test_locked_folder_refusal_keeps_old_location_and_retry_works(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp); row, _ = self.scene(root, 'session_a', 1800000000, 'before')
            library = MediaLibrary(root); library.scene_names([row]); previous = library.list()[0]['relative']
            with patch.object(pathlib.Path, 'rename', side_effect=PermissionError('video is open')):
                with self.assertRaisesRegex(ValueError, '本机.*video is open'): library.rename_scene('session_a', 'after')
            self.assertTrue(library.path(previous).is_file())
            self.assertEqual(library.list()[0]['display_name'], 'after')
            result = library.rename_scene('session_a', 'after')
            self.assertIn('after', pathlib.Path(result['folder']).name)

    def test_legacy_local_only_scene_migrates_without_losing_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp); row, video = self.scene(root, 'session_legacy', 1800000000)
            (root / 'scene_names.json').write_text(json.dumps({'session_legacy': '保留的旧场次'}))
            library = MediaLibrary(root); rows = library.list()
            self.assertEqual(rows[0].get('display_name'), '保留的旧场次')
            self.assertEqual(rows[0]['created_at_unix_s'], 1800000000)
            self.assertIn('保留的旧场次', rows[0]['relative'])
            library.scene_names([])
            self.assertEqual(MediaLibrary(root).list()[0]['display_name'], '保留的旧场次')

    def test_selective_import_adopts_board_timestamp_even_when_uncertain(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);row,_=self.scene(root,'session_a',1800000000,'board')
            (root/'session_a/manifest.json').unlink()
            library=MediaLibrary(root);before=library.list()[0]['created_at_unix_s']
            self.assertNotEqual(before,1800000000)
            row['created_time_uncertain']=True;library.scene_names([row])
            actual=library.list()[0]
            self.assertEqual(actual['created_at_unix_s'],1800000000)
            self.assertTrue(actual['created_time_uncertain'])
            (library.path(actual['relative'])).touch()
            library.scene_names([row])
            self.assertEqual(library.list()[0]['created_at_unix_s'],1800000000)

    def test_first_board_metadata_sync_renames_already_imported_uuid_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);row,_=self.scene(root,'session_a',1800000000,'新的板上测试名称')
            (root/'scene_names.json').write_text(json.dumps({'session_a':'旧UTC名称'}))
            library=MediaLibrary(root);self.assertIn('旧UTC名称',library.list()[0]['relative'])
            library.scene_names([row]);actual=library.list()[0]
            self.assertIn('新的板上测试名称',actual['relative'])
            row['display_name']='外部改名只更新显示'
            library.scene_names([row])
            self.assertIn('新的板上测试名称',library.list()[0]['relative'])

    def test_rename_waits_for_registered_writer_and_segment_sort_is_numeric(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp); row, _ = self.scene(root, 'session_a', 1800000000, 'before')
            library = MediaLibrary(root); library.scene_names([row])
            registry = getattr(library, 'registry', None); self.assertIsNotNone(registry)
            entered = threading.Event(); finished = threading.Event()
            def rename():
                entered.set(); library.rename_scene('session_a', 'after'); finished.set()
            with registry.operation('session_a'):
                thread = threading.Thread(target=rename); thread.start(); entered.wait(1)
                self.assertFalse(finished.wait(.05))
            thread.join(2); self.assertTrue(finished.is_set())
            folder = pathlib.Path(library.rename_scene('session_a', 'after')['folder'])
            for n in (10, 2):
                p = folder / f'exports/segment_{n}/rgb_video.view.mp4'; p.parent.mkdir(); p.touch()
            self.assertEqual([r['segment'] for r in library.list()], ['segment_1', 'segment_2', 'segment_10'])

    def test_long_conversion_does_not_block_another_scene_list_or_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)
            a,_=self.scene(root,'session_a',1800000000,'A'); b,_=self.scene(root,'session_b',1800000100,'B')
            library=MediaLibrary(root); library.scene_names([a,b])
            folder=library.registry.folder('session_a'); source=folder/'exports/segment_1/rgb_video.mp4'; source.write_bytes(b'video')
            gate=threading.Event(); entered=threading.Event(); finished=threading.Event(); errors=[]
            def blocked_run(args,*rest):
                entered.set(); gate.wait(2); pathlib.Path(args[-1]).write_bytes(b'converted')
            def convert():
                try:library.prepare({'folder':str(folder),'session_id':'session_a','kind':'rgb','files':[{'file':'exports/segment_1/rgb_video.mp4'}]})
                except Exception as e:errors.append(e)
            with patch.object(library,'_run',side_effect=blocked_run):
                thread=threading.Thread(target=convert);thread.start();self.assertTrue(entered.wait(1))
                def read():
                    rows=library.list(); row=next(r for r in rows if r['session_id']=='session_b')
                    with library.open_file(row['relative']) as stream:self.assertEqual(stream.read(),b'original-video-session_b')
                    finished.set()
                reader=threading.Thread(target=read);reader.start()
                try:self.assertTrue(finished.wait(.5),'another scene must remain readable while conversion is blocked')
                finally:gate.set();thread.join(2);reader.join(2)
                self.assertFalse(errors,errors)

    def test_long_download_keeps_other_scene_and_preview_start_responsive(self):
        from session_manager import SessionManager
        from preview_recorder import PreviewRecorder
        from test_transfer_regression import TransferRegression
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)/'board';root.mkdir();preview=root.parent/'preview'
            row,_=self.scene(root,'session_b',1800000100,'B');library=MediaLibrary(root,preview);library.scene_names([row])
            board,_=TransferRegression().fixture();connection=board.connection
            gate=threading.Event();entered=threading.Event();finished=threading.Event();errors=[]
            @contextlib.contextmanager
            def blocked_connection():
                entered.set();gate.wait(2)
                with connection() as client:yield client
            board.connection=blocked_connection;manager=SessionManager(board,root,library.registry)
            def download():
                try:manager.download('fixture',kind='rgb')
                except Exception as e:errors.append(e)
            thread=threading.Thread(target=download);thread.start();self.assertTrue(entered.wait(1))
            def independent():
                rows=library.list();row=next(r for r in rows if r['session_id']=='session_b')
                with library.open_file(row['relative']) as stream:self.assertEqual(stream.read(),b'original-video-session_b')
                recorder=PreviewRecorder(preview,library.registry);recorder.start('rgb');recorder.stop();finished.set()
            reader=threading.Thread(target=independent);reader.start()
            try:self.assertTrue(finished.wait(.5),'download must not block another video or preview recorder')
            finally:gate.set();thread.join(3);reader.join(3)
            self.assertFalse(errors,errors)

    def test_linked_preview_uses_capture_time_and_blocks_move_while_active(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp) / 'board'; preview = pathlib.Path(tmp) / 'preview'; root.mkdir()
            row, _ = self.scene(root, 'session_a', 1800000000, 'before')
            library = MediaLibrary(root, preview); library.scene_names([row])
            registry = getattr(library, 'registry', None); self.assertIsNotNone(registry)
            folder = registry.create_preview('preview_a', 'rgb', 1800000200, 'session_a')
            (folder / 'segment_0001.mp4').touch()
            with self.assertRaisesRegex(ValueError, '预览录像'): library.rename_scene('session_a', 'after')
            registry.finish_preview('preview_a')
            result = library.rename_scene('session_a', 'after')
            rows = library.list()
            self.assertEqual(rows[0]['source_session_id'], 'session_a')
            self.assertEqual(rows[0]['created_at_unix_s'], 1800000200)
            self.assertIn('after', rows[0]['relative'])
            self.assertTrue(result['preview_folders'])

    def test_maximum_valid_scene_name_also_supports_linked_preview(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)/'board';preview=root.parent/'preview';root.mkdir()
            name='测试'*50;row,_=self.scene(root,'session_a',1800000000,name)
            library=MediaLibrary(root,preview);library.scene_names([row])
            folder=library.registry.create_preview('preview_a','rgb',1800000100,'session_a')
            (folder/'segment_0001.mp4').touch();library.registry.finish_preview('preview_a')
            result=library.rename_scene('session_a','命名'*50)
            self.assertTrue(result['ok']);self.assertTrue(pathlib.Path(result['preview_folders'][0]).is_dir())

    def test_panel_exposes_partial_rename_and_same_name_retry(self):
        from dashboard import Panel
        from job_manager import JobManager
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp); row,_=self.scene(root,'session_a',1800000000,'before')
            library=MediaLibrary(root); library.scene_names([row])
            class Board:
                def call(self,action,params,request,timeout):
                    return {'result':{'session_id':'session_a','display_name':params['name']},'state':{}}
            panel=Panel.__new__(Panel); panel.lock=threading.RLock(); panel.jobs=JobManager(); panel.connected=True
            panel.receiver=SimpleNamespace(observe_board_clock=lambda *args:None)
            panel.scene_jobs={}; panel.progress={}; panel.cancelled=set(); panel.board=Board(); panel.media=library
            def submit(rid):
                jid=panel.submit({'action':'rename_session','request_id':rid,'params':{'session_id':'session_a','name':'after'}})
                for _ in range(100):
                    job=panel.jobs.get(jid)
                    if job['state'] in ('complete','error'): return job
                    time.sleep(.01)
                self.fail('rename job did not finish')
            try:
                with patch.object(pathlib.Path,'rename',side_effect=PermissionError('video is open')):
                    result=submit('first')
                self.assertEqual(result['state'],'complete',result)
                self.assertFalse(result['result']['result'].get('local_sync',{}).get('ok',True))
                self.assertIn('video is open',result['result']['result']['local_sync']['error'])
                result=submit('retry'); self.assertTrue(result['result']['result']['local_sync']['ok'])
                self.assertIn('after',library.list()[0]['relative'])
            finally: panel.jobs.pool.shutdown()

    def test_preview_recorder_persists_actual_creation_and_source_before_frames(self):
        from preview_recorder import PreviewRecorder
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)/'board'; preview=pathlib.Path(tmp)/'preview'; root.mkdir()
            row,_=self.scene(root,'session_a',1800000000,'capture')
            library=MediaLibrary(root,preview); library.scene_names([row])
            recorder=PreviewRecorder(preview,registry=library.registry)
            start=recorder.start('rgb',source_session_id='session_a')
            try:
                self.assertEqual(start['source_session_id'],'session_a')
                self.assertLess(abs(start['created_at_unix_s']-time.time()),2)
                metadata=json.loads((pathlib.Path(start['folder'])/'preview.json').read_text(encoding='utf-8'))
                self.assertEqual(metadata['source_session_id'],'session_a')
            finally: recorder.stop()


if __name__ == '__main__': unittest.main()
