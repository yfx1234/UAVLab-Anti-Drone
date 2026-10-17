"""Local deletion must affect only the confirmed, listed playback file."""
import pathlib
import sys
import tempfile
import threading
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from media_library import MediaLibrary
from dashboard import Panel
from job_manager import JobManager


class LocalMediaDeleteTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=pathlib.Path(self.tmp.name)/'sessions'
        self.video=self.root/'session_test/exports/segment_1/rgb_video.view.mp4'
        self.video.parent.mkdir(parents=True)
        self.video.write_bytes(b'playback')
        self.source=self.video.with_name('rgb_video.mp4')
        self.source.write_bytes(b'original')
        self.other=self.video.with_name('depth_video.view.mp4')
        self.other.write_bytes(b'depth')
        self.media=MediaLibrary(self.root,pathlib.Path(self.tmp.name)/'previews')
        row=next(r for r in self.media.list() if r['kind']=='rgb')
        self.relative=row['relative'];self.video=pathlib.Path(row['path'])
        self.source=self.video.with_name('rgb_video.mp4');self.other=self.video.with_name('depth_video.view.mp4')

    def test_deletes_only_confirmed_playback_and_removes_it_from_list(self):
        result=self.media.delete(self.relative,self.relative)
        self.assertTrue(result['deleted'])
        self.assertFalse(self.video.exists())
        self.assertEqual(self.source.read_bytes(),b'original')
        self.assertEqual(self.other.read_bytes(),b'depth')
        self.assertEqual([r['path'] for r in self.media.list()],[str(self.other)])

    def test_rejects_mismatched_confirmation_unlisted_and_traversal(self):
        source_relative=self.source.relative_to(self.root).as_posix()
        for relative,confirmation in [(self.relative,'another'),('../outside.mp4','../outside.mp4'),(source_relative,source_relative)]:
            with self.subTest(relative=relative),self.assertRaises(ValueError):
                self.media.delete(relative,confirmation)
        self.assertTrue(self.video.exists())
        self.assertTrue(self.source.exists())

    def test_active_preview_rejected_then_finished_preview_can_be_deleted(self):
        folder=self.media.registry.create_preview('preview_test','rgb',1,'session_test')
        video=folder/'preview.mp4';video.write_bytes(b'preview')
        relative='preview_videos/'+folder.name+'/preview.mp4'
        with self.assertRaises(ValueError):self.media.delete(relative,relative)
        self.assertTrue(video.exists())
        self.media.registry.finish_preview('preview_test')
        self.media.delete(relative,relative)
        self.assertFalse(video.exists())

    def test_submit_uses_actual_scene_reservation_and_works_offline(self):
        p=Panel.__new__(Panel);p.lock=threading.RLock();p.connected=False
        p.jobs=JobManager();p.scene_jobs={'session_test':'download'};p.media=self.media
        self.addCleanup(p.jobs.pool.shutdown)
        data={'action':'delete_local_media','request_id':'local_delete','params':{'relative':self.relative,'confirmation':self.relative,'session_id':'unrelated'}}
        with self.assertRaisesRegex(ValueError,'正在'):p.submit(data)
        self.assertTrue(self.video.exists())
        p.scene_jobs.clear()
        jid=p.submit(data)
        p.jobs.pool.shutdown()
        self.assertEqual(p.jobs.get(jid)['state'],'complete',p.jobs.get(jid))
        self.assertEqual(p.submit(data),jid)
        self.assertFalse(self.video.exists())
        self.assertEqual(p.scene_jobs,{})

    def test_annotation_reserves_selected_source_even_when_left_scene_differs(self):
        p=Panel.__new__(Panel);p.lock=threading.RLock();p.connected=True;p.media=self.media
        p.jobs=JobManager();p.scene_jobs={'session_test':'download'}
        self.addCleanup(p.jobs.pool.shutdown)
        with self.assertRaisesRegex(ValueError,'正在'):
            p.submit({'action':'annotate_session','params':{'relative':self.relative,'session_id':'unrelated'}})


if __name__=='__main__':unittest.main()
