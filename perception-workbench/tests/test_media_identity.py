import json
import os
import pathlib
import sys
import tempfile
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))
from media_library import MediaLibrary


class MediaIdentityTests(unittest.TestCase):
    def test_late_results_and_linked_preview_sort_with_original_scene(self):
        with tempfile.TemporaryDirectory() as tmp:
            library=MediaLibrary(pathlib.Path(tmp)/'scenes',pathlib.Path(tmp)/'previews')
            library.scene_names([{'session_id':'old','display_name':'旧场次','created_at_unix_s':1800000000},
                                 {'session_id':'new','display_name':'新场次','created_at_unix_s':1800000100}])
            for sid in ('old','new'):
                folder=library.registry.folder(sid,create=True)/'exports/session';folder.mkdir(parents=True)
                (folder/'rgb_video.view.mp4').write_bytes(b'rgb')
            old=library.registry.folder('old')
            annotation=old/'exports/annotated/detected.view.mp4';annotation.parent.mkdir();annotation.write_bytes(b'annotation')
            annotation.with_suffix('.json').write_text(json.dumps({'model_family':'general','input_nchw':[1,3,384,640],'confidence':.35,'source_segment':'session'}))
            preview=library.registry.create_preview('preview_old','rgb',1800009999,'old')
            (preview/'segment_0001.mp4').write_bytes(b'preview');library.registry.finish_preview('preview_old')
            os.utime(annotation,(1900000000,1900000000))
            rows=library.list()
            self.assertEqual(rows[0]['session_id'],'new')
            self.assertTrue(all(r['scene_created_at_unix_s']==1800000000 for r in rows[1:]))
            detected=next(r for r in rows if r['kind']=='annotated')
            self.assertIn('通用检测',detected['name']);self.assertIn('640 × 384',detected['name'])
            self.assertIn('0.35',detected['name'])
            self.assertIn('原视频',rows[0]['name'])

    def test_legacy_annotation_model_names_and_depth_are_distinct(self):
        with tempfile.TemporaryDirectory() as tmp:
            library=MediaLibrary(tmp);library.scene_names([{'session_id':'scene','display_name':'场次','created_at_unix_s':1800000000}])
            folder=library.registry.folder('scene',create=True)/'exports'
            for relative in ('annotated/annotated_exp_nx16_fp16_old.view.mp4','annotated/annotated_glad-384x640_20261016_175039.view.mp4','segment_1/depth_video.view.mp4'):
                p=folder/relative;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b'video')
            names=[r['name'] for r in library.list()]
            self.assertTrue(any('自训检测' in n for n in names))
            self.assertTrue(any('通用检测' in n for n in names))
            self.assertTrue(any('深度视频' in n for n in names))
            glad=next(n for n in names if '通用检测' in n)
            self.assertIn('640 × 384',glad)
            self.assertIn('17:50:39',glad)


if __name__=='__main__':unittest.main()
