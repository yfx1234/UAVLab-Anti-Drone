import pathlib,sys,unittest,tempfile,subprocess,json
ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'panel'))

class MediaWorkflowTests(unittest.TestCase):
    def test_pc_video_is_browser_playable_and_keeps_native_dimensions_frames(self):
        from media_library import MediaLibrary
        from preview_recorder import PreviewRecorder
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);video=root/'session_test/exports/segment_1/rgb_video.mp4';video.parent.mkdir(parents=True)
            ff=PreviewRecorder.ffmpeg()
            subprocess.run([ff,'-v','error','-f','lavfi','-i','testsrc=size=192x108:rate=15','-frames:v','8','-c:v','mpeg4',str(video)],check=True)
            result=MediaLibrary(root).prepare({'folder':str(root/'session_test'),'kind':'rgb','files':[{'file':'exports/segment_1/rgb_video.mp4'}]})
            self.assertEqual(len(result['media']),1)
            out=pathlib.Path(result['media'][0]['path'])
            probe=subprocess.run([ff,'-v','error','-i',str(out),'-f','rawvideo','-pix_fmt','rgb24','pipe:1'],stdout=subprocess.PIPE,check=True)
            self.assertEqual(len(probe.stdout),192*108*3*8)
            self.assertNotEqual(out,video,'preserve downloaded RGB original derivative')

if __name__=='__main__':unittest.main()
