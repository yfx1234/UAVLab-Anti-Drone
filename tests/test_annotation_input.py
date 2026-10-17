import hashlib
import importlib.util
import pathlib
import tempfile
import unittest
from unittest.mock import patch

ROOT=pathlib.Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('annotation_input',ROOT/'board/zed_tracking/tools/annotate_session.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class AnnotationInputTests(unittest.TestCase):
    def test_input_job_reads_one_confirmed_file_and_rejects_escape_or_wrong_digest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp);job='a'*32;folder=root/job;folder.mkdir()
            source=folder/'source.mp4';source.write_bytes(b'exact selected video')
            digest=hashlib.sha256(source.read_bytes()).hexdigest()
            with patch.object(module,'JOBS_ROOT',str(root),create=True):
                self.assertTrue(callable(getattr(module,'input_job',None)),'Selected-file CLI input is missing')
                paths,output=module.input_job(job,digest)
                self.assertEqual(paths,[str(source)]);self.assertEqual(output,str(folder/'output.mp4'))
                for bad_job,bad_digest in [('../escape',digest),(job,'f'*64),(job,None)]:
                    with self.assertRaises(ValueError):module.input_job(bad_job,bad_digest)


if __name__=='__main__':unittest.main()
