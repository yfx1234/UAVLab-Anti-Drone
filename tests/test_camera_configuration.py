import pathlib
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'board/zed_tracking/tools'))
import perception_supervisor as supervisor

class CameraConfigurationTests(unittest.TestCase):
    def test_zed_x_modes_accept_exact_sensor_pairs(self):
        for mode, rates in {'HD1080': (15,30,60), 'HD1200': (15,30,60), 'SVGA': (15,30,60,120)}.items():
            for rate in rates:
                supervisor.validate_action('start_capture', {'profile':'rgb','camera_resolution':mode,'capture_fps':rate})

    def test_invalid_or_unrelated_camera_pairs_rejected_before_launch(self):
        for mode, rate in [('HD1080',120),('HD1200',120),('HD720',30),('HD2K',15),('SVGA',0),('SVGA',True),('SVGA',30.0),('SVGA','30')]:
            with self.subTest(mode=mode, rate=rate), self.assertRaises(ValueError):
                supervisor.validate_action('start_capture', {'profile':'rgb','camera_resolution':mode,'capture_fps':rate})
        supervisor.validate_action('start_capture', {'profile':'rgb'})

    def test_selected_mode_reaches_node_without_shell_interpolation(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = pathlib.Path(directory); executable = folder/'node'; executable.touch()
            with mock.patch.object(supervisor,'ROOT',folder), mock.patch.object(supervisor,'EXE',executable), mock.patch.object(supervisor,'owns',return_value=False), mock.patch.object(supervisor,'proc_start',return_value='100'), mock.patch.object(supervisor.subprocess,'Popen',return_value=mock.Mock(pid=50)) as popen:
                supervisor.start({}, {'profile':'rgb','camera_resolution':'SVGA','capture_fps':120})
                command = popen.call_args.args[0][-1]
                self.assertIn('camera_resolution:=SVGA',command)
                self.assertIn('capture_fps:=120',command)

    def test_rgb_binds_available_model_without_starting_detector(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=pathlib.Path(directory); executable=folder/'node';executable.touch();state={}
            with mock.patch.object(supervisor,'ROOT',folder),mock.patch.object(supervisor,'EXE',executable),mock.patch.object(supervisor,'owns',return_value=False),mock.patch.object(supervisor,'proc_start',return_value='100'),mock.patch.object(supervisor,'model_by_id',return_value={'model_id':'existing'}),mock.patch.object(supervisor,'stage_model',return_value='/safe/engine'),mock.patch.object(supervisor.subprocess,'Popen',return_value=mock.Mock(pid=50)) as popen:
                supervisor.start(state,{'profile':'rgb'})
                self.assertEqual(state['model_id'],'existing')
                command=popen.call_args.args[0][-1]
                self.assertIn('profile:=rgb',command)
                self.assertIn('model_id:=existing',command)

if __name__ == '__main__': unittest.main()
