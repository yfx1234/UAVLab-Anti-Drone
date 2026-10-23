"""安装参数（以网枪中心为基准）：换算、校验、存储、启动脚本与面板换算一致性。"""
import importlib.util
import io
import json
import math
import os
import pathlib
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOL = ROOT / 'deploy' / 'mounting_geometry.py'
spec = importlib.util.spec_from_file_location('mounting_geometry', TOOL)
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)
HTML = (ROOT / 'panel' / 'index.html').read_text(encoding='utf-8')
LAUNCH = (ROOT / 'src' / 'netgun_intercept' / 'launch' / 'netgun_real.launch.py').read_text(encoding='utf-8')


def quaternion_matrix(q):
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def config(**changes):
    value = json.loads(json.dumps(geometry.DEFAULT_CONFIG))
    value.update(changes)
    return value


class TemporaryConfig:
    def __enter__(self):
        self.directory = tempfile.mkdtemp(prefix='netgun_config_')
        self.previous = os.environ.get('NETGUN_CONFIG_DIR')
        os.environ['NETGUN_CONFIG_DIR'] = self.directory
        return pathlib.Path(self.directory)

    def __exit__(self, *exc):
        if self.previous is None:
            os.environ.pop('NETGUN_CONFIG_DIR', None)
        else:
            os.environ['NETGUN_CONFIG_DIR'] = self.previous
        shutil.rmtree(self.directory, ignore_errors=True)


class DerivationTests(unittest.TestCase):
    def test_user_measurements_are_the_defaults(self):
        derived = geometry.derive(geometry.validate_editable(geometry.DEFAULT_CONFIG))
        self.assertEqual(geometry.DEFAULT_CONFIG['camera_center_m'], [0.0, 0.06, 0.0])
        self.assertEqual(geometry.DEFAULT_CONFIG['rtk_antenna_m'], [0.0, 0.06, 0.085])
        self.assertEqual(derived['left_eye_m'], [-0.025, 0.06, 0.0])
        self.assertEqual(derived['camera_translation_body_m'], [-0.025, 0.06, 0.0])
        self.assertEqual(derived['camera_q_body_to_optical_wxyz'], [0.0, 1.0, 0.0, 0.0])
        self.assertEqual(derived['muzzle_offset_body_m'], [0.0, 0.0, 0.0])
        self.assertEqual((derived['boresight_pitch_rad'], derived['boresight_yaw_rad']), (0.0, 0.0))
        self.assertEqual(derived['px4_gps_pos_m'], {'EKF2_GPS_POS_X': 0.0, 'EKF2_GPS_POS_Y': 0.06, 'EKF2_GPS_POS_Z': 0.085})

    def test_right_eye_quaternions_match_upward_optical_axes(self):
        for name, eye in geometry.RIGHT_EYE.items():
            with self.subTest(name=name):
                matrix = quaternion_matrix(eye['q_wxyz'])
                self.assertAlmostEqual(sum(v * v for v in eye['q_wxyz']), 1.0, places=12)
                x_axis, z_axis = list(eye['axis']), [0.0, 0.0, -1.0]
                rows = [x_axis, cross(z_axis, x_axis), z_axis]  # 光学系各轴在机体系的方向
                for row, expected in zip(matrix, rows):
                    for got, want in zip(row, expected):
                        self.assertAlmostEqual(got, want, places=12)
                # 180° 旋转：body→optical 与 optical→body 同一矩阵，估计器约定不影响结果
                for i in range(3):
                    for j in range(3):
                        self.assertAlmostEqual(matrix[i][j], matrix[j][i], places=12)

    def test_left_eye_is_half_baseline_against_right_eye(self):
        for name, eye in geometry.RIGHT_EYE.items():
            with self.subTest(name=name):
                derived = geometry.derive(geometry.validate_editable(config(camera_right_eye=name, camera_center_m=[0.01, 0.06, -0.02])))
                expected = [round(c - 0.025 * a, 4) for c, a in zip([0.01, 0.06, -0.02], eye['axis'])]
                self.assertEqual(derived['left_eye_m'], expected)

    def test_flight_controller_is_internal_origin(self):
        derived = geometry.derive(geometry.validate_editable(config(flight_controller_m=[0.02, -0.01, 0.03])))
        self.assertEqual(derived['camera_translation_body_m'], [-0.045, 0.07, -0.03])
        self.assertEqual(derived['muzzle_offset_body_m'], [-0.02, 0.01, -0.03])
        self.assertEqual(derived['px4_gps_pos_m'], {'EKF2_GPS_POS_X': -0.02, 'EKF2_GPS_POS_Y': 0.07, 'EKF2_GPS_POS_Z': 0.055})

    def test_tilt_reproduces_strike_gun_axis_formula(self):
        for tilt in ([0.0, 0.0], [2.0, 0.0], [0.0, -3.0], [1.5, 2.5], [-10.0, 10.0]):
            with self.subTest(tilt=tilt):
                axis, pitch, yaw = geometry.gun_axis_from_tilt(tilt)
                strike = [-math.sin(pitch) * math.cos(yaw), -math.sin(pitch) * math.sin(yaw), -math.cos(pitch)]
                tf, tr = math.tan(math.radians(tilt[0])), math.tan(math.radians(tilt[1]))
                norm = math.sqrt(1 + tf * tf + tr * tr)
                for got, from_strike, want in zip(axis, strike, [tf / norm, tr / norm, -1 / norm]):
                    self.assertAlmostEqual(got, want, places=12)
                    self.assertAlmostEqual(from_strike, want, places=12)
                self.assertAlmostEqual(math.degrees(math.atan2(axis[0], -axis[2])), tilt[0], places=9)
                self.assertAlmostEqual(math.degrees(math.atan2(axis[1], -axis[2])), tilt[1], places=9)
        self.assertEqual(geometry.gun_axis_from_tilt([0.0, 0.0])[1:], (0.0, 0.0))

    def test_launch_arguments_are_float_text_in_fixed_order(self):
        pairs = geometry.launch_arguments(geometry.derive(geometry.validate_editable(geometry.DEFAULT_CONFIG)))
        self.assertEqual([name for name, _ in pairs], [
            'camera_translation_body_x_m', 'camera_translation_body_y_m', 'camera_translation_body_z_m',
            'camera_q_body_to_optical_w', 'camera_q_body_to_optical_x', 'camera_q_body_to_optical_y', 'camera_q_body_to_optical_z',
            'muzzle_offset_body_x_m', 'muzzle_offset_body_y_m', 'muzzle_offset_body_z_m', 'boresight_pitch_rad', 'boresight_yaw_rad'])
        for _, text in pairs:
            self.assertIn('.', text)
            self.assertTrue(math.isfinite(float(text)))

    def test_launch_file_defaults_equal_default_geometry(self):
        declared = dict(re.findall(r'DeclareLaunchArgument\(\s*"([a-z_]+)",\s*default_value="([^"]*)"', LAUNCH))
        for name, text in geometry.launch_arguments(geometry.derive(geometry.validate_editable(geometry.DEFAULT_CONFIG))):
            with self.subTest(name=name):
                self.assertIn(name, declared)
                self.assertEqual(float(declared[name]), float(text))


class ValidationTests(unittest.TestCase):
    def test_rejects_malformed_or_out_of_range_values(self):
        bad = [
            config(camera_center_m=[0.0, 0.06]),
            config(camera_center_m=[0.0, True, 0.0]),
            config(rtk_antenna_m=[0.0, float('nan'), 0.0]),
            config(rtk_antenna_m=[0.0, 0.0, 0.51]),
            config(flight_controller_m='0,0,0'),
            config(gun_axis_tilt_deg=[10.5, 0.0]),
            config(gun_axis_tilt_deg=[0.0]),
            config(camera_right_eye='up'),
            config(extra=1),
        ]
        missing = config();missing.pop('rtk_antenna_m')
        for raw in bad + [missing, [], None]:
            with self.subTest(raw=raw), self.assertRaises(geometry.GeometryError):
                geometry.validate_editable(raw)

    def test_normalises_numbers(self):
        value = geometry.validate_editable(config(camera_center_m=[0, 0.060000001, -0.0], gun_axis_tilt_deg=[1.234, -0.0]))
        self.assertEqual(value['camera_center_m'], [0.0, 0.06, 0.0])
        self.assertEqual(value['gun_axis_tilt_deg'], [1.23, 0.0])
        self.assertTrue(all(isinstance(v, float) for v in value['camera_center_m']))


class StorageTests(unittest.TestCase):
    def run_main(self, *args, stdin=None):
        out = io.StringIO()
        old = sys.stdin
        try:
            sys.stdin = io.StringIO(stdin or '')
            with redirect_stdout(out):
                code = geometry.main(['mounting_geometry.py', *args])
        finally:
            sys.stdin = old
        return code, out.getvalue()

    def test_defaults_until_saved_then_history_is_kept(self):
        with TemporaryConfig() as directory:
            info = geometry.describe()
            self.assertEqual((info['source'], info['saved_at']), ('default', None))
            first = geometry.save(config(gun_axis_tilt_deg=[1.0, 0.0]))
            self.assertEqual(first['source'], 'file')
            self.assertTrue(first['saved_at'])
            stored = json.loads((directory / 'mounting_geometry.json').read_text(encoding='utf-8'))
            self.assertEqual((stored['reference'], stored['frame'], stored['schema_version']), ('netgun_center', 'body_frd', 1))
            geometry.save(config(gun_axis_tilt_deg=[2.0, 0.0]))
            history = sorted((directory / 'history').iterdir())
            self.assertEqual(len(history), 1)
            self.assertEqual(json.loads(history[0].read_text(encoding='utf-8'))['gun_axis_tilt_deg'], [1.0, 0.0])
            self.assertEqual(geometry.describe()['config']['gun_axis_tilt_deg'], [2.0, 0.0])
            self.assertEqual(sorted(p.name for p in directory.iterdir()), ['history', 'mounting_geometry.json'])

    def test_cli_save_reads_stdin_and_rejects_invalid_request(self):
        with TemporaryConfig():
            code, out = self.run_main('save', stdin=json.dumps({'config': config(camera_right_eye='right')}))
            self.assertEqual(code, 0)
            result = json.loads(out)
            self.assertEqual(result['derived']['camera_q_body_to_optical_wxyz'], [0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0])
            code, out = self.run_main('save', stdin=json.dumps({'config': config(camera_center_m=[0, 0.9, 0])}))
            self.assertEqual(code, 1)
            self.assertFalse(json.loads(out)['ok'])
            code, out = self.run_main('save', stdin=json.dumps({'config': config(), 'command': 'rm'}))
            self.assertEqual(code, 1)
            self.assertEqual(geometry.describe()['config']['camera_right_eye'], 'right')
            code, out = self.run_main('launch-args')
            self.assertEqual(code, 0)
            self.assertIn('camera_q_body_to_optical_x:=0.7071067811865476', out.splitlines())

    def test_invalid_file_blocks_launch_arguments(self):
        with TemporaryConfig() as directory:
            (directory / 'mounting_geometry.json').write_text('{"schema_version": 1}', encoding='utf-8')
            err = io.StringIO()
            old = sys.stderr
            try:
                sys.stderr = err
                code, out = self.run_main('launch-args')
            finally:
                sys.stderr = old
            self.assertEqual((code, out), (2, ''))
            self.assertIn('安装参数无效', err.getvalue())
            code, out = self.run_main('show')
            self.assertEqual(code, 1)
            self.assertIn('mounting_geometry.json', json.loads(out)['error'])

    def test_summary_line_is_parsed_by_status_script(self):
        status_spec = importlib.util.spec_from_file_location('interceptor_status', ROOT / 'deploy' / 'interceptor_status.py')
        status = importlib.util.module_from_spec(status_spec)
        status_spec.loader.exec_module(status)
        with TemporaryConfig():
            geometry.save(config(gun_axis_tilt_deg=[0.5, -0.5]))
            code, line = self.run_main('summary')
            self.assertEqual(code, 0)
            self.assertTrue(line.startswith('NETGUN_GEOMETRY {'))
            info = status.geometry_status('header\n' + line + 'later line\n')
            self.assertEqual(info['saved']['config']['gun_axis_tilt_deg'], [0.5, -0.5])
            self.assertEqual(info['applied']['config'], info['saved']['config'])
            self.assertNotIn('applied', status.geometry_status('no geometry here\n'))


@unittest.skipIf(os.name == 'nt' or not shutil.which('bash') or not shutil.which('setsid'), 'POSIX bash required')
class StartScriptTests(unittest.TestCase):
    def setUp(self):
        self.root = pathlib.Path(tempfile.mkdtemp(prefix='interceptor_ws_'))
        self.workspace = self.root / 'ws'
        (self.workspace / 'install').mkdir(parents=True)
        (self.workspace / 'deploy').mkdir()
        shutil.copy(TOOL, self.workspace / 'deploy' / 'mounting_geometry.py')
        self.config = self.root / 'config'
        self.logs = self.root / 'logs'
        bin_dir = self.root / 'bin'
        bin_dir.mkdir()
        self.args_file = self.root / 'ros2_args.txt'
        stub = bin_dir / 'ros2'
        stub.write_text('#!/bin/bash\nprintf "%s\\n" "$@" > "$ARGS_FILE"\nexec sleep 20\n', encoding='utf-8')
        stub.chmod(0o755)
        script = (ROOT / 'deploy' / 'run_interceptor_stack.sh').read_text(encoding='utf-8')
        # 测试副本：不 source 板上 ROS 环境，不跑 15s 后台自检；其余逻辑原样
        self.assertEqual(script.count('env_setup() {\n'), 1)
        self.assertEqual(script.count('sleep 15; run_start_check "$log_file" "$pid"'), 1)
        script = script.replace('env_setup() {\n', 'env_setup() {\n  return 0\n', 1)
        script = script.replace('sleep 15; run_start_check "$log_file" "$pid"', 'true', 1)
        self.script = self.root / 'run_interceptor_stack.sh'
        self.script.write_text(script, encoding='utf-8')
        self.env = dict(os.environ, INTERCEPTOR_WS=str(self.workspace), NETGUN_LOG_DIR=str(self.logs),
                        NETGUN_CONFIG_DIR=str(self.config), ARGS_FILE=str(self.args_file),
                        PATH=str(bin_dir) + os.pathsep + os.environ.get('PATH', ''))

    def tearDown(self):
        pid_file = self.logs / 'interceptor_stack.pid'
        if pid_file.exists():
            try:
                os.killpg(int(pid_file.read_text().strip()), signal.SIGTERM)
            except (OSError, ValueError):
                pass
        shutil.rmtree(self.root, ignore_errors=True)

    def start(self, *extra):
        return subprocess.run(['bash', str(self.script), 'start', *extra], env=self.env, capture_output=True, text=True, timeout=30)

    def test_saved_geometry_precedes_command_line_overrides(self):
        self.config.mkdir()
        env = dict(os.environ, NETGUN_CONFIG_DIR=str(self.config))
        request = json.dumps({'config': config(gun_axis_tilt_deg=[2.0, 0.0])})
        subprocess.run([sys.executable, str(TOOL), 'save'], input=request, env=env, capture_output=True, text=True, check=True)
        result = self.start('camera_translation_body_x_m:=0.5')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('安装参数 已保存', result.stdout)
        for _ in range(50):
            if self.args_file.exists() and self.args_file.read_text().strip():
                break
            time.sleep(0.1)
        args = self.args_file.read_text().splitlines()
        self.assertEqual(args[:2], ['launch', 'netgun_intercept'])
        self.assertEqual(args[2], 'netgun_real.launch.py')
        self.assertEqual(args[3], 'camera_translation_body_x_m:=-0.025')
        self.assertEqual(args[-1], 'camera_translation_body_x_m:=0.5', 'explicit arguments must come last and win')
        pitch = [a for a in args if a.startswith('boresight_pitch_rad:=')]
        self.assertAlmostEqual(float(pitch[0].split(':=')[1]), -math.radians(2.0), places=12)
        log = next(self.logs.glob('interceptor_stack_*.log')).read_text(encoding='utf-8')
        self.assertTrue(log.startswith('NETGUN_GEOMETRY {'))

    def test_invalid_geometry_refuses_to_start(self):
        self.config.mkdir()
        (self.config / 'mounting_geometry.json').write_text('not json', encoding='utf-8')
        result = self.start()
        self.assertEqual(result.returncode, 1)
        self.assertIn('安装参数无效', result.stderr)
        self.assertFalse(self.args_file.exists())
        self.assertEqual(list(self.logs.glob('interceptor_stack_*.log')), [])
        self.assertFalse((self.logs / 'interceptor_stack.pid').exists())


@unittest.skipUnless(shutil.which('node'), 'Node required')
class PanelDerivationTests(unittest.TestCase):
    def test_browser_derivation_and_readout_match_board_tool(self):
        block = HTML.split('// Mounting geometry controls', 1)[1].split('// End mounting geometry controls', 1)[0]
        cases = [
            geometry.validate_editable(geometry.DEFAULT_CONFIG),
            geometry.validate_editable(config(camera_right_eye='backward', camera_center_m=[0.012, 0.0575, -0.031], flight_controller_m=[0.01, -0.02, 0.04], gun_axis_tilt_deg=[1.25, -2.5])),
            geometry.validate_editable(config(camera_right_eye='right', rtk_antenna_m=[-0.1, 0.06, 0.12], gun_axis_tilt_deg=[-9.99, 0.01])),
            geometry.validate_editable(config(camera_right_eye='left', camera_center_m=[0.5, -0.5, 0.5], flight_controller_m=[-0.5, 0.5, -0.5])),
        ]
        points = [[0.3, -0.2, 6.0], [-1.0, 0.4, 9.5]]
        harness = """
const elements={};const byId=id=>elements[id]||(elements[id]={value:'',textContent:'',disabled:false,open:false,classList:{toggle(){}}});
const document={activeElement:null};let state={},busy=false,controlBusy=false,restarted=false,meta=null;
const number=(v,d=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—',message=()=>{},setStatus=()=>{},action=async()=>null,currentTargets=()=>[];
""" + block + """
const cases=%s,points=%s;
console.log(JSON.stringify(cases.map(config=>({derived:mountDerive(config),readout:points.map(p=>mountTargetReadout([{track_id:1,confidence:.9,mode:'depth',position_camera:p}],config))}))));
""" % (json.dumps(cases), json.dumps(points))
        result = subprocess.run(['node', '-e', harness], capture_output=True, text=True, encoding='utf-8', timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        for case, browser in zip(cases, json.loads(result.stdout)):
            board = geometry.derive(case)
            js = browser['derived']
            for key in ('left_eye_m', 'camera_translation_body_m', 'muzzle_offset_body_m'):
                self.assertEqual(js[key], board[key], key)
            self.assertEqual(js['px4_gps_pos_m'], [board['px4_gps_pos_m'][k] for k in ('EKF2_GPS_POS_X', 'EKF2_GPS_POS_Y', 'EKF2_GPS_POS_Z')])
            for key in ('boresight_pitch_rad', 'boresight_yaw_rad'):
                self.assertAlmostEqual(js[key], board[key], places=12)
            for a, b in zip(js['gun_axis_body'], board['gun_axis_body']):
                self.assertAlmostEqual(a, b, places=12)
            matrix = quaternion_matrix(board['camera_q_body_to_optical_wxyz'])
            for point, readout in zip(points, browser['readout']):
                # 估计器口径：p_body = R(q)ᵀ·p_optical + 左目（此处以网枪中心为原点）
                body = [sum(matrix[r][c] * point[r] for r in range(3)) + board['left_eye_m'][c] for c in range(3)]
                for a, b in zip(readout['position'], body):
                    self.assertAlmostEqual(a, b, places=12)
                if body[2] < 0:
                    self.assertAlmostEqual(readout['forward'], math.degrees(math.atan2(body[0], -body[2])), places=9)
                    self.assertAlmostEqual(readout['right'], math.degrees(math.atan2(body[1], -body[2])), places=9)
                    axis = board['gun_axis_body']
                    cosine = sum(a * b for a, b in zip(axis, body)) / math.sqrt(sum(v * v for v in body))
                    self.assertAlmostEqual(readout['error'], math.degrees(math.acos(max(-1.0, min(1.0, cosine)))), places=6)


if __name__ == '__main__':
    unittest.main()
