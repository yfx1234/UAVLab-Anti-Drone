import pathlib,sys,tempfile,hashlib,json,unittest
from unittest.mock import patch
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
class DeviceCapabilitiesTests(unittest.TestCase):
    def test_manifest_records_actual_platform_and_sdk(self):
        from build_model_manifest import build_environment
        facts={'l4t':'R36.4.3','sdk_version':'5.5.0','module_name':'Orin NX 16GB','module_sku':'P3767-0000','compatible':['nvidia,p3767-0000'],'platform_configuration_consistent':True,'nvpmodel_config':'/etc/nvpmodel/nvpmodel_p3767_0000_super.conf'}
        with patch('device_capabilities.collect',return_value=facts):result=build_environment()
        self.assertEqual(result['target'],'Orin NX 16GB')
        self.assertEqual(result['sdk'],'5.5.0')
        self.assertEqual(result.get('module_sku'),'P3767-0000')
    def test_collect_reads_fixture_sdk_and_keeps_acceptance_separate(self):
        from device_capabilities import collect
        with tempfile.TemporaryDirectory() as tmp:
            root=pathlib.Path(tmp)
            files={'proc/meminfo':'MemTotal: 16030520 kB\n','proc/device-tree/compatible':'nvidia,p3767-0000\0','usr/local/zed/include/sl/version.hpp':'#define ZED_SDK_MAJOR_VERSION 5\n#define ZED_SDK_MINOR_VERSION 5\n#define ZED_SDK_PATCH_VERSION 2\n','etc/nvpmodel.conf':'','etc/nv_tegra_release':'R36.4.3'}
            for name,data in files.items():p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(data)
            info=collect(root)
            self.assertEqual(info['sdk_version'],'5.5.2')
            self.assertEqual(info['module_sku'],'P3767-0000')
            self.assertFalse(info['platform_configuration_consistent'])
            self.assertFalse(info['nx16_ready'])
    def test_identifies_nx_from_sku_and_memory_not_model_label(self):
        from device_capabilities import classify_module
        nx=classify_module(['nvidia,p3767-0000'],16030520)
        self.assertEqual(nx['module_sku'],'P3767-0000');self.assertTrue(nx['runtime_identity_consistent'])
        nano=classify_module(['nvidia,p3767-0003'],8000000)
        self.assertEqual(nano['module_family'],'Orin Nano')
        unknown=classify_module(['nvidia,tegra234'],16030520)
        self.assertEqual(unknown['module_family'],'unknown')
    def test_eeprom_mismatch_does_not_enable_nx_runtime_optimizations(self):
        from device_capabilities import classify_module
        nx=classify_module(['nvidia,p3767-0003'],16030520,'699-13767-0000-303 G.1')
        self.assertEqual(nx['module_sku'],'P3767-0000');self.assertFalse(nx['runtime_identity_consistent'])
        wrong_memory=classify_module(['nvidia,p3767-0000'],8000000)
        self.assertFalse(wrong_memory['runtime_identity_consistent'])
    def test_selected_registered_engine_overrides_legacy_default(self):
        import perception_supervisor as s
        with tempfile.TemporaryDirectory() as tmp:
            base=pathlib.Path(tmp);weights=base/'weights/candidates/fixture';weights.mkdir(parents=True)
            (weights/'exp_opt3.engine').write_bytes(b'old');(weights/'exp_nx16.engine').write_bytes(b'new')
            config=base/'perception_config';config.mkdir();(config/'selected_model.json').write_text(json.dumps({'model_id':hashlib.sha256(b'new').hexdigest()}))
            with patch.object(s,'WS',base/'ros2_ws'):
                self.assertEqual(s.model_by_id()['name'],'exp_nx16.engine')
                self.assertEqual(sum(m['default'] for m in s.models()),1)
            (config/'selected_model.json').write_text(json.dumps({'model_id':'missing'}))
            with patch.object(s,'WS',base/'ros2_ws'):self.assertEqual(s.model_by_id()['name'],'exp_opt3.engine')
