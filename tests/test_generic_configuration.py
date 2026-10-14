import contextlib
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'board/zed_tracking/tools'))
import perception_supervisor as supervisor

class GenericConfigurationTests(unittest.TestCase):
    def test_retired_generic_can_still_be_disabled_with_existing_presets(self):
        for params in [{'enabled':False}]+[
            {'enabled':False,'sensitivity':preset} for preset in ('high','standard','low')]:
            supervisor.validate_action('set_generic_detector',params)

    def test_invalid_fields_and_types_rejected(self):
        for params in [{},{'enabled':1},{'enabled':'true'},{'enabled':True,'sensitivity':'ultra'},
                       {'enabled':True,'sensitivity':1},{'enabled':True,'engine_path':'x'}]:
            with self.subTest(params=params),self.assertRaises(ValueError):
                supervisor.validate_action('set_generic_detector',params)

    def test_generic_never_resolves_or_stages_a_yolo_model(self):
        state={'boot_id':'b','run_id':'r','revision':2,'pid':20}
        request={'action':'set_generic_detector','params':{'enabled':False,'sensitivity':'standard'},
                 'expected_boot_id':'b','expected_run_id':'r','expected_revision':2,'request_id':'generic_test'}
        with tempfile.TemporaryDirectory() as directory, \
             mock.patch.object(supervisor,'ROOT',pathlib.Path(directory)), \
             mock.patch.object(supervisor,'state',return_value=dict(state)), \
             mock.patch.object(supervisor,'state_lock',contextlib.nullcontext), \
             mock.patch.object(supervisor,'model_by_id',side_effect=AssertionError('model dependency')), \
             mock.patch.object(supervisor,'stage_model',side_effect=AssertionError('engine dependency')), \
             mock.patch.object(supervisor,'node_command',return_value={'ok':True}) as command:
            response=supervisor.dispatch(request)
            self.assertTrue(response['ok'])
            self.assertEqual(command.call_args.args[1:3],('set_generic_detector',request['params']))

if __name__=='__main__':unittest.main()
