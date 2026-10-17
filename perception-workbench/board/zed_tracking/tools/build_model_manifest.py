#!/usr/bin/env python3
"""Create provenance beside an engine; this never claims recognition accuracy."""
import argparse
import hashlib
import json
import pathlib
import subprocess
import sys

def sha256(path):
    if not path.is_file():return None
    h=hashlib.sha256()
    with path.open('rb') as f:
        while chunk:=f.read(1024*1024):h.update(chunk)
    return h.hexdigest()

def build(path):
    from perception_supervisor import EXE,models,state,atomic_json,run_sdk_tool
    if state().get('pid'):raise ValueError('stop camera before model validation')
    path=path.resolve()
    if str(path) not in {m['path'] for m in models()}:raise ValueError('engine not registered')
    check=run_sdk_tool([EXE,'--check-model',path])
    if check.returncode:raise ValueError(check.stderr[-1000:])
    contract=json.loads(check.stdout.strip().splitlines()[-1])
    stem='exp_portrait_1280x736' if 'portrait' in path.stem else ('exp' if path.stem.startswith('exp') else path.stem)
    pt=path.parent/(stem+'.pt');onnx=path.parent/(stem+'.onnx')
    manifest=dict(contract,engine_sha256=sha256(path),source_pt_sha256=sha256(pt),source_onnx_sha256=sha256(onnx),
        class_names=['drone'] if stem=='exp' and pt.is_file() and onnx.is_file() else None,
        preprocessing={'color':'RGB','normalization':'0_to_1','letterbox_fill':114,'rotation_degrees':0},
        output_layout='end_to_end_xyxy_score_class',batch=1,recognition_validated=False,
        build_environment=build_environment(),
        source_note='Source hashes establish provenance; graph equivalence and real-target accuracy require separate validation.')
    atomic_json(path.with_suffix('.manifest.json'),manifest)
    return manifest

def build_environment():
    from device_capabilities import collect
    facts=collect()
    return {'l4t':facts['l4t'],'sdk':facts['sdk_version'],'target':facts['module_name'],
            'module_sku':facts['module_sku'],'compatible':facts['compatible'],
            'platform_configuration_consistent':facts['platform_configuration_consistent'],
            'nvpmodel_config':facts['nvpmodel_config']}

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('engine',type=pathlib.Path);args=a.parse_args()
    print(json.dumps(build(args.engine),ensure_ascii=False))
