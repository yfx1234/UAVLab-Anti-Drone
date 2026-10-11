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
        class_names=['drone'] if path.name=='exp_opt3.engine' else None,
        preprocessing={'color':'RGB','normalization':'0_to_1','letterbox_fill':114,'rotation_degrees':0},
        output_layout='end_to_end_xyxy_score_class',batch=1,recognition_validated=False,
        build_environment={'l4t':pathlib.Path('/etc/nv_tegra_release').read_text().strip(),'sdk':'5.5','target':'current_8GB_Jetson'},
        source_note='Source hashes establish provenance; graph equivalence and real-target accuracy require separate validation.')
    atomic_json(path.with_suffix('.manifest.json'),manifest)
    return manifest

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('engine',type=pathlib.Path);args=a.parse_args()
    print(json.dumps(build(args.engine),ensure_ascii=False))
