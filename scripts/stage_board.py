#!/usr/bin/env python3
"""Copy reviewed source into the real ROS workspace; never change camera ownership."""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import shutil
import tempfile

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(1024 * 1024): h.update(block)
    return h.hexdigest()

def plan(repo, project):
    entries = []
    for package in ('zed_perception_interfaces', 'zed_tracking'):
        destination = project / 'ros2_ws/src' / package
        if destination.is_symlink(): raise ValueError('Workspace package must contain real files, not a symlink')
        for source in sorted((repo / 'board' / package).rglob('*')):
            if not source.is_file() or '__pycache__' in source.parts or source.suffix == '.pyc': continue
            target = destination / source.relative_to(repo / 'board' / package)
            if not target.resolve().is_relative_to(project): raise ValueError('Target escapes project directory')
            if not target.exists() or digest(target) != digest(source): entries.append((source, target))
    index = json.loads((repo / 'models/model-index.json').read_text(encoding='utf-8'))
    for item in index['source_artifacts']:
        source = repo / item['repository_path']
        if digest(source) != item['sha256']: raise ValueError('Model source checksum mismatch')
        target = project / 'weights/candidates/platform_exp_20260909' / source.name
        if not target.resolve().is_relative_to(project): raise ValueError('Model target escapes project directory')
        if not target.exists() or digest(target) != item['sha256']: entries.append((source, target))
    return entries

def main():
    args = argparse.ArgumentParser()
    args.add_argument('--project-root', type=pathlib.Path, required=True)
    args.add_argument('--dry-run', action='store_true')
    opts = args.parse_args()
    if os.name != 'posix': raise SystemExit('Run this script on the Jetson over SSH')
    if not opts.project_root.is_absolute(): raise SystemExit('Project root must be absolute')
    project = opts.project_root.resolve()
    if len(project.parts) < 4: raise SystemExit('Choose the named project directory, not a system root')
    repo = pathlib.Path(__file__).resolve().parents[1]
    changes = plan(repo, project)
    print(json.dumps({'dry_run': opts.dry_run, 'changed_files': len(changes),
                      'targets': [str(target) for _, target in changes]}, indent=2))
    if opts.dry_run or not changes: return
    state_file = pathlib.Path('/tmp/zed_perception/supervisor.json')
    if state_file.is_file():
        pid = json.loads(state_file.read_text()).get('pid')
        if isinstance(pid, int) and pathlib.Path('/proc', str(pid)).exists():
            raise SystemExit('Stop and release the camera before deploying a new supervisor/source version')
    backup = project / 'migration_backups' / datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    for source, target in changes:
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            saved = backup / target.relative_to(project)
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, saved)
        with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as temporary:
            temporary.write(source.read_bytes()); staged = pathlib.Path(temporary.name)
        try:
            staged.chmod(source.stat().st_mode & 0o777)
            os.replace(staged, target)
        finally:
            staged.unlink(missing_ok=True)
    print(json.dumps({'backup': str(backup), 'deployed_files': len(changes), 'camera_started': False}))

if __name__ == '__main__': main()
