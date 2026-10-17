#!/usr/bin/env python3
"""Reject credentials, recordings, generated engines and oversized Git objects."""
import argparse
import json
import pathlib
import re
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[1]
PATTERNS = {
    'private_key': re.compile(rb'-----BEGIN (?:OPENSSH |RSA |EC |DSA )?PRIVATE KEY-----'),
    'github_token': re.compile(rb'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})'),
    'aws_access_key': re.compile(rb'\b(?:AKIA|ASIA)[A-Z0-9]{16}\b'),
}
FORBIDDEN = {'.engine', '.svo', '.svo2', '.mp4', '.mkv', '.avi', '.pem', '.key', '.log', '.part'}

def git(*args):
    return subprocess.run(['git', '-c', 'core.quotePath=false', *args], cwd=ROOT,
                          capture_output=True, check=True).stdout

def inspect(name, data, problems):
    path = pathlib.PurePosixPath(name)
    if path.suffix.lower() in FORBIDDEN or path.name in {'id_rsa', 'id_ed25519', '.env', 'hosts.yml'}:
        problems.append({'file': name, 'reason': 'credential/generated/recorded file'})
    if len(data) >= 90 * 1024**2: problems.append({'file': name, 'reason': 'Git object exceeds 90 MiB'})
    if path.suffix.lower() in {'.pt', '.onnx'} or name.endswith('.dtbo.reference'): return
    for kind, pattern in PATTERNS.items():
        if pattern.search(data): problems.append({'file': name, 'reason': kind})

def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--history', action='store_true'); opts = parser.parse_args()
    problems = []; checked = 0
    for raw in git('ls-files', '-z').split(b'\0'):
        if not raw: continue
        name = raw.decode('utf-8'); path = ROOT / name
        if path.is_file(): inspect(name, path.read_bytes(), problems); checked += 1
    if opts.history:
        for line in git('rev-list', '--objects', '--all').decode('utf-8').splitlines():
            if ' ' not in line: continue
            object_id, name = line.split(' ', 1)
            if git('cat-file', '-t', object_id).strip() != b'blob': continue
            inspect(name, git('cat-file', 'blob', object_id), problems); checked += 1
    index = ROOT / 'models/model-index.json'
    if index.exists():
        import hashlib
        for item in json.loads(index.read_text(encoding='utf-8'))['source_artifacts']:
            p = ROOT / item['repository_path']
            if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != item['sha256']:
                problems.append({'file': item['repository_path'], 'reason': 'model checksum mismatch'})
    print(json.dumps({'checked_files_and_objects': checked, 'problems': problems}, ensure_ascii=False))
    raise SystemExit(bool(problems))

if __name__ == '__main__': main()
