"""Rename existing local MP4s after stopping the PC panel. Default: plan only."""
import argparse
import json
import pathlib
import shutil
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'panel'))
from media_library import MediaLibrary
from media_filename_migration import MediaFilenameMigration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=pathlib.Path)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--report', type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.report.exists():raise ValueError('报告目录已存在，请选择新目录')
    args.report.mkdir(parents=True)
    library = MediaLibrary(args.root)
    migration = MediaFilenameMigration(library);plan = migration.plan()
    report = args.report/'renames.json'
    report.write_text(json.dumps({'applied': False, 'files': plan}, ensure_ascii=False, indent=2), encoding='utf-8')
    if args.apply:
        for sid in library.registry.scenes:
            folder = library.registry.folder(sid)
            for path in [*folder.glob('exports/*/*.json'), *folder.glob('download_integrity_*.json')]:
                if path.is_symlink():raise ValueError('不允许备份链接元数据')
                target = args.report/'metadata'/path.relative_to(library.root)
                target.parent.mkdir(parents=True, exist_ok=True);shutil.copy2(path, target)
        migration.apply(plan)
        for change in plan:
            if migration.digest(library.root/change['new']) != change['sha256']:
                raise ValueError('迁移后视频校验失败：'+change['new'])
        report.write_text(json.dumps({'applied': True, 'verified_sha256': True, 'files': plan}, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'applied': args.apply, 'videos': len(plan), 'report': str(report)}, ensure_ascii=False))


if __name__ == '__main__':main()
