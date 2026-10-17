"""Explicit, offline local media rename with preflight and rollback."""
import hashlib
import json
import pathlib
import re
import uuid
from media_names import annotation_metadata, video_filename, video_kind


class MediaFilenameMigration:
    def __init__(self, library):self.library = library

    @staticmethod
    def digest(path):
        with path.open('rb') as stream:return hashlib.file_digest(stream, 'sha256').hexdigest()

    def plan(self):
        library = self.library
        with library.registry.lock:
            library.registry.discover();changes = []
            for sid in library.registry.scenes:
                folder = library.registry.folder(sid)
                for path in sorted(folder.glob('exports/*/*.mp4')):
                    if '.tmp.' in path.name or re.match(r'^\d{8}-\d{6}_', path.name):continue
                    relative = path.relative_to(library.root).as_posix()
                    library.selected(relative) if path.name.endswith('.view.mp4') else library.path(relative)
                    if path.is_symlink() or any(p.is_symlink() for p in path.parents if p.is_relative_to(folder)):
                        raise ValueError('不允许迁移链接视频')
                    kind = video_kind(path);data = {}
                    if kind == 'annotated':
                        sidecar = path.with_suffix('.json') if path.name.endswith('.view.mp4') else path.with_suffix('.view.json')
                        data = annotation_metadata(path.name, library.registry._json(sidecar))
                    token = hashlib.sha256(path.name.removesuffix('.view.mp4').removesuffix('.mp4').encode()).hexdigest()[:12]
                    name = video_filename(library.registry.scenes[sid]['created_at_unix_s'], kind, path.parent.name,
                                          annotation=data, token=token, original=not path.name.endswith('.view.mp4'))
                    changes.append({'old': relative, 'new': path.with_name(name).relative_to(library.root).as_posix(),
                                    'bytes': path.stat().st_size, 'sha256': self.digest(path), 'annotation': data})
            return changes

    def apply(self, changes):
        library = self.library;root = library.root
        with library.registry.lock:
            if changes != self.plan():raise ValueError('迁移清单已过期，请重新检查')
            moves = [];updates = {};mapping = {c['old']: c['new'] for c in changes}
            for change in changes:
                source = library.path(change['old']);target = root/change['new']
                moves.append((source, target))
                old_json = source.with_suffix('.json');new_json = target.with_suffix('.json')
                if new_json.exists():raise ValueError('目标元数据已存在，未覆盖任何文件')
                if old_json.exists():moves.append((old_json, new_json))
                if change['annotation'] and target.name.endswith('.view.mp4'):
                    updates[new_json] = change['annotation']
            targets = [str(target).casefold() for _, target in moves]
            if len(set(targets)) != len(targets) or any(target.exists() for _, target in moves):
                raise ValueError('目标文件已存在，未覆盖任何视频')
            moved_json = {source: target for source, target in moves if source.suffix == '.json'}
            for sid in library.registry.scenes:
                folder = library.registry.folder(sid)
                local_map = {str(pathlib.PurePosixPath(old).relative_to(folder.name)): str(pathlib.PurePosixPath(new).relative_to(folder.name))
                             for old, new in mapping.items() if pathlib.PurePosixPath(old).parts[0] == folder.name}
                def rewrite(value):
                    if isinstance(value, list):return [rewrite(v) for v in value]
                    if not isinstance(value, dict):return value
                    result = {key: rewrite(v) for key, v in value.items()}
                    if result.get('source_relative') in mapping:result['source_relative'] = mapping[result['source_relative']]
                    if result.get('file') in local_map:
                        # Integrity records retain the board-side protocol key.
                        if 'sha256' in result:result.setdefault('remote_file', result['file'])
                        result['file'] = local_map[result['file']]
                    return result
                for path in [*folder.glob('exports/*/*.json'), *folder.glob('download_integrity_*.json')]:
                    if path.is_symlink():raise ValueError('不允许迁移链接元数据')
                    target = moved_json.get(path, path)
                    data = updates.get(target, library.registry._json(path))
                    revised = rewrite(data)
                    if target in updates or revised != data:updates[target] = revised
            # Preflight new metadata paths too; preserve exact bytes for rollback.
            created = set(updates) - set(moved_json.values())
            backups = {path: path.read_bytes() if path.exists() else None for path in created}
            for source, target in moved_json.items():
                if target in updates:backups[target] = source.read_bytes()
            completed = []
            try:
                for source, target in moves:source.rename(target);completed.append((source, target))
                for path, data in updates.items():
                    temp = path.with_name(uuid.uuid4().hex + '.tmp')
                    try:
                        temp.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8');temp.replace(path)
                    finally:temp.unlink(missing_ok=True)
            except Exception:
                for path, data in backups.items():
                    if data is None:path.unlink(missing_ok=True)
                    elif path.exists():path.write_bytes(data)
                for source, target in reversed(completed):target.rename(source)
                raise
            return changes
