"""Stable scene identity, recording timestamps and actual Windows folder names."""
import hashlib
import json
import math
import pathlib
import re
import threading
import uuid
import contextlib
from datetime import datetime, timezone, timedelta


class SceneRegistry:
    _instances = {}
    _instances_lock = threading.Lock()

    @classmethod
    def shared(cls, root, preview_root=None):
        key = str(pathlib.Path(root).resolve())
        with cls._instances_lock:
            registry = cls._instances.get(key)
            if registry is None:
                registry = cls(root, preview_root); cls._instances[key] = registry
            elif preview_root is not None:
                registry.preview_root = pathlib.Path(preview_root).resolve()
            return registry

    def __init__(self, root, preview_root=None):
        self.root = pathlib.Path(root).resolve()
        self.preview_root = pathlib.Path(preview_root).resolve() if preview_root else None
        self.lock = threading.RLock()
        self.operations = {}
        self.active_previews = set()
        self.path = self.root / 'scene_registry.json'
        try:
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if data.get('version') != 1 or not isinstance(data.get('scenes'), dict):
                raise ValueError('本机场次登记文件无效')
            self.scenes = data['scenes']; self.previews = data.get('previews', {})
        except FileNotFoundError:
            self.scenes = {}; self.previews = {}

    @contextlib.contextmanager
    def operation(self, sid):
        """A long writer owns only its scene; global metadata stays responsive."""
        self.validate_id(sid)
        with self.lock: operation = self.operations.setdefault(sid, threading.RLock())
        with operation: yield

    @staticmethod
    def validate_id(sid):
        if not isinstance(sid, str) or not re.fullmatch('[a-zA-Z0-9_-]{1,80}', sid):
            raise ValueError('场次 ID 无效')
        return sid

    @staticmethod
    def validate_name(name):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 100 or any(ord(c) < 32 for c in name):
            raise ValueError('名称需为1至100个字符，不能含换行或控制字符')
        return name.strip()

    @classmethod
    def safe_folder(cls, name, sid):
        cls.validate_id(sid); name = cls.validate_name(name)
        safe = re.sub(r'[<>:"/\\|?*\x00-\x1f]', '-', name).strip(' .')[:48].rstrip(' .') or '场次'
        if re.fullmatch(r'(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])(?:\..*)?', safe, re.I): safe = '_' + safe
        return safe + '--' + hashlib.sha256(sid.encode()).hexdigest()[:12]

    @staticmethod
    def default_name(stamp):
        return datetime.fromtimestamp(stamp, timezone(timedelta(hours=8))).strftime('%Y-%m-%d %H:%M:%S 北京时间')

    @staticmethod
    def _json(path):
        try: return json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError): return {}

    @staticmethod
    def _stamp(value):
        return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0 else None

    @staticmethod
    def checked(root, component):
        if not isinstance(component, str) or pathlib.PureWindowsPath(component).name != component or component in ('.', '..'):
            raise ValueError('本机场次路径越界')
        raw = root / component
        if raw.is_symlink() or raw.resolve().parent != root:
            raise ValueError('本机场次路径越界或为符号链接')
        return raw

    def save(self):
        self.root.mkdir(parents=True, exist_ok=True)
        temp = self.path.with_name('scene_registry.' + uuid.uuid4().hex + '.tmp')
        temp.write_text(json.dumps({'version': 1, 'scenes': self.scenes, 'previews': self.previews}, ensure_ascii=False), encoding='utf-8')
        temp.replace(self.path)

    def discover(self):
        """Import local-only UUID folders and old preview metadata once."""
        with self.lock:
            known = {r['folder'] for r in self.scenes.values()}
            names = self._json(self.root / 'scene_names.json')
            if self.root.exists():
                for folder in self.root.iterdir():
                    if not folder.is_dir() or folder.is_symlink() or folder.name in known: continue
                    if not re.fullmatch('[a-zA-Z0-9_-]{1,80}', folder.name): continue
                    if not (folder / 'manifest.json').is_file() and not (folder / 'exports').is_dir(): continue
                    manifest = self._json(folder / 'manifest.json')
                    stamp = self._stamp(manifest.get('created_at_unix_s'))
                    uncertain = stamp is None or manifest.get('created_time_source') not in ('pc_reference_monotonic', 'pc_reference')
                    pc = self._stamp(manifest.get('metadata', {}).get('client_started_at_unix_s'))
                    if manifest.get('created_time_source', 'board_clock') == 'board_clock' and pc and 1704067200 <= pc < 4102444800:
                        stamp = pc; uncertain = False
                    if stamp is None:
                        marker = folder / 'manifest.json'
                        stamp = (marker.stat() if marker.is_file() else folder.stat()).st_mtime
                    name = names.get(folder.name) or self.default_name(stamp)
                    self.scenes[folder.name] = {'session_id': folder.name, 'display_name': name,
                        'created_at_unix_s': stamp, 'created_time_uncertain': uncertain, 'folder': folder.name,
                        'pending_board_metadata': True}
                    self._sync_scene(folder.name)
            if self.preview_root and self.preview_root.exists():
                known = {r['folder'] for r in self.previews.values()}
                for folder in self.preview_root.iterdir():
                    if not folder.is_dir() or folder.is_symlink() or folder.name in known or not folder.name.startswith('preview_'): continue
                    metadata = self._json(folder / 'preview.json'); summary = self._json(folder / 'summary.json')
                    pid = metadata.get('preview_id', folder.name); self.validate_id(pid)
                    stamp = self._stamp(metadata.get('created_at_unix_s')) or self._stamp(summary.get('created_at_unix_s'))
                    uncertain = stamp is None
                    if stamp is None:
                        stat = folder.stat(); stamp = getattr(stat, 'st_birthtime', stat.st_ctime)
                    sid = metadata.get('source_session_id') or summary.get('source_session_id')
                    if sid: self.validate_id(sid)
                    self.previews[pid] = {'preview_id': pid, 'source_session_id': sid,
                        'display_name': metadata.get('display_name') or self.scenes.get(sid, {}).get('display_name') or self.default_name(stamp),
                        'created_at_unix_s': stamp, 'created_time_uncertain': uncertain,
                        'view_mode': metadata.get('view_mode', summary.get('view_mode', 'rgb')), 'folder': folder.name}
            self.save()

    def update(self, rows):
        self.discover()
        for row in rows:
            sid = self.validate_id(row['session_id'])
            with self.operation(sid), self.lock:
                sid = self.validate_id(row['session_id']); name = self.validate_name(row['display_name'])
                stamp = self._stamp(row.get('created_at_unix_s'))
                old = self.scenes.get(sid)
                if old is None:
                    stamp = stamp or 0
                    self.scenes[sid] = {'session_id': sid, 'display_name': name, 'created_at_unix_s': stamp,
                        'created_time_uncertain': bool(row.get('created_time_uncertain', not stamp)), 'folder': self.safe_folder(name, sid)}
                else:
                    old['display_name'] = name
                    if stamp:
                        old['created_at_unix_s'] = stamp
                        old['created_time_uncertain'] = bool(row.get('created_time_uncertain', True))
                # First board metadata replaces a local-only import name/time. The operation
                # lock waits for any conversion of that scene; subsequent refreshes do not move it.
                entry = self.scenes[sid]
                try:
                    if entry.get('pending_board_metadata') or entry['folder'] == sid:
                        self._sync_scene(sid); entry['pending_board_metadata'] = False
                finally: self.save()

    def folder(self, sid, create=False):
        with self.lock:
            self.validate_id(sid); self.discover()
            if sid not in self.scenes: raise ValueError('本机场次尚未登记')
            path = self.checked(self.root, self.scenes[sid]['folder'])
            if create: path.mkdir(parents=True, exist_ok=True)
            return path

    def _move(self, root, entry, desired):
        source = self.checked(root, entry['folder']); target = self.checked(root, desired)
        if source == target: return target
        if target.exists(): raise ValueError('本机目标文件夹已存在，未覆盖：' + str(target))
        if source.exists():
            try: source.rename(target)
            except OSError as e: raise ValueError('本机文件夹改名失败（请关闭正在播放的视频后重试）：' + str(e)) from e
        entry['folder'] = desired
        return target

    def _sync_scene(self, sid):
        entry = self.scenes[sid]
        return self._move(self.root, entry, self.safe_folder(entry['display_name'], sid))

    def rename(self, sid, name):
        with self.operation(sid), self.lock:
            self.validate_id(sid); name = self.validate_name(name); self.discover()
            if sid not in self.scenes: raise ValueError('本机场次尚未登记，请刷新列表后重试')
            entry = self.scenes[sid]; entry['display_name'] = name
            entry['pending_board_metadata'] = False
            linked = [(pid, p) for pid, p in self.previews.items() if p.get('source_session_id') == sid]
            for _, p in linked: p['display_name'] = name
            try:
                if any(pid in self.active_previews for pid, _ in linked):
                    raise ValueError('本机关联预览录像正在录制或结束，请停止录像后重试文件夹改名')
                folder = self._sync_scene(sid); previews = []
                for pid, p in linked:
                    previews.append(str(self._move(self.preview_root, p, self.safe_folder(name, pid))))
                    self._write_preview(p)
                entry.pop('local_sync_error', None)
                return {'ok': True, 'folder': str(folder), 'preview_folders': previews}
            except ValueError as e:
                entry['local_sync_error'] = str(e); raise
            finally: self.save()

    def _write_preview(self, entry):
        folder = self.checked(self.preview_root, entry['folder'])
        if folder.exists(): (folder / 'preview.json').write_text(json.dumps(entry, ensure_ascii=False), encoding='utf-8')

    def create_preview(self, pid, mode, stamp, sid=None):
        with self.lock:
            self.validate_id(pid)
            if not self.preview_root: raise ValueError('本机预览目录未配置')
            if sid: self.validate_id(sid)
            name = self.scenes.get(sid, {}).get('display_name') or self.default_name(stamp)
            entry = {'preview_id': pid, 'source_session_id': sid, 'display_name': name,
                     'created_at_unix_s': stamp, 'created_time_uncertain': False, 'view_mode': mode,
                     'folder': self.safe_folder(name, pid)}
            self.preview_root.mkdir(parents=True, exist_ok=True)
            folder = self.checked(self.preview_root, entry['folder']); folder.mkdir()
            self.previews[pid] = entry; self.active_previews.add(pid); self._write_preview(entry); self.save()
            return folder

    def finish_preview(self, pid):
        with self.lock:
            self.active_previews.discard(pid); self.save()
