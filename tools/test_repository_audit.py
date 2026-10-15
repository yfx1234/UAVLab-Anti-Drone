"""Tests for repository housekeeping, using isolated temporary files."""

from pathlib import Path
import subprocess
import tempfile
import unittest

from repository_audit import git_files, inventory, is_snapshot, render


class RepositoryAuditTests(unittest.TestCase):
    def test_duplicates_require_identical_nonempty_bytes(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            contents = {'a.py': b'x\n', 'b.py': b'x\n', 'c.py': b'y\n',
                        'd.py': b'x\r\n', 'empty1': b'', 'empty2': b''}
            for name, data in contents.items():
                (root / name).write_bytes(data)
            duplicates = inventory(root, [Path(name) for name in contents])[3]
            self.assertEqual(len(duplicates), 1)
            self.assertEqual(duplicates[0][2], ['a.py', 'b.py'])

    def test_snapshot_names(self):
        for name in ['x.py.bak_20260910', 'calibration.bak.yaml',
                     'x.py.prediction_before_20260908', 'CMakeLists.txt.orig']:
            self.assertTrue(is_snapshot(Path(name)))
        self.assertFalse(is_snapshot(Path('backup_tools.py')))

    def test_manifest_ignore_marker_and_missing_file(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            package = root / '包'
            package.mkdir()
            (package / 'package.xml').write_text(
                '<package><name>example</name></package>', encoding='utf-8')
            (package / 'COLCON_IGNORE').touch()
            result = inventory(root, [Path('包/package.xml'), Path('missing')])
            self.assertEqual(result[0], [('example', '包', True)])
            self.assertEqual(result[5], ['missing'])

    def test_git_inventory_unicode_spaces_and_ignored_files(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            (root / '.gitignore').write_text('ignored.txt\n', encoding='utf-8')
            for name in ['已跟踪 文件.py', '新增 文件.py', 'ignored.txt']:
                (root / name).write_text('content', encoding='utf-8')
            subprocess.run(['git', '-C', str(root), 'add', '.gitignore',
                            '已跟踪 文件.py'], check=True)
            names = {path.as_posix() for path in git_files(root)}
            self.assertEqual(names, {'.gitignore', '已跟踪 文件.py', '新增 文件.py'})

    def test_render_is_stable_and_does_not_write(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            (root / 'large.py').write_bytes(b'# line\n' * 500)
            paths = [Path('large.py')]
            first = render(root, paths)
            self.assertEqual(first, render(root, paths))
            self.assertIn('| 500 | `large.py` |', first)
            self.assertEqual(list(root.iterdir()), [root / 'large.py'])

    def test_render_partitions_top_level_archive(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as directory:
            root = Path(directory)
            live_pkg = root / 'pkg'
            archived_pkg = root / 'archive' / 'deprecated_packages' / 'old_pkg'
            for directory_ in (live_pkg, archived_pkg,
                               root / 'archive' / 'snapshots'):
                directory_.mkdir(parents=True)
            (live_pkg / 'package.xml').write_text(
                '<package><name>live</name></package>', encoding='utf-8')
            (archived_pkg / 'package.xml').write_text(
                '<package><name>old</name></package>', encoding='utf-8')
            (root / 'live.py.bak_20260910').write_bytes(b'x\n')
            (root / 'archive' / 'snapshots' / 'old.py.bak_20260910').write_bytes(b'x\n')
            report = render(root, [Path('pkg/package.xml'),
                                   Path('archive/deprecated_packages/old_pkg/package.xml'),
                                   Path('live.py.bak_20260910'),
                                   Path('archive/snapshots/old.py.bak_20260910'),
                                   Path('ghost.py')])
            self.assertIn('- 文件路径（活树）：2', report)
            self.assertIn('- 索引中已不存在于工作区的路径：1', report)
            self.assertIn('- 归档区文件：2（archive/）', report)
            self.assertIn('- ROS 包：1', report)
            self.assertIn('- 备份命名文件：1', report)
            self.assertIn('`live.py.bak_20260910`', report)
            self.assertNotIn('`archive/', report)


if __name__ == '__main__':
    unittest.main()
