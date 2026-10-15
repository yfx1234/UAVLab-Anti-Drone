#!/usr/bin/env python3
"""Read-only inventory of Git files; no ROS or third-party dependencies."""

import argparse
from collections import Counter, defaultdict
import hashlib
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


SOURCE_SUFFIXES = {'.py', '.cpp', '.hpp', '.h', '.cu', '.sh', '.ps1'}


def git_files(root):
    result = subprocess.run(
        ['git', '-C', str(root), 'ls-files', '-z', '--cached', '--others',
         '--exclude-standard'], check=True, stdout=subprocess.PIPE)
    return sorted({Path(name.decode('utf-8'))
                   for name in result.stdout.split(b'\0') if name})


def is_snapshot(path):
    name = path.name.lower()
    return '.bak' in name or '.prediction_before_' in name or name.endswith('.orig')


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def inventory(root, paths):
    sizes = defaultdict(list)
    packages, snapshots, sources, skipped = [], [], [], []
    extensions = Counter()
    for relative in sorted(paths):
        absolute = root / relative
        if absolute.is_symlink() or not absolute.is_file():
            skipped.append(relative.as_posix())
            continue
        size = absolute.stat().st_size
        sizes[size].append(relative)
        extensions[relative.suffix or '(无扩展名)'] += 1
        if is_snapshot(relative):
            snapshots.append(relative.as_posix())
        elif relative.suffix in SOURCE_SUFFIXES:
            lines = len(absolute.read_bytes().splitlines())
            if lines >= 500:
                sources.append((lines, relative.as_posix()))
        if relative.name == 'package.xml':
            manifest = ET.parse(absolute).getroot()
            packages.append((manifest.findtext('name') or '(缺少名称)',
                             relative.parent.as_posix(),
                             (absolute.parent / 'COLCON_IGNORE').exists()))
    duplicates = []
    for size, candidates in sorted(sizes.items()):
        if size == 0 or len(candidates) < 2:
            continue
        hashes = defaultdict(list)
        for relative in candidates:
            hashes[digest(root / relative)].append(relative.as_posix())
        for checksum, names in sorted(hashes.items()):
            if len(names) > 1:
                duplicates.append((size, checksum, names))
    return packages, snapshots, sorted(sources, reverse=True), duplicates, extensions, skipped


def render(root, paths):
    live = [relative for relative in paths if relative.parts[0] != 'archive']
    archived = [relative for relative in paths if relative.parts[0] == 'archive']
    packages, snapshots, sources, duplicates, extensions, skipped = inventory(root, live)
    redundant = sum(size * (len(names) - 1) for size, _, names in duplicates)
    lines = [
        '# 仓库结构审计（自动生成）', '',
        '范围：Git 已跟踪文件及未被忽略的新增文件；跳过符号链接和缺失文件。',
        '仅报告文件结构、字节完全相同的副本及文件规模，不判断算法等价、运行活跃度或删除安全性。',
        '构建产物、被忽略的数据及仓库外调用不在本报告范围内。',
        '顶层 archive/ 为归档区：不参与构建，只计入汇总行，不计入下列活树指标。', '',
        f'- 文件路径（活树）：{len(live) - len(skipped)}',
        f'- 索引中已不存在于工作区的路径：{len(skipped)}',
        f'- 归档区文件：{sum(1 for relative in archived if (root / relative).is_file())}（archive/）',
        f'- ROS 包：{len(packages)}',
        f'- 备份命名文件：{len(snapshots)}',
        f'- 非空重复内容组：{len(duplicates)}',
        f'- 重复副本占用（理论值，未删除）：{redundant:,} 字节', '',
        '## 包目录', '', '| 包名 | 路径 | COLCON_IGNORE |', '|---|---|---|',
    ]
    for name, location, ignored in packages:
        lines.append(f'| {name} | `{location}` | {"是" if ignored else "否"} |')
    lines += ['', '## 完全相同的文件', '',
              '按大小筛选后计算 SHA-256；行尾不同的文件不会被当作完全相同。',
              '消息定义即使内容一致，所属包名也可能是接口身份的一部分，不能据此互换。', '']
    for size, checksum, names in duplicates:
        lines += [f'### {size:,} 字节 / SHA-256 `{checksum}`', '']
        lines.extend(f'- `{name}`' for name in names)
        lines.append('')
    lines += ['## 备份命名文件', '', '命名只表示候选历史文件，不表示已被 Git 历史覆盖。', '']
    lines.extend(f'- `{name}`' for name in snapshots)
    lines += ['', '## 较大源文件（至少 500 行）', '',
              '规模用于定位阅读成本；不据此推断职责混乱。', '',
              '| 行数 | 文件 |', '|---:|---|']
    lines.extend(f'| {count} | `{name}` |' for count, name in sources)
    lines += ['', '## 扩展名统计', '', '| 扩展名 | 数量 |', '|---|---:|']
    lines.extend(f'| `{suffix}` | {count} |' for suffix, count in sorted(extensions.items()))
    if skipped:
        lines += ['', '## 跳过的文件', '']
        lines.extend(f'- `{name}`' for name in skipped)
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    root = args.root.resolve()
    print(render(root, git_files(root)), end='')


if __name__ == '__main__':
    main()
