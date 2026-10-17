#!/usr/bin/env python3
"""One-off read-only scan: duplicate defs, unreferenced defs, nesting depth.

Usage: py tools/code_redundancy_scan.py  (from repo root)
"""

import ast
import hashlib
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCAN_DIRS = [
    "ros2_ws/src/falcon_intercept",
    "ros2_ws/src/falcon_contact",
    "ros2_ws/src/cooperative_intercept_fake_sim",
    "ros2_ws/src/px4_offboard_tracking",
    "ros2_ws/src/offboard_route_offset",
    "ros2_ws/scripts",
    "board_misc",
]
SKIP_NAMES = {"main", "shutdown", "spin"}


def normalize(text):
    return re.sub(r"\s+", " ", text).strip()


def max_depth(node, depth=0):
    best = depth
    for child in ast.iter_child_nodes(node):
        if isinstance(child, (ast.If, ast.For, ast.While, ast.With, ast.Try)):
            best = max(best, max_depth(child, depth + 1))
        else:
            best = max(best, max_depth(child, depth))
    return best


def main():
    texts = {}
    defs = []
    parse_failures = []
    for directory in SCAN_DIRS:
        base = ROOT / directory
        for path in sorted(base.rglob("*.py")):
            if "__pycache__" in path.parts or path.name.startswith("test_"):
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except UnicodeDecodeError as error:
                parse_failures.append((path, f"decode: {error}"))
                continue
            try:
                tree = ast.parse(text)
            except SyntaxError as error:
                parse_failures.append((path, f"syntax: {error}"))
                continue
            texts[path] = text
            for node in tree.body:
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                    body = ast.get_source_segment(text, node) or ""
                    defs.append({
                        "path": path.relative_to(ROOT).as_posix(),
                        "name": node.name,
                        "kind": type(node).__name__,
                        "hash": hashlib.md5(normalize(body).encode("utf-8")).hexdigest(),
                        "lines": len(body.splitlines()),
                        "depth": max_depth(node),
                    })

    all_repo_text = []
    for path in ROOT.rglob("*"):
        if (not path.is_file() or ".git" in path.parts or "archive" in path.parts
                or "__pycache__" in path.parts or path.suffix not in {
                    ".py", ".sh", ".cpp", ".hpp", ".h", ".cu", ".xml", ".yaml",
                    ".txt", ".md", ".ps1", ".cfg", ".json", ".service", ".timer"}):
            continue
        try:
            all_repo_text.append(path.read_text(encoding="utf-8", errors="ignore"))
        except OSError:
            pass
    corpus = "\n".join(all_repo_text)

    own_text = defaultdict(str)
    for entry in defs:
        own_text[entry["name"]] += texts.get(ROOT / entry["path"], "")

    print(f"scanned files: {len(texts)}, top-level defs: {len(defs)}")
    for path, reason in parse_failures:
        print(f"PARSE FAILURE {path}: {reason}")

    print("\n== identical normalized bodies (hash) ==")
    by_hash = defaultdict(list)
    for entry in defs:
        if entry["name"] in SKIP_NAMES or entry["lines"] < 5:
            continue
        by_hash[entry["hash"]].append(entry)
    for entries in sorted(by_hash.values(), key=lambda items: -items[0]["lines"]):
        if len(entries) < 2:
            continue
        first = entries[0]
        print(f"\n-- {first['lines']} lines, {len(entries)} copies:")
        for entry in entries:
            print(f"   {entry['path']} :: {entry['kind']} {entry['name']}")

    print("\n== same def name defined in multiple non-test modules ==")
    by_name = defaultdict(set)
    for entry in defs:
        if entry["name"] not in SKIP_NAMES:
            by_name[entry["name"]].add(entry["path"])
    for name, paths in sorted(by_name.items()):
        if len(paths) > 1:
            print(f"   {name}:")
            for path in sorted(paths):
                print(f"     - {path}")

    print("\n== zero-reference top-level defs (name appears nowhere else) ==")
    for entry in defs:
        name = entry["name"]
        if name.startswith("__") or name in SKIP_NAMES or len(name) < 5:
            continue
        occurrences = len(re.findall(rf"\b{re.escape(name)}\b", corpus))
        own = len(re.findall(rf"\b{re.escape(name)}\b", own_text[name]))
        if occurrences - own <= 0:
            print(f"   {entry['path']} :: {entry['kind']} {entry['name']} ({entry['lines']} lines)")

    print("\n== deepest nesting (top-level defs) ==")
    for entry in sorted(defs, key=lambda item: -item["depth"])[:12]:
        print(f"   depth {entry['depth']}: {entry['path']} :: {entry['name']} ({entry['lines']} lines)")


if __name__ == "__main__":
    sys.exit(main())
