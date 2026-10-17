#!/usr/bin/env python3
"""Summarize stable per-source FPS windows from a DeepStream node log."""

import argparse
import re
import statistics
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--source", choices=("search", "left", "right"), required=True)
    parser.add_argument("--skip", type=int, default=3)
    parser.add_argument("--min-samples", type=int, default=10)
    parser.add_argument("--min-mean", type=float, default=0.0)
    arguments = parser.parse_args()

    pattern = re.compile(rf"\b{re.escape(arguments.source)}_fps=([0-9]+(?:\.[0-9]+)?)")
    values = [
        float(match.group(1))
        for line in arguments.log.read_text(errors="replace").splitlines()
        if (match := pattern.search(line))
    ]
    stable_values = values[max(0, arguments.skip) :]
    if len(stable_values) < arguments.min_samples:
        print(
            "DEEPSTREAM_FPS_SUMMARY_FAIL "
            f"source={arguments.source} total_samples={len(values)} "
            f"stable_samples={len(stable_values)} required={arguments.min_samples}",
            file=sys.stderr,
        )
        return 1

    mean_fps = statistics.fmean(stable_values)
    if mean_fps < arguments.min_mean:
        print(
            "DEEPSTREAM_FPS_SUMMARY_FAIL "
            f"source={arguments.source} total_samples={len(values)} "
            f"stable_samples={len(stable_values)} skip={arguments.skip} "
            f"mean={mean_fps:.2f} required_mean={arguments.min_mean:.2f} "
            f"min={min(stable_values):.2f} "
            f"median={statistics.median(stable_values):.2f} "
            f"max={max(stable_values):.2f}",
            file=sys.stderr,
        )
        return 1

    print(
        "DEEPSTREAM_FPS_SUMMARY_PASS "
        f"source={arguments.source} total_samples={len(values)} "
        f"stable_samples={len(stable_values)} skip={arguments.skip} "
        f"mean={mean_fps:.2f} "
        f"min={min(stable_values):.2f} "
        f"median={statistics.median(stable_values):.2f} "
        f"max={max(stable_values):.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
