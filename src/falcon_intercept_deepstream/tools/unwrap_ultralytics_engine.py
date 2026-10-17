#!/usr/bin/env python3
"""Remove the Ultralytics JSON header and write a pure TensorRT plan for DeepStream."""

import argparse
import json
from pathlib import Path

import tensorrt as trt


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata-output", type=Path)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite {output}")
    data = source.read_bytes()
    if len(data) < 8:
        raise RuntimeError("engine file is too small")
    metadata_length = int.from_bytes(data[:4], byteorder="little", signed=True)
    if metadata_length <= 0 or 4 + metadata_length >= len(data):
        raise RuntimeError(f"invalid Ultralytics metadata length: {metadata_length}")
    metadata = json.loads(data[4:4 + metadata_length].decode("utf-8"))
    plan = data[4 + metadata_length:]

    logger = trt.Logger(trt.Logger.ERROR)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(plan)
    if engine is None:
        raise RuntimeError("TensorRT could not deserialize unwrapped plan")
    tensors = []
    for index in range(engine.num_io_tensors):
        name = engine.get_tensor_name(index)
        tensors.append(
            {
                "name": name,
                "mode": str(engine.get_tensor_mode(name)),
                "dtype": str(engine.get_tensor_dtype(name)),
                "shape": tuple(engine.get_tensor_shape(name)),
            }
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(plan)
    metadata_output = args.metadata_output or output.with_suffix(".metadata.json")
    metadata_output.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(
        f"UNWRAP_ULTRALYTICS_ENGINE_PASS path={output} bytes={len(plan)} "
        f"metadata={metadata_output}"
    )
    print(f"tensors={tensors}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
