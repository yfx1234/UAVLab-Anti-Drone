#!/usr/bin/env python3
"""Export an Ultralytics-managed TensorRT INT8 engine without replacing FP16 artifacts."""

import argparse
import json
import shutil
from pathlib import Path

import tensorrt as trt
from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--workspace", type=float, default=2.0)
    parser.add_argument("--device", default="0")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    data = args.data.expanduser().resolve()
    if output.suffix != ".engine":
        raise ValueError("--output must end in .engine")
    if output.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    staged_weight = output.with_suffix(".pt")
    if staged_weight.resolve() != source:
        shutil.copy2(source, staged_weight)

    model = YOLO(str(staged_weight))
    exported = Path(
        model.export(
            format="engine",
            imgsz=args.imgsz,
            batch=1,
            dynamic=False,
            half=False,
            int8=True,
            data=str(data),
            fraction=1.0,
            workspace=args.workspace,
            device=args.device,
            nms=False,
        )
    ).resolve()
    if exported != output:
        shutil.move(str(exported), output)

    logger = trt.Logger(trt.Logger.ERROR)
    runtime = trt.Runtime(logger)
    serialized = output.read_bytes()
    engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None and len(serialized) > 8:
        metadata_length = int.from_bytes(serialized[:4], byteorder="little", signed=True)
        if metadata_length > 0 and 4 + metadata_length < len(serialized):
            metadata = json.loads(serialized[4:4 + metadata_length].decode("utf-8"))
            serialized = serialized[4 + metadata_length:]
            output.with_suffix(".metadata.json").write_text(
                json.dumps(metadata, indent=2), encoding="utf-8"
            )
            output.write_bytes(serialized)
            engine = runtime.deserialize_cuda_engine(serialized)
    if engine is None:
        raise RuntimeError("TensorRT could not deserialize exported engine")
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
    print(f"EXPORT_YOLO26_INT8_ENGINE_PASS path={output} bytes={output.stat().st_size}")
    print(f"tensors={tensors}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
