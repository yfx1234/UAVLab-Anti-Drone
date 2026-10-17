#!/usr/bin/env python3
"""Export a separate static end-to-end YOLO26 ONNX without overwriting source weights."""

import argparse
import shutil
from pathlib import Path

import onnx
from ultralytics import YOLO


def tensor_shapes(items):
    result = []
    for item in items:
        dimensions = [dimension.dim_value for dimension in item.type.tensor_type.shape.dim]
        result.append((item.name, dimensions))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--imgsz",
        type=int,
        nargs="+",
        required=True,
        metavar="SIZE",
        help="One square size or two values in HEIGHT WIDTH order.",
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--force", action="store_true")
    arguments = parser.parse_args()

    source = arguments.source.expanduser().resolve()
    output = arguments.output.expanduser().resolve()
    if output.suffix != ".onnx":
        raise ValueError("--output must end in .onnx")
    if not source.is_file():
        raise FileNotFoundError(source)
    if output.exists() and not arguments.force:
        raise FileExistsError(f"refusing to overwrite {output}; pass --force if intentional")

    if len(arguments.imgsz) == 1:
        height = width = arguments.imgsz[0]
        export_imgsz = height
    elif len(arguments.imgsz) == 2:
        height, width = arguments.imgsz
        export_imgsz = (height, width)
    else:
        raise ValueError("--imgsz accepts one value or HEIGHT WIDTH")
    if height <= 0 or width <= 0 or height % 32 or width % 32:
        raise ValueError("ONNX input height and width must be positive multiples of 32")

    output.parent.mkdir(parents=True, exist_ok=True)
    staged_weight = output.with_suffix(".pt")
    if staged_weight.resolve() != source:
        shutil.copy2(source, staged_weight)

    model = YOLO(str(staged_weight))
    exported = Path(
        model.export(
            format="onnx",
            imgsz=export_imgsz,
            batch=1,
            dynamic=False,
            simplify=False,
            opset=20,
            nms=True,
            device=arguments.device,
        )
    ).resolve()
    if exported != output:
        shutil.move(str(exported), output)

    graph = onnx.load(str(output), load_external_data=False)
    inputs = tensor_shapes(graph.graph.input)
    outputs = tensor_shapes(graph.graph.output)
    expected_input = [1, 3, height, width]
    if not inputs or inputs[0][1] != expected_input:
        raise RuntimeError(f"unexpected ONNX input shapes: {inputs}")
    if not outputs or outputs[0][1] != [1, 300, 6]:
        raise RuntimeError(f"unexpected end-to-end ONNX output shapes: {outputs}")

    print(f"EXPORT_YOLO26_ONNX_PASS path={output}")
    print(f"inputs={inputs}")
    print(f"outputs={outputs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
