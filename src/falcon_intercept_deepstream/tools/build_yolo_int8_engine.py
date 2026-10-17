#!/usr/bin/env python3
"""Build a static TensorRT INT8 engine using DeepStream-compatible preprocessing."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import tensorrt as trt
import torch


class EntropyCalibrator(trt.IInt8EntropyCalibrator2):
    def __init__(self, image_paths: list[Path], cache_path: Path, height: int, width: int):
        super().__init__()
        self.image_paths = image_paths
        self.cache_path = cache_path
        self.height = height
        self.width = width
        self.index = 0
        self.device_input = torch.empty(
            (1, 3, height, width), dtype=torch.float32, device="cuda"
        )

    def get_batch_size(self) -> int:
        return 1

    def get_batch(self, names):
        del names
        if self.index >= len(self.image_paths):
            return None
        path = self.image_paths[self.index]
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"cannot read calibration image: {path}")
        source_height, source_width = image.shape[:2]
        scale = min(self.width / source_width, self.height / source_height)
        resized_width = max(1, round(source_width * scale))
        resized_height = max(1, round(source_height * scale))
        resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        x = (self.width - resized_width) // 2
        y = (self.height - resized_height) // 2
        canvas[y:y + resized_height, x:x + resized_width] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32) / 255.0
        self.device_input.copy_(torch.from_numpy(tensor).unsqueeze(0))
        self.index += 1
        if self.index % 50 == 0 or self.index == len(self.image_paths):
            print(f"INT8_CALIBRATION_PROGRESS {self.index}/{len(self.image_paths)}", flush=True)
        return [int(self.device_input.data_ptr())]

    def read_calibration_cache(self):
        if self.cache_path.is_file():
            data = self.cache_path.read_bytes()
            print(f"INT8_CALIBRATION_CACHE_LOAD bytes={len(data)} path={self.cache_path}")
            return data
        return None

    def write_calibration_cache(self, cache):
        self.cache_path.parent.mkdir(parents=True, exist_ok=True)
        self.cache_path.write_bytes(cache)
        print(f"INT8_CALIBRATION_CACHE_WRITE bytes={len(cache)} path={self.cache_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", type=Path, required=True)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--workspace-mb", type=int, default=2048)
    parser.add_argument("--optimization-level", type=int, default=3)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    onnx_path = args.onnx.expanduser().resolve()
    image_dir = args.images.expanduser().resolve()
    engine_path = args.engine.expanduser().resolve()
    cache_path = args.cache.expanduser().resolve()
    if engine_path.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite engine: {engine_path}")
    image_paths = sorted(image_dir.glob("*.jpg"))
    if not image_paths:
        raise RuntimeError(f"no calibration JPG images in {image_dir}")
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA-enabled PyTorch is required for calibration buffers")

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser_ = trt.OnnxParser(network, logger)
    if not parser_.parse(onnx_path.read_bytes()):
        errors = [str(parser_.get_error(index)) for index in range(parser_.num_errors)]
        raise RuntimeError("ONNX parse failed:\n" + "\n".join(errors))
    input_shape = tuple(network.get_input(0).shape)
    if input_shape != (1, 3, 1280, 1280):
        raise RuntimeError(f"expected static [1,3,1280,1280] input, got {input_shape}")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, args.workspace_mb * 1024 * 1024
    )
    config.builder_optimization_level = args.optimization_level
    config.set_flag(trt.BuilderFlag.FP16)
    config.set_flag(trt.BuilderFlag.INT8)
    config.int8_calibrator = EntropyCalibrator(
        image_paths, cache_path, input_shape[2], input_shape[3]
    )
    engine_path.parent.mkdir(parents=True, exist_ok=True)
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT returned no serialized INT8 engine")
    engine_path.write_bytes(serialized)
    print(
        f"BUILD_YOLO_INT8_ENGINE_PASS path={engine_path} bytes={engine_path.stat().st_size} "
        f"images={len(image_paths)} optimization_level={args.optimization_level}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
