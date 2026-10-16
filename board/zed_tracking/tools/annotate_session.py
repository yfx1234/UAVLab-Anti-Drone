#!/usr/bin/env python3
"""检测预览：对电脑选中的视频跑指定模型，画框输出标注 MP4。

用法（板上）：
  python3 annotate_session.py --input-job <job_id> --input-sha256 <sha256> --model-id <engine_sha256>
兼容旧场次脚本：
  python3 annotate_session.py --session <session_id> --model-id <engine_sha256>

流程：
  1. 在 weights/candidates 里按 engine sha256 找到模型目录与边车 manifest，
     再按 source_onnx_sha256 找同目录 ONNX（端到端 [1,N,6] 布局）
  2. 依次读 perception_records/<sid>/exports/segment_*/rgb_video.mp4
  3. 与平台完全一致的预处理（RGB、/255、letterbox 114）+ onnxruntime 推理，
     后过滤同平台（class 0、score>=0.55、退化框剔除）
  4. 画框+分数，写 <session>/annotated_<tag>_<ts>.mp4
  5. stdout 每段/每 600 帧打一行 JSON 进度，最后一行 JSON 结果

注意：onnxruntime CUDA EP 按 fp32 跑（板上实跑引擎为 fp16），预览用途的
数值差异可忽略；精确逐帧效果以实机切换模型为准。
"""

import argparse
import glob
import hashlib
import json
import os
import re
import sys
import time
import uuid
import math

import cv2
import numpy as np

WEIGHTS_ROOT = os.path.expanduser(
    "~/Jetson_ROS2_V0/weights/candidates")
RECORDS_ROOT = os.path.expanduser(
    "~/Jetson_ROS2_V0/perception_records")
JOBS_ROOT = os.path.expanduser("~/Jetson_ROS2_V0/perception_annotation_jobs")
CONF = 0.55          # 与 zed_perception_standalone decode_end_to_end 同阈值
CLASS_ID = 0         # 平台只放行 class 0


def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            data = f.read(chunk)
            if not data:
                break
            h.update(data)
    return h.hexdigest()


def find_model(model_id):
    for engine in glob.glob(
            os.path.join(WEIGHTS_ROOT, "**", "*.engine"), recursive=True):
        try:
            if sha256_file(engine) != model_id:
                continue
        except OSError:
            continue
        directory = os.path.dirname(engine)
        stem = os.path.splitext(os.path.basename(engine))[0]
        manifest_path = os.path.join(directory, stem + ".manifest.json")
        onnx_sha = None
        try:
            manifest = json.load(open(manifest_path, encoding="utf-8"))
            onnx_sha = manifest.get("source_onnx_sha256")
        except (OSError, ValueError):
            manifest = {}
        onnx_files = sorted(glob.glob(os.path.join(directory, "*.onnx")))
        chosen = None
        if onnx_sha:
            for candidate in onnx_files:
                try:
                    if sha256_file(candidate) == onnx_sha:
                        chosen = candidate
                        break
                except OSError:
                    continue
        if chosen is None and len(onnx_files) == 1:
            chosen = onnx_files[0]
        if chosen is None:
            raise SystemExit(json.dumps(
                {"ok": False, "error": "模型目录找不到对应 ONNX: " + directory}))
        return chosen, stem, manifest
    raise SystemExit(json.dumps(
        {"ok": False, "error": "未找到 model_id 对应的引擎"}))


def letterbox(frame, hw):
    h, w = hw
    sh, sw = frame.shape[:2]
    scale = min(w / sw, h / sh)
    nh, nw = round(sh * scale), round(sw * scale)
    resized = cv2.resize(frame, (nw, nh))
    canvas = np.full((h, w, 3), 114, np.uint8)
    top, left = (h - nh) // 2, (w - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized
    return canvas, scale, left, top


def input_job(job_id, digest):
    """Accept only an isolated upload with the exact digest submitted by the PC."""
    if not isinstance(job_id,str) or not re.fullmatch('[0-9a-f]{32}',job_id):raise ValueError('检测任务 ID 无效')
    if not isinstance(digest,str) or not re.fullmatch('[0-9a-f]{64}',digest):raise ValueError('源视频校验值无效')
    root=os.path.realpath(JOBS_ROOT);folder=os.path.join(root,job_id)
    source=os.path.join(folder,'source.mp4')
    if os.path.islink(folder) or os.path.islink(source) or os.path.realpath(source)!=source or not os.path.isfile(source):
        raise ValueError('所选视频不存在或路径无效')
    if sha256_file(source)!=digest:raise ValueError('所选视频传输校验失败，请重试')
    if os.path.lexists(os.path.join(folder,'output.mp4')):raise ValueError('检测任务输出已存在')
    return [source],os.path.join(folder,'output.mp4')


def main():
    parser = argparse.ArgumentParser()
    source_group=parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--session")
    source_group.add_argument("--input-job")
    parser.add_argument("--input-sha256")
    parser.add_argument("--model-id", required=True)
    parser.add_argument("--confidence", type=float, default=CONF)
    args = parser.parse_args()
    if (args.session is not None and not re.fullmatch(r"[a-zA-Z0-9_-]{1,80}", args.session)) or \
            not re.fullmatch(r"[0-9a-f]{64}", args.model_id):
        raise SystemExit(json.dumps({"ok": False, "error": "参数无效"}))
    if not math.isfinite(args.confidence) or not .25 <= args.confidence <= 1:
        raise ValueError("置信度阈值需在 0.25–1 之间")

    onnx_path, tag, manifest = find_model(args.model_id)
    if args.input_job:
        segments,job_output=input_job(args.input_job,args.input_sha256)
        session_dir=os.path.dirname(job_output)
    else:
        session_dir = os.path.join(RECORDS_ROOT, args.session)
        segments = sorted(
            glob.glob(os.path.join(session_dir, "exports", "segment_*", "rgb_video.mp4")),
            key=lambda p: int(re.search(r"(\d+)$", os.path.basename(os.path.dirname(p))).group(1)))
    if not segments:
        raise SystemExit(json.dumps(
            {"ok": False, "error": "场次没有已导出的 RGB 视频，请重试板上视频准备"}))

    total_frames = 0
    for path in segments:
        probe = cv2.VideoCapture(path)
        try:
            if not probe.isOpened(): raise ValueError("无法读取视频分段：" + path)
            count = int(probe.get(cv2.CAP_PROP_FRAME_COUNT))
            if count <= 0: raise ValueError("视频分段没有有效帧：" + path)
            total_frames += count
        finally: probe.release()
    print(json.dumps({"phase_code": "model_load", "stage": "加载检测模型", "total_frames": total_frames}), flush=True)

    import onnxruntime as ort
    providers = []
    available = ort.get_available_providers()
    if "CUDAExecutionProvider" in available:
        providers.append("CUDAExecutionProvider")
    providers.append("CPUExecutionProvider")
    session = ort.InferenceSession(onnx_path, providers=providers)
    inp = session.get_inputs()[0]
    shape = [d if isinstance(d, int) else -1 for d in inp.shape]
    if len(shape) != 4 or shape[0] != 1 or shape[1] != 3 or min(shape[1:]) <= 0:
        raise SystemExit(json.dumps(
            {"ok": False, "error": "ONNX 输入不是静态 [1,3,H,W]: %s" % inp.shape}))
    hw = (shape[2], shape[3])

    stamp = time.strftime("%Y%m%d_%H%M%S") + '_' + uuid.uuid4().hex[:8]
    output = job_output if args.input_job else os.path.join(session_dir,
                          "annotated_%s_%s.mp4" % (tag, stamp))
    writer = None
    frames = detected_frames = 0
    infer_ms = []
    started = last_report = time.monotonic()
    print(json.dumps({"phase_code": "inference", "stage": "逐帧检测", "frames": 0, "total_frames": total_frames}), flush=True)
    for segment_index, seg_path in enumerate(segments, 1):
        cap = cv2.VideoCapture(seg_path)
        if not cap.isOpened():
            raise SystemExit(json.dumps(
                {"ok": False, "error": "无法读取 " + seg_path}))
        fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if writer is None:
            writer = cv2.VideoWriter(output, cv2.VideoWriter_fourcc(*"mp4v"),
                                     fps, (width, height))
            if not writer.isOpened(): raise ValueError("无法创建标注视频，请检查板上磁盘空间")
            output_spec = (fps, width, height)
        elif output_spec != (fps, width, height):
            raise ValueError("场次分段的视频尺寸或帧率不一致，未发布不完整结果")
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                break
            frames += 1
            canvas, scale, pad_x, pad_y = letterbox(
                cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB), hw)
            blob = (canvas.astype(np.float32) / 255.0
                    ).transpose(2, 0, 1)[np.newaxis]
            t0 = time.perf_counter()
            rows = session.run(None, {inp.name: blob})[0][0]
            infer_ms.append((time.perf_counter() - t0) * 1000.0)
            drew = False
            for row in rows:
                x1, y1, x2, y2, score, cls = row
                if cls != CLASS_ID or not np.isfinite(score) or score < args.confidence or not x2 > x1 or not y2 > y1:
                    continue
                if not all(np.isfinite(v) for v in (x1, y1, x2, y2)):
                    continue
                ox1 = (x1 - pad_x) / scale
                oy1 = (y1 - pad_y) / scale
                ox2 = (x2 - pad_x) / scale
                oy2 = (y2 - pad_y) / scale
                ox1 = int(max(0, min(width - 1, ox1)))
                oy1 = int(max(0, min(height - 1, oy1)))
                ox2 = int(max(0, min(width - 1, ox2)))
                oy2 = int(max(0, min(height - 1, oy2)))
                cv2.rectangle(frame_bgr, (ox1, oy1), (ox2, oy2), (0, 237, 120), 2)
                cv2.putText(frame_bgr, "%.2f" % score,
                            (ox1, max(14, oy1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 237, 120), 1,
                            cv2.LINE_AA)
                drew = True
            detected_frames += bool(drew)
            writer.write(frame_bgr)
            if time.monotonic() - last_report >= 1 or frames == total_frames:
                elapsed = time.monotonic() - started
                print(json.dumps({"phase_code": "inference", "stage": "逐帧检测", "frames": frames,
                                  "total_frames": total_frames, "segment": segment_index, "segments": len(segments),
                                  "eta_s": max(0, (total_frames - frames) * elapsed / frames),
                                  "detected_frames": detected_frames}),
                      flush=True)
                last_report = time.monotonic()
        cap.release()
    if writer is not None:
        writer.release()
    if frames != total_frames:
        raise ValueError("视频读取提前结束：预期 %s 帧，实际 %s 帧；未发布不完整结果" % (total_frames, frames))
    if not frames or not os.path.getsize(output): raise ValueError("标注视频为空")
    print(json.dumps({
        "ok": True, "output": output, "frames": frames,
        "detected_frames": detected_frames,
        "avg_inference_ms": round(float(np.mean(infer_ms)), 1) if infer_ms else None,
        "model_tag": tag,
        "input_nchw": [1, 3, hw[0], hw[1]],
        "conf": args.confidence,
        "source_sha256": args.input_sha256 if args.input_job else None,
    }), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(json.dumps({"ok": False, "error": str(error)}, ensure_ascii=False), flush=True)
        sys.exit(1)
