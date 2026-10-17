# Generic Sky Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 增加独立、可联合YOLO运行的天空通用检测，明天可现场比较并支持远处尺寸估距。

**Architecture:** C++原图局部对比度产生候选，IMU旋转补偿与连续确认形成通用轨迹；现有单相机拥有者复用定位、预览和SVO。来源/评分/尺寸先验随帧保存，PC提供独立按钮。

**Tech Stack:** C++17、OpenCV、ZED SDK5.5、ROS2 Humble、Python、本机单文件HTML。

**Spec:** docs/generic-sky-spec.md

## Global Constraints

- 当前Nano默认1080p/15fps；唯一ZED拥有者；RGB默认，不自动录制。
- 通用检测不加载TensorRT，不继承YOLO框标定；深度/尺寸估距/仅方位保持分开。
- 通用候选最多24个，轨迹最多64个；图像时刻IMU无效时不伪造补偿。
- 前端保持固定高度目标表；不增加标定、抽帧、工程说明入口。
- 验证后合并并推送GitHub；用户已授权同步。实物和NX验收未完成不得宣称通过。

## Review Focus

- 相机快速转动、IMU缺失/旧时间戳：降级明确，不输出陈旧位置。
- 云边、低对比和瞬时噪声：允许无目标，不强选最大块，不虚称四旋翼。
- YOLO与通用切换/联合时录像：相机与SVO连续，不加载第二台相机或重复显示目标。
- 通用框与YOLO标定不一致：只用未标定尺寸先验，不误标可靠深度。
- 当前Nano资源与取消/停止：推理队列有界，下载进入保守档，停止相机可恢复。

---

### Task 1: Spatial detector and motion-aware tracker

**Files:** Create `board/zed_tracking/include/zed_tracking/camera_motion.hpp`, `generic_sky_detector.hpp`, `board/zed_tracking/test/test_generic_sky.cpp`.

**Interfaces:** `CameraMotionSample` stores valid, timestamp_ns, quaternion_wxyz; `SkyIntrinsics` stores fx/fy/cx/cy/width/height. `GenericSkyDetector::detect(cv::Mat, GenericSkyOptions)` returns candidate detections. `GenericSkyTracker::update(candidates, intrinsics, image_seconds, motion, confirmation_frames)` returns confirmed tracks; `confirm_depth(track_id, accepted)` gates measured depth independently. `sky_options(string)` validates high/standard/low.

- [ ] Write native synthetic tests for blank/smooth sky, dark and bright small targets, near target, hovering, transient noise, camera rotation, missing/stale IMU, no stale output, bounded candidates and tracks.
- [ ] Run isolated board native fixture before headers exist. Expected: explicit missing-detector failure.
- [ ] Implement full-resolution small-scale morphology plus half-resolution large-scale morphology, bounded components, conservative association and frame confirmation.
- [ ] Run the same fixture. Expected: all behavioral assertions pass and benchmark prints measured 1080p timings.
- [ ] Commit algorithm and tests.

### Task 2: Camera owner, RPC and localization integration

**Files:** Modify `src/zed_source.hpp`, `src/zed_perception_standalone.cpp`, `tools/perception_supervisor.py`, CMake and ROS messages; create `tests/test_generic_configuration.py`.

**Interfaces:** RPC `set_generic_detector` accepts enabled bool and optional sensitivity high/standard/low. Frame snapshots both detector flags, generic preset, nominal size and image-aligned camera orientation. Status exposes generic_detector_enabled/state, detector_generation and generic timings. Targets expose detector_source, score_kind and class_confirmed; generic class_id=-1 and IDs use high bit.

- [ ] Add failing RPC tests: valid independent toggle without model, unknown keys/preset/type rejection. Add range regression asserting 13px box at fx1266/size0.3 gives approximate >20m; clipped/sub8px boxes give bearing only.
- [ ] Run PC task tests. Expected: new generic action unsupported failure; unchanged range contract already passes.
- [ ] Integrate SDK IMU-to-camera extrinsic/fused pose, source-frame snapshots, independent state, shared depth and bounded inference queue. Save method comparison rows while recording and suppress duplicate fused observations.
- [ ] Fresh board build with new native fixture and PC tests. Expected: 6 native groups and all PC tests pass; no old flight dependency needed.
- [ ] Commit integration.

### Task 3: Panel operation and transfer protection

**Files:** Modify `panel/dashboard.py`, `panel/session_manager.py`, `panel/index.html`; create `tests/test_generic_panel.py`.

**Interfaces:** BOARD_ACTIONS includes set_generic_detector; new enable/disableGeneric buttons and sensitivity selector; viewYolo becomes common detection overlay. Generic source uses candidate score label and G track prefix; only fresh current-generation targets render. Generic active status always forces conservative transfer even when depth/status is otherwise incomplete.

- [ ] Add failing tests for generic RPC routing, active generic transfer limit, clean common overlay controls and freshness checks.
- [ ] Run targeted PC tests. Expected: missing generic action/transfer guard failures.
- [ ] Implement independent buttons/labels, source distinction, fixed-table rendering and conservative transfer policy.
- [ ] Run full PC suite and browser flows. Expected: no JS errors, clear RGB/generic/YOLO/combined states, unchanged fixed table and media library.
- [ ] Commit panel integration.

### Task 4: Deployment, live checks and field preparation

**Files:** Update operator/simple guides, `docs/generic-sky-field-test.md`, validation summary and knowledge base.

**Interfaces:** Deploy reviewed board packages with backups and sequential build; PC service retains existing data directory. Manual field guide covers raw sky recordings and comparing per-method logs, not automated reminder.

- [ ] Verify no recording is active, backup and deploy; build/tests must pass before replacing runtime binary.
- [ ] Run short RGB→generic→combined→YOLO→RGB sequence, including one SVO recording; inspect timestamps, unique camera owner, resource reserves and final SVO validation. Expected: camera/SVO continuous, no allocator/error loop, correct flags, IDs/source/score, source-frame geometry and conservative transfer.
- [ ] Verify PC UI after reload and source hashes, prepare tomorrow's instructions and report measured limits.
- [ ] Fresh independent whole-branch review; resolve important findings and repeat affected checks.
- [ ] Merge, push, confirm remote HEAD/tree and clean local status. Expected: GitHub matches deployed source; only actual tested outcomes reported.
