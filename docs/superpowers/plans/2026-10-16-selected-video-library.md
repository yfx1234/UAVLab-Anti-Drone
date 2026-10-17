# Selected video library implementation plan

> **For agentic workers:** Use executing-plans to implement these tasks in this session.

**Goal:** Detect the selected local video, group media by recording time, name video types clearly, and clean only the authorized media.

**Architecture:** Keep board inference and existing models. Upload the exact selected MP4 to an isolated, temporary annotation job; verify its SHA256, infer that file only, and clean the temporary job. Persist source identity and model settings alongside the playable result. The scene registry supplies recording time regardless of download or generation time.

**Tech Stack:** Python, existing SSH/SFTP, ONNX Runtime/OpenCV, FFmpeg, vanilla JavaScript.

**Spec:** User request in this chat: selected-video detection; recording-time sorting; distinct custom/general/original/depth names; keep dataset scenes, remove other material and redundant downloaded segments.

## Global constraints

- Inspect and modify only perception/media functionality. Do not inspect the restricted module.
- Do not stop acquisition; the camera is currently stopped, but recheck before board inference or deletion.
- Preserve all five dataset scene originals on the board; remove local segments only after checking whole-video equivalence.
- Verify deployment baselines, back up code, test, commit, push, and verify remote HEAD.

## Review focus

- A different scene selected on the left must not change the chosen local video.
- Recent downloads/results of an old scene must stay grouped with that old scene.
- Renames, deletion and conversion must share the source scene lock.
- Failed upload/inference must clean temporary files without publishing partial video.
- Never remove the sole complete dataset copy or an active scene.

### Task 1: Selected-video inference

**Files:** `panel/annotation_preview.py`, `panel/annotation_transport.py`, relevant lines of `panel/dashboard.py` and `panel/index.html`, `board/zed_tracking/tools/annotate_session.py`, annotation tests.

- [x] Add failing backend/frontend tests proving selected source, upload digest, settings, cleanup and source-scene reservation.
- [x] Implement `AnnotationPreview.run(params, progress)`, accepting `relative`, `model_id`, `confidence`; derive actual scene from media listing. Add `AnnotationTransport.upload(job_id, stream, size, progress)` and `cleanup(job_id)`.
- [x] Add board CLI `--input-job` plus required SHA256, isolated under `perception_annotation_jobs/<id>/`; keep legacy scene CLI for scripts.
- [x] Run annotation tests and verify input frames exactly match output frames.

### Task 2: Recording time and names

**Files:** `panel/media_library.py`, media tests.

- [x] Add failing tests covering reversed file mtimes, late annotation, linked preview and model/type names.
- [x] Add `scene_created_at_unix_s` and readable `type_label`/source scope; retain original recording creation timestamps. Sort by scene recording time then stable scope/type.
- [x] Run media and frontend regressions.

### Task 3: Cleanup and delivery

**Files:** task-local audit artifacts outside Git, delivery document in `docs/`.

- [x] Inventory explicit PC/board media roots, cross-check dataset labels, record a concrete deletion manifest.
- [x] Fully decode whole dataset videos and compare per-frame decoded hashes against concatenated segments before removing redundant local videos.
- [x] Delete non-dataset board scenes through the existing guarded API, and scoped PC files/folders after validating absolute roots; preserve dataset raw recordings and metadata.
- [x] Re-list preserved scenes, verify files, run relevant tests and scoped repository check; deploy backed-up code, smoke test temporary input, push and verify remote.

## Execution ledger

Completed inline with existing authorization to update/deploy/push this project. The selected-file inference and media identity tests failed before implementation and pass after it. Both deployed model families passed real SFTP/inference smoke checks. Cleanup followed an explicit manifest after four full decoded-frame equivalence checks. One independent review finding (a hidden cleanup warning) was reproduced with a failing test and fixed; 43 related tests pass, with only the named pre-existing out-of-scope error in the full suite. See `docs/所选视频检测与数据集清理_2026-10-16.md` for delivery evidence.
