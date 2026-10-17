# SDD ledger — plan: outputs/ZED感知与采集平台_详细代码实施计划_20261011.md

User authorization: implement on current Nano 8GB; NX comes after data collection.
Baseline: 9043a30, source-only copy from the board plus existing local dashboard, isolated branch perception-nano.
Ruling: use this isolated source repository rather than alter the board's master or unrelated referee changes. Deploy only explicit files with an archive first.
Ruling: add a standalone executable in zed_tracking, reusing acquisition/TensorRT/geometry helpers. Keep legacy control executable intact to preserve the user's existing code; new launcher/panel cannot invoke it.
Pre-flight 1→2→7: board lifecycle revision belongs to supervisor; node configuration revision is separate. Requests carry boot/run identity and expected revision.
Pre-flight 2→4→5: Camera belongs only to acquisition thread, owned immutable frame data; SDK recording survives receiver/SSH disconnect.
Pre-flight 3→5→8: frame_key binds JPEG, observations and recording logs; stale/image rollover cannot reuse another frame's boxes.
Ruling: vendor the pinned MIT JSON single header; current board has sample copies but no system nlohmann_json package, avoiding system package changes.
Ruling: actual SDK 5.5 RecordingStatus uses status/number_frames_ingested/number_frames_encoded; do not invent is_frame_recorded.
Tasks: 1-9 in progress. Hardware acceptance must explicitly separate code/short-run validation from real-target accuracy and 2-hour soak.

Implementation checkpoint 2026-10-11:
- Tasks 1-8 code implemented. Task 9 short-run integration under way; actual quad accuracy and full endurance acceptance remain pending.
- Board current clean isolated-prefix build: 2 packages; colcon tests: 67, zero errors/failures/skips.
- Windows platform suite: 13 total, 12 passed, Linux symlink case skipped; separate HTTP suite 2 passed. Linux symlink isolation test passed previously, rerun at final.
- RED→GREEN regressions: sparse/mixed/background depth; foreground cluster selection; model content SHA identity; dataset grouping; PC video disk guard; SDK helper lost ROS shared-library environment (real model-manifest command failed then passed after fixed setup).
- Ruling: latest-slot owned frame references replace an explicit pool of two — memory remains bounded through capture/inference/preview references; cost if wrong: higher peak memory; memory reserve guard and actual Nano load verification required.
- Ruling: raw SVO uses SDK LOSSLESS, rotated every 60s, exported while idle — Nano hardware H264 was unverified/corrupt historically; cost: large NVMe footprint and segment rotation gaps. No background data deletion.
- Ruling: UI is a new perception-only platform over SSH fixed RPC, with independent original SVO and shared JPEG preview — legacy flight launch sources preserved; cost: separate entry point must be used.
- Ruling: SFTP is limited to 256KiB/s without dynamic congestion control — combined budget remains below configured H100 link; cost: slow transfer and additional disk/network contention; large downloads recommended after capture.
- Ruling: the formal 30-minute performance and 2-hour soak are not substituted by short runs — deliver usable capture platform with explicit pending qualification; cost: long-session reliability unproven. Real-quad tests postponed by user 2026-10-11.
- No full-plan completion claim yet. Final review and live evidence must be recorded before delivery.
Final review: work/final-review.md, six Important findings, zero Critical or Minor. One fix pass in progress.
- verify mapping destructive regression actual SDK failed then passed.
- missing segments/frame-tail/metadata regression failed missing helper then passed; strict validation now uses submitted count plus capture/playback timestamp SHA256 per segment. SDK recording counters lagged 3 frames in actual short RGB test (73 SVO vs 70 status), so they are diagnostic rather than authoritative.
- model_change_allowed and run-monotonic track reset regression failed compile before helpers; now pass pure C++ test.
- SessionWorkCounter delayed-work barrier regression failed before class; now pass; inference/log work tracked and drained before scene close; timed-out finalization remains explicitly incomplete.
- frame config is bound at acquisition; scene full calibration snapshot plus SHA256 is immutable. Actual before/after-calibration scene test under way.
- Extra correctness regression: offline MP4 requested fps > source fps must not accelerate motion, timestamp frame repetition RED→GREEN passed. Video playback count checked. Observation lookup moved to disk SQLite to bound offline RAM.
- Nano15fps parallel180s passed: record survives receiver disconnect and same-model pause/resume;10 lifecycle starts/stops pass; original SVO2688 frames across4 segments verified, local1183 received frames/5 gap segments; this bench predates final metadata strictness.
- cancellation/resume/source hash passed on labelled1MiB transfer fixture; Linux symlink guard rerun passed.
Ruling: current Nano capture default15fps (all acquired frames eligible for YOLO), rather than planned30fps; actual1080p30+LOSSLESS dropped15–18Hz. Cost: lower temporal sampling than30fps; preserving1080p detail takes priority for current data collection.
Ruling: SDK per-grab recording status is provisional, not proof that that specific image reached disk; only full playback timestamp reconciliation can mark complete. Cost: older scenes without verified finalization remain incomplete until individually audited.
Formal30min/2h and realquad acceptance remain pending; no representation of completion for those gates.

Final software delivery checkpoint 2026-10-11:
- Final fresh suites: board colcon67/67; Windows14 pass+1 Linux-only skip; HTTP2/2. No errors/failures.
- Actual finalization test PASS: RGB127 and perception235 captured/playback timestamps reconcile; full immutable calibration hash; no later tail writes; recording identity guard; RGB/YOLO MP4 decode and timing PASS.
- Deployed source compared to isolated repository with LF normalization; dashboard UI connected, actual model preview15Hz observed and screenshot saved; camera explicitly released.
- Source-only branch retained for user operation and rollback, no remote push or merge required by authorized file deployment. Deliver docs/ZIP/evidence to current outputs.
- Task9 software/short bench completed; formal endurance/ground-truth acceptance explicitly pending.
Final: minor (deferred): OpenSSL3 incremental SHA256 API deprecation warning; build and checks pass; migrate EVP later.
