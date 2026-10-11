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
