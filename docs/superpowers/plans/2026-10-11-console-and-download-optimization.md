# Operator console and download optimization plan

**Goal:** Stop layout movement as YOLO counts change, substantially redesign daily operations, remove explanatory and frame-extraction UI, and improve measured download bottlenecks without disrupting capture.

**Architecture:** Two view-only workspaces share unchanged camera/recording state. A fixed target region scrolls internally. Downloader reuses fresh existing supervisor state rather than creating repeated SSH connections; stale/unknown state keeps conservative transfer limits. PC video preparation is selected by verified codec/compatibility and keeps capture dimensions/timing and file identity.

**Spec:** User request 2026-10-11 and attached running-YOLO screenshot. Current board running RGB15fps, detector paused and no recording at inspection; preserve its run/profile and restore any temporary detector test state.

## Constraints and owned files

- Frontend agent owns panel/index.html plus minimal product/design records and meaningful frontend behavior checks.
- Transfer agent owns panel/session_manager.py, panel/board_client.py, panel/dashboard.py and transfer tests.
- Media agent owns panel/media_library.py and media performance tests.
- No board source, camera-driver, clock, power or network changes. Offline first export remains unavailable while camera is owned; cached video download can coexist.
- Capture defaults remain1920x1080/15fps; currentYOLO engine input unchanged.
- Stop controls remain usable during downloads. New view tabs do not stop capture or recording.
- Remove extractFrames/extractFps from UI and event handlers; maintain engineering capability outside this daily interface.
- Keep deletion scope and actionable errors; move documentation/qualification prose to operator guide.

## Steps

- [x] Record current run/board state and download stage baseline; do not stop the user's camera for performance measurements.
- [x] Read request-specific impeccable new-work/Operate/craft guidance; build a coherent replacement workstation hierarchy and record the chosen direction.
- [x] Implement fixed-height targets, internal scroll, stable scrollbar and sticky headers; numeric fields/column widths stable across empty and populated data.
- [x] Diagnose network/SSH overhead and local encoding independently, then implement only evidenced improvements with RED/GREEN regressions.
- [x] Verify cancellation/resume/hash, source replacement, stale state limits, rename/path safety and independent preview/stop responsiveness.
- [x] Run full PC suite and one whole-change review; repair actual findings once.
- [x] Deploy PC source with board camera run unchanged; verify connection recovery, RGB/YOLO live data, view switching, targets-to-empty geometry and cached downloads/playback.
- [x] Conduct one batched desktop/narrow visual check, one correction/confirmation at most; no open-ended styling loops.
- [x] Update current guide/evidence/affected knowledge; commit and verify new source archive.

## Completion limits

Downloads while the camera runs retain a conservative rate cap; do not call a faster link achievable unless measured. Hardware/H100 throughput and first depth/RGB conversion remain separate costs. Physical quad accuracy, long stress, RTC power retention and future NX30fps remain separate pending tests.

## Verified outcome (2026-10-12 client date)

68 suite tests: 67 passed, 1 Windows symlink skip; 2 panel tests passed. One whole review closed; one visual repair and final 1440/640 confirmation batch completed. Live target geometry stayed 264 px for 0 and 8 targets. Cached RGB download completed with YOLO running. Original live deployment preserved its run; by the resumed turn the board had a new boot and stopped camera, which was preserved. Ten deployed panel files match source. See outputs v5 evidence and report for measurements and archive identity.
