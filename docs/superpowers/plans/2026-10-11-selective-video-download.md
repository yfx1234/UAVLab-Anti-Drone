# Selective video download and operator UI implementation plan

> **For agentic workers:** Use executing-plans inline, one final branch review. Checkboxes track task completion.

**Goal:** Separate RGB, optional depth visualization and original stereo downloads, preserve native resolution/timing, accelerate idle transfers, remove engineering-only UI.
**Architecture:** SVO remains the authoritative stereo source. Idle-only board conversion streams directly to videos without PNG intermediates; selected files transfer via bounded SFTP reads. PC handles viewing/organization and PNG extraction. Calibration/model validation/diagnostics remain SSH-accessible.
**Tech Stack:** ROS2/C++ ZED SDK/OpenCV; Python Paramiko/FFmpeg; existing loopback UI.
**Spec:** User request 2026-10-11: PC views/organizes/labels/trains; board may convert while idle. Raw downloaded RGB must remain1920x1080; target future NX30fps. Depth visualization off by default, no live depth preview.

## Global Constraints
- Current Nano remains default15fps; configurable15/30 for measured tests, never claim actual30 until recorded/playback evidence.
- Capture and offline export cannot overlap. No existing recordings deleted. Current camera run restored after necessary idle verification.
- RGB MP4 is a full-resolution viewing derivative; SVO separately offered as original stereo source. Depth colored MP4 is not metric depth data.
- Downloads select file kinds; no automatic logs/calibration/full-folder transfer. Auto speed uses256KiB/s during capture, no intentional cap while stopped, actual throughput measured.
- Scene finalization, source checksum, cancellation/resume, bounded queues and disk reserve retained.

## Review Focus
- Capture starts during fast transfer: downgrade within bounded poll/batch interval.
- Old/historical recordings lacking integrity flags: don't falsely certify or silently alter user data.
- Optional depth fails/missing GPU model: never present partial output as successful or download obsolete file.
- Wrong FPS or padded model resolution: output native capture time and dimensions; don't imply interpolated frames are new captures.
- Browsers retain removed-control scripts or stale auth after backend restart: real UI reload/test and visible paths.

## Tasks
1. [x] Selective SFTP: tests for selection excluding logs/unrequested exports, idle/active speed, resume and disk reserve. Implement session_manager + dashboard actions, prove failed tests then pass.
2. [x] Streaming native RGB/depth export: test SDK CLI native dimensions/playback/timestamps and unchanged PNG mapping; implement C++video mode + export_session orchestration/cache. Depth off unless requested.
3. [x] Operator UI: RGB/depth/images/original stereo downloads and local viewing; remove calibration/diagnostics/model-tests/board export buttons. Frontend browser check desktop/narrow; no stale references.
4. [x] Configurable captureFPS and measured30fps RGB trial after releasing current idle run; keep15 unless measurement qualifies. YOLO1920 estimate distinguishes pixel workload from empirical latency.
5. [x] Deploy explicit files/build/test; actual selected download/depth/video playback + transfer speed; fresh final review once; docs/evidence/source commit. Restore user's RGB run paused YOLO without starting recording.

## Completion evidence
Current Nano30fps trial did not qualify:18.2029 uniqueHz;15fps remains default. Board67 tests pass; PC24 tests (1 Windows skip) plus2 panel tests pass. Real typed downloads, native1920x1080 browser playback,65 local PNGs, dynamic transfer switch and restart recovery verified. Reviewer report retained separately; one repair pass completed. User camera restored RGB15, YOLO model bound and paused, no recording. Long-duration/NX/real-target testing deferred.
