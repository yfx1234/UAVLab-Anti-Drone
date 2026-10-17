# Session management, depth repair and offline time implementation plan

> Execute in parallel by owned files; one complete review after integration.

**Goal:** Time-based scene naming/sorting with rename/delete, credible ZED depth videos, local folder reveal and refined operator UI.
**Architecture:** Immutable UUID storage with editable display-name sidecar; UTC creation metadata from PC/monotonic anchor, legacy manifest time as uncertain fallback. Board supports guarded full-scene deletion, panel serializes scene operations. Native SDK depth visualization diagnosed independently. Root time correction is optional privileged fixed helper; no clock changes while camera owns it.
**Spec:** User request2026-10-11; delete entire board scene, keep downloaded PC copies with explicit UI notice. New names recording time, newest first, rename never reorders. PC/board clocks and RTC are separate: rootless PC anchor fixes application timestamps while privileged helper corrects Linux/RTC when installed. No claim of battery retention without power-cycle evidence.

- [x] Tests RED then implement catalog: timestamp order/default name, rename stability, active/symlink/path/confirmation deletion guards.
- [x] UTC start/monotonic metadata and PC clock anchor, status offset; retain old camera timestamps unchanged.
- [x] Local media folder reveal with registered-path guard; prevent scene deletion during downloads/exports/extraction.
- [x] Diagnose SDK depth from source/logs/actual file, repair/expire defective depth cache; verify nonconstant imagery on board replay.
- [x] Operator UI using impeccable scoped polish; desktop/mobile batched check, one detector, one repair if needed.
- [x] Board RTC/time diagnose; prepare minimal privileged solution only after safe-stop. Validate within available authorization/privileges, report unsolved hardware retention.
- [x] Build/deploy/tests/real renamed fixture/delete fixture/folder reveal/depth playback; do not delete user recordings; docs/review/archive/evidence.

## Completion evidence
PC: 33 distinct repository tests (32 pass, 1 Windows skip), plus 2 panel tests pass. Board: clean native rebuild after correcting UTC; colcon reports 67 tests, 0 errors/failures/skips. Deployed source and installed root helper hashes match. Real SVO depth conversion/download/browser playback is 1920x1080, with native VIEW::DEPTH and positive metric samples. Rename preserves time order; wrong confirmation rejected; only disposable no-camera scene fixtures deleted. New real recording is first, UTC name within one second of PC start. System UTC and RTC0 write succeed; actual camera lock blocks clock correction. PC folder button launches Explorer; native window observation is unavailable. Offline/power-loss RTC retention, real quad accuracy and future NX 30fps remain pending user/hardware tests. Camera left stopped.
