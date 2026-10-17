# Beijing scene names and local media library implementation plan

> **For agentic workers:** Execute the authorized request in parallel by owned files, then run one whole-change review and bounded visual verification.

**Goal:** Compare exported depth with the supplied ZED viewer image, improve the operator console, synchronize editable local folder names, order by actual recording time, and clean authorized obsolete recordings.

**Architecture:** Board UUID identity stays immutable; default display names use UTC+8. A PC scene registry stores immutable ID, display name, creation time, uncertainty and a safe actual folder name. Explicit rename relocates registered local material and reports failures; downloads/extraction/playback resolve through that registry. Timestamp sorting does not depend on display names or file modification after conversion.

**Tech Stack:** Existing Python HTTP panel/SFTP, native ZED SDK exporter, plain HTML/CSS/JS, Windows Explorer.

**Spec:** User request 2026-10-11, supplied Depth Viewer photo and explicit deletion authorization for old manual scenes and unnecessary test scenes.

## Global constraints

- Current Nano remains the collection board; defaults remain 1920x1080/15fps. No NX performance claim.
- Display dates use Asia/Shanghai (UTC+8), never browser zone or hard-coded PC+12h.
- Registered board UUIDs never change. PC folders really follow display-name edits, with safe characters and a stable ID suffix to disambiguate collisions.
- Preserve capture timestamps and original SVO. Do not fill invalid depth pixels to mimic the reference image.
- Inspect current camera/preview ownership before any offline ZED operation; do not stop a user-owned Viewer automatically.
- Deletion scope is explicit in the cleanup inventory; check resolved board/PC targets before each whole-directory removal.
- Reuse incumbent blue/white UI; one batched desktop/narrow visual review, one correction/confirmation at most. Do not repeat the prior context/detector runs for this session.

## Review focus

- Windows unsafe/reserved/duplicate/Unicode names: valid display names produce safe unique local paths, ID associations survive reload.
- Rename while a download/extraction is queued or video handle is open: reject concurrent work; report a filesystem refusal without claiming full success, support retry.
- Folder migration and previously downloaded PC-only scenes: preserve files/hash mappings and persistent scene metadata; no implicit data deletion.
- Sorting: rename/convert/download must not promote a scene; video segments stay ordered within source recording time.
- Depth reference: different range, rendering or scene settings must be evidenced; no assertion of exact viewer equivalence without matching parameters.

### Task 1: Board Beijing names and cleanup inventory (root)

Files: board/zed_tracking/tools/session_catalog.py, tests/test_session_catalog.py; isolated work scripts and evidence.

- [x] Add regression for default UTC+8 name and unchanged timestamp ordering; verify RED.
- [x] Format default name with UTC+8 and explicit 北京时间; renamed test names remain independent.
- [x] Read manifests/summary/local metadata of all scenes; classify retained validation fixtures vs authorized obsolete material.
- [x] Delete only explicit checked targets after checking active/finalizing state; record freed bytes and retained scenes.

### Task 2: PC registry, folder rename, temporal media ordering (media agent)

Files: panel/media_library.py, panel/session_manager.py, panel/dashboard.py, optional focused panel/scene_registry.py, panel/preview_recorder.py, dedicated tests.

Interfaces: media list rows provide session_id/display_name/created_at_unix_s/created_time_uncertain/kind/relative/url/name. Existing download returns folder/files/kind; resolution uses stable session ID. rename_session result reports board display_name and local rename status, with errors exposed to UI and retryable.

- [x] Write and run failing tests for real folder rename, unsafe-name/collision handling, timestamp sort after reversed names, reload persistence and locked-folder failure.
- [x] Add persistent registered ID-to-folder metadata; migrate existing UUID folders without losing files.
- [x] Integrate download destination, extraction/playback/reveal, same-scene task exclusion and explicit rename synchronization.
- [x] Persist preview creation/source metadata when available; sort previews by recording creation and segment index, flag legacy fallback.
- [x] Run affected tests and communicate final API to UI/root.

### Task 3: Console visual hierarchy and Beijing display (UI agent)

Files: panel/index.html only.

- [x] Read Operate/polish/craft-floor; retain product truth and blue/white identity.
- [x] Improve control density, media hierarchy and <=1099px layout, use native disclosures for optional detail.
- [x] Set all displayed time formatting to zh-CN/Asia/Shanghai and refresh media immediately after rename.
- [x] Validate script syntax and functional states; root conducts bounded actual browser checks after deployment.

### Task 4: Depth Viewer comparison (depth agent)

Files: export_svo_rgb.cpp/export_session.py/depth tests only if evidence calls for changes.

- [x] Inspect the supplied image, downloaded clip frames and SDK parameters; consult primary viewer/API sources.
- [x] Explain whether color/range/invalid regions match; compare identical SVO and calibrated settings where possible.
- [x] Implement only evidenced visualization changes after coordination, invalidate depth-only cache if rendering changes.
- [x] Build/replay/download/play the retained test clip and document limitations.

### Task 5: Integrate and deliver (root)

- [x] Run meaningful PC suite and required board build/tests, one whole-change review, fix actionable findings in one pass.
- [x] Deploy exact files, test rename -> real PC folder -> playback/extraction/reveal -> reverse names and chronological order.
- [x] Capture desktop/narrow UI together, correct confirmed issues in one batch, save final screenshot.
- [x] Update simple guide, detailed evidence, affected knowledge entries, commit and verify source archive.


## Completion evidence
All source changes deployed. PC 50 distinct tests:49pass/1Windows symlink skip, plus2 panel security tests pass; board6 new catalog regressions pass on Linux. Native C++ is unchanged from the prior verified67-test build. Cleanup removes11 board scenes (25.5326GiB) and5 PC folders, retaining3 named references. Actual UI rename of older scene to A_Codex proves chronological order independent of alphabet;75 files hashes unchanged after real Windows folder rename and restoration. All4 media HTTP range reads succeed. Same cabinet capture renders native yellow/green walls and orange/red cabinets, valid metric samples99.6615%; no depth code/color change warranted. One whole-change review completed; first-migration UI stale URL repaired with RED/GREEN regression. Desktop/narrow bounded verification and screenshot evidence accompany delivery. Camera left stopped; future NX30fps and real quad positioning/RTC retention remain untested.
