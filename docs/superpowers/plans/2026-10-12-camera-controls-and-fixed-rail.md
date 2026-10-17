# Camera controls and fixed operator rail

User scope: answer launcher and field portability questions; fix scrolling control rail; simplify to RGB startup plus YOLO buttons; remove PC preview recording UI; expose valid resolution/FPS pairs; improve daily operation without explanatory UI clutter.

## Implementation decisions

- Preserve the existing workstation visual identity. Sticky desktop rail has its own bounded scroll; narrow screens keep collapsible controls.
- Default HD1080/15fps, RGB displayed at startup, detector/depth computation paused. Keep neural depth initialized so the YOLO button can enable both detection and depth without reopening the camera or breaking a recording. Pause disables depth work.
- Offer actual ZED X Mini modes HD1080, HD1200 and SVGA, with paired hardware FPS choices. Above 15fps remains experimental on this Nano; configured rate is separate from actual achieved rate. No YOLO engine resizing.
- Remove profile selector and PC preview recorder controls from daily UI; retain backend compatibility/old local videos. Provide recovery stop only if an existing PC recorder is active.
- Add useful recording elapsed time/estimated remaining disk time, full-screen live image, and dynamic source resolution labels only if backed by existing state/metadata.
- Firmware, power, camera drivers, IP addressing and fixed H100 link remain unchanged. No Internet assets in the page. Explain portability from code and installed resources; do not promise RF range or new-environment recognition accuracy.

## Work

- [x] Capture existing state/source identity and verify official camera mode constraints and runtime depth behavior.
- [x] Add meaningful tests for parameter validation/launch propagation and depth toggling policy, then implement backend/C++.
- [x] Refine rail, controls, live image and supported selectors; test actions and recording continuity.
- [x] Build/test/deploy with backups; perform short default/alternate-mode and YOLO toggle checks, restore equivalent initial running settings.
- [x] One desktop/narrow visual batch; one repair/confirmation maximum.
- [x] Update guide, confirmed knowledge, evidence and verified source archive; commit.

## Source constraints

Official ZED X specifications: HD1080/HD1200 15/30/60fps, SVGA 15/30/60/120fps. Resolution/FPS change requires closing/reopening. RuntimeParameters toggles computation only when a depth mode was initialized; NONE cannot enable depth at runtime.

Actual Nano capacity, physical quad precision, NX30fps and long stress remain separate tests.

Verified: PC74 tests (73 pass/1 Windows skip), panel2, native7 groups; one continuous SVO verified across RGB/YOLO toggles; SVGA30 and HD1200/15 short previews; prior HD1080/15 detector state restored. One desktop/narrow visual batch plus one confirmation, no horizontal overflow, sticky rail retained its 20px header gap.
