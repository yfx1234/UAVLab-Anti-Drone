# 舵机工作台

## Scope and mode

- Target: `panel/servo_panel.html`, `panel/servo_panel.css`, `panel/servo_panel.js` and the standalone servo API.
- Mode: Operate. Extend the established UAVLab visual system in `DESIGN.md`; do not redesign the camera/data workbench.
- User: a lab operator at a normally lit indoor bench, looking between the computer and an independently powered servo.
- Main task: choose a position and behavior, verify PWM feedback, and assemble a repeatable motion sequence.
- Current hardware: I/O PWM MAIN5, function 302, logical slot 2. Read and verify configuration rather than hard-code successful status.

## Direction contract

- Chosen composition: two-column workbench. Position and sequence controls occupy the wide left column; connection, output feedback and a concise event history occupy the right.
- Inherit the neutral light ground, white surfaces, graphite primary actions, system UI typography, pill buttons and restrained semantic colors from the existing project.
- First viewport: one compact header with a persistent Stop action; editable target position with an estimated angle, familiar range control and behavior selector; an actual sequence table starts below. Device identity and measured PWM occupy the right column.
- Signature interaction: target and flight-controller PWM share a recent-history plot, with a dashed target trace and solid output trace. An empty connection shows no invented data.
- Motion: no page entrance choreography. Short control feedback transitions only, with reduced-motion support.
- Responsive: at tablet widths the device area flows below the controls; at phone widths it becomes one column. Only the sequence table may scroll horizontally.
- Avoid: cockpit ornament, fake dial hardware, decorative imagery, repeated metric cards, invented live values and technical command strings in the ordinary flow.

## Behavior

- The user explicitly requested selectable and editable motion behavior.
- Manual pulse: adjustable 0.2–30 seconds before releasing to the configured disarmed output.
- Hold: maintain the chosen position while the owning page renews its action session. Closing the page, losing the session or stale telemetry ends the test; each underlying flight-controller command still expires after at most one second.
- Sequence: editable points, move/hold timing, reorder, JSON import/export, repetition and completion policy (return or hold last point).
- A new page or a reload never silently resumes an earlier holding action.
- Distinguish target estimates, command acceptance and flight-controller feedback; no actual servo angle is claimed without a sensor.
- Keep current disarmed-only behavior. Do not arm the vehicle or write flight-controller parameters.

## Validation and reference

- Unit tests use fake links and loopback sockets. Live checks use only the independent servo and small changes around its center.
- Inspect desktop and narrow layouts in one round, correct material issues in one batch, then confirm once.
- Reference requested by user: https://github.com/houxin0010/distinctive-ai-design/blob/main/SKILL.md — direction, actual screenshots and subtraction, without generated decorative media.

## Implemented local components (2026-11-04)

These are surface-specific measurements from `panel/servo_panel.css`, not new global tokens. `DESIGN.md` and `.impeccable/design.json` remain the incumbent authority.

- Container: centered at a maximum 1424px including 32px side padding; at 1600px and wider, maximum 1460px. Desktop device column is 328px, 348px at the wide breakpoint and 300px at 1060px or below. The workspace gap is 24px, then 18px at 1060px.
- Responsive: at 860px or below controls become a full-width column and the device area becomes two equal shrinkable columns; at 560px or below the device area becomes one column with 14px outer padding. Grid children and surfaces use shrinkable widths so the document stays contained; the sequence table alone retains a 400px mobile minimum inside its horizontal scroller.
- Reachable controls: header is sticky at the top with a ground-colored background. Stop remains visible while scrolling; Escape invokes the same stop action. The connection status is also a button that scrolls to device settings, honoring reduced motion and a header-aware scroll margin.
- Readout hierarchy: editable estimated angle is 54px/550, 48px at 1060px or below, and 44px at 560px or below; its negative letter spacing applies only to the numeric readout. Target PWM is 22px/550 (20px on phones), main feedback PWM 29px/550. These are local operating readouts, not replacements for the global 18px metric role. Number fields and feedback use tabular numerals; angle input width follows the displayed character count.
- Feedback proximity: at 860px or below, a compact 18px/600 flight-controller PWM value and freshness text appear directly below Apply. The full recent-history chart remains in the device area; missing/stale feedback displays an em dash rather than an invented output.
- Controls: primary and secondary operating buttons have a 40px minimum height and pill shape; small buttons use 34px. Phone Stop is 36px. The behavior selector is a local radio group with a 12px outer radius and 10px white selected segment. It selects a motion mode rather than a workspace.
- Surfaces: white 24px containers become 18px on phones; local shadow is `0 3px 12px #20202c04`. The manual and sequence areas have different content density; the six-event history is an unboxed list rather than another identical card.
- Chart: dashed gray target preset and solid green flight-controller output, dynamic high/low labels and observed time window. Only fresh PWM samples are added; retained history is not proof of a current measurement. No physical angle trace is shown.
- Sequence: editable angle, interpolation time and dwell columns; add, delete and move-up actions; 1–20 repeats within a 200-point expanded limit; completion selection; progress and current-row highlight. JSON version 1 stores points, repeat and completion policy. Import uses a file input and validates the file before replacing points.
- Feedback semantics: the action message reports the last application; the actual PWM and freshness are separate. Errors also enter the six-event history. Sequence completion labeled “回到中位” releases the override to the configured disarmed output; it is not a guarantee that every installation's default equals the numerical midpoint.

## Recorded divergence, not global changes

- The existing implementation uses green normal-state text and a pale green active-mode badge. This differs from the incumbent neutral-text/green-dot rule; do not promote it into a new global status rule.
- Focus is a graphite 2px outline with 4px offset, versus the incumbent blue-gray focus and 2px offset; local disabled fills/text also differ from the incumbent disabled tokens. The local font stack omits several global cross-platform fallback names. These differences were present at the documentation handoff and are recorded without repairs or claims of token conformity.
- The incumbent product/design documents include the camera/data workbench and historical mission components. This extension does not adopt those workflows or rewrite that system. It retains UAVLab identity, Chinese operation language, light neutral surfaces, graphite primary actions, scaled radii and no decorative imagery.

## Evidence and remaining verification

- Source scan: PRODUCT.md, DESIGN.md, sidecar components/extensions, servo HTML/CSS/JavaScript; server/board source checked for the release and ownership semantics.
- Visual evidence: `.impeccable/review/servo-bench/user-1908.png`, `desktop.png`, `mobile.png`, `mobile-scrolled.png`. Ordinary viewport captures were reviewed at scrollTop 0; the phone document/client width was 375/375. They show contained phone content, nearby compact feedback and the sticky Stop during scrolling. Independent review accepted the fixes for document overflow and feedback distance; this is scoped acceptance, not an exhaustive accessibility audit.
- Main-session validation reported 27 passing Python tests in `tests/test_standalone_servo.py` and `tests/test_servo_motion_modes.py`, plus 5 passing Node tests in `tests/test_servo_frontend.cjs` (32 relevant automated checks total). The Node harness runs real frontend parsing/command functions with inert DOM/network seams: actual-row export, export/import roundtrip, rejected null/string/out-of-range imports preserving the sequence, and Apply reading edited PWM/angle before blur. Browser console error checks were empty. This documentation pass inspected the test/source evidence and screenshots, and did not repeat hardware operations.
- Main-session live checks: hold at 1530 µs continued beyond one second; Stop returned feedback to 1500 µs; a 1470 µs pulse returned to 1500 µs. Screenshot snapshots include a 1492 µs hold. All checked flight-controller states were disarmed. These observations verify the tested PWM behavior, not physical angle or unrestricted travel.
- Export click emitted “已导出动作序列”; browser tooling did not expose a download path, so saved-file content was not verified. Import/export data functions passed the inert harness roundtrip and invalid-input checks, but import was not exercised through a file chooser. Do not call either workflow fully end-to-end validated.
- The requested external reference was read and applied as a design reference, not installed as a skill.
