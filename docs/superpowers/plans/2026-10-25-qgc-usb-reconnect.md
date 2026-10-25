# QGC USB Reconnection Implementation Plan

> **For agentic workers:** Use executing-plans inline; test each boundary before implementation.

**Goal:** Automatically replace a stale MAVProxy serial session after USB removal or re-enumeration.

**Architecture:** A standard-library supervisor owns the existing MAVProxy child process. It watches the stable device path's inode/device identity without opening the serial port, stops the child on removal/change, and restarts only after the device is stable. An installer prefixes the existing ExecStart while preserving every original MAVProxy argument and a rollback backup.

**Tech Stack:** Python 3.10+, Linux user systemd, existing MAVProxy/pymavlink.

**Spec:** `docs/QGC重启失联定位_2026-10-25.md`.

## Global Constraints

- Do not change firmware, flight-controller parameters, ROS interfaces, or transmit flight/control commands.
- Keep all current UDP outputs, including existing local consumers; never open USB a second time to monitor it.
- Back up the active unit, preserve Environment/WorkingDirectory, deploy only the reviewed files.
- Do not reboot the flight controller automatically; real reboot validation is user-operated.

## Review Focus

- Device absent at startup: wait without spawning a disconnected child.
- Same path, replaced inode: retire the old child before opening the replacement.
- Signal during wait or while running: exit promptly with no orphan process.
- Child exits or refuses SIGINT: bounded cleanup, no runaway duplicate process.
- Installer rerun or unsupported ExecStart: preserve all arguments, reject unsafe rewrites, no duplicate wrapping.

### Task 1: Supervisor and installer

**Files:** Create `deploy/supervise_mavlink_usb.py`, `deploy/install_qgc_usb_reconnect.py`, `tests/test_qgc_usb_reconnect.py`.

**Interfaces:** `supervise(device: Path, command: list[str], poll: float, settle: float, stop_timeout: float) -> int`; `render_unit(text: str, supervisor: Path) -> str`.

- [ ] Write installer boundary tests and real Linux subprocess/device replacement tests; verify RED before implementation.
- [ ] Implement device identity supervision, bounded child shutdown, CLI, atomic unit rewrite with backup and rollback instructions.
- [ ] Run targeted tests on Windows and Linux (Linux covers the POSIX signal and device-replacement behavior).
- [ ] Run the repository's Python suite and repository audit, report pre-existing failures separately.
- [ ] Commit, obtain an independent review, fix meaningful findings with failing tests.

### Task 2: Deploy and verify

- [ ] Verify active service/deployment identity; copy only the tested tools and preserve the entire original service command.
- [ ] Install the wrapper, restart only the forwarding service, verify USB ownership and real heartbeat on the existing UDP mirror.
- [ ] Run isolated Linux replacement tests on NX; request one user-operated reboot for physical verification after deployment.
- [ ] Record exact verified boundaries, rollback path, pending physical reboot validation; synchronize both project remotes and maintenance records.

## Execution notes

- App-native worktree creation could not target this chat's parent directory (not a Git repository); used an isolated detached Git worktree of the confirmed project.
- Execution proceeds inline under the user's repair request; no extra plan approval gate is needed for this reversible fix.
