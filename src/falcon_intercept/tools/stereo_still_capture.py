#!/usr/bin/env python3
"""Stereo still-pair capture with live preview for checkerboard calibration.

Run on the Jetson DESKTOP (needs a display), not over plain SSH:

    python3 stereo_still_capture.py OUT_DIR

Layout written under OUT_DIR:
    front/pose_001.jpg   back/pose_001.jpg
    front/pose_002.jpg   back/pose_002.jpg
    capture_log.txt

Keys:
    SPACE / ENTER   capture one front+back pair (3840x2160 JPEG q95)
    q / ESC         quit

The preview shows both cameras side by side with a live 9x6 checkerboard
status and a board-motion readout. Capture only when STATIC OK is green:
the stereo cameras are not hardware-synced, so the board must be perfectly
still at the exposure instant (rigid mount, no handheld).
"""

import os
import sys
import time

import cv2
import numpy as np

SENSOR_LEFT = 1
SENSOR_RIGHT = 0
WIDTH, HEIGHT = 3840, 2160
FPS = 15
BOARD = (9, 6)
PREV_W, PREV_H = 960, 540
SCALE = WIDTH // PREV_W          # preview -> full-res factor (4)
STATIC_PX_PER_FRAME = 1.5        # full-res median corner speed gate
JPEG_QUALITY = 95


def gst_pipeline(sensor_id):
    return (
        "nvarguscamerasrc sensor-id=%d sensor-mode=0 wbmode=1 awblock=false ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,format=NV12,framerate=30/1 ! "
        "videorate drop-only=true ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,format=NV12,framerate=%d/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
        "appsink drop=true max-buffers=2"
        % (sensor_id, WIDTH, HEIGHT, WIDTH, HEIGHT, FPS)
    )


def detect(gray):
    ok, _ = cv2.findChessboardCorners(gray, BOARD, cv2.CALIB_CB_FAST_CHECK)
    if not ok:
        return None
    found, corners = cv2.findChessboardCorners(gray, BOARD)
    if not found:
        return None
    return corners.reshape(-1, 2)


def draw_status(img, text, ok):
    color = (60, 200, 60) if ok else (60, 60, 220)
    cv2.rectangle(img, (0, 0), (PREV_W, 34), (30, 30, 30), -1)
    cv2.putText(img, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def main():
    out_root = sys.argv[1] if len(sys.argv) > 1 else "stereo_stills"
    front_dir = os.path.join(out_root, "front")
    back_dir = os.path.join(out_root, "back")
    os.makedirs(front_dir, exist_ok=True)
    os.makedirs(back_dir, exist_ok=True)
    log_path = os.path.join(out_root, "capture_log.txt")

    caps = {
        "front": cv2.VideoCapture(gst_pipeline(SENSOR_LEFT), cv2.CAP_GSTREAMER),
        "back": cv2.VideoCapture(gst_pipeline(SENSOR_RIGHT), cv2.CAP_GSTREAMER),
    }
    for name, cap in caps.items():
        if not cap.isOpened():
            raise SystemExit("failed to open %s camera" % name)

    pose = 0
    prev_corners = {"front": None, "back": None}
    speed = {"front": None, "back": None}
    flash_until = 0.0
    print("preview running; SPACE=capture, q=quit; output=%s" % out_root)

    while True:
        frames, previews, corners = {}, {}, {}
        for name, cap in caps.items():
            ret, frame = cap.read()
            if not ret or frame is None:
                frame = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
            frames[name] = frame
            small = cv2.resize(frame, (PREV_W, PREV_H))
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            c = detect(gray)
            corners[name] = c
            if c is not None and prev_corners[name] is not None:
                v = float(np.median(np.linalg.norm(c - prev_corners[name], axis=1))) * SCALE
                speed[name] = v
            elif c is None:
                speed[name] = None
            if c is not None:
                prev_corners[name] = c
            previews[name] = small

        for name, small in previews.items():
            c = corners[name]
            v = speed[name]
            board_ok = c is not None
            static_ok = v is not None and v <= STATIC_PX_PER_FRAME
            if board_ok:
                cv2.drawChessboardCorners(small, BOARD, c.reshape(-1, 1, 2), True)
            if not board_ok:
                txt = "%s: board NOT found" % name.upper()
                ok = False
            elif static_ok:
                txt = "%s: 9x6 OK  motion %.2f px/f  STATIC OK" % (name.upper(), v)
                ok = True
            else:
                txt = "%s: 9x6 OK  motion %s px/f  MOVING" % (
                    name.upper(), "--" if v is None else "%.2f" % v)
                ok = False
            draw_status(small, txt, ok)

        canvas = np.hstack([previews["front"], previews["back"]])
        if time.time() < flash_until:
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1] - 1, canvas.shape[0] - 1),
                          (80, 255, 80), 12)
        cv2.putText(canvas, "pose=%03d   SPACE=capture  q=quit" % pose,
                    (PREV_W - 330, PREV_H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        cv2.imshow("stereo still capture", canvas)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key in (32, 13):
            pose += 1
            stem = "pose_%03d" % pose
            for name in ("front", "back"):
                path = os.path.join(out_root, name, stem + ".jpg")
                cv2.imwrite(path, frames[name],
                            [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            warn = ""
            if corners["front"] is None or corners["back"] is None:
                warn = "  WARNING: board not visible in preview at capture"
            with open(log_path, "a") as f:
                f.write("%s %s%s\n" % (stem, time.strftime("%H:%M:%S"), warn))
            print("saved %s%s" % (stem, warn))
            flash_until = time.time() + 0.4

    for cap in caps.values():
        cap.release()
    cv2.destroyAllWindows()
    print("done: %d pairs in %s" % (pose, out_root))


if __name__ == "__main__":
    main()
