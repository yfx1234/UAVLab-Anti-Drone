#!/usr/bin/env python3
"""Losslessly remux MJPEG MKV inputs from a common media-time offset."""

import argparse
from pathlib import Path

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def quote(value):
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


def trim(source, destination, start_sec):
    start_ns = int(start_sec * Gst.SECOND)
    description = (
        f"filesrc location={quote(source)} ! matroskademux ! queue ! "
        f"identity name=trim_gate ! identity ts-offset=-{start_ns} ! matroskamux ! "
        f"filesink location={quote(destination)} sync=false"
    )
    pipeline = Gst.parse_launch(description)
    try:
        gate = pipeline.get_by_name("trim_gate")
        gate_src = gate.get_static_pad("src")

        def discard_prefix(_pad, info):
            buffer = info.get_buffer()
            if buffer is None:
                return Gst.PadProbeReturn.OK
            timestamp = buffer.pts
            if timestamp == Gst.CLOCK_TIME_NONE:
                timestamp = buffer.dts
            if timestamp != Gst.CLOCK_TIME_NONE and timestamp < start_ns:
                return Gst.PadProbeReturn.DROP
            return Gst.PadProbeReturn.OK

        gate_src.add_probe(Gst.PadProbeType.BUFFER, discard_prefix)
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f"failed to remux {source}")
        bus = pipeline.get_bus()
        message = bus.timed_pop_filtered(
            Gst.CLOCK_TIME_NONE, Gst.MessageType.ERROR | Gst.MessageType.EOS
        )
        if message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            raise RuntimeError(f"remux failed for {source}: {error}; {debug}")
    finally:
        pipeline.set_state(Gst.State.NULL)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--start-sec", type=float, default=180.0)
    args = parser.parse_args()
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    # Legacy inputs were named by Argus enumeration.  The verified physical
    # mapping is csi1=left and csi0=right, so emit unambiguous logical names.
    names = (
        (
            "csi1_3840x2160_15fps_mjpeg.mkv",
            "left_sensor1_3840x2160_15fps_mjpeg.mkv",
        ),
        (
            "csi0_3840x2160_15fps_mjpeg.mkv",
            "right_sensor0_3840x2160_15fps_mjpeg.mkv",
        ),
        (
            "usb_2560x1440_25fps_native_mjpeg.mkv",
            "usb_2560x1440_25fps_native_mjpeg.mkv",
        ),
    )
    for source_name, destination_name in names:
        source = input_dir / source_name
        destination = output_dir / destination_name
        if not source.is_file():
            raise FileNotFoundError(source)
        print(f"REMUX_START source={source} start_sec={args.start_sec}", flush=True)
        trim(source, destination, args.start_sec)
        print(
            f"REMUX_DONE destination={destination} bytes={destination.stat().st_size}",
            flush=True,
        )
    (output_dir / "READY").write_text(
        f"start_sec={args.start_sec}\n", encoding="utf-8"
    )


if __name__ == "__main__":
    Gst.init(None)
    main()
