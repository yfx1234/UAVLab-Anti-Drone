#!/usr/bin/env python3
"""Compare FP16 and INT8 detections on synchronized, visibility-labelled frames."""

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import cv2


SOURCES = ("front", "back", "search")
VIDEO_NAMES = {
"front": "left_sensor1_3840x2160_15fps_mjpeg.mkv",
"back": "right_sensor0_3840x2160_15fps_mjpeg.mkv",
    "search": "usb_2560x1440_25fps_native_mjpeg.mkv",
}


def load_jsonl(path):
    frames = {}
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            payload = item["payload"]
            frames[(item["source"], int(payload["sequence"]))] = payload
    return frames


def visibility(manifest, source, time_sec):
    rules = manifest["visibility_intervals_clip_sec"][source]
    for label in ("positive", "negative", "ignore"):
        if any(float(start) <= time_sec < float(end) for start, end in rules[label]):
            return label
    return "excluded"


def detections(payload, threshold):
    return [d for d in payload["detections"] if float(d["confidence"]) >= threshold]


def iou(first, second):
    ax1, ay1, ax2, ay2 = map(float, first["bbox_xyxy"])
    bx1, by1, bx2, by2 = map(float, second["bbox_xyxy"])
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - intersection
    return intersection / union if union > 0.0 else 0.0


def greedy_matches(first, second, minimum_iou=0.5):
    candidates = sorted(
        ((iou(a, b), ai, bi) for ai, a in enumerate(first) for bi, b in enumerate(second)),
        reverse=True,
    )
    used_first, used_second, matches = set(), set(), []
    for score, first_index, second_index in candidates:
        if score < minimum_iou:
            break
        if first_index in used_first or second_index in used_second:
            continue
        used_first.add(first_index)
        used_second.add(second_index)
        matches.append((first_index, second_index, score))
    return matches


def safe_ratio(numerator, denominator):
    return numerator / denominator if denominator else None


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def profile_metrics(frames, manifest, threshold):
    by_source = {}
    aggregate = defaultdict(int)
    fps = float(manifest["sampling_fps"])
    for source in SOURCES:
        counts = defaultdict(int)
        confidences = []
        for (frame_source, sequence), payload in frames.items():
            if frame_source != source:
                continue
            label = visibility(manifest, source, sequence / fps)
            if label not in ("positive", "negative"):
                continue
            present = bool(detections(payload, threshold))
            counts[f"{label}_frames"] += 1
            counts[f"{label}_detected_frames"] += int(present)
            counts[f"{label}_boxes"] += len(detections(payload, threshold))
            confidences.extend(float(d["confidence"]) for d in detections(payload, threshold))
        tp = counts["positive_detected_frames"]
        fn = counts["positive_frames"] - tp
        fp = counts["negative_detected_frames"]
        tn = counts["negative_frames"] - fp
        counts.update(tp=tp, fn=fn, fp=fp, tn=tn)
        result = dict(counts)
        result.update(
            detection_rate_on_visible=safe_ratio(tp, tp + fn),
            false_positive_frame_rate=safe_ratio(fp, fp + tn),
            frame_precision=safe_ratio(tp, tp + fp),
            frame_f1=safe_ratio(2 * tp, 2 * tp + fp + fn),
            mean_detection_confidence=statistics.fmean(confidences) if confidences else None,
        )
        by_source[source] = result
        for name in ("tp", "fn", "fp", "tn"):
            aggregate[name] += result[name]
    aggregate_result = dict(aggregate)
    aggregate_result.update(
        detection_rate_on_visible=safe_ratio(aggregate["tp"], aggregate["tp"] + aggregate["fn"]),
        false_positive_frame_rate=safe_ratio(aggregate["fp"], aggregate["fp"] + aggregate["tn"]),
        frame_precision=safe_ratio(aggregate["tp"], aggregate["tp"] + aggregate["fp"]),
        frame_f1=safe_ratio(2 * aggregate["tp"], 2 * aggregate["tp"] + aggregate["fp"] + aggregate["fn"]),
    )
    return {"by_source": by_source, "aggregate": aggregate_result}


def segment_metrics(frames, manifest, threshold):
    fps = float(manifest["sampling_fps"])
    results = []
    for interval in manifest["selected_original_intervals"]:
        clip_start = float(interval["start"]) - 180.0
        clip_end = float(interval["end"]) - 180.0
        for source in SOURCES:
            counts = defaultdict(int)
            current_miss = 0
            longest_miss = 0
            for (frame_source, sequence), payload in frames.items():
                if frame_source != source:
                    continue
                time_sec = sequence / fps
                if not clip_start <= time_sec < clip_end:
                    continue
                label = visibility(manifest, source, time_sec)
                if label not in ("positive", "negative"):
                    continue
                present = bool(detections(payload, threshold))
                counts[f"{label}_frames"] += 1
                counts[f"{label}_detected_frames"] += int(present)
                if label == "positive":
                    current_miss = 0 if present else current_miss + 1
                    longest_miss = max(longest_miss, current_miss)
            results.append(
                {
                    "original_start_sec": interval["start"],
                    "original_end_sec": interval["end"],
                    "note": interval["note"],
                    "source": source,
                    "positive_frames": counts["positive_frames"],
                    "positive_detection_rate": safe_ratio(
                        counts["positive_detected_frames"], counts["positive_frames"]
                    ),
                    "negative_frames": counts["negative_frames"],
                    "negative_detection_rate": safe_ratio(
                        counts["negative_detected_frames"], counts["negative_frames"]
                    ),
                    "longest_visible_miss_sec": longest_miss / fps,
                }
            )
    return results


def agreement_metrics(fp16, int8, manifest, threshold):
    fps = float(manifest["sampling_fps"])
    counts = defaultdict(int)
    ious, confidence_deltas, center_deltas = [], [], []
    cases = []
    common_keys = sorted(set(fp16) & set(int8))
    for source, sequence in common_keys:
        label = visibility(manifest, source, sequence / fps)
        if label not in ("positive", "negative"):
            continue
        fp_detections = detections(fp16[(source, sequence)], threshold)
        int_detections = detections(int8[(source, sequence)], threshold)
        fp_present, int_present = bool(fp_detections), bool(int_detections)
        counts["labelled_frames"] += 1
        counts["presence_agree"] += int(fp_present == int_present)
        counts["both_present"] += int(fp_present and int_present)
        counts["fp16_only"] += int(fp_present and not int_present)
        counts["int8_only"] += int(int_present and not fp_present)
        counts["both_absent"] += int(not fp_present and not int_present)
        if fp_present != int_present:
            cases.append(
                {
                    "kind": "fp16_only" if fp_present else "int8_only",
                    "source": source,
                    "sequence": sequence,
                    "time_sec": sequence / fps,
                    "label": label,
                    "fp16": fp_detections,
                    "int8": int_detections,
                }
            )
        matches = greedy_matches(fp_detections, int_detections)
        counts["fp16_boxes"] += len(fp_detections)
        counts["int8_boxes"] += len(int_detections)
        counts["matched_boxes_iou50"] += len(matches)
        for fp_index, int_index, score in matches:
            fp_detection, int_detection = fp_detections[fp_index], int_detections[int_index]
            ious.append(score)
            confidence_deltas.append(
                float(int_detection["confidence"]) - float(fp_detection["confidence"])
            )
            fp_box = list(map(float, fp_detection["bbox_xyxy"]))
            int_box = list(map(float, int_detection["bbox_xyxy"]))
            width = float(fp16[(source, sequence)]["image_width"])
            height = float(fp16[(source, sequence)]["image_height"])
            fp_center = ((fp_box[0] + fp_box[2]) / 2, (fp_box[1] + fp_box[3]) / 2)
            int_center = ((int_box[0] + int_box[2]) / 2, (int_box[1] + int_box[3]) / 2)
            center_deltas.append(
                math.hypot(fp_center[0] - int_center[0], fp_center[1] - int_center[1])
                / math.hypot(width, height)
            )
    return {
        **dict(counts),
        "presence_agreement": safe_ratio(counts["presence_agree"], counts["labelled_frames"]),
        "matched_box_rate_vs_fp16": safe_ratio(counts["matched_boxes_iou50"], counts["fp16_boxes"]),
        "matched_iou": {
            "mean": statistics.fmean(ious) if ious else None,
            "p05": percentile(ious, 0.05),
            "p50": percentile(ious, 0.50),
        },
        "int8_minus_fp16_confidence": {
            "mean": statistics.fmean(confidence_deltas) if confidence_deltas else None,
            "p05": percentile(confidence_deltas, 0.05),
            "p50": percentile(confidence_deltas, 0.50),
            "p95": percentile(confidence_deltas, 0.95),
        },
        "normalized_center_delta": {
            "mean": statistics.fmean(center_deltas) if center_deltas else None,
            "p95": percentile(center_deltas, 0.95),
        },
        "disagreement_cases": cases,
    }


def select_review_cases(cases, maximum_per_group=4):
    grouped = defaultdict(list)
    for case in cases:
        grouped[(case["kind"], case["label"])].append(case)
    selected = []
    for group_cases in grouped.values():
        group_cases.sort(
            key=lambda case: max(
                [float(d["confidence"]) for d in case["fp16"] + case["int8"]] or [0.0]
            ),
            reverse=True,
        )
        accepted_times = defaultdict(list)
        for case in group_cases:
            if any(abs(case["time_sec"] - prior) < 3.0 for prior in accepted_times[case["source"]]):
                continue
            selected.append(case)
            accepted_times[case["source"]].append(case["time_sec"])
            if sum(1 for item in selected if (item["kind"], item["label"]) == (case["kind"], case["label"])) >= maximum_per_group:
                break
    return selected


def draw_case(frame, case):
    colors = ((255, 255, 0), (255, 0, 255))
    for profile_index, profile in enumerate(("fp16", "int8")):
        for detection in case[profile]:
            x1, y1, x2, y2 = map(lambda value: int(round(float(value))), detection["bbox_xyxy"])
            cv2.rectangle(frame, (x1, y1), (x2, y2), colors[profile_index], 4)
            cv2.putText(
                frame,
                f"{profile} {float(detection['confidence']):.3f}",
                (x1, max(30, y1 - 8 - profile_index * 32)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                colors[profile_index],
                2,
                cv2.LINE_AA,
            )
    cv2.putText(
        frame,
        f"{case['source']} t={case['time_sec']:.2f}s label={case['label']} {case['kind']}",
        (24, 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (255, 255, 255),
        3,
        cv2.LINE_AA,
    )
    return frame


def render_review(cases, clips_dir, output_dir):
    review_dir = output_dir / "review_frames"
    review_dir.mkdir(parents=True, exist_ok=True)
    thumbnails, rendered = [], []
    captures = {}
    try:
        for index, case in enumerate(cases):
            source = case["source"]
            if source not in captures:
                captures[source] = cv2.VideoCapture(str(Path(clips_dir) / VIDEO_NAMES[source]))
            capture = captures[source]
            capture.set(cv2.CAP_PROP_POS_MSEC, case["time_sec"] * 1000.0)
            ok, frame = capture.read()
            if not ok:
                case["render_error"] = "frame_read_failed"
                continue
            frame = draw_case(frame, case)
            filename = f"{index:02d}_{case['kind']}_{case['label']}_{source}_{case['time_sec']:.2f}.jpg"
            cv2.imwrite(str(review_dir / filename), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
            thumbnail = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
            thumbnails.append(thumbnail)
            case["image"] = str(Path("review_frames") / filename)
            rendered.append(case)
    finally:
        for capture in captures.values():
            capture.release()
    if thumbnails:
        columns = 2
        rows = math.ceil(len(thumbnails) / columns)
        blank = thumbnails[0] * 0
        while len(thumbnails) < rows * columns:
            thumbnails.append(blank.copy())
        row_images = [
            cv2.hconcat(thumbnails[row * columns : (row + 1) * columns])
            for row in range(rows)
        ]
        cv2.imwrite(str(output_dir / "disagreement_contact_sheet.jpg"), cv2.vconcat(row_images))
    return rendered


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--fp16-jsonl", required=True)
    parser.add_argument("--int8-jsonl", required=True)
    parser.add_argument("--clips-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    fp16, int8 = load_jsonl(args.fp16_jsonl), load_jsonl(args.int8_jsonl)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    thresholds = (0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60, 0.70, 0.80, 0.90)
    threshold_results = {}
    current_disagreement_cases = []
    for threshold in thresholds:
        agreement = agreement_metrics(fp16, int8, manifest, threshold)
        disagreement_cases = agreement.pop("disagreement_cases")
        if threshold == 0.50:
            current_disagreement_cases = disagreement_cases
        threshold_results[str(threshold)] = {
            "fp16": profile_metrics(fp16, manifest, threshold),
            "int8": profile_metrics(int8, manifest, threshold),
            "agreement": agreement,
        }
    review_cases = select_review_cases(current_disagreement_cases)
    rendered_cases = render_review(review_cases, args.clips_dir, output_dir)
    report = {
        "method": {
            "metric_level": "frame-level visibility plus cross-profile box agreement",
            "sampling_fps": manifest["sampling_fps"],
            "box_match_iou": 0.5,
            "ground_truth_limit": manifest["annotation_policy"]["limitation"],
        },
        "thresholds": threshold_results,
        "segments_at_0.50": {
            "fp16": segment_metrics(fp16, manifest, 0.50),
            "int8": segment_metrics(int8, manifest, 0.50),
        },
        "review_cases": rendered_cases,
    }
    (output_dir / "precision_ab_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "review_manifest.json").write_text(
        json.dumps(rendered_cases, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report["thresholds"]["0.5"], ensure_ascii=False, indent=2))
    print(f"REPORT={output_dir / 'precision_ab_report.json'}")


if __name__ == "__main__":
    main()
