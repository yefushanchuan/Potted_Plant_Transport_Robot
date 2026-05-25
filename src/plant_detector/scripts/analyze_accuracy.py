#!/usr/bin/env python3
"""Analyze plant_detector accuracy against ground truth.

Usage:
    python3 analyze_accuracy.py <bag_path>

Example:
    python3 analyze_accuracy.py accuracy_exp
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from plant_detector.msg import PlantClusterArray

# Ground truth: (x, y) in meters, base_footprint frame
GT = [
    (0.78, 0.43),
    (0.94, 0.19),
    (0.99, -0.14),
    (1.00, -0.48),
    (0.85, -0.78),
]


def read_clusters(bag_path: str) -> list[PlantClusterArray]:
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    messages = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in ("/plant_detector/clusters", "plant_detector/clusters"):
            msg = deserialize_message(data, PlantClusterArray)
            messages.append(msg)

    return messages


def match_detections_to_gt(detections, gt, max_dist=0.30):
    """Match detections to GT using nearest neighbor. Returns (matched, unmatched_det, unmatched_gt)."""
    n_det = len(detections)
    n_gt = len(gt)

    if n_det == 0:
        return [], [], list(range(n_gt))

    # Build distance matrix
    dist = np.zeros((n_det, n_gt))
    for i, d in enumerate(detections):
        for j, g in enumerate(gt):
            dist[i, j] = np.sqrt((d[0] - g[0])**2 + (d[1] - g[1])**2)

    matched = []
    used_det = set()
    used_gt = set()

    # Greedy matching by smallest distance
    while True:
        # Find smallest unused distance
        min_val = np.inf
        mi, mj = -1, -1
        for i in range(n_det):
            if i in used_det:
                continue
            for j in range(n_gt):
                if j in used_gt:
                    continue
                if dist[i, j] < min_val:
                    min_val = dist[i, j]
                    mi, mj = i, j

        if min_val > max_dist or mi == -1:
            break

        matched.append((mi, mj, min_val))
        used_det.add(mi)
        used_gt.add(mj)

    unmatched_det = [i for i in range(n_det) if i not in used_det]
    unmatched_gt = [j for j in range(n_gt) if j not in used_gt]

    return matched, unmatched_det, unmatched_gt


def analyze(messages: list[PlantClusterArray]):
    n_frames = len(messages)
    if n_frames == 0:
        print("No cluster messages found.")
        sys.exit(1)

    n_gt = len(GT)

    # Per-frame stats
    total_matched = 0
    total_fp = 0
    total_fn = 0
    all_errors = []  # position errors for matched pairs
    per_gt_errors = {i: [] for i in range(n_gt)}  # errors per GT plant
    per_gt_detected = {i: 0 for i in range(n_gt)}  # detection count per GT plant

    for msg in messages:
        dets = [(c.position.x, c.position.y) for c in msg.clusters]
        matched, unmatched_det, unmatched_gt = match_detections_to_gt(dets, GT)

        total_matched += len(matched)
        total_fp += len(unmatched_det)
        total_fn += len(unmatched_gt)

        for det_idx, gt_idx, err in matched:
            all_errors.append(err)
            per_gt_errors[gt_idx].append(err)
            per_gt_detected[gt_idx] += 1

    total_detections = total_matched + total_fp
    total_gt_instances = n_frames * n_gt

    # Aggregate
    mean_err = np.mean(all_errors) if all_errors else float('inf')
    std_err = np.std(all_errors) if all_errors else 0
    median_err = np.median(all_errors) if all_errors else float('inf')
    max_err = np.max(all_errors) if all_errors else float('inf')

    recall = total_matched / total_gt_instances * 100 if total_gt_instances > 0 else 0
    precision = total_matched / total_detections * 100 if total_detections > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

    print("=" * 60)
    print("  plant_detector Accuracy Report")
    print("=" * 60)
    print(f"  Frames:             {n_frames}")
    print(f"  GT plants:          {n_gt}")
    print(f"  GT instances:       {total_gt_instances}")
    print(f"  Total detections:   {total_detections}")
    print(f"  Matched:            {total_matched}")
    print(f"  False positives:    {total_fp}")
    print(f"  Missed (FN):        {total_fn}")
    print()

    print("  [Overall Metrics]")
    print(f"  Recall:             {recall:.1f}%")
    print(f"  Precision:          {precision:.1f}%")
    print(f"  F1:                 {f1:.1f}%")
    print()

    print("  [Position Error]")
    print(f"  Mean:               {mean_err*100:.1f} cm")
    print(f"  Std:                {std_err*100:.1f} cm")
    print(f"  Median:             {median_err*100:.1f} cm")
    print(f"  Max:                {max_err*100:.1f} cm")
    print()

    print("  [Per-Plant Detail]")
    print(f"  {'Plant':<6} {'GT (x, y)':<16} {'Detect%':>8} {'MeanErr':>8} {'MaxErr':>8}")
    print(f"  {'-'*6} {'-'*16} {'-'*8} {'-'*8} {'-'*8}")
    for i in range(n_gt):
        det_rate = per_gt_detected[i] / n_frames * 100
        if per_gt_errors[i]:
            mean_e = np.mean(per_gt_errors[i]) * 100
            max_e = np.max(per_gt_errors[i]) * 100
        else:
            mean_e = float('inf')
            max_e = float('inf')
        print(f"  #{i:<5} ({GT[i][0]:.2f}, {GT[i][1]:.2f})  "
              f"{det_rate:>7.1f}% {mean_e:>7.1f}cm {max_e:>7.1f}cm")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="Analyze plant_detector accuracy against GT")
    parser.add_argument("bag_path", help="Path to the bag directory")
    args = parser.parse_args()

    bag = Path(args.bag_path)
    if not bag.exists():
        print(f"ERROR: bag path '{bag}' does not exist")
        sys.exit(1)

    print(f"Reading clusters from: {bag}")
    messages = read_clusters(str(bag))
    print(f"Loaded {len(messages)} frames\n")

    analyze(messages)


if __name__ == "__main__":
    main()
