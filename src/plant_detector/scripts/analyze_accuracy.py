#!/usr/bin/env python3
"""Analyze plant_detector accuracy against ground truth.

Usage:
    python3 analyze_accuracy.py <bag_path> [--save-fig <output.svg>]

Example:
    python3 analyze_accuracy.py accuracy_exp
    python3 analyze_accuracy.py accuracy_exp --save-fig accuracy_report.svg
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from plant_detector.msg import PlantClusterArray

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Noto Serif CJK SC', 'Times New Roman']
plt.rcParams['mathtext.fontset'] = 'stix'

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
    """Match detections to GT using nearest neighbor."""
    n_det = len(detections)
    n_gt = len(gt)

    if n_det == 0:
        return [], [], list(range(n_gt))

    dist = np.zeros((n_det, n_gt))
    for i, d in enumerate(detections):
        for j, g in enumerate(gt):
            dist[i, j] = np.sqrt((d[0] - g[0])**2 + (d[1] - g[1])**2)

    matched = []
    used_det = set()
    used_gt = set()

    while True:
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

    total_matched = 0
    total_fp = 0
    total_fn = 0
    all_errors = []
    per_gt_errors = {i: [] for i in range(n_gt)}
    per_gt_detected = {i: 0 for i in range(n_gt)}

    # Collect all detections for plotting
    all_matched_dets = []  # (x, y, gt_idx)
    all_fp_dets = []       # (x, y)

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
            all_matched_dets.append((dets[det_idx][0], dets[det_idx][1], gt_idx))

        for det_idx in unmatched_det:
            all_fp_dets.append((dets[det_idx][0], dets[det_idx][1]))

    total_detections = total_matched + total_fp
    total_gt_instances = n_frames * n_gt

    mean_err = np.mean(all_errors) if all_errors else float('inf')
    std_err = np.std(all_errors) if all_errors else 0
    median_err = np.median(all_errors) if all_errors else float('inf')
    max_err = np.max(all_errors) if all_errors else float('inf')

    recall = total_matched / total_gt_instances * 100 if total_gt_instances > 0 else 0
    precision = total_matched / total_detections * 100 if total_detections > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

    # Print report
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

    return {
        "n_frames": n_frames,
        "recall": recall, "precision": precision, "f1": f1,
        "mean_err": mean_err, "median_err": median_err, "max_err": max_err,
        "per_gt_detected": per_gt_detected, "per_gt_errors": per_gt_errors,
        "all_matched_dets": all_matched_dets, "all_fp_dets": all_fp_dets,
        "total_fp": total_fp, "total_fn": total_fn,
    }


def plot_accuracy(stats, save_path=None):
    """Plot accuracy visualization."""
    gt = np.array(GT)
    n_frames = stats["n_frames"]
    n_gt = len(GT)
    colors = ["#2196F3", "#4CAF50", "#FF9800", "#9C27B0", "#F44336"]

    fig, axes = plt.subplots(1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": [3, 2]})

    # --- Left: spatial scatter plot ---
    ax = axes[0]

    # Plot false positives
    if stats["all_fp_dets"]:
        fp = np.array(stats["all_fp_dets"])
        ax.scatter(fp[:, 1], fp[:, 0], c="#BDBDBD", s=8, alpha=0.3, label="误检", zorder=1)

    # Dummy entries for legend
    ax.scatter([], [], c="gray", s=200, marker="*", edgecolors="black", linewidths=0.8, label="真值")
    ax.scatter([], [], c="gray", s=80, marker="D", edgecolors="black", linewidths=0.8, label="检测均值")
    ax.scatter([], [], c="#BDBDBD", s=12, alpha=0.4, label="逐帧检测")

    # Plot GT and matched detections per plant
    for i in range(n_gt):
        # GT position
        ax.scatter(gt[i, 1], gt[i, 0], c=colors[i], s=200, marker="*", edgecolors="black",
                   linewidths=0.8, zorder=5)

        # All matched detections for this GT
        matched = [(d[0], d[1]) for d in stats["all_matched_dets"] if d[2] == i]
        if matched:
            matched = np.array(matched)
            ax.scatter(matched[:, 1], matched[:, 0], c=colors[i], s=12, alpha=0.4, zorder=3)

            # Mean detection position
            mean_det = matched.mean(axis=0)
            ax.scatter(mean_det[1], mean_det[0], c=colors[i], s=80, marker="D",
                       edgecolors="black", linewidths=0.8, zorder=4)

            # Line from GT to mean detection
            ax.plot([gt[i, 1], mean_det[1]], [gt[i, 0], mean_det[0]],
                    c=colors[i], linestyle="--", linewidth=1.2, alpha=0.6, zorder=2)

    ax.set_xlabel("Y (m)  [左+ / 右-]", fontsize=11)
    ax.set_ylabel("X (m)  [前方]", fontsize=11)
    ax.set_title("检测散点图", fontsize=12)
    ax.legend(fontsize=6, loc="lower left", ncol=2, markerscale=0.8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # --- Right: per-plant bar chart ---
    ax2 = axes[1]

    plant_labels = [f"#{i}" for i in range(n_gt)]
    det_rates = [stats["per_gt_detected"][i] / n_frames * 100 for i in range(n_gt)]
    mean_errors = [np.mean(stats["per_gt_errors"][i]) * 100 if stats["per_gt_errors"][i] else 0
                   for i in range(n_gt)]

    x = np.arange(n_gt)
    width = 0.35

    bars1 = ax2.bar(x - width/2, det_rates, width, color="#2196F3", alpha=0.8, label="检测率 (%)")
    ax2_twin = ax2.twinx()
    bars2 = ax2_twin.bar(x + width/2, mean_errors, width, color="#FF9800", alpha=0.8, label="平均误差 (cm)")

    ax2.set_xlabel("盆栽", fontsize=11)
    ax2.set_ylabel("检测率 (%)", fontsize=11, color="#2196F3")
    ax2_twin.set_ylabel("平均误差 (cm)", fontsize=11, color="#FF9800")
    ax2.set_xticks(x)
    ax2.set_xticklabels(plant_labels)
    ax2.set_ylim(0, 110)
    ax2_twin.set_ylim(0, max(mean_errors) * 1.5 if mean_errors else 10)
    ax2.set_title("逐盆栽指标", fontsize=12)

    # Add value labels on bars
    for bar, val in zip(bars1, det_rates):
        ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                 f"{val:.0f}%", ha="center", va="bottom", fontsize=8, color="#2196F3")
    for bar, val in zip(bars2, mean_errors):
        ax2_twin.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                      f"{val:.1f}", ha="center", va="bottom", fontsize=8, color="#FF9800")

    # Combined legend
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc="upper right")


    plt.tight_layout(rect=[0, 0.1, 1, 1])

    if save_path:
        fig.savefig(save_path, format="svg", bbox_inches="tight")
        print(f"\nFigure saved to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analyze plant_detector accuracy against GT")
    parser.add_argument("bag_path", help="Path to the bag directory")
    parser.add_argument("--save-fig", metavar="SVG", help="Save figure to file instead of showing")
    args = parser.parse_args()

    bag = Path(args.bag_path)
    if not bag.exists():
        print(f"ERROR: bag path '{bag}' does not exist")
        sys.exit(1)

    print(f"Reading clusters from: {bag}")
    messages = read_clusters(str(bag))
    print(f"Loaded {len(messages)} frames\n")

    stats = analyze(messages)
    if args.save_fig:
        plot_accuracy(stats, save_path=args.save_fig)
    else:
        plot_accuracy(stats)


if __name__ == "__main__":
    main()
