#!/usr/bin/env python3
"""Analyze plant_detector metrics from a ROS 2 bag.

Usage:
    python3 analyze_metrics.py <bag_path> [--save-fig <output.png>]

Example:
    python3 analyze_metrics.py realtime_exp
    python3 analyze_metrics.py realtime_exp --save-fig metrics_report.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from plant_detector.msg import PlantMetrics


def read_metrics(bag_path: str) -> list[PlantMetrics]:
    """Read all PlantMetrics messages from a bag."""
    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if "plant_detector/metrics" not in topic_types and "/plant_detector/metrics" not in topic_types:
        print(f"ERROR: topic 'plant_detector/metrics' not found in bag")
        print(f"  Available topics: {list(topic_types.keys())}")
        sys.exit(1)

    messages = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in ("plant_detector/metrics", "/plant_detector/metrics"):
            msg = deserialize_message(data, PlantMetrics)
            messages.append(msg)

    return messages


def analyze(messages: list[PlantMetrics]):
    """Compute and print statistics."""
    n = len(messages)
    if n == 0:
        print("No metrics messages found.")
        sys.exit(1)

    total = np.array([m.total_ms for m in messages])
    filt = np.array([m.filter_ms for m in messages])
    clust = np.array([m.cluster_ms for m in messages])
    cls = np.array([m.classify_ms for m in messages])
    raw_pts = np.array([m.raw_points for m in messages])
    filt_pts = np.array([m.filtered_points for m in messages])
    n_clust = np.array([m.cluster_count for m in messages])
    n_plant = np.array([m.detected_plants for m in messages])

    total_mean = np.mean(total)

    print("=" * 60)
    print("  plant_detector Metrics Report")
    print("=" * 60)
    print(f"  Frames:         {n}")
    print(f"  Duration:       {n * total_mean / 1000:.1f} s (estimated)")
    print()

    # --- Timing ---
    print("  [Timing]")
    print(f"  {'Stage':<14} {'Mean':>8} {'Std':>8} {'Min':>8} {'Max':>8} {'Ratio':>8}")
    print(f"  {'-'*14} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")

    for name, data in [("total_ms", total), ("filter_ms", filt),
                        ("cluster_ms", clust), ("classify_ms", cls)]:
        ratio = np.mean(data) / total_mean * 100 if total_mean > 0 else 0
        print(f"  {name:<14} {np.mean(data):>8.2f} {np.std(data):>8.2f} "
              f"{np.min(data):>8.2f} {np.max(data):>8.2f} {ratio:>7.1f}%")

    fps = 1000.0 / total_mean if total_mean > 0 else 0
    print()
    print(f"  Avg FPS:        {fps:.2f} Hz")
    print()

    # --- Point cloud ---
    print("  [Point Cloud]")
    print(f"  Raw points:     mean={np.mean(raw_pts):.0f}  std={np.std(raw_pts):.0f}")
    print(f"  Filtered:       mean={np.mean(filt_pts):.0f}  std={np.std(filt_pts):.0f}")
    removal = (1 - np.mean(filt_pts) / np.mean(raw_pts)) * 100 if np.mean(raw_pts) > 0 else 0
    print(f"  Removal rate:   {removal:.1f}%")
    print(f"  Clusters/frame: mean={np.mean(n_clust):.1f}  max={np.max(n_clust)}")
    print(f"  Plants/frame:   mean={np.mean(n_plant):.1f}  max={np.max(n_plant)}")
    print("=" * 60)

    return total, filt, clust, cls, fps


def plot_histograms(total, filt, clust, cls, fps, save_path=None):
    """Plot timing distribution histograms."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"plant_detector Timing (avg {fps:.1f} Hz)", fontsize=14)

    datasets = [
        (total, "Total", "tab:blue"),
        (filt, "Filter (ground removal)", "tab:orange"),
        (clust, "Cluster (euclidean)", "tab:green"),
        (cls, "Classify", "tab:red"),
    ]

    for ax, (data, title, color) in zip(axes.flat, datasets):
        ax.hist(data, bins=40, color=color, alpha=0.8, edgecolor="white")
        ax.axvline(np.mean(data), color="black", linestyle="--", linewidth=1.5,
                   label=f"mean={np.mean(data):.2f}ms")
        ax.set_title(title)
        ax.set_xlabel("Time (ms)")
        ax.set_ylabel("Frame count")
        ax.legend(fontsize=9)

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\nFigure saved to: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analyze plant_detector metrics from a ROS 2 bag")
    parser.add_argument("bag_path", help="Path to the bag directory")
    parser.add_argument("--save-fig", metavar="PNG", help="Save histogram to file instead of showing")
    args = parser.parse_args()

    bag = Path(args.bag_path)
    if not bag.exists():
        print(f"ERROR: bag path '{bag}' does not exist")
        sys.exit(1)

    print(f"Reading metrics from: {bag}")
    messages = read_metrics(str(bag))
    print(f"Loaded {len(messages)} frames\n")

    total, filt, clust, cls, fps = analyze(messages)
    plot_histograms(total, filt, clust, cls, fps, save_path=args.save_fig)


if __name__ == "__main__":
    main()
