#!/usr/bin/env python3
"""Offline planar chassis / dual-IMU calibration, with optional ROS 2 recording.

T_A_B maps points in B into A: p_A = R_A_B p_B + t_A_B.
B: configured chassis frame, E: external IMU, M: MID360 IMU, L: cloud.
Time convention: external(t) corresponds to mid360(t - td).
See docs/imu_chain_calibration.md for assumptions and capture procedure.
"""

import argparse
import csv
import json
from pathlib import Path
import sys
import time

import numpy as np
import yaml
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import minimize_scalar
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation


G = 9.80665
COLUMNS = ("t", "ax", "ay", "az", "gx", "gy", "gz")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def vector(value, name):
    result = np.asarray(value, dtype=float)
    require(result.shape == (3,) and np.isfinite(result).all(),
            f"{name}: supply three finite numbers (measured values, not null)")
    return result


def rotation(value, name):
    result = np.asarray(value, dtype=float)
    require(result.size == 9, f"{name}: need a row-major 3x3 matrix")
    result = result.reshape(3, 3)
    require(np.isfinite(result).all()
            and np.allclose(result.T @ result, np.eye(3), atol=1e-6)
            and abs(np.linalg.det(result) - 1) < 1e-6,
            f"{name}: not a proper rotation")
    return result


def read_csv(path, accel_scale, gyro_scale):
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    require(data.dtype.names == COLUMNS, f"{path}: header must be {','.join(COLUMNS)}")
    raw = np.column_stack([np.atleast_1d(data[k]) for k in COLUMNS])
    require(len(raw) >= 20 and np.isfinite(raw).all(), f"{path}: short/nonfinite data")
    require(np.all(np.diff(raw[:, 0]) > 0),
            f"{path}: timestamps must increase strictly; do not mix clock epochs")
    require(accel_scale > 0 and gyro_scale > 0
            and np.isfinite([accel_scale, gyro_scale]).all(), "invalid unit scales")
    raw[:, 1:4] *= accel_scale
    raw[:, 4:7] *= gyro_scale
    return raw


def interpolate(raw, times, max_gap):
    stamps = raw[:, 0]
    require(times.min() >= stamps[0] and times.max() <= stamps[-1],
            "timestamp coverage insufficient; extrapolation is forbidden")
    indices = np.clip(np.searchsorted(stamps, times), 1, len(stamps) - 1)
    # A sample exactly on a stamp needs no interpolation across the preceding gap.
    exact = stamps[indices] == times
    require(np.all(exact | ((stamps[indices] - stamps[indices - 1]) <= max_gap)),
            "IMU gap exceeds max_gap_s; split/re-record this phase")
    return np.column_stack([np.interp(times, stamps, raw[:, j]) for j in range(1, 7)])


def grid(pair, td, rate, margin=0.0):
    lo = max(pair[0][0, 0], pair[1][0, 0] + td) + margin
    hi = min(pair[0][-1, 0], pair[1][-1, 0] + td) - margin
    require(hi - lo >= 2.0, "need at least two seconds of common phase coverage")
    # Relative subtraction happens before generating the uniform grid.
    return lo + np.arange(0.0, hi - lo, 1.0 / rate)


def skew(v):
    result = np.zeros((*v.shape[:-1], 3, 3))
    result[..., 0, 1], result[..., 0, 2] = -v[..., 2], v[..., 1]
    result[..., 1, 0], result[..., 1, 2] = v[..., 2], -v[..., 0]
    result[..., 2, 0], result[..., 2, 1] = -v[..., 1], v[..., 0]
    return result


def static_info(raw, cfg):
    require(np.max(np.diff(raw[:, 0])) <= cfg["max_gap_s"], "gap in static data")
    require(raw[-1, 0] - raw[0, 0] >= 2.0, "record at least 2 s stationary")
    accel, gyro = raw[:, 1:4], raw[:, 4:7]
    mean = accel.mean(axis=0)
    require(abs(np.linalg.norm(mean) - G) <= cfg["gravity_tolerance_m_s2"],
            "static acceleration is not near g: check units / gravity removal")
    require(np.max(np.linalg.norm(gyro, axis=1)) < cfg["static_max_gyro_rad_s"],
            "static phase is rotating (or gyro bias is excessive)")
    require(np.linalg.norm(accel.std(axis=0)) < cfg["static_max_accel_std_m_s2"],
            "static phase is moving/vibrating too much")
    return {"mean_accel": mean, "gyro_bias": gyro.mean(axis=0),
            "accel_std": accel.std(axis=0), "gyro_std": gyro.std(axis=0)}


def leveling(mean_accel):
    """Minimum rotation from stationary specific force to chassis +z."""
    unit = mean_accel / np.linalg.norm(mean_accel)
    z = np.array([0.0, 0.0, 1.0])
    cross = np.cross(unit, z)
    if np.dot(unit, z) < -1 + 1e-8:
        return Rotation.from_rotvec([np.pi, 0, 0]).as_matrix()
    k = skew(cross)
    return np.eye(3) + k + k @ k / (1 + np.dot(unit, z))


def estimate_orientation(raw, info, prior, cfg):
    """Gravity tilt + PCA straight acceleration; mechanical prior resolves +/-x."""
    rate, gap = cfg["sample_rate_hz"], cfg["max_gap_s"]
    ts = raw[0, 0] + np.arange(0, raw[-1, 0] - raw[0, 0], 1 / rate)
    samples = interpolate(raw, ts, gap)
    require(np.max(np.linalg.norm(samples[:, 3:] - info["gyro_bias"], axis=1))
            <= cfg["straight_max_gyro_rad_s"],
            "straight phase rotates too much; repeat on a straight, level path")
    level = leveling(info["mean_accel"])
    accel = (samples[:, :3] - info["mean_accel"]) @ level.T
    width = max(5, int(round(cfg["straight_smooth_s"] * rate)) | 1)
    require(len(accel) > 3 * width, "straight phase too short for smoothing")
    accel = savgol_filter(accel, width, 3, axis=0)[width:-width]
    horizontal = accel[:, :2] - accel[:, :2].mean(axis=0)
    _, s, vt = np.linalg.svd(horizontal, full_matrices=False)
    require(s[0] / np.sqrt(len(horizontal)) >= cfg["straight_min_accel_rms_m_s2"],
            "straight acceleration excitation too weak")
    ratio = float(s[1] / s[0])
    require(ratio <= cfg["straight_max_pca_ratio"],
            "straight acceleration is not predominantly one-dimensional")
    direction = vt[0]
    yaw = -np.arctan2(direction[1], direction[0])
    candidates = [Rotation.from_euler("z", yaw + k * np.pi).as_matrix() @ level
                  for k in (0, 1)]
    distances = [Rotation.from_matrix(r @ prior.T).magnitude() for r in candidates]
    index = int(np.argmin(distances))
    require(np.degrees(distances[index]) <= cfg["max_rotation_from_prior_deg"],
            "orientation disagrees with mechanical prior; check axes and +/-x")
    return candidates[index], {"pca_minor_major_ratio": ratio,
                              "prior_difference_deg": float(np.degrees(distances[index])),
                              "forward_sign_source": "mechanical rotation prior"}


def estimate_td(pair, infos, cfg):
    bound = cfg["td_search_bound_s"]
    rate, gap = cfg["sample_rate_hz"], cfg["max_gap_s"]
    ts = grid(pair, 0.0, rate, margin=bound + 0.1)
    # Axial signed rotation is yaw-invariant, unlike a planar gyro SVD for full R.
    axes = [s["mean_accel"] / np.linalg.norm(s["mean_accel"]) for s in infos]
    ref = (interpolate(pair[0], ts, gap)[:, 3:] - infos[0]["gyro_bias"]) @ axes[0]
    mid = len(ts) // 2
    subsets = [np.arange(0, mid), np.arange(mid, len(ts))]

    def solve(indices):
        tt, aa = ts[indices], ref[indices]
        aa = aa - aa.mean()
        require(np.std(aa) >= cfg["td_min_gyro_std_rad_s"],
                "td needs variable angular velocity in BOTH halves")

        def loss(td):
            bb = (interpolate(pair[1], tt - td, gap)[:, 3:]
                  - infos[1]["gyro_bias"]) @ axes[1]
            residual = aa - (bb - bb.mean())
            return float(np.mean(residual ** 2))

        candidates = np.linspace(-bound, bound, 161)
        losses = np.array([loss(td) for td in candidates])
        best = int(np.argmin(losses))
        require(0 < best < len(candidates) - 1, "td minimum hits search boundary")
        fit = minimize_scalar(loss, bounds=(candidates[best-1], candidates[best+1]),
                              method="bounded", options={"xatol": 1e-7})
        require(fit.success, "td optimization failed")
        # At least one distinguishable timing direction is required.
        probe = min(0.01, bound / 4)
        contrast = min(loss(fit.x - probe), loss(fit.x + probe)) - fit.fun
        require(contrast >= cfg["td_min_loss_contrast"],
                "td objective is flat; accelerate/decelerate rotation")
        require(np.sqrt(fit.fun) < cfg["max_gyro_rmse_rad_s"],
                "dual gyro alignment residual too large")
        return float(fit.x), float(np.sqrt(fit.fun))

    train, test = [solve(indices) for indices in subsets]
    require(abs(train[0] - test[0]) <= cfg["max_td_split_difference_s"],
            "td differs between halves: clock drift/jitter or inadequate excitation")
    return train[0], {"source": "first_half_rotation_gyro",
                      "train_td_s": train[0], "test_td_s": test[0],
                      "split_difference_s": abs(train[0] - test[0]),
                      "gyro_rmse_rad_s": [train[1], test[1]]}


def integrated_model(pair, rotations, biases, td, cfg):
    """Average both sides equally; do not replace mean(w*w) with mean(w)**2."""
    rate = cfg["sample_rate_hz"]
    ts = grid(pair, td, rate)
    ext = interpolate(pair[0], ts, cfg["max_gap_s"])
    mid = interpolate(pair[1], ts - td, cfg["max_gap_s"])
    difference = mid[:, :3] @ rotations[1].T - ext[:, :3] @ rotations[0].T
    w0 = (ext[:, 3:] - biases[0]) @ rotations[0].T
    w1 = (mid[:, 3:] - biases[1]) @ rotations[1].T
    gyro_rmse = float(np.sqrt(np.mean((w0 - w1) ** 2)))
    require(gyro_rmse < cfg["max_gyro_rmse_rad_s"],
            "fixed R/td inconsistent with paired gyro data")
    w = (w0 + w1) / 2
    q = skew(w) @ skew(w)
    h = max(2, int(round(cfg["integration_window_s"] * rate / 2)))
    require(len(ts) > 8 * h, "rotation phase too short")
    duration = 2 * h / rate

    def average(values):
        integral = cumulative_trapezoid(values, dx=1/rate, axis=0, initial=0)
        return (integral[2*h:] - integral[:-2*h]) / duration

    alpha = (w[2*h:] - w[:-2*h]) / duration
    return ts[h:-h], skew(alpha) + average(q), average(difference), gyro_rmse


def fit_xy(a, difference, z, cfg):
    """All quantities in chassis B; t points E -> M; bias is M minus E."""
    require(len(a) >= 50, "too few translation samples")
    features = a[:, :, :2]
    centered = features - features.mean(axis=0)
    singular = np.linalg.svd(centered.reshape(-1, 2), compute_uv=False)
    excitation = singular / np.sqrt(len(a))
    require(excitation[-1] >= cfg["xy_min_excitation_rad_s2"]
            and singular[0] / singular[-1] <= cfg["xy_max_condition"],
            "xy unobservable after removing constant bias; vary spin speed / include stops")
    design = np.concatenate([features, np.broadcast_to(np.eye(3), (len(a), 3, 3))], axis=2)
    x = design.reshape(-1, 5)
    y = (difference - a[:, :, 2] * z).reshape(-1)
    result = np.linalg.lstsq(x, y, rcond=None)[0]
    for _ in range(15):
        residual = x @ result - y
        scale = max(1e-6, 1.4826 * np.median(abs(residual - np.median(residual))))
        weights = np.sqrt(np.minimum(1, 1.5 * scale / np.maximum(abs(residual), 1e-12)))
        result = np.linalg.lstsq(x * weights[:, None], y * weights, rcond=None)[0]
    return np.r_[result[:2], z], result[2:], excitation


def transform(r, t):
    return {"R": r.tolist(), "t_m": t.tolist(),
            "rpy_rad": Rotation.from_matrix(r).as_euler("xyz").tolist()}


DEFAULTS = {
    "sample_rate_hz": 200.0, "max_gap_s": 0.05,
    "gravity_tolerance_m_s2": 0.5, "static_max_gyro_rad_s": 0.05,
    "static_max_accel_std_m_s2": 0.2, "straight_max_gyro_rad_s": 0.05,
    "straight_smooth_s": 0.1, "straight_min_accel_rms_m_s2": 0.15,
    "straight_max_pca_ratio": 0.2, "max_rotation_from_prior_deg": 45.0,
    "td_search_bound_s": 0.1, "td_min_gyro_std_rad_s": 0.15,
    "td_min_loss_contrast": 1e-6, "max_td_split_difference_s": 0.003,
    "max_gyro_rmse_rad_s": 0.03, "integration_window_s": 0.15,
    "xy_min_excitation_rad_s2": 0.05, "xy_max_condition": 100.0,
    "max_xy_split_difference_m": 0.02, "max_accel_rmse_m_s2": 0.1,
}


def calibrate(config, directory):
    require(isinstance(config, dict), "config must be a YAML mapping")
    require(config.get("schema_version") == 1, "schema_version must be 1")
    require(config.get("factory_transform_confirmed") is True,
            "verify MID360 IMU-to-cloud transform, then set factory_transform_confirmed: true")
    cfg = dict(DEFAULTS)
    overrides = config.get("checks", {})
    require(isinstance(overrides, dict) and not (set(overrides) - set(cfg)),
            "unknown check name")
    cfg.update(overrides)
    require(all(np.isfinite(v) and v > 0 for v in cfg.values()), "checks must be positive finite numbers")
    require(cfg["max_rotation_from_prior_deg"] < 90, "yaw sign prior tolerance must be < 90 deg")
    require(cfg["sample_rate_hz"] >= 20, "sample_rate_hz must be >= 20")
    require(cfg["td_search_bound_s"] <= 1, "td search bound must be <= 1 s")
    body_frame = config["body_frame"]
    require(isinstance(body_frame, str) and body_frame.strip(), "body_frame is required")
    t_be = vector(config["t_body_external_m"], "t_body_external_m")
    r_ml = rotation(config["R_mid360_lidar"], "R_mid360_lidar")
    t_ml = vector(config["t_mid360_lidar_m"], "t_mid360_lidar_m")
    z = float(config["external_to_mid360_z_in_body_m"])
    require(np.isfinite(z), "measured relative z is required")
    mode = config.get("rotation_mode", "estimate")
    modes = [mode.get(sensor) if isinstance(mode, dict) else mode
             for sensor in ("external", "mid360")]
    require(all(m in ("estimate", "fixed") for m in modes),
            "rotation_mode must be estimate/fixed or a mapping for external and mid360")
    rotations = [rotation(config[f"R_body_{sensor}"], f"R_body_{sensor}")
                 for sensor in ("external", "mid360")]

    def load(phase):
        metadata = Path(directory) / f"{phase}_metadata.json"
        if metadata.exists():
            require(json.loads(metadata.read_text()).get("error") is None,
                    f"{phase}: recording was interrupted/invalid; use a complete phase")
        return [read_csv(Path(directory) / f"{phase}_{sensor}.csv",
                         float(config[f"{sensor}_accel_scale"]),
                         float(config[f"{sensor}_gyro_scale"]))
                for sensor in ("external", "mid360")]

    stationary, rotating = load("static"), load("rotate")
    infos = [static_info(raw, cfg) for raw in stationary]
    rotation_report = {sensor: {"source": "fixed_mechanical_or_precalibrated"}
                       for sensor in ("external", "mid360")}
    if "estimate" in modes:
        straight = load("straight")
        for i, sensor in enumerate(("external", "mid360")):
            if modes[i] == "estimate":
                rotations[i], diagnostics = estimate_orientation(straight[i], infos[i], rotations[i], cfg)
                rotation_report[sensor] = dict(
                    source="level_static_gravity_and_straight_acceleration", **diagnostics)
    td = config.get("time_offset_s")
    if td is None:
        td, time_report = estimate_td(rotating, infos, cfg)
    else:
        td = float(td)
        require(np.isfinite(td), "time_offset_s must be finite")
        time_report = {"source": "fixed_prior", "td_s": td}
    ts, a, diff, gyro_rmse = integrated_model(
        rotating, rotations, [s["gyro_bias"] for s in infos], td, cfg)
    middle = (ts[0] + ts[-1]) / 2
    guard = cfg["integration_window_s"]
    train, test = ts < middle - guard, ts > middle + guard
    arm, bias, excitation = fit_xy(a[train], diff[train], z, cfg)
    other, other_bias, _ = fit_xy(a[test], diff[test], z, cfg)
    prediction = np.einsum("nij,j->ni", a, arm) + bias
    split = float(np.linalg.norm(arm[:2] - other[:2]))
    rms = float(np.sqrt(np.mean((diff[test] - prediction[test]) ** 2)))
    baseline = diff[train].mean(axis=0)
    baseline_rms = float(np.sqrt(np.mean((diff[test] - baseline) ** 2)))
    failures = []
    if split > cfg["max_xy_split_difference_m"]:
        failures.append("xy_split_difference")
    if rms > cfg["max_accel_rmse_m_s2"]:
        failures.append("heldout_acceleration_residual")

    r_be, r_bm = rotations
    r_bl = r_bm @ r_ml
    t_bm = t_be + arm
    t_bl = t_bm + r_bm @ t_ml
    r_el = r_be.T @ r_bl
    t_el = r_be.T @ (t_bl - t_be)
    transforms = {"T_body_external": transform(r_be, t_be),
                  "T_body_mid360": transform(r_bm, t_bm),
                  "T_body_lidar": transform(r_bl, t_bl),
                  "T_external_mid360": transform(r_be.T @ r_bm, r_be.T @ arm),
                  "T_external_lidar": transform(r_el, t_el)}
    result = {
        "schema_version": 1, "quality_passed": not failures, "failed_checks": failures,
        "body_frame": body_frame,
        "convention": "T_A_B maps B to A; td = stamp_external - stamp_mid360 for same event",
        "time_offset_s": td, "time_diagnostics": time_report,
        "rotation_diagnostics": rotation_report,
        "translation_diagnostics": {
            "source": "first_half_rotation_acceleration_difference_fixed_R_td_z",
            "external_to_mid360_in_body_m": arm.tolist(),
            "second_half_independent_in_body_m": other.tolist(),
            "xy_split_difference_m": split, "heldout_rmse_m_s2": rms,
            "heldout_bias_only_rmse_m_s2": baseline_rms,
            "accel_difference_bias_in_body_m_s2": bias.tolist(),
            "second_half_bias_in_body_m_s2": other_bias.tolist(),
            "excitation_singular_values_per_sqrt_sample": excitation.tolist(),
            "gyro_alignment_rmse_rad_s": gyro_rmse,
            "train_samples": int(train.sum()), "test_samples": int(test.sum())},
        "static_diagnostics": [{k: v.tolist() for k, v in info.items()} for info in infos],
        "transforms": transforms, "checks": cfg,
        "limitations": ["No axle translation is estimated: external IMU anchor is measured input.",
                        "z is fixed in body axes, not sensor axes.",
                        "Repeatability/residuals are not absolute accuracy or ground truth.",
                        "IMU-IMU td is a LiDAR-IMU prior only if cloud and MID360 IMU share a clock."]}
    # Rejected fits remain diagnostic-only; never export filter config for them.
    if not failures:
        result["fastlio2_online_yaml_patch"] = {
            "body_frame": body_frame, "online_use_tf_extrinsics": False,
            "imu_to_baselink_r_il": r_be.reshape(-1).tolist(),
            "imu_to_baselink_t_il": t_be.tolist(),
            "laser_to_baselink_r_il": r_bl.reshape(-1).tolist(),
            "laser_to_baselink_t_il": t_bl.tolist()}
        result["external_imu_lidar_extrinsic"] = {"r_il": r_el.reshape(-1).tolist(),
                                                 "t_il": t_el.tolist()}
    return result


def record(args):
    """Record independently, retaining header times; no approximate synchronization."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    require(np.isfinite(args.duration) and args.duration > 0, "duration must be positive")
    require(args.external_topic != args.mid360_topic, "IMU topics must be different")
    directory = Path(args.directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = [directory / f"{args.phase}_{sensor}.csv" for sensor in ("external", "mid360")]
    require(not any(p.exists() for p in paths), "phase already exists; use a new directory")
    rclpy.init(args=[])
    node = Node("imu_chain_recorder")
    handles, writers, counts, frames, previous = [], [], [0, 0], [None, None], [None, None]
    error = None
    start = time.monotonic()
    next_log = start
    subscriptions = []
    try:
        for path in paths:
            handle = path.open("x", newline="")
            handles.append(handle)
            writer = csv.writer(handle)
            writer.writerow(COLUMNS)
            writers.append(writer)

        def callback(index, msg):
            nonlocal error
            stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            values = [msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z, msg.angular_velocity.x,
                      msg.angular_velocity.y, msg.angular_velocity.z]
            if (not np.isfinite(values).all() or stamp <= 0
                    or (previous[index] is not None and stamp <= previous[index])):
                error = "nonfinite data / invalid or non-increasing header time"
                return
            if frames[index] is not None and frames[index] != msg.header.frame_id:
                error = "IMU frame_id changed during recording"
                return
            frames[index], previous[index] = msg.header.frame_id, stamp
            writers[index].writerow([f"{stamp // 1000000000}.{stamp % 1000000000:09d}"] + values)
            counts[index] += 1

        for i, topic in enumerate((args.external_topic, args.mid360_topic)):
            subscriptions.append(node.create_subscription(
                Imu, topic, lambda msg, index=i: callback(index, msg), qos_profile_sensor_data))
        print(f"Recording {args.phase} for {args.duration}s; no velocity commands are sent.", flush=True)
        while rclpy.ok() and time.monotonic() - start < args.duration and error is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_log:
                print(f"{time.monotonic() - start:.1f}s samples: external={counts[0]}, mid360={counts[1]}", flush=True)
                next_log = time.monotonic() + 5
    except KeyboardInterrupt:
        error = "recording interrupted; partial files retained for inspection"
    finally:
        for handle in handles:
            handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    metadata = {"phase": args.phase, "topics": [args.external_topic, args.mid360_topic],
                "frames": frames, "counts": counts, "error": error,
                "timestamps": "original ROS header seconds", "units": "unchanged from driver"}
    (directory / f"{args.phase}_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    require(error is None and min(counts) >= 20, f"record failed: {error or 'missing IMU samples'}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    capture = sub.add_parser("record", help="record ROS 2 sensor_msgs/Imu to raw CSV")
    capture.add_argument("--phase", choices=("static", "straight", "rotate"), required=True)
    capture.add_argument("--directory", required=True)
    capture.add_argument("--duration", type=float, default=60.0)
    capture.add_argument("--external-topic", default="/imu")
    capture.add_argument("--mid360-topic", default="/livox/imu")
    solve = sub.add_parser("solve", help="offline fit; ROS is not required")
    solve.add_argument("--config", required=True)
    solve.add_argument("--directory", required=True)
    solve.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "record":
            record(args)
            return 0
        output = Path(args.output)
        require(not output.exists(), "output already exists; choose a new output filename")
        config = yaml.safe_load(Path(args.config).read_text())
        result = calibrate(config, args.directory)
        with output.open("x") as handle:
            yaml.safe_dump(result, handle, sort_keys=False, allow_unicode=True)
        print(yaml.safe_dump({k: result[k] for k in
                              ("quality_passed", "time_offset_s", "translation_diagnostics")}, sort_keys=False))
        return 0 if result["quality_passed"] else 2
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
