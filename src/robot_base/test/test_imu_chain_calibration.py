"""Independent rigid-body fixtures: cross products, unequal clocks/rates and frames."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "imu_chain_calibration.py"
SPEC = importlib.util.spec_from_file_location("imu_chain_calibration", SCRIPT)
cal = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cal)


def fixture(directory, td=0.023, moving_center=True):
    rotations = [Rotation.from_euler("xyz", [3, -5, 170], degrees=True).as_matrix(),
                 Rotation.from_euler("xyz", [-2, 45, -12], degrees=True).as_matrix()]
    anchor = np.array([-0.4, 0.002, 0.16])  # deliberately almost directly behind axle
    arm = np.array([0.52, -0.21, 0.22])
    bg = [np.array([0.003, -0.002, 0.004]), np.array([-0.004, 0.001, -0.002])]
    scales = [1.0, cal.G]
    rng = np.random.default_rng(700)
    for phase, duration, start in [("static", 4, 0), ("straight", 20, 10), ("rotate", 32, 40)]:
        for sensor, hz in enumerate([400, 200]):
            t = np.arange(0, duration, 1 / hz)
            w = np.zeros((len(t), 3))
            alpha = np.zeros_like(w)
            common = np.tile([0., 0., cal.G], (len(t), 1))
            if phase == "straight":
                common[:, 0] = 0.8 * np.sin(1.3*t) + 0.3 * np.sin(0.43*t)
            if phase == "rotate":
                w[:, 2] = 0.7*np.sin(0.8*t) + 0.4*np.sin(1.7*t)
                alpha[:, 2] = 0.56*np.cos(0.8*t) + 0.68*np.cos(1.7*t)
                if moving_center:
                    common[:, 0] = 0.5*np.sin(3.1*t)
                    common[:, 1] = 0.3*np.cos(2.2*t)
                    common[:, 2] += 0.08*np.sin(0.6*t)
            position = anchor + sensor*arm
            force = common + np.cross(alpha, position) + np.cross(w, np.cross(w, position))
            force = force @ rotations[sensor]
            gyro = w @ rotations[sensor] + bg[sensor]
            force += rng.normal(0, 0.001, force.shape)
            gyro += rng.normal(0, 0.00003, gyro.shape)
            # Large epoch tests floating-point time handling; M stamps lag by td.
            stamps = 1700000000.0 + start + t - sensor*td
            data = np.column_stack([stamps, force/scales[sensor], gyro])
            name = ("external", "mid360")[sensor]
            np.savetxt(Path(directory) / f"{phase}_{name}.csv", data,
                       delimiter=",", header=",".join(cal.COLUMNS), comments="", fmt="%.12f")
    prior = [Rotation.from_euler("z", angle, degrees=True).as_matrix() @ r
             for angle, r in zip([10, -10], rotations)]
    config = {"schema_version": 1, "body_frame": "base_footprint",
              "factory_transform_confirmed": True,
              "t_body_external_m": anchor.tolist(),
              "external_to_mid360_z_in_body_m": float(arm[2]),
              "rotation_mode": "estimate", "time_offset_s": None,
              "R_body_external": prior[0].tolist(), "R_body_mid360": prior[1].tolist(),
              "R_mid360_lidar": np.eye(3).tolist(), "t_mid360_lidar_m": [-.011, -.02329, .04412],
              "external_accel_scale": 1., "mid360_accel_scale": cal.G,
              "external_gyro_scale": 1., "mid360_gyro_scale": 1.}
    return config, rotations, anchor, arm


class ChainCalibrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.cfg, cls.rotations, cls.anchor, cls.arm = fixture(cls.directory.name)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_full_chain_with_shared_center_motion_and_positive_td(self):
        result = cal.calibrate(self.cfg, self.directory.name)
        self.assertTrue(result["quality_passed"])
        self.assertAlmostEqual(result["time_offset_s"], .023, delta=0.0002)
        diag = result["translation_diagnostics"]
        np.testing.assert_allclose(diag["external_to_mid360_in_body_m"], self.arm, atol=.0005)
        self.assertLess(diag["xy_split_difference_m"], .001)
        tf = result["transforms"]
        for label, r in zip(["external", "mid360"], self.rotations):
            np.testing.assert_allclose(tf[f"T_body_{label}"]["R"], r, atol=0.0001)
        cloud = self.anchor + self.arm + self.rotations[1] @ np.array(self.cfg["t_mid360_lidar_m"])
        np.testing.assert_allclose(tf["T_body_lidar"]["t_m"], cloud, atol=.0005)
        np.testing.assert_allclose(tf["T_external_lidar"]["t_m"],
                                   self.rotations[0].T @ (cloud - self.anchor), atol=.0005)
        self.assertFalse(result["fastlio2_online_yaml_patch"]["online_use_tf_extrinsics"])

    def test_negative_td(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg, _, _, arm = fixture(directory, td=-.037)
            result = cal.calibrate(cfg, directory)
            self.assertAlmostEqual(result["time_offset_s"], -.037, delta=.0002)
            np.testing.assert_allclose(result["translation_diagnostics"]["external_to_mid360_in_body_m"],
                                       arm, atol=.0005)

    def test_fixed_priors_are_preserved(self):
        cfg = copy.deepcopy(self.cfg)
        cfg.update(rotation_mode="fixed", time_offset_s=.023)
        for name, r in zip(["external", "mid360"], self.rotations):
            cfg[f"R_body_{name}"] = r.tolist()
        result = cal.calibrate(cfg, self.directory.name)
        self.assertEqual(result["time_offset_s"], .023)
        np.testing.assert_array_equal(result["transforms"]["T_body_external"]["R"], self.rotations[0])

    def test_zero_fixed_td(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg, _, _, _ = fixture(directory, td=0)
            cfg["time_offset_s"] = 0.0
            self.assertTrue(cal.calibrate(cfg, directory)["quality_passed"])

    def test_only_mid360_orientation_is_estimated(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["rotation_mode"] = {"external": "fixed", "mid360": "estimate"}
        cfg["R_body_external"] = self.rotations[0].tolist()
        result = cal.calibrate(cfg, self.directory.name)
        np.testing.assert_array_equal(result["transforms"]["T_body_external"]["R"], self.rotations[0])
        np.testing.assert_allclose(result["transforms"]["T_body_mid360"]["R"], self.rotations[1], atol=.0001)

    def test_time_drift_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg, _, _, _ = fixture(directory)
            path = Path(directory) / "rotate_mid360.csv"
            raw = np.loadtxt(path, delimiter=",", skiprows=1)
            raw[:, 0] += .001 * (raw[:, 0] - raw[0, 0])
            np.savetxt(path, raw, delimiter=",", header=",".join(cal.COLUMNS), comments="", fmt="%.12f")
            with self.assertRaisesRegex(ValueError, "td differs"):
                cal.calibrate(cfg, directory)

    def test_constant_gyro_cannot_determine_time_offset(self):
        stamps = np.arange(0, 8, .005)
        raw = np.column_stack([stamps, np.tile([0., 0., cal.G, 0., 0., .5], (len(stamps), 1))])
        infos = [{"mean_accel": np.array([0., 0., cal.G]), "gyro_bias": np.zeros(3)}] * 2
        with self.assertRaisesRegex(ValueError, "variable angular velocity"):
            cal.estimate_td([raw, raw], infos, cal.DEFAULTS)

    def test_nonmonotonic_csv_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.csv"
            raw = np.zeros((30, 7))
            raw[:, 0] = np.arange(30)*.005
            raw[15, 0] = raw[14, 0]
            np.savetxt(path, raw, delimiter=",", header=",".join(cal.COLUMNS), comments="")
            with self.assertRaisesRegex(ValueError, "increase strictly"):
                cal.read_csv(path, 1, 1)

    def test_full_3d_fixed_height_and_outliers(self):
        rng = np.random.default_rng(182)
        w, alpha = rng.normal(size=(2, 1200, 3))
        arm = np.array([-.3, .001, .42])
        bias = np.array([.06, -.03, .02])
        difference = np.cross(alpha, arm) + np.cross(w, np.cross(w, arm)) + bias
        difference[::30] += 3.0
        a = cal.skew(alpha) + cal.skew(w) @ cal.skew(w)
        fitted, b, _ = cal.fit_xy(a, difference, arm[2], cal.DEFAULTS)
        np.testing.assert_allclose(fitted, arm, atol=1e-5)
        np.testing.assert_allclose(b, bias, atol=1e-5)

    def test_constant_spin_cannot_separate_bias(self):
        w = np.tile([0., 0., .8], (200, 1))
        a = cal.skew(w) @ cal.skew(w)
        with self.assertRaisesRegex(ValueError, "unobservable"):
            cal.fit_xy(a, np.zeros((200, 3)), .2, cal.DEFAULTS)

    def test_timestamp_gaps_and_no_extrapolation(self):
        raw = np.column_stack([[0., .01, .5, .51], np.zeros((4, 6))])
        with self.assertRaisesRegex(ValueError, "gap"):
            cal.interpolate(raw, np.array([.2]), .05)
        with self.assertRaisesRegex(ValueError, "extrapolation"):
            cal.interpolate(raw, np.array([-.001]), .05)

    def test_bad_units_rotation_and_missing_anchor_rejected(self):
        for field, bad in [("mid360_accel_scale", 1.0),
                           ("R_body_external", np.zeros((3, 3)).tolist()),
                           ("t_body_external_m", [None, None, None]),
                           ("factory_transform_confirmed", False)]:
            cfg = copy.deepcopy(self.cfg)
            cfg[field] = bad
            with self.subTest(field=field), self.assertRaises(ValueError):
                cal.calibrate(cfg, self.directory.name)

    def test_rejected_fit_does_not_export_filter_parameters(self):
        cfg = copy.deepcopy(self.cfg)
        cfg["checks"] = {"max_xy_split_difference_m": 1e-12}
        result = cal.calibrate(cfg, self.directory.name)
        self.assertFalse(result["quality_passed"])
        self.assertNotIn("fastlio2_online_yaml_patch", result)
        self.assertNotIn("external_imu_lidar_extrinsic", result)

    def test_cli_outputs_yaml_and_prevents_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "config.yaml"
            config.write_text(yaml.safe_dump(self.cfg))
            output = Path(directory) / "result.yaml"
            argv = ["solve", "--config", str(config), "--directory", self.directory.name,
                    "--output", str(output)]
            self.assertEqual(cal.main(argv), 0)
            self.assertTrue(yaml.safe_load(output.read_text())["quality_passed"])
            self.assertEqual(cal.main(argv), 2)


if __name__ == "__main__":
    unittest.main()
