"""ROS transport smoke test, skipped when running offline without ROS 2."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

try:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, "ROS 2 is not installed")
class RecorderTest(unittest.TestCase):
    def test_real_ros_topics_preserve_independent_headers_units_and_frames(self):
        script = Path(__file__).resolve().parents[1] / "scripts" / "imu_chain_calibration.py"
        rclpy.init(args=[])
        node = rclpy.create_node("imu_chain_test_publisher")
        topics = [f"/imu_chain_test_{os.getpid()}/{name}" for name in ["external", "mid360"]]
        pubs = [node.create_publisher(Imu, topic, qos_profile_sensor_data) for topic in topics]
        sent = [set(), set()]
        count = 0

        def publish():
            nonlocal count
            count += 1
            stamp = node.get_clock().now().nanoseconds
            for i, pub in enumerate(pubs):
                if i and count % 2:
                    continue
                msg = Imu()
                msg.header.frame_id = ["imu_link", "livox_frame"][i]
                shifted = stamp - i * 23000000
                msg.header.stamp.sec = shifted // 1000000000
                msg.header.stamp.nanosec = shifted % 1000000000
                msg.linear_acceleration.z = [9.80665, 1.0][i]
                msg.angular_velocity.z = .123
                sent[i].add(f"{shifted // 1000000000}.{shifted % 1000000000:09d}")
                pub.publish(msg)

        timer = node.create_timer(.005, publish)
        process = None
        try:
            with tempfile.TemporaryDirectory() as directory:
                process = subprocess.Popen(
                    [sys.executable, str(script), "record", "--phase", "static", "--duration", "4",
                     "--directory", directory, "--external-topic", topics[0], "--mid360-topic", topics[1]],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                deadline = time.monotonic() + 15
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                self.assertIsNotNone(process.poll(), "recorder did not terminate")
                output = process.communicate(timeout=2)[0]
                self.assertEqual(process.returncode, 0, output)
                metadata = json.loads((Path(directory) / "static_metadata.json").read_text())
                self.assertEqual(metadata["frames"], ["imu_link", "livox_frame"])
                self.assertIsNone(metadata["error"])
                for i, name in enumerate(["external", "mid360"]):
                    path = Path(directory) / f"static_{name}.csv"
                    raw = np.loadtxt(path, delimiter=",", skiprows=1)
                    self.assertGreater(len(raw), 100)
                    np.testing.assert_allclose(raw[:, 3], [9.80665, 1.0][i])
                    np.testing.assert_allclose(raw[:, 6], .123)
                    recorded_stamps = {line.split(",")[0] for line in path.read_text().splitlines()[1:]}
                    self.assertTrue(recorded_stamps.issubset(sent[i]))
                self.assertGreater(metadata["counts"][0], 1.5 * metadata["counts"][1])
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.communicate(timeout=5)
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
