import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from unitree_pose import PoseHistory
from pointcloud_core import (
    ReferencePose,
    compute_pose_relative_transform,
    save_reference_pose,
    load_reference_pose,
)


class UnitreePoseTests(unittest.TestCase):
    def message(self, stamp, position=(0, 0, 0), angle=0, session="test"):
        return {
            "stamp_ns": stamp,
            "frame_id": "unitree_lio_origin_" + session,
            "child_frame_id": "unitree_lio_body",
            "position": list(position),
            "quaternion_xyzw": Rotation.from_euler("z", angle, degrees=True)
            .as_quat()
            .tolist(),
        }

    def test_interpolation_converts_imu_lever_arm_to_native_lidar_pose(self):
        history = PoseHistory()
        now = time.time_ns()
        history.append(self.message(now - 100_000_000))
        history.append(self.message(now, (0.2, 0, 0), angle=90))
        pose = history.at(now - 50_000_000)
        expected = np.array([0.1, 0, 0]) + Rotation.from_euler(
            "z", 45, degrees=True
        ).apply([0.007698, 0.014655, -0.00667])
        np.testing.assert_allclose(pose.position, expected)
        np.testing.assert_allclose(pose.pose_to_cloud, np.eye(3))

    def test_stale_pose_and_unbracketed_capture_are_rejected(self):
        history = PoseHistory()
        now = time.time_ns()
        history.append(self.message(now))
        with self.assertRaisesRegex(RuntimeError, "extrapola"):
            history.at(now + 1)
        with patch("unitree_pose.time.time_ns", return_value=now + 2_000_000_000):
            with self.assertRaisesRegex(RuntimeError, "caducada"):
                history.latest()

    def test_native_axes_transform_compensates_known_motion_and_preserves_session(self):
        rotation = Rotation.from_euler("XYZ", [4, -6, 15], degrees=True).as_matrix()
        translation = np.array([0.12, -0.04, 0.025])
        base = ReferencePose(np.zeros(3), np.zeros(3), np.eye(3), "origin", "session")
        after = ReferencePose(
            translation, np.array([4, -6, 15]), np.eye(3), "origin", "session"
        )
        r, t = compute_pose_relative_transform(base, after)
        np.testing.assert_allclose(r, rotation, atol=1e-14)
        np.testing.assert_allclose(t, translation)
        world_points = np.array([[1, 2, 3], [2, 3, 1], [0, 2, 1.0]])
        after_points = (world_points - translation) @ rotation
        np.testing.assert_allclose(after_points @ r.T + t, world_points, atol=1e-14)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "after.ply"
            save_reference_pose(path, *after, source_pose=after)
            loaded = load_reference_pose(path)
            self.assertEqual(loaded.session_id, "session")
            np.testing.assert_allclose(
                compute_pose_relative_transform(base, loaded)[1], translation
            )

    def test_reset_tracking_and_mixed_sensor_frames_are_not_compensated(self):
        base = ReferencePose(np.zeros(3), np.zeros(3), np.eye(3), "origin", "first")
        after = ReferencePose(np.zeros(3), np.zeros(3), np.eye(3), "origin", "second")
        with self.assertRaisesRegex(ValueError, "sesiones"):
            compute_pose_relative_transform(base, after)
        with self.assertRaisesRegex(ValueError, "sensores"):
            compute_pose_relative_transform(base, (np.zeros(3), np.zeros(3)))
        history = PoseHistory()
        now = time.time_ns()
        history.append(self.message(now, session="first"))
        history.append(self.message(now + 1, session="second"))
        self.assertEqual(len(history.samples), 1)

    def test_stationary_average_reduces_jitter_without_hiding_real_motion(self):
        history = PoseHistory()
        now = time.time_ns()
        for i, offset in enumerate((-0.003, 0.003, -0.002, 0.002, -0.001, 0.001)):
            history.append(self.message(now - (5 - i) * 80_000_000, (offset, 0, 0)))
        raw = history.latest()
        smooth = history.smoothed()
        self.assertLess(
            abs(smooth.position[0] - 0.007698), abs(raw.position[0] - 0.007698)
        )
        history.append(self.message(now + 1, (0.20, 0, 0)))
        np.testing.assert_array_equal(
            history.smoothed().position, history.latest().position
        )


if __name__ == "__main__":
    unittest.main()
