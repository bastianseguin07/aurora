"""Native capture contract, using organized SDK frames without sensor hardware."""

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import aurora_sensor
from live_viewer import LiveViewer


def connection_for(frames):
    frames = iter(frames)

    def peek(_frame_type):
        points = next(frames)
        return SimpleNamespace(
            data=b"frame", width=3, height=2, timestamp_ns=0,
            to_point3d_array=lambda: points.copy(),
        )

    imaging = SimpleNamespace(
        wait_depth_camera_next_frame=lambda _timeout: True,
        peek_depth_camera_frame=peek,
    )
    return aurora_sensor.AuroraConnection(
        sdk=SimpleNamespace(enhanced_imaging=imaging)
    )


class NativeCaptureTests(unittest.TestCase):
    def setUp(self):
        self.sdk = patch.object(aurora_sensor, "_import_sdk", return_value=(None,) * 4)
        self.sdk.start()
        self.addCleanup(self.sdk.stop)
        # Include valid points outside the former implicit 0.1–50 m range.
        self.frame = np.array([
            [0, 0, 0.05], [0.2, 0.1, 1.8], [0.4, 0.1, 2.0],
            [0, 0, 60], [0, 0, 0], [np.nan, 0, 1],
        ], dtype=np.float32)
        self.expected = self.frame[:4]

    def test_last_frame_and_ply_preserve_sdk_coordinates(self):
        earlier = self.frame.copy()
        earlier[:4, 2] += 0.5
        connection = connection_for([earlier] * 14 + [self.frame])
        cloud = aurora_sensor.capture_snapshot(connection, capture_duration_s=1e-9)
        np.testing.assert_array_equal(np.asarray(cloud.points), self.expected)
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "native.ply")
            self.assertTrue(o3d.io.write_point_cloud(path, cloud))
            np.testing.assert_array_equal(
                np.asarray(o3d.io.read_point_cloud(path).points), self.expected
            )

    def test_distance_crop_only_removes_points_when_requested(self):
        points, _ = aurora_sensor.read_frame_points_and_colors(
            connection_for([self.frame]), max_distance_m=1.9
        )
        np.testing.assert_array_equal(points, self.expected[:2])

    def test_best_coverage_remains_a_single_native_frame(self):
        sparse = self.frame.copy()
        sparse[1] = 0
        sparse[2, 2] += 0.5
        with patch.object(aurora_sensor, "_fuse_static_frames") as fusion:
            cloud = aurora_sensor.capture_snapshot(
                connection_for([self.frame, sparse]),
                stabilization_frames=2, capture_duration_s=1e-9,
            )
        fusion.assert_not_called()
        np.testing.assert_array_equal(np.asarray(cloud.points), self.expected)

    def test_optional_static_accumulation_still_receives_organized_frames(self):
        with patch.object(
            aurora_sensor, "_fuse_static_frames", wraps=aurora_sensor._fuse_static_frames
        ) as fusion:
            cloud = aurora_sensor.capture_snapshot(
                connection_for([self.frame] * 3),
                stabilization_frames=3, capture_duration_s=1e-9,
                accumulate_static=True,
            )
        fusion.assert_called_once()
        np.testing.assert_array_equal(np.asarray(cloud.points), self.expected)

    def test_persistence_keeps_original_coordinates(self):
        earlier = self.frame.copy()
        earlier[1, 2] += 0.5
        cloud = aurora_sensor.capture_snapshot(
            connection_for([earlier, self.frame]),
            stabilization_frames=2, capture_duration_s=1e-9, persistence_ratio=1.0,
        )
        np.testing.assert_array_equal(np.asarray(cloud.points), self.expected[[0, 2, 3]])

    def test_live_feed_preserves_sdk_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "live.npz"
            viewer = LiveViewer(o3d.geometry.PointCloud(), live_output_path=path)
            viewer.aurora_connection = connection_for([self.frame])
            read = aurora_sensor.read_frame_points_and_colors

            def read_and_stop(*args, **kwargs):
                result = read(*args, **kwargs)
                viewer._stop_event.set()
                return result

            with patch("live_viewer.read_frame_points_and_colors", side_effect=read_and_stop):
                viewer._run_live_feed()
            with np.load(path) as saved:
                np.testing.assert_array_equal(saved["points"], self.expected)


if __name__ == "__main__":
    unittest.main()
