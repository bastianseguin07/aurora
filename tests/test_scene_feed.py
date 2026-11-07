import io
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scene_feed import SceneFeed, scene_view
from unitree_protocol import CloudFrame, POINT_DTYPE, encode_frame, read_frame
from unitree_pose import PoseHistory
from ros_unitree_bridge import encode_pointcloud2


def pose(position, session="unitree_lio_origin_test"):
    return PoseHistory._pose(dict(position=position, quaternion_xyzw=[0, 0, 0, 1],
                                  stamp_ns=time.time_ns(), frame_id=session))


class SceneFeedTests(unittest.TestCase):
    def test_gui_opening_starts_at_zero_without_reusing_tracking_origin(self):
        current = pose([12, -4, 3])
        frame = CloudFrame(np.zeros(0, dtype=POINT_DTYPE), time.time(), 0,
                           current.session_id, time.monotonic(), time.time_ns())
        connection = NS(read_registered_batch=lambda: [frame], pose_history=NS(smoothed=lambda: current))
        for _ in range(2):
            feed = SceneFeed(start=False)
            feed.set_connection(connection)
            feed._poll(connection)
            state = feed.snapshot()
            np.testing.assert_allclose(state["position"], [0, 0, 0])
            self.assertIsNone(state["reference"])
            self.assertFalse(state["capture_reference_ready"])
            feed.set_reference(current)
            feed._poll(connection)
            self.assertTrue(feed.snapshot()["capture_reference_ready"])

    def test_first_capture_origin_and_tracking_session_are_not_mixed(self):
        feed = SceneFeed(start=False)
        first = pose([1, 2, 3])
        self.assertTrue(feed.set_reference(first))
        self.assertFalse(feed.set_reference(pose([2, 2, 3])))
        feed._update_pose(pose([1.2, 2, 3]))
        np.testing.assert_allclose(feed.position - feed.reference.position, [.2, 0, 0])
        feed._update_pose(pose([0, 0, 0], "unitree_lio_origin_next"))
        self.assertIsNone(feed.reference)
        self.assertEqual(len(feed.path), 1)
        self.assertTrue(feed.set_reference(pose([0, 0, 0], "unitree_lio_origin_next")))

    def test_invalid_native_origins_do_not_become_old_sensor_positions(self):
        fields = np.zeros(2, dtype=POINT_DTYPE)
        fields["x"][1] = 2
        original = fields.copy()
        frame = CloudFrame(fields, time.time(), 0, received_at=time.monotonic(), stamp_ns=time.time_ns())
        current = pose([5, 2, 1])
        def transform(valid_frame):
            self.assertEqual(len(valid_frame.fields), 1)
            return valid_frame.xyz + current.position, current.session_id
        connection = NS(read_live_batch=lambda _timeout: [frame],
                        pose_history=NS(smoothed=lambda: current, points_to_world=transform))
        feed = SceneFeed(start=False)
        feed.set_connection(connection)
        feed._poll(connection)
        self.assertEqual(len(feed.snapshot()["points"]), 1)
        np.testing.assert_allclose(feed.snapshot()["points"], [[2, 0, 0]], atol=1e-6)
        np.testing.assert_array_equal(fields, original)

    def test_registered_coordinates_are_relative_to_capture_without_altering_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.bin"
            current = pose([2, 2, 3])
            fields = np.zeros(2, dtype=POINT_DTYPE)
            fields["x"], fields["y"], fields["z"] = [4, 5], 2, 3
            fields["intensity"] = [10, 20]
            payload = encode_frame(CloudFrame(fields, time.time(), 0, current.session_id,
                                              stamp_ns=time.time_ns()))
            path.write_bytes(payload)
            frame = read_frame(io.BytesIO(payload), received_at=time.monotonic())
            connection = NS(read_registered_batch=lambda: [frame], pose_history=NS(smoothed=lambda: current))
            feed = SceneFeed(start=False)
            feed.set_connection(connection)
            feed.set_reference(pose([1, 2, 3]))
            with patch("scene_feed.sensor_backend.read_view_samples", side_effect=AssertionError("raw capture stream used")):
                feed._poll(connection)
            state = feed.snapshot()
            xyz = np.column_stack([fields[name] for name in ("x", "y", "z")])
            np.testing.assert_allclose(state["points"], xyz - feed.reference.position)
            np.testing.assert_allclose(state["position"], [1, 0, 0])
            self.assertEqual(path.read_bytes(), payload)
            self.assertTrue(state["pose_ok"])
            self.assertTrue(state["registered"])

    def test_expired_cloud_and_pose_are_not_presented_as_live(self):
        feed = SceneFeed(start=False)
        feed.mode = "registered"
        feed._append(np.ones((2, 3)), np.ones((2, 3)), time.monotonic() - 20)
        history = NS(smoothed=lambda: (_ for _ in ()).throw(RuntimeError("stale")))
        connection = NS(pose_history=history, scene_path=None)
        feed.connection = connection
        feed._poll(connection)
        self.assertEqual(len(feed.snapshot()["points"]), 0)
        self.assertFalse(feed.snapshot()["pose_ok"])

    def test_no_point_cap_or_density_reduction_above_previous_limits(self):
        feed = SceneFeed(start=False)
        points = np.ones((30000, 3))
        beginning = time.monotonic()
        for i in range(30):
            feed._append(points, points, beginning + i * .15)
        self.assertEqual(len(feed.frames), 30)
        self.assertEqual(feed.frames[0][0], beginning)
        self.assertEqual(sum(len(frame[1]) for frame in feed.frames), 900000)
        feed.mode = "registered"
        connection = NS(pose_history=NS(smoothed=lambda: (_ for _ in ()).throw(RuntimeError("stale"))))
        feed.connection = connection
        feed._poll(connection)
        self.assertEqual(len(feed.snapshot()["points"]), 900000)

    def test_full_visual_copy_preserves_padded_ros_capture_payload(self):
        import struct
        fields = [NS(name=n, datatype=8, count=1, offset=i * 8)
                  for i, n in enumerate(("x", "y", "z", "intensity", "time"))]
        fields.append(NS(name="ring", datatype=6, count=1, offset=40))
        rows = [struct.pack(">dddddI", i, i + 1, i + 2, i + 3, .01, i) + b"PAD!" for i in range(10)]
        msg = NS(fields=fields, width=1, height=10, point_step=44, row_step=48,
                 data=b"".join(rows), is_bigendian=True,
                 header=NS(frame_id="unilidar_lidar", stamp=NS(sec=1, nanosec=1)))
        native = encode_pointcloud2(msg)
        preview = read_frame(io.BytesIO(encode_pointcloud2(msg)))
        self.assertEqual(len(preview.fields), 10)
        np.testing.assert_array_equal(preview.fields["x"], np.arange(10))
        self.assertEqual(native, encode_pointcloud2(msg))
        self.assertEqual(len(read_frame(io.BytesIO(native)).fields), 10)

    def test_render_limit_preserves_source_and_color_alignment(self):
        state = SceneFeed._empty_state("")
        values = np.arange(400000, dtype=np.float32)
        state.update(points=np.column_stack([values, values * 0, values * 0]),
                     colors=np.ones((len(values), 3), dtype=np.float32), intensity=values)
        view = scene_view(state)
        self.assertEqual(len(view["points"]), 300000)
        self.assertEqual(len(view["colors"]), 300000)
        np.testing.assert_array_equal(view["points"][[0, -1], 0], [0, 399999])
        self.assertEqual(len(state["points"]), 400000)
        np.testing.assert_array_equal(state["intensity"], values)

    def test_capture_arc_and_front_wall_preview_use_sensor_coordinates(self):
        state = SceneFeed._empty_state("")
        state["points"] = np.array([[2, 0, 0], [-2, 0, 0], [0, 0, 3], [0, 5, 0]], dtype=float)
        state["colors"] = np.ones((4, 3))
        state["intensity"] = np.arange(4)
        arc = dict(enabled=True, cone_angle_deg=360, max_lateral_m=.5, forward_axis="x")
        self.assertEqual(len(scene_view(state, capture_settings=arc)["points"]), 3)
        front = {**arc, "cone_angle_deg": 60}
        np.testing.assert_array_equal(scene_view(state, capture_settings=front)["points"], [[2, 0, 0]])
        self.assertEqual(len(scene_view(state)["points"]), 4)
        rotation = Rotation.from_euler("z", 90, degrees=True).as_matrix()
        state.update(points=state["points"] @ rotation.T + [3, 4, 5], position=np.array([3, 4, 5]),
                     rotation=rotation, registered=True)
        self.assertEqual(len(scene_view(state, capture_settings=front)["points"]), 1)

    def test_color_ranges_and_clear_history_do_not_change_capture_reference(self):
        feed = SceneFeed(start=False)
        reference = pose([0, 0, 0])
        feed.set_reference(reference)
        state = feed._empty_state("")
        state.update(points=np.array([[1, 0, 0], [0, 0, 3]], dtype=float),
                     colors=np.ones((2, 3)), intensity=np.array([0, 255]))
        for mode in ("intensity", "height", "distance"):
            view = scene_view(state, mode, bounds=(0, 10))
            self.assertEqual(len(view["points"]), 2)
            self.assertTrue(np.isfinite(view["colors"]).all())
        feed._append(state["points"], state["colors"], time.monotonic())
        feed.clear_history()
        self.assertEqual(len(feed.frames), 0)
        self.assertIs(feed.reference, reference)

    def test_native_points_are_all_transformed_at_their_own_timestamps(self):
        history = PoseHistory()
        beginning = time.time_ns() - 300_000_000
        for offset, position in ((0, [0, 0, 0]), (200_000_000, [1, 0, 0])):
            history.append(dict(stamp_ns=beginning + offset, position=position,
                                quaternion_xyzw=[0, 0, 0, 1], frame_id="unitree_lio_origin_test",
                                child_frame_id="unitree_lio_body"))
        fields = np.zeros(40000, dtype=POINT_DTYPE)
        fields["x"] = 2
        fields["time"] = np.linspace(0, .2, len(fields))
        original = fields.copy()
        frame = CloudFrame(fields, beginning / 1e9, 0, stamp_ns=beginning)
        world, session = history.points_to_world(frame)
        self.assertEqual(len(world), 40000)
        np.testing.assert_allclose(world[[0, -1], 0], [2.007698, 3.007698])
        np.testing.assert_array_equal(fields, original)


if __name__ == "__main__":
    unittest.main()
