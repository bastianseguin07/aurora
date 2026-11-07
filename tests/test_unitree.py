import io
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from unitree_protocol import CloudFrame, POINT_DTYPE, HEADER, encode_frame, read_frame
from unitree_sensor import (
    UnitreeConfig,
    UnitreeConnection,
    FrameHub,
    capture_snapshot,
    save_metadata,
    connect,
)
from live_decay import DecayBuffer
import sensor_backend
from sensor_backend import LiveSample
from ros_unitree_bridge import encode_pointcloud2


def frame(number=0, size=4, stamp=None):
    data = np.zeros(size, dtype=POINT_DTYPE)
    data["x"] = np.linspace(1.123456789, 2.123456789, size)
    data["y"] = number * 0.003
    data["z"] = np.arange(size) * 0.01
    data["intensity"] = np.arange(size) + 25
    data["time"] = np.arange(size) * 0.00001
    data["ring"] = np.arange(size) + 7
    return CloudFrame(
        data,
        123456.789 if stamp is None else stamp,
        number,
        received_at=time.monotonic(),
    )


def memory_connection():
    c = UnitreeConnection.__new__(UnitreeConnection)
    c.config = UnitreeConfig()
    c.hub = FrameHub()
    c.log = lambda _: None
    c.capture_lock = threading.Lock()
    c.local = threading.local()
    return c


def publish_during_capture(connection, frames, delay=0.005):
    def producer():
        while not connection.capture_lock.locked():
            time.sleep(0.001)
        for f in frames:
            time.sleep(delay)
            connection.hub.publish(replace(f, received_at=time.monotonic()))

    thread = threading.Thread(target=producer, daemon=True)
    thread.start()
    return thread


class ProtocolTests(unittest.TestCase):
    def test_full_frames_fields_and_boundaries(self):
        frames = [frame(1), frame(2, size=9)]
        stream = io.BytesIO(b"".join(encode_frame(f) for f in frames))
        for expected in frames:
            got = read_frame(stream)
            np.testing.assert_array_equal(got.fields, expected.fields)
            self.assertEqual(got.cloud_id, expected.cloud_id)
            self.assertEqual(got.stamp, expected.stamp)
            self.assertEqual(got.frame_id, expected.frame_id)
        with self.assertRaises(EOFError):
            read_frame(stream)

    def test_ros_integer_nanoseconds_are_not_rounded_through_float(self):
        msg = NS(
            fields=[
                NS(name=n, datatype=8, count=1, offset=i * 8)
                for i, n in enumerate(("x", "y", "z"))
            ],
            width=1,
            height=1,
            point_step=24,
            row_step=24,
            is_bigendian=False,
            data=struct.pack("<ddd", 1, 2, 3),
            header=NS(
                frame_id="unilidar_lidar", stamp=NS(sec=1_800_000_000, nanosec=1)
            ),
        )
        got = read_frame(io.BytesIO(encode_pointcloud2(msg)))
        self.assertEqual(got.stamp_ns, 1_800_000_000_000_000_001)
        saved = sensor_backend.unitree.UnitreeCloud(
            [got], UnitreeConfig(transport="ros2")
        )
        with tempfile.TemporaryDirectory() as d:
            target = save_metadata(Path(d) / "ros.ply", saved)
            with np.load(target) as data:
                self.assertEqual(data["cloud_stamps_ns"][0], 1_800_000_000_000_000_001)

    def test_fragmented_pipe_and_truncated_frame(self):
        class Fragmented(io.BytesIO):
            def read(self, n=-1):
                return super().read(min(n, 7))

        raw = encode_frame(frame())
        np.testing.assert_array_equal(
            read_frame(Fragmented(raw)).fields, frame().fields
        )
        with self.assertRaises((ValueError, EOFError)):
            read_frame(io.BytesIO(raw[:-10]))
        with self.assertRaises(ValueError):
            read_frame(io.BytesIO(b"BAD!" + raw[4:]))

    def test_pointcloud2_endianness_row_padding_and_float64(self):
        for endian in (">", "<"):
            fields = [
                NS(name=n, datatype=8, count=1, offset=i * 8)
                for i, n in enumerate(("x", "y", "z", "intensity", "time"))
            ]
            fields.append(NS(name="ring", datatype=6, count=1, offset=40))
            records = frame().fields
            rows = []
            for point in records:
                rows.append(struct.pack(endian + "dddddI", *point.tolist()) + b"PAD!")
            msg = NS(
                fields=fields,
                width=1,
                height=4,
                point_step=44,
                row_step=48,
                data=b"".join(rows),
                is_bigendian=endian == ">",
                header=NS(
                    frame_id="unilidar_lidar", stamp=NS(sec=7, nanosec=500000000)
                ),
            )
            decoded = read_frame(io.BytesIO(encode_pointcloud2(msg, sequence=27)))
            np.testing.assert_array_equal(decoded.fields, records)
            self.assertEqual(decoded.cloud_id, 27)
            self.assertEqual(decoded.stamp, 7.5)


class CaptureTests(unittest.TestCase):
    def test_native_is_one_fresh_complete_cloud_and_saves_auxiliary_fields(self):
        connection = memory_connection()
        connection.hub.publish(frame(99))  # stale buffered data must not be captured
        expected = frame(3)
        producer = publish_during_capture(connection, [expected])
        cloud = capture_snapshot(connection)
        producer.join(1)
        np.testing.assert_array_equal(cloud.unitree_fields, expected.fields)
        np.testing.assert_array_equal(np.asarray(cloud.points), expected.xyz)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "native.ply"
            self.assertTrue(o3d.io.write_point_cloud(str(path), cloud.geometry))
            sidecar = save_metadata(path, cloud)
            np.testing.assert_array_equal(
                np.asarray(o3d.io.read_point_cloud(str(path)).points), expected.xyz
            )
            with np.load(sidecar, allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved["fields"], expected.fields)
                np.testing.assert_array_equal(saved["cloud_offsets"], [0, 4])
                np.testing.assert_array_equal(saved["cloud_ids"], [3])
                self.assertEqual(saved["units"], "meters")

    def test_reused_filename_does_not_attach_an_old_pose_or_photo(self):
        cloud = sensor_backend.unitree.UnitreeCloud([frame()], UnitreeConfig())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.ply"
            old_pose = path.with_name("capture_pose.npz")
            old_photo = path.with_name("capture_ref.npz")
            old_pose.write_bytes(b"previous pose")
            old_photo.write_bytes(b"previous photo")
            sensor_backend.save_capture_metadata(path, cloud)
            self.assertFalse(old_pose.exists())
            self.assertFalse(old_photo.exists())
            self.assertEqual(
                next(path.parent.glob("capture_pose_superseded_*.npz")).read_bytes(),
                b"previous pose",
            )
            sensor_backend.save_capture_metadata(path, o3d.geometry.PointCloud())
            self.assertFalse(path.with_name("capture_unitree.npz").exists())
            self.assertTrue(list(path.parent.glob("capture_unitree_superseded_*.npz")))

    def test_accumulation_concatenates_every_return_without_averaging(self):
        connection = memory_connection()
        frames = [frame(1), frame(2), frame(3)]
        producer = publish_during_capture(connection, frames)
        cloud = capture_snapshot(
            connection, capture_duration_s=0.06, accumulate_static=True
        )
        producer.join(1)
        np.testing.assert_array_equal(
            cloud.unitree_fields, np.concatenate([f.fields for f in frames])
        )
        np.testing.assert_array_equal(cloud.cloud_offsets, [0, 4, 8, 12])
        np.testing.assert_array_equal(cloud.cloud_ids, [1, 2, 3])

    def test_cancel_connection_while_waiting_for_first_cloud(self):
        command = [sys.executable, "-c", "import time;time.sleep(20)"]
        stop = threading.Event()
        timer = threading.Timer(0.03, stop.set)
        timer.start()
        start = time.monotonic()
        with patch.object(UnitreeConfig, "command", return_value=command):
            with self.assertRaisesRegex(RuntimeError, "cancelada"):
                connect(UnitreeConfig(transport="ros1"), timeout_s=5, stop_event=stop, log=lambda _: None)
        timer.join(1)
        self.assertLess(time.monotonic() - start, 1)

    def test_capture_drains_frames_queued_before_deadline(self):
        connection = memory_connection()
        frames = [frame(1), frame(2), frame(3)]
        producer = publish_during_capture(connection, frames)
        original = sensor_backend.unitree.filter_frame

        def slow_filter(*args, **kwargs):
            time.sleep(0.03)
            return original(*args, **kwargs)

        with patch("unitree_sensor.filter_frame", side_effect=slow_filter):
            cloud = capture_snapshot(
                connection, capture_duration_s=0.025, accumulate_static=True
            )
        producer.join(1)
        np.testing.assert_array_equal(cloud.cloud_ids, [1, 2, 3])

    def test_cancel_without_data_returns_promptly(self):
        connection = memory_connection()
        stop = threading.Event()
        timer = threading.Timer(0.03, stop.set)
        timer.start()
        start = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, "sin puntos"):
            capture_snapshot(connection, stop_event=stop)
        timer.join(1)
        self.assertLess(time.monotonic() - start, 0.5)

    def test_no_data_timeout_and_closed_connection(self):
        connection = memory_connection()
        with self.assertRaisesRegex(RuntimeError, "sin puntos"):
            capture_snapshot(connection, timeout_s=0.05)
        connection.hub.close()
        with self.assertRaisesRegex(RuntimeError, "cerrada"):
            capture_snapshot(connection)

    def test_overload_aborts_capture_without_silent_loss(self):
        connection = memory_connection()
        producer = publish_during_capture(connection, [frame(1)])
        with self.assertRaisesRegex(RuntimeError, "limite de memoria"):
            capture_snapshot(connection, max_capture_bytes=1)
        producer.join(1)
        self.assertFalse(connection.capture_lock.locked())
        hub = FrameHub(max_bytes=frame().nbytes)
        hub.publish(frame(1))
        hub.publish(frame(2))
        with self.assertRaisesRegex(RuntimeError, "perdieron 1"):
            hub.read(0, strict=True)
        _, got, dropped = hub.read(0)
        self.assertEqual(got.cloud_id, 2)
        self.assertEqual(dropped, 1)

    def test_empty_messages_are_count_bounded_too(self):
        hub = FrameHub(max_bytes=1024, max_frames=2)
        for i in range(3):
            hub.publish(frame(i, size=0))
        self.assertEqual(len(hub.frames), 2)
        with self.assertRaisesRegex(RuntimeError, "perdieron 1"):
            hub.read(0, strict=True)
        hub.close()
        with self.assertRaisesRegex(RuntimeError, "cerrada"):
            hub.publish(frame(4))

    def test_independent_live_and_capture_cursors(self):
        hub = FrameHub()
        hub.publish(frame(1))
        hub.publish(frame(2))
        for _ in range(2):
            cursor, a, _ = hub.read(0)
            _, b, _ = hub.read(cursor)
            self.assertEqual((a.cloud_id, b.cloud_id), (1, 2))

    def test_unitree_never_fakes_pose_or_camera(self):
        connection = memory_connection()
        self.assertFalse(sensor_backend.capabilities(connection).pose)
        self.assertEqual(
            sensor_backend.capture_reference_frame(connection), (None, None)
        )
        with self.assertRaisesRegex(RuntimeError, "SLAM"):
            sensor_backend.get_current_pose(connection)
        with self.assertRaisesRegex(RuntimeError, "pose validada"):
            sensor_backend.capture_aligned_snapshot(connection)


class ProcessTests(unittest.TestCase):
    def test_stderr_is_drained_and_idle_process_can_be_closed(self):
        code = 'import sys,time; sys.stderr.write("x"*200000+"\\n"); sys.stderr.flush(); time.sleep(20)'
        process = subprocess.Popen(
            [sys.executable, "-c", code],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        connection = UnitreeConnection(UnitreeConfig(), process=process)
        try:
            time.sleep(0.05)
            start = time.monotonic()
            connection.disconnect()
            self.assertLess(time.monotonic() - start, 3)
            self.assertIsNotNone(process.poll())
            self.assertTrue(all(not t.is_alive() for t in connection.threads))
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()

    def test_connection_timeout_cleans_up_process(self):
        command = [sys.executable, "-c", "import time;time.sleep(20)"]
        with patch.object(UnitreeConfig, "command", return_value=command):
            with self.assertRaisesRegex(RuntimeError, "No llegaron"):
                connect(UnitreeConfig(transport="ros1"), timeout_s=0.03, log=lambda _: None)

    def test_missing_ros_is_a_clear_subprocess_error(self):
        config = UnitreeConfig(transport="ros1", ros_python=sys.executable)
        with self.assertRaisesRegex(RuntimeError, "ROS no disponible"):
            connect(config, timeout_s=2, log=lambda _: None)


class DecayTests(unittest.TestCase):
    def test_live_batches_keep_fields_and_do_not_save_the_decay_union_or_axis_flip(
        self,
    ):
        from live_viewer import LiveViewer

        connection = memory_connection()
        viewer = LiveViewer(o3d.geometry.PointCloud())
        viewer.set_decay_time(10)
        connection.hub.publish(frame(1))
        viewer._read_live_cloud(connection, None, None, "x", True, False)
        viewer.capture_live_baseline()
        connection.hub.publish(frame(2))
        connection.hub.publish(frame(3))
        shown = viewer._read_live_cloud(connection, None, None, "x", True, False)
        self.assertEqual(len(shown.points), 12)
        base, current, _, base_meta, current_meta = viewer.get_live_snapshot_clouds(
            include_metadata=True
        )
        np.testing.assert_array_equal(np.asarray(base.points), frame(1).xyz)
        np.testing.assert_array_equal(np.asarray(current.points), frame(3).xyz)
        np.testing.assert_array_equal(base_meta.unitree_fields, frame(1).fields)
        np.testing.assert_array_equal(current_meta.unitree_fields, frame(3).fields)
        viewer.set_decay_time(0)
        _, current_again, _ = viewer.get_live_snapshot_clouds()
        np.testing.assert_array_equal(np.asarray(current_again.points), frame(3).xyz)

    def test_empty_frames_are_transmitted_to_clear_the_phone_view(self):
        from live_stream_server import LiveStreamServer, MSG_LIVE, MSG_BASE
        from unittest.mock import Mock

        server = LiveStreamServer()
        with patch.object(server, "is_running", return_value=True), patch.object(
            server, "_broadcast"
        ) as send:
            empty = np.empty((0, 3))
            server.broadcast_live(empty, empty)
            self.assertEqual(
                struct.unpack("<II", send.call_args.args[0]), (MSG_LIVE, 0)
            )
            self.assertTrue(server.broadcast_base(empty, empty))
            self.assertEqual(
                struct.unpack("<II", send.call_args.args[0]), (MSG_BASE, 0)
            )

    def test_expiration_without_new_messages(self):
        b = DecayBuffer(decay_s=1)
        b.append(frame().xyz, np.ones((4, 3)), now=0)
        self.assertEqual(len(b.snapshot(0.9)[0]), 4)
        self.assertEqual(len(b.snapshot(1.01)[0]), 0)
        self.assertEqual(b.bytes, 0)

    def test_zero_means_latest_cloud_not_unlimited_history(self):
        b = DecayBuffer(decay_s=0)
        b.append(frame(1).xyz, np.ones((4, 3)), now=0)
        b.append(frame(2).xyz, np.ones((4, 3)), now=1)
        np.testing.assert_array_equal(b.snapshot(100)[0], frame(2).xyz)

    def test_memory_drops_are_visible(self):
        messages = []
        b = DecayBuffer(decay_s=10, max_bytes=200, log=messages.append)
        b.append(frame(1).xyz, np.ones((4, 3)), now=0)
        b.append(frame(2).xyz, np.ones((4, 3)), now=1)
        self.assertEqual(b.dropped, 1)
        self.assertTrue(messages)
        np.testing.assert_array_equal(b.snapshot(1)[0], frame(2).xyz)

    def test_native_live_snapshot_is_independent_from_display_history(self):
        from live_viewer import LiveViewer

        viewer = LiveViewer(o3d.geometry.PointCloud())
        viewer.set_decay_time(1)
        connection = object()
        with patch(
            "live_viewer.read_view_samples",
            side_effect=[
                [LiveSample(frame(1).xyz, np.ones((4, 3)), time.monotonic())],
                [LiveSample(frame(2).xyz, np.ones((4, 3)), time.monotonic())],
            ],
        ):
            viewer._read_live_cloud(connection, None, None, "x", False, False)
            shown = viewer._read_live_cloud(connection, None, None, "x", False, False)
        self.assertEqual(len(shown.points), 8)
        _, native, _ = viewer.get_live_snapshot_clouds()
        np.testing.assert_array_equal(np.asarray(native.points), frame(2).xyz)
        viewer._decay.frames[0] = (-2, *viewer._decay.frames[0][1:])
        viewer._decay.frames[1] = (-2, *viewer._decay.frames[1][1:])
        viewer.set_color_mode("continuous", 0.05, 0.1, None)
        viewer._distance_reference_cloud = native
        with patch("live_viewer.read_view_samples", return_value=[]):
            shown = viewer._read_live_cloud(connection, None, None, "x", False, False)
        self.assertEqual(len(shown.points), 0)


if __name__ == "__main__":
    unittest.main()
