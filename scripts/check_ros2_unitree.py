"""Real ROS smoke check using a synthetic publisher on an isolated DDS domain."""

import os

os.environ["ROS_DOMAIN_ID"] = "137"
import struct
import threading
import time
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
from unitree_sensor import UnitreeConfig, connect, capture_snapshot
from unitree_protocol import read_frame
from scene_feed import SceneFeed


def main():
    rclpy.init()
    node = rclpy.create_node("aurora_unitree_synthetic_check")
    publisher = node.create_publisher(
        PointCloud2, "/unilidar/cloud", qos_profile_sensor_data
    )
    imu_publisher = node.create_publisher(Imu, "/unilidar/imu", qos_profile_sensor_data)
    pose_publisher = node.create_publisher(Odometry, "/unitree_lio/odometry", qos_profile_sensor_data)
    scene_publisher = node.create_publisher(PointCloud2, "/unitree_lio/cloud_registered", qos_profile_sensor_data)
    records = [(1.25 + i, 0.5, 0.25, 12 + i, 0.125, 70000 + i) for i in range(4)]

    def publish():
        message = PointCloud2()
        message.header.frame_id = "unilidar_lidar"
        message.header.stamp = node.get_clock().now().to_msg()
        message.height = 1
        message.width = 4
        message.is_bigendian = False
        message.is_dense = True
        message.point_step = 44
        message.row_step = 176
        message.fields = [
            PointField(name=n, offset=i * 8, datatype=PointField.FLOAT64, count=1)
            for i, n in enumerate(("x", "y", "z", "intensity", "time"))
        ]
        message.fields.append(
            PointField(name="ring", offset=40, datatype=PointField.UINT32, count=1)
        )
        message.data = list(
            b"".join(struct.pack("<dddddI", *point) for point in records)
        )
        publisher.publish(message)
        import copy
        registered = copy.deepcopy(message)
        registered.header.frame_id = "unitree_lio_origin_synthetic_check"
        scene_publisher.publish(registered)
        imu = Imu()
        imu.header.stamp = message.header.stamp
        imu.header.frame_id = "unilidar_imu"
        imu.orientation.w = 1.0
        imu_publisher.publish(imu)
        pose = Odometry()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = "unitree_lio_origin_synthetic_check"
        pose.child_frame_id = "unitree_lio_body"
        pose.pose.pose.orientation.w = 1.0
        pose_publisher.publish(pose)

    node.create_timer(0.1, publish)

    def spin_node():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin_node, daemon=True)
    thread.start()
    connection = None
    try:
        config = UnitreeConfig(
            transport="ros2", ros_python=str(Path(__file__).with_name("ros2_python.sh"))
        )
        connection = connect(config, timeout_s=10)
        cloud = capture_snapshot(connection, timeout_s=5)
        np.testing.assert_array_equal(
            np.asarray(cloud.points), np.array(records)[:, :3]
        )
        np.testing.assert_array_equal(
            cloud.unitree_fields["ring"], [70000, 70001, 70002, 70003]
        )
        assert np.all(cloud.cloud_stamps_ns > 0)
        accumulated = capture_snapshot(
            connection, capture_duration_s=0.4, accumulate_static=True, timeout_s=3
        )
        assert len(accumulated.cloud_ids) >= 2
        assert len(accumulated.points) == 4 * len(accumulated.cloud_ids)
        # ROS 2 covariance uses NumPy scalars: diagnostic JSON must convert its
        # validity flag to a Python bool and keep the cloud process alive.
        deadline = time.monotonic() + 2.0
        received = 0
        while time.monotonic() < deadline:
            if connection.read_live(500) is not None:
                received += 1
        assert received >= 5
        assert connection.process.poll() is None
        assert connection.imu_diagnostic["orientation_valid"] is True
        assert connection.pose_history.latest().session_id == "unitree_lio_origin_synthetic_check"
        assert connection.pose_history.for_capture(accumulated).position_spread_m < 1e-9
        registered = connection.read_registered_batch(1000)[-1]
        assert registered.frame_id == "unitree_lio_origin_synthetic_check"
        np.testing.assert_array_equal(registered.xyz, np.array(records)[:, :3])
        scene = SceneFeed(start=False)
        scene.set_connection(connection)
        reference = connection.pose_history.for_capture(accumulated)
        assert scene.set_reference(reference)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            scene._poll(connection)
            if len(scene.snapshot()["points"]):
                break
            time.sleep(.05)
        state = scene.snapshot()
        assert state["pose_ok"] and state["registered"]
        np.testing.assert_allclose(state["position"], [0, 0, 0], atol=1e-9)
        assert len(state["points"]) >= len(records)
        np.testing.assert_allclose(state["points"][:4], np.array(records)[:, :3], atol=1e-6)
        np.testing.assert_array_equal(cloud.unitree_fields["ring"], [70000, 70001, 70002, 70003])
        print(
            "ROS real: captura nativa, IMU, pose, nube registrada y origen de la escena verificados. Dominio aislado 137."
        )
    finally:
        if connection:
            connection.disconnect()
        rclpy.shutdown()
        thread.join(timeout=2)
        node.destroy_node()


if __name__ == "__main__":
    main()
