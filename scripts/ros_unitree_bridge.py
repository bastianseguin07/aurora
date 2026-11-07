#!/usr/bin/env python3
"""Run with the ROS environment's Python, independently of Aurora's venv.

No SDK instance is opened here: this process only subscribes to the existing
ROS driver. stdout is exclusively full-cloud binary IPC, stderr diagnostics.
"""

import argparse
import math
import json
import os
import time
import struct
import sys
import threading

HEADER = struct.Struct("<4sIdQIqH")
RECORD = struct.Struct("<dddddI")
TYPES = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}


def encode_pointcloud2(msg, sequence=0):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 no contiene x,y,z")
    endian = ">" if msg.is_bigendian else "<"
    decoders = {}
    for name in ("x", "y", "z", "intensity", "time", "ring"):
        if name in fields:
            field = fields[name]
            if field.datatype not in TYPES or field.count != 1:
                raise ValueError("Tipo PointCloud2 no soportado: " + name)
            decoder = struct.Struct(endian + TYPES[field.datatype])
            if field.offset < 0 or field.offset + decoder.size > msg.point_step:
                raise ValueError("Offset PointCloud2 invalido")
            decoders[name] = (decoder, field.offset)
    if (
        msg.row_step < msg.width * msg.point_step
        or len(msg.data) < msg.row_step * msg.height
    ):
        raise ValueError("PointCloud2 truncado o stride invalido")
    count = msg.width * msg.height
    frame = msg.header.frame_id.encode("utf-8")
    if count > 4000000 or len(frame) > 1024:
        raise ValueError("PointCloud2 demasiado grande")
    stamp = msg.header.stamp
    seconds = (
        stamp.to_sec() if hasattr(stamp, "to_sec") else stamp.sec + stamp.nanosec * 1e-9
    )
    if hasattr(stamp, "to_nsec"):
        stamp_ns = stamp.to_nsec()
    elif hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    else:
        stamp_ns = -1  # unavailable: do not invent precision from a float
    out = bytearray(
        HEADER.pack(b"UTL2", 2, seconds, sequence, count, stamp_ns, len(frame))
    )
    out.extend(frame)
    data = memoryview(msg.data)
    for row in range(msg.height):
        for col in range(msg.width):
            offset = row * msg.row_step + col * msg.point_step
            values = {}
            for name, (decoder, position) in decoders.items():
                values[name] = decoder.unpack_from(data, offset + position)[0]
            ring = int(values.get("ring", 0xFFFFFFFF))
            if not 0 <= ring <= 0xFFFFFFFF:
                raise ValueError("Ring PointCloud2 fuera de rango")
            out.extend(
                RECORD.pack(
                    values["x"],
                    values["y"],
                    values["z"],
                    values.get("intensity", math.nan),
                    values.get("time", math.nan),
                    ring,
                )
            )
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", choices=("ros1", "ros2"), required=True)
    parser.add_argument("--topic", default="/unilidar/cloud")
    parser.add_argument("--imu-topic", default="/unilidar/imu")
    parser.add_argument("--frame", default="unilidar_lidar")
    args = parser.parse_args()
    # ROS and native dependencies may write logs to fd 1. Keep the original
    # pipe for IPC and move all ordinary stdout logging to stderr first.
    ipc = os.fdopen(os.dup(sys.stdout.fileno()), "wb")
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    sys.stdout = sys.stderr
    lock = threading.Lock()
    sequence = 0
    last_imu_log = 0.0

    def imu_callback(msg):
        nonlocal last_imu_log
        now = time.monotonic()
        if now - last_imu_log < 1:
            return
        last_imu_log = now
        stamp = msg.header.stamp
        seconds = (
            stamp.to_sec()
            if hasattr(stamp, "to_sec")
            else stamp.sec + stamp.nanosec * 1e-9
        )
        print(
            "AURORA_IMU "
            + json.dumps(
                {
                    "stamp": seconds,
                    "frame_id": msg.header.frame_id,
                    "orientation_xyzw": [
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z,
                        msg.orientation.w,
                    ],
                    "orientation_valid": bool(msg.orientation_covariance[0] >= 0),
                }
            ),
            file=sys.stderr,
            flush=True,
        )

    def callback(msg):
        nonlocal sequence
        try:
            if msg.header.frame_id != args.frame:
                raise ValueError(
                    f"Frame {msg.header.frame_id!r} distinto de {args.frame!r}"
                )
            with lock:
                payload = encode_pointcloud2(msg, getattr(msg.header, "seq", sequence))
                ipc.write(payload)
                ipc.flush()
                sequence += 1
        except Exception as exc:
            print(f"ROS bridge: {exc}", file=sys.stderr, flush=True)
            # Do not silently continue after a malformed or unwritable message.
            os._exit(2)

    try:
        from sensor_msgs.msg import PointCloud2, Imu

        if args.ros == "ros1":
            import rospy

            rospy.init_node(
                "aurora_unitree_bridge", anonymous=True, disable_signals=False
            )
            rospy.Subscriber(
                args.topic,
                PointCloud2,
                callback,
                queue_size=10,
                buff_size=32 * 1024 * 1024,
            )
            rospy.Subscriber(args.imu_topic, Imu, imu_callback, queue_size=10)
            rospy.spin()
        else:
            import rclpy
            from rclpy.qos import qos_profile_sensor_data

            rclpy.init()
            node = rclpy.create_node("aurora_unitree_bridge")
            node.create_subscription(
                PointCloud2, args.topic, callback, qos_profile_sensor_data
            )
            node.create_subscription(
                Imu, args.imu_topic, imu_callback, qos_profile_sensor_data
            )
            try:
                rclpy.spin(node)
            finally:
                node.destroy_node()
                rclpy.shutdown()
    except ImportError as exc:
        print(
            f"ROS no disponible en {sys.executable}: {exc}. Activa el entorno ROS en un proceso separado o usa UDP.",
            file=sys.stderr,
        )
        return 1
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
