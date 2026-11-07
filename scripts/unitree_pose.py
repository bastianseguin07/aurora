"""Timestamped Point-LIO poses in native Unitree cloud axes."""

from collections import deque
from dataclasses import dataclass, replace
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class UnitreePose:
    position: np.ndarray
    rpy_deg: np.ndarray
    timestamp_ns: int
    session_id: str
    frame_id: str
    quaternion_xyzw: np.ndarray
    pose_to_cloud: np.ndarray
    position_spread_m: float = 0.0


class PoseHistory:
    def __init__(self):
        self.samples = deque(maxlen=2048)
        self.lock = threading.Lock()

    def append(self, message, received_at=None):
        frame = str(message["frame_id"])
        if not frame.startswith("unitree_lio_origin_"):
            raise ValueError("Pose Unitree sin identificador de sesion Point-LIO")
        if message["child_frame_id"] != "unitree_lio_body":
            raise ValueError("Frame corporal Point-LIO no reconocido")
        position = np.asarray(message["position"], dtype=float).reshape(3)
        quaternion = np.asarray(message["quaternion_xyzw"], dtype=float).reshape(4)
        stamp = int(message["stamp_ns"])
        if (
            not np.isfinite(position).all()
            or not np.isfinite(quaternion).all()
            or stamp <= 0
        ):
            raise ValueError("Pose Point-LIO no valida")
        if abs(np.linalg.norm(quaternion) - 1) > 0.01:
            raise ValueError("Cuaternion Point-LIO no normalizado")
        now = time.monotonic() if received_at is None else received_at
        with self.lock:
            if self.samples and frame != self.samples[-1][0]["frame_id"]:
                self.samples.clear()
            if self.samples and stamp <= self.samples[-1][0]["stamp_ns"]:
                return
            self.samples.append(({**message, "stamp_ns": stamp}, now))

    def latest(self, max_age=0.75):
        with self.lock:
            if not self.samples:
                raise RuntimeError(
                    "Sin posicion SLAM Point-LIO. Ejecuta ./run_unitree_lio.sh y espera la inicializacion."
                )
            value, received = self.samples[-1]
        if (
            time.monotonic() - received > max_age
            or abs(time.time_ns() - value["stamp_ns"]) > max_age * 1e9
        ):
            raise RuntimeError(
                "Posicion Point-LIO caducada; no se usa para guiar ni compensar."
            )
        return self._pose(value)

    @staticmethod
    def _pose(value):
        rotation = Rotation.from_quat(value["quaternion_xyzw"])
        # Point-LIO estimates the IMU body pose. Our stored XYZ originate at
        # the LiDAR: apply the manufacturer's IMU->LiDAR lever arm first.
        position = np.asarray(value["position"], dtype=float) + rotation.apply(
            [0.007698, 0.014655, -0.00667]
        )
        return UnitreePose(
            position,
            rotation.as_euler("XYZ", degrees=True),
            int(value["stamp_ns"]),
            str(value["frame_id"]),
            str(value["frame_id"]),
            rotation.as_quat(),
            np.eye(3),
        )

    def at(self, stamp_ns, max_gap_ns=250_000_000):
        self.latest()  # Reject a stopped tracker even when old samples bracket a capture.
        with self.lock:
            values = [value for value, _ in self.samples]
        for before, after in zip(values, values[1:]):
            if before["stamp_ns"] <= stamp_ns <= after["stamp_ns"]:
                gap = after["stamp_ns"] - before["stamp_ns"]
                if gap > max_gap_ns:
                    break
                alpha = (stamp_ns - before["stamp_ns"]) / gap
                q = Slerp(
                    [0, 1],
                    Rotation.from_quat(
                        [before["quaternion_xyzw"], after["quaternion_xyzw"]]
                    ),
                )([alpha]).as_quat()[0]
                p = (1 - alpha) * np.asarray(before["position"]) + alpha * np.asarray(
                    after["position"]
                )
                return self._pose(
                    {
                        **before,
                        "position": p,
                        "quaternion_xyzw": q,
                        "stamp_ns": stamp_ns,
                    }
                )
        raise RuntimeError(
            "No hay poses Point-LIO que encierren el instante de captura; no se extrapola."
        )

    @staticmethod
    def average(poses, timestamp_ns):
        if any(p.session_id != poses[0].session_id for p in poses):
            raise RuntimeError("Point-LIO cambio de sesion durante la captura")
        positions = np.array([p.position for p in poses])
        rotation = Rotation.from_quat([p.quaternion_xyzw for p in poses]).mean()
        center = positions.mean(axis=0)
        spread = float(np.percentile(np.linalg.norm(positions - center, axis=1), 95))
        return replace(
            poses[-1],
            position=center,
            rpy_deg=rotation.as_euler("XYZ", degrees=True),
            quaternion_xyzw=rotation.as_quat(),
            timestamp_ns=timestamp_ns,
            position_spread_m=spread,
        )

    def smoothed(self, window_s=0.25):
        latest = self.latest()
        with self.lock:
            poses = [
                self._pose(v)
                for v, _ in self.samples
                if v["stamp_ns"] >= latest.timestamp_ns - int(window_s * 1e9)
            ]
        if len(poses) < 3:
            return latest
        # A short stationary average must not conceal an intentional move.
        if any(
            np.linalg.norm(p.position - latest.position) > 0.03
            or (
                Rotation.from_quat(p.quaternion_xyzw).inv()
                * Rotation.from_quat(latest.quaternion_xyzw)
            ).magnitude()
            > np.radians(1)
            for p in poses
        ):
            return latest
        return self.average(poses, latest.timestamp_ns)

    def for_capture(self, cloud, timeout=1.0):
        start = int(cloud.cloud_stamps_ns[0])
        end = max(
            int(stamp) + int(np.nanmax(cloud.unitree_fields["time"][a:b]) * 1e9)
            for stamp, a, b in zip(
                cloud.cloud_stamps_ns, cloud.cloud_offsets[:-1], cloud.cloud_offsets[1:]
            )
        )
        deadline = time.monotonic() + timeout
        while True:
            try:
                first, last = self.at(start), self.at(end)
                with self.lock:
                    values = [
                        self._pose(v)
                        for v, _ in self.samples
                        if start <= v["stamp_ns"] <= end
                    ]
                values += [first, last]
                values.sort(key=lambda pose: pose.timestamp_ns)
                split = max(1, len(values) // 3)
                early = self.average(values[:split], start)
                late = self.average(values[-split:], end)
                angle = (
                    Rotation.from_quat(early.quaternion_xyzw).inv()
                    * Rotation.from_quat(late.quaternion_xyzw)
                ).magnitude()
                average = self.average(values, (start + end) // 2)
                if (
                    np.linalg.norm(late.position - early.position) > 0.01
                    or angle > np.radians(1)
                    or average.position_spread_m > 0.015
                ):
                    raise ValueError(
                        "Pose inestable o sensor en movimiento durante la captura fija; se conserva la nube sin compensacion por pose."
                    )
                return average
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.02)
