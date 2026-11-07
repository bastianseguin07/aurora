"""Unitree L2: one producer, full-cloud fanout, native capture and stationary union."""

from __future__ import annotations
import ipaddress
import json
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import numpy as np
import open3d as o3d
from unitree_protocol import CloudFrame, read_frame
from unitree_pose import PoseHistory

ROOT = Path(__file__).resolve().parent.parent


@dataclass(frozen=True)
class UnitreeConfig:
    sensor_ip: str = "192.168.1.62"
    local_ip: str = "192.168.1.2"
    sensor_port: int = 6101
    local_port: int = 6201
    cloud_scan_num: int = 18
    transport: str = "udp"
    executable: str = str(ROOT / "build/unitree/unitree_bridge")
    ros_python: str = "/usr/bin/python3"
    ros_topic: str = "/unilidar/cloud"
    imu_topic: str = "/unilidar/imu"
    frame_id: str = "unilidar_lidar"
    use_system_timestamp: bool = True

    def command(self):
        if self.transport == "udp":
            ipaddress.IPv4Address(self.sensor_ip)
            ipaddress.IPv4Address(self.local_ip)
            if not 1 <= self.sensor_port <= 65535 or not 1 <= self.local_port <= 65535:
                raise ValueError("Puertos UDP fuera de rango")
            if not 1 <= self.cloud_scan_num <= 65535:
                raise ValueError("cloud_scan_num fuera de rango")
            if self.frame_id != "unilidar_lidar":
                raise ValueError(
                    "El SDK UDP publica en unilidar_lidar; un nombre distinto requiere una transformacion externa validada"
                )
            return [
                self.executable,
                "--sensor-ip",
                self.sensor_ip,
                "--local-ip",
                self.local_ip,
                "--sensor-port",
                str(self.sensor_port),
                "--local-port",
                str(self.local_port),
                "--scans",
                str(self.cloud_scan_num),
                "--system-time",
                str(int(self.use_system_timestamp)),
            ]
        if self.transport not in ("ros1", "ros2"):
            raise ValueError("Transporte Unitree desconocido")
        return [
            self.ros_python,
            str(ROOT / "scripts/ros_unitree_bridge.py"),
            "--ros",
            self.transport,
            "--topic",
            self.ros_topic,
            "--imu-topic",
            self.imu_topic,
            "--frame",
            self.frame_id,
        ]


class ConnectionClosed(RuntimeError):
    pass


class FrameHub:
    """Separate cursors let live display and capture see the same complete frames."""

    def __init__(self, max_bytes=128 * 1024 * 1024, max_frames=4096):
        self.condition = threading.Condition()
        self.frames = deque()
        self.max_bytes = max_bytes
        self.max_frames = max_frames
        self.bytes = 0
        self.sequence = 0
        self.closed = False
        self.error = None

    def publish(self, frame):
        with self.condition:
            if self.closed:
                raise ConnectionClosed(self.error or "Conexion Unitree cerrada")
            if frame.nbytes > self.max_bytes:
                raise RuntimeError(
                    "Una nube Unitree supera el limite de memoria de recepcion"
                )
            self.sequence += 1
            self.frames.append((self.sequence, frame))
            self.bytes += frame.nbytes
            while (
                self.bytes > self.max_bytes or len(self.frames) > self.max_frames
            ) and self.frames:
                self.bytes -= self.frames.popleft()[1].nbytes
            self.condition.notify_all()

    def cursor(self):
        with self.condition:
            return self.sequence - 1 if self.frames else self.sequence

    def read(self, cursor, timeout=0.2, strict=False):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                if self.frames and cursor < self.frames[-1][0]:
                    dropped = max(0, self.frames[0][0] - cursor - 1)
                    if strict and dropped:
                        raise RuntimeError(
                            f"Sobrecarga Unitree: se perdieron {dropped} nubes completas; captura incompleta"
                        )
                    item = next(item for item in self.frames if item[0] > cursor)
                    return (*item, dropped)
                if self.closed:
                    raise ConnectionClosed(self.error or "Conexion Unitree cerrada")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return cursor, None, 0
                self.condition.wait(min(remaining, 0.05))

    def close(self, error=None):
        with self.condition:
            self.closed = True
            self.error = error
            self.condition.notify_all()


class UnitreeConnection:
    def __init__(self, config, log=print, process=None, max_bytes=128 * 1024 * 1024):
        self.config = config
        self.log = log
        self.hub = FrameHub(max_bytes)
        self.local = threading.local()
        self.capture_lock = threading.Lock()
        self.stop = threading.Event()
        self.stderr_tail = deque(maxlen=12)
        self.imu_diagnostic = None
        self.pose_history = PoseHistory()
        self.process = process or subprocess.Popen(
            config.command(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )
        self.threads = [
            threading.Thread(target=self._read, daemon=True),
            threading.Thread(target=self._stderr, daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def _stderr(self):
        for line in iter(lambda: self.process.stderr.readline(4096), b""):
            # Keep diagnostics bounded even if the driver is noisy.
            text = line[-2000:].decode("utf-8", errors="replace").strip()
            if text.startswith("AURORA_POSE "):
                try:
                    self.pose_history.append(json.loads(text[len("AURORA_POSE ") :]))
                except (ValueError, KeyError, TypeError) as exc:
                    self.log(f"Pose Point-LIO descartada: {exc}")
            elif text.startswith("AURORA_IMU "):
                try:
                    self.imu_diagnostic = json.loads(text[len("AURORA_IMU ") :])
                except ValueError:
                    self.log("Diagnostico IMU ROS no valido")
            else:
                self.stderr_tail.append(text)

    def _read(self):
        try:
            while not self.stop.is_set():
                frame = read_frame(self.process.stdout)
                frame = CloudFrame(
                    frame.fields,
                    frame.stamp,
                    frame.cloud_id,
                    frame.frame_id,
                    time.monotonic(),
                    frame.stamp_ns,
                )
                if frame.frame_id != self.config.frame_id:
                    raise RuntimeError(
                        f"Frame recibido {frame.frame_id!r}; se esperaba {self.config.frame_id!r}. No se transforman ejes automaticamente."
                    )
                self.hub.publish(frame)
        except Exception as exc:
            self.hub.close(str(exc))
        finally:
            if self.process.poll() is None:
                self.process.terminate()

    def read_live(self, timeout_ms=200):
        cursor = getattr(self.local, "cursor", self.hub.cursor())
        seq, frame, dropped = self.hub.read(cursor, timeout_ms / 1000)
        self.local.cursor = seq
        if dropped:
            self.log(f"Sobrecarga de vista Unitree: {dropped} nubes descartadas")
        return frame

    def read_live_batch(self, timeout_ms=200, max_bytes=64 * 1024 * 1024):
        first = self.read_live(timeout_ms)
        frames = [] if first is None else [first]
        size = 0 if first is None else first.nbytes
        while frames and size < max_bytes and len(frames) < 256:
            try:
                more = self.read_live(0)
            except ConnectionClosed:
                break
            if more is None:
                break
            frames.append(more)
            size += more.nbytes
        return frames

    def disconnect(self):
        self.stop.set()
        self.hub.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        for thread in self.threads:
            thread.join(timeout=2)
        for pipe in (self.process.stdout, self.process.stderr):
            if pipe is not None:
                pipe.close()


def connect(config=None, log=print, timeout_s=5.0, stop_event=None):
    config = config or UnitreeConfig()
    try:
        connection = UnitreeConnection(config, log)
    except FileNotFoundError as exc:
        raise RuntimeError(
            "No se encontro el puente Unitree o el Python de ROS. Ejecuta ./install_unitree_sdk.sh o revisa la configuracion ROS."
        ) from exc
    try:
        frame = None
        deadline = time.monotonic() + timeout_s
        while frame is None and time.monotonic() < deadline:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("Conexion Unitree cancelada")
            _, frame, _ = connection.hub.read(
                0, min(0.05, max(0.0, deadline - time.monotonic()))
            )
        if frame is None:
            raise RuntimeError(
                "No llegaron nubes Unitree: revisa red, IP local, puertos, driver y sensor"
            )
        log(
            f"Unitree conectado: {frame.frame_id}, SDK cloud_scan_num={config.cloud_scan_num}"
            if config.transport == "udp"
            else f"Unitree conectado por {config.transport}: {config.ros_topic}, {frame.frame_id}"
        )
        return connection
    except Exception as exc:
        connection.disconnect()
        details = "; ".join(connection.stderr_tail)
        raise RuntimeError(f"{exc}. {details}") from exc


def filter_frame(
    frame, max_distance_m=None, cone_angle_deg=None, forward_axis="x", **_
):
    xyz = frame.xyz
    valid = np.isfinite(xyz).all(axis=1) & np.any(xyz != 0, axis=1)
    distance = np.linalg.norm(xyz, axis=1)
    if max_distance_m is not None and max_distance_m > 0:
        valid &= distance < max_distance_m
    if cone_angle_deg is not None:
        if not 0 < cone_angle_deg <= 180 or forward_axis not in ("x", "y", "z"):
            raise ValueError("Cono o eje frontal invalido")
        safe = np.where(distance > 0, distance, 1)
        valid &= xyz[:, "xyz".index(forward_axis)] / safe >= np.cos(
            np.radians(cone_angle_deg / 2)
        )
    return frame.subset(valid)


def intensity_colors(fields):
    intensity = np.nan_to_num(fields["intensity"], nan=0, posinf=0, neginf=0)
    span = np.ptp(intensity) if len(intensity) else 0
    gray = (
        (intensity - intensity.min()) / span
        if span > 0
        else np.full(len(intensity), 0.72)
    )
    return np.repeat(gray[:, None], 3, axis=1)


class UnitreeCloud:
    """Open3D-compatible result plus original scalar fields and cloud boundaries."""

    def __init__(self, frames, config):
        self.geometry = o3d.geometry.PointCloud()
        self.unitree_fields = np.concatenate([frame.fields for frame in frames])
        self.cloud_offsets = np.r_[0, np.cumsum([len(f.fields) for f in frames])]
        self.cloud_stamps = np.array([f.stamp for f in frames])
        self.cloud_stamps_ns = np.array([f.stamp_ns for f in frames], dtype=np.int64)
        self.cloud_ids = np.array([f.cloud_id for f in frames], dtype=np.uint64)
        self.frame_id = frames[0].frame_id
        self.config = config
        xyz = np.column_stack([self.unitree_fields[n] for n in ("x", "y", "z")])
        self.geometry.points = o3d.utility.Vector3dVector(xyz)
        self.geometry.colors = o3d.utility.Vector3dVector(
            intensity_colors(self.unitree_fields)
        )

    @property
    def points(self):
        return self.geometry.points

    @property
    def colors(self):
        return self.geometry.colors


def capture_snapshot(
    connection,
    capture_duration_s=0,
    accumulate_static=False,
    stop_event=None,
    timeout_s=10,
    persistence_ratio=0,
    max_points_per_frame=None,
    max_distance_m=None,
    cone_angle_deg=None,
    forward_axis="x",
    max_capture_bytes=256 * 1024 * 1024,
    **_,
):
    if (
        not np.isfinite(capture_duration_s)
        or not 0 <= capture_duration_s <= 60
        or not np.isfinite(timeout_s)
        or timeout_s <= 0
    ):
        raise ValueError("Duracion de captura invalida")
    if accumulate_static and capture_duration_s <= 0:
        raise ValueError("Acumular requiere una duracion mayor que cero")
    if persistence_ratio or max_points_per_frame:
        raise ValueError(
            "Unitree nativo no aplica persistencia ni limite de puntos. Desactiva esas opciones."
        )
    if not connection.capture_lock.acquire(blocking=False):
        raise RuntimeError("Ya hay una captura Unitree en curso")
    try:
        frames = []
        used_bytes = 0
        cursor = connection.hub.sequence
        start = time.monotonic()
        deadline = start + capture_duration_s
        while time.monotonic() - start < timeout_s:
            if stop_event is not None and stop_event.is_set():
                break
            # Drain complete frames received within the interval, including any
            # queued before the deadline while previous frames were processed.
            remaining = (
                max(0.0, deadline - time.monotonic()) if accumulate_static else 0.05
            )
            try:
                cursor, frame, _ = connection.hub.read(
                    cursor, min(0.05, remaining), strict=True
                )
            except ConnectionClosed:
                if accumulate_static and time.monotonic() >= deadline:
                    break
                raise
            if frame is None:
                if accumulate_static and time.monotonic() >= deadline:
                    break
                continue
            if accumulate_static and frame.received_at > deadline:
                break
            frame = filter_frame(frame, max_distance_m, cone_angle_deg, forward_axis)
            if not len(frame.fields):
                continue
            used_bytes += frame.nbytes
            if used_bytes > max_capture_bytes:
                raise RuntimeError(
                    "Captura Unitree excede el limite de memoria; no se guarda una captura parcial. Reduce la duracion."
                )
            frames.append(frame)
            if not accumulate_static:
                break
        if not frames:
            raise RuntimeError(
                "Captura Unitree sin puntos validos (cancelada, sin datos o filtros demasiado restrictivos)"
            )
        elapsed = time.monotonic() - start
        if (
            accumulate_static
            and elapsed < capture_duration_s
            and not (stop_event and stop_event.is_set())
        ):
            raise RuntimeError("No se completo el tiempo de acumulacion Unitree")
        connection.log(
            f"Captura Unitree: {len(frames)} nubes, {sum(len(f.fields) for f in frames)} puntos, {elapsed:.2f} s"
        )
        return UnitreeCloud(frames, connection.config)
    finally:
        connection.capture_lock.release()


def save_metadata(path, cloud):
    if not isinstance(cloud, UnitreeCloud):
        return
    target = Path(path).with_name(Path(path).stem + "_unitree.npz")
    np.savez_compressed(
        target,
        fields=cloud.unitree_fields,
        cloud_offsets=cloud.cloud_offsets,
        cloud_stamps=cloud.cloud_stamps,
        cloud_stamps_ns=cloud.cloud_stamps_ns,
        cloud_ids=cloud.cloud_ids,
        frame_id=cloud.frame_id,
        sensor="unitree_l2",
        units="meters",
        cloud_scan_num=(
            cloud.config.cloud_scan_num if cloud.config.transport == "udp" else -1
        ),
        timestamp_source=(
            ("system" if cloud.config.use_system_timestamp else "device")
            if cloud.config.transport == "udp"
            else "ros_header"
        ),
        transport=cloud.config.transport,
        point_time_semantics="seconds_relative_to_cloud_stamp",
    )
    return target
