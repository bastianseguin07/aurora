"""Application sensor API: hardware-specific logic stays inside each adapter."""

from dataclasses import dataclass, replace
from typing import Union
import time
from pathlib import Path
import open3d as o3d
import aurora_sensor as aurora
import unitree_sensor as unitree

AuroraConnection = Union[aurora.AuroraConnection, unitree.UnitreeConnection]
SensorPose = aurora.SensorPose


@dataclass(frozen=True)
class Capabilities:
    name: str
    pose: bool
    reference_image: bool
    organized_cloud: bool
    forward_axis: str
    frame_description: str


AURORA = Capabilities(
    "Slamtec Aurora",
    True,
    True,
    True,
    "z",
    "Camara: X derecha, Y abajo, Z adelante; metros",
)
UNITREE = Capabilities(
    "Unitree LiDAR L2",
    False,
    False,
    False,
    "x",
    "LiDAR: +X opuesto al cable, +Y a 90° antihorario, +Z hacia arriba; metros",
)


def capabilities(connection=None, kind="aurora"):
    if isinstance(connection, unitree.UnitreeConnection):
        try:
            connection.pose_history.latest()
        except (RuntimeError, AttributeError):
            return UNITREE
        return replace(UNITREE, pose=True)
    return (
        UNITREE
        if isinstance(connection, unitree.UnitreeConnection) or kind == "unitree"
        else AURORA
    )


def connect(
    connection_string="192.168.11.1",
    *,
    kind="aurora",
    config=None,
    log=print,
    stop_event=None,
):
    return (
        unitree.connect(config, log, stop_event=stop_event)
        if kind == "unitree"
        else aurora.connect(connection_string)
    )


def disconnect(connection):
    return (
        connection.disconnect()
        if isinstance(connection, unitree.UnitreeConnection)
        else aurora.disconnect(connection)
    )


def read_frame_points_and_colors(connection, **kwargs):
    if not isinstance(connection, unitree.UnitreeConnection):
        return aurora.read_frame_points_and_colors(connection, **kwargs)
    frame = connection.read_live(kwargs.get("timeout_ms", 200))
    if frame is None:
        return None, None
    frame = unitree.filter_frame(frame, **kwargs)
    connection.local.last_frame = frame
    return frame.xyz, unitree.intensity_colors(frame.fields)


def capture_snapshot(connection, **kwargs):
    return (
        unitree.capture_snapshot(connection, **kwargs)
        if isinstance(connection, unitree.UnitreeConnection)
        else aurora.capture_snapshot(connection, **kwargs)
    )


def get_current_pose(connection):
    if isinstance(connection, unitree.UnitreeConnection):
        if not hasattr(connection, "pose_history"):
            raise RuntimeError("Sin posicion SLAM Point-LIO disponible")
        return connection.pose_history.smoothed()
    if not capabilities(connection).pose:
        raise RuntimeError(
            "Unitree L2 no proporciona posicion SLAM validada; su IMU no es una pose 6DoF"
        )
    return aurora.get_current_pose(connection)


def get_capture_pose(connection, capture):
    if isinstance(connection, unitree.UnitreeConnection):
        return connection.pose_history.for_capture(capture)
    return get_current_pose(connection)


def capture_reference_frame(connection, **kwargs):
    if not capabilities(connection).reference_image:
        return None, None
    return aurora.capture_reference_frame(connection, **kwargs)


def capture_aligned_snapshot(connection, **kwargs):
    if isinstance(connection, unitree.UnitreeConnection) or not capabilities(connection).pose:
        raise RuntimeError(
            "Captura movil no disponible: Unitree requiere una fuente externa de pose validada"
        )
    return aurora.capture_aligned_snapshot(connection, **kwargs)


def save_capture_metadata(path, cloud):
    result = unitree.save_metadata(path, cloud)
    # Reusing a filename must not attach an older sensor's pose/photo to new
    # geometry. Preserve superseded auxiliary files under distinct names.
    suffixes = (
        ("_pose.npz", "_ref.npz")
        if isinstance(cloud, unitree.UnitreeCloud)
        else (("_unitree.npz",) if isinstance(cloud, o3d.geometry.PointCloud) else ())
    )
    base = Path(path)
    for suffix in suffixes:
        stale = base.with_name(base.stem + suffix)
        if stale.exists():
            archive = stale.with_name(stale.stem + f"_superseded_{time.time_ns()}.npz")
            stale.rename(archive)
    return result


def cloud_geometry(result):
    return result.geometry if isinstance(result, unitree.UnitreeCloud) else result


def latest_native_capture(connection):
    if isinstance(connection, unitree.UnitreeConnection):
        frame = getattr(connection.local, "last_frame", None)
        return (
            unitree.UnitreeCloud([frame], connection.config)
            if frame is not None
            else None
        )
    return None


@dataclass(frozen=True)
class LiveSample:
    points: object
    colors: object
    received_at: float
    native_capture: object = None


def read_view_samples(connection, **kwargs):
    if not isinstance(connection, unitree.UnitreeConnection):
        points, colors = aurora.read_frame_points_and_colors(connection, **kwargs)
        return [] if points is None else [LiveSample(points, colors, time.monotonic())]
    frames = connection.read_live_batch(kwargs.get("timeout_ms", 200))
    samples = []
    for i, frame in enumerate(frames):
        filtered = unitree.filter_frame(frame, **kwargs)
        native = (
            unitree.UnitreeCloud([filtered], connection.config)
            if i == len(frames) - 1 and len(filtered.fields)
            else None
        )
        samples.append(
            LiveSample(
                filtered.xyz,
                unitree.intensity_colors(filtered.fields),
                frame.received_at,
                native,
            )
        )
    return samples
