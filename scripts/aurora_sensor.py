"""
Integracion con el sensor Slamtec Aurora (SLAMWARE-Aurora-XXXX) via su SDK
oficial (paquete 'slamtec_aurora_sdk', repo Slamtec/py_aurora_remote).

Este modulo NO se pudo probar contra hardware real (no hay un Aurora
conectado en el entorno de desarrollo). Las llamadas a la SDK siguen
exactamente el patron del ejemplo oficial 'examples/dense_point_cloud.py'
del repositorio. Antes de confiar en el flujo completo, probalo primero
con el sensor conectado.

Instalacion del SDK: ver la seccion correspondiente del README del proyecto.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation, Slerp

_STATIC_ACCUMULATION_MAX_FRAMES = 60
_STATIC_ACCUMULATION_SAMPLE_RATE_HZ = 2
_STATIC_ACCUMULATION_MEMORY_BUDGET_BYTES = 128 * 1024 * 1024


@dataclass
class AlignedCaptureResult:
    cloud: o3d.geometry.PointCloud
    reference_pose: "SensorPose"
    accepted_frames: int
    rejected_frames: int
    path_length_m: float
    final_translation_m: float
    final_rotation_deg: float
    pose_augmentation_used: bool = False
    extrinsic_calibration_used: bool = False


class AuroraNotAvailable(RuntimeError):
    pass


def _import_sdk():
    try:
        from slamtec_aurora_sdk import (
            AuroraSDK,
            ENHANCED_IMAGE_TYPE_DEPTH,
            DEPTHCAM_FRAME_TYPE_POINT3D,
        )
        from slamtec_aurora_sdk.exceptions import AuroraSDKError
    except ImportError as exc:
        raise AuroraNotAvailable(
            "No se encontro el SDK de Slamtec Aurora en el entorno Python activo. "
            "Instalalo siguiendo la seccion del sensor en el README."
        ) from exc
    return AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, DEPTHCAM_FRAME_TYPE_POINT3D, AuroraSDKError


@dataclass
class AuroraConnection:
    sdk: object
    depth_enabled: bool = False
    sdk_lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    pose_listener: object | None = field(default=None, repr=False)


def _make_pose_listener():
    """Create an SDK listener that buffers high-rate timestamped poses when supported."""
    try:
        from slamtec_aurora_sdk import SDKListener
    except ImportError:
        return None

    class PoseListener(SDKListener):
        def __init__(self):
            super().__init__()
            self.samples: deque[SensorPose] = deque(maxlen=4000)
            self.samples_lock = threading.Lock()

        def on_pose_augmentation_result(self, timestamp_ns, _mode, pose):
            if pose is None:
                return
            try:
                position = np.asarray(pose.translation.to_tuple(), dtype=np.float64)
                quaternion = np.asarray(pose.quaternion.to_tuple(), dtype=np.float64)
                if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
                    return
                rpy_deg = np.degrees(Rotation.from_quat(quaternion).as_euler("xyz"))
                sample = SensorPose(position, rpy_deg, int(timestamp_ns))
                with self.samples_lock:
                    self.samples.append(sample)
            except Exception:
                # SDK callbacks must stay lightweight and never break the session.
                return

    try:
        return PoseListener()
    except Exception:
        return None


def connect(connection_string: str = "192.168.11.1") -> AuroraConnection:
    AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, _, AuroraSDKError = _import_sdk()

    pose_listener = _make_pose_listener()
    if pose_listener is not None:
        try:
            sdk = AuroraSDK(listener=pose_listener)
        except TypeError:
            # Older SDK packages may not expose the 2.1 listener constructor.
            pose_listener = None
            sdk = AuroraSDK()
    else:
        sdk = AuroraSDK()
    try:
        sdk.connect(connection_string=connection_string)
    except AuroraSDKError as exc:
        raise RuntimeError(f"No se pudo conectar al sensor Aurora en '{connection_string}': {exc}") from exc
    except Exception as exc:
        raise RuntimeError(f"No se pudo conectar al sensor Aurora en '{connection_string}': {exc}") from exc

    if not sdk.enhanced_imaging.is_depth_camera_supported():
        sdk.disconnect()
        sdk.release()
        raise RuntimeError("Este dispositivo Aurora no soporta camara de profundidad (depth camera).")

    enabled = sdk.controller.set_enhanced_imaging_subscription(ENHANCED_IMAGE_TYPE_DEPTH, True)
    if not enabled:
        sdk.disconnect()
        sdk.release()
        raise RuntimeError("No se pudo activar la suscripcion a la camara de profundidad del sensor.")

    time.sleep(2.0)  # el stream tarda en arrancar, igual que en el demo oficial
    return AuroraConnection(
        sdk=sdk,
        depth_enabled=True,
        pose_listener=pose_listener,
    )


def disconnect(connection: AuroraConnection) -> None:
    _, ENHANCED_IMAGE_TYPE_DEPTH, _, _ = _import_sdk()
    with connection.sdk_lock:
        try:
            if connection.depth_enabled:
                connection.sdk.controller.set_enhanced_imaging_subscription(ENHANCED_IMAGE_TYPE_DEPTH, False)
        finally:
            connection.sdk.disconnect()
            connection.sdk.release()


@dataclass
class SensorPose:
    position: np.ndarray  # (x, y, z) en metros, coordenadas del mapa SLAM
    rpy_deg: np.ndarray  # (roll, pitch, yaw) en grados
    timestamp_ns: int


def get_current_pose(connection: AuroraConnection) -> SensorPose:
    """Pose actual del sensor (posicion + orientacion) segun el tracking
    visual-inercial (SLAM+IMU) del Aurora, expresada en coordenadas del mapa.
    Su marco local (X adelante, Y izquierda, Z arriba) difiere del marco de la
    nube de profundidad (X derecha, Y abajo, Z adelante); las transformaciones
    entre capturas deben convertir ambos marcos. Se usa para guiar al usuario
    a devolver el sensor a la posicion donde capturo la nube base."""
    try:
        with connection.sdk_lock:
            position, rpy, timestamp_ns = connection.sdk.get_current_pose(use_se3=False)
    except Exception as exc:
        raise RuntimeError(f"No se pudo obtener la posicion actual del sensor: {exc}") from exc
    return SensorPose(
        position=np.array(position, dtype=np.float64),
        rpy_deg=np.degrees(np.array(rpy, dtype=np.float64)),
        timestamp_ns=int(timestamp_ns),
    )


def _interpolate_pose(before: SensorPose, after: SensorPose, timestamp_ns: int) -> SensorPose | None:
    """Interpolate SLAM pose at a depth frame timestamp; reject unbracketed data."""
    span = int(after.timestamp_ns) - int(before.timestamp_ns)
    if span <= 0 or span > 250_000_000:
        return None
    if timestamp_ns < before.timestamp_ns or timestamp_ns > after.timestamp_ns:
        return None
    fraction = (timestamp_ns - before.timestamp_ns) / span
    rotations = Rotation.from_euler("xyz", np.radians([before.rpy_deg, after.rpy_deg]))
    orientation = Slerp([0.0, 1.0], rotations)([fraction])[0]
    rpy = np.degrees(orientation.as_euler("xyz"))
    position = before.position + fraction * (after.position - before.position)
    return SensorPose(position=position, rpy_deg=rpy, timestamp_ns=int(timestamp_ns))


def _listener_pose_at_timestamp(
    connection: AuroraConnection, timestamp_ns: int
) -> SensorPose | None:
    listener = connection.pose_listener
    if listener is None or not hasattr(listener, "samples_lock"):
        return None
    with listener.samples_lock:
        samples = tuple(listener.samples)
    before = [sample for sample in samples if sample.timestamp_ns <= timestamp_ns]
    after = [sample for sample in samples if sample.timestamp_ns >= timestamp_ns]
    if not before or not after:
        return None
    pose_before = max(before, key=lambda sample: sample.timestamp_ns)
    pose_after = min(after, key=lambda sample: sample.timestamp_ns)
    if pose_before.timestamp_ns == pose_after.timestamp_ns:
        return SensorPose(
            pose_before.position.copy(), pose_before.rpy_deg.copy(), int(timestamp_ns)
        )
    return _interpolate_pose(pose_before, pose_after, timestamp_ns)


def _base_to_camera_calibration(
    connection: AuroraConnection,
) -> tuple[np.ndarray, np.ndarray, bool]:
    """Return SDK base-to-camera extrinsics, falling back to the known axis map."""
    fallback_rotation = np.array(
        [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    fallback_translation = np.zeros(3, dtype=np.float64)
    try:
        transform = connection.sdk.data_provider.get_transform_calibration().t_base_cam
        rotation = Rotation.from_quat(
            np.asarray(transform.quaternion.to_tuple(), dtype=np.float64)
        ).as_matrix()
        translation = np.asarray(transform.translation.to_tuple(), dtype=np.float64)
        if (
            rotation.shape == (3, 3)
            and translation.shape == (3,)
            and np.isfinite(rotation).all()
            and np.isfinite(translation).all()
        ):
            return rotation, translation, True
    except Exception:
        pass
    return fallback_rotation, fallback_translation, False


def _refine_frame_registration(
    frame_cloud: o3d.geometry.PointCloud,
    target_cloud: o3d.geometry.PointCloud,
) -> tuple[o3d.geometry.PointCloud, bool]:
    """Refine SLAM placement with ICP, rejecting frames without reliable overlap."""
    registration = o3d.pipelines.registration
    source = frame_cloud.voxel_down_sample(0.008)
    target = target_cloud.voxel_down_sample(0.008)
    if len(source.points) < 30 or len(target.points) < 100:
        return frame_cloud, False
    result = registration.registration_icp(
        source,
        target,
        0.015,
        np.eye(4),
        registration.TransformationEstimationPointToPoint(),
        registration.ICPConvergenceCriteria(max_iteration=20),
    )
    correction = np.asarray(result.transformation)
    # Open3D exposes its result matrix as a read-only NumPy view; SciPy's
    # Rotation constructor needs a writable buffer on some supported builds.
    angle = Rotation.from_matrix(correction[:3, :3].copy()).magnitude()
    shift = float(np.linalg.norm(correction[:3, 3]))
    # Low overlap means the frame cannot be checked against the accumulated
    # surface. Do not fuse it: that is how floating duplicate surfaces appear.
    if result.inlier_rmse > 0.010 or shift > 0.020 or angle > np.radians(3.0):
        return frame_cloud, False
    if result.fitness < 0.20:
        return frame_cloud, False
    frame_cloud.transform(correction.copy())
    return frame_cloud, True


def _fuse_static_frames(
    frame_grids: list[np.ndarray],
    color_grids: list[np.ndarray],
    fallback_grid: np.ndarray,
    fallback_colors: np.ndarray,
    max_points: int | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Use stable real samples while preserving coverage from the best native frame."""
    if len(frame_grids) < 3:
        raise ValueError("Se necesitan al menos 3 frames para acumular una captura fija.")
    stack = np.stack(frame_grids).astype(np.float32, copy=False)
    if stack.ndim != 4 or stack.shape[-1] != 3:
        raise ValueError("La acumulación fija requiere frames organizados HxWx3.")

    valid = np.isfinite(stack).all(axis=3)
    valid_count = valid.sum(axis=0)
    depths = np.linalg.norm(np.where(valid[..., None], stack, 0.0), axis=3)
    ordered_depths = np.where(valid, depths, np.inf)
    ordered_depths.sort(axis=0)
    median_indices = np.maximum(valid_count - 1, 0) // 2
    gather = np.broadcast_to(median_indices[None, ...], (1, *median_indices.shape))
    median_depth = np.take_along_axis(ordered_depths, gather, axis=0)[0]

    tolerance_m = np.maximum(0.008, median_depth * 0.002)
    stable = valid & (np.abs(depths - median_depth[None, ...]) <= tolerance_m[None, ...])
    required = max(3, int(np.ceil(0.60 * len(frame_grids))))
    stable_keep = (valid_count >= required) & (stable.sum(axis=0) >= required)

    error = np.where(stable, np.abs(depths - median_depth[None, ...]), np.inf)
    selected_index = np.argmin(error, axis=0)
    point_indices = np.broadcast_to(selected_index[None, ..., None], (1, *selected_index.shape, 3))
    selected_points = np.take_along_axis(stack, point_indices, axis=0)[0]
    color_stack = np.stack(color_grids)
    color_indices = np.broadcast_to(
        selected_index[None, ..., None], (1, *selected_index.shape, 3)
    )
    selected_colors = np.take_along_axis(color_stack, color_indices, axis=0)[0]
    fallback_valid = np.isfinite(fallback_grid).all(axis=2)
    use_fallback = ~stable_keep & fallback_valid
    selected_points[use_fallback] = fallback_grid[use_fallback]
    selected_colors[use_fallback] = fallback_colors[use_fallback]
    keep = stable_keep | fallback_valid
    selected_points = selected_points[keep]
    selected_colors = selected_colors[keep].astype(np.float32) / 255.0
    if len(selected_points) < 3:
        raise RuntimeError("La acumulación fija no reunió suficientes puntos estables.")
    if max_points is not None and max_points > 0 and len(selected_points) > max_points:
        indices = np.random.choice(len(selected_points), max_points, replace=False)
        selected_points = selected_points[indices]
        selected_colors = selected_colors[indices]
    return selected_points, selected_colors


def read_frame_points(
    connection: AuroraConnection,
    max_points: int | None = None,
    timeout_ms: int = 200,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
) -> np.ndarray | None:
    """
    Espera y devuelve un frame de puntos 3D (Nx3) del sensor, filtrando puntos
    invalidos (ceros, NaN/inf) y aplicando solo los recortes solicitados. Devuelve
    None si no hubo un frame disponible dentro del timeout.

    'cone_angle_deg' (opcional) descarta puntos fuera de un cono de ese
    angulo total, centrado en 'forward_axis' (x/y/z) — util para ignorar
    reflejos o superficies fuera del area de interes frente al sensor.
    """
    _, _, DEPTHCAM_FRAME_TYPE_POINT3D, _ = _import_sdk()
    sdk = connection.sdk
    with connection.sdk_lock:
        if not sdk.enhanced_imaging.wait_depth_camera_next_frame(timeout_ms):
            return None

        frame = sdk.enhanced_imaging.peek_depth_camera_frame(DEPTHCAM_FRAME_TYPE_POINT3D)
        if frame is None or not frame.data:
            return None

        points = frame.to_point3d_array()
        if points is None or len(points) == 0:
            return None
        points = np.asarray(points).copy()

    valid = ~np.all(points == 0, axis=1)
    valid &= np.all(np.isfinite(points), axis=1)
    distance = np.linalg.norm(points, axis=1)
    if max_distance_m is not None and max_distance_m > 0:
        valid &= distance < max_distance_m

    if cone_angle_deg is not None:
        if cone_angle_deg <= 0 or cone_angle_deg > 180:
            raise ValueError("El angulo de cono debe estar en el rango (0, 180].")
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_idx = axis_map.get(forward_axis.lower())
        if axis_idx is None:
            raise ValueError("El eje frontal debe ser x, y o z.")
        # Invalid SDK returns can contain NaN distance values. Keep those
        # points out through `valid` without emitting divide warnings while
        # calculating the optional field-of-view filter.
        safe_distance = np.where(np.isfinite(distance) & (distance > 0), distance, 1.0)
        cos_theta = np.clip(points[:, axis_idx] / safe_distance, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_theta))
        valid &= angle_deg <= (cone_angle_deg / 2.0)

    points = points[valid]

    if len(points) == 0:
        return None
    if max_points is not None and max_points > 0 and len(points) > max_points:
        idx = np.random.choice(len(points), max_points, replace=False)
        points = points[idx]

    return points


def _build_strided_uint8_image(data, width, height, channels, stride=0):
    """Arma una vista NumPy de una imagen respetando el stride (igual que en
    el demo oficial 'examples/dense_point_cloud.py' del SDK)."""
    if data is None:
        return None
    packed_row_bytes = width * channels
    row_stride = stride or packed_row_bytes
    required_size = packed_row_bytes + row_stride * (height - 1 if height > 1 else 0)
    if len(data) < required_size or row_stride < packed_row_bytes:
        return None
    shape = (height, width) if channels == 1 else (height, width, channels)
    strides = (row_stride, 1) if channels == 1 else (row_stride, channels, 1)
    try:
        return np.ndarray(shape=shape, dtype=np.uint8, buffer=data, strides=strides)
    except (TypeError, ValueError, BufferError):
        return None


def _extract_camera_rgb_image(camera_image) -> np.ndarray | None:
    """Convierte la imagen de camara alineada al frame de profundidad en
    colores RGB normalizados (0-1), igual criterio que el demo oficial."""
    if camera_image is None or not getattr(camera_image, "data", None):
        return None

    if hasattr(camera_image, "to_numpy_image"):
        img_rgb = camera_image.to_numpy_image(color_order="rgb")
        if img_rgb is not None:
            return img_rgb.astype(np.float32) / 255.0

    width = camera_image.width
    height = camera_image.height
    pixel_format = camera_image.pixel_format
    stride = getattr(camera_image, "stride", 0)
    data = camera_image.data

    if pixel_format == 0:
        gray = _build_strided_uint8_image(data, width, height, 1, stride)
        if gray is None:
            return None
        normalized = gray.astype(np.float32) / 255.0
        return np.repeat(normalized[:, :, np.newaxis], 3, axis=2)
    if pixel_format == 1:
        bgr = _build_strided_uint8_image(data, width, height, 3, stride)
        if bgr is None:
            return None
        return bgr[:, :, ::-1].astype(np.float32) / 255.0
    if pixel_format == 2:
        rgba = _build_strided_uint8_image(data, width, height, 4, stride)
        if rgba is None:
            return None
        return rgba[:, :, :3].astype(np.float32) / 255.0
    return None


def _neutral_gray_colors(points_xyz: np.ndarray) -> np.ndarray:
    """Use the GUI's neutral gray when no aligned camera RGB is available."""
    return np.full((len(points_xyz), 3), 0.72, dtype=np.float32)


def _colorize_from_camera(
    points_xyz: np.ndarray,
    valid_indices: np.ndarray,
    width: int,
    height: int,
    is_organized: bool,
    camera_rgb: np.ndarray | None,
) -> np.ndarray:
    img_rgb = camera_rgb
    if img_rgb is None or not is_organized:
        return _neutral_gray_colors(points_xyz)

    img_height, img_width = img_rgb.shape[0], img_rgb.shape[1]
    rows = valid_indices // width
    cols = valid_indices % width
    if width != img_width or height != img_height:
        cols = (cols.astype(np.float64) * img_width / width).astype(np.int64)
        rows = (rows.astype(np.float64) * img_height / height).astype(np.int64)
    rows = np.clip(rows, 0, img_height - 1)
    cols = np.clip(cols, 0, img_width - 1)
    colors = img_rgb[rows, cols]

    if not colors.size:
        return _neutral_gray_colors(points_xyz)
    # Aurora often returns a dark, monochrome infrared image. Brightness is
    # not a reliable way to decide whether the RGB frame is useful: rejecting
    # a dark frame loses its native grayscale contrast and paints every point
    # the same gray. Fall back only when the sampled image has almost no
    # contrast at all (for example, an all-black unavailable image).
    luminance = colors @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    if float(np.percentile(luminance, 99) - np.percentile(luminance, 1)) < 0.02:
        return _neutral_gray_colors(points_xyz)
    return colors


def read_frame_points_and_colors(
    connection: AuroraConnection,
    max_points: int | None = None,
    timeout_ms: int = 200,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
    preserve_organization: bool = False,
    include_timestamp: bool = False,
):
    """
    Igual que 'read_frame_points', pero ademas devuelve el color real de cada
    punto tomado de la imagen de camara alineada al frame de profundidad
    (mismo enfoque que el demo oficial 'examples/dense_point_cloud.py' del
    SDK). Si no hay imagen RGB correlacionada, usa gris neutro para que la
    captura no parezca una medicion coloreada por altura. Con
    'preserve_organization=True', una nube organizada se devuelve como grids
    HxWx3; los puntos invalidos llevan NaN. No mezcla frames ni modifica
    las coordenadas validas recibidas del SDK.
    """
    _, _, DEPTHCAM_FRAME_TYPE_POINT3D, _ = _import_sdk()
    sdk = connection.sdk
    frame_timestamp_ns = 0
    with connection.sdk_lock:
        if not sdk.enhanced_imaging.wait_depth_camera_next_frame(timeout_ms):
            return (None, None, 0) if include_timestamp else (None, None)

        frame = sdk.enhanced_imaging.peek_depth_camera_frame(DEPTHCAM_FRAME_TYPE_POINT3D)
        if frame is None or not frame.data:
            return (None, None, 0) if include_timestamp else (None, None)

        points = frame.to_point3d_array()
        if points is None or len(points) == 0:
            return (None, None, 0) if include_timestamp else (None, None)
        points = np.asarray(points).copy()
        frame_timestamp_ns = int(getattr(frame, "timestamp_ns", 0) or 0)
        width = frame.width
        height = frame.height
        is_organized = len(points) == width * height
        camera_rgb = None
        if hasattr(frame, "timestamp_ns") and frame.timestamp_ns > 0:
            try:
                camera_image = sdk.enhanced_imaging.peek_depth_camera_related_rectified_image(
                    frame.timestamp_ns
                )
                camera_rgb = _extract_camera_rgb_image(camera_image)
            except Exception:
                camera_rgb = None

    valid = ~np.all(points == 0, axis=1)
    valid &= np.all(np.isfinite(points), axis=1)
    distance = np.linalg.norm(points, axis=1)
    if max_distance_m is not None and max_distance_m > 0:
        valid &= distance < max_distance_m

    if cone_angle_deg is not None:
        if cone_angle_deg <= 0 or cone_angle_deg > 180:
            raise ValueError("El angulo de cono debe estar en el rango (0, 180].")
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_idx = axis_map.get(forward_axis.lower())
        if axis_idx is None:
            raise ValueError("El eje frontal debe ser x, y o z.")
        safe_distance = np.where(np.isfinite(distance) & (distance > 0), distance, 1.0)
        cos_theta = np.clip(points[:, axis_idx] / safe_distance, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_theta))
        valid &= angle_deg <= (cone_angle_deg / 2.0)

    original_indices = np.arange(len(points))
    valid_indices = original_indices[valid]
    if not np.any(valid):
        if preserve_organization and is_organized:
            empty_points = np.full((height, width, 3), np.nan, dtype=np.float32)
            empty_colors = np.zeros((height, width, 3), dtype=np.float32)
            return (empty_points, empty_colors, frame_timestamp_ns) if include_timestamp else (empty_points, empty_colors)
        return (None, None, frame_timestamp_ns) if include_timestamp else (None, None)

    valid_points = points[valid]
    colors = _colorize_from_camera(valid_points, valid_indices, width, height, is_organized, camera_rgb)

    if preserve_organization and is_organized:
        organized_points = np.full((len(points), 3), np.nan, dtype=np.float32)
        organized_colors = np.zeros((len(points), 3), dtype=np.float32)
        organized_points[valid] = valid_points
        organized_colors[valid] = colors
        result = (organized_points.reshape(height, width, 3), organized_colors.reshape(height, width, 3))
        return (*result, frame_timestamp_ns) if include_timestamp else result

    if max_points is not None and max_points > 0 and len(valid_points) > max_points:
        idx = np.random.choice(len(valid_points), max_points, replace=False)
        valid_points = valid_points[idx]
        colors = colors[idx]

    return (valid_points, colors, frame_timestamp_ns) if include_timestamp else (valid_points, colors)


def capture_aligned_snapshot(
    connection: AuroraConnection,
    duration_s: float = 3.0,
    timeout_s: float | None = None,
    min_frames: int = 8,
    voxel_size: float = 0.003,
    stop_event: threading.Event | None = None,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
) -> AlignedCaptureResult:
    """Capture aligned frames using the SDK's high-rate IMU/vision pose when available."""
    augmentation_started = False
    try:
        from slamtec_aurora_sdk import (
            POSE_AUGMENTATION_MODE_IMU_VISION_MIXED,
            POSE_OUTPUT_FREQ_200HZ,
            PoseAugmentationConfig,
        )

        config = PoseAugmentationConfig()
        config.output_frequency = POSE_OUTPUT_FREQ_200HZ
        config.enable_smoothing = 0
        connection.sdk.data_provider.start_pose_augmentation(
            POSE_AUGMENTATION_MODE_IMU_VISION_MIXED,
            config,
        )
        augmentation_started = True
    except Exception:
        # Keep support for SDK/firmware versions without pose augmentation.
        pass
    try:
        result = _capture_aligned_snapshot_impl(
            connection,
            duration_s=duration_s,
            timeout_s=timeout_s,
            min_frames=min_frames,
            voxel_size=voxel_size,
            stop_event=stop_event,
            max_distance_m=max_distance_m,
            cone_angle_deg=cone_angle_deg,
            forward_axis=forward_axis,
        )
        result.pose_augmentation_used = augmentation_started
        return result
    finally:
        if augmentation_started:
            try:
                connection.sdk.data_provider.stop_pose_augmentation()
            except Exception:
                pass


def _capture_aligned_snapshot_impl(
    connection: AuroraConnection,
    duration_s: float = 3.0,
    timeout_s: float | None = None,
    min_frames: int = 8,
    voxel_size: float = 0.003,
    stop_event: threading.Event | None = None,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
) -> AlignedCaptureResult:
    """Capture a short moving scan, register each depth frame by Aurora pose,
    and fuse it in the first accepted frame's camera coordinate system."""
    if not np.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("El tiempo de captura debe ser mayor que cero.")
    if min_frames <= 0:
        raise ValueError("Se requiere al menos un frame válido.")
    if not np.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError("El tamaño de voxel de fusión debe ser mayor que cero.")
    timeout_s = float(timeout_s if timeout_s is not None else max(10.0, duration_s + 5.0))
    start = time.monotonic()
    reference_pose = None
    previous_pose = None
    fused = o3d.geometry.PointCloud()
    accepted = rejected = 0
    path_length = 0.0
    final_translation = final_rotation = 0.0
    (
        pose_to_camera,
        base_to_camera_translation,
        extrinsic_calibration_used,
    ) = _base_to_camera_calibration(connection)

    while time.monotonic() - start < timeout_s:
        if stop_event is not None and stop_event.is_set():
            break
        try:
            pose_before = get_current_pose(connection)
        except RuntimeError:
            rejected += 1
            continue
        points_grid, colors_grid, frame_ts = read_frame_points_and_colors(
            connection,
            timeout_ms=300,
            max_distance_m=max_distance_m,
            cone_angle_deg=cone_angle_deg,
            forward_axis=forward_axis,
            preserve_organization=True,
            include_timestamp=True,
        )
        if points_grid is None or not frame_ts:
            rejected += 1
            continue
        try:
            pose_after = get_current_pose(connection)
        except RuntimeError:
            rejected += 1
            continue
        # Pair the depth frame with a pose interpolated at its own device time.
        # A stale latest-pose sample can smear walls during a moving scan.
        pose = _listener_pose_at_timestamp(connection, int(frame_ts))
        if pose is None:
            pose = _interpolate_pose(pose_before, pose_after, int(frame_ts))
        if pose is None:
            rejected += 1
            continue
        if points_grid.ndim == 3:
            valid = np.isfinite(points_grid).all(axis=2)
            points = points_grid[valid].astype(np.float64, copy=False)
            colors = colors_grid[valid].astype(np.float64, copy=False)
        else:
            valid = np.isfinite(points_grid).all(axis=1)
            points = points_grid[valid].astype(np.float64, copy=False)
            colors = colors_grid[valid].astype(np.float64, copy=False)
        if len(points) < 3:
            rejected += 1
            continue

        if reference_pose is None:
            reference_pose = pose
        if previous_pose is not None:
            step_m = float(np.linalg.norm(pose.position - previous_pose.position))
            step_rpy = ((pose.rpy_deg - previous_pose.rpy_deg + 180.0) % 360.0) - 180.0
            step_deg = float(np.max(np.abs(step_rpy)))
            # Large discontinuities usually mean relocalization/tracking loss.
            if step_m > 0.12 or step_deg > 12.0:
                rejected += 1
                continue
            path_length += step_m
        base_r = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(reference_pose.rpy_deg))
        frame_r = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(pose.rpy_deg))
        relative_r = pose_to_camera @ base_r.T @ frame_r @ pose_to_camera.T
        relative_t = (
            base_to_camera_translation
            + pose_to_camera @ base_r.T @ (pose.position - reference_pose.position)
            - relative_r @ base_to_camera_translation
        )
        points = points @ relative_r.T + relative_t

        frame_cloud = o3d.geometry.PointCloud()
        frame_cloud.points = o3d.utility.Vector3dVector(points)
        frame_cloud.colors = o3d.utility.Vector3dVector(colors)
        if accepted == 0:
            fused = frame_cloud
        else:
            frame_cloud, registration_ok = _refine_frame_registration(frame_cloud, fused)
            if not registration_ok:
                rejected += 1
                continue
            fused += frame_cloud
        fused = fused.voxel_down_sample(voxel_size)
        accepted += 1
        previous_pose = pose
        final_translation = float(np.linalg.norm(pose.position - reference_pose.position))
        relative_rpy = ((pose.rpy_deg - reference_pose.rpy_deg + 180.0) % 360.0) - 180.0
        final_rotation = float(np.linalg.norm(relative_rpy))
        if accepted >= min_frames and time.monotonic() - start >= duration_s:
            break

    if accepted < min_frames or time.monotonic() - start < duration_s and not (stop_event and stop_event.is_set()):
        raise RuntimeError(
            f"La captura alineada reunió {accepted} frames válidos en "
            f"{time.monotonic() - start:.1f} de {duration_s:.1f} s; se requieren "
            f"{min_frames} frames y completar el tiempo configurado. Mantén el tracking "
            "activo, mueve el sensor lentamente y repite."
        )
    if reference_pose is None or not len(fused.points):
        raise RuntimeError("No se pudo registrar ningún frame de profundidad con la pose Aurora.")
    return AlignedCaptureResult(
        cloud=fused,
        reference_pose=reference_pose,
        accepted_frames=accepted,
        rejected_frames=rejected,
        path_length_m=path_length,
        final_translation_m=final_translation,
        final_rotation_deg=final_rotation,
        extrinsic_calibration_used=extrinsic_calibration_used,
    )


def capture_reference_frame(
    connection: AuroraConnection,
    timeout_ms: int = 300,
    max_distance_m: float | None = None,
) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    """
    Captura un unico frame "de referencia": la foto de camara tal cual la ve
    el sensor, mas un grid 3D (alto x ancho x 3) con el punto correspondiente
    a cada pixel del frame de profundidad (NaN donde no hay profundidad
    valida). Sirve para elegir puntos de referencia sobre la foto en vez de
    sobre la nube de puntos cruda, para el caso en que un punto (ej. la
    punta de un perno) se ve claro en la foto pero el sensor de profundidad
    no llega a resolverlo como puntos 3D limpios (objeto muy fino).

    Devuelve (None, None) si no hay frame organizado o no hay imagen de
    camara disponible (best-effort: no debe romper la captura principal).
    """
    _, _, DEPTHCAM_FRAME_TYPE_POINT3D, _ = _import_sdk()
    sdk = connection.sdk
    with connection.sdk_lock:
        if not sdk.enhanced_imaging.wait_depth_camera_next_frame(timeout_ms):
            return None, None

        frame = sdk.enhanced_imaging.peek_depth_camera_frame(DEPTHCAM_FRAME_TYPE_POINT3D)
        if frame is None or not frame.data:
            return None, None

        points = frame.to_point3d_array()
        if points is None or len(points) == 0:
            return None, None
        points = np.asarray(points).copy()
        width, height = frame.width, frame.height
        if len(points) != width * height:
            return None, None  # necesitamos un grid organizado para mapear pixel -> punto

        camera_image = None
        if hasattr(frame, "timestamp_ns") and frame.timestamp_ns > 0:
            try:
                camera_image = sdk.enhanced_imaging.peek_depth_camera_related_rectified_image(
                    frame.timestamp_ns
                )
            except Exception:
                camera_image = None

        img_rgb = _extract_camera_rgb_image(camera_image) if camera_image is not None else None
    if img_rgb is None:
        return None, None

    point_grid = points.reshape(height, width, 3).astype(np.float32)
    valid = ~np.all(point_grid == 0, axis=2)
    valid &= np.all(np.isfinite(point_grid), axis=2)
    distance = np.linalg.norm(point_grid, axis=2)
    if max_distance_m is not None and max_distance_m > 0:
        valid &= distance < max_distance_m
    point_grid[~valid] = np.nan

    image_uint8 = np.clip(img_rgb * 255.0, 0, 255).astype(np.uint8)
    return image_uint8, point_grid


def _voxel_keys(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Codifica cada punto en un entero unico que identifica su celda de voxel,
    para poder comparar presencia de celdas entre frames de forma vectorizada."""
    idx = np.floor(points / voxel_size).astype(np.int64)
    offset = 1 << 20  # soporta +-1.048.576 celdas por eje (~5 km a 5mm/voxel)
    shifted = idx + offset
    return (shifted[:, 0] << 42) | (shifted[:, 1] << 21) | shifted[:, 2]


def capture_snapshot(
    connection: AuroraConnection,
    max_points_per_frame: int | None = None,
    voxel_size: float = 0.005,
    frame_timeout_ms: int = 300,
    stabilization_frames: int = 15,
    capture_duration_s: float = 0.0,
    timeout_s: float = 10.0,
    persistence_ratio: float = 0.0,
    stop_event: threading.Event | None = None,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
    accumulate_static: bool = False,
) -> o3d.geometry.PointCloud:
    """
    Adquiere frames durante el tiempo configurado y conserva el frame nativo
    con mayor cobertura, salvo que se solicite acumulacion fija: en ese modo
    conserva la cobertura del mejor frame nativo y usa mediciones estables reales para
    mejorar o completar pixeles, sin promediar coordenadas.
    No transforma por pose/SLAM ni aplica voxel downsample por defecto.
    Con capture_duration_s=0 y stabilization_frames=1 devuelve el primer
    frame valido, sin espera temporal ni acumulacion.

    Si se pasa 'stop_event' y se activa, guarda el mejor frame recibido hasta
    ese momento. 'timeout_s' evita esperar indefinidamente si no llegan frames.

    'cone_angle_deg' (opcional) limita cada frame a un cono de ese angulo
    total centrado en 'forward_axis', para excluir lo que esta fuera del
    campo de vision deseado (por ejemplo, todo lo que queda detras de una
    caja de prueba). 'max_distance_m' (opcional) descarta puntos mas lejos
    que ese valor. Sin ninguno de los dos, se captura todo el campo de
    vision del sensor.
    """
    if not np.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError("El tiempo limite debe ser mayor que cero.")
    if stabilization_frames <= 0:
        raise ValueError("La cantidad de frames de estabilizacion debe ser mayor que cero.")
    if not np.isfinite(capture_duration_s) or capture_duration_s < 0:
        raise ValueError("El tiempo de captura debe ser cero o mayor.")
    if accumulate_static and capture_duration_s == 0:
        raise ValueError("La acumulacion requiere un tiempo de captura mayor que cero.")
    if not np.isfinite(persistence_ratio) or not 0.0 <= persistence_ratio <= 1.0:
        raise ValueError("La persistencia debe estar entre 0 y 1.")
    if max_points_per_frame is not None and max_points_per_frame <= 0:
        raise ValueError("max_points_per_frame debe ser mayor que cero o None para conservar todos los puntos.")
    if not np.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError("voxel_size debe ser cero o mayor.")
    if persistence_ratio > 0 and voxel_size <= 0:
        raise ValueError("La persistencia requiere un voxel_size mayor que cero.")

    collected_points: list[np.ndarray] = []
    static_frame_grids: list[np.ndarray] = []
    static_color_grids: list[np.ndarray] = []
    static_frames_seen = 0
    static_frame_limit = 0
    latest_points: np.ndarray | None = None
    latest_colors: np.ndarray | None = None
    latest_grid: np.ndarray | None = None
    latest_color_grid: np.ndarray | None = None
    best_point_count = -1
    ready = False
    valid_frames_received = 0
    start = time.monotonic()

    while time.monotonic() - start < timeout_s:
        if stop_event is not None and stop_event.is_set():
            break
        points, colors = read_frame_points_and_colors(
            connection,
            max_points=max_points_per_frame,
            timeout_ms=frame_timeout_ms,
            max_distance_m=max_distance_m,
            cone_angle_deg=cone_angle_deg,
            forward_axis=forward_axis,
            preserve_organization=accumulate_static,
        )
        if points is None:
            continue
        organized_points = points.copy() if points.ndim == 3 else None
        organized_colors = (
            np.clip(colors * 255.0, 0, 255).astype(np.uint8)
            if colors.ndim == 3
            else None
        )
        if points.ndim == 3:
            if accumulate_static:
                if static_frame_limit == 0:
                    target_samples = min(
                        _STATIC_ACCUMULATION_MAX_FRAMES,
                        max(8, int(np.ceil(capture_duration_s * _STATIC_ACCUMULATION_SAMPLE_RATE_HZ))),
                    )
                    bytes_per_sample = points.nbytes + points.shape[0] * points.shape[1] * 3
                    memory_limited_samples = (
                        _STATIC_ACCUMULATION_MEMORY_BUDGET_BYTES // max(1, bytes_per_sample)
                    )
                    static_frame_limit = min(target_samples, int(memory_limited_samples))
                    if static_frame_limit < 3:
                        raise RuntimeError(
                            "La resolución del frame excede la memoria disponible para la "
                            "acumulación fija. Desactiva la acumulación para guardar un frame nativo."
                        )
                static_frames_seen += 1
                color_frame = np.clip(colors * 255.0, 0, 255).astype(np.uint8)
                if len(static_frame_grids) < static_frame_limit:
                    static_frame_grids.append(points.copy())
                    static_color_grids.append(color_frame)
                else:
                    slot = int(np.random.randint(0, static_frames_seen))
                    if slot < len(static_frame_grids):
                        static_frame_grids[slot] = points.copy()
                        static_color_grids[slot] = color_frame
            valid = np.isfinite(points).all(axis=2)
            points = points[valid]
            colors = colors[valid]
        else:
            valid = np.isfinite(points).all(axis=1)
            points = points[valid]
            colors = colors[valid]
        if not len(points):
            continue

        valid_frames_received += 1
        if persistence_ratio > 0:
            collected_points.append(points)
        # Keep one complete sensor frame. Selecting by valid coverage avoids
        # temporal blending while favoring frames with fewer missing returns.
        if len(points) >= best_point_count:
            if max_points_per_frame is not None and len(points) > max_points_per_frame:
                indices = np.random.choice(len(points), max_points_per_frame, replace=False)
                latest_points = points[indices].copy()
                latest_colors = colors[indices].copy()
            else:
                latest_points = points.copy()
                latest_colors = colors.copy()
            if organized_points is not None and organized_colors is not None:
                latest_grid = organized_points
                latest_color_grid = organized_colors
            best_point_count = len(points)
        ready = (
            valid_frames_received >= stabilization_frames
            and time.monotonic() - start >= capture_duration_s
        )
        if ready:
            break

    if latest_points is None or latest_colors is None:
        raise RuntimeError(
            f"No se pudieron recibir {stabilization_frames} frames validos "
            f"en {timeout_s:.0f} s. "
            "Verifica la conexion y que el sensor apunte a una superficie dentro de su "
            "rango de medicion."
        )
    if not ready and (stop_event is None or not stop_event.is_set()):
        raise RuntimeError(
            f"Solo se recibieron {valid_frames_received} de {stabilization_frames} "
            "frames necesarios "
            "antes del tiempo limite. Verifica la conexion con el sensor."
        )

    if accumulate_static and static_frames_seen == 0:
        raise RuntimeError(
            "El sensor no entregó frames organizados para acumular por píxel. "
            "Desactiva la acumulación fija y captura un frame nativo."
        )

    assert latest_points is not None and latest_colors is not None
    if (
        accumulate_static
        and len(static_frame_grids) >= 3
        and latest_grid is not None
        and latest_color_grid is not None
    ):
        latest_points, latest_colors = _fuse_static_frames(
            static_frame_grids,
            static_color_grids,
            latest_grid,
            latest_color_grid,
            max_points=max_points_per_frame,
        )
    if persistence_ratio > 0:
        num_frames = len(collected_points)
        frame_keys = [np.unique(_voxel_keys(frame, voxel_size)) for frame in collected_points]
        unique_keys, counts = np.unique(np.concatenate(frame_keys), return_counts=True)
        min_frames = max(1, int(round(persistence_ratio * num_frames)))
        persistent_keys = unique_keys[counts >= min_frames]
        latest_keys = _voxel_keys(latest_points, voxel_size)
        keep = np.isin(latest_keys, persistent_keys)
        latest_points = latest_points[keep]
        latest_colors = latest_colors[keep]
        if not len(latest_points):
            raise RuntimeError(
                "Ningun punto del frame elegido persistio lo suficiente durante la captura. "
                "Reduce la persistencia requerida o repite la captura."
            )

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(latest_points)
    cloud.colors = o3d.utility.Vector3dVector(np.clip(latest_colors, 0.0, 1.0))
    return cloud


def stream_frames(
    connection: AuroraConnection,
    callback: Callable[[np.ndarray], None],
    stop_event: threading.Event,
    max_points: int | None = None,
    rate_hz: float = 10.0,
) -> None:
    """
    Bucle bloqueante que entrega frames del sensor en vivo a
    'callback(points_xyz)' hasta que se active 'stop_event'. Pensado para
    correr en un hilo aparte (no llamar desde el hilo principal de la GUI).
    """
    interval = 1.0 / rate_hz
    last = 0.0
    while not stop_event.is_set():
        now = time.time()
        if now - last < interval:
            time.sleep(0.01)
            continue
        points = read_frame_points(connection, max_points=max_points, timeout_ms=100)
        if points is not None:
            callback(points)
        last = now
