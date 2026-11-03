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
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
import open3d as o3d


@dataclass
class AlignedCaptureResult:
    cloud: o3d.geometry.PointCloud
    reference_pose: "SensorPose"
    accepted_frames: int
    rejected_frames: int
    path_length_m: float
    final_translation_m: float
    final_rotation_deg: float


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


def connect(connection_string: str = "192.168.11.1") -> AuroraConnection:
    AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, _, AuroraSDKError = _import_sdk()

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
    return AuroraConnection(sdk=sdk, depth_enabled=True)


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
    invalidos (ceros, NaN/inf, fuera del rango 0.1m-max_distance_m). Devuelve
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
    max_range = max_distance_m if (max_distance_m is not None and max_distance_m > 0) else 50.0
    valid &= (distance > 0.1) & (distance < max_range)

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


class TemporalFrameMedianFilter:
    """Median-filter an organized Aurora frame over a short time window."""

    def __init__(self, window: int = 5):
        self.window = max(1, int(window))
        self._frames: deque[tuple[np.ndarray, np.ndarray]] = deque(maxlen=self.window)
        self._shape: tuple[int, int, int] | None = None

    def update(
        self,
        points_grid: np.ndarray,
        colors_grid: np.ndarray,
        max_points: int | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        points = np.asarray(points_grid, dtype=np.float32)
        colors = np.asarray(colors_grid, dtype=np.float32)
        if points.ndim != 3 or points.shape[-1] != 3 or colors.shape != points.shape:
            raise ValueError("La mediana temporal requiere grids de puntos y colores HxWx3 compatibles.")
        if self._shape != points.shape:
            self._frames.clear()
            self._shape = points.shape
        self._frames.append((points.copy(), colors.copy()))

        point_stack = np.stack([frame[0] for frame in self._frames])
        color_stack = np.stack([frame[1] for frame in self._frames])
        valid_stack = np.isfinite(point_stack).all(axis=3)
        valid_count = valid_stack.sum(axis=0)
        minimum_observations = (len(self._frames) + 1) // 2
        valid = valid_count >= minimum_observations
        # Sorting the short time axis is faster than nanmedian on the full grid.
        point_values = np.where(valid_stack[..., None], point_stack, np.inf)
        color_values = np.where(valid_stack[..., None], color_stack, np.inf)
        point_values.sort(axis=0)
        color_values.sort(axis=0)
        median_index = np.maximum(valid_count - 1, 0) // 2
        gather_index = np.broadcast_to(
            median_index[None, :, :, None], (1, *median_index.shape, 3)
        )
        filtered_points = np.take_along_axis(point_values, gather_index, axis=0)[0]
        filtered_colors = np.take_along_axis(color_values, gather_index, axis=0)[0]
        filtered_points[~valid] = np.nan
        filtered_colors[~valid] = 0.0

        indices = np.flatnonzero(valid)
        if max_points is not None and max_points > 0 and len(indices) > max_points:
            indices = indices[np.linspace(0, len(indices) - 1, max_points, dtype=np.int64)]
        flat_points = filtered_points.reshape((-1, 3))
        flat_colors = filtered_colors.reshape((-1, 3))
        return flat_points[indices].copy(), flat_colors[indices].copy()


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
    HxWx3; los puntos invalidos llevan NaN para poder estabilizar la lectura
    temporal por pixel en la vista en vivo.
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
    max_range = max_distance_m if (max_distance_m is not None and max_distance_m > 0) else 50.0
    valid &= (distance > 0.1) & (distance < max_range)

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
    min_frames: int = 15,
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
    pose_to_camera = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])

    while time.monotonic() - start < timeout_s:
        if stop_event is not None and stop_event.is_set():
            break
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
        pose = get_current_pose(connection)
        # Both timestamps come from the device. Reject poorly paired frame/pose
        # samples instead of fusing geometry with a potentially stale pose.
        if abs(int(pose.timestamp_ns) - frame_ts) > 150_000_000:
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
            if step_m > 0.5 or step_deg > 45.0:
                rejected += 1
                continue
            path_length += step_m
        base_r = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(reference_pose.rpy_deg))
        frame_r = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(pose.rpy_deg))
        relative_r = pose_to_camera @ base_r.T @ frame_r @ pose_to_camera.T
        relative_t = pose_to_camera @ base_r.T @ (pose.position - reference_pose.position)
        points = points @ relative_r.T + relative_t

        frame_cloud = o3d.geometry.PointCloud()
        frame_cloud.points = o3d.utility.Vector3dVector(points)
        frame_cloud.colors = o3d.utility.Vector3dVector(colors)
        if accepted == 0:
            fused = frame_cloud
        else:
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
    max_range = max_distance_m if (max_distance_m is not None and max_distance_m > 0) else 50.0
    valid &= (distance > 0.1) & (distance < max_range)
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
    capture_duration_s: float = 3.0,
    timeout_s: float = 10.0,
    persistence_ratio: float = 0.0,
    stop_event: threading.Event | None = None,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
) -> o3d.geometry.PointCloud:
    """
    Espera hasta tener 'stabilization_frames' frames validos, calcula la misma
    mediana temporal de cinco frames que la vista en vivo y devuelve solo el
    ultimo resultado. No combina frames ni aplica voxel downsample por defecto.
    Los puntos siguen en el marco local de camara, sin transformarlos por
    pose/SLAM. Si se pide persistencia, se usa la secuencia para filtrar el
    ultimo frame.

    Si se pasa 'stop_event' y se activa, guarda el ultimo frame estabilizado
    disponible. 'timeout_s' evita esperar indefinidamente si no llegan frames.

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
    if not np.isfinite(capture_duration_s) or capture_duration_s <= 0:
        raise ValueError("El tiempo de captura debe ser mayor que cero.")
    if not np.isfinite(persistence_ratio) or not 0.0 <= persistence_ratio <= 1.0:
        raise ValueError("La persistencia debe estar entre 0 y 1.")
    if max_points_per_frame is not None and max_points_per_frame <= 0:
        raise ValueError("max_points_per_frame debe ser mayor que cero o None para conservar todos los puntos.")
    if not np.isfinite(voxel_size) or voxel_size < 0:
        raise ValueError("voxel_size debe ser cero o mayor.")
    if persistence_ratio > 0 and voxel_size <= 0:
        raise ValueError("La persistencia requiere un voxel_size mayor que cero.")

    collected_points: list[np.ndarray] = []
    latest_points: np.ndarray | None = None
    latest_colors: np.ndarray | None = None
    ready = False
    valid_frames_received = 0
    temporal_filter = TemporalFrameMedianFilter(window=5)
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
            preserve_organization=True,
        )
        if points is None:
            continue
        if points.ndim == 3:
            points, colors = temporal_filter.update(
                points,
                colors,
                max_points=max_points_per_frame,
            )
        if not len(points):
            continue

        valid_frames_received += 1
        if persistence_ratio > 0:
            collected_points.append(points)
        latest_points = points
        latest_colors = colors
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

    assert latest_points is not None and latest_colors is not None
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
                "Ningun punto del ultimo frame persistio lo suficiente durante la captura. "
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
