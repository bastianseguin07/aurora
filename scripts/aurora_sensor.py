"""
Integracion con el sensor Slamtec Aurora (SLAMWARE-Aurora-XXXX) via su SDK
oficial (paquete 'slamtec_aurora_sdk', repo Slamtec/py_aurora_remote).

Este modulo NO se pudo probar contra hardware real (no hay un Aurora
conectado en el entorno de desarrollo). Las llamadas a la SDK siguen
exactamente el patron del ejemplo oficial 'examples/dense_point_cloud.py'
del repositorio. Antes de confiar en el flujo completo, probalo primero
con el sensor conectado.

Instalacion del SDK (no esta en PyPI, hay que compilarlo):
    git clone --recursive https://github.com/Slamtec/py_aurora_remote.git
    cd py_aurora_remote
    pip install -r requirements-dev.txt
    python tools/build_package.py --platforms win64
    pip install wheels/slamtec_aurora_python_sdk_win64-2.1.1-py3-none-any.whl
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np
import open3d as o3d


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
            "No se encontro el SDK de Slamtec Aurora (paquete 'slamtec_aurora_sdk'). "
            "Revisa la seccion de instalacion del sensor en el README."
        ) from exc
    return AuroraSDK, ENHANCED_IMAGE_TYPE_DEPTH, DEPTHCAM_FRAME_TYPE_POINT3D, AuroraSDKError


@dataclass
class AuroraConnection:
    sdk: object
    depth_enabled: bool = False


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
    try:
        if connection.depth_enabled:
            connection.sdk.controller.set_enhanced_imaging_subscription(ENHANCED_IMAGE_TYPE_DEPTH, False)
    finally:
        connection.sdk.disconnect()
        connection.sdk.release()


def read_frame_points(
    connection: AuroraConnection,
    max_points: int = 50000,
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

    if not sdk.enhanced_imaging.wait_depth_camera_next_frame(timeout_ms):
        return None

    frame = sdk.enhanced_imaging.peek_depth_camera_frame(DEPTHCAM_FRAME_TYPE_POINT3D)
    if frame is None or not frame.data:
        return None

    points = frame.to_point3d_array()
    if points is None or len(points) == 0:
        return None

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
        safe_distance = np.where(distance == 0, 1e-8, distance)
        cos_theta = np.clip(points[:, axis_idx] / safe_distance, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_theta))
        valid &= angle_deg <= (cone_angle_deg / 2.0)

    points = points[valid]

    if len(points) == 0:
        return None
    if len(points) > max_points:
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


def _height_based_colors(points_xyz: np.ndarray) -> np.ndarray:
    """Degrade por altura usado como respaldo cuando no hay imagen de camara
    disponible para colorear (mismo criterio que el demo oficial)."""
    if len(points_xyz) == 0:
        return np.zeros((0, 3))
    heights = points_xyz[:, 1]
    span = heights.max() - heights.min() + 1e-8
    height_normalized = (heights - heights.min()) / span
    colors = np.zeros((len(points_xyz), 3))
    colors[:, 0] = height_normalized * 0.8 + 0.1
    colors[:, 1] = (1 - height_normalized) * 0.8 + 0.1
    colors[:, 2] = 0.5
    return colors


def _colorize_from_camera(
    points_xyz: np.ndarray,
    valid_indices: np.ndarray,
    width: int,
    height: int,
    is_organized: bool,
    camera_image,
) -> np.ndarray:
    img_rgb = _extract_camera_rgb_image(camera_image) if camera_image is not None else None
    if img_rgb is None or not is_organized:
        return _height_based_colors(points_xyz)

    img_height, img_width = img_rgb.shape[0], img_rgb.shape[1]
    rows = valid_indices // width
    cols = valid_indices % width
    if width != img_width or height != img_height:
        cols = (cols.astype(np.float64) * img_width / width).astype(np.int64)
        rows = (rows.astype(np.float64) * img_height / height).astype(np.int64)
    rows = np.clip(rows, 0, img_height - 1)
    cols = np.clip(cols, 0, img_width - 1)
    colors = img_rgb[rows, cols]

    avg_brightness = colors.mean() if colors.size else 0.0
    if avg_brightness > 0.9 or avg_brightness < 0.1:
        return _height_based_colors(points_xyz)
    return colors


def read_frame_points_and_colors(
    connection: AuroraConnection,
    max_points: int = 50000,
    timeout_ms: int = 200,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    """
    Igual que 'read_frame_points', pero ademas devuelve el color real de cada
    punto tomado de la imagen de camara alineada al frame de profundidad
    (mismo enfoque que el demo oficial 'examples/dense_point_cloud.py' del
    SDK). Si no hay imagen de camara disponible, usa un degrade por altura
    como respaldo.
    """
    _, _, DEPTHCAM_FRAME_TYPE_POINT3D, _ = _import_sdk()
    sdk = connection.sdk

    if not sdk.enhanced_imaging.wait_depth_camera_next_frame(timeout_ms):
        return None, None

    frame = sdk.enhanced_imaging.peek_depth_camera_frame(DEPTHCAM_FRAME_TYPE_POINT3D)
    if frame is None or not frame.data:
        return None, None

    points = frame.to_point3d_array()
    if points is None or len(points) == 0:
        return None, None

    width = frame.width
    height = frame.height
    is_organized = len(points) == width * height

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
        safe_distance = np.where(distance == 0, 1e-8, distance)
        cos_theta = np.clip(points[:, axis_idx] / safe_distance, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_theta))
        valid &= angle_deg <= (cone_angle_deg / 2.0)

    original_indices = np.arange(len(points))
    valid_indices = original_indices[valid]
    points = points[valid]
    if len(points) == 0:
        return None, None

    camera_image = None
    if hasattr(frame, "timestamp_ns") and frame.timestamp_ns > 0:
        try:
            camera_image = sdk.enhanced_imaging.peek_depth_camera_related_rectified_image(frame.timestamp_ns)
        except Exception:
            camera_image = None

    colors = _colorize_from_camera(points, valid_indices, width, height, is_organized, camera_image)

    if len(points) > max_points:
        idx = np.random.choice(len(points), max_points, replace=False)
        points = points[idx]
        colors = colors[idx]

    return points, colors


def _voxel_keys(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Codifica cada punto en un entero unico que identifica su celda de voxel,
    para poder comparar presencia de celdas entre frames de forma vectorizada."""
    idx = np.floor(points / voxel_size).astype(np.int64)
    offset = 1 << 20  # soporta +-1.048.576 celdas por eje (~5 km a 5mm/voxel)
    shifted = idx + offset
    return (shifted[:, 0] << 42) | (shifted[:, 1] << 21) | shifted[:, 2]


def capture_snapshot(
    connection: AuroraConnection,
    duration_s: float = 15.0,
    max_points_per_frame: int = 100000,
    voxel_size: float = 0.005,
    frame_timeout_ms: int = 300,
    persistence_ratio: float = 0.6,
    stop_event: threading.Event | None = None,
    max_distance_m: float | None = None,
    cone_angle_deg: float | None = None,
    forward_axis: str = "z",
) -> o3d.geometry.PointCloud:
    """
    Acumula frames del sensor durante 'duration_s' segundos en una sola nube
    de puntos ("foto fija"). Para evitar que particulas de polvo en el aire
    (que solo aparecen en algunos frames, en posiciones distintas cada vez)
    contaminen la captura, cada celda de voxel debe estar presente en al
    menos 'persistence_ratio' de los frames capturados para conservarse; el
    resto se descarta por no ser lo suficientemente persistente.

    Si se pasa 'stop_event' y se activa antes de que termine la duracion,
    la captura corta ahi y procesa los frames acumulados hasta ese momento.

    'cone_angle_deg' (opcional) limita cada frame a un cono de ese angulo
    total centrado en 'forward_axis', para excluir lo que esta fuera del
    campo de vision deseado (por ejemplo, todo lo que queda detras de una
    caja de prueba). 'max_distance_m' (opcional) descarta puntos mas lejos
    que ese valor. Sin ninguno de los dos, se captura todo el campo de
    vision del sensor.
    """
    collected_points: list[np.ndarray] = []
    collected_colors: list[np.ndarray] = []
    start = time.time()

    while time.time() - start < duration_s:
        if stop_event is not None and stop_event.is_set():
            break
        points, colors = read_frame_points_and_colors(
            connection,
            max_points=max_points_per_frame,
            timeout_ms=frame_timeout_ms,
            max_distance_m=max_distance_m,
            cone_angle_deg=cone_angle_deg,
            forward_axis=forward_axis,
        )
        if points is not None:
            collected_points.append(points)
            collected_colors.append(colors)

    if not collected_points:
        raise RuntimeError(
            f"No se pudo capturar ningun frame valido del sensor en {duration_s:.0f} s. "
            "Verifica la conexion y que el sensor apunte a una superficie dentro de su "
            "rango de medicion."
        )

    num_frames = len(collected_points)
    frame_keys = [np.unique(_voxel_keys(points, voxel_size)) for points in collected_points]
    unique_keys, counts = np.unique(np.concatenate(frame_keys), return_counts=True)
    min_frames = max(1, int(round(persistence_ratio * num_frames)))
    persistent_keys = unique_keys[counts >= min_frames]

    if len(persistent_keys) == 0:
        raise RuntimeError(
            "Ningun punto persistio en suficientes frames durante la captura (posible "
            "polvo o ruido excesivo). Reduce la persistencia requerida o repite la captura."
        )

    kept_points = []
    kept_colors = []
    for points, colors in zip(collected_points, collected_colors):
        mask = np.isin(_voxel_keys(points, voxel_size), persistent_keys)
        if mask.any():
            kept_points.append(points[mask])
            kept_colors.append(colors[mask])

    merged_points = np.concatenate(kept_points, axis=0)
    merged_colors = np.clip(np.concatenate(kept_colors, axis=0), 0.0, 1.0)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(merged_points)
    cloud.colors = o3d.utility.Vector3dVector(merged_colors)
    if voxel_size and voxel_size > 0:
        cloud = cloud.voxel_down_sample(voxel_size)
    return cloud


def stream_frames(
    connection: AuroraConnection,
    callback: Callable[[np.ndarray], None],
    stop_event: threading.Event,
    max_points: int = 40000,
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
