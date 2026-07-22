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
    connection: AuroraConnection, max_points: int = 50000, timeout_ms: int = 200
) -> np.ndarray | None:
    """
    Espera y devuelve un frame de puntos 3D (Nx3) del sensor, filtrando puntos
    invalidos (ceros, NaN/inf, fuera del rango 0.1m-50m). Devuelve None si no
    hubo un frame disponible dentro del timeout.
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
    valid &= (distance > 0.1) & (distance < 50.0)
    points = points[valid]

    if len(points) == 0:
        return None
    if len(points) > max_points:
        idx = np.random.choice(len(points), max_points, replace=False)
        points = points[idx]

    return points


def capture_snapshot(
    connection: AuroraConnection,
    num_frames: int = 15,
    max_points_per_frame: int = 100000,
    voxel_size: float = 0.005,
    frame_timeout_ms: int = 300,
) -> o3d.geometry.PointCloud:
    """
    Acumula varios frames del sensor en una sola nube de puntos ("foto fija"),
    para reducir el ruido de un unico frame individual.
    """
    collected: list[np.ndarray] = []
    attempts = 0
    max_attempts = max(num_frames * 20, 20)

    while len(collected) < num_frames and attempts < max_attempts:
        attempts += 1
        points = read_frame_points(connection, max_points=max_points_per_frame, timeout_ms=frame_timeout_ms)
        if points is not None:
            collected.append(points)

    if not collected:
        raise RuntimeError(
            "No se pudo capturar ningun frame valido del sensor. Verifica la conexion "
            "y que el sensor apunte a una superficie dentro de su rango de medicion."
        )

    merged = np.concatenate(collected, axis=0)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(merged)
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
