"""
Ventana Open3D aparte, pensada como un "gizmo" de posicionamiento en vez de
un visor de nubes: no muestra ninguna nube de puntos, solo un piso de
referencia y la posicion objetivo/actual del sensor (esfera que pasa de
rojo a ambar a verde segun la distancia).

La indicacion de "hacia donde mover" NO sale del sensor (version anterior,
con flechas en escalera desde su posicion): quedaba dificil de leer porque
se mezclaba con la esfera y cambiaba de tamaño/angulo todo el tiempo. En
cambio, hay un unico indicador FIJO en la esquina superior izquierda de la
pantalla (recalculado cada frame a partir de la camara actual, asi se
mantiene pegado a esa esquina aunque el usuario orbite/haga zoom): una
flecha que apunta, en pantalla, hacia donde corregir el eje activo. Los
ejes se corrigen de a uno (orden X, Y, Z): mientras el eje X este fuera de
tolerancia se muestra su flecha (color cian); una vez corregido pasa a
mostrar la de Y (magenta), despues Z (amarillo); cuando los tres estan
alineados se muestra un punto verde en ese mismo lugar. Los colores
coinciden con los usados para X/Y/Z en la ventana de metricas flotante
(ver _build_imu_hud_window en gui_gtk.py).

Sigue el mismo patron que live_viewer.py: hilo propio con la API "clasica"
de Open3D (Visualizer), loop de render con poll_events()/update_renderer().

No se dibuja la orientacion (roll/pitch/yaw) del sensor aca — ese dato ya
se muestra como numero en la ventana de metricas; agregarlo tambien en 3D
sumaba triedros superpuestos sin aportar claridad.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import open3d as o3d

TARGET_COLOR = (0.20, 0.80, 0.35)  # verde - posicion objetivo / "en posicion"
NEAR_COLOR = (0.95, 0.65, 0.10)  # ambar - acercandose
FAR_COLOR = (0.90, 0.25, 0.25)  # rojo - lejos de la posicion objetivo

# Un color por eje (cian/magenta/amarillo), a proposito distinto de la
# semantica rojo/ambar/verde de arriba (esa es "que tan lejos", esta es
# "que eje") — deben coincidir con AXIS_HUD_COLORS_HEX en gui_gtk.py.
AXIS_COLORS = {
    "x": (0.133, 0.827, 0.933),
    "y": (0.910, 0.475, 0.976),
    "z": (0.980, 0.800, 0.082),
}
MARKER_RADIUS_M = 0.025
ARROW_CYLINDER_RADIUS_M = 0.006
ARROW_CONE_RADIUS_M = 0.014
GRID_EXTENT_M = 0.4
GRID_STEP_M = 0.1

# El indicador de "hacia donde mover" se ancla a un pixel fijo de la
# pantalla (fraccion del ancho/alto de la ventana) proyectando con la
# intrinseca/extrinseca real de la camara — asi queda siempre en el mismo
# lugar (esquina superior izquierda) sin importar el zoom o si el usuario
# orbita la vista con el mouse.
HUD_SCREEN_FRACTION_X = 0.14
HUD_SCREEN_FRACTION_Y = 0.14
HUD_DEPTH_M = 0.4
HUD_ARROW_LENGTH_M = 0.07
HUD_DONE_RADIUS_M = 0.018

_Z_AXIS = np.array([0.0, 0.0, 1.0])


def _rotation_aligning_z_to(direction: np.ndarray) -> np.ndarray:
    """Matriz de rotacion que lleva el eje +Z al vector 'direction' — para
    orientar la flecha (que Open3D arma apuntando en +Z) hacia donde
    corresponda."""
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return np.eye(3)
    unit = direction / norm
    axis = np.cross(_Z_AXIS, unit)
    axis_norm = float(np.linalg.norm(axis))
    dot = float(np.clip(np.dot(_Z_AXIS, unit), -1.0, 1.0))
    if axis_norm < 1e-9:
        if dot > 0:
            return np.eye(3)
        return o3d.geometry.get_rotation_matrix_from_axis_angle(np.array([1.0, 0.0, 0.0]) * np.pi)
    angle = np.arccos(dot)
    return o3d.geometry.get_rotation_matrix_from_axis_angle(axis / axis_norm * angle)


def _proximity_color(distance_m: float, tolerance_m: float) -> tuple[float, float, float]:
    if distance_m <= tolerance_m:
        return TARGET_COLOR
    if distance_m <= tolerance_m * 4.0:
        return NEAR_COLOR
    return FAR_COLOR


def _marker_sphere(
    position: np.ndarray, color: tuple[float, float, float], radius: float = MARKER_RADIUS_M
) -> o3d.geometry.TriangleMesh:
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
    sphere.compute_vertex_normals()
    sphere.paint_uniform_color(color)
    sphere.translate(position)
    return sphere


def _direction_arrow(
    start: np.ndarray, end: np.ndarray, color: tuple[float, float, float]
) -> Optional[o3d.geometry.TriangleMesh]:
    """Flecha solida desde 'start' hacia 'end' cuyo largo es la distancia
    real entre ambos puntos, para que tanto la direccion como la magnitud
    del movimiento necesario se lean de un vistazo."""
    vector = end - start
    length = float(np.linalg.norm(vector))
    if length < 1e-3:
        return None
    cone_height = min(0.03, length * 0.35)
    cylinder_height = max(length - cone_height, 1e-4)
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=ARROW_CYLINDER_RADIUS_M,
        cone_radius=ARROW_CONE_RADIUS_M,
        cylinder_height=cylinder_height,
        cone_height=cone_height,
        resolution=16,
    )
    arrow.compute_vertex_normals()
    arrow.paint_uniform_color(color)
    arrow.rotate(_rotation_aligning_z_to(vector), center=(0, 0, 0))
    arrow.translate(start)
    return arrow


def _hud_frame(view_control) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Punto de anclaje del indicador de la esquina superior izquierda, y
    los ejes derecha/arriba de la camara en ese instante. Se proyecta un
    pixel fijo (HUD_SCREEN_FRACTION_X/Y del ancho/alto) con la intrinseca
    real de la camara — la relacion (pixel - centro)/foco es independiente
    de la profundidad elegida, asi que HUD_DEPTH_M solo define que tan
    'cerca' queda el indicador, no en que pixel aparece."""
    params = view_control.convert_to_pinhole_camera_parameters()
    extrinsic = np.asarray(params.extrinsic)
    rotation = extrinsic[:3, :3]
    translation = extrinsic[:3, 3]
    camera_position = -rotation.T @ translation
    right = rotation.T @ np.array([1.0, 0.0, 0.0])
    down = rotation.T @ np.array([0.0, 1.0, 0.0])  # convencion Open3D/OpenCV: y de camara apunta hacia abajo
    forward = rotation.T @ np.array([0.0, 0.0, 1.0])
    up = -down

    intrinsic = np.asarray(params.intrinsic.intrinsic_matrix)
    fx, fy = float(intrinsic[0, 0]), float(intrinsic[1, 1])
    cx, cy = float(intrinsic[0, 2]), float(intrinsic[1, 2])
    px = params.intrinsic.width * HUD_SCREEN_FRACTION_X
    py = params.intrinsic.height * HUD_SCREEN_FRACTION_Y
    x_cam = (px - cx) / fx * HUD_DEPTH_M
    y_cam = (py - cy) / fy * HUD_DEPTH_M

    anchor = camera_position + right * x_cam + down * y_cam + forward * HUD_DEPTH_M
    return anchor, right, up


def _active_axis_correction(offset: np.ndarray, tolerance_m: float) -> Optional[tuple[str, np.ndarray]]:
    """Primer eje (en orden X, Y, Z) todavia fuera de tolerancia, junto con
    el vector de correccion (hacia donde hay que mover) en ese eje — se
    corrige de a un eje por vez en vez de mostrar los tres juntos. None si
    ya estan los tres alineados."""
    for index, axis in enumerate(("x", "y", "z")):
        if abs(offset[index]) > tolerance_m:
            correction = np.zeros(3)
            correction[index] = -offset[index]
            return axis, correction
    return None


def _hud_indicator(
    anchor: np.ndarray, right: np.ndarray, up: np.ndarray, active: Optional[tuple[str, np.ndarray]]
) -> o3d.geometry.TriangleMesh:
    """Indicador fijo de la esquina: si queda algun eje por corregir, una
    flecha plana (en el plano de la pantalla) apuntando hacia donde mover;
    si los tres ejes ya estan alineados, un punto verde de 'listo'."""
    if active is None:
        return _marker_sphere(anchor, TARGET_COLOR, radius=HUD_DONE_RADIUS_M)

    axis, correction = active
    dx = float(np.dot(correction, right))
    dy = float(np.dot(correction, up))
    screen_dir = np.array([dx, dy])
    norm = float(np.linalg.norm(screen_dir))
    if norm < 1e-6:
        screen_dir, norm = np.array([0.0, 1.0]), 1.0
    screen_dir /= norm

    direction_3d = right * screen_dir[0] + up * screen_dir[1]
    direction_3d /= float(np.linalg.norm(direction_3d))

    half = HUD_ARROW_LENGTH_M / 2.0
    arrow = _direction_arrow(anchor - direction_3d * half, anchor + direction_3d * half, AXIS_COLORS[axis])
    return arrow if arrow is not None else _marker_sphere(anchor, AXIS_COLORS[axis], radius=HUD_DONE_RADIUS_M)


def _floor_grid(extent: float, step: float, color=(0.22, 0.24, 0.30)) -> o3d.geometry.LineSet:
    """Piso de referencia (lineas finas en el plano horizontal que pasa por
    el objetivo) solo para dar nocion de escala y profundidad — sin el, una
    esfera y una flecha flotando en el vacio son dificiles de ubicar."""
    points: list[list[float]] = []
    lines: list[list[int]] = []
    steps = int(round(extent / step))
    for i in range(-steps, steps + 1):
        offset = i * step
        idx = len(points)
        points.append([offset, -extent, 0.0])
        points.append([offset, extent, 0.0])
        lines.append([idx, idx + 1])
        idx = len(points)
        points.append([-extent, offset, 0.0])
        points.append([extent, offset, 0.0])
        lines.append([idx, idx + 1])
    grid = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    grid.colors = o3d.utility.Vector3dVector([color] * len(lines))
    return grid


class PoseAlignmentViewer:
    def __init__(self, reference_position: np.ndarray, position_tolerance_m: float, log=print):
        self._reference_position = np.asarray(reference_position, dtype=np.float64)
        self._position_tolerance_m = float(position_tolerance_m)
        self._log = log

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._current_position: np.ndarray | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def update_current_pose(self, position: np.ndarray) -> None:
        with self._lock:
            self._current_position = np.asarray(position, dtype=np.float64)

    def _run(self) -> None:
        vis = o3d.visualization.Visualizer()
        # left=340: deja lugar a la izquierda para la ventana de metricas
        # (ver _build_imu_hud_window en gui_gtk.py, que se abre en (20, 20)),
        # para que no queden superpuestas por default.
        vis.create_window(window_name="Aurora - Alineacion IMU", width=760, height=760, left=340, top=20)

        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.059, 0.067, 0.082])  # #0F1115
        opt.mesh_show_back_face = True
        opt.line_width = 2.0

        grid = _floor_grid(GRID_EXTENT_M, GRID_STEP_M)
        vis.add_geometry(grid, reset_bounding_box=True)

        target_marker = _marker_sphere(np.zeros(3), TARGET_COLOR)
        vis.add_geometry(target_marker, reset_bounding_box=False)

        view = vis.get_view_control()
        # 'front'/'up' deben ser perpendiculares entre si: si no lo son,
        # Open3D arma una extrinseca con una rotacion no ortonormal (sesgada),
        # lo que rompe la proyeccion pinhole manual de _hud_frame (el
        # indicador terminaba en cualquier lado de la pantalla). Se
        # ortogonaliza 'up' contra 'front' (Gram-Schmidt) antes de fijarlos.
        front = np.array([0.5, -0.7, 0.55])
        front /= np.linalg.norm(front)
        up = np.array([0.0, 0.0, 1.0])
        up -= np.dot(up, front) * front
        up /= np.linalg.norm(up)

        view.set_lookat([0.0, 0.0, 0.0])
        view.set_front(front.tolist())
        view.set_up(up.tolist())
        view.set_zoom(0.7)

        current_marker: o3d.geometry.TriangleMesh | None = None
        hud_indicator: o3d.geometry.TriangleMesh | None = None
        last_poll = 0.0

        self._log(
            "Vista 3D de alineacion IMU abierta — verde = objetivo; la flecha de la "
            "esquina superior izquierda indica hacia donde mover, un eje a la vez "
            "(cian=X, magenta=Y, amarillo=Z)."
        )

        while not self._stop_event.is_set():
            now = time.time()
            if now - last_poll > 0.1:
                with self._lock:
                    position = self._current_position
                last_poll = now

                if position is not None:
                    offset = position - self._reference_position
                    distance = float(np.linalg.norm(offset))
                    color = _proximity_color(distance, self._position_tolerance_m)

                    if current_marker is not None:
                        vis.remove_geometry(current_marker, reset_bounding_box=False)
                    current_marker = _marker_sphere(offset, color)
                    vis.add_geometry(current_marker, reset_bounding_box=False)

                    # Se recalcula siempre (no solo si el sensor se movio) para
                    # que el indicador siga la camara si el usuario orbita/hace zoom.
                    if hud_indicator is not None:
                        vis.remove_geometry(hud_indicator, reset_bounding_box=False)
                    anchor, right, up = _hud_frame(view)
                    active = _active_axis_correction(offset, self._position_tolerance_m)
                    hud_indicator = _hud_indicator(anchor, right, up, active)
                    vis.add_geometry(hud_indicator, reset_bounding_box=False)

            if not vis.poll_events():
                break
            vis.update_renderer()
            time.sleep(0.01)

        vis.destroy_window()
