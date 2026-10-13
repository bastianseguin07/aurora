"""Visor Open3D auxiliar para inspeccionar nubes y compartirlas por la red local.

En modo en vivo muestra frames locales del sensor sin compensar movimiento ni
registrarlos con una sesion de medicion.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import open3d as o3d

from aurora_sensor import AuroraConnection, read_frame_points
from live_stream_server import LiveStreamServer
LIVE_STREAM_INTERVAL_S = 0.2  # ritmo de envio al celular, mas lento que el redibujado local


class LiveViewer:
    def __init__(self, base_cloud: o3d.geometry.PointCloud, log=print):
        self._base_cloud_initial = o3d.geometry.PointCloud(base_cloud)
        self._log = log

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._stream_server: LiveStreamServer | None = None
        self._stream_base_sent = False

        self.show_updated = True
        self.aurora_connection: AuroraConnection | None = None
        self._pending_static_cloud: o3d.geometry.PointCloud | None = None
        self.live_max_distance_m: float | None = None
        self.live_cone_angle_deg: float | None = None
        self.live_forward_axis: str = "z"
        self.live_invert_y: bool = False
        self.live_invert_z: bool = False

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
        self.stop_stream()

    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def start_stream(self) -> str:
        """Arranca (si hace falta) el servidor de transmision al celular y devuelve su URL."""
        if self._stream_server is None:
            self._stream_server = LiveStreamServer(log=self._log)
        url = self._stream_server.start()
        self._stream_base_sent = False
        return url

    def stop_stream(self) -> None:
        if self._stream_server is not None:
            self._stream_server.stop()
            self._stream_server = None
        self._stream_base_sent = False

    def is_streaming(self) -> bool:
        return bool(self._stream_server and self._stream_server.is_running())

    def set_show_updated(self, value: bool) -> None:
        with self._lock:
            self.show_updated = value

    def set_live_sensor(self, connection: AuroraConnection | None) -> None:
        with self._lock:
            self.aurora_connection = connection

    def set_live_filters(
        self,
        max_distance_m: float | None,
        cone_angle_deg: float | None,
        forward_axis: str,
        invert_y: bool,
        invert_z: bool,
    ) -> None:
        """Ajustes de captura en vivo: rango maximo, angulo de cono (FOV), eje
        frontal del sensor, e inversion de ejes para adaptarse al montaje."""
        with self._lock:
            self.live_max_distance_m = max_distance_m
            self.live_cone_angle_deg = cone_angle_deg
            self.live_forward_axis = forward_axis
            self.live_invert_y = invert_y
            self.live_invert_z = invert_z

    def push_static_points(self, points_xyz: np.ndarray) -> None:
        """Entrega una nube 'actualizada' fija (por ejemplo, la cargada desde archivo)."""
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points_xyz)
        with self._lock:
            self._pending_static_cloud = cloud
            self.aurora_connection = None

    def _apply_axis_transform(self, points_xyz: np.ndarray, invert_y: bool, invert_z: bool) -> np.ndarray:
        transformed = points_xyz.copy()
        if invert_y:
            transformed[:, 1] *= -1.0
        if invert_z:
            transformed[:, 2] *= -1.0
        return transformed

    def _make_inspection_cloud(self, points_xyz: np.ndarray) -> o3d.geometry.PointCloud:
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points_xyz)
        # This viewer does not register the clouds. Avoid presenting unaligned
        # nearest-neighbour distances as a thickness heatmap.
        cloud.paint_uniform_color((0.22, 0.72, 0.82))
        return cloud

    def _run(self) -> None:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Aurora - Vista 3D", width=1024, height=768)

        base_vis = o3d.geometry.PointCloud(self._base_cloud_initial)
        vis.add_geometry(base_vis)

        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.059, 0.067, 0.082])  # #0F1115
        opt.point_size = 1.5

        updated_vis = o3d.geometry.PointCloud()
        updated_added = False
        last_colored: o3d.geometry.PointCloud | None = None
        last_sensor_poll = 0.0
        last_stream_send = 0.0

        while not self._stop_event.is_set():
            with self._lock:
                show = self.show_updated
                connection = self.aurora_connection
                pending_static = self._pending_static_cloud
                self._pending_static_cloud = None
                live_max_distance_m = self.live_max_distance_m
                live_cone_angle_deg = self.live_cone_angle_deg
                live_forward_axis = self.live_forward_axis
                live_invert_y = self.live_invert_y
                live_invert_z = self.live_invert_z

            if self._stream_server is not None and self._stream_server.is_running() and not self._stream_base_sent:
                base_colors = (
                    np.asarray(base_vis.colors)
                    if base_vis.has_colors()
                    else np.full((len(base_vis.points), 3), 0.6)
                )
                self._stream_base_sent = self._stream_server.broadcast_base(np.asarray(base_vis.points), base_colors)

            new_colored = None
            if connection is not None:
                now = time.time()
                if now - last_sensor_poll > 0.08:
                    points = read_frame_points(
                        connection,
                        max_points=40000,
                        timeout_ms=50,
                        max_distance_m=live_max_distance_m,
                        cone_angle_deg=live_cone_angle_deg,
                        forward_axis=live_forward_axis,
                    )
                    if points is not None and len(points) > 0:
                        points = self._apply_axis_transform(points, live_invert_y, live_invert_z)
                        new_colored = self._make_inspection_cloud(points)
                    last_sensor_poll = now
            elif pending_static is not None:
                new_colored = self._make_inspection_cloud(np.asarray(pending_static.points))

            if new_colored is not None:
                if self._stream_server is not None and self._stream_server.is_running():
                    now = time.time()
                    if now - last_stream_send > LIVE_STREAM_INTERVAL_S:
                        self._stream_server.broadcast_live(
                            np.asarray(new_colored.points), np.asarray(new_colored.colors)
                        )
                        last_stream_send = now
                last_colored = new_colored
                updated_vis.points = new_colored.points
                updated_vis.colors = new_colored.colors
                if show and not updated_added:
                    vis.add_geometry(updated_vis, reset_bounding_box=False)
                    updated_added = True
                elif updated_added:
                    vis.update_geometry(updated_vis)

            if show and not updated_added and last_colored is not None:
                updated_vis.points = last_colored.points
                updated_vis.colors = last_colored.colors
                vis.add_geometry(updated_vis, reset_bounding_box=False)
                updated_added = True
            elif not show and updated_added:
                vis.remove_geometry(updated_vis, reset_bounding_box=False)
                updated_added = False

            if not vis.poll_events():
                break
            vis.update_renderer()
            time.sleep(0.01)

        vis.destroy_window()
