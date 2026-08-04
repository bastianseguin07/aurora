"""
Visor 3D independiente (ventana propia de Open3D, separada de la GUI de
tkinter). Muestra siempre la nube base (gris) y, opcionalmente, la nube
"actualizada" (con shotcrete) coloreada por espesor - en modo estatico
(una nube ya cargada/calculada) o en vivo (leyendo frames del sensor
Aurora de forma continua).
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np
import open3d as o3d

from aurora_sensor import AuroraConnection, read_frame_points
from pointcloud_core import build_heatmap_cloud, build_heatmap_cloud_banded


class LiveViewer:
    def __init__(self, base_cloud: o3d.geometry.PointCloud):
        self._base_cloud_initial = o3d.geometry.PointCloud(base_cloud)
        self._distance_reference_cloud = o3d.geometry.PointCloud(base_cloud)
        self._using_live_baseline = False

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.show_updated = True
        self.color_mode = "continuous"  # o "banded"
        self.low_threshold = 0.02
        self.high_threshold = 0.05
        self.max_distance: float | None = None

        self.aurora_connection: AuroraConnection | None = None
        self._pending_static_cloud: o3d.geometry.PointCloud | None = None
        self._pending_base_cloud: o3d.geometry.PointCloud | None = None
        self._last_live_points: np.ndarray | None = None
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

    def is_running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def set_show_updated(self, value: bool) -> None:
        with self._lock:
            self.show_updated = value

    def set_color_mode(self, mode: str, low_threshold: float, high_threshold: float, max_distance: float | None) -> None:
        with self._lock:
            self.color_mode = mode
            self.low_threshold = low_threshold
            self.high_threshold = high_threshold
            self.max_distance = max_distance

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
        with self._lock:
            self.live_max_distance_m = max_distance_m
            self.live_cone_angle_deg = cone_angle_deg
            self.live_forward_axis = forward_axis
            self.live_invert_y = invert_y
            self.live_invert_z = invert_z

    def capture_live_baseline(self) -> int:
        with self._lock:
            if self._last_live_points is None or len(self._last_live_points) == 0:
                raise RuntimeError("Aun no hay un frame en vivo disponible para fijar la base.")
            baseline = o3d.geometry.PointCloud()
            baseline.points = o3d.utility.Vector3dVector(self._last_live_points.copy())
            self._distance_reference_cloud = baseline
            self._pending_base_cloud = baseline
            self._using_live_baseline = True
            return len(self._last_live_points)

    def clear_live_baseline(self) -> None:
        with self._lock:
            baseline = o3d.geometry.PointCloud(self._base_cloud_initial)
            self._distance_reference_cloud = baseline
            self._pending_base_cloud = baseline
            self._using_live_baseline = False

    def get_live_snapshot_clouds(self) -> tuple[o3d.geometry.PointCloud | None, o3d.geometry.PointCloud | None, bool]:
        with self._lock:
            if self._last_live_points is None or len(self._last_live_points) == 0:
                return None, None, self._using_live_baseline

            current = o3d.geometry.PointCloud()
            current.points = o3d.utility.Vector3dVector(self._last_live_points.copy())
            baseline = o3d.geometry.PointCloud(self._distance_reference_cloud)
            using_live_baseline = self._using_live_baseline

        return baseline, current, using_live_baseline

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

    def _colorize(self, points_xyz: np.ndarray) -> o3d.geometry.PointCloud:
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points_xyz)
        with self._lock:
            reference_cloud = o3d.geometry.PointCloud(self._distance_reference_cloud)
        distances = np.asarray(cloud.compute_point_cloud_distance(reference_cloud))
        with self._lock:
            mode = self.color_mode
            low, high = self.low_threshold, self.high_threshold
            max_d = self.max_distance
        if mode == "banded":
            return build_heatmap_cloud_banded(cloud, distances, low, high)
        return build_heatmap_cloud(cloud, distances, max_d)

    def _run(self) -> None:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Aurora - Vista 3D", width=1024, height=768)

        base_vis = o3d.geometry.PointCloud(self._base_cloud_initial)
        base_vis.paint_uniform_color([0.6, 0.6, 0.6])
        vis.add_geometry(base_vis)

        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.08, 0.08, 0.08])
        opt.point_size = 2.5

        updated_vis = o3d.geometry.PointCloud()
        updated_added = False
        last_colored: o3d.geometry.PointCloud | None = None
        last_sensor_poll = 0.0

        while not self._stop_event.is_set():
            with self._lock:
                show = self.show_updated
                connection = self.aurora_connection
                pending_static = self._pending_static_cloud
                pending_base = self._pending_base_cloud
                self._pending_static_cloud = None
                self._pending_base_cloud = None
                live_max_distance_m = self.live_max_distance_m
                live_cone_angle_deg = self.live_cone_angle_deg
                live_forward_axis = self.live_forward_axis
                live_invert_y = self.live_invert_y
                live_invert_z = self.live_invert_z

            if pending_base is not None:
                base_vis.points = pending_base.points
                base_vis.paint_uniform_color([0.6, 0.6, 0.6])
                vis.update_geometry(base_vis)

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
                        with self._lock:
                            self._last_live_points = points.copy()
                        new_colored = self._colorize(points)
                    last_sensor_poll = now
            elif pending_static is not None:
                new_colored = self._colorize(np.asarray(pending_static.points))

            if new_colored is not None:
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
