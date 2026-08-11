"""
Visor 3D embebido DENTRO de la propia ventana GTK (seccion experimental /
de prueba), a diferencia de live_viewer.py que abre una ventana de Open3D
aparte. Muestra la nube base y una superposicion con un resaltado sutil
(gris -> ambar tenue) donde la nube actualizada difiere de la base.

Nota tecnica: el renderizador nuevo de Open3D (open3d.visualization.rendering
.OffscreenRenderer, basado en Filament) NO soporta modo headless en Windows
("EGL Headless is not supported on this platform" - confirmado en pruebas).
Por eso este modulo usa la API "clasica" (o3d.visualization.Visualizer con
create_window(visible=False) + capture_screen_float_buffer), que si funciona
en Windows y Linux por igual (es la misma API que ya usa live_viewer.py).

El mouse controla la camara sobre el widget:
    - Arrastrar con click izquierdo: orbitar
    - Rueda del mouse: zoom
"""

from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gdk, Gtk  # noqa: E402

import cairo  # noqa: E402
import numpy as np  # noqa: E402
import open3d as o3d  # noqa: E402


class EmbeddedComparisonViewer(Gtk.DrawingArea):
    def __init__(self, width: int = 900, height: int = 550) -> None:
        super().__init__()
        self._width = width
        self._height = height
        self.set_size_request(width, height)

        self._vis: o3d.visualization.Visualizer | None = None
        self._last_mouse: tuple[float, float] | None = None
        self._has_geometry = False

        self.set_events(
            self.get_events()
            | Gdk.EventMask.BUTTON_PRESS_MASK
            | Gdk.EventMask.BUTTON_RELEASE_MASK
            | Gdk.EventMask.POINTER_MOTION_MASK
            | Gdk.EventMask.SCROLL_MASK
        )
        self.connect("draw", self._on_draw)
        self.connect("button-press-event", self._on_button_press)
        self.connect("button-release-event", self._on_button_release)
        self.connect("motion-notify-event", self._on_motion)
        self.connect("scroll-event", self._on_scroll)
        self.connect("destroy", self._on_destroy)

    def _ensure_visualizer(self) -> None:
        if self._vis is not None:
            return
        self._vis = o3d.visualization.Visualizer()
        self._vis.create_window(width=self._width, height=self._height, visible=False)
        opt = self._vis.get_render_option()
        opt.background_color = np.array([0.059, 0.067, 0.082])  # #0F1115
        opt.point_size = 1.5

    def load_clouds(
        self, base_cloud: o3d.geometry.PointCloud, overlay_cloud: o3d.geometry.PointCloud
    ) -> None:
        """Carga (o reemplaza) la nube base (gris) y la nube superpuesta con resaltado sutil."""
        self._ensure_visualizer()
        self._vis.clear_geometries()

        base_copy = o3d.geometry.PointCloud(base_cloud)
        if not base_copy.has_colors():
            base_copy.paint_uniform_color([0.5, 0.5, 0.5])

        self._vis.add_geometry(base_copy, reset_bounding_box=True)
        self._vis.add_geometry(overlay_cloud, reset_bounding_box=False)
        self._vis.get_view_control().set_zoom(0.7)
        self._has_geometry = True
        self.queue_draw()

    def clear(self) -> None:
        if self._vis is not None:
            self._vis.clear_geometries()
        self._has_geometry = False
        self.queue_draw()

    def _render_frame(self) -> np.ndarray | None:
        if self._vis is None or not self._has_geometry:
            return None
        image = self._vis.capture_screen_float_buffer(do_render=True)
        return (np.asarray(image) * 255).astype(np.uint8)

    def _on_draw(self, _widget, ctx) -> bool:
        img = self._render_frame()
        if img is None:
            ctx.set_source_rgb(0.059, 0.067, 0.082)  # #0F1115
            ctx.paint()
            return True

        height, width, _ = img.shape
        bgra = np.empty((height, width, 4), dtype=np.uint8)
        bgra[:, :, 0] = img[:, :, 2]
        bgra[:, :, 1] = img[:, :, 1]
        bgra[:, :, 2] = img[:, :, 0]
        bgra[:, :, 3] = 255

        stride = cairo.ImageSurface.format_stride_for_width(cairo.FORMAT_ARGB32, width)
        if stride != width * 4:
            padded = np.zeros((height, stride // 4, 4), dtype=np.uint8)
            padded[:, :width, :] = bgra
            bgra = padded

        surface = cairo.ImageSurface.create_for_data(bytearray(bgra.tobytes()), cairo.FORMAT_ARGB32, width, height, stride)
        ctx.set_source_surface(surface, 0, 0)
        ctx.paint()
        return True

    def _on_button_press(self, _widget, event) -> bool:
        self._last_mouse = (event.x, event.y)
        return True

    def _on_button_release(self, _widget, _event) -> bool:
        self._last_mouse = None
        return True

    def _on_motion(self, _widget, event) -> bool:
        if self._last_mouse is None or self._vis is None:
            return True
        dx = event.x - self._last_mouse[0]
        dy = event.y - self._last_mouse[1]
        self._last_mouse = (event.x, event.y)
        self._vis.get_view_control().rotate(dx * 2.0, dy * 2.0)
        self.queue_draw()
        return True

    def _on_scroll(self, _widget, event) -> bool:
        if self._vis is None:
            return True
        ctr = self._vis.get_view_control()
        if event.direction == Gdk.ScrollDirection.UP:
            ctr.scale(2.0)
        elif event.direction == Gdk.ScrollDirection.DOWN:
            ctr.scale(-2.0)
        self.queue_draw()
        return True

    def _on_destroy(self, *_args) -> None:
        if self._vis is not None:
            self._vis.destroy_window()
            self._vis = None
