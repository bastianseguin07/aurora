"""
GUI de escritorio (GTK3 via PyGObject) para comparar dos nubes de puntos .ply
sin usar la consola. Incluye recorte (crop) manual o seleccionado
visualmente en un visor 3D, captura desde el sensor Aurora, coloreado por
espesor (continuo o 3 niveles) y vista 3D en vivo o estatica.

Requiere las librerias de sistema GTK3 + PyGObject:
    Ubuntu: sudo apt install python3-gi gir1.2-gtk-3.0
(el venv del proyecto debe crearse con --system-site-packages para heredar
estos bindings; ver setup.sh).

Ejecutar con:
    python gui_gtk.py
"""

from __future__ import annotations

import os
import math
import json
import cairo
import queue
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

# Pide a GTK que muestre Gtk.FileChooserNative mediante el portal del escritorio
# GNOME cuando está disponible, incluso fuera de una aplicación empaquetada.
os.environ.setdefault("GTK_USE_PORTAL", "1")

import gi

gi.require_version("Gdk", "3.0")
gi.require_version("Gtk", "3.0")
from gi.repository import Gdk, GdkPixbuf, GLib, Gtk, Pango  # noqa: E402

import numpy as np  # noqa: E402

import aurora_sensor  # noqa: E402
from embedded_viewer import EmbeddedComparisonViewer  # noqa: E402
from live_stream_server import make_qr_image  # noqa: E402
from live_viewer import LiveViewer  # noqa: E402
from pose_alignment_viewer import PoseAlignmentViewer  # noqa: E402
from pointcloud_core import (  # noqa: E402
    DistanceStats,
    PipelineParams,
    RaycastPipelineParams,
    RaycastPipelineResult,
    SIX_BAND_COLORS,
    SIX_BAND_LABELS,
    HEATMAP_COLOR_HIGH,
    HEATMAP_COLOR_LOW,
    BAND_COLOR_HIGH,
    BAND_COLOR_LOW,
    BAND_COLOR_MEDIUM,
    apply_rigid_transform,
    build_heatmap_cloud,
    build_heatmap_cloud_banded,
    build_heatmap_cloud_six_bands,
    build_subtle_overlay_cloud,
    compute_c2c_distance,
    compute_rigid_transform,
    compute_pose_relative_transform,
    crop_pair_by_face_depth,
    estimate_wall_crop_bounds,
    grayscale_point_cloud,
    load_point_cloud,
    load_reference_photo,
    load_reference_photo_file,
    load_reference_pose,
    reference_photo_path_for,
    reference_pose_alignment_version,
    pick_crop_seed_from_pair,
    rectangular_face_bounds,
    pick_landmark_points,
    pick_landmark_points_from_photo,
    render_reference_photo_from_cloud,
    rigid_transform_rms_error,
    run_pipeline,
    save_reference_photo,
    save_transformed_reference_photo,
    world_delta_to_sensor_view,
    save_reference_pose,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent


# Paleta "Dark Industrial" - estados de sensor/pipeline.
COLOR_OK = "#34C759"
COLOR_ERROR = "#FF3B30"
COLOR_WARN = "#FFCC00"
COLOR_FILE_NAME = "#FFD54F"

# Un color por eje (cian/magenta/amarillo) para la guia de alineacion IMU —
# deben coincidir con AXIS_COLORS en pose_alignment_viewer.py, asi el
# tramo de flecha en la vista 3D se puede relacionar de un vistazo con su
# numero correspondiente en el panel/HUD.
AXIS_HUD_COLORS = {"x": "#22D3EE", "y": "#E879F9", "z": "#FACC15"}
IMU_SENSOR_VIEW_DIRECTIONS = {
    "x": ("derecha", "izquierda"),
    "y": ("abajo", "arriba"),
    "z": ("adelante", "atras"),
}

# Paleta base del tema.
COLOR_BG = "#0F1115"
COLOR_CARD = "#1A1D24"
COLOR_BORDER = "#2E3440"
COLOR_ACCENT = "#56B4C5"
COLOR_ACCENT_DARK = "#3B91A1"
COLOR_MEDIAN = "#FFCC00"
COLOR_TEXT = "#F8F9FA"
COLOR_TEXT_MUTED = "#8E95A5"

CSS = f"""
* {{
    font-family: "Inter", "Roboto", "Segoe UI", sans-serif;
    font-size: 10pt;
}}

window, .background {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
}}

window.csd,
window.csd decoration {{
    border: none;
    border-radius: 10px;
    box-shadow: none;
    background-image: none;
}}

box#aurora-shell {{
    background-color: {COLOR_BG};
    border-radius: 10px;
}}

scrolledwindow#aurora-page-scroll,
scrolledwindow#aurora-page-scroll viewport {{
    background-color: {COLOR_BG};
    border: none;
    border-radius: 0 0 9px 0;
}}

scrolledwindow#aurora-sidebar-scroll,
scrolledwindow#aurora-sidebar-scroll viewport,
list#aurora-sidebar-list {{
    background-color: {COLOR_CARD};
    border-radius: 0 0 0 9px;
}}

box#aurora-titlebar {{
    background-color: {COLOR_CARD};
    background-image: none;
    color: {COLOR_TEXT};
    border: none;
    border-bottom: 1px solid rgba(255, 255, 255, 0.06);
    box-shadow: none;
    padding: 4px 8px;
    border-radius: 9px 9px 0 0;
}}

box#aurora-titlebar button {{
    min-width: 42px;
    min-height: 32px;
    padding: 0;
    border: none;
    border-radius: 7px;
    background-color: transparent;
    color: {COLOR_TEXT_MUTED};
}}

box#aurora-titlebar button:focus,
box#aurora-titlebar button:focus-visible {{
    outline: none;
    box-shadow: none;
    border-color: transparent;
}}

box#aurora-titlebar button:hover {{
    background-color: rgba(255, 255, 255, 0.10);
    background-image: none;
    box-shadow: none;
    border-color: transparent;
    color: {COLOR_TEXT};
}}

box#aurora-titlebar button:active {{
    background-color: rgba(255, 255, 255, 0.16);
    background-image: none;
    box-shadow: none;
    border-color: transparent;
    color: {COLOR_TEXT};
}}

box#aurora-titlebar button.close:hover {{
    background-color: rgba(255, 255, 255, 0.10);
}}

window.maximized,
window.fullscreen,
window.maximized decoration,
window.fullscreen decoration,
window.maximized headerbar,
window.fullscreen headerbar,
window.maximized box#aurora-shell,
window.fullscreen box#aurora-shell,
window.maximized scrolledwindow#aurora-page-scroll,
window.maximized scrolledwindow#aurora-page-scroll viewport,
window.fullscreen scrolledwindow#aurora-page-scroll,
window.fullscreen scrolledwindow#aurora-page-scroll viewport,
window.maximized scrolledwindow#aurora-sidebar-scroll,
window.maximized scrolledwindow#aurora-sidebar-scroll viewport,
window.maximized list#aurora-sidebar-list,
window.fullscreen scrolledwindow#aurora-sidebar-scroll,
window.fullscreen scrolledwindow#aurora-sidebar-scroll viewport,
window.fullscreen list#aurora-sidebar-list {{
    border-radius: 0;
    box-shadow: none;
}}

label#aurora-section-title {{
    color: #94A3B8;
    font-size: 11pt;
    font-weight: 700;
}}

headerbar windowcontrols {{
    padding: 0;
    background-color: transparent;
    background-image: none;
    border: none;
    box-shadow: none;
}}

headerbar button.sidebar-toggle,
headerbar button.titlebutton,
headerbar windowcontrols button.titlebutton,
headerbar .titlebutton {{
    min-width: 32px;
    min-height: 32px;
    padding: 0;
    background-color: transparent;
    background-image: none;
    border: none;
    border-radius: 6px;
    box-shadow: none;
    color: {COLOR_TEXT_MUTED};
    outline: none;
}}

headerbar button.sidebar-toggle {{
    min-width: 40px;
    min-height: 32px;
    border-radius: 8px;
}}

headerbar button.sidebar-toggle:hover,
headerbar button.titlebutton:hover,
headerbar windowcontrols button.titlebutton:hover,
headerbar .titlebutton:hover {{
    background-color: rgba(255, 255, 255, 0.08);
    color: {COLOR_TEXT};
}}

headerbar button.titlebutton:active,
headerbar windowcontrols button.titlebutton:active {{
    background-color: rgba(255, 255, 255, 0.14);
    background-image: none;
    box-shadow: none;
}}

stacksidebar, list.sidebar {{
    background-color: {COLOR_CARD};
    border-right: 1px solid rgba(255, 255, 255, 0.06);
    padding-top: 8px;
}}

/* Filas estándar de la barra lateral: borde grueso a la izquierda en color gris */
list.sidebar row {{
    background-color: transparent;
    background-image: none;
    box-shadow: none;
    border: none;
    border-left: 3px solid #3E4656;
    border-radius: 4px;
    margin: 2px 8px;
    padding: 2px 0;
}}

/* Tipografía de filas: tamaño estrictamente 10pt */
list.sidebar row label {{
    color: #94A3B8;
    font-size: 10pt;
    font-weight: 400;
}}

/* Hover en items: SIN sombreado de fondo (transparente), solo ilumina el texto y el borde */
list.sidebar row:hover {{
    background-color: transparent;
    background-image: none;
    box-shadow: none;
    border-left: 3px solid #64748B;
}}

list.sidebar row:hover label {{
    color: #FFFFFF;
    font-size: 10pt;
}}

/* Item seleccionado / activo: indicador cyan en el borde y fondo sutil */
list.sidebar row:selected {{
    background-color: rgba(86, 180, 197, 0.12);
    border-left: 3px solid {COLOR_ACCENT};
}}

list.sidebar row:selected label {{
    color: #FFFFFF;
    font-weight: 700;
    font-size: 10pt;
}}

list.sidebar row:selected:hover {{
    background-color: rgba(86, 180, 197, 0.15);
    border-left: 3px solid {COLOR_ACCENT};
}}

list.sidebar row:selected:hover label {{
    color: #FFFFFF;
    font-size: 10pt;
}}

/* Entrada principal: énfasis tipográfico sin borde adicional. */
list.sidebar row.quick-nav label,
list.sidebar row.quick-nav:hover label,
list.sidebar row.quick-nav:selected label,
list.sidebar row.quick-nav:selected:hover label {{
    font-size: 12pt;
    font-weight: 700;
}}

/* Encabezados de sección ('Flujo de trabajo', 'Configuración', 'Resultados'):
   100% estáticos: no hover, no iluminación, no selección, sin bordes ni foco */
list.sidebar row.sidebar-section-header,
list.sidebar row.sidebar-section-header:hover,
list.sidebar row.sidebar-section-header:focus,
list.sidebar row.sidebar-section-header:active,
list.sidebar row.sidebar-section-header:selected {{
    background-color: transparent;
    background-image: none;
    border: none;
    border-left: none;
    box-shadow: none;
    outline: none;
    margin: 8px 8px 2px 8px;
    padding: 0;
}}

list.sidebar row.sidebar-section-header label,
list.sidebar row.sidebar-section-header:hover label,
list.sidebar row.sidebar-section-header:focus label,
list.sidebar row.sidebar-section-header:active label,
list.sidebar row.sidebar-section-header:selected label {{
    color: #64748B;
    font-size: 8pt;
    font-weight: 700;
}}

frame {{
    margin: 8px 2px;
    background-color: {COLOR_CARD};
    border: 1px solid rgba(255, 255, 255, 0.07);
    border-radius: 8px;
}}

frame > label {{
    color: {COLOR_TEXT};
}}

frame > border {{
    border: none;
}}

label {{
    color: {COLOR_TEXT};
}}

label.dim-label {{
    color: {COLOR_TEXT_MUTED};
    opacity: 1;
}}

button {{
    background-color: #1E232E;
    background-image: none;
    box-shadow: none;
    color: {COLOR_TEXT};
    border: 1px solid #2B3342;
    border-radius: 6px;
    padding: 8px 16px;
    min-height: 30px;
    font-weight: 500;
}}

button:hover {{
    background-color: #1E232E;
    background-image: none;
    box-shadow: none;
    border-color: {COLOR_ACCENT};
    color: #FFFFFF;
}}

button:active, button:checked {{
    background-color: #14171F;
    background-image: none;
    box-shadow: none;
    border-color: {COLOR_ACCENT_DARK};
}}

button:disabled {{
    color: #555C6D;
    border-color: #20242D;
    background-color: #16181F;
}}

button.suggested-action {{
    background-color: {COLOR_ACCENT};
    background-image: none;
    color: #0A0D14;
    border: 1px solid {COLOR_ACCENT};
    border-radius: 6px;
    font-weight: 700;
}}

button.suggested-action:hover {{
    background-color: #6DC2D1;
    border-color: #6DC2D1;
}}

button.suggested-action:active {{
    background-color: {COLOR_ACCENT_DARK};
}}

scale.capture-duration-scale trough {{
    min-height: 6px;
    border-radius: 4px;
}}

scale.capture-duration-scale highlight {{
    background-color: {COLOR_ACCENT};
    background-image: none;
    border-radius: 4px;
}}

scale.capture-duration-scale slider {{
    background-color: {COLOR_ACCENT};
    background-image: none;
    border: 1px solid {COLOR_ACCENT_DARK};
    border-radius: 50%;
    padding: 0;
    min-width: 20px;
    min-height: 20px;
}}

entry {{
    background-color: #11141B;
    background-image: none;
    color: {COLOR_TEXT};
    border: 1px solid #282F3D;
    border-radius: 6px;
    padding: 7px 10px;
    min-height: 26px;
    font-family: "Cascadia Code", "Consolas", "DejaVu Sans Mono", monospace;
}}

entry:focus,
textview:focus {{
    outline: none;
    box-shadow: none;
    border: 1px solid #596579;
}}

checkbutton, radiobutton {{
    color: {COLOR_TEXT};
    min-height: 30px;
}}

/* Indicadores vectoriales y de alto contraste, independientes del tema GTK. */
checkbutton check, radiobutton radio {{
    min-width: 20px;
    min-height: 20px;
    padding: 0;
    color: transparent;
    border: 2px solid #8B95A5;
    background-color: #11141B;
    background-image: none;
    box-shadow: none;
    -gtk-icon-shadow: none;
}}

checkbutton check {{
    border-radius: 5px;
}}

radiobutton radio {{
    border-radius: 50%;
    transition: none;
    animation: none;
}}

checkbutton check:hover, radiobutton radio:hover {{
    border-color: #D5DCE7;
    background-color: #1A202B;
}}

checkbutton check:checked {{
    color: #081116;
    border-color: #7DDBE7;
    background-color: {COLOR_ACCENT};
    background-image: none;
    -gtk-icon-source: -gtk-icontheme("object-select-symbolic");
}}

radiobutton radio:checked {{
    color: #081116;
    border: 6px solid {COLOR_ACCENT};
    background-color: #11141B;
    background-image: none;
    transition: none;
    animation: none;
}}

checkbutton check:indeterminate {{
    border-color: #7DDBE7;
    background-color: {COLOR_ACCENT};
}}

checkbutton check:disabled, radiobutton radio:disabled {{
    color: transparent;
    border-color: #454C59;
    background-color: #191C24;
}}

expander title {{
    color: {COLOR_TEXT};
    font-weight: 600;
}}

.expander-content {{
    background-color: {COLOR_CARD};
    border: 1px solid rgba(255, 255, 255, 0.07);
    border-radius: 8px;
    padding: 10px 12px;
    margin-top: 6px;
}}

.capture-panel {{
    background-color: {COLOR_CARD};
    border: 1px solid #303746;
    border-radius: 15px;
    padding: 11px 14px;
}}

label.capture-panel-title {{
    color: {COLOR_TEXT};
    font-size: 10pt;
    font-weight: 700;
}}

label.capture-panel-subtitle {{
    color: {COLOR_TEXT_MUTED};
    font-size: 9pt;
}}

textview.technical-log, textview.technical-log text {{
    background-color: {COLOR_CARD};
    color: {COLOR_TEXT};
    font-family: "Cascadia Code", "Consolas", "DejaVu Sans Mono", monospace;
}}

scrolledwindow.technical-log {{
    background-color: {COLOR_CARD};
    border: none;
    border-radius: 8px;
}}

scrolledwindow, viewport {{
    background-color: {COLOR_BG};
}}

menubutton, menubutton button {{
    background-color: transparent;
    border: none;
    min-width: 32px;
    min-height: 32px;
    padding: 4px;
}}

menubutton:hover {{
    background-color: rgba(255, 255, 255, 0.06);
}}

popover {{
    background-color: {COLOR_CARD};
    border: 1px solid rgba(255, 255, 255, 0.08);
    color: {COLOR_TEXT};
    border-radius: 8px;
}}

popover contents {{
    background-color: {COLOR_CARD};
    color: {COLOR_TEXT};
    border-radius: 8px;
}}

.kpi-card {{
    background-color: #151821;
    border: 1px solid rgba(255, 255, 255, 0.07);
    border-radius: 8px;
    padding: 12px 18px;
    min-width: 120px;
}}

.kpi-value {{
    font-family: "Cascadia Code", "Consolas", "DejaVu Sans Mono", monospace;
}}

.status-bar {{
    background-color: #12141A;
    border-top: 1px solid rgba(255, 255, 255, 0.06);
}}

box.calculation-indicator {{
    background-color: #171B23;
    border: 1px solid #303A4B;
    border-radius: 8px;
    padding: 8px 10px;
}}

/* GTK themes draw a heavy blue focus halo by default; keep focus subtle across all pages. */
*:focus {{
    outline: none;
    box-shadow: none;
}}
""".encode("utf-8")


class AuroraGUI:
    def __init__(self, window: Gtk.Window) -> None:
        self.window = window
        self.window.set_default_size(1440, 960)
        self.window.set_size_request(1120, 760)
        self.window.connect("destroy", self._on_close)

        # No precargar resultados antiguos: el operador debe elegir el par que
        # corresponde a esta medicion y evitar comparar capturas de sesiones distintas.
        self.base_path = ""
        self.updated_path = ""
        self.output_dir = str(PROJECT_ROOT / "output")
        config_home = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config"))
        self._last_file_dialog_dir_file = config_home / "aurora" / "last_file_dialog_dir"
        self._last_file_dialog_dir: Path | None = None
        try:
            saved_dir = Path(self._last_file_dialog_dir_file.read_text(encoding="utf-8").strip())
            if saved_dir.is_dir():
                self._last_file_dialog_dir = saved_dir
        except (OSError, ValueError):
            pass

        self.sensor_connection = None
        self.capture_stop_event: threading.Event | None = None
        self.last_base_capture_path: str | None = None
        self.last_updated_capture_path: str | None = None
        self.result = None
        self.raycast_result = None
        self._active_result_kind: str | None = None
        self._result_generation = 0
        self.alignment_applied = False
        self._pose_alignment_original_path: str | None = None
        self._pose_alignment_aligned_path: str | None = None
        self.viewer: LiveViewer | None = None
        self.live_viewer_process: subprocess.Popen | None = None
        self.live_view_temp_dir: Path | None = None
        self.pose_viewer: PoseAlignmentViewer | None = None
        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.worker_thread: threading.Thread | None = None
        self.external_viewer_processes: list[subprocess.Popen] = []
        self.test_crop_preview_process: subprocess.Popen | None = None
        self.test_crop_preview_temp_dir: Path | None = None
        self.calculation_method = "cloud_to_cloud"
        self._progress_timer_source: int | None = None
        self._progress_started_at = 0.0
        self._progress_stage = ""
        self._capture_progress_started_at = 0.0
        self._capture_progress_stage = ""
        self._capture_progress_duration_s: float | None = None
        self._capture_progress_timer: int | None = None
        self._live_view_monitor_source: int | None = None
        self._log_poll_source: int | None = None
        self._syncing_capture_live_filters = False

        self._build_layout()
        self._log_poll_source = GLib.timeout_add(150, self._poll_log_queue)

    # ------------------------------------------------------------------ UI

    def _build_layout(self) -> None:
        self.window.set_decorated(False)
        self.window.set_app_paintable(True)
        self.window.connect_after("draw", self._draw_rounded_window_corners)
        titlebar = Gtk.EventBox()
        titlebar.set_visible_window(False)
        titlebar.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        titlebar.connect("button-press-event", self._move_window_from_titlebar)
        titlebar_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        titlebar_box.set_name("aurora-titlebar")
        titlebar_box.set_size_request(-1, 46)
        titlebar.add(titlebar_box)

        self.sidebar_toggle_button = Gtk.Button()
        self.sidebar_toggle_button.get_style_context().add_class("sidebar-toggle")
        self.sidebar_toggle_button.set_relief(Gtk.ReliefStyle.NONE)
        menu_icon = Gtk.DrawingArea()
        menu_icon.set_size_request(20, 20)
        menu_icon.set_halign(Gtk.Align.CENTER)
        menu_icon.set_valign(Gtk.Align.CENTER)
        menu_icon.connect("draw", self._draw_sidebar_menu_icon)
        self.sidebar_toggle_button.add(menu_icon)
        self.sidebar_toggle_button.set_tooltip_text("Ocultar menú lateral")
        self.sidebar_toggle_button.get_accessible().set_name("Mostrar u ocultar menú lateral")
        self.sidebar_toggle_button.connect("clicked", lambda _button: self._toggle_sidebar())
        titlebar_box.pack_start(self.sidebar_toggle_button, False, False, 0)
        self.section_title = Gtk.Label(label="Medición rápida")
        self.section_title.set_hexpand(True)
        self.section_title.set_halign(Gtk.Align.CENTER)
        self.section_title.set_name("aurora-section-title")
        titlebar_box.pack_start(self.section_title, True, True, 0)
        for icon_name, tooltip, callback, style_class in (
            ("window-minimize-symbolic", "Minimizar", lambda: self.window.iconify(), ""),
            ("window-maximize-symbolic", "Maximizar o restaurar", self._toggle_maximize, ""),
            ("window-close-symbolic", "Cerrar", self.window.close, "close"),
        ):
            button = Gtk.Button()
            button.set_image(Gtk.Image.new_from_icon_name(icon_name, Gtk.IconSize.BUTTON))
            button.set_tooltip_text(tooltip)
            if style_class:
                button.get_style_context().add_class(style_class)
            button.connect("clicked", lambda _button, cb=callback: cb())
            titlebar_box.pack_end(button, False, False, 0)
        root_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        root_box.set_name("aurora-shell")
        root_box.pack_start(titlebar, False, False, 0)
        self.window.add(root_box)

        content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        content_box.set_vexpand(True)
        root_box.pack_start(content_box, True, True, 0)

        self.stack = Gtk.Stack()
        self.stack.set_name("aurora-content")
        self.stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self.stack.set_transition_duration(150)
        self.stack.connect("notify::visible-child-name", self._update_section_title)

        self.stack.add_titled(self._build_quick_measurement_page(), "medicion_rapida", "Medicion rapida")
        self.stack.add_titled(self._build_capture_page(), "captura", "Captura")
        self.stack.add_titled(self._build_imu_alignment_page(), "alineacion_imu", "Guia de pose IMU")
        self.stack.add_titled(self._build_alignment_page(), "alineacion", "Alineacion")
        self.analysis_settings_widget = self._build_analysis_controls()
        self.stack.add_titled(self._build_raycast_test_page(), "prueba_raycasting", "Raycasting (prueba)")
        self.stack.add_titled(self._build_configuration_page(), "configuracion", "Parametros avanzados")
        self.stack.add_titled(self._build_results_page(), "resultados", "Resultados")
        self.stack.add_titled(self._build_embedded_test_page(), "prueba_embebida", "Comparacion (prueba)")
        for page_name in (
            "medicion_rapida",
            "captura",
            "alineacion_imu",
            "alineacion",
            "configuracion",
            "resultados",
            "prueba_embebida",
            "prueba_raycasting",
        ):
            page = self.stack.get_child_by_name(page_name)
            if page is not None:
                page.set_name("aurora-page-scroll")

        self.sidebar = self._build_sidebar()
        self.sidebar_revealer = Gtk.Revealer()
        self.sidebar_revealer.set_transition_type(Gtk.RevealerTransitionType.SLIDE_LEFT)
        self.sidebar_revealer.set_transition_duration(180)
        self.sidebar_revealer.set_reveal_child(True)
        self.sidebar_revealer.add(self.sidebar)
        content_box.pack_start(self.sidebar_revealer, False, False, 0)
        content_box.pack_start(self.stack, True, True, 0)

    @staticmethod
    def _draw_sidebar_menu_icon(_widget, cr) -> bool:
        cr.set_source_rgba(0.88, 0.92, 0.96, 1.0)
        cr.set_line_width(1.8)
        cr.set_line_cap(1)  # cairo.LINE_CAP_ROUND
        for y in (5, 10, 15):
            cr.move_to(2, y)
            cr.line_to(18, y)
        cr.stroke()
        return False

    @staticmethod
    def _draw_rounded_window_corners(widget, cr) -> bool:
        allocation = widget.get_allocation()
        width, height = allocation.width, allocation.height
        radius = min(10.0, width / 2.0, height / 2.0)
        if width <= 0 or height <= 0 or radius <= 0:
            return False
        cr.save()
        cr.set_operator(cairo.OPERATOR_CLEAR)
        cr.set_fill_rule(cairo.FILL_RULE_EVEN_ODD)
        cr.new_path()
        cr.rectangle(0, 0, width, height)
        cr.new_sub_path()
        cr.move_to(radius, 0)
        cr.line_to(width - radius, 0)
        cr.arc(width - radius, radius, radius, -math.pi / 2, 0)
        cr.line_to(width, height - radius)
        cr.arc(width - radius, height - radius, radius, 0, math.pi / 2)
        cr.line_to(radius, height)
        cr.arc(radius, height - radius, radius, math.pi / 2, math.pi)
        cr.line_to(0, radius)
        cr.arc(radius, radius, radius, math.pi, 3 * math.pi / 2)
        cr.close_path()
        cr.fill()
        cr.restore()
        return False

    def _toggle_maximize(self) -> None:
        if self.window.is_maximized():
            self.window.unmaximize()
        else:
            self.window.maximize()

    def _move_window_from_titlebar(self, _widget, event) -> bool:
        if event.type == Gdk.EventType.BUTTON_PRESS and event.button == 1:
            if event.get_click_count()[1] == 2:
                self._toggle_maximize()
            else:
                self.window.begin_move_drag(
                    event.button, int(event.x_root), int(event.y_root), event.time
                )
            return True
        return False

    def _update_section_title(self, stack, _pspec) -> None:
        child = stack.get_visible_child()
        if child is not None:
            self.section_title.set_text((stack.child_get_property(child, "title") or "").upper())

    def _toggle_sidebar(self) -> None:
        is_visible = self.sidebar_revealer.get_reveal_child()
        self.sidebar_revealer.set_reveal_child(not is_visible)
        self.sidebar_toggle_button.set_tooltip_text(
            "Mostrar menú lateral" if is_visible else "Ocultar menú lateral"
        )

    def _build_sidebar(self) -> Gtk.Widget:
        """Barra lateral propia (en vez de Gtk.StackSidebar) para poder agrupar las
        pestanas por proposito: flujo de trabajo (pasos secuenciales, algunos
        opcionales) vs. configuracion (no son pasos, ajustan como se calcula/ve el
        resultado) vs. experimental."""
        listbox = Gtk.ListBox()
        listbox.set_selection_mode(Gtk.SelectionMode.SINGLE)
        listbox.get_style_context().add_class("sidebar")
        self._sidebar_rows: dict[str, Gtk.ListBoxRow] = {}

        def add_header(text: str) -> None:
            row = Gtk.ListBoxRow()
            row.set_selectable(False)
            row.set_activatable(False)
            row.set_can_focus(False)
            row.get_style_context().add_class("sidebar-section-header")
            label = Gtk.Label(xalign=0)
            label.set_text(text.upper())
            label.set_margin_start(16)
            label.set_margin_top(8)
            label.set_margin_bottom(2)
            row.add(label)
            listbox.add(row)

        def add_page(title: str, stack_name: str) -> None:
            row = Gtk.ListBoxRow()
            row.set_can_focus(False)
            if stack_name == "medicion_rapida":
                row.get_style_context().add_class("quick-nav")
            label = Gtk.Label(label=title, xalign=0)
            label.set_margin_start(16)
            label.set_margin_end(12)
            label.set_margin_top(7)
            label.set_margin_bottom(7)
            row.add(label)
            row.stack_name = stack_name
            listbox.add(row)
            self._sidebar_rows[stack_name] = row

        add_page("Medicion rapida", "medicion_rapida")

        add_header("Flujo de trabajo")
        add_page("Captura", "captura")
        add_page("Guia de pose IMU (reposicionar sensor)", "alineacion_imu")
        add_page("Alineacion por puntos (respaldo)", "alineacion")

        add_header("Configuracion")
        add_page("Parametros avanzados", "configuracion")

        add_header("Resultados")
        add_page("Resultados del calculo", "resultados")

        def on_row_selected(_listbox, row) -> None:
            if row is not None and getattr(row, "stack_name", None):
                self.stack.set_visible_child_name(row.stack_name)

        listbox.connect("row-selected", on_row_selected)
        listbox.select_row(self._sidebar_rows["medicion_rapida"])

        def on_stack_page_changed(stack: Gtk.Stack, _pspec) -> None:
            name = stack.get_visible_child_name()
            row = self._sidebar_rows.get(name)
            if row is not None and listbox.get_selected_row() is not row:
                listbox.select_row(row)
            if name == "alineacion_imu":
                self._refresh_imu_reference()
                self._start_imu_poll()
            else:
                self._stop_imu_poll()
            if name == "prueba_raycasting":
                self._set_path_label(self.raycast_base_path_label, self.base_path)
                self._set_path_label(self.raycast_updated_path_label, self.updated_path)

        self.stack.connect("notify::visible-child-name", on_stack_page_changed)

        scroller = Gtk.ScrolledWindow()
        scroller.set_name("aurora-sidebar-scroll")
        scroller.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        listbox.set_name("aurora-sidebar-list")
        scroller.set_size_request(240, -1)
        scroller.add(listbox)
        return scroller

    def _build_results_page(self) -> Gtk.Widget:
        page = self._new_page()

        status_frame, status_box = self._section("Estado del analisis")
        status_frame.get_style_context().add_class("status-bar")
        status_row = self._row(status_box)
        self.status_label = Gtk.Label(label="Todavia no se calculo ningun resultado.", xalign=0)
        self.status_label.set_hexpand(True)
        status_row.pack_start(self.status_label, True, True, 0)
        self.generate_report_button = Gtk.Button(label="Generar informe PDF")
        self.generate_report_button.get_style_context().add_class("suggested-action")
        self.generate_report_button.set_tooltip_text(
            "Exporta las estadisticas, umbrales y archivos de la ultima medicion a un PDF."
        )
        self.generate_report_button.set_sensitive(False)
        self.generate_report_button.connect("clicked", lambda _b: self._generate_report())
        status_row.pack_end(self.generate_report_button, False, False, 0)
        self.calculation_indicator = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        self.calculation_indicator.get_style_context().add_class("calculation-indicator")
        self.calculation_indicator.set_no_show_all(True)
        self.calculation_spinner = Gtk.Spinner()
        self.calculation_spinner.set_size_request(20, 20)
        self.calculation_indicator.pack_start(self.calculation_spinner, False, False, 0)
        self.calculation_label = Gtk.Label(label="")
        self.calculation_label.set_xalign(0)
        self.calculation_label.set_hexpand(True)
        self.calculation_indicator.pack_start(self.calculation_label, True, True, 0)
        self.calculation_indicator.hide()
        status_box.pack_start(self.calculation_indicator, False, True, 0)
        page.pack_start(status_frame, False, True, 0)

        stats_frame, stats_box = self._section("Estadisticas de espesor")
        self.cards_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=24)
        self.cards_row.set_halign(Gtk.Align.CENTER)
        self.cards_row.set_margin_top(8)
        self.cards_row.set_margin_bottom(4)
        self.cards_row.set_no_show_all(True)
        card, self.stat_points = self._stat_card("Puntos analizados")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_median = self._stat_card("Espesor mediano")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_mean = self._stat_card("Espesor promedio")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_std = self._stat_card("Desviación estándar")
        self.cards_row.pack_start(card, False, False, 0)
        stats_box.pack_start(self.cards_row, False, False, 0)
        self.thickness_alert_label = Gtk.Label(xalign=0)
        self.thickness_alert_label.set_line_wrap(True)
        self.thickness_alert_label.set_margin_top(8)
        self.thickness_alert_label.set_margin_bottom(8)
        self.thickness_alert_label.set_no_show_all(True)
        self.thickness_alert_label.hide()
        stats_box.pack_start(self.thickness_alert_label, False, True, 0)
        self.results_empty_label = Gtk.Label(
            label="Al terminar una medicion, aqui apareceran las estadisticas y el acceso al informe.",
            xalign=0,
        )
        self.results_empty_label.set_line_wrap(True)
        stats_box.pack_start(self.results_empty_label, False, False, 4)
        self.open_result_viewer_button = Gtk.Button(label="Abrir vista 3D")
        self.open_result_viewer_button.set_sensitive(False)
        self.open_result_viewer_button.connect("clicked", lambda _b: self._open_active_result_visualization())
        stats_box.pack_start(self.open_result_viewer_button, False, False, 4)
        page.pack_start(stats_frame, False, True, 0)

        method_frame, method_box = self._section("Metodo y parametros principales")
        self.results_method_box = method_box
        method_row = self._row(method_box)
        self.result_method_cloud_rb = Gtk.RadioButton.new_with_label_from_widget(None, "Cloud-to-Cloud")
        self.result_method_mesh_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.result_method_cloud_rb, "Malla / Raycasting"
        )
        self.result_method_cloud_rb.connect(
            "toggled", self._on_calculation_method_radio_toggled, "cloud_to_cloud"
        )
        self.result_method_mesh_rb.connect(
            "toggled", self._on_calculation_method_radio_toggled, "raycast"
        )
        method_row.pack_start(self.result_method_cloud_rb, False, False, 0)
        method_row.pack_start(self.result_method_mesh_rb, False, False, 0)

        self.quick_c2c_frame, quick_c2c_box = self._section("Configuracion rapida Cloud-to-Cloud")
        self.quick_c2c_frame.set_no_show_all(True)
        for quick_row in (self.c2c_voxel_row, self.c2c_outliers_row):
            quick_row.get_parent().remove(quick_row)
            quick_c2c_box.pack_start(quick_row, False, False, 0)
        self.quick_c2c_frame.hide()
        method_box.pack_start(self.quick_c2c_frame, False, True, 0)

        self.quick_mesh_method_row.get_parent().remove(self.quick_mesh_method_row)
        self.quick_mesh_method_row.set_no_show_all(True)
        self.quick_mesh_method_row.hide()
        method_box.pack_start(self.quick_mesh_method_row, False, False, 0)
        self.quick_mesh_voxel_row.get_parent().remove(self.quick_mesh_voxel_row)
        self.quick_mesh_voxel_row.set_no_show_all(True)
        self.quick_mesh_voxel_row.hide()
        method_box.pack_start(self.quick_mesh_voxel_row, False, False, 0)
        for mesh_parameter_row in (self.quick_delaunay_params_row, self.quick_poisson_params_row):
            mesh_parameter_row.get_parent().remove(mesh_parameter_row)
            mesh_parameter_row.set_no_show_all(True)
            mesh_parameter_row.hide()
            method_box.pack_start(mesh_parameter_row, False, False, 0)

        self.results_ray_method_row = self._row(method_box)
        self.results_ray_method_row.pack_start(Gtk.Label(label="Direccion de medicion:"), False, False, 0)
        self.results_ray_method_combo = Gtk.ComboBoxText()
        self.results_ray_method_combo.append("normal", "Normal a la superficie")
        self.results_ray_method_combo.append("origin", "Desde el origen del sensor")
        self.results_ray_method_combo.set_active_id("normal")
        self.results_ray_method_combo.connect("changed", lambda _combo: self._on_results_ray_method_changed())
        self.results_ray_method_row.pack_start(self.results_ray_method_combo, False, False, 0)
        self.results_ray_method_row.set_no_show_all(True)
        self.results_ray_method_row.hide()

        self.results_manual_origin_row = self._row(method_box)
        self.results_manual_origin_row.pack_start(Gtk.Label(label="Origen ANTES (x y z, m):"), False, False, 0)
        self.results_base_origin_entry = Gtk.Entry()
        self.results_base_origin_entry.set_text("0 0 0")
        self.results_base_origin_entry.set_width_chars(18)
        self.results_manual_origin_row.pack_start(self.results_base_origin_entry, False, False, 0)
        self.results_manual_origin_row.pack_start(Gtk.Label(label="DESPUES:"), False, False, 0)
        self.results_updated_origin_entry = Gtk.Entry()
        self.results_updated_origin_entry.set_text("0 0 0")
        self.results_updated_origin_entry.set_width_chars(18)
        self.results_manual_origin_row.pack_start(self.results_updated_origin_entry, False, False, 0)
        self.results_manual_origin_row.set_no_show_all(True)
        self.results_manual_origin_row.hide()

        action_row = self._row(method_box)
        advanced_button = Gtk.Button(label="Opciones avanzadas")
        advanced_button.connect("clicked", lambda _b: self.stack.set_visible_child_name("configuracion"))
        action_row.pack_start(advanced_button, False, False, 0)
        self.result_recalculate_button = Gtk.Button(label="Recalcular espesor")
        self.result_recalculate_button.get_style_context().add_class("suggested-action")
        self.result_recalculate_button.set_sensitive(False)
        self.result_recalculate_button.connect("clicked", lambda _b: self._recalculate_from_results())
        action_row.pack_start(self.result_recalculate_button, False, False, 0)
        page.pack_start(method_frame, False, True, 0)
        self._on_results_method_changed()

        log_frame, log_box = self._section("Registro del calculo")
        log_expander = Gtk.Expander(label="Detalles tecnicos")
        log_expander.set_expanded(False)
        log_scroller = Gtk.ScrolledWindow()
        log_scroller.set_min_content_height(220)
        log_scroller.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.log_view = Gtk.TextView()
        self.log_view.get_style_context().add_class("technical-log")
        self.log_view.set_editable(False)
        self.log_view.set_monospace(True)
        self.log_view.set_left_margin(6)
        self.log_view.set_right_margin(6)
        self.log_view.set_top_margin(6)
        self.log_view.set_bottom_margin(6)
        self.log_view.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self.log_buffer = self.log_view.get_buffer()
        log_scroller.get_style_context().add_class("technical-log")
        log_scroller.add(self.log_view)
        log_expander.add(log_scroller)
        log_box.pack_start(log_expander, False, True, 0)
        page.pack_start(log_frame, False, True, 0)

        return self._scrolled(page)

    def _build_configuration_page(self) -> Gtk.Widget:
        page = self._new_page()
        advanced_frame, advanced_box = self._section("Parametros avanzados de calculo")
        advanced_box.pack_start(self.analysis_settings_widget, False, True, 0)
        advanced_box.pack_start(self.quick_mesh_expander, False, True, 0)
        page.pack_start(advanced_frame, False, True, 0)
        self._build_visualization_controls(page)
        return self._scrolled(page)

    def _build_quick_measurement_page(self) -> Gtk.Widget:
        """Flujo guiado de entrada para operadores nuevos; delega en los flujos existentes."""
        page = self._new_page()

        files_frame, files_box = self._section("1. Sensor Aurora y nubes de entrada")
        sensor_row = self._row(files_box)
        sensor_row.pack_start(Gtk.Label(label="Sensor · IP:", xalign=0), False, False, 0)
        self.quick_sensor_address_entry = Gtk.Entry()
        self.quick_sensor_address_entry.set_text("192.168.11.1")
        self.quick_sensor_address_entry.set_width_chars(16)
        self.quick_sensor_address_entry.connect("changed", self._sync_sensor_address)
        sensor_row.pack_start(self.quick_sensor_address_entry, False, False, 0)
        self.quick_connect_button = Gtk.Button(label="Conectar sensor")
        self.quick_connect_button.connect("clicked", lambda _b: self._toggle_sensor_connection())
        sensor_row.pack_start(self.quick_connect_button, False, False, 0)
        self.quick_sensor_status_label = Gtk.Label()
        self.quick_sensor_status_label.set_markup(f'<span foreground="{COLOR_ERROR}">●</span>  Desconectado')
        sensor_row.pack_start(self.quick_sensor_status_label, False, False, 0)

        self.quick_base_path_label = Gtk.Label(xalign=0)
        self.quick_base_path_label.set_hexpand(True)
        self._set_path_label(self.quick_base_path_label, self.base_path)
        row = self._row(files_box)
        row.pack_start(Gtk.Label(label="ANTES  ·  tunel original", xalign=0), False, False, 0)
        row.pack_start(self.quick_base_path_label, True, True, 0)
        load_base = Gtk.Button(label="Cargar .ply")
        load_base.connect("clicked", lambda _b: self._quick_choose_cloud("base"))
        row.pack_start(load_base, False, False, 0)
        capture_base = Gtk.Button(label="Capturar antes")
        capture_base.connect("clicked", lambda _b: self._quick_capture("base"))
        self.quick_capture_base_button = capture_base
        capture_base.set_sensitive(self.sensor_connection is not None)
        row.pack_start(capture_base, False, False, 0)
        self.quick_view_base_button = Gtk.Button(label="Ver nube")
        self.quick_view_base_button.set_sensitive(bool(self.base_path and Path(self.base_path).is_file()))
        self.quick_view_base_button.connect("clicked", lambda _b: self._quick_view_cloud("base"))
        row.pack_start(self.quick_view_base_button, False, False, 0)

        self.quick_updated_path_label = Gtk.Label(xalign=0)
        self.quick_updated_path_label.set_hexpand(True)
        self._set_path_label(self.quick_updated_path_label, self.updated_path)
        row = self._row(files_box)
        row.pack_start(Gtk.Label(label="DESPUES  ·  tunel con shotcrete", xalign=0), False, False, 0)
        row.pack_start(self.quick_updated_path_label, True, True, 0)
        load_updated = Gtk.Button(label="Cargar .ply")
        load_updated.connect("clicked", lambda _b: self._quick_choose_cloud("updated"))
        row.pack_start(load_updated, False, False, 0)
        capture_updated = Gtk.Button(label="Capturar despues")
        capture_updated.connect("clicked", lambda _b: self._quick_capture("updated"))
        self.quick_capture_updated_button = capture_updated
        capture_updated.set_sensitive(self.sensor_connection is not None)
        row.pack_start(capture_updated, False, False, 0)
        self.quick_view_updated_button = Gtk.Button(label="Ver nube")
        self.quick_view_updated_button.set_sensitive(bool(self.updated_path and Path(self.updated_path).is_file()))
        self.quick_view_updated_button.connect("clicked", lambda _b: self._quick_view_cloud("updated"))
        row.pack_start(self.quick_view_updated_button, False, False, 0)
        capture_status_row = self._row(files_box)
        self.quick_capture_spinner = Gtk.Spinner()
        self.quick_capture_spinner.set_size_request(18, 18)
        self.quick_capture_spinner.set_no_show_all(True)
        self.quick_capture_spinner.hide()
        capture_status_row.pack_start(self.quick_capture_spinner, False, False, 0)
        self.quick_capture_status_label = Gtk.Label(label="", xalign=0)
        self.quick_capture_status_label.set_hexpand(True)
        capture_status_row.pack_start(self.quick_capture_status_label, True, True, 0)
        capture_time_row = self._row(files_box)
        capture_time_row.pack_start(Gtk.Label(label="Tiempo de captura:"), False, False, 0)
        self.quick_capture_duration_scale = Gtk.Scale.new_with_range(
            Gtk.Orientation.HORIZONTAL, 1.0, 30.0, 1.0
        )
        self.quick_capture_duration_scale.set_value(3.0)
        self.quick_capture_duration_scale.set_digits(0)
        self.quick_capture_duration_scale.set_draw_value(True)
        self.quick_capture_duration_scale.set_value_pos(Gtk.PositionType.RIGHT)
        self.quick_capture_duration_scale.set_hexpand(True)
        self.quick_capture_duration_scale.set_size_request(260, -1)
        self.quick_capture_duration_scale.get_style_context().add_class(
            "capture-duration-scale"
        )
        for seconds in (1, 5, 10, 15, 20, 25, 30):
            self.quick_capture_duration_scale.add_mark(
                seconds, Gtk.PositionType.BOTTOM, str(seconds)
            )
        self.quick_capture_duration_scale.set_tooltip_text(
            "Tiempo mínimo de adquisición (1–30 s). En modo normal estabiliza una vista; "
            "con Captura móvil alineada registra y fusiona frames con la pose SLAM."
        )
        capture_time_row.pack_start(self.quick_capture_duration_scale, True, True, 0)
        capture_time_row.pack_start(Gtk.Label(label="segundos"), False, False, 0)
        self.quick_aligned_capture_check = Gtk.CheckButton(
            label="Captura móvil alineada (fusionar frames con pose SLAM)"
        )
        self.quick_aligned_capture_check.set_tooltip_text(
            "Actívala para mover el Aurora lentamente alrededor de la zona durante la captura. "
            "Cada frame se transforma con su pose SLAM y se fusiona en el marco del primer frame. "
            "Requiere tracking continuo; los saltos grandes se descartan."
        )
        capture_align_row = self._row(files_box)
        capture_align_row.pack_start(self.quick_aligned_capture_check, False, False, 0)
        imu_row = self._row(files_box)
        self.quick_imu_compensate_check = Gtk.CheckButton(
            label="Compensar automaticamente el movimiento del sensor con IMU/SLAM"
        )
        self.quick_imu_compensate_check.set_active(True)
        self.quick_imu_compensate_check.set_tooltip_text(
            "Si ambas capturas tienen pose guardada y el sensor se movio, transforma DESPUES "
            "al marco de ANTES antes de calcular. Requiere que el tracking conserve el mismo mapa."
        )
        imu_row.pack_start(self.quick_imu_compensate_check, False, False, 0)
        self.quick_imu_status_label = Gtk.Label(
            label="La compensacion se activara si ambas capturas contienen pose del sensor.",
            xalign=0,
        )
        self.quick_imu_status_label.set_line_wrap(True)
        self.quick_imu_status_label.set_hexpand(True)
        imu_row.pack_start(self.quick_imu_status_label, True, True, 0)
        note = Gtk.Label(
            label=(
                "Puedes mover el sensor entre capturas si el tracking sigue en el mismo mapa. "
                "Con la casilla activada, Medicion rapida corrige automaticamente el cambio de pose antes de calcular."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        files_box.pack_start(note, False, False, 0)
        page.pack_start(files_frame, False, True, 0)

        crop_frame, crop_box = self._section("2. Preparar nubes")
        crop_note = Gtk.Label(
            label=(
                "Define la region de trabajo en ambas capturas. La deteccion automatica viene seleccionada; "
                "tambien puedes elegir un punto de la pared."
            ),
            xalign=0,
        )
        crop_note.set_line_wrap(True)
        crop_box.pack_start(crop_note, False, False, 0)
        crop_row = self._row(crop_box)
        self.quick_crop_auto_radio = Gtk.RadioButton.new_with_label_from_widget(
            None, "Automatico"
        )
        self.quick_crop_manual_radio = Gtk.RadioButton.new_with_label_from_widget(
            self.quick_crop_auto_radio, "Elegir 1 punto"
        )
        self.quick_crop_auto_radio.set_active(True)
        self.quick_crop_auto_radio.connect("toggled", self._on_quick_crop_mode_changed)
        crop_row.pack_start(self.quick_crop_auto_radio, False, False, 0)
        crop_row.pack_start(self.quick_crop_manual_radio, False, False, 0)
        self.quick_crop_action_button = Gtk.Button(label="Detectar pared")
        self.quick_crop_action_button.set_tooltip_text(
            "Detecta automaticamente la pared frontal o permite seleccionar un punto manualmente."
        )
        self.quick_crop_action_button.connect("clicked", lambda _b: self._run_quick_crop_selection())
        self.quick_crop_action_button.set_sensitive(False)
        crop_row.pack_start(self.quick_crop_action_button, False, False, 0)
        self.quick_crop_preview_button = Gtk.Button(label="Vista previa")
        self.quick_crop_preview_button.connect("clicked", lambda _b: self._preview_test_crop_bounds())
        self.quick_crop_preview_button.set_sensitive(False)
        crop_row.pack_start(self.quick_crop_preview_button, False, False, 0)
        self.quick_crop_apply_button = Gtk.Button(label="Guardar recortes y calcular")
        self.quick_crop_apply_button.get_style_context().add_class("suggested-action")
        self.quick_crop_apply_button.set_tooltip_text(
            "Guarda ANTES y DESPUÉS recortados en data/recortes, conserva intactos los originales y calcula el espesor."
        )
        self.quick_crop_apply_button.connect(
            "clicked", lambda _b: self._apply_test_crop(
                overwrite_inputs=False, calculate_after=True
            )
        )
        self.quick_crop_apply_button.set_sensitive(False)
        crop_row.pack_start(self.quick_crop_apply_button, False, False, 0)
        crop_dimensions_row = self._row(crop_box)
        crop_dimensions_row.pack_start(Gtk.Label(label="Ancho (cm):"), False, False, 0)
        self.quick_crop_face_size_entry = self._numeric_entry("20", 7)
        self.quick_crop_face_size_entry.connect(
            "changed", lambda entry: self._sync_quick_crop_dimension(entry, "width")
        )
        crop_dimensions_row.pack_start(self.quick_crop_face_size_entry, False, False, 0)
        crop_dimensions_row.pack_start(Gtk.Label(label="Alto (cm):"), False, False, 0)
        self.quick_crop_height_entry = self._numeric_entry("20", 7)
        self.quick_crop_height_entry.connect(
            "changed", lambda entry: self._sync_quick_crop_dimension(entry, "height")
        )
        crop_dimensions_row.pack_start(self.quick_crop_height_entry, False, False, 0)
        crop_dimensions_row.pack_start(Gtk.Label(label="Profundidad (cm):"), False, False, 0)
        self.quick_crop_depth_entry = self._numeric_entry("100", 7)
        self.quick_crop_depth_entry.connect(
            "changed", lambda entry: self._sync_quick_crop_dimension(entry, "depth")
        )
        crop_dimensions_row.pack_start(self.quick_crop_depth_entry, False, False, 0)
        self.quick_crop_status_label = Gtk.Label(label="Carga ANTES y DESPUES para definir la region.", xalign=0)
        self.quick_crop_status_label.set_line_wrap(True)
        crop_box.pack_start(self.quick_crop_status_label, False, False, 0)
        self.quick_crop_bounds_label = Gtk.Label(label="Limites sin definir", xalign=0)
        crop_box.pack_start(self.quick_crop_bounds_label, False, False, 0)

        # The crop workflow lives here now. Keep the existing processing
        # handlers pointed at these controls so both capture paths stay in sync.
        self.test_crop_before_path = self.base_path or None
        self.test_crop_after_path = self.updated_path or None
        self.test_crop_output_paths = None
        self.test_crop_output_dir = None
        self.test_crop_output_dir_is_custom = False
        self.test_crop_bounds = None
        self.test_crop_seed = None
        self.test_crop_pick_button = self.quick_crop_action_button
        self.test_crop_auto_button = self.quick_crop_action_button
        self.test_crop_preview_button = self.quick_crop_preview_button
        self.test_crop_apply_button = self.quick_crop_apply_button
        self.test_crop_face_size_entry = self.quick_crop_face_size_entry
        self.test_crop_height_entry = self.quick_crop_height_entry
        self.test_crop_depth_spin = self.quick_crop_depth_entry
        self.test_crop_bounds_label = self.quick_crop_bounds_label
        self.test_crop_status_label = self.quick_crop_status_label
        self.test_crop_use_pair_button = Gtk.Button()
        self.test_crop_view_pair_button = Gtk.Button()
        self.quick_crop_face_size_entry.connect(
            "changed", lambda _entry: self._on_test_crop_face_size_changed()
        )
        self.quick_crop_height_entry.connect(
            "changed", lambda _entry: self._on_test_crop_face_size_changed()
        )
        self.quick_crop_depth_entry.connect(
            "changed", lambda _entry: self._on_test_crop_depth_changed()
        )
        page.pack_start(crop_frame, False, True, 0)

        # Keep pose status available to the existing pose-alignment flow without
        # taking space in the quick measurement page.
        self.quick_alignment_status = Gtk.Label(label="Alineacion no aplicada", xalign=0)

        method_frame, method_box = self._section("3. Calcula el espesor")
        method_note = Gtk.Label(
            label="Recomendado para empezar: Cloud-to-Cloud. Calcula distancias entre superficies sin reconstruir mallas.",
            xalign=0,
        )
        method_note.set_line_wrap(True)
        method_box.pack_start(method_note, False, False, 0)

        mesh_expander = Gtk.Expander(label="Opciones avanzadas de malla")
        mesh_expander.set_expanded(False)
        self.quick_mesh_expander = mesh_expander
        mesh_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        mesh_box.get_style_context().add_class("expander-content")
        self.quick_mesh_check = Gtk.CheckButton(label="Usar una malla para calcular el espesor")
        self.quick_mesh_check.connect("toggled", self._on_quick_mesh_check_toggled)
        self.quick_mesh_voxel_row = self._row(mesh_box)
        self.quick_mesh_voxel_row.pack_start(Gtk.Label(label="Resolucion de malla (voxel, m):"), False, False, 0)
        self.quick_mesh_voxel_entry = self._numeric_entry("0", 8)
        self.quick_mesh_voxel_entry.set_tooltip_text(
            "0 conserva todos los puntos capturados. Un valor mayor reduce la nube solo para "
            "reconstruir la malla; el calculo lanza rayos usando los puntos originales."
        )
        self.quick_mesh_voxel_row.pack_start(self.quick_mesh_voxel_entry, False, False, 0)
        self.quick_mesh_method_row = self._row(mesh_box)
        self.quick_mesh_method_row.pack_start(Gtk.Label(label="Algoritmo de malla:"), False, False, 0)
        self.quick_poisson_rb = Gtk.RadioButton.new_with_label_from_widget(None, "Poisson")
        self.quick_mesh_method_row.pack_start(self.quick_poisson_rb, False, False, 0)
        self.quick_delaunay_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.quick_poisson_rb, "Delaunay"
        )
        self.quick_mesh_method_row.pack_start(self.quick_delaunay_rb, False, False, 0)
        self.quick_delaunay_params_row = self._row(mesh_box)
        self.quick_delaunay_params_row.pack_start(Gtk.Label(label="Limite Delaunay:"), False, False, 0)
        self.quick_delaunay_trim = self._numeric_entry("4.0", 7)
        self.quick_delaunay_trim.set_tooltip_text(
            "Descarta triangulos demasiado largos respecto al borde tipico de la nube."
        )
        self.quick_delaunay_trim.set_sensitive(False)
        self.quick_delaunay_params_row.pack_start(self.quick_delaunay_trim, False, False, 0)
        self.quick_poisson_params_row = self._row(mesh_box)
        self.quick_poisson_params_row.pack_start(Gtk.Label(label="Profundidad Poisson:"), False, False, 0)
        self.quick_poisson_depth = self._numeric_entry("9", 7, integer=True)
        self.quick_poisson_depth.set_tooltip_text(
            "Menor profundidad genera una superficie menos detallada y puede reducir picos pequeños."
        )
        self.quick_poisson_params_row.pack_start(self.quick_poisson_depth, False, False, 0)
        mesh_advanced_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        mesh_advanced_box.get_style_context().add_class("expander-content")
        density_row = self._row(mesh_advanced_box)
        density_row.pack_start(Gtk.Label(label="Recorte de densidad Poisson:"), False, False, 0)
        self.quick_density_trim = self._numeric_entry("0.02", 7)
        self.quick_density_trim.set_tooltip_text(
            "Elimina los vertices de menor densidad de Poisson en los bordes de la superficie."
        )
        density_row.pack_start(self.quick_density_trim, False, False, 0)
        self.quick_smooth_check = Gtk.CheckButton(label="Suavizar malla (Taubin)")
        self.quick_smooth_check.set_tooltip_text(
            "Modifica la superficie usada para medir; compara el espesor con y sin suavizado."
        )
        smooth_row = self._row(mesh_advanced_box)
        smooth_row.pack_start(self.quick_smooth_check, False, False, 0)
        self.quick_smooth_iterations = self._numeric_entry("5", 7, integer=True)
        self.quick_smooth_iterations.set_sensitive(False)
        self.quick_smooth_check.connect(
            "toggled", lambda check: self.quick_smooth_iterations.set_sensitive(check.get_active())
        )
        smooth_row.pack_start(Gtk.Label(label="Iteraciones:"), False, False, 0)
        smooth_row.pack_start(self.quick_smooth_iterations, False, False, 0)
        ray_limit_row = self._row(mesh_advanced_box)
        ray_limit_row.pack_start(Gtk.Label(label="Limite de espesor (m, opcional):"), False, False, 0)
        self.quick_ray_max_distance_entry = Gtk.Entry()
        self.quick_ray_max_distance_entry.set_width_chars(8)
        self.quick_ray_max_distance_entry.set_tooltip_text(
            "Descarta impactos a distancias mayores. Dejalo vacio para no limitar."
        )
        ray_limit_row.pack_start(self.quick_ray_max_distance_entry, False, False, 0)
        mesh_explanation = Gtk.Label(
            label=(
                "En Resultados, elige Malla / Raycasting para reconstruir las superficies. Poisson crea "
                "una superficie continua; Delaunay triangula directamente los puntos. "
                "Cloud-to-Cloud mide distancias sin crear mallas. "
                "Bajar la profundidad Poisson o activar Taubin reduce irregularidades, pero "
                "tambien puede cambiar el espesor calculado; deja Taubin apagado para medir sin suavizado."
            ),
            xalign=0,
        )
        mesh_explanation.set_line_wrap(True)
        mesh_advanced_box.pack_start(mesh_explanation, False, False, 0)
        mesh_box.pack_start(mesh_advanced_box, False, False, 0)
        self.quick_delaunay_rb.connect("toggled", self._on_quick_mesh_method_toggled)
        mesh_expander.add(mesh_box)

        self.quick_calculate_button = Gtk.Button(label="Calcular espesor")
        self.quick_calculate_button.get_style_context().add_class("suggested-action")
        self.quick_calculate_button.connect("clicked", lambda _b: self._quick_calculate())
        method_box.pack_start(self.quick_calculate_button, False, False, 0)
        self.quick_status_label = Gtk.Label(label="Carga las nubes antes y despues para comenzar.", xalign=0)
        self.quick_status_label.set_line_wrap(True)
        method_box.pack_start(self.quick_status_label, False, False, 0)

        self.quick_mesh_result_row = self._row(method_box)
        self.quick_mesh_result_row.set_no_show_all(True)
        self.quick_mesh_result_button = Gtk.Button(label="Ir a resultados de malla")
        self.quick_mesh_result_button.connect(
            "clicked", lambda _b: self.stack.set_visible_child_name("resultados")
        )
        self.quick_mesh_result_row.pack_start(self.quick_mesh_result_button, False, False, 0)
        self._hide_quick_mesh_result_actions()

        extra_row = self._row(method_box)
        advanced_button = Gtk.Button(label="Parametros avanzados")
        advanced_button.connect("clicked", lambda _b: self.stack.set_visible_child_name("configuracion"))
        extra_row.pack_start(advanced_button, False, False, 0)
        page.pack_start(method_frame, False, True, 0)

        self._refresh_quick_status()
        return self._scrolled(page)

    def _quick_choose_cloud(self, target: str) -> None:
        path = self._run_open_file_dialog()
        if not path:
            return
        if target == "base":
            self._set_base_path(path)
        else:
            self._set_updated_path(path)
        self._refresh_quick_status()

    def _on_quick_crop_mode_changed(self, radio, *_args) -> None:
        if not radio.get_active():
            return
        manual = radio is self.quick_crop_manual_radio
        self.quick_crop_action_button.set_label(
            "Elegir punto en 3D" if manual else "Detectar pared"
        )
        self.quick_crop_action_button.set_tooltip_text(
            "Abre la vista 3D para elegir un punto; otro clic reemplaza la seleccion."
            if manual
            else "Busca automaticamente la pared frontal en las dos nubes."
        )

    def _run_quick_crop_selection(self) -> None:
        if self.quick_crop_manual_radio.get_active():
            self._pick_test_crop_seed()
        else:
            self._auto_detect_crop_wall()

    def _quick_capture(self, target: str) -> None:
        if self.sensor_connection is None:
            self.quick_capture_status_label.set_text("Conecta el Aurora antes de capturar.")
            return
        self._capture_clicked(target)

    def _quick_view_cloud(self, target: str) -> None:
        path = self.base_path if target == "base" else self.updated_path
        if not path or not Path(path).is_file():
            return
        stage = "ANTES · tunel original" if target == "base" else "DESPUES · tunel con shotcrete"
        self._launch_viewer_process(
            "cloud",
            [path],
            f"Aurora - {stage}",
            "No se pudo abrir la nube",
        )

    def _on_results_method_changed(self) -> None:
        use_mesh = self.calculation_method == "raycast"
        self._sync_calculation_method_radios()
        self.quick_mesh_check.set_active(use_mesh)
        self._show_control_row(self.quick_c2c_frame, not use_mesh)
        self._show_control_row(self.quick_mesh_method_row, use_mesh)
        self._show_control_row(self.quick_mesh_voxel_row, use_mesh)
        use_delaunay = self.quick_delaunay_rb.get_active()
        self._show_control_row(self.quick_delaunay_params_row, use_mesh and use_delaunay)
        self._show_control_row(self.quick_poisson_params_row, use_mesh and not use_delaunay)
        self._show_control_row(self.results_ray_method_row, use_mesh)
        self._on_results_ray_method_changed()

    def _on_quick_mesh_check_toggled(self, button) -> None:
        """Keep the quick-method checkbox and Results method radios in sync."""
        method_id = "raycast" if button.get_active() else "cloud_to_cloud"
        if self.calculation_method != method_id:
            self._set_calculation_method(method_id)

    @staticmethod
    def _show_control_row(widget: Gtk.Widget, show: bool) -> None:
        if show:
            widget.set_no_show_all(False)
            widget.show_all()
        else:
            widget.hide()
            widget.set_no_show_all(True)

    def _sync_calculation_method_radios(self) -> None:
        use_mesh = self.calculation_method == "raycast"
        for name, active in (
            ("result_method_cloud_rb", not use_mesh),
            ("result_method_mesh_rb", use_mesh),
        ):
            button = getattr(self, name, None)
            if button is not None and button.get_active() != active:
                button.set_active(active)

    def _on_calculation_method_radio_toggled(self, button, method_id: str) -> None:
        if not button.get_active() or self.calculation_method == method_id:
            return
        self.calculation_method = method_id
        self._sync_calculation_method_radios()
        self._on_results_method_changed()

    def _set_calculation_method(self, method_id: str) -> None:
        if method_id not in {"cloud_to_cloud", "raycast"}:
            return
        self.calculation_method = method_id
        self._sync_calculation_method_radios()
        self._on_results_method_changed()

    def _on_results_ray_method_changed(self) -> None:
        if not hasattr(self, "results_manual_origin_row"):
            return
        show_origin = (
            self.calculation_method == "raycast"
            and self.results_ray_method_combo.get_active_id() == "origin"
        )
        self._show_control_row(self.results_manual_origin_row, show_origin)

    def _on_quick_mesh_method_toggled(self, button) -> None:
        use_delaunay = button.get_active()
        self.quick_poisson_depth.set_sensitive(not use_delaunay)
        self.quick_density_trim.set_sensitive(not use_delaunay)
        self.quick_delaunay_trim.set_sensitive(use_delaunay)
        if hasattr(self, "quick_delaunay_params_row"):
            use_mesh = self.calculation_method == "raycast"
            self._show_control_row(self.quick_delaunay_params_row, use_mesh and use_delaunay)
            self._show_control_row(self.quick_poisson_params_row, use_mesh and not use_delaunay)

    def _recalculate_from_results(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            self._show_warning("En progreso", "Espera a que termine el calculo actual.")
            return
        if self.calculation_method == "raycast":
            self.raycast_mesh_poisson_rb.set_active(self.quick_poisson_rb.get_active())
            self.raycast_mesh_delaunay_rb.set_active(self.quick_delaunay_rb.get_active())
            self.raycast_ray_origin_rb.set_active(self.results_ray_method_combo.get_active_id() == "origin")
            self.raycast_ray_normal_rb.set_active(self.results_ray_method_combo.get_active_id() != "origin")
            self.raycast_base_origin_entry.set_text(self.results_base_origin_entry.get_text())
            self.raycast_updated_origin_entry.set_text(self.results_updated_origin_entry.get_text())
        self._quick_calculate()

    def _quick_calculate(self) -> None:
        if not self.base_path or not Path(self.base_path).is_file():
            self._show_warning("Falta la captura ANTES", "Carga un archivo .ply o captura el tunel antes del shotcrete.")
            return
        if not self.updated_path or not Path(self.updated_path).is_file():
            self._show_warning("Falta la captura DESPUES", "Carga un archivo .ply o captura el tunel con shotcrete.")
            return

        # Older app versions wrote the base pose into *_alineado_pose.ply even
        # though their transform treated camera XYZ as SLAM XYZ. Reopen only
        # legacy artifacts; current aligned/cropped files carry version 2.
        updated_file = Path(self.updated_path)
        aligned_suffix = "_alineado_pose"
        if (
            self.quick_imu_compensate_check.get_active()
            and updated_file.stem.endswith(aligned_suffix)
            and reference_pose_alignment_version(updated_file) < 2
        ):
            original_file = updated_file.with_name(
                updated_file.stem[: -len(aligned_suffix)] + updated_file.suffix
            )
            if original_file.is_file():
                self._set_updated_path(str(original_file))
                self.alignment_applied = False
                self._log(
                    f"Se recalculará la compensación desde la captura DESPUÉS original: {original_file.name}."
                )

        self._hide_quick_mesh_result_actions()

        base_pose = load_reference_pose(Path(self.base_path))
        updated_pose = load_reference_pose(Path(self.updated_path))
        if base_pose is not None and updated_pose is not None:
            position_error_cm = float(np.linalg.norm(base_pose[0] - updated_pose[0])) * 100.0
            rotation_error_deg = ((base_pose[1] - updated_pose[1] + 180.0) % 360.0) - 180.0
            max_rotation_error = float(np.max(np.abs(rotation_error_deg)))
            if (
                position_error_cm > self.IMU_POSITION_TOLERANCE_CM
                or max_rotation_error > self.IMU_ROTATION_TOLERANCE_DEG
            ):
                if self.quick_imu_compensate_check.get_active():
                    self.quick_imu_status_label.set_text(
                        f"Corrigiendo pose: {position_error_cm:.1f} cm y "
                        f"{max_rotation_error:.1f}° detectados..."
                    )
                    self.quick_status_label.set_text(
                        "Alineando automaticamente DESPUES con la pose IMU/SLAM de ANTES..."
                    )
                    self._log(
                        f"Compensacion IMU/SLAM automatica: desplazamiento {position_error_cm:.1f} cm, "
                        f"rotacion maxima {max_rotation_error:.1f}°."
                    )
                    if not self._apply_pose_alignment(interactive=False):
                        self.quick_imu_status_label.set_text(
                            "No se pudo compensar el movimiento. Revisa las poses guardadas y el tracking."
                        )
                        self._show_error(
                            "No se pudo compensar el movimiento",
                            "No se pudo transformar la nube DESPUES con la pose del sensor. "
                            "Verifica que ambos archivos tengan su _pose.npz y que el tracking sea continuo.",
                        )
                        return
                    self.alignment_applied = True
                    self.quick_imu_status_label.set_text(
                        f"Movimiento compensado: {position_error_cm:.1f} cm y "
                        f"{max_rotation_error:.1f}°. Se calculara con las nubes alineadas."
                    )
                else:
                    dialog = Gtk.MessageDialog(
                        transient_for=self.window,
                        flags=0,
                        message_type=Gtk.MessageType.QUESTION,
                        buttons=Gtk.ButtonsType.NONE,
                        text="Diferencia de posición del sensor detectada",
                    )
                    dialog.format_secondary_text(
                        f"El sensor se desplazo {position_error_cm:.1f} cm y roto {max_rotation_error:.1f}° entre ambas tomas.\n\n"
                        "La compensacion automatica esta desactivada. Puedes alinearlas esta vez, "
                        "abrir la alineacion manual o calcular sin corregir."
                    )
                    dialog.add_button("Cancelar", Gtk.ResponseType.CANCEL)
                    dialog.add_button("Alinear manualmente...", Gtk.ResponseType.REJECT)
                    dialog.add_button("Calcular sin corregir", Gtk.ResponseType.NO)
                    dialog.add_button("Compensar esta vez", Gtk.ResponseType.ACCEPT)
                    response = dialog.run()
                    dialog.destroy()

                    if response == Gtk.ResponseType.ACCEPT:
                        if not self._apply_pose_alignment(interactive=False):
                            return
                        self.alignment_applied = True
                    elif response == Gtk.ResponseType.REJECT:
                        self.alignment_applied = False
                        self.stack.set_visible_child_name("alineacion")
                        return
                    elif response == Gtk.ResponseType.NO:
                        self.alignment_applied = False
                    else:
                        return
            else:
                self.alignment_applied = True
                self.quick_imu_status_label.set_text(
                    f"Pose dentro de tolerancia: {position_error_cm:.1f} cm y "
                    f"{max_rotation_error:.1f}°. No hace falta corregir."
                )

        self.quick_status_label.set_text("Calculando espesor; el estado de carga y la etapa aparecen en Resultados.")
        if self.calculation_method == "raycast":
            self.raycast_mesh_poisson_rb.set_active(self.quick_poisson_rb.get_active())
            self.raycast_mesh_delaunay_rb.set_active(self.quick_delaunay_rb.get_active())
            use_origin = (
                hasattr(self, "results_ray_method_combo")
                and self.calculation_method == "raycast"
                and self.results_ray_method_combo.get_active_id() == "origin"
            )
            self.raycast_ray_origin_rb.set_active(use_origin)
            self.raycast_ray_normal_rb.set_active(not use_origin)
            if use_origin:
                self.raycast_base_origin_entry.set_text(self.results_base_origin_entry.get_text())
                self.raycast_updated_origin_entry.set_text(self.results_updated_origin_entry.get_text())
            self._run_raycast_clicked()
        else:
            self._run_pipeline_clicked()

    def _hide_quick_mesh_result_actions(self) -> None:
        row = getattr(self, "quick_mesh_result_row", None)
        if row is None:
            return
        row.set_no_show_all(True)
        row.hide()

    def _show_quick_mesh_result_actions(self) -> None:
        row = getattr(self, "quick_mesh_result_row", None)
        if row is None:
            return
        row.set_no_show_all(False)
        row.show_all()

    def _refresh_quick_status(self) -> None:
        if not hasattr(self, "quick_status_label"):
            return
        has_base = bool(self.base_path and Path(self.base_path).is_file())
        has_updated = bool(self.updated_path and Path(self.updated_path).is_file())
        if has_base and has_updated:
            base_pose = load_reference_pose(Path(self.base_path))
            updated_pose = load_reference_pose(Path(self.updated_path))
            if base_pose is not None and updated_pose is not None:
                self.quick_imu_compensate_check.set_sensitive(True)
                position_error_cm = float(np.linalg.norm(base_pose[0] - updated_pose[0])) * 100.0
                rotation_error_deg = ((base_pose[1] - updated_pose[1] + 180.0) % 360.0) - 180.0
                max_rotation_error = float(np.max(np.abs(rotation_error_deg)))
                position_ok = position_error_cm <= self.IMU_POSITION_TOLERANCE_CM
                rotation_ok = max_rotation_error <= self.IMU_ROTATION_TOLERANCE_DEG
                self.alignment_applied = position_ok and rotation_ok
                if self.alignment_applied:
                    self.quick_alignment_status.set_text(
                        f"Diferencia entre poses: {position_error_cm:.1f} cm y "
                        f"{max_rotation_error:.1f}Â° de diferencia."
                    )
                    self.quick_status_label.set_text(
                        "Poses dentro de tolerancia. Esto asume que ambas capturas conservan el mismo tracking."
                    )
                    self.quick_imu_status_label.set_text(
                        f"Poses compatibles: {position_error_cm:.1f} cm y {max_rotation_error:.1f}°."
                    )
                else:
                    self.quick_alignment_status.set_text(
                        f"Poses distintas: {position_error_cm:.1f} cm y "
                        f"{max_rotation_error:.1f}°. Se puede compensar por software al calcular."
                    )
                    self.quick_status_label.set_text(
                        "Hay movimiento entre capturas. Se corregira automaticamente al calcular."
                    )
                    self.quick_imu_status_label.set_text(
                        f"Movimiento detectado: {position_error_cm:.1f} cm y "
                        f"{max_rotation_error:.1f}°. "
                        + ("Se compensara al calcular." if self.quick_imu_compensate_check.get_active()
                           else "Compensacion automatica desactivada.")
                    )
            else:
                self.quick_imu_compensate_check.set_sensitive(False)
                if self.alignment_applied:
                    self.quick_alignment_status.set_text("Alineacion por puntos aplicada a la nube DESPUES.")
                    self.quick_status_label.set_text(
                        "Ambas capturas estan listas y la alineacion manual aplicada. Puedes calcular el espesor."
                    )
                    self.quick_imu_status_label.set_text(
                        "La nube ya tiene alineacion manual; no hay dos poses IMU para comparar."
                    )
                else:
                    self.quick_alignment_status.set_text(
                        "No se pudo verificar: falta pose IMU en una o ambas capturas."
                    )
                    self.quick_status_label.set_text(
                        "Ambas nubes estan cargadas, pero falta pose IMU. Para verificar por IMU, "
                        "captura con el Aurora conectado y tracking activo entre ambas tomas."
                    )
                    self.quick_imu_status_label.set_text(
                        "Falta pose IMU/SLAM en una o ambas nubes; la compensacion automatica no esta disponible."
                    )
        elif has_base:
            self.quick_imu_compensate_check.set_sensitive(False)
            self.quick_alignment_status.set_text("Falta la captura DESPUES para verificar las poses IMU.")
            self.quick_status_label.set_text("Captura ANTES lista. Falta cargar o capturar DESPUES.")
            self.quick_imu_status_label.set_text("Falta cargar o capturar DESPUES para comparar poses.")
        elif has_updated:
            self.quick_imu_compensate_check.set_sensitive(False)
            self.quick_alignment_status.set_text("Falta la captura ANTES para verificar las poses IMU.")
            self.quick_status_label.set_text("Captura DESPUES lista. Falta cargar o capturar ANTES.")
            self.quick_imu_status_label.set_text("Falta cargar o capturar ANTES para comparar poses.")
        else:
            self.quick_imu_compensate_check.set_sensitive(False)
            self.quick_alignment_status.set_text("Las poses IMU se verifican cuando ambas capturas estan listas.")
            self.quick_status_label.set_text("Carga las nubes antes y despues para comenzar.")
            self.quick_imu_status_label.set_text(
                "La compensacion se activa cuando ambas nubes tienen pose guardada."
            )
        self._refresh_recalculate_button()

    # -- Pagina: Captura ---------------------------------------------------------

    def _build_capture_page(self) -> Gtk.Widget:
        page = self._new_page()

        sensor_panel, sensor_box = self._capture_panel(
            "Sensor Aurora",
            "Conecta el sensor, captura ambas etapas y revisa las nubes guardadas.",
        )
        page.pack_start(sensor_panel, False, True, 0)

        row = self._row(sensor_box)
        row.pack_start(Gtk.Label(label="Direccion del sensor:"), False, False, 0)
        self.sensor_address_entry = Gtk.Entry()
        self.sensor_address_entry.set_text("192.168.11.1")
        self.sensor_address_entry.set_width_chars(16)
        self.sensor_address_entry.connect("changed", self._sync_sensor_address)
        row.pack_start(self.sensor_address_entry, False, False, 0)
        self.connect_button = Gtk.Button(label="Conectar")
        self.connect_button.connect("clicked", lambda _b: self._toggle_sensor_connection())
        row.pack_start(self.connect_button, False, False, 0)
        self.sensor_status_label = Gtk.Label()
        self.sensor_status_label.set_markup(f'<span foreground="{COLOR_ERROR}">●</span>  Desconectado')
        row.pack_start(self.sensor_status_label, False, False, 0)
        row.pack_start(
            self._info_button(
                "Direccion IP del sensor Aurora en la red (por defecto 192.168.11.1). "
                "Conecta la PC a la red WiFi del sensor antes de presionar Conectar."
            ),
            False,
            False,
            0,
        )

        row = self._row(sensor_box)
        self.capture_base_button = Gtk.Button(label="Capturar tunel original")
        self.capture_base_button.set_sensitive(False)
        self.capture_base_button.connect("clicked", lambda _b: self._capture_clicked("base"))
        row.pack_start(self.capture_base_button, False, False, 0)
        self.capture_updated_button = Gtk.Button(label="Capturar tunel con shotcrete")
        self.capture_updated_button.set_sensitive(False)
        self.capture_updated_button.connect("clicked", lambda _b: self._capture_clicked("updated"))
        row.pack_start(self.capture_updated_button, False, False, 0)
        self.stop_capture_button = Gtk.Button(label="Detener captura")
        self.stop_capture_button.set_sensitive(False)
        self.stop_capture_button.connect("clicked", lambda _b: self._stop_capture_clicked())
        row.pack_start(self.stop_capture_button, False, False, 0)
        self.capture_progress_spinner = Gtk.Spinner()
        self.capture_progress_spinner.set_size_request(18, 18)
        self.capture_progress_spinner.set_no_show_all(True)
        self.capture_progress_spinner.hide()
        row.pack_start(self.capture_progress_spinner, False, False, 0)
        self.capture_status_label = Gtk.Label(xalign=0)
        self.capture_status_label.set_hexpand(True)
        row.pack_start(self.capture_status_label, True, True, 0)
        row.pack_start(
            self._info_button(
                "En Medición rápida configura de 1 a 30 segundos. Por defecto se captura "
                "una vista estabilizada; opcionalmente activa Captura móvil alineada para "
                "mover el Aurora lentamente y fusionar frames usando la pose SLAM. Mantén "
                "el tracking continuo y evita saltos bruscos. Detener captura guarda los "
                "frames aceptados hasta ese momento."
            ),
            False,
            False,
            0,
        )

        row = self._row(sensor_box)
        self.view_base_capture_button = Gtk.Button(label="Ver ultima captura: tunel original")
        self.view_base_capture_button.set_sensitive(False)
        self.view_base_capture_button.connect("clicked", lambda _b: self._view_last_capture("base"))
        row.pack_start(self.view_base_capture_button, False, False, 0)
        self.view_updated_capture_button = Gtk.Button(label="Ver ultima captura: tunel con shotcrete")
        self.view_updated_capture_button.set_sensitive(False)
        self.view_updated_capture_button.connect("clicked", lambda _b: self._view_last_capture("updated"))
        row.pack_start(self.view_updated_capture_button, False, False, 0)
        row.pack_start(
            self._info_button(
                "Abre en una ventana 3D la ultima captura guardada de cada tunel, para "
                "verificar que se capturo lo esperado antes de seguir con el analisis."
            ),
            False,
            False,
            0,
        )

        capture_options_panel, capture_options_box = self._capture_panel(
            "Ajustes de captura",
            "Espera el tiempo configurado en Medición rápida y aplica una mediana temporal de cinco frames.",
        )
        page.pack_start(capture_options_panel, False, True, 0)

        row = self._row(capture_options_box)
        row.pack_start(Gtk.Label(label="Persistencia minima de puntos (0-1):"), False, False, 0)
        self.capture_persistence_entry = Gtk.Entry()
        self.capture_persistence_entry.set_text("0")
        self.capture_persistence_entry.set_width_chars(6)
        row.pack_start(self.capture_persistence_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "La captura espera 15 frames válidos y guarda el último con la misma "
                "mediana temporal de cinco frames que la vista en vivo. Si se detiene antes, "
                "guarda el último frame estabilizado disponible.\n\n"
                "Persistencia minima: fraccion de los 15 frames recibidos en la que un punto debe "
                "aparecer (en la misma zona) para conservarse. Un valor mas alto (por "
                "ejemplo 0.6-0.8) descarta mejor el polvo en el aire u otro ruido "
                "transitorio, ya que esas particulas no aparecen siempre en el mismo "
                "lugar entre un frame y otro. Un valor muy alto puede descartar tambien "
                "puntos reales si el sensor tiembla."
            ),
            False,
            False,
            0,
        )

        row = self._row(capture_options_box)
        self.capture_limit_fov_check = Gtk.CheckButton(label="Activar filtros de captura")
        self.capture_limit_fov_check.set_active(False)
        row.pack_start(self.capture_limit_fov_check, False, False, 0)
        row.pack_start(Gtk.Label(label="Cono (grados):"), False, False, 0)
        self.capture_cone_angle_entry = Gtk.Entry()
        self.capture_cone_angle_entry.set_text("90")
        self.capture_cone_angle_entry.set_width_chars(6)
        row.pack_start(self.capture_cone_angle_entry, False, False, 0)
        row.pack_start(Gtk.Label(label="Eje frontal:"), False, False, 0)
        self.capture_forward_axis_combo = Gtk.ComboBoxText()
        for axis in ("x", "y", "z"):
            self.capture_forward_axis_combo.append_text(axis)
        self.capture_forward_axis_combo.set_active(2)  # "z"
        row.pack_start(self.capture_forward_axis_combo, False, False, 0)
        row.pack_start(Gtk.Label(label="Distancia maxima (m):"), False, False, 0)
        self.capture_max_distance_entry = Gtk.Entry()
        self.capture_max_distance_entry.set_text("0")
        self.capture_max_distance_entry.set_width_chars(8)
        row.pack_start(self.capture_max_distance_entry, False, False, 0)
        self.capture_limit_fov_check.connect(
            "toggled", lambda _widget: self._sync_capture_filters_to_live()
        )
        self.capture_cone_angle_entry.connect(
            "changed", lambda _widget: self._sync_capture_filters_to_live()
        )
        self.capture_forward_axis_combo.connect(
            "changed", lambda _widget: self._sync_capture_filters_to_live()
        )
        self.capture_max_distance_entry.connect(
            "changed", lambda _widget: self._sync_capture_filters_to_live()
        )
        row.pack_start(
            self._info_button(
                "Cono visual: 90 grados por defecto, aplicado solo cuando activas los filtros. "
                "Descarta puntos fuera "
                "del cono centrado en el eje frontal indicado; puede ayudar a aislar una "
                "zona, pero tambien puede dejar fuera parte de la pared.\n\n"
                "Distancia maxima: 0 = sin limite. Cuando activas los filtros, un valor mayor "
                "que cero descarta puntos mas lejanos. El check activa o desactiva ambos filtros."
            ),
            False,
            False,
            0,
        )

        live_tools_panel, live_tools_box = self._capture_panel(
            "Vista en vivo y transmision",
            "Inspecciona el sensor, fija una referencia o transmite la vista a otro dispositivo.",
        )
        page.pack_start(live_tools_panel, False, True, 0)

        row = self._row(live_tools_box)
        self.start_live_capture_button = Gtk.Button(label="Iniciar captura en tiempo real")
        self.start_live_capture_button.set_sensitive(False)
        self.start_live_capture_button.connect("clicked", lambda _b: self._start_live_capture_clicked())
        row.pack_start(self.start_live_capture_button, False, False, 0)
        self.capture_live_baseline_button = Gtk.Button(label="Fijar BASE en vivo")
        self.capture_live_baseline_button.set_sensitive(False)
        self.capture_live_baseline_button.connect("clicked", lambda _b: self._capture_live_baseline_clicked())
        row.pack_start(self.capture_live_baseline_button, False, False, 0)
        self.clear_live_baseline_button = Gtk.Button(label="Quitar BASE en vivo")
        self.clear_live_baseline_button.set_sensitive(False)
        self.clear_live_baseline_button.connect("clicked", lambda _b: self._clear_live_baseline_clicked())
        row.pack_start(self.clear_live_baseline_button, False, False, 0)
        self.save_live_clouds_button = Gtk.Button(label="Guardar nubes en vivo")
        self.save_live_clouds_button.set_sensitive(False)
        self.save_live_clouds_button.connect("clicked", lambda _b: self._save_live_clouds_clicked())
        row.pack_start(self.save_live_clouds_button, False, False, 0)
        row.pack_start(
            self._info_button(
                "'Iniciar captura en tiempo real' abre la vista 3D mostrando lo que el sensor "
                "ve ahora. 'Fijar BASE en vivo' usa ese frame como referencia para medir espesor "
                "en vivo, sin necesidad de guardar un archivo antes. 'Guardar nubes en vivo' "
                "vuelca a disco la base de referencia y el frame actual como dos .ply."
            ),
            False,
            False,
            0,
        )

        row = self._row(live_tools_box)
        self.stream_toggle_button = Gtk.Button(label="Iniciar transmision")
        self.stream_toggle_button.connect("clicked", lambda _b: self._toggle_stream_clicked())
        row.pack_start(self.stream_toggle_button, False, False, 0)
        row.pack_start(
            self._info_button(
                "Abre un servidor local en esta PC para ver la vista 3D en vivo desde un "
                "celular. Conecta el celular a la MISMA red WiFi y escanea el codigo QR que "
                "aparece aca abajo con la camara — vas a poder orbitar y hacer zoom con el "
                "dedo. Si la vista en vivo todavia no esta abierta, se abre automaticamente."
            ),
            False,
            False,
            0,
        )

        row = self._row(live_tools_box)
        self.stream_qr_image = Gtk.Image()
        row.pack_start(self.stream_qr_image, False, False, 0)
        qr_side_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        self.stream_url_label = Gtk.Label(label="", xalign=0)
        self.stream_url_label.set_selectable(True)
        qr_side_box.pack_start(self.stream_url_label, False, False, 0)
        row.pack_start(qr_side_box, False, False, 0)

        return self._scrolled(page)

    # -- Pagina: Alineacion IMU ----------------------------------------------------

    IMU_POSITION_TOLERANCE_CM = 0.5
    IMU_ROTATION_TOLERANCE_DEG = 3.0

    def _build_imu_alignment_page(self) -> Gtk.Widget:
        page = self._new_page()

        self.imu_reference_pose: tuple[np.ndarray, np.ndarray] | None = None
        self.imu_reference_source_path: str | None = None
        self.imu_direction_words: dict[str, tuple[str, str]] | None = None
        self.imu_poll_source_id: int | None = None

        info_frame, info_box = self._section("Reposicionar el sensor (IMU/SLAM)")
        note = Gtk.Label(
            label=(
                "El sensor Aurora sigue su propia posicion (tracking visual-inercial) "
                "mientras queda encendido. Al capturar el 'Tunel original' se guarda esa "
                "posicion como referencia. Podes mover el sensor libremente para aplicar "
                "el shotcrete; antes de capturar el 'Tunel con shotcrete', usa esta "
                "pagina para devolverlo a la misma posicion fisica siguiendo la guia de "
                "abajo (no apagues el sensor entre medio o pierde el tracking).\n\n"
                "La guia indica hacia donde mover con palabras (adelante/atras, "
                "izquierda/derecha, arriba/abajo) calculadas a partir de hacia donde "
                "miraba el sensor al capturar el 'Tunel original' — 'adelante' es la cara "
                "con las camaras en ese momento. Adelante/atras ya se valido contra el "
                "sensor real; izquierda/derecha y arriba/abajo son la mejor aproximacion "
                "disponible pero todavia no se confirmaron — si notas que no coinciden con "
                "el movimiento real, confia en el numero (cm) y el color en vez de la "
                "palabra."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        info_box.pack_start(note, False, False, 0)
        page.pack_start(info_frame, False, True, 0)

        ref_frame, ref_box = self._section("Posicion de referencia")
        row = self._row(ref_box)
        self.imu_reference_label = Gtk.Label(label="(sin capturar todavia)", xalign=0)
        row.pack_start(self.imu_reference_label, False, False, 0)
        refresh_button = Gtk.Button(label="Actualizar")
        refresh_button.set_tooltip_text(
            "Vuelve a buscar la pose de referencia guardada junto al 'Tunel original' actual."
        )
        refresh_button.connect("clicked", lambda _b: self._refresh_imu_reference())
        row.pack_start(refresh_button, False, False, 0)
        page.pack_start(ref_frame, False, True, 0)

        guide_frame, guide_box = self._section("Guia en vivo")
        self.imu_guide_status_label = Gtk.Label(label="", xalign=0)
        guide_box.pack_start(self.imu_guide_status_label, False, False, 0)

        grid = Gtk.Grid(column_spacing=16, row_spacing=6)
        guide_box.pack_start(grid, False, False, 0)

        def add_axis_row(row_idx: int, title: str, swatch_color: str | None = None) -> Gtk.Label:
            title_label = Gtk.Label(xalign=0)
            swatch = f'<span foreground="{swatch_color}">●</span> ' if swatch_color else ""
            title_label.set_markup(f"{swatch}{GLib.markup_escape_text(title)}")
            title_label.set_size_request(150, -1)
            grid.attach(title_label, 0, row_idx, 1, 1)
            value_label = Gtk.Label(label="", xalign=0)
            value_label.set_width_chars(14)
            value_label.set_xalign(0.0)
            grid.attach(value_label, 1, row_idx, 1, 1)
            return value_label

        self.imu_axis_labels = {
            "x": add_axis_row(0, "Eje X:", AXIS_HUD_COLORS["x"]),
            "y": add_axis_row(1, "Eje Y:", AXIS_HUD_COLORS["y"]),
            "z": add_axis_row(2, "Eje Z:", AXIS_HUD_COLORS["z"]),
            "roll": add_axis_row(3, "Rotacion (roll):"),
            "pitch": add_axis_row(4, "Rotacion (pitch):"),
            "yaw": add_axis_row(5, "Rotacion (yaw):"),
        }

        total_row = self._row(guide_box)
        total_caption = Gtk.Label(label="Distancia total:", xalign=0)
        total_caption.set_size_request(150, -1)
        total_row.pack_start(total_caption, False, False, 0)
        self.imu_total_distance_label = Gtk.Label(label="", xalign=0)
        self.imu_total_distance_label.set_width_chars(14)
        total_row.pack_start(self.imu_total_distance_label, False, False, 0)

        page.pack_start(guide_frame, False, True, 0)

        viewer_frame, viewer_box = self._section("Ver posiciones en 3D")
        viewer_note = Gtk.Label(
            label=(
                "Abre el visor 3D con los ejes X (rojo), Y (verde) y Z (azul) fijos "
                "en el origen; la esfera muestra la posicion actual respecto del "
                "objetivo y cambia de rojo a ambar y verde al acercarte. Las métricas "
                "aparecen dentro del mismo visor. El eje Z azul representa adelante, "
                "como en las vistas de las nubes."
            ),
            xalign=0,
        )
        viewer_note.set_line_wrap(True)
        viewer_box.pack_start(viewer_note, False, False, 0)
        row = self._row(viewer_box)
        self.open_imu_viewer_button = Gtk.Button(label="Abrir vista 3D")
        self.open_imu_viewer_button.connect("clicked", lambda _b: self._open_imu_pose_viewer())
        row.pack_start(self.open_imu_viewer_button, False, False, 0)
        self.close_imu_viewer_button = Gtk.Button(label="Cerrar vista 3D")
        self.close_imu_viewer_button.set_sensitive(False)
        self.close_imu_viewer_button.connect("clicked", lambda _b: self._close_imu_pose_viewer())
        row.pack_start(self.close_imu_viewer_button, False, False, 0)
        page.pack_start(viewer_frame, False, True, 0)

        return self._scrolled(page)

    def _open_imu_pose_viewer(self) -> None:
        if self.imu_reference_pose is None:
            self._show_warning(
                "Sin posicion de referencia",
                "No hay una posicion de referencia cargada (ver 'Posicion de referencia' arriba).",
            )
            return
        if self.pose_viewer is not None and self.pose_viewer.is_running():
            self._show_info("Vista 3D", "La vista 3D de alineacion IMU ya esta abierta.")
            return

        reference_position, reference_rpy_deg = self.imu_reference_pose
        self.pose_viewer = PoseAlignmentViewer(
            reference_position,
            self.IMU_POSITION_TOLERANCE_CM / 100.0,
            log=self._log,
            reference_rpy_deg=reference_rpy_deg,
        )
        self.pose_viewer.start()
        self.close_imu_viewer_button.set_sensitive(True)

    def _close_imu_pose_viewer(self) -> None:
        if self.pose_viewer is not None:
            self.pose_viewer.stop()
            self.pose_viewer = None
        self.close_imu_viewer_button.set_sensitive(False)

    def _refresh_imu_reference(self) -> None:
        self._close_imu_pose_viewer()
        self.imu_reference_pose = None
        self.imu_reference_source_path = None
        base_path = getattr(self, "base_path", None)
        if not base_path or not Path(base_path).is_file():
            self.imu_reference_label.set_text("(sin capturar todavia)")
            self._set_imu_guide_disabled("Todavia no hay un 'Tunel original' cargado.")
            return

        pose = load_reference_pose(Path(base_path))
        if pose is None:
            self.imu_reference_label.set_text(f"No hay pose guardada para: {Path(base_path).name}")
            self._set_imu_guide_disabled(
                "Esta captura no tiene posicion de referencia (se hizo sin sensor conectado, o "
                "es anterior a esta funcion). Volve a capturar el 'Tunel original' con el "
                "sensor conectado."
            )
            return

        self.imu_reference_pose = pose
        self.imu_reference_source_path = base_path
        self.imu_direction_words = dict(IMU_SENSOR_VIEW_DIRECTIONS)
        self.imu_reference_label.set_text(f"Guardada desde: {Path(base_path).name}")
        self.imu_guide_status_label.set_text("Conecta el sensor para ver la guia en vivo.")

    def _set_imu_guide_disabled(self, message: str) -> None:
        self.imu_guide_status_label.set_text(message)
        for label in self.imu_axis_labels.values():
            label.set_text("")
        self.imu_total_distance_label.set_text("")

    def _start_imu_poll(self) -> None:
        if self.imu_poll_source_id is not None:
            return
        self.imu_poll_source_id = GLib.timeout_add(300, self._imu_poll_tick)

    def _stop_imu_poll(self) -> None:
        source_id = getattr(self, "imu_poll_source_id", None)
        if source_id is not None:
            GLib.source_remove(source_id)
            self.imu_poll_source_id = None

    def _imu_poll_tick(self) -> bool:
        if self.stack.get_visible_child_name() != "alineacion_imu":
            self.imu_poll_source_id = None
            return False

        if self.sensor_connection is None:
            self._set_imu_guide_disabled("Conecta el sensor en la pestana 'Captura' para ver la guia en vivo.")
            return True

        if self.imu_reference_pose is None:
            self._set_imu_guide_disabled("No hay posicion de referencia cargada (ver arriba).")
            return True

        try:
            pose = aurora_sensor.get_current_pose(self.sensor_connection)
        except Exception as exc:
            self._set_imu_guide_disabled(f"No se pudo leer la posicion del sensor: {exc}")
            return True

        ref_position, ref_rpy_deg = self.imu_reference_pose
        position_error_m = world_delta_to_sensor_view(
            ref_position - pose.position, ref_rpy_deg
        )
        rotation_error_deg = ((ref_rpy_deg - pose.rpy_deg + 180.0) % 360.0) - 180.0

        position_ok = True
        pending_move: tuple[str, float] | None = None  # (palabra, cm) del primer eje sin corregir
        for axis, value_m in zip(("x", "y", "z"), position_error_m):
            value_cm = value_m * 100.0
            positive_word, negative_word = self.imu_direction_words[axis]
            word = positive_word if value_cm >= 0 else negative_word
            within = abs(value_cm) <= self.IMU_POSITION_TOLERANCE_CM
            position_ok = position_ok and within
            if not within and pending_move is None:
                pending_move = (word, abs(value_cm))
            color = COLOR_OK if within else COLOR_WARN
            markup = f'<span foreground="{color}">mover hacia {word.upper()}: {abs(value_cm):.1f} cm</span>'
            self.imu_axis_labels[axis].set_markup(markup)

        rotation_ok = True
        for axis, value_deg in zip(("roll", "pitch", "yaw"), rotation_error_deg):
            direction = "+" if value_deg >= 0 else "-"
            within = abs(value_deg) <= self.IMU_ROTATION_TOLERANCE_DEG
            rotation_ok = rotation_ok and within
            color = COLOR_OK if within else COLOR_WARN
            markup = f'<span foreground="{color}">girar {direction}: {abs(value_deg):.1f}°</span>'
            self.imu_axis_labels[axis].set_markup(markup)

        total_distance_cm = float(np.linalg.norm(position_error_m)) * 100.0
        self.imu_total_distance_label.set_text(f"{total_distance_cm:.1f} cm")
        if self.pose_viewer is not None and self.pose_viewer.is_running():
            self.pose_viewer.update_current_pose(pose.position, pose.rpy_deg)

        if pending_move is not None:
            word, value_cm = pending_move
            status_markup = (
                f'<span foreground="{COLOR_WARN}" size="large" weight="bold">'
                f"Mover hacia {word.upper()}: {value_cm:.1f} cm</span>"
            )
        elif not rotation_ok:
            status_markup = (
                f'<span foreground="{COLOR_WARN}">Posicion correcta — ajusta la rotacion (ver abajo)</span>'
            )
        else:
            status_markup = f'<span foreground="{COLOR_OK}">✓ Sensor en posicion, listo para capturar</span>'
        self.imu_guide_status_label.set_markup(status_markup)

        return True

    def _invalidate_results(self) -> None:
        """Clear output tied to the previous pair so it cannot be reported as current."""
        self._result_generation += 1
        self.result = None
        self.raycast_result = None
        self._active_result_kind = None
        self._hide_quick_mesh_result_actions()
        if hasattr(self, "log_buffer"):
            self.log_buffer.set_text("")
        if hasattr(self, "cards_row"):
            self.cards_row.set_no_show_all(True)
            self.cards_row.hide()
            self.results_empty_label.show()
            self.thickness_alert_label.set_text("")
            self.thickness_alert_label.set_no_show_all(True)
            self.thickness_alert_label.hide()
            self.status_label.set_text("Todavia no se calculo ningun resultado.")
            self.generate_report_button.set_sensitive(False)
            self.open_result_viewer_button.set_sensitive(False)
            self.open_result_viewer_button.set_label("Abrir vista 3D")
            self._finish_calculation_progress(False)
        if hasattr(self, "raycast_status_label"):
            self.raycast_status_label.set_text("(nada calculado todavia)")

    def _set_base_path(self, path: str) -> None:
        self._invalidate_results()
        self.base_path = path
        self.alignment_applied = False
        self._hide_quick_mesh_result_actions()
        if hasattr(self, "quick_alignment_status"):
            self.quick_alignment_status.set_text("Alineacion no aplicada")
        self.landmark_base_manual_photo_path = None
        for name in ("base_path_label", "raycast_base_path_label", "quick_base_path_label"):
            label = getattr(self, name, None)
            if label is not None:
                self._set_path_label(label, path)
        if hasattr(self, "quick_view_base_button"):
            self.quick_view_base_button.set_sensitive(bool(path and Path(path).is_file()))
        self._sync_test_crop_input_path("before", path)
        if hasattr(self, "landmark_base_manual_photo_label"):
            self.landmark_base_manual_photo_label.set_text("")
        if hasattr(self, "imu_reference_label"):
            self._refresh_imu_reference()
        self._refresh_quick_status()

    def _set_updated_path(self, path: str) -> None:
        self._invalidate_results()
        self.updated_path = path
        self._pose_alignment_original_path = None
        self._pose_alignment_aligned_path = None
        if hasattr(self, "undo_pose_alignment_btn"):
            self.undo_pose_alignment_btn.set_no_show_all(True)
            self.undo_pose_alignment_btn.hide()
        self.alignment_applied = False
        self._hide_quick_mesh_result_actions()
        if hasattr(self, "quick_alignment_status"):
            self.quick_alignment_status.set_text("Alineacion no aplicada")
        self.landmark_updated_manual_photo_path = None
        for name in ("updated_path_label", "raycast_updated_path_label", "quick_updated_path_label"):
            label = getattr(self, name, None)
            if label is not None:
                self._set_path_label(label, path)
        if hasattr(self, "quick_view_updated_button"):
            self.quick_view_updated_button.set_sensitive(bool(path and Path(path).is_file()))
        self._sync_test_crop_input_path("after", path)
        if hasattr(self, "landmark_updated_manual_photo_label"):
            self.landmark_updated_manual_photo_label.set_text("")
        if hasattr(self, "imu_reference_label"):
            self._refresh_imu_reference()
        self._refresh_quick_status()

    def _sync_test_crop_input_path(self, stage: str, path: str) -> None:
        """Keep quick-measurement crop controls bound to the selected pair."""
        if not hasattr(self, "test_crop_before_path"):
            return
        if stage == "before":
            self.test_crop_before_path = path
            if not self.test_crop_output_dir_is_custom:
                self.test_crop_output_dir = PROJECT_ROOT / "data" / "recortes"
                if hasattr(self, "test_crop_output_dir_label"):
                    self._set_test_crop_output_dir_label()
        else:
            self.test_crop_after_path = path

        self.test_crop_output_paths = None
        self.test_crop_bounds = None
        self.test_crop_seed = None
        self.test_crop_bounds_label.set_text("Limites sin definir")
        self.test_crop_preview_button.set_label("Vista previa del box")
        self.test_crop_use_pair_button.set_sensitive(False)
        if path:
            self.test_crop_status_label.set_text(
                "Par sincronizado. La pose se compensará antes del recorte si está activada y ambas capturas la guardaron."
            )
        else:
            self.test_crop_status_label.set_text(
                "Selecciona ambas nubes en Medicion rapida para empezar."
            )
        self._refresh_test_crop_controls()

    # -- Pagina: Alineacion (Procrustes por puntos de referencia) ---------------

    def _build_alignment_page(self) -> Gtk.Widget:
        page = self._new_page()

        self.landmarks_base: list | None = None
        self.landmarks_updated: list | None = None
        self.landmark_base_manual_photo_path: str | None = None
        self.landmark_updated_manual_photo_path: str | None = None

        info_frame, info_box = self._section("Alinear con puntos de referencia (opcional)")
        note = Gtk.Label(
            label=(
                "Si las dos capturas no comparten exactamente la misma posicion (el sensor "
                "se reubico entre una y otra), puedes alinearlas eligiendo 3 o mas puntos "
                "fijos que NO se muevan entre capturas — por ejemplo cabezas de pernos de "
                "anclaje, marcas o esquinas rigidas. A diferencia de ICP (que ajusta toda "
                "la superficie y puede confundir el espesor real con error de alineacion), "
                "esto usa solo esos puntos fijos como referencia.\n\n"
                "Si el punto de referencia queda tapado por el shotcrete (ej. solo sobresale "
                "la punta de un perno, muy fina para que el sensor la resuelva como puntos "
                "3D limpios), elige 'Foto de referencia' en vez de 'Nube 3D': se elige el "
                "punto sobre la foto que el sensor capturo junto con la nube, y el sistema "
                "busca el punto 3D correspondiente. Solo esta disponible si esa captura se "
                "hizo con el sensor Aurora conectado."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        info_box.pack_start(note, False, False, 0)
        page.pack_start(info_frame, False, True, 0)

        pose_frame, pose_box = self._section("Opción rápida: Alineación automática por Pose IMU/SLAM")
        pose_desc = Gtk.Label(
            label=(
                "Si ambas capturas conservan el tracking del sensor, el sistema puede compensar "
                "automáticamente los centímetros o grados que se haya movido el trípode sin necesidad de marcar puntos a mano."
            ),
            xalign=0,
        )
        pose_desc.set_line_wrap(True)
        pose_box.pack_start(pose_desc, False, False, 0)

        pose_row = self._row(pose_box)
        self.apply_pose_alignment_btn = Gtk.Button(label="Compensar movimiento automáticamente por pose")
        self.apply_pose_alignment_btn.connect("clicked", lambda _b: self._apply_pose_alignment(interactive=True))
        pose_row.pack_start(self.apply_pose_alignment_btn, False, False, 0)
        self.undo_pose_alignment_btn = Gtk.Button(label="Quitar compensación")
        self.undo_pose_alignment_btn.set_tooltip_text(
            "Volver a seleccionar la nube DESPUES original sin compensación por pose."
        )
        self.undo_pose_alignment_btn.set_no_show_all(True)
        self.undo_pose_alignment_btn.hide()
        self.undo_pose_alignment_btn.connect("clicked", lambda _b: self._undo_pose_alignment())
        pose_row.pack_start(self.undo_pose_alignment_btn, False, False, 0)
        self.pose_alignment_status_label = Gtk.Label(label="")
        pose_row.pack_start(self.pose_alignment_status_label, False, False, 0)
        page.pack_start(pose_frame, False, True, 0)

        step_frame, step_box = self._section("Método alternativo: marcar puntos fijos (respaldo)")

        row = self._row(step_box)
        pick_base_button = Gtk.Button(label="1. Elegir puntos en el tunel original...")
        pick_base_button.set_tooltip_text(
            "Clic izquierdo sobre cada punto de referencia, en orden. Puedes usar Shift+clic tambien; despues cierra la ventana con Q."
        )
        pick_base_button.connect("clicked", lambda _b: self._pick_alignment_points("base"))
        row.pack_start(pick_base_button, False, False, 0)
        self.landmarks_base_label = Gtk.Label(label="(ninguno)")
        row.pack_start(self.landmarks_base_label, False, False, 0)

        row = self._row(step_box)
        row.pack_start(Gtk.Label(label="Elegir sobre:"), False, False, 0)
        self.landmark_base_source_3d_rb = Gtk.RadioButton.new_with_label(None, "Nube 3D")
        self.landmark_base_source_photo_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.landmark_base_source_3d_rb, "Foto de referencia"
        )
        row.pack_start(self.landmark_base_source_3d_rb, False, False, 0)
        row.pack_start(self.landmark_base_source_photo_rb, False, False, 0)
        load_base_photo_button = Gtk.Button(label="Cargar foto de referencia...")
        load_base_photo_button.set_tooltip_text(
            "Elegir manualmente el archivo _ref.npz (foto + profundidad) si el .ply se "
            "movio/renombro y ya no se detecta automaticamente."
        )
        load_base_photo_button.connect("clicked", lambda _b: self._load_manual_reference_photo("base"))
        row.pack_start(load_base_photo_button, False, False, 0)
        self.landmark_base_manual_photo_label = Gtk.Label(
            label=Path(self.landmark_base_manual_photo_path).name if self.landmark_base_manual_photo_path else ""
        )
        row.pack_start(self.landmark_base_manual_photo_label, False, False, 0)
        if self.landmark_base_manual_photo_path:
            self.landmark_base_source_photo_rb.set_active(True)

        row = self._row(step_box)
        pick_updated_button = Gtk.Button(label="2. Elegir los MISMOS puntos en el tunel con shotcrete...")
        pick_updated_button.set_tooltip_text(
            "Elige los puntos en el MISMO orden que en el paso 1, sobre los mismos puntos fisicos."
        )
        pick_updated_button.connect("clicked", lambda _b: self._pick_alignment_points("updated"))
        row.pack_start(pick_updated_button, False, False, 0)
        self.landmarks_updated_label = Gtk.Label(label="(ninguno)")
        row.pack_start(self.landmarks_updated_label, False, False, 0)

        row = self._row(step_box)
        row.pack_start(Gtk.Label(label="Elegir sobre:"), False, False, 0)
        self.landmark_updated_source_3d_rb = Gtk.RadioButton.new_with_label(None, "Nube 3D")
        self.landmark_updated_source_photo_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.landmark_updated_source_3d_rb, "Foto de referencia"
        )
        row.pack_start(self.landmark_updated_source_3d_rb, False, False, 0)
        row.pack_start(self.landmark_updated_source_photo_rb, False, False, 0)
        load_updated_photo_button = Gtk.Button(label="Cargar foto de referencia...")
        load_updated_photo_button.set_tooltip_text(
            "Elegir manualmente el archivo _ref.npz (foto + profundidad) si el .ply se "
            "movio/renombro y ya no se detecta automaticamente."
        )
        load_updated_photo_button.connect("clicked", lambda _b: self._load_manual_reference_photo("updated"))
        row.pack_start(load_updated_photo_button, False, False, 0)
        self.landmark_updated_manual_photo_label = Gtk.Label(label="")
        row.pack_start(self.landmark_updated_manual_photo_label, False, False, 0)

        page.pack_start(step_frame, False, True, 0)

        apply_frame, apply_box = self._section("Paso 2: calcular y aplicar")
        row = self._row(apply_box)
        self.apply_alignment_button = Gtk.Button(label="Calcular alineacion y aplicar")
        self.apply_alignment_button.connect("clicked", lambda _b: self._apply_alignment())
        row.pack_start(self.apply_alignment_button, False, False, 0)
        self.alignment_result_label = Gtk.Label(label="")
        row.pack_start(self.alignment_result_label, False, False, 0)
        page.pack_start(apply_frame, False, True, 0)

        return self._scrolled(page)

    def _load_manual_reference_photo(self, which: str) -> None:
        dialog = Gtk.FileChooserNative.new(
            "Seleccionar foto de referencia (_ref.npz)",
            self.window,
            Gtk.FileChooserAction.OPEN,
            "_Abrir",
            "_Cancelar",
        )
        filter_npz = Gtk.FileFilter()
        filter_npz.set_name("Foto de referencia (*_ref.npz)")
        filter_npz.add_pattern("*.npz")
        dialog.add_filter(filter_npz)
        path = self._run_native_file_chooser(dialog)
        if not path:
            return

        try:
            load_reference_photo_file(Path(path))
        except Exception as exc:
            self._show_error("Error al cargar la foto de referencia", str(exc))
            return

        name = Path(path).name
        if which == "base":
            self.landmark_base_manual_photo_path = path
            self.landmark_base_manual_photo_label.set_text(name)
            self.landmark_base_source_photo_rb.set_active(True)
        else:
            self.landmark_updated_manual_photo_path = path
            self.landmark_updated_manual_photo_label.set_text(name)
            self.landmark_updated_source_photo_rb.set_active(True)

    def _pick_alignment_points(self, which: str) -> None:
        path = Path(self.base_path if which == "base" else self.updated_path)

        use_photo = (
            self.landmark_base_source_photo_rb.get_active()
            if which == "base"
            else self.landmark_updated_source_photo_rb.get_active()
        )

        manual_photo_path = (
            self.landmark_base_manual_photo_path if which == "base" else self.landmark_updated_manual_photo_path
        )

        # Si se eligio una foto de referencia manual, no hace falta el .ply
        # (base_path/updated_path) para nada: todo el picking usa el .npz.
        if not (use_photo and manual_photo_path) and not path.is_file():
            self._show_error("Error", f"No se encontro el archivo:\n{path}")
            return

        def worker():
            if use_photo:
                try:
                    reference = (
                        load_reference_photo_file(Path(manual_photo_path))
                        if manual_photo_path
                        else load_reference_photo(path)
                    )
                    if reference is None:
                        self._ui(self._on_alignment_pick_finished, which, use_photo, None, None, "missing_photo")
                        return
                    image, point_grid = reference
                    title = (
                        "Tunel original - click en cada punto de referencia, en orden, luego cerrar la ventana"
                        if which == "base"
                        else "Tunel con shotcrete - click en los MISMOS puntos, en el mismo orden, luego cerrar la ventana"
                    )
                    points = pick_landmark_points_from_photo(image, point_grid, title, log=self._log)
                except Exception as exc:
                    self._ui(self._on_alignment_pick_finished, which, use_photo, None, str(exc), None)
                    return
            else:
                try:
                    cloud = load_point_cloud(path)
                    title = (
                        "Tunel original - clic en cada punto de referencia, en orden; Q para terminar"
                        if which == "base"
                        else "Tunel con shotcrete - Elige los MISMOS puntos, en el mismo orden, luego cerrar (Q)"
                    )
                    points = pick_landmark_points(cloud, title)
                except Exception as exc:
                    self._ui(self._on_alignment_pick_finished, which, use_photo, None, str(exc), None)
                    return
            self._ui(self._on_alignment_pick_finished, which, use_photo, points, None, None)

        threading.Thread(target=worker, daemon=True).start()

    def _on_alignment_pick_finished(self, which, use_photo, points, error, warning) -> None:
        if warning == "missing_photo":
            path = self.base_path if which == "base" else self.updated_path
            self._show_warning(
                "Sin foto de referencia",
                f"No hay foto de referencia guardada para:\n{path}\n\n"
                "Solo se genera al capturar esa nube con el sensor Aurora conectado. "
                "Si tienes el archivo _ref.npz en otra ubicacion (ej. el .ply se "
                "movio/renombro), usa 'Cargar foto de referencia...'. Si no, elige "
                "'Nube 3D' para este archivo.",
            )
            return
        if error:
            self._show_error("Error al elegir los puntos", error)
            return
        if points is None:
            self._show_info("Sin seleccion", "Elige al menos 3 puntos antes de cerrar la ventana.")
            return

        method_label = "foto" if use_photo else "nube 3D"
        if which == "base":
            self.landmarks_base = points
            self.landmarks_base_label.set_text(f"{len(points)} puntos elegidos ({method_label}) ✓")
        else:
            self.landmarks_updated = points
            self.landmarks_updated_label.set_text(f"{len(points)} puntos elegidos ({method_label}) ✓")

    def _apply_alignment(self) -> None:
        if self.landmarks_base is None or self.landmarks_updated is None:
            self._show_warning("Faltan puntos", "Elige los puntos de referencia en las dos nubes primero.")
            return
        if len(self.landmarks_base) != len(self.landmarks_updated):
            self._show_error(
                "Cantidad distinta de puntos",
                f"Elegiste {len(self.landmarks_base)} puntos en el original y "
                f"{len(self.landmarks_updated)} en el de shotcrete. Tienen que ser la misma cantidad, "
                "en el mismo orden.",
            )
            return

        try:
            rotation, translation = compute_rigid_transform(self.landmarks_base, self.landmarks_updated)
            rms = rigid_transform_rms_error(self.landmarks_base, self.landmarks_updated, rotation, translation)
        except Exception as exc:
            self._show_error("Error al calcular la alineacion", str(exc))
            return

        self.apply_alignment_button.set_sensitive(False)
        self.alignment_result_label.set_text("Aplicando...")
        updated_file = Path(self.updated_path)
        base_pose = load_reference_pose(Path(self.base_path))
        manual_photo_path = (
            Path(self.landmark_updated_manual_photo_path)
            if self.landmark_updated_source_photo_rb.get_active()
            and self.landmark_updated_manual_photo_path
            else reference_photo_path_for(updated_file)
        )
        if not manual_photo_path.is_file():
            manual_photo_path = None

        def worker():
            try:
                cloud = load_point_cloud(updated_file)
                aligned = apply_rigid_transform(cloud, rotation, translation)
                aligned_path = updated_file.with_name(updated_file.stem + "_alineado.ply")
                import open3d as o3d

                if not o3d.io.write_point_cloud(str(aligned_path), aligned):
                    raise OSError(f"No se pudo escribir la nube alineada: {aligned_path}")
                if base_pose is not None:
                    save_reference_pose(aligned_path, base_pose[0], base_pose[1])
                if manual_photo_path is not None:
                    save_transformed_reference_photo(
                        manual_photo_path, aligned_path, rotation, translation
                    )
            except Exception as exc:
                self._ui(self._on_alignment_failed, str(exc))
                return
            self._ui(self._on_alignment_done, str(aligned_path), rms)

        threading.Thread(target=worker, daemon=True).start()

    def _on_alignment_done(self, aligned_path: str, rms: float) -> None:
        self.apply_alignment_button.set_sensitive(True)
        self.alignment_result_label.set_text(f"Error residual: {rms * 100:.2f} cm — nube alineada guardada")
        self._set_updated_path(aligned_path)
        self.alignment_applied = True
        self.quick_alignment_status.set_text(f"Alineacion aplicada (residual: {rms * 100:.2f} cm)")
        self._refresh_quick_status()
        self._show_info(
            "Alineacion aplicada",
            f"Error residual en los puntos de referencia: {rms * 100:.2f} cm.\n\n"
            f"Se guardo la nube alineada en:\n{aligned_path}\n\n"
            "Medicion rapida ya usa este archivo como nube DESPUES.",
        )

    def _on_alignment_failed(self, message: str) -> None:
        self.apply_alignment_button.set_sensitive(True)
        self.alignment_result_label.set_text("Error al aplicar la alineacion")
        self._show_error("Error", message)

    def _apply_pose_alignment(self, interactive: bool = True) -> bool:
        base_file = Path(self.base_path)
        updated_file = Path(self.updated_path)
        aligned_suffix = "_alineado_pose"
        if (
            updated_file.stem.endswith(aligned_suffix)
            and reference_pose_alignment_version(updated_file) < 2
        ):
            original_file = updated_file.with_name(
                updated_file.stem[: -len(aligned_suffix)] + updated_file.suffix
            )
            if original_file.is_file():
                updated_file = original_file
                self._set_updated_path(str(original_file))
        if not base_file.is_file() or not updated_file.is_file():
            if interactive:
                self._show_error("Archivos no encontrados", "Verifica que ambas nubes estén seleccionadas.")
            return False

        base_pose = load_reference_pose(base_file)
        updated_pose = load_reference_pose(updated_file)

        if base_pose is None or updated_pose is None:
            if interactive:
                self._show_error(
                    "Faltan datos de pose",
                    "Uno o ambos archivos .ply no tienen archivo de pose (_pose.npz) asociado.\n\n"
                    "Para usar la compensación automática, las capturas deben haberse realizado "
                    "con el sensor Aurora conectado.",
                )
            return False

        try:
            R_rel, t_rel = compute_pose_relative_transform(base_pose, updated_pose)
            dist_cm = float(np.linalg.norm(t_rel)) * 100.0
            cos_angle = float(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))
            rot_deg = float(np.degrees(np.arccos(cos_angle)))

            cloud = load_point_cloud(updated_file)
            aligned = apply_rigid_transform(cloud, R_rel, t_rel)
            aligned_path = updated_file.with_name(updated_file.stem + "_alineado_pose.ply")
            import open3d as o3d
            if not o3d.io.write_point_cloud(str(aligned_path), aligned):
                raise OSError(f"No se pudo escribir la nube alineada por pose: {aligned_path}")

            # Guardar pose de la base para la nueva nube alineada para reflejar que ya coinciden
            save_reference_pose(aligned_path, base_pose[0], base_pose[1], alignment_version=2)

            # La rejilla 3D asociada a la foto tambien debe recibir la misma
            # transformacion que la nube; copiarla sin transformar la desincroniza.
            ref_photo_src = reference_photo_path_for(updated_file)
            if ref_photo_src.exists():
                try:
                    save_transformed_reference_photo(ref_photo_src, aligned_path, R_rel, t_rel)
                except Exception as exc:
                    self._log(f"No se pudo actualizar la foto de referencia alineada: {exc}")

            self._set_updated_path(str(aligned_path))
            self._pose_alignment_original_path = str(updated_file)
            self._pose_alignment_aligned_path = str(aligned_path)
            if hasattr(self, "undo_pose_alignment_btn"):
                self.undo_pose_alignment_btn.set_no_show_all(False)
                self.undo_pose_alignment_btn.show()
            self.alignment_applied = True

            msg = f"Compensación aplicada: {dist_cm:.1f} cm y {rot_deg:.1f}° corregidos."
            if hasattr(self, "quick_alignment_status"):
                self.quick_alignment_status.set_text(f"Alineación por pose aplicada ({dist_cm:.1f} cm)")
            if hasattr(self, "pose_alignment_status_label"):
                self.pose_alignment_status_label.set_text(f"✓ {msg}")
            self._refresh_quick_status()
            self._log(f"Alineación por pose completada: {msg} (guardado en {aligned_path.name})")

            if interactive:
                self._show_info(
                    "Alineación por pose aplicada",
                    f"Se compensó el movimiento del sensor exitosamente:\n\n"
                    f"• Desplazamiento compensado: {dist_cm:.1f} cm\n"
                    f"• Rotación compensada: {rot_deg:.1f}°\n\n"
                    f"Nube alineada guardada como:\n{aligned_path.name}\n\n"
                    "Medición rápida ya usa esta nube alineada.",
                )
            return True
        except Exception as exc:
            if interactive:
                self._show_error("Error al compensar pose", str(exc))
            return False

    def _undo_pose_alignment(self) -> None:
        original_path = self._pose_alignment_original_path
        aligned_path = self._pose_alignment_aligned_path
        if not original_path or not Path(original_path).is_file():
            self._show_error(
                "No se pudo quitar la compensación",
                "No encuentro la nube DESPUES original para volver a seleccionarla.",
            )
            return
        if aligned_path and self.updated_path != aligned_path:
            self._show_warning(
                "La nube seleccionada cambió",
                "La compensación ya no está aplicada a la nube seleccionada.",
            )
            self._pose_alignment_original_path = None
            self._pose_alignment_aligned_path = None
            self.undo_pose_alignment_btn.set_no_show_all(True)
            self.undo_pose_alignment_btn.hide()
            return

        self._set_updated_path(original_path)
        self.alignment_applied = False
        message = "Compensación por pose quitada; se volvió a la nube DESPUES original."
        self.quick_alignment_status.set_text(message)
        self.pose_alignment_status_label.set_text(message)
        self._log(message)

    # -- Pagina: preparar y recortar un par de nubes -----------------------------

    def _build_prepare_clouds_page(self) -> Gtk.Widget:
        page = self._new_page()
        self.test_crop_before_path: str | None = self.base_path or None
        self.test_crop_after_path: str | None = self.updated_path or None
        self.test_crop_output_paths: tuple[str, str] | None = None
        self.test_crop_output_dir: Path | None = PROJECT_ROOT / "data" / "recortes"
        self.test_crop_output_dir_is_custom = False
        self.test_crop_bounds: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None
        self.test_crop_seed: np.ndarray | None = None

        info_frame, info_box = self._section("Preparar un par de nubes")
        note = Gtk.Label(
            label=(
                "Selecciona las nubes ANTES y DESPUES en Medicion rapida. Puedes hacer clic en un punto de la pared "
                "o detectar automaticamente la pared frontal. El lado de la cara cuadrada y la profundidad se "
                "ajustan por separado; la vista previa solo muestra puntos dentro del box. Al guardar, los dos PLY "
                "recortados quedan en data/recortes y los originales se conservan. Con la compensación activa y poses guardadas, DESPUÉS se alinea "
                "al marco de ANTES antes de mostrar o aplicar el mismo box; sin poses, deben compartir coordenadas."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        info_box.pack_start(note, False, False, 0)
        page.pack_start(info_frame, False, True, 0)

        bounds_frame, bounds_box = self._section("1. Detectar la pared y definir la region")
        self.test_crop_view_pair_button = Gtk.Button(label="Ver ANTES y DESPUES juntos")
        self.test_crop_view_pair_button.set_sensitive(False)
        self.test_crop_view_pair_button.connect("clicked", lambda _b: self._view_test_crop_pair())
        bounds_box.pack_start(self.test_crop_view_pair_button, False, False, 0)
        row = self._row(bounds_box)
        self.test_crop_pick_button = Gtk.Button(label="Elegir 1 punto de la pared...")
        self.test_crop_pick_button.set_tooltip_text(
            "Mantén Shift y haz clic sobre la pared; otro Shift + clic reemplaza el punto."
        )
        self.test_crop_pick_button.connect("clicked", lambda _b: self._pick_test_crop_seed())
        self.test_crop_pick_button.set_sensitive(False)
        row.pack_start(self.test_crop_pick_button, False, False, 0)
        self.test_crop_bounds_label = Gtk.Label(label="Limites sin definir", xalign=0)
        row.pack_start(self.test_crop_bounds_label, False, False, 0)

        auto_row = self._row(bounds_box)
        self.test_crop_auto_button = Gtk.Button(label="Detectar pared frontal automáticamente")
        self.test_crop_auto_button.set_tooltip_text(
            "Busca el punto más cercano al centro de la vista del sensor y estima el box desde esa dirección."
        )
        self.test_crop_auto_button.connect("clicked", lambda _b: self._auto_detect_crop_wall())
        self.test_crop_auto_button.set_sensitive(False)
        auto_row.pack_start(self.test_crop_auto_button, False, False, 0)

        row = self._row(bounds_box)
        row.pack_start(Gtk.Label(label="Lado de la cara cuadrada (cm):"), False, False, 0)
        self.test_crop_face_size_entry = self._numeric_entry("20", 7)
        self.test_crop_face_size_entry.set_tooltip_text(
            "Controla el ancho y el alto iguales de la cara cuadrada, centrada en el punto elegido."
        )
        self.test_crop_face_size_entry.connect("changed", lambda _entry: self._on_test_crop_face_size_changed())
        self.test_crop_face_size_entry.connect(
            "changed", lambda entry: self._sync_prepare_crop_dimension(entry, "face")
        )
        row.pack_start(self.test_crop_face_size_entry, False, False, 0)

        row = self._row(bounds_box)
        row.pack_start(Gtk.Label(label="Profundidad (cm):"), False, False, 0)
        self.test_crop_depth_spin = Gtk.Entry()
        self.test_crop_depth_spin.set_text("100")
        self.test_crop_depth_spin.set_width_chars(8)
        self.test_crop_depth_spin.set_input_purpose(Gtk.InputPurpose.NUMBER)
        self.test_crop_depth_spin.connect("changed", lambda _entry: self._on_test_crop_depth_changed())
        self.test_crop_depth_spin.connect(
            "changed", lambda entry: self._sync_prepare_crop_dimension(entry, "depth")
        )
        row.pack_start(self.test_crop_depth_spin, False, False, 0)
        self.test_crop_preview_button = Gtk.Button(label="Vista previa del box")
        self.test_crop_preview_button.connect("clicked", lambda _button: self._preview_test_crop_bounds())
        row.pack_start(self.test_crop_preview_button, False, False, 0)
        row.pack_start(
            self._info_button(
                "Elige un punto con Shift + clic; otro Shift + clic antes de cerrar lo reemplaza. "
                "También puedes detectar automáticamente la pared frontal. Ajusta la profundidad "
                "y actualiza la vista previa sin volver a elegir el punto."
            ),
            False,
            False,
            0,
        )
        hint = Gtk.Label(
            label=(
                "En la selección manual, usa Shift + clic sobre la pared (gris = ANTES, colores = espesor DESPUES); "
                "otro Shift + clic reemplaza el punto. La opción automática busca la pared más cercana al centro "
                "de la vista del sensor. Luego ajusta la profundidad y actualiza el box."
            ),
            xalign=0,
        )
        hint.set_line_wrap(True)
        self.test_crop_hint_label = hint
        bounds_box.pack_start(hint, False, False, 0)
        page.pack_start(bounds_frame, False, True, 0)

        output_frame, output_box = self._section("2. Aplicar el recorte a las capturas")
        self.test_crop_output_dir_label = Gtk.Label(xalign=0)
        self._set_test_crop_output_dir_label()
        output_box.pack_start(self.test_crop_output_dir_label, False, False, 0)
        self.test_crop_apply_button = Gtk.Button(label="Guardar recortes ANTES + DESPUÉS")
        self.test_crop_apply_button.get_style_context().add_class("suggested-action")
        self.test_crop_apply_button.set_sensitive(False)
        self.test_crop_apply_button.set_tooltip_text(
            "Guarda ambas nubes recortadas en data/recortes, sin modificar los PLY originales."
        )
        self.test_crop_apply_button.connect(
            "clicked", lambda _b: self._apply_test_crop(overwrite_inputs=False)
        )
        output_box.pack_start(self.test_crop_apply_button, False, False, 0)
        self.test_crop_status_label = Gtk.Label(
            label="Selecciona ambas nubes en Medicion rapida para empezar.", xalign=0
        )
        self.test_crop_status_label.set_line_wrap(True)
        output_box.pack_start(self.test_crop_status_label, False, False, 0)

        self.test_crop_use_pair_button = Gtk.Button(label="Usar el par recortado en Medicion rapida")
        self.test_crop_use_pair_button.set_sensitive(False)
        self.test_crop_use_pair_button.connect("clicked", lambda _b: self._use_test_crop_pair())
        self.test_crop_use_pair_button.set_no_show_all(True)
        self.test_crop_use_pair_button.hide()
        output_box.pack_start(self.test_crop_use_pair_button, False, False, 0)
        page.pack_start(output_frame, False, True, 0)

        self.test_crop_preview_button.set_sensitive(False)
        self._refresh_test_crop_controls()

        return self._scrolled(page)

    def _get_test_crop_depth_m(self) -> float:
        value_text = self.test_crop_depth_spin.get_text().strip().replace(",", ".")
        try:
            value_cm = float(value_text)
        except ValueError as exc:
            raise ValueError("Introduce una profundidad numérica mayor que cero.") from exc
        if not np.isfinite(value_cm) or value_cm <= 0:
            raise ValueError("La profundidad debe ser mayor que cero.")
        return value_cm / 100.0

    def _sync_quick_crop_dimension(self, entry, dimension: str) -> None:
        if getattr(self, "_syncing_crop_dimensions", False):
            return
        attr_by_dimension = {
            "width": "test_crop_face_size_entry",
            "height": "test_crop_height_entry",
            "depth": "test_crop_depth_spin",
        }
        target = getattr(self, attr_by_dimension[dimension], None)
        if target is None or target.get_text() == entry.get_text():
            return
        self._syncing_crop_dimensions = True
        try:
            target.set_text(entry.get_text())
        finally:
            self._syncing_crop_dimensions = False

    def _sync_prepare_crop_dimension(self, entry, dimension: str) -> None:
        quick_entry = getattr(
            self,
            "quick_crop_face_size_entry" if dimension == "face" else "quick_crop_depth_entry",
            None,
        )
        if quick_entry is None or quick_entry.get_text() == entry.get_text():
            return
        self._syncing_crop_dimensions = True
        try:
            quick_entry.set_text(entry.get_text())
        finally:
            self._syncing_crop_dimensions = False

    def _get_test_crop_face_size_m(self) -> float:
        value_text = self.test_crop_face_size_entry.get_text().strip().replace(",", ".")
        try:
            value_cm = float(value_text)
        except ValueError as exc:
            raise ValueError("Introduce un ancho de box numérico mayor que cero.") from exc
        if not np.isfinite(value_cm) or value_cm <= 0:
            raise ValueError("El ancho del box debe ser mayor que cero.")
        return value_cm / 100.0

    def _get_test_crop_height_m(self) -> float:
        value_text = self.test_crop_height_entry.get_text().strip().replace(",", ".")
        try:
            value_cm = float(value_text)
        except ValueError as exc:
            raise ValueError("Introduce un alto de box numérico mayor que cero.") from exc
        if not np.isfinite(value_cm) or value_cm <= 0:
            raise ValueError("El alto del box debe ser mayor que cero.")
        return value_cm / 100.0

    def _on_test_crop_face_size_changed(self) -> None:
        if self.test_crop_seed is None or self.test_crop_bounds is None:
            return
        try:
            width_m = self._get_test_crop_face_size_m()
            height_m = self._get_test_crop_height_m()
            self.test_crop_bounds = rectangular_face_bounds(
                self.test_crop_bounds, self.test_crop_seed, width_m, height_m
            )
        except ValueError:
            return
        self.test_crop_output_paths = None
        self.test_crop_use_pair_button.set_sensitive(False)
        low, high = (np.asarray(axis, dtype=np.float64) for axis in self.test_crop_bounds)
        depth_axis = int(np.argmin(high - low))
        axis_name = ("X", "Y", "Z")[depth_axis]
        self.test_crop_bounds_label.set_text(
            f"Cara {width_m * 100:g} × {height_m * 100:g} cm · profundidad eje {axis_name}"
        )
        self.test_crop_status_label.set_text(
            "Dimensiones actualizadas; pulsa «Actualizar vista previa» para aplicar el nuevo tamaño."
        )
        self._refresh_test_crop_controls()

    def _on_test_crop_depth_changed(self) -> None:
        if self.test_crop_bounds is None:
            return
        self.test_crop_output_paths = None
        self.test_crop_use_pair_button.set_sensitive(False)
        self.test_crop_status_label.set_text(
            "Profundidad cambiada; se conserva el punto y la pared detectada. Pulsa «Actualizar vista previa» para ver el nuevo box."
        )
        self._refresh_test_crop_controls()

    def _set_test_crop_output_dir_label(self) -> None:
        if not hasattr(self, "test_crop_output_dir_label"):
            return
        if self.test_crop_output_dir is None:
            self.test_crop_output_dir_label.set_text("Destino: data/recortes")
            self.test_crop_output_dir_label.set_tooltip_text("")
        else:
            self.test_crop_output_dir_label.set_text(f"Destino: {self.test_crop_output_dir}")
            self.test_crop_output_dir_label.set_tooltip_text(str(self.test_crop_output_dir))

    def _choose_test_crop_output_dir(self) -> None:
        dialog = Gtk.FileChooserNative.new(
            "Elegir carpeta para los dos recortes",
            self.window,
            Gtk.FileChooserAction.SELECT_FOLDER,
            "_Seleccionar",
            "_Cancelar",
        )
        if self.test_crop_output_dir and self.test_crop_output_dir.is_dir():
            dialog.set_current_folder(str(self.test_crop_output_dir))
        path = self._run_native_file_chooser(dialog)
        if path:
            self.test_crop_output_dir = Path(path)
            self.test_crop_output_dir_is_custom = True
            self._set_test_crop_output_dir_label()

    def _refresh_test_crop_controls(self) -> None:
        has_before = bool(self.test_crop_before_path and Path(self.test_crop_before_path).is_file())
        has_after = bool(self.test_crop_after_path and Path(self.test_crop_after_path).is_file())
        has_pair = has_before and has_after
        self.test_crop_view_pair_button.set_sensitive(has_pair)
        self.test_crop_pick_button.set_sensitive(has_pair)
        self.test_crop_auto_button.set_sensitive(has_pair)
        has_region = has_pair and self.test_crop_bounds is not None
        self.test_crop_preview_button.set_sensitive(has_region)
        self.test_crop_apply_button.set_sensitive(has_region)
        self.test_crop_use_pair_button.set_sensitive(
            bool(self.test_crop_output_paths and all(Path(p).is_file() for p in self.test_crop_output_paths))
        )
        if hasattr(self, "quick_crop_action_button"):
            self.quick_crop_action_button.set_sensitive(has_pair)
            self.quick_crop_preview_button.set_sensitive(bool(has_region))
            self.quick_crop_apply_button.set_sensitive(bool(has_region))
            if not has_pair:
                self.quick_crop_status_label.set_text(
                    "Carga ANTES y DESPUES para definir la region."
                )

    def _load_test_crop_pair(self, compensate_pose: bool = True):
        if not self.test_crop_before_path or not self.test_crop_after_path:
            raise RuntimeError("Selecciona ANTES y DESPUES en Medicion rapida primero.")
        before_path = self._crop_source_path(Path(self.test_crop_before_path))
        after_path = self._crop_source_path(Path(self.test_crop_after_path))
        before = load_point_cloud(before_path)
        after = load_point_cloud(after_path)
        before_pose = load_reference_pose(before_path)
        after_pose = load_reference_pose(after_path)
        if compensate_pose and before_pose is not None and after_pose is not None:
            rotation, translation = compute_pose_relative_transform(before_pose, after_pose)
            after = apply_rigid_transform(after, rotation, translation)
        return before, after

    @staticmethod
    def _crop_original_backup_path(path: Path) -> Path:
        return path.with_name(f".{path.stem}.aurora_original{path.suffix}")

    def _crop_source_path(self, selected_path: Path) -> Path:
        """Use the first full capture again when redefining an applied crop."""
        original = self._crop_original_backup_path(selected_path)
        return original if original.is_file() else selected_path

    def _preserve_crop_original(self, path: Path) -> None:
        """Keep one hidden, durable original plus its pose/photo sidecars."""
        backup = self._crop_original_backup_path(path)
        if backup.exists():
            return
        shutil.copy2(path, backup)
        for suffix in ("_pose.npz", "_ref.npz"):
            sidecar = path.with_name(path.stem + suffix)
            backup_sidecar = backup.with_name(backup.stem + suffix)
            if sidecar.is_file() and not backup_sidecar.exists():
                shutil.copy2(sidecar, backup_sidecar)

    def _view_test_crop_pair(self) -> None:
        if not self.test_crop_before_path or not self.test_crop_after_path:
            self._show_warning(
                "Faltan capturas",
                "Selecciona ANTES y DESPUES en Medicion rapida primero.",
            )
            return
        try:
            heatmap_spec = self._selected_preview_heatmap_spec()
        except ValueError as exc:
            self._show_error("Parámetros de espesor no válidos", str(exc))
            return
        self.test_crop_view_pair_button.set_sensitive(False)
        compensate_pose = self.quick_imu_compensate_check.get_active()
        temp_dir = Path(tempfile.mkdtemp(prefix="aurora_pose_pair_view_"))

        def worker():
            try:
                import open3d as o3d

                before, after = self._load_test_crop_pair(compensate_pose)
                before_path = temp_dir / "antes.ply"
                after_path = temp_dir / "despues.ply"
                if not o3d.io.write_point_cloud(str(before_path), before):
                    raise RuntimeError("No se pudo preparar la nube ANTES para la vista.")
                if not o3d.io.write_point_cloud(str(after_path), after):
                    raise RuntimeError("No se pudo preparar la nube DESPUES para la vista.")
                self._ui(
                    self._launch_viewer_process,
                    "pair",
                    [str(before_path), str(after_path)],
                    "Aurora - Antes y despues (superpuestos)",
                    "No se pudieron abrir las dos capturas",
                    str(temp_dir),
                    None,
                    None,
                    heatmap_spec,
                )
            except Exception as exc:
                shutil.rmtree(temp_dir, ignore_errors=True)
                self._ui(
                    self._show_error,
                    "No se pudieron abrir las dos capturas",
                    str(exc),
                )
            finally:
                self._ui(self.test_crop_view_pair_button.set_sensitive, True)

        threading.Thread(target=worker, daemon=True).start()

    def _selected_preview_heatmap_spec(self) -> dict:
        """Snapshot of the selected thickness palette for cloud previews."""
        if self.color_six_bands_rb.get_active():
            target_cm = self._parse_positive_entry(
                self.target_thickness_entry, "El espesor objetivo"
            )
            return {"mode": "six", "target_m": target_cm / 100.0}
        if self.color_banded_rb.get_active():
            low_cm, high_cm = self._band_thresholds_cm()
            return {"mode": "three", "low_m": low_cm / 100.0, "high_m": high_cm / 100.0}
        text = self.max_distance_entry.get_text().strip().replace(",", ".")
        max_distance_m = float(text) if text else 0.0
        if not np.isfinite(max_distance_m) or max_distance_m < 0:
            raise ValueError("La distancia máxima debe ser cero o mayor.")
        return {
            "mode": "continuous",
            "max_distance_m": max_distance_m if max_distance_m > 0 else None,
        }

    def _preview_thickness_legend(
        self, heatmap_spec: dict, before_cloud, after_cloud
    ) -> dict | None:
        """Build the same legend as the selected preview palette, without a saved result."""
        if not len(after_cloud.points):
            return None
        mode = heatmap_spec.get("mode", "continuous")
        if mode == "six":
            target_cm = float(heatmap_spec["target_m"]) * 100.0
            band_width_cm = target_cm / len(SIX_BAND_LABELS)
            rows = []
            for index, (label, color) in enumerate(zip(SIX_BAND_LABELS, SIX_BAND_COLORS)):
                lower = index * band_width_cm
                value = (
                    f">= {lower:.1f} cm (tope {target_cm:.1f})"
                    if index == len(SIX_BAND_LABELS) - 1
                    else f"{lower:.1f}-{(index + 1) * band_width_cm:.1f} cm"
                )
                rows.append((color, label, value))
            return {"title": "Espesor · 6 niveles", "rows": rows}
        if mode == "three":
            low_cm = float(heatmap_spec["low_m"]) * 100.0
            high_cm = float(heatmap_spec["high_m"]) * 100.0
            return {
                "title": "Espesor · 3 niveles",
                "rows": [
                    (BAND_COLOR_LOW, "Bajo", f"< {low_cm:g} cm"),
                    (BAND_COLOR_MEDIUM, "Intermedio", f"{low_cm:g}-{high_cm:g} cm"),
                    (BAND_COLOR_HIGH, "Alto", f">= {high_cm:g} cm"),
                ],
            }
        distances = compute_c2c_distance(after_cloud, before_cloud)
        maximum = heatmap_spec.get("max_distance_m")
        if maximum is None:
            maximum = float(np.percentile(distances, 98)) if distances.size else 1e-9
        return {
            "title": "Espesor · escala continua",
            "continuous_max_cm": max(float(maximum), 1e-9) * 100.0,
            "low_color": HEATMAP_COLOR_LOW,
            "high_color": HEATMAP_COLOR_HIGH,
        }

    def _pick_test_crop_seed(self) -> None:
        self.test_crop_pick_button.set_sensitive(False)
        self.test_crop_auto_button.set_sensitive(False)
        compensate_pose = self.quick_imu_compensate_check.get_active()
        if hasattr(self, "quick_crop_action_button"):
            self.quick_crop_action_button.set_sensitive(False)
            self.quick_crop_status_label.set_text("Abriendo la vista 3D para elegir un punto...")
        try:
            heatmap_spec = self._selected_preview_heatmap_spec()
        except ValueError as exc:
            self.test_crop_pick_button.set_sensitive(True)
            self.test_crop_auto_button.set_sensitive(True)
            if hasattr(self, "quick_crop_action_button"):
                self.quick_crop_action_button.set_sensitive(True)
                self.quick_crop_status_label.set_text("Revisa la configuracion de colores antes de seleccionar.")
            self._show_error("Parámetros de espesor no válidos", str(exc))
            return

        def worker():
            try:
                before, after = self._load_test_crop_pair(compensate_pose)
                seed = pick_crop_seed_from_pair(before, after, heatmap_spec)
                if seed is None:
                    self._ui(self._on_test_crop_wall_detected, None, None, "manual", None)
                    return
                seed, bounds = estimate_wall_crop_bounds(before, after, seed)
                self._ui(self._on_test_crop_wall_detected, seed, bounds, "manual", None)
            except Exception as exc:
                self._ui(self._on_test_crop_wall_detected, None, None, "manual", str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _auto_detect_crop_wall(self) -> None:
        self.test_crop_pick_button.set_sensitive(False)
        self.test_crop_auto_button.set_sensitive(False)
        if hasattr(self, "quick_crop_action_button"):
            self.quick_crop_action_button.set_sensitive(False)
            self.quick_crop_status_label.set_text("Detectando automaticamente la pared frontal...")
        self.test_crop_status_label.set_text(
            "Buscando el punto más cercano al centro de la vista y ajustando la pared..."
        )
        compensate_pose = self.quick_imu_compensate_check.get_active()

        def worker():
            try:
                before, after = self._load_test_crop_pair(compensate_pose)
                seed, bounds = estimate_wall_crop_bounds(before, after)
                self._ui(self._on_test_crop_wall_detected, seed, bounds, "automático", None)
            except Exception as exc:
                self._ui(self._on_test_crop_wall_detected, None, None, "automático", str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_test_crop_wall_detected(self, seed, bounds, source: str, error: str | None) -> None:
        self.test_crop_pick_button.set_sensitive(True)
        self.test_crop_auto_button.set_sensitive(True)
        if hasattr(self, "quick_crop_action_button"):
            has_pair = bool(
                self.test_crop_before_path and Path(self.test_crop_before_path).is_file()
                and self.test_crop_after_path and Path(self.test_crop_after_path).is_file()
            )
            self.quick_crop_action_button.set_sensitive(has_pair)
        if error:
            if hasattr(self, "quick_crop_status_label"):
                self.quick_crop_status_label.set_text(f"No se pudo definir la region: {error}")
            self._show_error("No se pudo detectar la pared", error)
            return
        if seed is None or bounds is None:
            if source == "manual":
                self._show_info("Sin punto seleccionado", "Haz clic sobre la pared o cierra la vista para cancelar.")
            return
        self.test_crop_seed = np.asarray(seed, dtype=np.float64).copy()
        try:
            self.test_crop_bounds = rectangular_face_bounds(
                bounds,
                self.test_crop_seed,
                self._get_test_crop_face_size_m(),
                self._get_test_crop_height_m(),
            )
        except ValueError as exc:
            self._show_error("Tamaño del box no válido", str(exc))
            return
        self.test_crop_preview_button.set_label("Actualizar vista previa")
        self.test_crop_output_paths = None
        self.test_crop_use_pair_button.set_sensitive(False)
        low, high = (np.asarray(axis, dtype=np.float64) for axis in self.test_crop_bounds)
        depth_axis = int(np.argmin(high - low))
        in_plane_axes = [axis for axis in range(3) if axis != depth_axis]
        width_cm = float(high[in_plane_axes[0]] - low[in_plane_axes[0]]) * 100
        height_cm = float(high[in_plane_axes[1]] - low[in_plane_axes[1]]) * 100
        axis_name = ("X", "Y", "Z")[depth_axis]
        self.test_crop_bounds_label.set_text(
            f"Cara {width_cm:.0f} × {height_cm:.0f} cm · profundidad eje {axis_name}"
        )
        self.test_crop_status_label.set_text(
            "Punto guardado. Puedes ajustar la profundidad; no necesitas seleccionarlo otra vez."
        )
        if hasattr(self, "quick_crop_status_label"):
            self.quick_crop_status_label.set_text(
                f"Region definida ({source}). Puedes previsualizarla o recortar y guardar ambas nubes."
            )
        self._refresh_test_crop_controls()
        self._preview_test_crop_bounds()

    def _preview_test_crop_bounds(self) -> None:
        if self.test_crop_bounds is None:
            return
        try:
            depth_m = self._get_test_crop_depth_m()
            bounds = tuple(np.asarray(axis, dtype=np.float64).copy() for axis in self.test_crop_bounds)
            seed = np.asarray(self.test_crop_seed, dtype=np.float64).copy()
            heatmap_spec = self._selected_preview_heatmap_spec()
        except Exception as exc:
            self.test_crop_status_label.set_text("No se pudo previsualizar el par recortado.")
            self._show_error("Error en la vista previa de las dos nubes", str(exc))
            return

        self.test_crop_preview_button.set_sensitive(False)
        depth_cm = depth_m * 100.0
        self.test_crop_status_label.set_text(
            f"Actualizando el box a {depth_cm:g} cm alrededor de la pared detectada..."
        )
        if hasattr(self, "quick_crop_status_label"):
            self.quick_crop_status_label.set_text("Generando vista previa del recorte...")
        compensate_pose = self.quick_imu_compensate_check.get_active()

        def worker():
            try:
                before, after = self._load_test_crop_pair(compensate_pose)
                before_cut, after_cut, effective_bounds, box_bounds, depth_axis, margin = crop_pair_by_face_depth(
                    before, after, bounds, depth_m, seed
                )
                self._ui(
                    self._on_test_crop_preview_ready,
                    before_cut,
                    after_cut,
                    box_bounds,
                    heatmap_spec,
                    depth_cm,
                    effective_bounds,
                    depth_axis,
                    margin,
                    before,
                    after,
                )
            except Exception as exc:
                self._ui(self._on_test_crop_preview_failed, str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_test_crop_preview_ready(
        self, before_cut, after_cut, box_bounds, heatmap_spec, depth_cm: float,
        effective_bounds, depth_axis: int, margin: float, before_source, after_source,
    ) -> None:
        self.test_crop_preview_button.set_sensitive(True)
        self.test_crop_bounds = effective_bounds
        axis_name = ("X", "Y", "Z")[depth_axis]
        empty_hint = (
            " Aumenta la profundidad para incluir ANTES."
            if not len(before_cut.points) or not len(after_cut.points)
            else ""
        )
        self.test_crop_status_label.set_text(
            f"Cara {self._get_test_crop_face_size_m() * 100:g} × "
            f"{self._get_test_crop_height_m() * 100:g} cm; profundidad {depth_cm:g} cm en eje {axis_name}. "
            f"Dentro del box — ANTES: {len(before_cut.points):,}; DESPUES: {len(after_cut.points):,}.{empty_hint}"
        )
        if hasattr(self, "quick_crop_status_label"):
            self.quick_crop_status_label.set_text(
                f"Vista previa lista · ANTES: {len(before_cut.points):,} puntos; "
                f"DESPUES: {len(after_cut.points):,} puntos dentro del box."
            )
        temp_dir = Path(tempfile.mkdtemp(prefix="aurora_crop_preview_"))

        def worker():
            try:
                import open3d as o3d

                before_path = temp_dir / "antes.ply"
                after_path = temp_dir / "despues.ply"
                camera_before_path = temp_dir / "camara_antes.ply"
                camera_after_path = temp_dir / "camara_despues.ply"
                if not o3d.io.write_point_cloud(str(camera_before_path), before_source):
                    raise RuntimeError("No se pudo preparar la referencia de cámara ANTES.")
                if not o3d.io.write_point_cloud(str(camera_after_path), after_source):
                    raise RuntimeError("No se pudo preparar la referencia de cámara DESPUES.")
                empty_stages = []
                if len(before_cut.points):
                    if not o3d.io.write_point_cloud(str(before_path), before_cut):
                        raise RuntimeError("No se pudo preparar la nube ANTES para la vista.")
                else:
                    before_path = camera_before_path
                    empty_stages.append("antes")
                if len(after_cut.points):
                    if not o3d.io.write_point_cloud(str(after_path), after_cut):
                        raise RuntimeError("No se pudo preparar la nube DESPUES para la vista.")
                else:
                    after_path = camera_after_path
                    empty_stages.append("despues")
                legend = self._preview_thickness_legend(
                    heatmap_spec,
                    before_cut if len(before_cut.points) else before_source,
                    after_cut,
                )
                self._ui(
                    self._launch_test_crop_preview_viewer,
                    temp_dir,
                    str(before_path),
                    str(after_path),
                    box_bounds,
                    heatmap_spec,
                    [str(camera_before_path), str(camera_after_path)],
                    empty_stages,
                    legend,
                )
            except Exception as exc:
                shutil.rmtree(temp_dir, ignore_errors=True)
                self._ui(self._show_error, "Error en la vista previa de las dos nubes", str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _launch_test_crop_preview_viewer(
        self, temp_dir: Path, before_path: str, after_path: str, box_bounds, heatmap_spec,
        camera_reference_paths: list[str], empty_stages: list[str], legend: dict | None,
    ) -> None:
        self._close_test_crop_preview_viewer()
        process = self._launch_viewer_process(
            "pair_box",
            [before_path, after_path],
            "Aurora - Vista previa del box",
            "Error en la vista previa de las dos nubes",
            str(temp_dir),
            legend=legend,
            box_bounds=box_bounds,
            heatmap_spec=heatmap_spec,
            camera_reference_paths=camera_reference_paths,
            empty_stages=empty_stages,
        )
        if process is not None:
            self.test_crop_preview_process = process
            self.test_crop_preview_temp_dir = temp_dir

    def _close_test_crop_preview_viewer(self) -> None:
        process = self.test_crop_preview_process
        self.test_crop_preview_process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=2.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if self.test_crop_preview_temp_dir is not None:
            shutil.rmtree(self.test_crop_preview_temp_dir, ignore_errors=True)
        self.test_crop_preview_temp_dir = None

    def _on_test_crop_preview_failed(self, error: str) -> None:
        self.test_crop_preview_button.set_sensitive(True)
        self.test_crop_status_label.set_text("No se pudo previsualizar el par recortado.")
        if hasattr(self, "quick_crop_status_label"):
            self.quick_crop_status_label.set_text("No se pudo generar la vista previa del recorte.")
        self._show_error("Error en la vista previa de las dos nubes", error)

    def _apply_test_crop(
        self, overwrite_inputs: bool = False, calculate_after: bool = False
    ) -> None:
        if not self.test_crop_before_path or not self.test_crop_after_path or self.test_crop_bounds is None:
            self._show_warning(
                "Faltan datos",
                "Selecciona ANTES y DESPUES en Medicion rapida y define la region antes de guardar.",
            )
            return
        selected_before = Path(self.test_crop_before_path)
        selected_after = Path(self.test_crop_after_path)
        if overwrite_inputs and selected_before.resolve() == selected_after.resolve():
            self._show_error(
                "Capturas repetidas",
                "ANTES y DESPUES apuntan al mismo archivo. Selecciona dos nubes distintas para sobrescribirlas.",
            )
            return
        output_dir = self.test_crop_output_dir or selected_before.parent

        if overwrite_inputs:
            before_destination = selected_before
            after_destination = selected_after
        else:
            suffix = ""
            index = 2
            before_destination = output_dir / f"ANTES_{selected_before.stem}_recortada.ply"
            after_destination = output_dir / f"DESPUES_{selected_after.stem}_recortada.ply"
            while before_destination.exists() or after_destination.exists():
                suffix = f"_{index}"
                before_destination = output_dir / f"ANTES_{selected_before.stem}_recortada{suffix}.ply"
                after_destination = output_dir / f"DESPUES_{selected_after.stem}_recortada{suffix}.ply"
                index += 1

        before_source = self._crop_source_path(selected_before)
        after_source = self._crop_source_path(selected_after)

        bounds = self.test_crop_bounds
        compensate_pose = self.quick_imu_compensate_check.get_active()
        try:
            depth_m = self._get_test_crop_depth_m()
        except ValueError as exc:
            self._show_error("Profundidad no válida", str(exc))
            return
        if not overwrite_inputs:
            try:
                output_dir.mkdir(parents=True, exist_ok=True)
            except OSError as exc:
                self._show_error("No se pudo crear la carpeta de recortes", str(exc))
                return
        for button in (
            self.test_crop_pick_button,
            self.test_crop_preview_button,
            self.test_crop_apply_button,
        ):
            button.set_sensitive(False)
        self.test_crop_status_label.set_text("Recortando y guardando ambas nubes...")

        def worker() -> None:
            before_temp = before_destination.with_name(f".{before_destination.stem}.{os.getpid()}.tmp.ply")
            after_temp = after_destination.with_name(f".{after_destination.stem}.{os.getpid()}.tmp.ply")
            before_backup = before_destination.with_name(f".{before_destination.stem}.{os.getpid()}.backup")
            after_backup = after_destination.with_name(f".{after_destination.stem}.{os.getpid()}.backup")
            try:
                before_source_cloud = load_point_cloud(before_source)
                after_source_cloud = load_point_cloud(after_source)
                before_pose = load_reference_pose(before_source)
                after_pose = load_reference_pose(after_source)
                pose_transform = None
                if compensate_pose and before_pose is not None and after_pose is not None:
                    rotation, translation = compute_pose_relative_transform(before_pose, after_pose)
                    after_source_cloud = apply_rigid_transform(
                        after_source_cloud, rotation, translation
                    )
                    pose_transform = (rotation, translation)
                before, after, _effective_bounds, _box_bounds, _depth_axis, _margin = crop_pair_by_face_depth(
                    before_source_cloud, after_source_cloud, bounds, depth_m, self.test_crop_seed
                )
                if not len(before.points) or not len(after.points):
                    raise RuntimeError(
                        "El box no contiene puntos en ambas capturas. Aumenta la profundidad, el ancho o el alto."
                    )
                import open3d as o3d

                if not o3d.io.write_point_cloud(str(before_temp), before):
                    raise RuntimeError("Open3D no pudo guardar el recorte ANTES.")
                if not o3d.io.write_point_cloud(str(after_temp), after):
                    raise RuntimeError("Open3D no pudo guardar el recorte DESPUES.")
                if overwrite_inputs:
                    if before_backup.exists() or after_backup.exists():
                        raise RuntimeError("Ya existe un respaldo temporal de esta operación; no se sobrescribió nada.")
                    self._preserve_crop_original(before_destination)
                    self._preserve_crop_original(after_destination)
                    os.replace(before_destination, before_backup)
                    os.replace(after_destination, after_backup)
                    os.replace(before_temp, before_destination)
                    os.replace(after_temp, after_destination)
                    for backup in (before_backup, after_backup):
                        try:
                            backup.unlink(missing_ok=True)
                        except OSError:
                            pass
                else:
                    before_temp.replace(before_destination)
                    after_temp.replace(after_destination)

                # The crop box is defined in ANTES coordinates. If pose data
                # was available, DESPUES has already been transformed into
                # that same frame, so both saved clouds and their sidecars must
                # now describe the common ANTES frame.
                try:
                    if before_pose is not None:
                        save_reference_pose(before_destination, *before_pose)
                    if pose_transform is not None and before_pose is not None:
                        save_reference_pose(
                            after_destination, *before_pose, alignment_version=2
                        )
                    elif after_pose is not None:
                        save_reference_pose(after_destination, *after_pose)

                    before_photo = load_reference_photo(before_source)
                    if before_photo is not None and before_destination.resolve() != before_source.resolve():
                        save_reference_photo(before_destination, *before_photo)
                    after_photo_path = reference_photo_path_for(after_source)
                    if after_photo_path.exists():
                        if pose_transform is not None:
                            save_transformed_reference_photo(
                                after_photo_path,
                                after_destination,
                                pose_transform[0],
                                pose_transform[1],
                            )
                        elif after_destination.resolve() != after_source.resolve():
                            after_photo = load_reference_photo(after_source)
                            if after_photo is not None:
                                save_reference_photo(after_destination, *after_photo)
                except Exception as metadata_exc:
                    self._ui(
                        self._log,
                        f"Aviso: se recortaron las nubes, pero no se pudo actualizar un sidecar de pose/foto: {metadata_exc}",
                    )
                if pose_transform is not None:
                    self._ui(
                        self._log,
                        "Se compensó la pose de DESPUÉS respecto de ANTES antes de aplicar el mismo box de recorte.",
                    )
            except Exception as exc:
                if overwrite_inputs:
                    for destination, backup in (
                        (before_destination, before_backup),
                        (after_destination, after_backup),
                    ):
                        if backup.exists():
                            destination.unlink(missing_ok=True)
                            os.replace(backup, destination)
                before_temp.unlink(missing_ok=True)
                after_temp.unlink(missing_ok=True)
                self._ui(self._on_test_crop_failed, str(exc))
                return
            self._ui(
                self._on_test_crop_done,
                str(before_destination),
                str(after_destination),
                len(before.points),
                len(after.points),
                overwrite_inputs,
                calculate_after,
            )

        threading.Thread(target=worker, daemon=True).start()

    def _on_test_crop_done(
        self,
        before_path: str,
        after_path: str,
        before_count: int,
        after_count: int,
        overwrite_inputs: bool = False,
        calculate_after: bool = False,
    ) -> None:
        if overwrite_inputs:
            # Re-select the same paths to invalidate any previous calculation and
            # synchronize every section with the newly cropped files.
            self._set_base_path(before_path)
            self._set_updated_path(after_path)
            for label, path in (
                (self.quick_base_path_label, before_path),
                (self.quick_updated_path_label, after_path),
            ):
                escaped_name = GLib.markup_escape_text(Path(path).name)
                label.set_markup(
                    f'<span foreground="{COLOR_FILE_NAME}">{escaped_name} · RECORTADA</span>'
                )
                label.set_tooltip_text(f"{path}\nRecorte aplicado y sobrescrito")
        self.test_crop_output_paths = (before_path, after_path)
        self._refresh_test_crop_controls()
        if overwrite_inputs:
            message = (
                f"Recorte aplicado sobre ANTES y DESPUES · {before_count:,} y "
                f"{after_count:,} puntos. Los archivos seleccionados fueron sobrescritos."
            )
        else:
            message = (
                f"Par recortado guardado: {Path(before_path).name} ({before_count:,}) y "
                f"{Path(after_path).name} ({after_count:,}). Los originales siguen intactos."
            )
        self.test_crop_status_label.set_text(message)
        if hasattr(self, "quick_crop_status_label"):
            self.quick_crop_status_label.set_text(message)
        if calculate_after and not overwrite_inputs:
            self._set_base_path(before_path)
            self._set_updated_path(after_path)
        if calculate_after:
            self._quick_calculate()

    def _on_test_crop_failed(self, message: str) -> None:
        self._refresh_test_crop_controls()
        self.test_crop_status_label.set_text("No se pudieron guardar las dos nubes recortadas.")
        self._show_error("Error al recortar el par", message)

    def _use_test_crop_pair(self) -> None:
        pair = self.test_crop_output_paths
        if not pair or not all(Path(p).is_file() for p in pair):
            self._show_warning("No hay par recortado", "Recorta y guarda ambas nubes antes de usarlas.")
            return
        before_path, after_path = pair
        self._set_base_path(before_path)
        self._set_updated_path(after_path)
        self.stack.set_visible_child_name("medicion_rapida")

    # -- Parametros del calculo (integrados en Resultados) ------------------------

    def _build_analysis_controls(self) -> Gtk.Widget:
        return self._build_advanced_expander_page2()[0]

    def _build_advanced_expander_page2(self) -> tuple[Gtk.Widget, Gtk.Box]:
        expander = Gtk.Expander(label="Opciones avanzadas de Cloud-to-Cloud")
        expander.set_expanded(False)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.get_style_context().add_class("expander-content")
        box.set_margin_start(4)
        box.set_margin_top(10)
        box.set_margin_bottom(4)
        expander.add(box)

        self.c2c_voxel_row = self._row(box)
        row = self.c2c_voxel_row
        row.pack_start(Gtk.Label(label="Reducir densidad de puntos (voxel, en metros):"), False, False, 0)
        self.voxel_entry = Gtk.Entry()
        self.voxel_entry.set_text("0.0")
        self.voxel_entry.set_width_chars(10)
        row.pack_start(self.voxel_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "0 = sin reducir. Un valor como 0.01 (1cm) acelera el analisis en nubes muy "
                "densas, promediando puntos cercanos."
            ),
            False,
            False,
            0,
        )

        self.c2c_outliers_row = self._row(box)
        row = self.c2c_outliers_row
        self.remove_outliers_check = Gtk.CheckButton(label="Quitar ruido / puntos aislados")
        row.pack_start(self.remove_outliers_check, False, False, 0)
        row.pack_start(
            self._info_button("Descarta puntos que estan anormalmente lejos de sus vecinos (ruido del sensor)."),
            False,
            False,
            0,
        )

        row = self._row(box)
        self.use_icp_check = Gtk.CheckButton(label="Corregir alineacion entre capturas (ICP)")
        row.pack_start(self.use_icp_check, False, False, 0)
        row.pack_start(Gtk.Label(label="Umbral (m):"), False, False, 0)
        self.icp_threshold_entry = Gtk.Entry()
        self.icp_threshold_entry.set_text("0.05")
        self.icp_threshold_entry.set_width_chars(8)
        row.pack_start(self.icp_threshold_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "Usar solo si las dos capturas no quedaron en la misma posicion exacta "
                "(el sensor se movio levemente entre una y otra). No usar si lo que "
                "cambio de una nube a otra es precisamente el espesor que quieres medir "
                "en TODA la superficie, porque podria confundirse con error de alineacion."
            ),
            False,
            False,
            0,
        )

        row = self._row(box)
        row.pack_start(Gtk.Label(label="Carpeta donde guardar los resultados:"), False, False, 0)
        self.output_dir_label = Gtk.Label(xalign=0)
        self._set_path_label(self.output_dir_label, self.output_dir)
        row.pack_start(self.output_dir_label, True, True, 0)
        change_output_button = Gtk.Button(label="Cambiar...")
        change_output_button.connect("clicked", lambda _b: self._browse_output_dir())
        row.pack_start(change_output_button, False, False, 0)

        return expander, box

    # -- Controles de visualizacion integrados en Resultados ----------------------

    def _build_visualization_controls(self, page: Gtk.Box) -> None:

        color_frame, color_box = self._section("Color del espesor")

        row = self._row(color_box)
        self.color_continuous_rb = Gtk.RadioButton.new_with_label_from_widget(None, "Escala continua")
        self.color_continuous_rb.connect("toggled", lambda button: self._on_color_mode_changed() if button.get_active() else None)
        row.pack_start(self.color_continuous_rb, False, False, 0)
        self.color_banded_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.color_continuous_rb, "3 niveles de color"
        )
        self.color_banded_rb.connect("toggled", lambda button: self._on_color_mode_changed() if button.get_active() else None)
        row.pack_start(self.color_banded_rb, False, False, 0)
        self.color_six_bands_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.color_continuous_rb, "6 niveles (espesor objetivo)"
        )
        self.color_six_bands_rb.connect("toggled", lambda button: self._on_color_mode_changed() if button.get_active() else None)
        row.pack_start(self.color_six_bands_rb, False, False, 0)
        row.pack_start(
            self._info_button(
                "Escala continua: un degrade de color proporcional al espesor (azul = poco, "
                "rojo = mucho).\n\n3 niveles: clasifica cada punto en verde/amarillo/rojo "
                "segun los umbrales que definas abajo.\n\n6 niveles: divide el espesor "
                "objetivo en 6 tramos iguales con una escala termica (Muy Frio=morado, "
                "Frio=azul, Fresco=cian, Templado=verde, Calido=naranja, Caliente=rojo = "
                "objetivo alcanzado o superado). Util para ver de un vistazo el progreso de "
                "aplicacion del shotcrete durante la operacion."
            ),
            False,
            False,
            0,
        )

        self.band_revealer = Gtk.Revealer()
        self.band_revealer.set_transition_type(Gtk.RevealerTransitionType.NONE)
        band_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        band_row.pack_start(Gtk.Label(label="Hasta este espesor = verde (cm):"), False, False, 0)
        self.band_low_entry = Gtk.Entry()
        self.band_low_entry.set_text("5")
        self.band_low_entry.set_width_chars(8)
        band_row.pack_start(self.band_low_entry, False, False, 0)
        band_row.pack_start(Gtk.Label(label="Desde este espesor = rojo (cm):"), False, False, 0)
        self.band_high_entry = Gtk.Entry()
        self.band_high_entry.set_text("10")
        self.band_high_entry.set_width_chars(8)
        band_row.pack_start(self.band_high_entry, False, False, 0)
        self.band_revealer.add(band_row)
        color_box.pack_start(self.band_revealer, False, False, 0)

        self.six_bands_revealer = Gtk.Revealer()
        self.six_bands_revealer.set_transition_type(Gtk.RevealerTransitionType.NONE)
        six_bands_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        six_bands_row.pack_start(Gtk.Label(label="Espesor objetivo (cm):"), False, False, 0)
        self.target_thickness_entry = Gtk.Entry()
        self.target_thickness_entry.set_text("12")
        self.target_thickness_entry.set_width_chars(8)
        six_bands_row.pack_start(self.target_thickness_entry, False, False, 0)
        six_bands_row.pack_start(
            Gtk.Label(label="(6 tramos iguales: Muy Frio/Frio/Fresco/Templado/Calido/Caliente)"),
            False,
            False,
            0,
        )
        self.six_bands_revealer.add(six_bands_row)
        color_box.pack_start(self.six_bands_revealer, False, False, 0)

        self.color_banded_rb.set_active(True)  # despues de crear los revealers: dispara _on_color_mode_changed

        advanced_frame = Gtk.Expander(label="Opciones avanzadas")
        advanced_frame.set_expanded(False)
        adv_inner = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        adv_inner.get_style_context().add_class("expander-content")
        adv_inner.set_margin_start(4)
        adv_inner.set_margin_top(10)
        advanced_frame.add(adv_inner)
        row = self._row(adv_inner)
        row.pack_start(Gtk.Label(label="Escala continua, tope de la barra de color (m):"), False, False, 0)
        self.max_distance_entry = Gtk.Entry()
        self.max_distance_entry.set_width_chars(10)
        row.pack_start(self.max_distance_entry, False, False, 0)
        row.pack_start(
            self._info_button("Vacio = se ajusta automaticamente segun el espesor maximo encontrado."),
            False,
            False,
            0,
        )
        color_box.pack_start(advanced_frame, False, True, 6)

        page.pack_start(color_frame, False, True, 0)

        view_frame, view_box = self._section("Vista 3D en pantalla")

        row = self._row(view_box)
        self.show_updated_check = Gtk.CheckButton(label="Mostrar resultado sobre el tunel original")
        self.show_updated_check.set_active(True)
        self.show_updated_check.connect("toggled", lambda _b: self._on_show_updated_changed())
        row.pack_start(self.show_updated_check, False, False, 0)

        row = self._row(view_box)
        row.pack_start(Gtk.Label(label="Origen de la vista:"), False, False, 0)
        self.source_static_rb = Gtk.RadioButton.new_with_label_from_widget(None, "Ultima captura / archivo")
        self.source_static_rb.connect("toggled", lambda _b: self._on_updated_source_changed())
        row.pack_start(self.source_static_rb, False, False, 0)
        self.source_live_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.source_static_rb, "En vivo (sensor conectado)"
        )
        self.source_live_rb.connect("toggled", lambda _b: self._on_updated_source_changed())
        row.pack_start(self.source_live_rb, False, False, 0)
        row.pack_start(
            self._info_button(
                "'Ultima captura / archivo' muestra un resultado fijo. 'En vivo' redibuja "
                "continuamente lo que el sensor ve en este momento (necesita el sensor "
                "conectado)."
            ),
            False,
            False,
            0,
        )

        row = self._row(view_box)
        self.open_viewer_button = Gtk.Button(label="Abrir vista 3D")
        self.open_viewer_button.connect("clicked", lambda _b: self._open_viewer())
        row.pack_start(self.open_viewer_button, False, False, 0)
        self.close_viewer_button = Gtk.Button(label="Cerrar vista 3D")
        self.close_viewer_button.set_sensitive(False)
        self.close_viewer_button.connect("clicked", lambda _b: self._close_viewer())
        row.pack_start(self.close_viewer_button, False, False, 0)

        page.pack_start(view_frame, False, True, 0)

        live_frame, live_box = self._section("Ajustes de captura en vivo")
        row = self._row(live_box)
        row.pack_start(Gtk.Label(label="Distancia maxima (m):"), False, False, 0)
        self.live_max_distance_entry = Gtk.Entry()
        self.live_max_distance_entry.set_text("0")
        self.live_max_distance_entry.set_width_chars(8)
        row.pack_start(self.live_max_distance_entry, False, False, 0)
        row.pack_start(Gtk.Label(label="Cono (grados):"), False, False, 0)
        self.live_cone_angle_entry = Gtk.Entry()
        self.live_cone_angle_entry.set_text("")
        self.live_cone_angle_entry.set_width_chars(8)
        row.pack_start(self.live_cone_angle_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "Filtran los puntos leidos del sensor en cada frame en vivo: descartan lo que "
                "esta mas lejos de la distancia maxima, o fuera del cono (campo de vision) "
                "indicado, centrado en el eje frontal."
            ),
            False,
            False,
            0,
        )

        row = self._row(live_box)
        row.pack_start(Gtk.Label(label="Eje frontal:"), False, False, 0)
        self.live_forward_axis_combo = Gtk.ComboBoxText()
        for axis in ("x", "y", "z"):
            self.live_forward_axis_combo.append_text(axis)
        self.live_forward_axis_combo.set_active(2)  # "z"
        row.pack_start(self.live_forward_axis_combo, False, False, 0)
        self.live_invert_y_check = Gtk.CheckButton(label="Invertir Y")
        row.pack_start(self.live_invert_y_check, False, False, 0)
        self.live_invert_z_check = Gtk.CheckButton(label="Invertir Z")
        row.pack_start(self.live_invert_z_check, False, False, 0)

        self.visualization_status_label = Gtk.Label(xalign=0)
        self.visualization_status_label.set_line_wrap(True)
        self.visualization_status_label.set_text("Los cambios se aplican al abrir la vista 3D.")
        live_box.pack_start(self.visualization_status_label, False, False, 0)
        for entry in (
            self.band_low_entry,
            self.band_high_entry,
            self.target_thickness_entry,
            self.max_distance_entry,
            self.live_max_distance_entry,
            self.live_cone_angle_entry,
        ):
            entry.connect("changed", lambda _entry: self._apply_viewer_settings())
        self.live_forward_axis_combo.connect("changed", lambda _combo: self._apply_viewer_settings())
        self.live_invert_y_check.connect("toggled", lambda _check: self._apply_viewer_settings())
        self.live_invert_z_check.connect("toggled", lambda _check: self._apply_viewer_settings())
        self.live_max_distance_entry.connect(
            "changed", lambda _entry: self._sync_live_filters_to_capture()
        )
        self.live_cone_angle_entry.connect(
            "changed", lambda _entry: self._sync_live_filters_to_capture()
        )
        self.live_forward_axis_combo.connect(
            "changed", lambda _combo: self._sync_live_filters_to_capture()
        )
        self._sync_live_filters_to_capture()

        page.pack_start(live_frame, False, True, 0)
    # -- Pagina: Comparacion (prueba / experimental) ----------------------------

    def _build_embedded_test_page(self) -> Gtk.Widget:
        page = self._new_page()

        note_frame, note_box = self._section("Seccion experimental")
        note = Gtk.Label(
            label=(
                "Prueba: visor 3D embebido directamente en la ventana (a diferencia de la "
                "vista 3D disponible en Resultados, que abre una ventana aparte). Muestra ambas "
                "nubes superpuestas, con un resaltado sutil donde difieren. Todavia no esta "
                "decidido si esto queda en la version final."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        note_box.pack_start(note, False, False, 0)

        row = self._row(note_box)
        load_button = Gtk.Button(label="Cargar comparacion aqui")
        load_button.set_tooltip_text("Usa los archivos elegidos en Medicion rapida.")
        load_button.connect("clicked", lambda _b: self._load_embedded_comparison())
        row.pack_start(load_button, False, False, 0)
        self.embedded_status_label = Gtk.Label(label="(nada cargado todavia)")
        row.pack_start(self.embedded_status_label, False, False, 0)

        page.pack_start(note_frame, False, True, 0)

        viewer_frame, viewer_box = self._section("Vista (arrastrar = orbitar, rueda = zoom)")
        self.embedded_viewer = EmbeddedComparisonViewer(width=900, height=520)
        viewer_box.pack_start(self.embedded_viewer, True, True, 0)
        page.pack_start(viewer_frame, True, True, 0)

        return page

    def _load_embedded_comparison(self) -> None:
        base_file = Path(self.base_path)
        updated_file = Path(self.updated_path)
        if not base_file.is_file() or not updated_file.is_file():
            self._show_error(
                "Error", "Elige ambos archivos en Medicion rapida antes de cargar la vista."
            )
            return

        self.embedded_status_label.set_text("Cargando...")

        def worker():
            try:
                base_cloud = load_point_cloud(base_file)
                updated_cloud = load_point_cloud(updated_file)
                distances = compute_c2c_distance(updated_cloud, base_cloud)
                overlay_cloud = build_subtle_overlay_cloud(updated_cloud, distances)
            except Exception as exc:
                self._ui(self._on_embedded_load_failed, str(exc))
                return
            self._ui(self._on_embedded_load_done, base_cloud, overlay_cloud)

        threading.Thread(target=worker, daemon=True).start()

    def _on_embedded_load_done(self, base_cloud, overlay_cloud) -> None:
        self.embedded_viewer.load_clouds(base_cloud, overlay_cloud)
        self.embedded_status_label.set_text("Cargado ✓")

    def _on_embedded_load_failed(self, message: str) -> None:
        self.embedded_status_label.set_text("Error al cargar")
        self._show_error("Error", message)

    # -- Pagina: Raycasting (prueba) --------------------------------------------

    def _build_raycast_test_page(self) -> Gtk.Widget:
        page = self._new_page()

        note_frame, note_box = self._section("Seccion experimental")
        note = Gtk.Label(
            label=(
                "Prueba: en vez de Cloud-to-Cloud (vecino mas cercano), reconstruye una "
                "malla de cada nube y mide el espesor por raycasting en vez de por "
                "distancia euclidiana al vecino mas cercano. Los archivos se comparten "
                "con Medicion rapida (elegirlos aqui o alla es lo mismo). "
                "Todavia no esta decidido si esto reemplaza al metodo Cloud-to-Cloud "
                "actual."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        note_box.pack_start(note, False, False, 0)
        page.pack_start(note_frame, False, True, 0)

        method_frame, method_box = self._section("Metodo")
        row = self._row(method_box)
        row.pack_start(Gtk.Label(label="Malla:"), False, False, 0)
        self.raycast_mesh_poisson_rb = Gtk.RadioButton.new_with_label_from_widget(None, "Poisson")
        row.pack_start(self.raycast_mesh_poisson_rb, False, False, 0)
        self.raycast_mesh_delaunay_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.raycast_mesh_poisson_rb, "Delaunay"
        )
        row.pack_start(self.raycast_mesh_delaunay_rb, False, False, 0)
        mesh_note = Gtk.Label(
            label=(
                "Poisson reconstruye una superficie cerrada, pensado para una nube "
                "completa. Delaunay triangula la proyeccion sobre el plano que mejor "
                "ajusta la nube — pensado para un recorte casi-plano (pestaña "
                "'Preparar nubes'), no genera artefactos en los bordes abiertos."
            ),
            xalign=0,
        )
        mesh_note.set_line_wrap(True)
        method_box.pack_start(mesh_note, False, False, 0)

        row = self._row(method_box)
        row.pack_start(Gtk.Label(label="Rayos:"), False, False, 0)
        self.raycast_ray_origin_rb = Gtk.RadioButton.new_with_label_from_widget(
            None, "Desde el origen del sensor"
        )
        row.pack_start(self.raycast_ray_origin_rb, False, False, 0)
        self.raycast_ray_normal_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.raycast_ray_origin_rb, "A lo largo de la normal del plano"
        )
        row.pack_start(self.raycast_ray_normal_rb, False, False, 0)
        ray_note = Gtk.Label(
            label=(
                "'Desde el origen del sensor' asume que el sensor no se movio entre "
                "capturas (o que volvio a la misma posicion via 'Alineacion IMU') y usa "
                "el sidecar '_pose.npz' de cada .ply (o el origen manual de abajo si no "
                "existe). 'A lo largo de la normal del plano' no necesita la pose del "
                "sensor, pero requiere que las dos nubes ya esten alineadas (pestaña "
                "'Alineacion' o 'Alineacion IMU') — mide el espesor perpendicular a la "
                "pared original en vez de a lo largo de un rayo desde el sensor."
            ),
            xalign=0,
        )
        ray_note.set_line_wrap(True)
        method_box.pack_start(ray_note, False, False, 0)
        self.raycast_ray_origin_rb.connect("toggled", lambda _b: self._on_raycast_ray_method_toggled())
        page.pack_start(method_frame, False, True, 0)

        files_frame, files_box = self._section("Archivos")
        self.raycast_base_path_label = self._file_row(
            files_box,
            self.base_path,
            lambda p: self._set_base_path(p),
            info_text="Tunel original (antes del shotcrete). Comparte archivo con "
            "Medicion rapida — elegirlo aqui tambien lo actualiza alla.",
        )
        self.raycast_updated_path_label = self._file_row(
            files_box,
            self.updated_path,
            lambda p: self._set_updated_path(p),
            info_text="Tunel con shotcrete (despues). Comparte archivo con "
            "Medicion rapida — elegirlo aqui tambien lo actualiza alla.",
        )
        page.pack_start(files_frame, False, True, 0)

        origin_frame, origin_box = self._section("Origen del sensor (si el .ply no tiene sidecar de pose)")
        origin_note = Gtk.Label(
            label=(
                "Se usa el sidecar '_pose.npz' guardado junto al .ply si existe. Si no "
                "existe (nube cargada a mano, sin sensor conectado), se usa el origen "
                "manual de aca abajo — por defecto (0,0,0), es decir la nube ya esta en "
                "el sistema de coordenadas del sensor."
            ),
            xalign=0,
        )
        origin_note.set_line_wrap(True)
        origin_box.pack_start(origin_note, False, False, 0)

        row = self._row(origin_box)
        row.pack_start(Gtk.Label(label="Origen base (x y z, m):"), False, False, 0)
        self.raycast_base_origin_entry = Gtk.Entry()
        self.raycast_base_origin_entry.set_text("0 0 0")
        self.raycast_base_origin_entry.set_width_chars(18)
        row.pack_start(self.raycast_base_origin_entry, False, False, 0)

        row = self._row(origin_box)
        row.pack_start(Gtk.Label(label="Origen actualizada (x y z, m):"), False, False, 0)
        self.raycast_updated_origin_entry = Gtk.Entry()
        self.raycast_updated_origin_entry.set_text("0 0 0")
        self.raycast_updated_origin_entry.set_width_chars(18)
        row.pack_start(self.raycast_updated_origin_entry, False, False, 0)
        page.pack_start(origin_frame, False, True, 0)
        self.raycast_origin_frame = origin_frame

        action_frame, action_box = self._section("Calcular")
        row = self._row(action_box)
        self.raycast_run_button = Gtk.Button(label="Calcular espesor (raycasting)")
        self.raycast_run_button.connect("clicked", lambda _b: self._run_raycast_clicked())
        row.pack_start(self.raycast_run_button, False, False, 0)
        self.raycast_status_label = Gtk.Label(label="(nada calculado todavia)")
        row.pack_start(self.raycast_status_label, False, False, 0)
        page.pack_start(action_frame, False, True, 0)

        self._on_raycast_ray_method_toggled()
        return self._scrolled(page)

    def _on_raycast_ray_method_toggled(self) -> None:
        self.raycast_origin_frame.set_sensitive(self.raycast_ray_origin_rb.get_active())

    def _raycast_log(self, message: str) -> None:
        self._log(f"[Malla] {message}")

    def _run_raycast_clicked(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            self._show_warning("En progreso", "Ya hay un analisis en ejecucion.")
            return

        base_file = Path(self.base_path)
        updated_file = Path(self.updated_path)
        if not base_file.is_file() or not updated_file.is_file():
            self._show_error(
                "Error", "Elige ambos archivos (arriba, o en Medicion rapida) antes de calcular."
            )
            return

        ray_method = "origin" if self.raycast_ray_origin_rb.get_active() else "normal"
        mesh_method = "poisson" if self.raycast_mesh_poisson_rb.get_active() else "delaunay"

        if ray_method == "normal" and not self.alignment_applied:
            if not self._confirm_missing_alignment():
                self._refresh_quick_status()
                return

        try:
            max_distance_text = self.quick_ray_max_distance_entry.get_text().strip()
            max_distance = (
                self._parse_positive_entry(self.quick_ray_max_distance_entry, "El limite de espesor")
                if max_distance_text
                else None
            )
            poisson_depth = (
                self._read_numeric_entry(self.quick_poisson_depth, "Profundidad Poisson", 5, 12, integer=True)
                if mesh_method == "poisson"
                else 9
            )
            density_trim = (
                self._read_numeric_entry(self.quick_density_trim, "Recorte de densidad Poisson", 0, 0.20)
                if mesh_method == "poisson"
                else 0.02
            )
            delaunay_trim = (
                self._read_numeric_entry(self.quick_delaunay_trim, "Limite Delaunay", 1, 10)
                if mesh_method == "delaunay"
                else 4.0
            )
            mesh_voxel_size = self._read_numeric_entry(
                self.quick_mesh_voxel_entry, "Resolucion de malla", 0, 0.2
            )
            smooth_iterations = (
                self._read_numeric_entry(self.quick_smooth_iterations, "Iteraciones Taubin", 1, 30, integer=True)
                if self.quick_smooth_check.get_active()
                else 0
            )
        except ValueError as exc:
            self._show_error("Parametros invalidos", str(exc))
            return

        base_origin = updated_origin = None
        if ray_method == "origin":
            try:
                base_origin = self._parse_xyz(self.raycast_base_origin_entry.get_text(), "Origen base")
                updated_origin = self._parse_xyz(
                    self.raycast_updated_origin_entry.get_text(), "Origen actualizada"
                )
            except ValueError as exc:
                self._show_error("Parametros invalidos", str(exc))
                return
            # Si el .ply tiene sidecar de pose (captura real con sensor), usarlo
            # tiene prioridad sobre el origen manual del formulario; el manual es
            # solo un respaldo para probar el flujo sin sensor conectado.
            base_origin = None if load_reference_pose(base_file) is not None else base_origin
            updated_origin = None if load_reference_pose(updated_file) is not None else updated_origin

        params = RaycastPipelineParams(
            base_path=base_file,
            updated_path=updated_file,
            output_dir=Path(self.output_dir),
            max_distance=max_distance,
            base_origin=base_origin,
            updated_origin=updated_origin,
            mesh_method=mesh_method,
            mesh_voxel_size=float(mesh_voxel_size),
            ray_method=ray_method,
            density_trim_quantile=float(density_trim),
            poisson_depth=int(poisson_depth),
            mesh_smoothing_iterations=int(smooth_iterations),
            delaunay_max_edge_trim_factor=float(delaunay_trim),
            color_spec=self._selected_preview_heatmap_spec(),
        )

        self._invalidate_results()
        generation = self._result_generation
        self.log_buffer.set_text("")
        self.raycast_run_button.set_sensitive(False)
        self.quick_calculate_button.set_sensitive(False)
        self.result_recalculate_button.set_sensitive(False)
        self.raycast_status_label.set_text("Calculando...")
        self._start_calculation_progress("Preparando reconstruccion de malla")
        self.status_label.set_text("Calculando espesor con malla...")
        self.stack.set_visible_child_name("resultados")

        self.worker_thread = threading.Thread(
            target=self._run_raycast_worker, args=(params, generation), daemon=True
        )
        self.worker_thread.start()

    def _run_raycast_worker(self, params: RaycastPipelineParams, generation: int) -> None:
        try:
            payload = {
                "base_path": str(params.base_path),
                "updated_path": str(params.updated_path),
                "output_dir": str(params.output_dir),
                "max_distance": params.max_distance,
                "base_origin": params.base_origin,
                "updated_origin": params.updated_origin,
                "mesh_method": params.mesh_method,
                "mesh_voxel_size": params.mesh_voxel_size,
                "ray_method": params.ray_method,
                "density_trim_quantile": params.density_trim_quantile,
                "poisson_depth": params.poisson_depth,
                "mesh_smoothing_iterations": params.mesh_smoothing_iterations,
                "delaunay_max_edge_trim_factor": params.delaunay_max_edge_trim_factor,
                "color_spec": params.color_spec,
            }
            self._raycast_log("Iniciando calculo aislado; la interfaz seguira disponible...")
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(PROJECT_ROOT / "scripts" / "raycast_worker.py"),
                    json.dumps(payload),
                ],
                cwd=str(PROJECT_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            output_lines = []
            assert process.stdout is not None
            for line in process.stdout:
                message = line.rstrip("\r\n")
                if message:
                    output_lines.append(message)
                    self._raycast_log(message)
            return_code = process.wait()
            if return_code != 0:
                details = "\n".join(output_lines[-20:]) or f"El proceso termino con codigo {return_code}."
                raise RuntimeError(details)

            self._raycast_log("Cargando el resultado en la interfaz...")
            import open3d as o3d

            stats_data = json.loads(
                (params.output_dir / "raycast_stats.json").read_text(encoding="utf-8")
            )
            stats = DistanceStats(**stats_data)
            csv_path = params.output_dir / "thickness_raycast_per_point.csv"
            rows = np.loadtxt(csv_path, delimiter=",", skiprows=1, ndmin=2)
            base_mesh = o3d.io.read_triangle_mesh(str(params.output_dir / "raycast_base_mesh.ply"))
            updated_mesh = o3d.io.read_triangle_mesh(str(params.output_dir / "raycast_updated_mesh.ply"))
            heatmap_path = params.output_dir / "thickness_raycast_heatmap.ply"
            result = RaycastPipelineResult(
                stats=stats,
                distances=rows[:, 3].copy(),
                hit_points=rows[:, :3].copy(),
                unmatched_points=np.load(params.output_dir / "raycast_unmatched_points.npy"),
                base_mesh=base_mesh,
                updated_mesh=updated_mesh,
                heatmap_cloud=load_point_cloud(heatmap_path),
                csv_path=csv_path,
                histogram_path=params.output_dir / "thickness_raycast_histogram.png",
                heatmap_path=heatmap_path,
            )
            self._raycast_log("\nListo.")
            self._ui(self._on_raycast_success, result, generation)
        except Exception as exc:
            self._raycast_log(f"\nERROR: {exc}")
            self._ui(self._on_raycast_failure, str(exc), generation)
        finally:
            self._ui(self.raycast_run_button.set_sensitive, True)
            self._ui(self.quick_calculate_button.set_sensitive, True)
            self._ui(self._refresh_recalculate_button)

    def _on_raycast_success(self, result, generation: int) -> None:
        if generation != self._result_generation:
            return
        self._finish_calculation_progress(True)
        self.raycast_result = result
        stats = self.raycast_result.stats
        self._active_result_kind = "raycast"
        self._set_stat(self.stat_points, f"{stats.n_points:,}")
        self._set_stat(self.stat_mean, f"{stats.mean * 100:.2f} cm")
        self._set_stat(self.stat_median, f"{stats.median * 100:.2f} cm", COLOR_MEDIAN)
        self._set_stat(self.stat_std, f"{stats.std * 100:.2f} cm")
        self.cards_row.set_no_show_all(False)
        self.cards_row.show_all()
        self.results_empty_label.hide()
        self.status_label.set_markup(
            f'<span foreground="{COLOR_OK}">●</span>  Analisis con malla completo — '
            f"espesor promedio {stats.mean * 100:.2f} cm"
        )
        self.generate_report_button.set_sensitive(True)
        self.open_result_viewer_button.set_sensitive(True)
        self.open_result_viewer_button.set_label("Ver mapa de espesor 3D")
        self.quick_status_label.set_text(
            f"Medicion con malla lista: promedio {stats.mean * 100:.2f} cm, "
            f"mediana {stats.median * 100:.2f} cm, desviación estándar {stats.std * 100:.2f} cm."
        )
        self.raycast_status_label.set_text(f"Listo — espesor promedio {stats.mean * 100:.2f} cm")
        self._show_quick_mesh_result_actions()
        self._generate_alerts()
        self._set_calculation_method("raycast")
        self.results_ray_method_combo.set_active_id(
            "origin" if self.raycast_ray_origin_rb.get_active() else "normal"
        )
        self.stack.set_visible_child_name("resultados")
        self._view_raycast_result()

    def _on_raycast_failure(self, message: str, generation: int) -> None:
        if generation != self._result_generation:
            return
        self._hide_quick_mesh_result_actions()
        self._finish_calculation_progress(False)
        self.raycast_status_label.set_text("Error al calcular")
        self.quick_status_label.set_text(f"No se pudo calcular la malla: {message}")
        self.status_label.set_markup(f'<span foreground="{COLOR_ERROR}">●</span>  El analisis con malla fallo')
        self.stack.set_visible_child_name("resultados")
        self._show_error("Error durante el analisis por raycasting", message)

    def _view_raycast_result(self) -> None:
        if not self.raycast_result:
            return
        self._show_result_in_viewer()

    # -- Helpers de layout -----------------------------------------------------

    def _new_page(self) -> Gtk.Box:
        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        page.set_margin_start(12)
        page.set_margin_end(12)
        page.set_margin_top(10)
        page.set_margin_bottom(10)
        return page

    @staticmethod
    def _numeric_entry(value: str, width: int, integer: bool = False) -> Gtk.Entry:
        entry = Gtk.Entry()
        entry.set_text(value)
        entry.set_width_chars(width)
        entry.set_alignment(0.5)
        entry.set_input_purpose(Gtk.InputPurpose.DIGITS if integer else Gtk.InputPurpose.NUMBER)
        return entry

    @staticmethod
    def _read_numeric_entry(
        entry: Gtk.Entry, label: str, minimum: float, maximum: float, integer: bool = False
    ) -> float | int:
        raw = entry.get_text().strip().replace(",", ".")
        try:
            value = float(raw)
        except ValueError as exc:
            raise ValueError(f"{label}: escribe un numero valido.") from exc
        if not np.isfinite(value) or value < minimum or value > maximum:
            raise ValueError(f"{label}: el valor debe estar entre {minimum:g} y {maximum:g}.")
        if integer:
            if not value.is_integer():
                raise ValueError(f"{label}: usa un numero entero.")
            return int(value)
        return value

    def _scrolled(self, widget: Gtk.Widget) -> Gtk.Widget:
        scroller = Gtk.ScrolledWindow()
        scroller.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroller.add(widget)
        return scroller

    def _section(self, title: str) -> tuple[Gtk.Frame, Gtk.Box]:
        frame = Gtk.Frame()
        label = Gtk.Label()
        label.set_markup(f"<b>{title}</b>")
        label.set_margin_start(8)
        label.set_margin_end(8)
        label.set_margin_top(4)
        label.set_margin_bottom(4)
        frame.set_label_widget(label)
        frame.set_shadow_type(Gtk.ShadowType.ETCHED_IN)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_start(12)
        box.set_margin_end(12)
        box.set_margin_top(10)
        box.set_margin_bottom(12)
        frame.add(box)
        return frame, box

    @staticmethod
    def _capture_panel(title: str, subtitle: str) -> tuple[Gtk.Box, Gtk.Box]:
        panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        panel.get_style_context().add_class("capture-panel")
        heading = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        title_label = Gtk.Label(label=title, xalign=0)
        title_label.get_style_context().add_class("capture-panel-title")
        subtitle_label = Gtk.Label(label=subtitle, xalign=0)
        subtitle_label.get_style_context().add_class("capture-panel-subtitle")
        subtitle_label.set_line_wrap(True)
        heading.pack_start(title_label, False, False, 0)
        heading.pack_start(subtitle_label, False, False, 0)
        panel.pack_start(heading, False, False, 0)
        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        panel.pack_start(content, False, False, 0)
        return panel, content

    def _row(self, parent_box: Gtk.Box) -> Gtk.Box:
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        parent_box.pack_start(row, False, False, 0)
        return row

    def _stat_card(self, title: str) -> tuple[Gtk.Widget, Gtk.Label]:
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        box.set_halign(Gtk.Align.CENTER)
        box.get_style_context().add_class("kpi-card")
        title_label = Gtk.Label(label=title.upper())
        title_label.get_style_context().add_class("dim-label")
        box.pack_start(title_label, False, False, 0)
        value_label = Gtk.Label()
        value_label.get_style_context().add_class("kpi-value")
        value_label.set_markup(f"<span size='xx-large' weight='bold' foreground='{COLOR_ACCENT}'>—</span>")
        box.pack_start(value_label, False, False, 0)
        return box, value_label

    def _set_stat(self, label: Gtk.Label, text: str, color: str = COLOR_ACCENT) -> None:
        label.set_markup(
            f"<span size='xx-large' weight='bold' foreground='{color}'>"
            f"{GLib.markup_escape_text(text)}</span>"
        )

    def _info_button(self, text: str) -> Gtk.MenuButton:
        button = Gtk.MenuButton()
        icon = Gtk.Image.new_from_icon_name("dialog-information-symbolic", Gtk.IconSize.BUTTON)
        button.set_image(icon)
        button.set_relief(Gtk.ReliefStyle.NONE)
        button.set_tooltip_text("Mas informacion")
        popover = Gtk.Popover()
        label = Gtk.Label(label=text)
        label.set_line_wrap(True)
        label.set_max_width_chars(46)
        label.set_margin_start(10)
        label.set_margin_end(10)
        label.set_margin_top(8)
        label.set_margin_bottom(8)
        popover.add(label)
        label.show()
        button.set_popover(popover)
        return button

    def _set_path_label(self, label: Gtk.Label, path: str) -> None:
        name = os.path.basename(path) if path else ""
        if name:
            escaped_name = GLib.markup_escape_text(name)
            label.set_markup(f'<span foreground="{COLOR_FILE_NAME}">{escaped_name}</span>')
        else:
            label.set_text("Sin seleccionar")
        label.set_tooltip_text(path or "")

    def _file_row(self, parent_box: Gtk.Box, initial_path: str, on_selected, info_text: str | None = None) -> Gtk.Label:
        row = self._row(parent_box)
        label = Gtk.Label(xalign=0)
        label.set_hexpand(True)
        self._set_path_label(label, initial_path)
        row.pack_start(label, True, True, 0)

        def on_click(_button):
            path = self._run_open_file_dialog()
            if path:
                self._set_path_label(label, path)
                on_selected(path)

        button = Gtk.Button(label="Elegir archivo...")
        button.connect("clicked", on_click)
        row.pack_start(button, False, False, 0)
        if info_text:
            row.pack_start(self._info_button(info_text), False, False, 0)
        return label

    def _run_open_file_dialog(self) -> str | None:
        dialog = Gtk.FileChooserNative.new(
            "Seleccionar archivo .ply",
            self.window,
            Gtk.FileChooserAction.OPEN,
            "_Abrir",
            "_Cancelar",
        )
        filter_ply = Gtk.FileFilter()
        filter_ply.set_name("Archivos PLY")
        filter_ply.add_pattern("*.ply")
        dialog.add_filter(filter_ply)
        return self._run_native_file_chooser(dialog)

    def _browse_output_dir(self) -> None:
        dialog = Gtk.FileChooserNative.new(
            "Seleccionar carpeta de salida",
            self.window,
            Gtk.FileChooserAction.SELECT_FOLDER,
            "_Seleccionar",
            "_Cancelar",
        )
        path = self._run_native_file_chooser(dialog)
        if path:
            self.output_dir = path
            self._set_path_label(self.output_dir_label, self.output_dir)

    def _save_file_dialog(self, title: str, initial_dir: Path, initial_name: str) -> str | None:
        dialog = Gtk.FileChooserNative.new(
            title,
            self.window,
            Gtk.FileChooserAction.SAVE,
            "_Guardar",
            "_Cancelar",
        )
        dialog.set_do_overwrite_confirmation(True)
        dialog.set_current_folder(str(initial_dir))
        dialog.set_current_name(initial_name)
        return self._run_native_file_chooser(dialog)

    def _run_native_file_chooser(self, dialog) -> str | None:
        """Muestra el selector nativo y recuerda la ultima carpeta entre sesiones."""
        action = dialog.get_action()
        if action != Gtk.FileChooserAction.SAVE and self._last_file_dialog_dir is not None:
            if self._last_file_dialog_dir.is_dir():
                dialog.set_current_folder(str(self._last_file_dialog_dir))

        path = None
        if dialog.run() == Gtk.ResponseType.ACCEPT:
            path = dialog.get_filename()
            if path:
                selected = Path(path)
                folder = selected if action == Gtk.FileChooserAction.SELECT_FOLDER else selected.parent
                if folder.is_dir():
                    self._last_file_dialog_dir = folder
                    try:
                        self._last_file_dialog_dir_file.parent.mkdir(parents=True, exist_ok=True)
                        self._last_file_dialog_dir_file.write_text(str(folder), encoding="utf-8")
                    except OSError:
                        pass
        dialog.hide()
        return path

    def _show_error(self, title: str, message: str) -> None:
        dialog = Gtk.MessageDialog(
            transient_for=self.window, flags=0, message_type=Gtk.MessageType.ERROR, buttons=Gtk.ButtonsType.OK, text=title
        )
        dialog.format_secondary_text(message)
        dialog.run()
        dialog.destroy()

    def _show_warning(self, title: str, message: str) -> None:
        dialog = Gtk.MessageDialog(
            transient_for=self.window,
            flags=0,
            message_type=Gtk.MessageType.WARNING,
            buttons=Gtk.ButtonsType.OK,
            text=title,
        )
        dialog.format_secondary_text(message)
        dialog.run()
        dialog.destroy()

    def _show_info(self, title: str, message: str) -> None:
        dialog = Gtk.MessageDialog(
            transient_for=self.window, flags=0, message_type=Gtk.MessageType.INFO, buttons=Gtk.ButtonsType.OK, text=title
        )
        dialog.format_secondary_text(message)
        dialog.run()
        dialog.destroy()

    def _ui(self, func, *args, **kwargs) -> None:
        """Encola 'func' para que corra en el hilo principal de GTK (equivalente a root.after(0, ...))."""

        def wrapper():
            func(*args, **kwargs)
            return False

        GLib.idle_add(wrapper)

    def _log(self, message: str) -> None:
        self.log_queue.put(message)
        progress = self._progress_for_log(message)
        if progress is not None:
            fraction, label = progress
            self._ui(self._advance_calculation_progress, fraction, label)

    @staticmethod
    def _progress_for_log(message: str) -> tuple[float, str] | None:
        text = message.lower()
        if "listo." in text:
            return 1.0, "Calculo terminado"
        if "heatmap guardado" in text:
            return 0.97, "Preparando mapa de espesor"
        if "histograma guardado" in text:
            return 0.92, "Generando histograma"
        if "csv guardado" in text:
            return 0.87, "Guardando distancias"
        if "tirando rayos" in text:
            return 0.74, "Midiendo espesor sobre las mallas"
        if "malla actualizada:" in text:
            return 0.70, "Malla DESPUES lista"
        if "reconstruyendo malla" in text and "actualizada" in text:
            return 0.48, "Reconstruyendo malla DESPUES"
        if "malla base:" in text:
            return 0.43, "Malla ANTES lista"
        if "reconstruyendo malla" in text:
            return 0.14, "Reconstruyendo malla ANTES"
        if "reduciendo densidad de malla" in text or "ajustando resolucion" in text:
            return 0.08, "Ajustando resolucion de las nubes"
        if "resultados:" in text:
            return 0.82, "Calculando estadisticas"
        if "calculando distancia cloud-to-cloud" in text:
            return 0.66, "Calculando distancias entre superficies"
        if "alineando nubes" in text:
            return 0.44, "Alineando las capturas"
        if "nubes listas para medir" in text:
            return 0.42, "Nubes preparadas"
        if "nube antes lista" in text:
            return 0.30, "Nube ANTES preparada"
        if "preparando nube despues" in text:
            return 0.32, "Preparando nube DESPUES"
        if "preparando nube antes" in text:
            return 0.22, "Preparando nube ANTES"
        if "preparando nubes" in text:
            return 0.20, "Filtrando y preparando nubes"
        if "cargando nubes" in text:
            return 0.06, "Cargando archivos"
        return None

    def _start_calculation_progress(self, label: str) -> None:
        self._progress_started_at = time.monotonic()
        self._progress_stage = label
        self.calculation_label.set_text(label)
        self.calculation_indicator.set_no_show_all(False)
        self.calculation_indicator.show_all()
        self.calculation_spinner.start()
        if self._progress_timer_source is None:
            self._progress_timer_source = GLib.timeout_add_seconds(1, self._update_calculation_indicator)
        self._update_calculation_indicator()

    def _update_calculation_indicator(self) -> bool:
        if not self.calculation_indicator.get_visible():
            self._progress_timer_source = None
            return GLib.SOURCE_REMOVE
        elapsed = max(0, int(time.monotonic() - self._progress_started_at))
        minutes, seconds = divmod(elapsed, 60)
        self.calculation_label.set_text(
            f"Cargando · {self._progress_stage} · {minutes:02d}:{seconds:02d}"
        )
        return GLib.SOURCE_CONTINUE

    def _advance_calculation_progress(self, fraction: float, label: str) -> None:
        self._progress_stage = label
        self._update_calculation_indicator()

    def _finish_calculation_progress(self, succeeded: bool) -> None:
        if self._progress_timer_source is not None:
            GLib.source_remove(self._progress_timer_source)
            self._progress_timer_source = None
        self.calculation_spinner.stop()
        self.calculation_indicator.set_no_show_all(True)
        self.calculation_indicator.hide()

    def _refresh_recalculate_button(self) -> None:
        if not hasattr(self, "result_recalculate_button"):
            return
        ready = bool(
            self.base_path
            and self.updated_path
            and Path(self.base_path).is_file()
            and Path(self.updated_path).is_file()
        )
        self.result_recalculate_button.set_sensitive(ready)

    def _poll_log_queue(self) -> bool:
        try:
            while True:
                message = self.log_queue.get_nowait()
                end_iter = self.log_buffer.get_end_iter()
                self.log_buffer.insert(end_iter, message + "\n")
                if self.log_view.get_mapped():
                    self.log_view.scroll_to_iter(
                        self.log_buffer.get_end_iter(), 0.0, False, 0, 0
                    )
        except queue.Empty:
            pass
        return True

    def _parse_xyz(self, text: str, field_name: str) -> tuple[float, float, float]:
        parts = text.replace(",", " ").split()
        if len(parts) != 3:
            raise ValueError(f"{field_name} debe tener 3 numeros (x y z), separados por espacio o coma.")
        return tuple(float(p) for p in parts)

    # ------------------------------------------------------------ Sensor

    def _sync_sensor_address(self, source_entry) -> None:
        if not hasattr(self, "sensor_address_entry") or not hasattr(self, "quick_sensor_address_entry"):
            return
        target = self.quick_sensor_address_entry if source_entry is self.sensor_address_entry else self.sensor_address_entry
        if target.get_text() != source_entry.get_text():
            target.set_text(source_entry.get_text())

    def _set_sensor_ui_state(self, connected: bool, connecting: bool = False) -> None:
        button_text = "Conectando..." if connecting else ("Desconectar" if connected else "Conectar sensor")
        status = "Conectando..." if connecting else ("Conectado" if connected else "Desconectado")
        color = COLOR_WARN if connecting else (COLOR_OK if connected else COLOR_ERROR)
        for button in (self.connect_button, self.quick_connect_button):
            button.set_label(button_text)
            button.set_sensitive(not connecting)
        for label in (self.sensor_status_label, self.quick_sensor_status_label):
            label.set_markup(f'<span foreground="{color}">●</span>  {status}')
        if hasattr(self, "quick_capture_base_button"):
            self.quick_capture_base_button.set_sensitive(connected and not connecting)
        if hasattr(self, "quick_capture_updated_button"):
            self.quick_capture_updated_button.set_sensitive(connected and not connecting)

    def _toggle_sensor_connection(self, automatic: bool = False) -> None:
        if self.sensor_connection is not None:
            self._disconnect_sensor()
            return

        address = self.sensor_address_entry.get_text().strip()
        self._set_sensor_ui_state(False, connecting=True)

        def worker():
            try:
                connection = aurora_sensor.connect(address)
                self._ui(self._on_sensor_connected, connection)
            except Exception as exc:
                self._ui(self._on_sensor_connect_failed, exc, automatic)

        threading.Thread(target=worker, daemon=True).start()

    def _on_sensor_connected(self, connection) -> None:
        self.sensor_connection = connection
        self._set_sensor_ui_state(True)
        self.capture_base_button.set_sensitive(True)
        self.capture_updated_button.set_sensitive(True)
        self.start_live_capture_button.set_sensitive(True)
        self.capture_live_baseline_button.set_sensitive(True)
        self.clear_live_baseline_button.set_sensitive(True)
        self.save_live_clouds_button.set_sensitive(True)
        if self.viewer is not None and self.source_live_rb.get_active():
            self.viewer.set_live_sensor(connection)
            self._apply_viewer_settings()
        self._log(f"Conectado al sensor Aurora en {self.sensor_address_entry.get_text()}.")

    def _on_sensor_connect_failed(self, exc: Exception, automatic: bool = False) -> None:
        self._set_sensor_ui_state(False)
        if automatic:
            self.quick_capture_status_label.set_text(
                f"Conexión automática falló: {exc}. Puedes reintentar desde aquí."
            )
            self._log(f"Fallo de conexión automática al Aurora: {exc}")
        else:
            self._show_error("Error de conexion", str(exc))

    def _disconnect_sensor(self) -> None:
        if self.viewer is not None:
            self.viewer.set_live_sensor(None)
        try:
            aurora_sensor.disconnect(self.sensor_connection)
        except Exception as exc:
            self._log(f"Aviso al desconectar: {exc}")
        self.sensor_connection = None
        self._set_sensor_ui_state(False)
        self.capture_base_button.set_sensitive(False)
        self.capture_updated_button.set_sensitive(False)
        self.stop_capture_button.set_sensitive(False)
        self.start_live_capture_button.set_sensitive(False)
        self.capture_live_baseline_button.set_sensitive(False)
        self.clear_live_baseline_button.set_sensitive(False)
        self.save_live_clouds_button.set_sensitive(False)

    def _capture_clicked(self, target: str) -> None:
        if self.sensor_connection is None:
            self._show_warning("Sensor no conectado", "Conecta el sensor antes de capturar.")
            return

        try:
            persistence_ratio = float(self.capture_persistence_entry.get_text() or 0.0)
        except ValueError:
            self._show_error("Parametros invalidos", "La persistencia debe ser un numero.")
            return
        if not np.isfinite(persistence_ratio) or not 0 <= persistence_ratio <= 1:
            self._show_error("Parametros invalidos", "La persistencia debe estar entre 0 y 1.")
            return

        capture_duration_s = float(self.quick_capture_duration_scale.get_value())
        aligned_multiview = self.quick_aligned_capture_check.get_active()

        cone_angle_deg = None
        max_distance_m = None
        try:
            if self.capture_limit_fov_check.get_active():
                cone_text = self.capture_cone_angle_entry.get_text().strip()
                if cone_text:
                    cone_angle_deg = float(cone_text)
                    if not np.isfinite(cone_angle_deg) or cone_angle_deg <= 0 or cone_angle_deg > 180:
                        raise ValueError("El cono debe estar entre 0 y 180 grados.")
                dist_text = self.capture_max_distance_entry.get_text().strip()
                if dist_text:
                    distance_value = float(dist_text)
                    if not np.isfinite(distance_value) or distance_value < 0:
                        raise ValueError("La distancia maxima debe ser cero o mayor.")
                    if distance_value > 0:
                        max_distance_m = distance_value
        except ValueError as exc:
            self._show_error("Parametros invalidos", str(exc) or "Cono y distancia maxima deben ser numeros.")
            return
        forward_axis = self.capture_forward_axis_combo.get_active_text() or "z"

        self.capture_base_button.set_sensitive(False)
        self.capture_updated_button.set_sensitive(False)
        self.stop_capture_button.set_sensitive(True)
        self.capture_stop_event = threading.Event()
        label = "tunel original" if target == "base" else "tunel con shotcrete"
        self._start_capture_progress(
            f"Capturando {label}", capture_duration_s
        )
        self._log(
            f"Capturando nube ({label}) durante al menos {capture_duration_s:.0f} s; "
            + (
                "modo móvil alineado con pose SLAM; mueve el sensor lentamente..."
                if aligned_multiview
                else "se esperan 15 frames validos y se aplica mediana temporal de 5..."
            )
            + ("" if aligned_multiview else f" persistencia >= {persistence_ratio:.2f}...")
        )

        stop_event = self.capture_stop_event

        def worker():
            try:
                if aligned_multiview:
                    aligned_result = aurora_sensor.capture_aligned_snapshot(
                        self.sensor_connection,
                        duration_s=capture_duration_s,
                        timeout_s=max(10.0, capture_duration_s + 5.0),
                        stop_event=stop_event,
                        max_distance_m=max_distance_m,
                        cone_angle_deg=cone_angle_deg,
                        forward_axis=forward_axis,
                    )
                    self._log(
                        f"Fusion SLAM: {aligned_result.accepted_frames} frames aceptados, "
                        f"{aligned_result.rejected_frames} rechazados, recorrido "
                        f"{aligned_result.path_length_m:.2f} m; desplazamiento final "
                        f"{aligned_result.final_translation_m:.2f} m / "
                        f"{aligned_result.final_rotation_deg:.1f}°."
                    )
                    self._ui(
                        self._on_capture_done,
                        target,
                        aligned_result.cloud,
                        None,
                        None,
                        aligned_result.reference_pose,
                    )
                    return
                cloud = aurora_sensor.capture_snapshot(
                    self.sensor_connection,
                    persistence_ratio=persistence_ratio,
                    capture_duration_s=capture_duration_s,
                    timeout_s=max(10.0, capture_duration_s + 5.0),
                    stop_event=stop_event,
                    max_distance_m=max_distance_m,
                    cone_angle_deg=cone_angle_deg,
                    forward_axis=forward_axis,
                )
                try:
                    ref_image, ref_point_grid = aurora_sensor.capture_reference_frame(
                        self.sensor_connection, max_distance_m=max_distance_m
                    )
                except Exception:
                    ref_image, ref_point_grid = None, None
                try:
                    ref_pose = aurora_sensor.get_current_pose(self.sensor_connection)
                except Exception:
                    ref_pose = None
                self._ui(self._on_capture_done, target, cloud, ref_image, ref_point_grid, ref_pose)
            except Exception as exc:
                self._ui(self._on_capture_failed, exc)

        threading.Thread(target=worker, daemon=True).start()

    def _stop_capture_clicked(self) -> None:
        if self.capture_stop_event is not None:
            self.capture_stop_event.set()
            self.stop_capture_button.set_sensitive(False)
            self._capture_progress_stage = "Guardando ultimo frame estabilizado"
            self._capture_progress_duration_s = None
            self._update_capture_progress()
            self._log("Captura detenida manualmente; se guardara el ultimo frame estabilizado.")

    def _on_capture_done(self, target: str, cloud, ref_image=None, ref_point_grid=None, ref_pose=None) -> None:
        self._finish_capture_progress()
        self.capture_base_button.set_sensitive(True)
        self.capture_updated_button.set_sensitive(True)
        self.stop_capture_button.set_sensitive(False)
        self.capture_stop_event = None
        self.capture_status_label.set_text(
            "Captura lista. Selecciona dónde guardar la nube..."
        )
        if hasattr(self, "quick_capture_status_label"):
            self.quick_capture_status_label.set_text(
                "Captura lista. Elige dónde guardar la nube..."
            )

        import datetime

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"base_capturada_{timestamp}.ply" if target == "base" else f"updated_capturada_{timestamp}.ply"
        default_dir = PROJECT_ROOT / "data"
        default_dir.mkdir(parents=True, exist_ok=True)
        path = self._save_file_dialog("Guardar captura como", default_dir, default_name)
        if not path:
            self.capture_status_label.set_text("Captura descartada (no se eligio archivo de destino).")
            if hasattr(self, "quick_capture_status_label"):
                self.quick_capture_status_label.set_text("Captura descartada; no se guardó ningún archivo.")
            self._log("Captura descartada (no se eligio archivo de destino).")
            return

        import open3d as o3d

        o3d.io.write_point_cloud(path, cloud)
        self._log(f"Captura guardada en: {path} ({len(cloud.points)} puntos)")

        if ref_image is None or ref_point_grid is None:
            try:
                ref_image, ref_point_grid = render_reference_photo_from_cloud(cloud)
                self._log("Sin foto real del sensor; se genero una foto de referencia sintetica a partir de la nube.")
            except Exception as exc:
                self._log(f"No se pudo generar una foto de referencia sintetica: {exc}")
                ref_image, ref_point_grid = None, None

        if ref_image is not None and ref_point_grid is not None:
            try:
                save_reference_photo(Path(path), ref_image, ref_point_grid)
                self._log("Foto de referencia guardada junto a la nube (util para alinear puntos tapados por shotcrete).")
            except Exception as exc:
                self._log(f"No se pudo guardar la foto de referencia: {exc}")
        else:
            self._log("Sin foto de referencia disponible para esta captura (se podra alinear igual con la nube 3D).")

        if ref_pose is not None:
            try:
                save_reference_pose(Path(path), ref_pose.position, ref_pose.rpy_deg)
                if target == "base":
                    self._log(
                        "Pose IMU/SLAM guardada junto a ANTES. Medicion rapida puede compensar "
                        "automaticamente el cambio de pose al calcular, si el tracking sigue en el mismo mapa."
                    )
                else:
                    self._log(
                        "Pose IMU/SLAM guardada junto a DESPUES para comparar y compensar el movimiento entre capturas."
                    )
            except Exception as exc:
                self._log(f"No se pudo guardar la posicion de referencia del sensor: {exc}")
        else:
            self._log(
                "No se pudo leer la posicion del sensor para esta captura; la guia de "
                "pose IMU no estara disponible para esta nube. Aun puedes usar alineacion por puntos."
            )

        if target == "base":
            self._set_base_path(path)
            self.last_base_capture_path = path
            self.view_base_capture_button.set_sensitive(True)
            self.capture_status_label.set_text("Tunel original capturado")
            if hasattr(self, "quick_capture_status_label"):
                self.quick_capture_status_label.set_text("Túnel original capturado")
            self._refresh_imu_reference()
        else:
            self._set_updated_path(path)
            self.last_updated_capture_path = path
            self.view_updated_capture_button.set_sensitive(True)
            self.capture_status_label.set_text("Tunel con shotcrete capturado")
            if hasattr(self, "quick_capture_status_label"):
                self.quick_capture_status_label.set_text("Túnel con shotcrete capturado")
        self._refresh_quick_status()

    def _view_last_capture(self, target: str) -> None:
        path = self.last_base_capture_path if target == "base" else self.last_updated_capture_path
        if not path:
            self._show_info("Sin captura", "Todavia no capturaste este tunel.")
            return
        label = "Tunel original" if target == "base" else "Tunel con shotcrete"
        self._launch_viewer_process(
            "cloud",
            [path],
            f"Aurora - {label} (ultima captura)",
            "Error al abrir la captura",
        )

    def _on_capture_failed(self, exc: Exception) -> None:
        self._finish_capture_progress()
        self.capture_base_button.set_sensitive(True)
        self.capture_updated_button.set_sensitive(True)
        self.stop_capture_button.set_sensitive(False)
        self.capture_stop_event = None
        self.capture_status_label.set_text("")
        if hasattr(self, "quick_capture_status_label"):
            self.quick_capture_status_label.set_text(f"Error al capturar: {exc}")
        self._show_error("Error de captura", str(exc))

    def _start_capture_progress(self, label: str, duration_s: float | None) -> None:
        self._capture_progress_started_at = time.monotonic()
        self._capture_progress_stage = label
        self._capture_progress_duration_s = duration_s
        self.capture_progress_spinner.set_no_show_all(False)
        self.capture_progress_spinner.show()
        self.capture_progress_spinner.start()
        if hasattr(self, "quick_capture_spinner"):
            self.quick_capture_spinner.set_no_show_all(False)
            self.quick_capture_spinner.show()
            self.quick_capture_spinner.start()
        if hasattr(self, "quick_capture_base_button"):
            self.quick_capture_base_button.set_sensitive(False)
            self.quick_capture_updated_button.set_sensitive(False)
        if self._capture_progress_timer is None:
            self._capture_progress_timer = GLib.timeout_add_seconds(
                1, self._update_capture_progress
            )
        self._update_capture_progress()

    def _update_capture_progress(self) -> bool:
        elapsed = max(0, int(time.monotonic() - self._capture_progress_started_at))
        minutes, seconds = divmod(elapsed, 60)
        if self._capture_progress_duration_s is not None:
            duration_s = max(0, int(round(self._capture_progress_duration_s)))
            duration_min, duration_sec = divmod(
                duration_s, 60
            )
            if elapsed <= duration_s:
                stage = self._capture_progress_stage
                text = f"{stage} · {minutes:02d}:{seconds:02d} / {duration_min:02d}:{duration_sec:02d}"
            else:
                text = f"Procesando la nube · {minutes:02d}:{seconds:02d}"
        else:
            text = f"{self._capture_progress_stage} · {minutes:02d}:{seconds:02d}"
        self.capture_status_label.set_text(text)
        if hasattr(self, "quick_capture_status_label"):
            self.quick_capture_status_label.set_text(text)
        return GLib.SOURCE_CONTINUE

    def _finish_capture_progress(self) -> None:
        if self._capture_progress_timer is not None:
            GLib.source_remove(self._capture_progress_timer)
            self._capture_progress_timer = None
        self.capture_progress_spinner.stop()
        self.capture_progress_spinner.set_no_show_all(True)
        self.capture_progress_spinner.hide()
        if hasattr(self, "quick_capture_spinner"):
            self.quick_capture_spinner.stop()
            self.quick_capture_spinner.set_no_show_all(True)
            self.quick_capture_spinner.hide()
        if hasattr(self, "quick_capture_base_button"):
            connected = self.sensor_connection is not None
            self.quick_capture_base_button.set_sensitive(connected)
            self.quick_capture_updated_button.set_sensitive(connected)

    # ------------------------------------------------------- Captura en vivo (MVP)

    def _ensure_live_capture_running(self) -> bool:
        self.source_live_rb.set_active(True)
        process_running = (
            self.live_viewer_process is not None
            and self.live_viewer_process.poll() is None
        )
        if self.viewer is None or not self.viewer.is_running() or not process_running:
            self._close_viewer()
            try:
                import open3d as o3d

                # The live preview is a raw sensor view by default. Only the
                # explicit "Fijar BASE en vivo" action should enable thickness
                # coloring; a previously selected analysis file is not an
                # implicit reference for this preview.
                base_cloud = o3d.geometry.PointCloud()
                temp_dir = Path(tempfile.mkdtemp(prefix="aurora_live_view_"))
                live_path = temp_dir / "live_frame.npz"
                viewer = LiveViewer(
                    base_cloud,
                    log=self._log,
                    live_output_path=live_path,
                )
                viewer.set_live_sensor(self.sensor_connection)
                self.viewer = viewer
                self.live_view_temp_dir = temp_dir
                # Apply range, cone and color settings before the polling
                # thread starts so the first live frame is filtered too.
                self._apply_viewer_settings()
                self.viewer.start()
                process = self._launch_viewer_process(
                    "live",
                    [str(live_path)],
                    "Aurora - Captura en vivo",
                    "No se pudo abrir la vista en vivo",
                    str(temp_dir),
                )
                if process is None:
                    self._close_viewer()
                    return False
                self.live_viewer_process = process
                if self._live_view_monitor_source is None:
                    self._live_view_monitor_source = GLib.timeout_add(
                        500, self._monitor_live_viewer_process
                    )
            except Exception as exc:
                self._close_viewer()
                self._show_error("No se pudo iniciar la vista en vivo", str(exc))
                return False
        self._apply_viewer_settings()
        self.viewer.set_live_sensor(self.sensor_connection)
        return True

    def _monitor_live_viewer_process(self) -> bool:
        if self.live_viewer_process is None:
            self._live_view_monitor_source = None
            return GLib.SOURCE_REMOVE
        if self.live_viewer_process.poll() is None:
            return GLib.SOURCE_CONTINUE
        self._log("La ventana 3D en vivo se cerro; se detuvo la lectura continua del sensor.")
        self._live_view_monitor_source = None
        self._close_viewer()
        return GLib.SOURCE_REMOVE

    def _start_live_capture_clicked(self) -> None:
        if self.sensor_connection is None:
            self._show_warning("Sensor no conectado", "Conecta el sensor antes de iniciar captura en tiempo real.")
            return
        if not self._ensure_live_capture_running():
            return
        self._log("Captura en tiempo real activa en la vista 3D.")

    def _capture_live_baseline_clicked(self) -> None:
        if self.sensor_connection is None:
            self._show_warning("Sensor no conectado", "Conecta el sensor antes de fijar la base en vivo.")
            return
        if not self._ensure_live_capture_running():
            return
        try:
            count = self.viewer.capture_live_baseline()
        except RuntimeError as exc:
            self._show_info("Frame aun no disponible", str(exc))
            return
        self._log(f"BASE en vivo fijada con {count} puntos. La comparacion continua se actualiza en tiempo real.")

    def _clear_live_baseline_clicked(self) -> None:
        if self.viewer is None or not self.viewer.is_running():
            self._show_info("Vista 3D cerrada", "Abre primero la vista 3D para quitar la base en vivo.")
            return
        self.viewer.clear_live_baseline()
        self._log("BASE en vivo quitada. Se restauro la nube base original.")

    def _save_live_clouds_clicked(self) -> None:
        if not self._ensure_live_capture_running():
            return

        baseline_cloud, current_cloud, using_live_baseline = self.viewer.get_live_snapshot_clouds()
        if baseline_cloud is None or current_cloud is None:
            self._show_info("Sin frame disponible", "Aun no hay nube en vivo para guardar.")
            return

        dialog = Gtk.FileChooserNative.new(
            "Seleccionar carpeta para guardar nubes en vivo",
            self.window,
            Gtk.FileChooserAction.SELECT_FOLDER,
            "_Seleccionar",
            "_Cancelar",
        )
        target_dir = self._run_native_file_chooser(dialog)
        if not target_dir:
            self._log("Guardado de nubes en vivo cancelado.")
            return

        import datetime

        import open3d as o3d

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        baseline_name = "base_live_fijada" if using_live_baseline else "base_referencia_original"
        base_path = Path(target_dir) / f"{baseline_name}_{timestamp}.ply"
        updated_path = Path(target_dir) / f"updated_live_{timestamp}.ply"

        o3d.io.write_point_cloud(str(base_path), baseline_cloud)
        o3d.io.write_point_cloud(str(updated_path), current_cloud)
        self._log(f"Nubes en vivo guardadas en: {base_path} y {updated_path}")

        for cloud, cloud_path in ((baseline_cloud, base_path), (current_cloud, updated_path)):
            try:
                ref_image, ref_point_grid = render_reference_photo_from_cloud(cloud)
                save_reference_photo(cloud_path, ref_image, ref_point_grid)
            except Exception as exc:
                self._log(f"No se pudo generar foto de referencia para {cloud_path.name}: {exc}")
        self._log("Fotos de referencia (sinteticas) guardadas junto a las nubes en vivo.")

    # ---------------------------------------------------------- Pipeline

    def _confirm_missing_alignment(self) -> bool:
        """Advierte si no se aplico alineacion por puntos de referencia ni ICP.

        Sin alguna de las dos, un desplazamiento del sensor entre capturas se
        mide como si fuera espesor real de shotcrete.
        """
        dialog = Gtk.MessageDialog(
            transient_for=self.window,
            flags=0,
            message_type=Gtk.MessageType.WARNING,
            buttons=Gtk.ButtonsType.NONE,
            text="No se aplico alineacion entre las dos capturas",
        )
        dialog.format_secondary_text(
            "Si el sensor se reubico entre la captura del tunel original y la del tunel "
            "con shotcrete, el resultado puede incluir ese desplazamiento como si fuera "
            "espesor real, en vez de solo el shotcrete aplicado.\n\n"
            "Puedes ir a 'Alineacion por puntos' y marcar puntos de referencia fijos "
            "(por ejemplo pernos o marcas) antes de calcular, o continuar si sabes que "
            "las dos capturas se tomaron desde la misma posicion."
        )
        dialog.add_button("Ir a puntos de referencia", Gtk.ResponseType.CANCEL)
        continue_button = dialog.add_button("Continuar de todas formas", Gtk.ResponseType.OK)
        continue_button.get_style_context().add_class("destructive-action")
        response = dialog.run()
        dialog.destroy()
        if response == Gtk.ResponseType.OK:
            return True
        if hasattr(self, "quick_status_label"):
            self.quick_status_label.set_text("Alinea las capturas con puntos de referencia y luego vuelve a calcular.")
        self.stack.set_visible_child_name("alineacion")
        return False

    def _run_pipeline_clicked(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            self._show_warning("En progreso", "Ya hay un analisis en ejecucion.")
            return

        try:
            params = self._build_params()
        except Exception as exc:
            self._show_error("Parametros invalidos", str(exc))
            return

        if not self.alignment_applied and not params.use_icp:
            if not self._confirm_missing_alignment():
                return

        self._invalidate_results()
        generation = self._result_generation
        self.log_buffer.set_text("")
        self.quick_calculate_button.set_sensitive(False)
        self.result_recalculate_button.set_sensitive(False)
        self._start_calculation_progress("Preparando Cloud-to-Cloud")
        self.status_label.set_markup("Calculando el espesor...")
        self.stack.set_visible_child_name("resultados")

        self.worker_thread = threading.Thread(
            target=self._run_pipeline_worker, args=(params, generation), daemon=True
        )
        self.worker_thread.start()

    def _build_params(self) -> PipelineParams:
        base_path = Path(self.base_path)
        updated_path = Path(self.updated_path)
        output_dir = Path(self.output_dir)

        max_distance_text = self.max_distance_entry.get_text().strip()
        max_distance = float(max_distance_text) if max_distance_text else None

        return PipelineParams(
            base_path=base_path,
            updated_path=updated_path,
            output_dir=output_dir,
            voxel_size=float(self.voxel_entry.get_text() or 0.0),
            remove_outliers=self.remove_outliers_check.get_active(),
            use_icp=self.use_icp_check.get_active(),
            icp_threshold=float(self.icp_threshold_entry.get_text() or 0.05),
            max_distance=max_distance,
        )

    def _run_pipeline_worker(self, params: PipelineParams, generation: int) -> None:
        try:
            result = run_pipeline(params, log=self._log)
            self._log("\nListo.")
            self._ui(self._on_pipeline_success, result, generation)
        except Exception as exc:
            self._log(f"\nERROR: {exc}")
            self._ui(self._on_pipeline_failure, str(exc), generation)
        finally:
            self._ui(self.quick_calculate_button.set_sensitive, True)
            self._ui(self._refresh_recalculate_button)

    def _on_pipeline_success(self, result, generation: int) -> None:
        if generation != self._result_generation:
            return
        self._finish_calculation_progress(True)
        self.result = result
        stats = self.result.stats
        self._active_result_kind = "cloud_to_cloud"
        self._set_stat(self.stat_points, f"{stats.n_points:,}")
        self._set_stat(self.stat_mean, f"{stats.mean * 100:.2f} cm")
        self._set_stat(self.stat_median, f"{stats.median * 100:.2f} cm", COLOR_MEDIAN)
        self._set_stat(self.stat_std, f"{stats.std * 100:.2f} cm")
        self.cards_row.set_no_show_all(False)
        self.cards_row.show_all()
        self.results_empty_label.hide()
        self.open_result_viewer_button.set_sensitive(True)
        self.open_result_viewer_button.set_label("Abrir vista 3D")
        self.status_label.set_markup(
            f'<span foreground="{COLOR_OK}">●</span>  Analisis completo — '
            f"espesor promedio {stats.mean * 100:.2f} cm"
        )
        self.generate_report_button.set_sensitive(True)
        self.quick_status_label.set_text(
            f"Medicion lista: promedio {stats.mean * 100:.2f} cm, "
            f"mediana {stats.median * 100:.2f} cm, desviación estándar {stats.std * 100:.2f} cm."
        )
        self._set_calculation_method("cloud_to_cloud")
        self._generate_alerts()
        self.stack.set_visible_child_name("resultados")
        self._show_result_in_viewer()

    def _on_pipeline_failure(self, message: str, generation: int) -> None:
        if generation != self._result_generation:
            return
        self._finish_calculation_progress(False)
        self.quick_status_label.set_text(f"No se pudo calcular: {message}")
        self.status_label.set_markup(f'<span foreground="{COLOR_ERROR}">●</span>  El analisis fallo')
        self.stack.set_visible_child_name("resultados")
        self._show_error("Error durante el analisis", message)

    # ------------------------------------------------------- Alertas e informe

    def _active_result(self):
        if self._active_result_kind == "raycast":
            return self.raycast_result
        if self._active_result_kind == "cloud_to_cloud":
            return self.result
        return self.result or self.raycast_result

    def _generate_alerts(self) -> None:
        result = self._active_result()
        if result is None:
            self.thickness_alert_label.hide()
            return
        try:
            low_cm, high_cm = self._band_thresholds_cm()
        except ValueError as exc:
            self.thickness_alert_label.set_markup(
                f'<span foreground="{COLOR_WARN}"><b>Revisa los umbrales de espesor:</b> '
                f'{GLib.markup_escape_text(str(exc))}</span>'
            )
            self.thickness_alert_label.set_no_show_all(False)
            self.thickness_alert_label.show()
            return
        mean_cm = result.stats.mean * 100

        if mean_cm < low_cm:
            message = (
                f"Espesor medio: {mean_cm:.2f} cm, bajo el minimo de {low_cm:g} cm. "
                "Falta aplicar shotcrete."
            )
        elif mean_cm >= high_cm:
            message = (
                f"Espesor medio: {mean_cm:.2f} cm, sobre el maximo de {high_cm:g} cm. "
                "Exceso de shotcrete."
            )
        else:
            self.thickness_alert_label.set_text("")
            self.thickness_alert_label.set_no_show_all(True)
            self.thickness_alert_label.hide()
            return

        self.thickness_alert_label.set_markup(
            f'<span foreground="{COLOR_WARN}"><b>Alerta de espesor:</b> '
            f'{GLib.markup_escape_text(message)}</span>'
        )
        self.thickness_alert_label.set_no_show_all(False)
        self.thickness_alert_label.show()

    def _generate_report(self) -> None:
        result = self._active_result()
        if result is None:
            return

        try:
            legend = self._selected_color_legend()
            low_cm, high_cm = (self._band_thresholds_cm()
                               if self.color_banded_rb.get_active() else (None, None))
        except ValueError as exc:
            self._show_warning("Revisa los ajustes de visualizacion en Resultados", str(exc))
            self.stack.set_visible_child_name("resultados")
            return

        import datetime
        from xml.sax.saxutils import escape

        try:
            from reportlab.lib import colors as rl_colors
            from reportlab.lib.pagesizes import A4
            from reportlab.lib.units import cm as rl_cm
            from reportlab.platypus import (
                Image,
                Paragraph,
                SimpleDocTemplate,
                Spacer,
                Table,
                TableStyle,
            )
            from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
            from reportlab.lib.enums import TA_CENTER
        except ImportError as exc:
            self._show_error(
                "Falta la dependencia del informe PDF",
                f"No se pudo cargar ReportLab: {exc}. Reinstala las dependencias de Aurora.",
            )
            return

        default_dir = Path(self.output_dir)
        default_dir.mkdir(parents=True, exist_ok=True)
        now = datetime.datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        report_path = default_dir / f"informe_aurora_{timestamp}.pdf"

        stats = result.stats
        mean_cm = stats.mean * 100

        if low_cm is not None and mean_cm < low_cm:
            status_text, status_color = "FALTA SHOTCRETE (bajo el umbral)", rl_colors.HexColor("#FFCC00")
        elif high_cm is not None and mean_cm >= high_cm:
            status_text, status_color = "EXCESO DE SHOTCRETE (sobre el umbral)", rl_colors.HexColor("#FF3B30")
        elif self.color_six_bands_rb.get_active() and mean_cm < float(legend["target_cm"]):
            status_text, status_color = "ESPESOR OBJETIVO AUN NO ALCANZADO", rl_colors.HexColor("#FFCC00")
        elif self.color_six_bands_rb.get_active():
            status_text, status_color = "ESPESOR OBJETIVO ALCANZADO", rl_colors.HexColor("#34C759")
        elif self.color_continuous_rb.get_active():
            status_text, status_color = "MEDICION DE ESPESOR COMPLETADA", rl_colors.HexColor("#007AFF")
        else:
            status_text, status_color = "DENTRO DE PARAMETRO", rl_colors.HexColor("#34C759")

        styles = getSampleStyleSheet()
        title_style = ParagraphStyle(
            "AuroraTitle", parent=styles["Title"], textColor=rl_colors.HexColor("#1A1D24"), spaceAfter=4,
        )
        subtitle_style = ParagraphStyle(
            "AuroraSubtitle", parent=styles["Normal"], textColor=rl_colors.HexColor("#8E95A5"), fontSize=10,
        )
        h2_style = ParagraphStyle(
            "AuroraH2", parent=styles["Heading2"], textColor=rl_colors.HexColor("#CC7000"), spaceBefore=14, spaceAfter=6,
        )
        body_style = styles["Normal"]
        status_style = ParagraphStyle(
            "AuroraStatus",
            parent=styles["Heading2"],
            textColor=rl_colors.white,
            backColor=status_color,
            alignment=TA_CENTER,
            borderPadding=8,
            spaceAfter=6,
        )

        story = [
            Paragraph("Informe de Medición de Espesor de Shotcrete", title_style),
            Paragraph("Proyecto: CORFO Eureka (Aurora)", subtitle_style),
            Paragraph(f"Fecha y hora: {now.strftime('%Y-%m-%d %H:%M:%S')}", subtitle_style),
            Spacer(1, 12),
            Paragraph(status_text, status_style),
            Spacer(1, 6),
            Paragraph("1. Archivos analizados", h2_style),
            Paragraph(f"<b>Nube base (original):</b> {escape(str(self.base_path))}", body_style),
            Paragraph(f"<b>Nube actualizada (shotcrete):</b> {escape(str(self.updated_path))}", body_style),
            Paragraph(
                f"<b>Metodo de calculo:</b> {'Raycasting con malla' if self._active_result_kind == 'raycast' else 'Cloud-to-Cloud'}",
                body_style,
            ),
            Paragraph("2. Criterio de color", h2_style),
            Paragraph(
                (f"Umbral bajo: {low_cm:g} cm — Umbral alto: {high_cm:g} cm"
                 if low_cm is not None else
                 f"Espesor objetivo: {float(legend['target_cm']):.2f} cm"
                 if self.color_six_bands_rb.get_active() else
                 f"Escala continua entre 0 y {float(legend['continuous_max_cm']):.2f} cm"),
                body_style,
            ),
            Paragraph("3. Estadísticas de espesor", h2_style),
        ]

        stats_rows = [
            ["Métrica", "Valor (cm)"],
            ["Puntos analizados", f"{stats.n_points:,}"],
            ["Espesor medio", f"{stats.mean * 100:.2f}"],
            ["Espesor mediano", f"{stats.median * 100:.2f}"],
            ["Desviación estándar", f"{stats.std * 100:.2f}"],
        ]
        stats_table = Table(stats_rows, colWidths=[8 * rl_cm, 6 * rl_cm])
        stats_table.setStyle(
            TableStyle(
                [
                    ("BACKGROUND", (0, 0), (-1, 0), rl_colors.HexColor("#1A1D24")),
                    ("TEXTCOLOR", (0, 0), (-1, 0), rl_colors.white),
                    ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                    ("FONTNAME", (0, 1), (0, 1), "Helvetica-Bold"),
                    ("BACKGROUND", (0, 1), (-1, 1), rl_colors.HexColor("#FFE9CC")),
                    ("GRID", (0, 0), (-1, -1), 0.5, rl_colors.HexColor("#2E3440")),
                    ("ROWBACKGROUNDS", (0, 2), (-1, -1), [rl_colors.white, rl_colors.HexColor("#F5F6F8")]),
                    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                    ("TOPPADDING", (0, 0), (-1, -1), 6),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
                    ("LEFTPADDING", (0, 0), (-1, -1), 8),
                ]
            )
        )
        story.append(stats_table)

        story.append(Paragraph(f"4. Escala de colores ({escape(legend['title'])})", h2_style))
        story.append(Spacer(1, 6))
        band_rows = [["Rango (cm)", "Nivel", "Color"]]
        band_styles = [
            ("BACKGROUND", (0, 0), (-1, 0), rl_colors.HexColor("#1A1D24")),
            ("TEXTCOLOR", (0, 0), (-1, 0), rl_colors.white),
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("GRID", (0, 0), (-1, -1), 0.5, rl_colors.HexColor("#2E3440")),
            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
            ("TOPPADDING", (0, 0), (-1, -1), 6),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
        ]
        legend_rows = legend.get("rows")
        if legend_rows:
            for color, label, value in legend_rows:
                r, g, b = color
                band_rows.append([value, label, ""])
                row_idx = len(band_rows) - 1
                band_styles.append(("BACKGROUND", (2, row_idx), (2, row_idx), rl_colors.Color(r, g, b)))
        else:
            max_cm = float(legend["continuous_max_cm"])
            band_rows.extend([
                ["0 cm", "Mínimo", ""],
                [f">= {max_cm:.2f} cm", "Máximo (color recortado)", ""],
            ])
            band_styles.extend([
                ("BACKGROUND", (2, 1), (2, 1), rl_colors.Color(*legend["low_color"])),
                ("BACKGROUND", (2, 2), (2, 2), rl_colors.Color(*legend["high_color"])),
            ])
        band_table = Table(band_rows, colWidths=[5 * rl_cm, 5 * rl_cm, 4 * rl_cm])
        band_table.setStyle(TableStyle(band_styles))
        story.append(band_table)

        story.append(Paragraph("5. Distribución (histograma)", h2_style))
        try:
            story.append(Image(str(result.histogram_path), width=15 * rl_cm, height=9 * rl_cm))
        except Exception as exc:
            story.append(Paragraph(f"(No se pudo incluir el histograma: {exc})", body_style))

        story.append(Paragraph("6. Archivos generados", h2_style))
        story.append(
            Paragraph(
                f"Los siguientes archivos se encuentran en el mismo directorio de este reporte "
                f"({default_dir}):",
                body_style,
            )
        )
        story.append(Paragraph(f"• Detalle por punto: {result.csv_path.name}", body_style))
        story.append(Paragraph(f"• Histograma: {result.histogram_path.name}", body_style))
        story.append(Paragraph(f"• Nube 3D heatmap: {result.heatmap_path.name}", body_style))

        doc = SimpleDocTemplate(
            str(report_path),
            pagesize=A4,
            leftMargin=2 * rl_cm,
            rightMargin=2 * rl_cm,
            topMargin=1.5 * rl_cm,
            bottomMargin=1.5 * rl_cm,
        )
        try:
            doc.build(story)
        except Exception as exc:
            self._show_error("No se pudo generar el informe PDF", str(exc))
            return

        self._log(f"Informe generado: {report_path}")
        try:
            opened = Gtk.show_uri_on_window(
                self.window, report_path.resolve().as_uri(), Gdk.CURRENT_TIME
            )
            if not opened:
                raise RuntimeError("Ubuntu no encontro una aplicacion asociada a archivos PDF.")
        except Exception as exc:
            self._log(f"No se pudo abrir automaticamente el PDF: {exc}")
            self._show_info(
                "Informe generado",
                f"El informe se guardo en:\n{report_path}\n\n"
                f"No se pudo abrir automaticamente: {exc}",
            )
            return

        self._show_info(
            "Informe generado",
            f"El informe PDF se guardo y se abrio:\n{report_path}",
        )

    # --------------------------------------------------------------- Viewer

    @staticmethod
    def _parse_positive_entry(entry: Gtk.Entry, label: str) -> float:
        text = entry.get_text().strip().replace(",", ".")
        try:
            value = float(text) if text else 12.0
        except ValueError as exc:
            raise ValueError(f"{label} debe ser un numero mayor que cero.") from exc
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{label} debe ser un numero mayor que cero.")
        return value

    def _band_thresholds_cm(self) -> tuple[float, float]:
        try:
            low_cm = float((self.band_low_entry.get_text().strip() or "5").replace(",", "."))
            high_cm = float((self.band_high_entry.get_text().strip() or "10").replace(",", "."))
        except ValueError as exc:
            raise ValueError("Los umbrales verde y rojo deben ser numeros en centimetros.") from exc
        if not np.isfinite(low_cm) or not np.isfinite(high_cm) or low_cm < 0 or high_cm <= low_cm:
            raise ValueError("El umbral rojo debe ser mayor que el verde y ambos deben ser cero o mas.")
        return low_cm, high_cm

    def _band_thresholds_m(self) -> tuple[float, float]:
        low_cm, high_cm = self._band_thresholds_cm()
        return low_cm / 100.0, high_cm / 100.0

    def _parse_live_filter_params(self) -> tuple[float | None, float | None]:
        max_distance = None
        text = self.live_max_distance_entry.get_text().strip()
        if text:
            try:
                distance_value = float(text.replace(",", "."))
            except ValueError as exc:
                raise ValueError("la distancia maxima debe ser un numero mayor o igual a cero") from exc
            if not np.isfinite(distance_value) or distance_value < 0:
                raise ValueError("la distancia maxima debe ser >= 0")
            max_distance = distance_value if distance_value > 0 else None

        cone_angle = None
        text = self.live_cone_angle_entry.get_text().strip()
        if text:
            try:
                cone_angle = float(text.replace(",", "."))
            except ValueError as exc:
                raise ValueError("el cono debe ser un numero entre 0 y 180 grados") from exc
            if not np.isfinite(cone_angle) or cone_angle <= 0 or cone_angle > 180:
                raise ValueError("el cono debe estar en (0, 180]")

        return max_distance, cone_angle

    def _sync_capture_filters_to_live(self) -> None:
        if self._syncing_capture_live_filters or not hasattr(self, "live_max_distance_entry"):
            return
        self._syncing_capture_live_filters = True
        try:
            filters_enabled = self.capture_limit_fov_check.get_active()
            distance = self.capture_max_distance_entry.get_text().strip() if filters_enabled else "0"
            self.live_max_distance_entry.set_text(distance)
            cone = (
                self.capture_cone_angle_entry.get_text().strip()
                if filters_enabled
                else ""
            )
            self.live_cone_angle_entry.set_text(cone)
            axis = self.capture_forward_axis_combo.get_active_text() or "z"
            for index in range(3):
                if self.live_forward_axis_combo.get_model()[index][0] == axis:
                    self.live_forward_axis_combo.set_active(index)
                    break
        finally:
            self._syncing_capture_live_filters = False
        self._apply_viewer_settings()

    def _sync_live_filters_to_capture(self) -> None:
        if self._syncing_capture_live_filters or not hasattr(self, "capture_max_distance_entry"):
            return
        self._syncing_capture_live_filters = True
        try:
            self.capture_max_distance_entry.set_text(self.live_max_distance_entry.get_text())
            cone = self.live_cone_angle_entry.get_text().strip()
            distance = self.live_max_distance_entry.get_text().strip()
            try:
                distance_active = float(distance.replace(",", ".")) > 0
            except ValueError:
                distance_active = bool(distance)
            self.capture_limit_fov_check.set_active(bool(cone or distance_active))
            if cone:
                self.capture_cone_angle_entry.set_text(cone)
            active_axis = self.live_forward_axis_combo.get_active_text() or "z"
            for index in range(3):
                if self.capture_forward_axis_combo.get_model()[index][0] == active_axis:
                    self.capture_forward_axis_combo.set_active(index)
                    break
        finally:
            self._syncing_capture_live_filters = False

    def _open_active_result_visualization(self) -> None:
        self._show_result_in_viewer()

    def _build_raycast_colored_meshes(self, result):
        """Color the reconstructed surfaces from their nearest measured samples."""
        import open3d as o3d
        from scipy.spatial import cKDTree

        base_mesh = o3d.geometry.TriangleMesh(result.base_mesh)
        updated_mesh = o3d.geometry.TriangleMesh(result.updated_mesh)
        base_cloud = grayscale_point_cloud(load_point_cloud(Path(self.base_path)))
        base_points = np.asarray(base_cloud.points)
        base_vertices = np.asarray(base_mesh.vertices)
        if len(base_points) and len(base_vertices):
            base_colors = np.asarray(base_cloud.colors)
            nearest = cKDTree(base_points).query(base_vertices, k=1)[1]
            base_mesh.vertex_colors = o3d.utility.Vector3dVector(base_colors[nearest])
        else:
            base_mesh.paint_uniform_color((0.45, 0.45, 0.45))

        vertices = np.asarray(updated_mesh.vertices)
        hit_points = np.asarray(result.hit_points, dtype=np.float64)
        distances = np.asarray(result.distances, dtype=np.float64)
        if not len(vertices) or not len(hit_points) or len(hit_points) != len(distances):
            raise ValueError("No hay suficientes impactos para colorear la malla DESPUÉS.")

        hit_tree = cKDTree(hit_points)
        nearest_distance, nearest_index = hit_tree.query(vertices, k=1)
        if len(hit_points) > 1:
            sample_spacing = cKDTree(hit_points).query(hit_points, k=2)[0][:, 1]
            color_radius = min(0.05, max(0.015, float(np.percentile(sample_spacing, 90)) * 3.0))
        else:
            color_radius = 0.015

        vertex_cloud = o3d.geometry.PointCloud()
        vertex_cloud.points = o3d.utility.Vector3dVector(vertices)
        vertex_distances = distances[nearest_index]
        spec = self._selected_preview_heatmap_spec()
        if spec["mode"] == "six":
            colored = build_heatmap_cloud_six_bands(vertex_cloud, vertex_distances, spec["target_m"])
        elif spec["mode"] == "three":
            colored = build_heatmap_cloud_banded(
                vertex_cloud, vertex_distances, spec["low_m"], spec["high_m"]
            )
        else:
            colored = build_heatmap_cloud(
                vertex_cloud, vertex_distances, spec.get("max_distance_m")
            )
        vertex_colors = np.asarray(colored.colors).copy()
        vertex_colors[nearest_distance > color_radius] = (0.45, 0.45, 0.45)
        updated_mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)
        base_mesh.compute_vertex_normals()
        updated_mesh.compute_vertex_normals()
        return base_mesh, updated_mesh

    def _launch_viewer_process(
        self,
        kind: str,
        paths: list[str],
        title: str,
        error_title: str,
        cleanup_dir: str | None = None,
        legend: dict | None = None,
        box_bounds: tuple[list[float], list[float]] | None = None,
        heatmap_spec: dict | None = None,
        thickness_stats: dict | None = None,
        camera_reference_paths: list[str] | None = None,
        empty_stages: list[str] | None = None,
    ) -> subprocess.Popen | None:
        command = [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "viewer_host.py"),
            kind,
            *paths,
            "--title",
            title,
        ]
        if cleanup_dir:
            command.extend(("--cleanup-dir", cleanup_dir))
        if legend is not None:
            command.extend(("--legend-json", json.dumps(legend)))
        if box_bounds is not None:
            command.extend(("--box-bounds-json", json.dumps(box_bounds)))
        if camera_reference_paths:
            for path in camera_reference_paths:
                command.extend(("--camera-reference", path))
        if empty_stages:
            command.extend(("--empty-stage", *empty_stages))
        if heatmap_spec is not None:
            command.extend(("--heatmap-spec-json", json.dumps(heatmap_spec)))
        if thickness_stats is not None:
            command.extend(("--thickness-stats-json", json.dumps(thickness_stats)))
        try:
            process = subprocess.Popen(command, cwd=PROJECT_ROOT, start_new_session=True)
        except OSError as exc:
            if cleanup_dir:
                shutil.rmtree(cleanup_dir, ignore_errors=True)
            self._show_error(error_title, str(exc))
            return None

        self.external_viewer_processes = [
            child for child in self.external_viewer_processes if child.poll() is None
        ]
        self.external_viewer_processes.append(process)
        return process

    def _show_result_in_viewer(self) -> None:
        result = self._active_result()
        if result is None:
            self._show_warning("Sin resultado", "Calcula el espesor antes de abrir la vista 3D.")
            return
        temp_dir: Path | None = None
        try:
            legend = self._selected_color_legend()
            temp_dir = Path(tempfile.mkdtemp(prefix="aurora_thickness_view_"))
            import open3d as o3d

            if self._active_result_kind == "raycast":
                base_mesh, updated_mesh = self._build_raycast_colored_meshes(result)
                base_path = temp_dir / "malla_antes.ply"
                updated_path = temp_dir / "malla_despues_espesor.ply"
                if not o3d.io.write_triangle_mesh(str(base_path), base_mesh):
                    raise OSError("No se pudo preparar la malla ANTES.")
                if not o3d.io.write_triangle_mesh(str(updated_path), updated_mesh):
                    raise OSError("No se pudo preparar la malla DESPUÉS coloreada por espesor.")
                legend = {
                    **legend,
                    "title": f"Espesor · {legend.get('title', 'Mapa de color')}",
                    "extra_rows": [((0.45, 0.45, 0.45), "ANTES", "Superficie original")],
                }
                viewer_kind = "meshes"
                viewer_paths = [str(base_path), str(updated_path)]
            else:
                colored_cloud, legend = self._build_result_heatmap(result)
                heatmap_path = temp_dir / "espesor_coloreado.ply"
                if not o3d.io.write_point_cloud(str(heatmap_path), colored_cloud):
                    raise OSError("No se pudo preparar la nube coloreada para la vista 3D.")
                viewer_kind = "cloud"
                viewer_paths = [str(heatmap_path)]
            if self._active_result_kind == "cloud_to_cloud":
                before_path = temp_dir / "nube_antes.ply"
                if not o3d.io.write_point_cloud(str(before_path), result.base_cloud):
                    raise OSError("No se pudo preparar la nube ANTES para la vista 3D.")
                viewer_kind = "thickness"
                viewer_paths = [str(before_path), str(heatmap_path)]
        except Exception as exc:
            if temp_dir is not None:
                shutil.rmtree(temp_dir, ignore_errors=True)
            self._show_error("No se pudo preparar la leyenda de espesor", str(exc))
            return

        process = self._launch_viewer_process(
            viewer_kind,
            viewer_paths,
            "Aurora - Espesor: Q para volver",
            "No se pudo abrir la vista 3D",
            str(temp_dir),
            legend,
            thickness_stats={
                "mean_cm": result.stats.mean * 100.0,
                "median_cm": result.stats.median * 100.0,
                "std_cm": result.stats.std * 100.0,
            },
        )

    def _selected_color_legend(self) -> dict:
        """Single source of truth for the selected GUI and PDF thickness legend."""
        if self.color_six_bands_rb.get_active():
            target_cm = self._parse_positive_entry(self.target_thickness_entry, "El espesor objetivo")
            band_width_cm = target_cm / len(SIX_BAND_LABELS)
            rows = []
            for index, (label, color) in enumerate(zip(SIX_BAND_LABELS, SIX_BAND_COLORS)):
                lower = index * band_width_cm
                value = (
                    f">= {lower:.1f} cm (tope {target_cm:.1f})"
                    if index == len(SIX_BAND_LABELS) - 1
                    else f"{lower:.1f}-{(index + 1) * band_width_cm:.1f} cm"
                )
                rows.append((color, label, value))
            return {"title": "6 niveles", "rows": rows, "target_cm": target_cm}

        if self.color_banded_rb.get_active():
            low_cm, high_cm = self._band_thresholds_cm()
            return {
                "title": "3 niveles",
                "rows": [
                    (BAND_COLOR_LOW, "Bajo", f"< {low_cm:g} cm"),
                    (BAND_COLOR_MEDIUM, "Intermedio", f"{low_cm:g}-{high_cm:g} cm"),
                    (BAND_COLOR_HIGH, "Alto", f">= {high_cm:g} cm"),
                ],
            }

        max_text = self.max_distance_entry.get_text().strip().replace(",", ".")
        max_distance = float(max_text) if max_text else None
        result = self._active_result()
        distances = np.asarray(result.distances, dtype=np.float64)
        clip_max = max_distance if max_distance else float(np.percentile(distances, 98))
        return {
            "title": "Escala continua",
            "continuous_max_cm": max(clip_max, 1e-9) * 100.0,
            "low_color": HEATMAP_COLOR_LOW,
            "high_color": HEATMAP_COLOR_HIGH,
        }

    def _build_result_heatmap(self, result) -> tuple[object, dict]:
        """Make the result viewer colors match the selected legend exactly."""
        import open3d as o3d

        if self._active_result_kind == "raycast":
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(result.hit_points)
        else:
            cloud = o3d.geometry.PointCloud(result.updated_cloud)
        distances = np.asarray(result.distances, dtype=np.float64)
        if len(cloud.points) != distances.size or distances.size == 0:
            raise ValueError("El resultado no contiene puntos y distancias compatibles.")

        legend = self._selected_color_legend()
        if self.color_six_bands_rb.get_active():
            target_cm = self._parse_positive_entry(self.target_thickness_entry, "El espesor objetivo")
            target_m = target_cm / 100.0
            colored = build_heatmap_cloud_six_bands(cloud, distances, target_m)
            legend = {**legend, "title": "Espesor · 6 niveles"}
            return self._append_raycast_unmatched(colored, legend, result)

        if self.color_banded_rb.get_active():
            low_cm, high_cm = self._band_thresholds_cm()
            colored = build_heatmap_cloud_banded(
                cloud, distances, low_cm / 100.0, high_cm / 100.0
            )
            legend = {**legend, "title": "Espesor · 3 niveles"}
            return self._append_raycast_unmatched(colored, legend, result)

        max_text = self.max_distance_entry.get_text().strip().replace(",", ".")
        max_distance = float(max_text) if max_text else None
        clip_max = max_distance if max_distance else float(np.percentile(distances, 98))
        clip_max = max(clip_max, 1e-9)
        colored = build_heatmap_cloud(cloud, distances, max_distance)
        legend = {**legend, "title": "Espesor · escala continua"}
        return self._append_raycast_unmatched(colored, legend, result)

    @staticmethod
    def _append_raycast_unmatched(colored, legend: dict, result):
        """Show ray misses in neutral gray without inventing thickness values."""
        if getattr(result, "unmatched_points", np.empty((0, 3))).size:
            import open3d as o3d

            unmatched = o3d.geometry.PointCloud()
            unmatched.points = o3d.utility.Vector3dVector(result.unmatched_points)
            unmatched.paint_uniform_color((0.58, 0.60, 0.63))
            colored += unmatched
            legend = {
                **legend,
                "extra_rows": [((0.58, 0.60, 0.63), "Sin dato", "Sin impacto de rayo")],
            }
        return colored, legend

    def _open_viewer(self) -> None:
        if self.viewer is not None and self.viewer.is_running():
            self._show_info("Vista 3D", "La vista 3D ya esta abierta.")
            return

        try:
            base_file = Path(self.base_path) if self.base_path else None
            has_base_file = base_file is not None and base_file.is_file()
            live_without_baseline = (
                self.source_live_rb.get_active()
                and self.sensor_connection is not None
                and not has_base_file
            )
            if live_without_baseline:
                import open3d as o3d

                base_cloud = o3d.geometry.PointCloud()
            elif not has_base_file:
                expected_path = base_file if base_file is not None else "(sin seleccionar)"
                self._show_error("Error", f"No se encontro la nube base:\n{expected_path}")
                return
            else:
                base_cloud = self.result.base_cloud if self.result is not None else load_point_cloud(base_file)
        except Exception as exc:
            self._show_error("Error", str(exc))
            return

        self.viewer = LiveViewer(base_cloud, log=self._log)
        self.viewer.start()
        self.close_viewer_button.set_sensitive(True)
        self._apply_viewer_settings()
        self._log("Vista 3D abierta.")

    def _close_viewer(self) -> None:
        if self.viewer is not None:
            self.viewer.stop()
            self.viewer = None
        if self.live_viewer_process is not None:
            if self.live_viewer_process.poll() is None:
                self.live_viewer_process.terminate()
                try:
                    self.live_viewer_process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.live_viewer_process.kill()
            self.live_viewer_process = None
        if self.live_view_temp_dir is not None:
            shutil.rmtree(self.live_view_temp_dir, ignore_errors=True)
            self.live_view_temp_dir = None
        if self._live_view_monitor_source is not None:
            GLib.source_remove(self._live_view_monitor_source)
            self._live_view_monitor_source = None
        self.close_viewer_button.set_sensitive(False)
        self._clear_stream_ui()

    def _clear_stream_ui(self) -> None:
        self.stream_toggle_button.set_label("Iniciar transmision")
        self.stream_url_label.set_text("")
        self.stream_qr_image.clear()

    def _set_stream_qr(self, url: str) -> None:
        qr_img = make_qr_image(url).resize((160, 160))
        data = qr_img.tobytes()
        pixbuf = GdkPixbuf.Pixbuf.new_from_data(
            data, GdkPixbuf.Colorspace.RGB, False, 8, qr_img.width, qr_img.height, qr_img.width * 3
        )
        self.stream_qr_image.set_from_pixbuf(pixbuf)
        # GdkPixbuf.new_from_data no copia el buffer: se guarda una referencia para que no lo recolecte el GC.
        self.stream_qr_image._qr_data_ref = data

    def _toggle_stream_clicked(self) -> None:
        if self.viewer is not None and self.viewer.is_streaming():
            self.viewer.stop_stream()
            self._clear_stream_ui()
            self._log("Transmision al celular detenida.")
            return

        if self.viewer is None or not self.viewer.is_running():
            if self.sensor_connection is not None:
                if not self._ensure_live_capture_running():
                    return
            else:
                self._open_viewer()
                if self.viewer is None:
                    return

        try:
            url = self.viewer.start_stream()
        except Exception as exc:
            self._show_error("Error al iniciar la transmision", str(exc))
            return

        self.stream_toggle_button.set_label("Detener transmision")
        self.stream_url_label.set_text(f"Escanea con el celular (misma red WiFi):\n{url}")
        self._set_stream_qr(url)
        self._log(f"Transmision al celular activa en: {url}")

    def _on_color_mode_changed(self) -> None:
        self.band_revealer.set_reveal_child(self.color_banded_rb.get_active())
        self.six_bands_revealer.set_reveal_child(self.color_six_bands_rb.get_active())
        self._apply_viewer_settings()

    def _apply_viewer_settings(self) -> None:
        if not hasattr(self, "live_max_distance_entry"):
            return
        try:
            low_m, high_m = self._band_thresholds_m()
            max_distance_text = self.max_distance_entry.get_text().strip()
            max_distance = float(max_distance_text.replace(",", ".")) if max_distance_text else None
            if max_distance is not None and (not np.isfinite(max_distance) or max_distance <= 0):
                raise ValueError("El tope de la escala debe ser mayor que cero.")
            target_thickness_m = self._parse_positive_entry(
                self.target_thickness_entry, "El espesor objetivo"
            ) / 100.0
            live_max_distance, live_cone_angle = self._parse_live_filter_params()
        except ValueError as exc:
            if hasattr(self, "visualization_status_label"):
                self.visualization_status_label.set_markup(
                    f'<span foreground="{COLOR_ERROR}">Ajustes no aplicados: '
                    f"{GLib.markup_escape_text(str(exc))}</span>"
                )
            return

        if self.viewer is None:
            if hasattr(self, "visualization_status_label"):
                self.visualization_status_label.set_text(
                    "Ajustes validos; se aplicaran al abrir la vista 3D."
                )
            return

        if self.color_banded_rb.get_active():
            color_mode = "banded"
        elif self.color_six_bands_rb.get_active():
            color_mode = "six_bands"
        else:
            color_mode = "continuous"
        self.viewer.set_color_mode(color_mode, low_m, high_m, max_distance, target_thickness_m)
        self.viewer.set_show_updated(self.show_updated_check.get_active())
        self.viewer.set_live_filters(
            max_distance_m=live_max_distance,
            cone_angle_deg=live_cone_angle,
            forward_axis=self.live_forward_axis_combo.get_active_text() or "z",
            invert_y=self.live_invert_y_check.get_active(),
            invert_z=self.live_invert_z_check.get_active(),
        )

        if self.source_live_rb.get_active():
            self.viewer.set_live_sensor(self.sensor_connection)
        else:
            self.viewer.set_live_sensor(None)
            self._push_static_result_to_viewer()
        if hasattr(self, "visualization_status_label"):
            self.visualization_status_label.set_text("Ajustes aplicados a la vista 3D.")

    def _push_static_result_to_viewer(self) -> None:
        if self.viewer is None or not self.source_static_rb.get_active():
            return
        try:
            if self.result is not None:
                points = np.asarray(self.result.updated_cloud.points)
            else:
                updated_file = Path(self.updated_path)
                if not updated_file.is_file():
                    return
                points = np.asarray(load_point_cloud(updated_file).points)
        except Exception as exc:
            self._log(f"No se pudo cargar la nube actualizada para la vista 3D: {exc}")
            return
        self.viewer.push_static_points(points)

    def _on_show_updated_changed(self) -> None:
        if self.viewer is not None:
            self.viewer.set_show_updated(self.show_updated_check.get_active())

    def _on_updated_source_changed(self) -> None:
        self._apply_viewer_settings()

    def _on_close(self, *_args) -> None:
        if self._log_poll_source is not None:
            GLib.source_remove(self._log_poll_source)
            self._log_poll_source = None
        self._close_viewer()
        self._close_test_crop_preview_viewer()
        self._finish_capture_progress()
        self._close_imu_pose_viewer()
        if self.sensor_connection is not None:
            try:
                aurora_sensor.disconnect(self.sensor_connection)
            except Exception:
                pass
        if Gtk.main_level() > 0:
            Gtk.main_quit()


def main() -> None:
    settings = Gtk.Settings.get_default()
    if settings is not None:
        settings.set_property("gtk-application-prefer-dark-theme", True)

    style_provider = Gtk.CssProvider()
    style_provider.load_from_data(CSS)
    Gtk.StyleContext.add_provider_for_screen(
        Gdk.Screen.get_default(), style_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
    )

    window = Gtk.Window()
    rgba_visual = Gdk.Screen.get_default().get_rgba_visual()
    if rgba_visual is not None:
        window.set_visual(rgba_visual)
    window.set_title("Aurora — Medicion de espesor de shotcrete")
    app = AuroraGUI(window)
    window.show_all()
    GLib.idle_add(app._toggle_sensor_connection, True)
    Gtk.main()


if __name__ == "__main__":
    main()
