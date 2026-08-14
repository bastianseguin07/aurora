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
import queue
import threading
from pathlib import Path

import gi

gi.require_version("Gdk", "3.0")
gi.require_version("Gtk", "3.0")
from gi.repository import Gdk, GLib, Gtk  # noqa: E402

import numpy as np  # noqa: E402

import aurora_sensor  # noqa: E402
from embedded_viewer import EmbeddedComparisonViewer  # noqa: E402
from live_viewer import LiveViewer  # noqa: E402
from pointcloud_core import (  # noqa: E402
    PipelineParams,
    SIX_BAND_COLORS,
    SIX_BAND_LABELS,
    apply_rigid_transform,
    build_subtle_overlay_cloud,
    compute_c2c_distance,
    compute_rigid_transform,
    crop_cloud_by_quad_box,
    load_point_cloud,
    pick_crop_bounds,
    pick_landmark_points,
    pick_quad_points,
    rigid_transform_rms_error,
    run_pipeline,
    show_point_cloud,
    show_quad_box_preview,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Paleta "Dark Industrial" - estados de sensor/pipeline.
COLOR_OK = "#34C759"
COLOR_ERROR = "#FF3B30"
COLOR_WARN = "#FFCC00"

# Paleta base del tema.
COLOR_BG = "#0F1115"
COLOR_CARD = "#1A1D24"
COLOR_BORDER = "#2E3440"
COLOR_ACCENT = "#FF8C00"
COLOR_ACCENT_DARK = "#CC7000"
COLOR_TEXT = "#F8F9FA"
COLOR_TEXT_MUTED = "#8E95A5"

CSS = f"""
* {{
    font-family: "Inter", "Roboto", "Segoe UI", sans-serif;
}}

window, .background {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
}}

headerbar {{
    background-color: {COLOR_CARD};
    background-image: none;
    color: {COLOR_TEXT};
    border-bottom: 1px solid {COLOR_BORDER};
    box-shadow: none;
    padding: 4px 8px;
}}

headerbar .title {{
    color: {COLOR_TEXT};
    font-weight: 700;
}}

headerbar .subtitle {{
    color: {COLOR_TEXT_MUTED};
}}

stacksidebar {{
    background-color: {COLOR_CARD};
    border-right: 1px solid {COLOR_BORDER};
    font-size: 1.02em;
}}

stacksidebar row {{
    padding: 10px 6px;
    min-height: 30px;
    color: {COLOR_TEXT_MUTED};
}}

stacksidebar row:selected {{
    background-color: rgba(255, 140, 0, 0.16);
    border-left: 3px solid {COLOR_ACCENT};
    color: {COLOR_TEXT};
    font-weight: 600;
}}

stacksidebar row:hover {{
    background-color: rgba(255, 255, 255, 0.04);
}}

frame {{
    margin: 6px 2px;
    background-color: {COLOR_CARD};
    border: 1px solid {COLOR_BORDER};
    border-radius: 6px;
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
    background-color: #22252D;
    background-image: none;
    color: {COLOR_TEXT};
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    padding: 10px 16px;
    min-height: 28px;
    transition: background-color 100ms ease;
}}

button:hover {{
    background-color: #2A2E38;
    border-color: {COLOR_ACCENT};
}}

button:active, button:checked {{
    background-color: #171920;
}}

button:disabled {{
    color: {COLOR_TEXT_MUTED};
    border-color: {COLOR_BORDER};
    background-color: #1A1D24;
}}

button.suggested-action {{
    background-color: {COLOR_ACCENT};
    background-image: none;
    color: #0F1115;
    border: 1px solid {COLOR_ACCENT};
    font-weight: 700;
}}

button.suggested-action:hover {{
    background-color: #FFA033;
}}

button.suggested-action:active {{
    background-color: {COLOR_ACCENT_DARK};
}}

entry {{
    background-color: #14161B;
    background-image: none;
    color: {COLOR_TEXT};
    border: 1px solid {COLOR_BORDER};
    border-radius: 4px;
    padding: 8px 10px;
    min-height: 24px;
    font-family: "Consolas", "DejaVu Sans Mono", monospace;
}}

entry:focus {{
    border-color: {COLOR_ACCENT};
}}

checkbutton, radiobutton {{
    color: {COLOR_TEXT};
    min-height: 30px;
}}

checkbutton check, radiobutton radio {{
    min-width: 20px;
    min-height: 20px;
    border: 1px solid {COLOR_BORDER};
    background-color: #14161B;
}}

checkbutton check:checked, radiobutton radio:checked {{
    background-color: {COLOR_ACCENT};
    border-color: {COLOR_ACCENT};
}}

expander title {{
    color: {COLOR_TEXT};
    font-weight: 600;
}}

textview, textview text {{
    background-color: #0B0C0F;
    color: {COLOR_TEXT};
    font-family: "Consolas", "DejaVu Sans Mono", monospace;
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
    border: 1px solid {COLOR_BORDER};
    color: {COLOR_TEXT};
}}

popover contents {{
    background-color: {COLOR_CARD};
    color: {COLOR_TEXT};
}}

.kpi-card {{
    background-color: #14161B;
    border: 1px solid {COLOR_BORDER};
    border-radius: 6px;
    padding: 12px 18px;
    min-width: 120px;
}}

.kpi-value {{
    font-family: "Consolas", "DejaVu Sans Mono", monospace;
}}

.status-bar {{
    background-color: #14161B;
    border-top: 1px solid {COLOR_BORDER};
}}
""".encode("utf-8")


class AuroraGUI:
    def __init__(self, window: Gtk.Window) -> None:
        self.window = window
        self.window.set_default_size(1050, 820)
        self.window.set_size_request(880, 660)
        self.window.connect("destroy", self._on_close)

        self.base_path = str(PROJECT_ROOT / "data" / "base.ply")
        self.updated_path = str(PROJECT_ROOT / "data" / "updated.ply")
        self.output_dir = str(PROJECT_ROOT / "output")

        self.sensor_connection = None
        self.capture_stop_event: threading.Event | None = None
        self.last_base_capture_path: str | None = None
        self.last_updated_capture_path: str | None = None
        self.result = None
        self.alignment_applied = False
        self.viewer: LiveViewer | None = None
        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.worker_thread: threading.Thread | None = None

        self._build_layout()
        GLib.timeout_add(150, self._poll_log_queue)

    # ------------------------------------------------------------------ UI

    def _build_layout(self) -> None:
        header_bar = Gtk.HeaderBar()
        header_bar.set_show_close_button(True)
        header_bar.set_title("Aurora")
        header_bar.set_subtitle("Medicion de espesor de shotcrete")
        self.window.set_titlebar(header_bar)

        self.run_button = Gtk.Button(label="▶  Calcular espesor")
        self.run_button.set_tooltip_text(
            "Compara las dos nubes de puntos y calcula el espesor de shotcrete. "
            "El resultado se muestra automaticamente en 3D."
        )
        self.run_button.get_style_context().add_class("suggested-action")
        self.run_button.connect("clicked", lambda _b: self._run_pipeline_clicked())
        header_bar.pack_start(self.run_button)

        self.generate_report_button = Gtk.Button(label="Generar informe")
        self.generate_report_button.set_tooltip_text(
            "Exporta un informe en Markdown con las estadisticas y el estado (dentro/fuera de "
            "los umbrales configurados en Visualizacion 3D)."
        )
        self.generate_report_button.set_sensitive(False)
        self.generate_report_button.connect("clicked", lambda _b: self._generate_report())
        header_bar.pack_start(self.generate_report_button)

        root_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.window.add(root_box)

        content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        content_box.set_vexpand(True)
        root_box.pack_start(content_box, True, True, 0)

        self.stack = Gtk.Stack()
        self.stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self.stack.set_transition_duration(150)

        sidebar = Gtk.StackSidebar()
        sidebar.set_stack(self.stack)
        content_box.pack_start(sidebar, False, False, 0)
        content_box.pack_start(self.stack, True, True, 0)

        self.stack.add_titled(self._build_capture_page(), "captura", "Captura")
        self.stack.add_titled(self._build_comparison_page(), "comparacion", "Comparacion")
        self.stack.add_titled(self._build_alignment_page(), "alineacion", "Alineacion")
        self.stack.add_titled(self._build_segmentation_page(), "segmentacion", "Segmentacion")
        self.stack.add_titled(self._build_processing_page(), "procesamiento", "Ajustes de analisis")
        self.stack.add_titled(self._build_visualization_page(), "visualizacion", "Visualizacion 3D")
        self.stack.add_titled(self._build_embedded_test_page(), "prueba_embebida", "Comparacion (prueba)")

        result_frame, result_box = self._section("Resultado")
        result_frame.get_style_context().add_class("status-bar")

        status_row = self._row(result_box)
        self.status_spinner = Gtk.Spinner()
        self.status_spinner.set_no_show_all(True)
        self.status_spinner.set_visible(False)
        status_row.pack_start(self.status_spinner, False, False, 0)
        self.status_label = Gtk.Label(label="Todavia no se calculo ningun resultado.")
        self.status_label.set_xalign(0)
        status_row.pack_start(self.status_label, False, False, 0)

        self.cards_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=24)
        self.cards_row.set_halign(Gtk.Align.CENTER)
        self.cards_row.set_margin_top(8)
        self.cards_row.set_margin_bottom(4)
        self.cards_row.set_no_show_all(True)
        card, self.stat_points = self._stat_card("Puntos analizados")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_mean = self._stat_card("Espesor medio")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_median = self._stat_card("Espesor mediano")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_min = self._stat_card("Minimo")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_max = self._stat_card("Maximo")
        self.cards_row.pack_start(card, False, False, 0)
        card, self.stat_p95 = self._stat_card("Percentil 95")
        self.cards_row.pack_start(card, False, False, 0)
        result_box.pack_start(self.cards_row, False, False, 0)

        log_expander = Gtk.Expander(label="Detalles tecnicos")
        log_expander.set_expanded(False)
        log_scroller = Gtk.ScrolledWindow()
        log_scroller.set_min_content_height(130)
        log_scroller.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.log_view = Gtk.TextView()
        self.log_view.set_editable(False)
        self.log_view.set_monospace(True)
        self.log_view.set_left_margin(6)
        self.log_view.set_top_margin(6)
        self.log_buffer = self.log_view.get_buffer()
        log_scroller.add(self.log_view)
        log_expander.add(log_scroller)
        result_box.pack_start(log_expander, False, True, 6)

        root_box.pack_start(result_frame, False, True, 8)

    # -- Pagina: Captura ---------------------------------------------------------

    def _build_capture_page(self) -> Gtk.Widget:
        page = self._new_page()

        sensor_frame, sensor_box = self._section("Sensor Aurora")

        row = self._row(sensor_box)
        row.pack_start(Gtk.Label(label="Direccion del sensor:"), False, False, 0)
        self.sensor_address_entry = Gtk.Entry()
        self.sensor_address_entry.set_text("192.168.11.1")
        self.sensor_address_entry.set_width_chars(16)
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
        row.pack_start(
            self._info_button(
                "Con el sensor conectado, acumula frames del tunel durante la duracion "
                "configurada abajo y guarda el resultado automaticamente como la nube "
                "'original' o 'con shotcrete' segun el boton elegido."
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

        row = self._row(sensor_box)
        self.capture_status_label = Gtk.Label(xalign=0)
        row.pack_start(self.capture_status_label, False, False, 0)

        row = self._row(sensor_box)
        row.pack_start(Gtk.Label(label="Duracion de la captura (s):"), False, False, 0)
        self.capture_duration_entry = Gtk.Entry()
        self.capture_duration_entry.set_text("15")
        self.capture_duration_entry.set_width_chars(6)
        row.pack_start(self.capture_duration_entry, False, False, 0)
        row.pack_start(Gtk.Label(label="Persistencia minima de puntos (0-1):"), False, False, 0)
        self.capture_persistence_entry = Gtk.Entry()
        self.capture_persistence_entry.set_text("0")
        self.capture_persistence_entry.set_width_chars(6)
        row.pack_start(self.capture_persistence_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "Duracion: cuanto tiempo se acumulan frames del sensor para armar la "
                "captura (por defecto 15 s).\n\n"
                "Persistencia minima: fraccion de esos frames en la que un punto debe "
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

        row = self._row(sensor_box)
        self.capture_limit_fov_check = Gtk.CheckButton(label="Limitar campo de vision")
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
        self.capture_max_distance_entry.set_text("")
        self.capture_max_distance_entry.set_width_chars(8)
        row.pack_start(self.capture_max_distance_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "Opcional, desactivado por defecto (captura todo el campo de vision del "
                "sensor). Al activarlo, descarta puntos fuera de un cono centrado en el "
                "eje frontal indicado — util para capturar solo una caja de prueba sin "
                "incluir lo que esta detras o a los costados.\n\n"
                "Distancia maxima: vacio = sin limite. Descarta puntos mas lejos que ese "
                "valor, aplique o no el cono."
            ),
            False,
            False,
            0,
        )

        row = self._row(sensor_box)
        self.start_live_capture_button = Gtk.Button(label="Iniciar captura en tiempo real")
        self.start_live_capture_button.set_sensitive(False)
        self.start_live_capture_button.connect("clicked", lambda _b: self._start_live_capture_clicked())
        row.pack_start(self.start_live_capture_button, False, False, 0)
        self.capture_live_baseline_button = Gtk.Button(label="Fijar BASE en vivo (MVP)")
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

        page.pack_start(sensor_frame, False, True, 0)
        return self._scrolled(page)

    # -- Pagina: Comparacion ------------------------------------------------------

    def _build_comparison_page(self) -> Gtk.Widget:
        page = self._new_page()

        base_frame, base_box = self._section("Tunel original (antes del shotcrete)")
        self.base_path_label = self._file_row(
            base_box,
            self.base_path,
            lambda p: self._set_base_path(p),
            info_text="La nube de puntos del tunel ANTES de aplicar el shotcrete. "
            "Puede ser un archivo ya guardado o una captura hecha en la pestaña 'Captura'.",
        )
        page.pack_start(base_frame, False, True, 0)

        updated_frame, updated_box = self._section("Tunel con shotcrete (despues)")
        self.updated_path_label = self._file_row(
            updated_box,
            self.updated_path,
            lambda p: self._set_updated_path(p),
            info_text="La nube de puntos del mismo tunel DESPUES de aplicar el shotcrete. "
            "Puede ser un archivo ya guardado o una captura hecha en la pestaña 'Captura'.",
        )
        page.pack_start(updated_frame, False, True, 0)

        return self._scrolled(page)

    def _set_base_path(self, path: str) -> None:
        self.base_path = path
        self.alignment_applied = False

    def _set_updated_path(self, path: str) -> None:
        self.updated_path = path
        self.alignment_applied = False

    # -- Pagina: Alineacion (Procrustes por puntos de referencia) ---------------

    def _build_alignment_page(self) -> Gtk.Widget:
        page = self._new_page()

        self.landmarks_base: list | None = None
        self.landmarks_updated: list | None = None
        self.alignment_rotation: np.ndarray | None = None
        self.alignment_translation: np.ndarray | None = None
        self._raw_updated_path: str | None = None

        info_frame, info_box = self._section("Alinear con puntos de referencia (opcional)")
        note = Gtk.Label(
            label=(
                "Si las dos capturas no comparten exactamente la misma posicion (el sensor "
                "se reubico entre una y otra), puedes alinearlas eligiendo 3 o mas puntos "
                "fijos que NO se muevan entre capturas — por ejemplo cabezas de pernos de "
                "anclaje, marcas o esquinas rigidas. A diferencia de ICP (que ajusta toda "
                "la superficie y puede confundir el espesor real con error de alineacion), "
                "esto usa solo esos puntos fijos como referencia."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        info_box.pack_start(note, False, False, 0)
        page.pack_start(info_frame, False, True, 0)

        step_frame, step_box = self._section("Paso 1: elegir los mismos puntos en ambas nubes")

        row = self._row(step_box)
        pick_base_button = Gtk.Button(label="1. Elegir puntos en el tunel original...")
        pick_base_button.set_tooltip_text(
            "Shift+Click sobre cada punto de referencia, en orden, despues cerrar la ventana."
        )
        pick_base_button.connect("clicked", lambda _b: self._pick_alignment_points("base"))
        row.pack_start(pick_base_button, False, False, 0)
        self.landmarks_base_label = Gtk.Label(label="(ninguno)")
        row.pack_start(self.landmarks_base_label, False, False, 0)

        row = self._row(step_box)
        pick_updated_button = Gtk.Button(label="2. Elegir los MISMOS puntos en el tunel con shotcrete...")
        pick_updated_button.set_tooltip_text(
            "Elige los puntos en el MISMO orden que en el paso 1, sobre los mismos puntos fisicos."
        )
        pick_updated_button.connect("clicked", lambda _b: self._pick_alignment_points("updated"))
        row.pack_start(pick_updated_button, False, False, 0)
        self.landmarks_updated_label = Gtk.Label(label="(ninguno)")
        row.pack_start(self.landmarks_updated_label, False, False, 0)

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

    def _pick_alignment_points(self, which: str) -> None:
        path = Path(self.base_path if which == "base" else self.updated_path)
        if not path.exists():
            self._show_error("Error", f"No se encontro el archivo:\n{path}")
            return
        try:
            cloud = load_point_cloud(path)
            title = (
                "Tunel original - Shift+Click en cada punto de referencia, en orden, luego cerrar (Q)"
                if which == "base"
                else "Tunel con shotcrete - Elige los MISMOS puntos, en el mismo orden, luego cerrar (Q)"
            )
            points = pick_landmark_points(cloud, title)
        except Exception as exc:
            self._show_error("Error al abrir el visor 3D", str(exc))
            return

        if points is None:
            self._show_info("Sin seleccion", "Elige al menos 3 puntos (Shift+Click) antes de cerrar la ventana.")
            return

        if which == "base":
            self.landmarks_base = points
            self.landmarks_base_label.set_text(f"{len(points)} puntos elegidos ✓")
        else:
            self.landmarks_updated = points
            self.landmarks_updated_label.set_text(f"{len(points)} puntos elegidos ✓")

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

        # Se guardan ya calculadas (y la ruta cruda, previa a esta alineacion) para que
        # la pestaña "Segmentacion" pueda ubicar un box en la nube con shotcrete sin
        # tener que transformar la nube completa.
        self.alignment_rotation = rotation
        self.alignment_translation = translation
        self._raw_updated_path = self.updated_path

        self.apply_alignment_button.set_sensitive(False)
        self.alignment_result_label.set_text("Aplicando...")

        def worker():
            try:
                updated_file = Path(self.updated_path)
                cloud = load_point_cloud(updated_file)
                aligned = apply_rigid_transform(cloud, rotation, translation)
                aligned_path = updated_file.with_name(updated_file.stem + "_alineado.ply")
                import open3d as o3d

                o3d.io.write_point_cloud(str(aligned_path), aligned)
            except Exception as exc:
                self._ui(self._on_alignment_failed, str(exc))
                return
            self._ui(self._on_alignment_done, str(aligned_path), rms)

        threading.Thread(target=worker, daemon=True).start()

    def _on_alignment_done(self, aligned_path: str, rms: float) -> None:
        self.apply_alignment_button.set_sensitive(True)
        self.alignment_result_label.set_text(f"Error residual: {rms * 1000:.2f} mm — nube alineada guardada")
        self.alignment_applied = True
        self.updated_path = aligned_path
        self._set_path_label(self.updated_path_label, aligned_path)
        self._show_info(
            "Alineacion aplicada",
            f"Error residual en los puntos de referencia: {rms * 1000:.2f} mm.\n\n"
            f"Se guardo la nube alineada en:\n{aligned_path}\n\n"
            "La pestaña 'Comparacion' ya usa este archivo como nube con shotcrete.",
        )

    def _on_alignment_failed(self, message: str) -> None:
        self.apply_alignment_button.set_sensitive(True)
        self.alignment_result_label.set_text("Error al aplicar la alineacion")
        self._show_error("Error", message)

    # -- Pagina: Segmentacion -----------------------------------------------

    def _build_segmentation_page(self) -> Gtk.Widget:
        page = self._new_page()

        self.segmentation_quad: np.ndarray | None = None
        self._segmentation_source_base_path: str | None = None
        self._segmentation_source_updated_path: str | None = None

        info_frame, info_box = self._section("Recortar una region antes de analizar (opcional)")
        note = Gtk.Label(
            label=(
                "Elegi 4 puntos (Shift+Click), en orden alrededor del perimetro, sobre la "
                "nube CON shotcrete (la capturada despues), para marcar una region "
                "cuadrada/rectangular (por ejemplo, un tramo especifico del tunel o una de "
                "tus cajas de prueba). La app arma un 'box' 3D extruyendo esa region a lo "
                "largo de su normal (el ancho del box es configurable abajo) y recorta las "
                "DOS nubes (con shotcrete y original) a los puntos que caen dentro de ese "
                "box, generando dos archivos .ply nuevos que el resto del analisis va a usar "
                "en vez de las nubes completas.\n\n"
                "La misma region (mismas coordenadas) se aplica tal cual a la nube original. "
                "Esto asume que ambas nubes comparten el mismo sistema de coordenadas: si el "
                "sensor se reubico entre una captura y otra, calcula la alineacion primero "
                "(pestaña 'Alineacion') antes de segmentar; si no se movio, no hace falta."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        info_box.pack_start(note, False, False, 0)
        page.pack_start(info_frame, False, True, 0)

        step_frame, step_box = self._section("Paso 1: elegir la region y el ancho del box")
        row = self._row(step_box)
        pick_button = Gtk.Button(label="Elegir 4 puntos en el tunel con shotcrete...")
        pick_button.set_tooltip_text(
            "Shift+Click en las 4 esquinas de la region, en orden alrededor del perimetro, "
            "despues cerrar la ventana."
        )
        pick_button.connect("clicked", lambda _b: self._pick_segmentation_quad())
        row.pack_start(pick_button, False, False, 0)
        self.segmentation_quad_label = Gtk.Label(label="(ninguno)")
        row.pack_start(self.segmentation_quad_label, False, False, 0)

        row = self._row(step_box)
        row.pack_start(Gtk.Label(label="Ancho del box (cm):"), False, False, 0)
        width_adjustment = Gtk.Adjustment(value=10.0, lower=1.0, upper=100.0, step_increment=1.0, page_increment=5.0)
        self.segmentation_width_spin = Gtk.SpinButton(adjustment=width_adjustment, climb_rate=1.0, digits=1)
        row.pack_start(self.segmentation_width_spin, False, False, 0)
        row.pack_start(
            self._info_button(
                "Profundidad del box a lo largo de la normal de la region elegida, centrada "
                "en ese plano (+/- la mitad del ancho para cada lado). Tiene que ser mayor "
                "que el espesor de shotcrete esperado, para no cortar la superficie con "
                "shotcrete que quedo mas cerca del sensor."
            ),
            False, False, 0,
        )
        page.pack_start(step_frame, False, True, 0)

        apply_frame, apply_box = self._section("Paso 2: aplicar")
        row = self._row(apply_box)
        self.apply_segmentation_button = Gtk.Button(label="Aplicar segmentacion")
        self.apply_segmentation_button.connect("clicked", lambda _b: self._apply_segmentation())
        row.pack_start(self.apply_segmentation_button, False, False, 0)
        clear_button = Gtk.Button(label="Quitar segmentacion (usar nube completa)")
        clear_button.connect("clicked", lambda _b: self._clear_segmentation())
        row.pack_start(clear_button, False, False, 0)

        row = self._row(apply_box)
        self.view_segmented_base_button = Gtk.Button(label="Ver nube segmentada: antes")
        self.view_segmented_base_button.set_sensitive(False)
        self.view_segmented_base_button.connect("clicked", lambda _b: self._view_segmented_cloud("base"))
        row.pack_start(self.view_segmented_base_button, False, False, 0)
        self.view_segmented_updated_button = Gtk.Button(label="Ver nube segmentada: despues")
        self.view_segmented_updated_button.set_sensitive(False)
        self.view_segmented_updated_button.connect("clicked", lambda _b: self._view_segmented_cloud("updated"))
        row.pack_start(self.view_segmented_updated_button, False, False, 0)

        self.segmentation_result_label = Gtk.Label(xalign=0)
        self.segmentation_result_label.set_line_wrap(True)
        apply_box.pack_start(self.segmentation_result_label, False, False, 0)
        self.segmentation_paths_label = Gtk.Label(xalign=0)
        self.segmentation_paths_label.set_line_wrap(True)
        self.segmentation_paths_label.set_selectable(True)
        apply_box.pack_start(self.segmentation_paths_label, False, False, 0)
        page.pack_start(apply_frame, False, True, 0)

        return self._scrolled(page)

    def _pick_segmentation_quad(self) -> None:
        path = Path(self.updated_path)
        if not path.exists():
            self._show_error("Error", f"No se encontro el archivo:\n{path}")
            return
        try:
            cloud = load_point_cloud(path)
            points = pick_quad_points(
                cloud,
                "Tunel con shotcrete - Shift+Click en las 4 esquinas de la region, en orden, luego cerrar (Q)",
            )
        except Exception as exc:
            self._show_error("Error al abrir el visor 3D", str(exc))
            return

        if points is None:
            self._show_info(
                "Seleccion invalida",
                "Elegi exactamente 4 puntos (Shift+Click), en orden alrededor del perimetro, "
                "antes de cerrar la ventana.",
            )
            return

        self.segmentation_quad = points
        self.segmentation_quad_label.set_text("4 puntos elegidos ✓")

        width_m = self.segmentation_width_spin.get_value() / 100.0
        try:
            show_quad_box_preview(cloud, points, width_m, window_name="Aurora - Region elegida (despues, con shotcrete)")
        except Exception as exc:
            self._show_error("Error al mostrar la previsualizacion", str(exc))
            return

        try:
            base_cloud = load_point_cloud(Path(self.base_path))
            show_quad_box_preview(
                base_cloud, points, width_m, window_name="Aurora - Misma region aplicada a la nube original (antes)"
            )
        except Exception as exc:
            self._show_error("Error al mostrar la previsualizacion en la nube original", str(exc))

    def _apply_segmentation(self) -> None:
        if self.segmentation_quad is None:
            self._show_warning("Falta la region", "Elegi los 4 puntos de la region primero.")
            return

        if self._segmentation_source_base_path is None:
            self._segmentation_source_base_path = self.base_path
            self._segmentation_source_updated_path = self.updated_path

        width_m = self.segmentation_width_spin.get_value() / 100.0
        quad = self.segmentation_quad
        base_source = Path(self._segmentation_source_base_path)
        updated_source = Path(self._segmentation_source_updated_path)

        self.apply_segmentation_button.set_sensitive(False)
        self.segmentation_result_label.set_text("Recortando...")

        def worker():
            try:
                # La region se eligio sobre la nube con shotcrete; se aplica tal cual
                # (mismas coordenadas) a la nube original, ambas en el mismo sistema
                # de referencia (si el sensor no se movio, o si ya se aplico la
                # alineacion previamente).
                updated_cloud = load_point_cloud(updated_source)
                updated_cropped = crop_cloud_by_quad_box(updated_cloud, quad, width_m)

                base_cloud = load_point_cloud(base_source)
                base_cropped = crop_cloud_by_quad_box(base_cloud, quad, width_m)

                if len(base_cropped.points) == 0 or len(updated_cropped.points) == 0:
                    raise RuntimeError(
                        "El box elegido no contiene puntos en una de las dos nubes. "
                        "Proba con otra region o con un ancho de box distinto."
                    )

                base_out = base_source.with_name(base_source.stem + "_segmento.ply")
                updated_out = updated_source.with_name(updated_source.stem + "_segmento.ply")
                import open3d as o3d

                o3d.io.write_point_cloud(str(base_out), base_cropped)
                o3d.io.write_point_cloud(str(updated_out), updated_cropped)
            except Exception as exc:
                self._ui(self._on_segmentation_failed, str(exc))
                return
            self._ui(
                self._on_segmentation_done,
                str(base_out),
                str(updated_out),
                len(base_cropped.points),
                len(updated_cropped.points),
            )

        threading.Thread(target=worker, daemon=True).start()

    def _on_segmentation_done(self, base_out: str, updated_out: str, n_base: int, n_updated: int) -> None:
        self.apply_segmentation_button.set_sensitive(True)
        self.segmentation_result_label.set_text(f"Recorte: {n_base} / {n_updated} puntos (original/shotcrete)")
        self.segmentation_paths_label.set_text(
            f"Antes: {Path(base_out).name}\n  {base_out}\n"
            f"Despues: {Path(updated_out).name}\n  {updated_out}"
        )
        self.base_path = base_out
        self.updated_path = updated_out
        self._set_path_label(self.base_path_label, base_out)
        self._set_path_label(self.updated_path_label, updated_out)
        self.view_segmented_base_button.set_sensitive(True)
        self.view_segmented_updated_button.set_sensitive(True)
        self._show_info(
            "Segmentacion aplicada",
            f"Nube original recortada: {n_base} puntos.\nNube con shotcrete recortada: {n_updated} puntos.\n\n"
            f"Guardadas en:\n{base_out}\n{updated_out}\n\n"
            "El resto del analisis (Ajustes, Visualizacion, Reporte) va a usar estas nubes recortadas.",
        )

    def _on_segmentation_failed(self, message: str) -> None:
        self.apply_segmentation_button.set_sensitive(True)
        self.segmentation_result_label.set_text("Error al recortar")
        self._show_error("Error", message)

    def _clear_segmentation(self) -> None:
        if self._segmentation_source_base_path is None:
            self._show_info("Nada que quitar", "Todavia no se aplico ninguna segmentacion.")
            return
        self.base_path = self._segmentation_source_base_path
        self.updated_path = self._segmentation_source_updated_path
        self._set_path_label(self.base_path_label, self.base_path)
        self._set_path_label(self.updated_path_label, self.updated_path)
        self.segmentation_result_label.set_text("Segmentacion quitada, usando la nube completa")
        self.segmentation_paths_label.set_text("")
        self.view_segmented_base_button.set_sensitive(False)
        self.view_segmented_updated_button.set_sensitive(False)
        self._segmentation_source_base_path = None
        self._segmentation_source_updated_path = None

    def _view_segmented_cloud(self, target: str) -> None:
        path = self.base_path if target == "base" else self.updated_path
        label = "Antes (segmentado)" if target == "base" else "Despues (segmentado)"
        try:
            cloud = load_point_cloud(Path(path))
        except Exception as exc:
            self._show_error("Error al abrir la nube segmentada", str(exc))
            return
        show_point_cloud(cloud, window_name=f"Aurora - {label}")

    # -- Pagina: Ajustes de analisis --------------------------------------------

    def _build_processing_page(self) -> Gtk.Widget:
        page = self._new_page()

        crop_frame, crop_box = self._section("Analizar solo una zona (opcional)")
        row = self._row(crop_box)
        self.use_crop_check = Gtk.CheckButton(label="Analizar solo la zona seleccionada")
        row.pack_start(self.use_crop_check, False, False, 0)
        row.pack_start(
            self._info_button(
                "Por defecto se analiza toda la nube. Marca esta opcion para enfocar el "
                "analisis en una sola zona (por ejemplo, un tramo especifico del tunel o "
                "un objeto de prueba), en vez de que el resultado se mezcle con el resto "
                "de la escena."
            ),
            False,
            False,
            0,
        )
        row = self._row(crop_box)
        pick_button = Gtk.Button(label="Seleccionar zona en el visor 3D...")
        pick_button.set_tooltip_text(
            "Abre la nube original en una ventana 3D. Shift+Click sobre 2 o mas puntos "
            "para marcar la zona, despues cerrar la ventana."
        )
        pick_button.connect("clicked", lambda _b: self._pick_crop_interactively())
        row.pack_start(pick_button, False, False, 0)
        self.crop_status_label = Gtk.Label(label="(ninguna zona seleccionada)")
        row.pack_start(self.crop_status_label, False, False, 0)
        page.pack_start(crop_frame, False, True, 0)

        advanced_frame, advanced_box = self._build_advanced_expander_page2()
        page.pack_start(advanced_frame, False, True, 0)

        return self._scrolled(page)

    def _build_advanced_expander_page2(self) -> tuple[Gtk.Widget, Gtk.Box]:
        expander = Gtk.Expander(label="Opciones avanzadas")
        expander.set_expanded(False)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        box.set_margin_start(4)
        box.set_margin_top(10)
        box.set_margin_bottom(4)
        expander.add(box)

        row = self._row(box)
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

        row = self._row(box)
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
        row.pack_start(Gtk.Label(label="Zona - esquina minima (x y z):"), False, False, 0)
        self.crop_min_entry = Gtk.Entry()
        self.crop_min_entry.set_width_chars(22)
        row.pack_start(self.crop_min_entry, False, False, 0)
        row.pack_start(Gtk.Label(label="maxima:"), False, False, 0)
        self.crop_max_entry = Gtk.Entry()
        self.crop_max_entry.set_width_chars(22)
        row.pack_start(self.crop_max_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "Coordenadas manuales de la zona a analizar (alternativa a elegirla en el "
                "visor 3D)."
            ),
            False,
            False,
            0,
        )

        row = self._row(box)
        row.pack_start(Gtk.Label(label="Margen extra al seleccionar en 3D (m):"), False, False, 0)
        self.crop_margin_entry = Gtk.Entry()
        self.crop_margin_entry.set_text("0.08")
        self.crop_margin_entry.set_width_chars(8)
        row.pack_start(self.crop_margin_entry, False, False, 0)
        row.pack_start(
            self._info_button(
                "Debe ser mayor al desplazamiento esperado entre ambas capturas, o la zona "
                "elegida puede quedar vacia en la nube con shotcrete."
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

    # -- Pagina: Visualizacion 3D --------------------------------------------------

    def _build_visualization_page(self) -> Gtk.Widget:
        page = self._new_page()

        color_frame, color_box = self._section("Color del espesor")

        row = self._row(color_box)
        self.color_continuous_rb = Gtk.RadioButton.new_with_label_from_widget(None, "Escala continua")
        self.color_continuous_rb.connect("toggled", lambda _b: self._on_color_mode_changed())
        row.pack_start(self.color_continuous_rb, False, False, 0)
        self.color_banded_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.color_continuous_rb, "3 niveles de color"
        )
        self.color_banded_rb.connect("toggled", lambda _b: self._on_color_mode_changed())
        row.pack_start(self.color_banded_rb, False, False, 0)
        self.color_six_bands_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.color_continuous_rb, "6 niveles (espesor objetivo)"
        )
        self.color_six_bands_rb.connect("toggled", lambda _b: self._on_color_mode_changed())
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
        self.band_revealer.set_transition_type(Gtk.RevealerTransitionType.SLIDE_DOWN)
        band_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        band_row.pack_start(Gtk.Label(label="Hasta este espesor = verde (mm):"), False, False, 0)
        self.band_low_entry = Gtk.Entry()
        self.band_low_entry.set_text("20")
        self.band_low_entry.set_width_chars(8)
        band_row.pack_start(self.band_low_entry, False, False, 0)
        band_row.pack_start(Gtk.Label(label="Desde este espesor = rojo (mm):"), False, False, 0)
        self.band_high_entry = Gtk.Entry()
        self.band_high_entry.set_text("50")
        self.band_high_entry.set_width_chars(8)
        band_row.pack_start(self.band_high_entry, False, False, 0)
        self.band_revealer.add(band_row)
        color_box.pack_start(self.band_revealer, False, False, 0)

        self.six_bands_revealer = Gtk.Revealer()
        self.six_bands_revealer.set_transition_type(Gtk.RevealerTransitionType.SLIDE_DOWN)
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

        advanced_frame, advanced_box = Gtk.Expander(label="Opciones avanzadas"), None
        advanced_frame.set_expanded(False)
        adv_inner = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
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

        live_frame, live_box = self._section("Ajustes de captura en vivo (MVP)")
        row = self._row(live_box)
        row.pack_start(Gtk.Label(label="Distancia maxima (m):"), False, False, 0)
        self.live_max_distance_entry = Gtk.Entry()
        self.live_max_distance_entry.set_text("5.0")
        self.live_max_distance_entry.set_width_chars(8)
        row.pack_start(self.live_max_distance_entry, False, False, 0)
        row.pack_start(Gtk.Label(label="Cono (grados):"), False, False, 0)
        self.live_cone_angle_entry = Gtk.Entry()
        self.live_cone_angle_entry.set_text("90")
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

        page.pack_start(live_frame, False, True, 0)
        return self._scrolled(page)

    # -- Pagina: Comparacion (prueba / experimental) ----------------------------

    def _build_embedded_test_page(self) -> Gtk.Widget:
        page = self._new_page()

        note_frame, note_box = self._section("Seccion experimental")
        note = Gtk.Label(
            label=(
                "Prueba: visor 3D embebido directamente en la ventana (a diferencia de la "
                "pestaña 'Visualizacion 3D', que abre una ventana aparte). Muestra ambas "
                "nubes superpuestas, con un resaltado sutil donde difieren. Todavia no esta "
                "decidido si esto queda en la version final."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        note_box.pack_start(note, False, False, 0)

        row = self._row(note_box)
        load_button = Gtk.Button(label="Cargar comparacion aqui")
        load_button.set_tooltip_text("Usa los archivos elegidos en la pestaña 'Comparacion'.")
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
        if not base_file.exists() or not updated_file.exists():
            self._show_error(
                "Error", "Elige ambos archivos en la pestaña 'Comparacion' antes de cargar la vista."
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

    # -- Helpers de layout -----------------------------------------------------

    def _new_page(self) -> Gtk.Box:
        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        page.set_margin_start(12)
        page.set_margin_end(12)
        page.set_margin_top(10)
        page.set_margin_bottom(10)
        return page

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

    def _set_stat(self, label: Gtk.Label, text: str) -> None:
        label.set_markup(
            f"<span size='xx-large' weight='bold' foreground='{COLOR_ACCENT}'>"
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
        label.set_text(name if name else "Sin seleccionar")
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
        dialog = Gtk.FileChooserDialog(
            title="Seleccionar archivo .ply", parent=self.window, action=Gtk.FileChooserAction.OPEN
        )
        dialog.add_buttons("_Cancelar", Gtk.ResponseType.CANCEL, "_Abrir", Gtk.ResponseType.OK)
        filter_ply = Gtk.FileFilter()
        filter_ply.set_name("Archivos PLY")
        filter_ply.add_pattern("*.ply")
        dialog.add_filter(filter_ply)
        path = None
        if dialog.run() == Gtk.ResponseType.OK:
            path = dialog.get_filename()
        dialog.destroy()
        return path

    def _browse_output_dir(self) -> None:
        dialog = Gtk.FileChooserDialog(
            title="Seleccionar carpeta de salida", parent=self.window, action=Gtk.FileChooserAction.SELECT_FOLDER
        )
        dialog.add_buttons("_Cancelar", Gtk.ResponseType.CANCEL, "_Seleccionar", Gtk.ResponseType.OK)
        if dialog.run() == Gtk.ResponseType.OK:
            self.output_dir = dialog.get_filename()
            self._set_path_label(self.output_dir_label, self.output_dir)
        dialog.destroy()

    def _save_file_dialog(self, title: str, initial_dir: Path, initial_name: str) -> str | None:
        dialog = Gtk.FileChooserDialog(title=title, parent=self.window, action=Gtk.FileChooserAction.SAVE)
        dialog.add_buttons("_Cancelar", Gtk.ResponseType.CANCEL, "_Guardar", Gtk.ResponseType.OK)
        dialog.set_do_overwrite_confirmation(True)
        dialog.set_current_folder(str(initial_dir))
        dialog.set_current_name(initial_name)
        path = None
        if dialog.run() == Gtk.ResponseType.OK:
            path = dialog.get_filename()
        dialog.destroy()
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

    def _poll_log_queue(self) -> bool:
        try:
            while True:
                message = self.log_queue.get_nowait()
                end_iter = self.log_buffer.get_end_iter()
                self.log_buffer.insert(end_iter, message + "\n")
                self.log_view.scroll_to_iter(self.log_buffer.get_end_iter(), 0.0, False, 0, 0)
        except queue.Empty:
            pass
        return True

    def _parse_xyz(self, text: str, field_name: str) -> tuple[float, float, float]:
        parts = text.replace(",", " ").split()
        if len(parts) != 3:
            raise ValueError(f"{field_name} debe tener 3 numeros (x y z), separados por espacio o coma.")
        return tuple(float(p) for p in parts)

    # ------------------------------------------------------------ Sensor

    def _toggle_sensor_connection(self) -> None:
        if self.sensor_connection is not None:
            self._disconnect_sensor()
            return

        address = self.sensor_address_entry.get_text().strip()
        self.connect_button.set_sensitive(False)
        self.sensor_status_label.set_markup(f'<span foreground="{COLOR_WARN}">●</span>  Conectando...')

        def worker():
            try:
                connection = aurora_sensor.connect(address)
                self._ui(self._on_sensor_connected, connection)
            except Exception as exc:
                self._ui(self._on_sensor_connect_failed, exc)

        threading.Thread(target=worker, daemon=True).start()

    def _on_sensor_connected(self, connection) -> None:
        self.sensor_connection = connection
        self.connect_button.set_label("Desconectar")
        self.connect_button.set_sensitive(True)
        self.sensor_status_label.set_markup(f'<span foreground="{COLOR_OK}">●</span>  Conectado')
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

    def _on_sensor_connect_failed(self, exc: Exception) -> None:
        self.connect_button.set_sensitive(True)
        self.sensor_status_label.set_markup(f'<span foreground="{COLOR_ERROR}">●</span>  Desconectado')
        self._show_error("Error de conexion", str(exc))

    def _disconnect_sensor(self) -> None:
        if self.viewer is not None:
            self.viewer.set_live_sensor(None)
        try:
            aurora_sensor.disconnect(self.sensor_connection)
        except Exception as exc:
            self._log(f"Aviso al desconectar: {exc}")
        self.sensor_connection = None
        self.connect_button.set_label("Conectar")
        self.sensor_status_label.set_markup(f'<span foreground="{COLOR_ERROR}">●</span>  Desconectado')
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
            duration_s = float(self.capture_duration_entry.get_text() or 15.0)
            persistence_ratio = float(self.capture_persistence_entry.get_text() or 0.0)
        except ValueError:
            self._show_error("Parametros invalidos", "Duracion y persistencia deben ser numeros.")
            return

        cone_angle_deg = None
        max_distance_m = None
        try:
            if self.capture_limit_fov_check.get_active():
                cone_text = self.capture_cone_angle_entry.get_text().strip()
                if cone_text:
                    cone_angle_deg = float(cone_text)
                    if cone_angle_deg <= 0 or cone_angle_deg > 180:
                        raise ValueError("El cono debe estar entre 0 y 180 grados.")
            dist_text = self.capture_max_distance_entry.get_text().strip()
            if dist_text:
                max_distance_m = float(dist_text)
        except ValueError as exc:
            self._show_error("Parametros invalidos", str(exc) or "Cono y distancia maxima deben ser numeros.")
            return
        forward_axis = self.capture_forward_axis_combo.get_active_text() or "z"

        self.capture_base_button.set_sensitive(False)
        self.capture_updated_button.set_sensitive(False)
        self.stop_capture_button.set_sensitive(True)
        self.capture_stop_event = threading.Event()
        label = "tunel original" if target == "base" else "tunel con shotcrete"
        self.capture_status_label.set_text(f"Capturando {label} durante {duration_s:.0f} s...")
        self._log(f"Capturando nube ({label}) durante {duration_s:.0f} s, persistencia >= {persistence_ratio:.2f}...")

        stop_event = self.capture_stop_event

        def worker():
            try:
                cloud = aurora_sensor.capture_snapshot(
                    self.sensor_connection,
                    duration_s=duration_s,
                    persistence_ratio=persistence_ratio,
                    stop_event=stop_event,
                    max_distance_m=max_distance_m,
                    cone_angle_deg=cone_angle_deg,
                    forward_axis=forward_axis,
                )
                self._ui(self._on_capture_done, target, cloud)
            except Exception as exc:
                self._ui(self._on_capture_failed, exc)

        threading.Thread(target=worker, daemon=True).start()

    def _stop_capture_clicked(self) -> None:
        if self.capture_stop_event is not None:
            self.capture_stop_event.set()
            self.stop_capture_button.set_sensitive(False)
            self.capture_status_label.set_text("Deteniendo captura...")
            self._log("Captura detenida manualmente, procesando frames acumulados hasta ahora...")

    def _on_capture_done(self, target: str, cloud) -> None:
        self.capture_base_button.set_sensitive(True)
        self.capture_updated_button.set_sensitive(True)
        self.stop_capture_button.set_sensitive(False)
        self.capture_stop_event = None

        default_name = "base_capturada.ply" if target == "base" else "updated_capturada.ply"
        default_dir = PROJECT_ROOT / "data"
        default_dir.mkdir(parents=True, exist_ok=True)
        path = self._save_file_dialog("Guardar captura como", default_dir, default_name)
        if not path:
            self.capture_status_label.set_text("Captura descartada (no se eligio archivo de destino).")
            self._log("Captura descartada (no se eligio archivo de destino).")
            return

        import open3d as o3d

        o3d.io.write_point_cloud(path, cloud)
        self._log(f"Captura guardada en: {path} ({len(cloud.points)} puntos)")
        self.alignment_applied = False
        if target == "base":
            self.base_path = path
            self._set_path_label(self.base_path_label, path)
            self.last_base_capture_path = path
            self.view_base_capture_button.set_sensitive(True)
            self.capture_status_label.set_text("Tunel original capturado")
        else:
            self.updated_path = path
            self._set_path_label(self.updated_path_label, path)
            self.last_updated_capture_path = path
            self.view_updated_capture_button.set_sensitive(True)
            self.capture_status_label.set_text("Tunel con shotcrete capturado")

    def _view_last_capture(self, target: str) -> None:
        path = self.last_base_capture_path if target == "base" else self.last_updated_capture_path
        if not path:
            self._show_info("Sin captura", "Todavia no capturaste este tunel.")
            return
        label = "Tunel original" if target == "base" else "Tunel con shotcrete"

        def worker():
            try:
                cloud = load_point_cloud(Path(path))
            except Exception as exc:
                self._ui(self._show_error, "Error al abrir la captura", str(exc))
                return
            show_point_cloud(cloud, window_name=f"Aurora - {label} (ultima captura)")

        threading.Thread(target=worker, daemon=True).start()

    def _on_capture_failed(self, exc: Exception) -> None:
        self.capture_base_button.set_sensitive(True)
        self.capture_updated_button.set_sensitive(True)
        self.stop_capture_button.set_sensitive(False)
        self.capture_stop_event = None
        self.capture_status_label.set_text("")
        self._show_error("Error de captura", str(exc))

    # ------------------------------------------------------- Captura en vivo (MVP)

    def _ensure_live_capture_running(self) -> bool:
        self.source_live_rb.set_active(True)
        if self.viewer is None or not self.viewer.is_running():
            self._open_viewer()
            if self.viewer is None:
                return False
        self._apply_viewer_settings()
        self.viewer.set_live_sensor(self.sensor_connection)
        return True

    def _start_live_capture_clicked(self) -> None:
        if self.sensor_connection is None:
            self._show_warning("Sensor no conectado", "Conecta el sensor antes de iniciar captura en tiempo real.")
            return
        if not self._ensure_live_capture_running():
            return
        self.stack.set_visible_child_name("visualizacion")
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

        dialog = Gtk.FileChooserDialog(
            title="Seleccionar carpeta para guardar nubes en vivo",
            parent=self.window,
            action=Gtk.FileChooserAction.SELECT_FOLDER,
        )
        dialog.add_buttons("_Cancelar", Gtk.ResponseType.CANCEL, "_Seleccionar", Gtk.ResponseType.OK)
        target_dir = None
        if dialog.run() == Gtk.ResponseType.OK:
            target_dir = dialog.get_filename()
        dialog.destroy()
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

    # ---------------------------------------------------------------- Crop

    def _pick_crop_interactively(self) -> None:
        base_file = Path(self.base_path)
        if not base_file.exists():
            self._show_error("Error", f"No se encontro la nube base:\n{base_file}")
            return
        try:
            margin = float(self.crop_margin_entry.get_text() or 0.08)
            cloud = load_point_cloud(base_file)
            bounds = pick_crop_bounds(cloud, margin=margin)
        except Exception as exc:
            self._show_error("Error al abrir el visor 3D", str(exc))
            return

        if bounds is None:
            self._show_info(
                "Sin seleccion", "No se eligieron al menos 2 puntos. Repite usando Shift+Click sobre 2 esquinas."
            )
            return

        crop_min, crop_max = bounds
        self.crop_min_entry.set_text(f"{crop_min[0]:.4f} {crop_min[1]:.4f} {crop_min[2]:.4f}")
        self.crop_max_entry.set_text(f"{crop_max[0]:.4f} {crop_max[1]:.4f} {crop_max[2]:.4f}")
        self.use_crop_check.set_active(True)
        self.crop_status_label.set_text("Zona seleccionada ✓")
        self._show_info("Zona definida", "Se guardaron los limites de la zona a partir de los puntos elegidos.")

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
            "Puedes ir a la pestaña 'Alineacion' y marcar puntos de referencia fijos "
            "(por ejemplo pernos o marcas) antes de calcular, o continuar si sabes que "
            "las dos capturas se tomaron desde la misma posicion."
        )
        dialog.add_button("Ir a Alineacion", Gtk.ResponseType.CANCEL)
        continue_button = dialog.add_button("Continuar de todas formas", Gtk.ResponseType.OK)
        continue_button.get_style_context().add_class("destructive-action")
        response = dialog.run()
        dialog.destroy()
        if response == Gtk.ResponseType.OK:
            return True
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

        self.log_buffer.set_text("")
        self.run_button.set_sensitive(False)
        self.status_spinner.start()
        self.status_spinner.set_visible(True)
        self.status_label.set_markup("Calculando el espesor...")

        self.worker_thread = threading.Thread(target=self._run_pipeline_worker, args=(params,), daemon=True)
        self.worker_thread.start()

    def _build_params(self) -> PipelineParams:
        base_path = Path(self.base_path)
        updated_path = Path(self.updated_path)
        output_dir = Path(self.output_dir)

        crop_min = crop_max = None
        if self.use_crop_check.get_active():
            crop_min = self._parse_xyz(self.crop_min_entry.get_text(), "Esquina minima")
            crop_max = self._parse_xyz(self.crop_max_entry.get_text(), "Esquina maxima")

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
            crop_min=crop_min,
            crop_max=crop_max,
            max_distance=max_distance,
        )

    def _run_pipeline_worker(self, params: PipelineParams) -> None:
        try:
            self.result = run_pipeline(params, log=self._log)
            self._log("\nListo.")
            self._ui(self._on_pipeline_success)
            self._ui(self._show_result_in_viewer)
        except Exception as exc:
            self._log(f"\nERROR: {exc}")
            self._ui(self._on_pipeline_failure, str(exc))
            self._ui(self._show_error, "Error durante el analisis", str(exc))
        finally:
            self._ui(self.run_button.set_sensitive, True)
            self._ui(self.status_spinner.stop)
            self._ui(self.status_spinner.set_visible, False)

    def _on_pipeline_success(self) -> None:
        stats = self.result.stats
        self._set_stat(self.stat_points, f"{stats.n_points:,}")
        self._set_stat(self.stat_mean, f"{stats.mean * 100:.2f} cm")
        self._set_stat(self.stat_median, f"{stats.median * 100:.2f} cm")
        self._set_stat(self.stat_min, f"{stats.min * 100:.2f} cm")
        self._set_stat(self.stat_max, f"{stats.max * 100:.2f} cm")
        self._set_stat(self.stat_p95, f"{stats.p95 * 100:.2f} cm")
        self.cards_row.set_no_show_all(False)
        self.cards_row.show_all()
        self.status_label.set_markup(
            f'<span foreground="{COLOR_OK}">●</span>  Analisis completo — '
            f"espesor promedio {stats.mean * 100:.2f} cm"
        )
        self.generate_report_button.set_sensitive(True)
        self._generate_alerts()

    def _on_pipeline_failure(self, message: str) -> None:
        self.status_label.set_markup(f'<span foreground="{COLOR_ERROR}">●</span>  El analisis fallo')

    # ------------------------------------------------------- Alertas e informe

    def _generate_alerts(self) -> None:
        if not self.result:
            return
        low_mm = float(self.band_low_entry.get_text() or 20)
        high_mm = float(self.band_high_entry.get_text() or 50)
        mean_mm = self.result.stats.mean * 1000

        if mean_mm < low_mm:
            self._show_warning(
                "Alerta de espesor (falta)",
                f"El espesor medio ({mean_mm:.2f} mm) esta por debajo del umbral minimo ({low_mm} mm).\n"
                "Falta aplicar mas shotcrete.",
            )
        elif mean_mm >= high_mm:
            self._show_warning(
                "Alerta de espesor (exceso)",
                f"El espesor medio ({mean_mm:.2f} mm) supera el umbral maximo ({high_mm} mm).\n"
                "Exceso de shotcrete.",
            )

    def _generate_report(self) -> None:
        if not self.result:
            return

        import datetime

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

        default_dir = Path(self.output_dir)
        default_dir.mkdir(parents=True, exist_ok=True)
        now = datetime.datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")
        report_path = default_dir / f"informe_aurora_{timestamp}.pdf"

        stats = self.result.stats
        low_mm = float(self.band_low_entry.get_text() or 20)
        high_mm = float(self.band_high_entry.get_text() or 50)
        mean_mm = stats.mean * 1000

        if mean_mm < low_mm:
            status_text, status_color = "FALTA SHOTCRETE (bajo el umbral)", rl_colors.HexColor("#FFCC00")
        elif mean_mm >= high_mm:
            status_text, status_color = "EXCESO DE SHOTCRETE (sobre el umbral)", rl_colors.HexColor("#FF3B30")
        else:
            status_text, status_color = "DENTRO DE PARAMETRO", rl_colors.HexColor("#34C759")

        try:
            target_thickness_cm = float(self.target_thickness_entry.get_text() or 12)
        except ValueError:
            target_thickness_cm = 12.0
        band_width_cm = target_thickness_cm / len(SIX_BAND_LABELS)

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
            Paragraph(f"<b>Nube base (original):</b> {self.base_path}", body_style),
            Paragraph(f"<b>Nube actualizada (shotcrete):</b> {self.updated_path}", body_style),
            Paragraph("2. Umbrales de alerta", h2_style),
            Paragraph(f"Umbral bajo (mínimo): {low_mm / 10:.2f} cm — Umbral alto (máximo): {high_mm / 10:.2f} cm", body_style),
            Paragraph("3. Estadísticas de espesor", h2_style),
        ]

        stats_rows = [
            ["Métrica", "Valor (cm)"],
            ["Puntos analizados", f"{stats.n_points:,}"],
            ["Espesor medio", f"{stats.mean * 100:.2f}"],
            ["Espesor mediano", f"{stats.median * 100:.2f}"],
            ["Desviación estándar", f"{stats.std * 100:.2f}"],
            ["Mínimo", f"{stats.min * 100:.2f}"],
            ["Máximo", f"{stats.max * 100:.2f}"],
            ["Percentil 95", f"{stats.p95 * 100:.2f}"],
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

        story.append(Paragraph("4. Escala de colores (6 niveles, según espesor objetivo)", h2_style))
        story.append(Paragraph(f"Espesor objetivo: {target_thickness_cm:.2f} cm", body_style))
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
        for i, label in enumerate(SIX_BAND_LABELS):
            band_low = band_width_cm * i
            band_high = band_width_cm * (i + 1)
            r, g, b = SIX_BAND_COLORS[i]
            band_rows.append([f"{band_low:.2f} - {band_high:.2f}", label, ""])
            row_idx = i + 1
            band_styles.append(("BACKGROUND", (2, row_idx), (2, row_idx), rl_colors.Color(r, g, b)))
        band_table = Table(band_rows, colWidths=[5 * rl_cm, 5 * rl_cm, 4 * rl_cm])
        band_table.setStyle(TableStyle(band_styles))
        story.append(band_table)

        story.append(Paragraph("5. Distribución (histograma)", h2_style))
        try:
            story.append(Image(str(self.result.histogram_path), width=15 * rl_cm, height=9 * rl_cm))
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
        story.append(Paragraph(f"• Detalle por punto: {self.result.csv_path.name}", body_style))
        story.append(Paragraph(f"• Histograma: {self.result.histogram_path.name}", body_style))
        story.append(Paragraph(f"• Nube 3D heatmap: {self.result.heatmap_path.name}", body_style))

        doc = SimpleDocTemplate(
            str(report_path),
            pagesize=A4,
            leftMargin=2 * rl_cm,
            rightMargin=2 * rl_cm,
            topMargin=1.5 * rl_cm,
            bottomMargin=1.5 * rl_cm,
        )
        doc.build(story)

        self._log(f"Informe generado: {report_path}")
        self._show_info("Informe generado", f"El informe (PDF) fue guardado en:\n{report_path}")

    # --------------------------------------------------------------- Viewer

    def _band_thresholds_m(self) -> tuple[float, float]:
        low_mm = float(self.band_low_entry.get_text() or 20)
        high_mm = float(self.band_high_entry.get_text() or 50)
        return low_mm / 1000.0, high_mm / 1000.0

    def _parse_live_filter_params(self) -> tuple[float | None, float | None]:
        max_distance = None
        text = self.live_max_distance_entry.get_text().strip()
        if text:
            max_distance = float(text)
            if max_distance <= 0:
                raise ValueError("la distancia maxima debe ser > 0")

        cone_angle = None
        text = self.live_cone_angle_entry.get_text().strip()
        if text:
            cone_angle = float(text)
            if cone_angle <= 0 or cone_angle > 180:
                raise ValueError("el cono debe estar en (0, 180]")

        return max_distance, cone_angle

    def _show_result_in_viewer(self) -> None:
        self.stack.set_visible_child_name("visualizacion")
        if self.viewer is not None and self.viewer.is_running():
            self._apply_viewer_settings()
        else:
            self._open_viewer()

    def _open_viewer(self) -> None:
        base_file = Path(self.base_path)
        if not base_file.exists():
            self._show_error("Error", f"No se encontro la nube base:\n{base_file}")
            return

        if self.viewer is not None and self.viewer.is_running():
            self._show_info("Vista 3D", "La vista 3D ya esta abierta.")
            return

        try:
            base_cloud = self.result.base_cloud if self.result is not None else load_point_cloud(base_file)
        except Exception as exc:
            self._show_error("Error", str(exc))
            return

        self.viewer = LiveViewer(base_cloud)
        self.viewer.start()
        self.close_viewer_button.set_sensitive(True)
        self._apply_viewer_settings()
        self._log("Vista 3D abierta.")

    def _close_viewer(self) -> None:
        if self.viewer is not None:
            self.viewer.stop()
            self.viewer = None
        self.close_viewer_button.set_sensitive(False)

    def _on_color_mode_changed(self) -> None:
        self.band_revealer.set_reveal_child(self.color_banded_rb.get_active())
        self.six_bands_revealer.set_reveal_child(self.color_six_bands_rb.get_active())
        self._apply_viewer_settings()

    def _apply_viewer_settings(self) -> None:
        if self.viewer is None:
            return
        try:
            low_m, high_m = self._band_thresholds_m()
        except ValueError:
            low_m, high_m = 0.02, 0.05
        max_distance_text = self.max_distance_entry.get_text().strip()
        max_distance = float(max_distance_text) if max_distance_text else None
        try:
            target_thickness_m = float(self.target_thickness_entry.get_text() or 12) / 100.0
        except ValueError:
            target_thickness_m = 0.12

        if self.color_banded_rb.get_active():
            color_mode = "banded"
        elif self.color_six_bands_rb.get_active():
            color_mode = "six_bands"
        else:
            color_mode = "continuous"
        self.viewer.set_color_mode(color_mode, low_m, high_m, max_distance, target_thickness_m)
        self.viewer.set_show_updated(self.show_updated_check.get_active())

        try:
            live_max_distance, live_cone_angle = self._parse_live_filter_params()
        except ValueError as exc:
            self._log(f"Parametros de captura en vivo invalidos: {exc}. Se usan valores por defecto.")
            live_max_distance, live_cone_angle = None, None
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

    def _push_static_result_to_viewer(self) -> None:
        if self.viewer is None or not self.source_static_rb.get_active():
            return
        try:
            if self.result is not None:
                points = np.asarray(self.result.updated_cloud.points)
            else:
                updated_file = Path(self.updated_path)
                if not updated_file.exists():
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
        self._close_viewer()
        if self.sensor_connection is not None:
            try:
                aurora_sensor.disconnect(self.sensor_connection)
            except Exception:
                pass
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
    window.set_title("Aurora — Medicion de espesor de shotcrete")
    AuroraGUI(window)
    window.show_all()
    Gtk.main()


if __name__ == "__main__":
    main()
