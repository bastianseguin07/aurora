"""GUI GTK para sesiones exploratorias de medicion por tunel y sector.

Captura o importa nubes, registra cada etapa posterior con referencias y guarda
comparaciones e informes. La vista 3D auxiliar no certifica espesor.

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
import cairo
from pathlib import Path

import gi

gi.require_version("Gdk", "3.0")
gi.require_version("Gtk", "3.0")
from gi.repository import Gdk, GdkPixbuf, GLib, Gtk  # noqa: E402

import numpy as np  # noqa: E402

import aurora_sensor  # noqa: E402
from live_stream_server import make_qr_image  # noqa: E402
from live_viewer import LiveViewer  # noqa: E402
from sector_workflow import SectorWorkflowMixin
from pointcloud_core import (  # noqa: E402
    PipelineParams,
    apply_rigid_transform,
    compute_rigid_transform,
    crop_cloud_by_quad_box,
    load_point_cloud,
    pick_landmark_points,
    pick_quad_points,
    rigid_transform_rms_error,
    save_reference_pose,
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
COLOR_ACCENT = "#56B4C5"
COLOR_ACCENT_DARK = "#3B91A1"
COLOR_TEXT = "#F8F9FA"
COLOR_TEXT_MUTED = "#8E95A5"

CSS = f"""
* {{
    font-family: "Inter", "Roboto", "Segoe UI", sans-serif;
}}

window, .background {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
    border: none;
    box-shadow: none;
    padding: 0;
    margin: 0;
}}

window.csd,
window.background.csd,
window.csd decoration,
window.background.csd decoration,
decoration {{
    border: none;
    box-shadow: none;
    border-radius: 2px;
    padding: 0;
    margin: 0;
    background-image: none;
}}

decoration {{
    background-color: transparent;
}}

headerbar {{
    background-color: {COLOR_CARD};
    background-image: none;
    color: {COLOR_TEXT};
    border-bottom: 1px solid {COLOR_BORDER};
    border-radius: 2px 2px 0 0;
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

/* Un único foco visible para todos los controles interactivos. */
*:focus {{
    outline-color: rgba(86, 180, 197, 0.78);
    outline-style: solid;
    outline-width: 1px;
    outline-offset: 1px;
    box-shadow: none;
}}

list, listbox {{
    background-color: {COLOR_BG};
    color: {COLOR_TEXT};
}}

list row {{
    background-color: transparent;
    color: {COLOR_TEXT_MUTED};
    border: none;
    border-radius: 2px;
    margin: 2px 8px;
}}

list row:hover {{
    background-color: #1B2028;
    color: {COLOR_TEXT};
}}

list row.sidebar-heading,
list row.sidebar-heading:hover,
list row.sidebar-heading:selected {{
    background-color: transparent;
    border: none;
    box-shadow: none;
    color: {COLOR_TEXT_MUTED};
}}

list row:selected {{
    background-color: rgba(86, 180, 197, 0.16);
    color: {COLOR_TEXT};
    box-shadow: inset 3px 0 0 {COLOR_ACCENT};
}}

list row:selected:focus {{
    outline: none;
}}

stacksidebar {{
    background-color: {COLOR_BG};
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
    border-radius: 2px;
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
    border-radius: 2px;
    padding: 10px 16px;
    min-height: 28px;
    box-shadow: none;
    text-shadow: none;
    transition: none;
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

button:disabled label, checkbutton:disabled label, radiobutton:disabled label {{
    color: {COLOR_TEXT_MUTED};
}}

button.suggested-action {{
    background-color: {COLOR_ACCENT};
    background-image: none;
    color: {COLOR_TEXT};
    border: 1px solid {COLOR_ACCENT};
    font-weight: 700;
    opacity: 1;
}}

button.suggested-action:hover {{
    background-color: #73C8D6;
    background-image: none;
    color: {COLOR_TEXT};
    border-color: #73C8D6;
}}

button.suggested-action:disabled,
button.suggested-action:disabled label {{
    color: {COLOR_TEXT_MUTED};
    opacity: 1;
}}

button.suggested-action:disabled {{
    background-color: #20242B;
    background-image: none;
    border-color: {COLOR_BORDER};
}}

button.suggested-action:active {{
    background-color: {COLOR_ACCENT_DARK};
}}

entry {{
    background-color: #14161B;
    background-image: none;
    box-shadow: none;
    color: {COLOR_TEXT};
    border: 1px solid {COLOR_BORDER};
    border-radius: 2px;
    padding: 8px 10px;
    min-height: 24px;
    font-family: "Consolas", "DejaVu Sans Mono", monospace;
}}

entry:focus {{
    border-color: {COLOR_ACCENT};
}}

checkbutton, radiobutton {{
    color: {COLOR_TEXT};
    min-height: 32px;
}}

checkbutton check, radiobutton radio {{
    min-width: 16px;
    min-height: 16px;
    margin: 2px 8px 2px 2px;
    border: 1px solid #596271;
    background-color: #171B22;
    background-image: none;
    box-shadow: none;
    -gtk-icon-shadow: none;
    -gtk-icon-transform: none;
    transition: none;
}}

checkbutton check {{
    border-radius: 2px;
}}

radiobutton radio {{
    border-radius: 50%;
}}

checkbutton check:checked, radiobutton radio:checked {{
    background-color: {COLOR_ACCENT};
    border-color: {COLOR_ACCENT};
    color: #FFFFFF;
}}

checkbutton check:hover, radiobutton radio:hover {{
    border-color: {COLOR_ACCENT};
}}

checkbutton check:disabled, radiobutton radio:disabled {{
    background-color: #20242B;
    border-color: #373D48;
}}

checkbutton check:focus, radiobutton radio:focus {{
    outline-color: rgba(86, 180, 197, 0.78);
    outline-style: solid;
    outline-width: 1px;
    outline-offset: 2px;
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

.status-bar {{
    background-color: #14161B;
    border-top: 1px solid {COLOR_BORDER};
}}

.sector-flow button {{
    min-height: 22px;
    padding: 6px 12px;
}}

.sector-flow checkbutton {{
    min-height: 24px;
}}

.quick-result-panel {{
    background-color: #10141A;
    border: 1px solid {COLOR_BORDER};
    border-radius: 2px;
    padding: 12px;
}}

.quick-result-heading {{
    background-color: rgba(86, 180, 197, 0.16);
    border: 1px solid rgba(86, 180, 197, 0.42);
    border-radius: 2px;
    color: #E8FAFC;
    padding: 6px 12px;
    font-weight: 600;
}}

.quick-result-stage {{
    background-color: {COLOR_CARD};
    border: 1px solid {COLOR_BORDER};
    border-radius: 2px;
    padding: 10px;
}}

.quick-result-stage-title {{
    color: {COLOR_ACCENT};
    font-weight: 700;
}}

.quick-result-metric {{
    background-color: #111820;
    border: 1px solid #303946;
    border-radius: 2px;
    padding: 7px 12px;
}}

.quick-result-value {{
    color: #FFFFFF;
    font-size: 17px;
    font-weight: 700;
}}

.quick-result-caption {{
    color: #AEBAC8;
    font-size: 10px;
}}

.quick-result-note {{
    background-color: rgba(255, 255, 255, 0.045);
    border-radius: 2px;
    color: #D3DCE5;
    padding: 8px 10px;
}}

.quick-result-files {{
    color: #8FD8C8;
    font-size: 11px;
}}
""".encode("utf-8")


class AuroraGUI(SectorWorkflowMixin):
    def __init__(self, window: Gtk.Window) -> None:
        self.window = window
        rgba_visual = Gdk.Screen.get_default().get_rgba_visual()
        if rgba_visual is not None:
            self.window.set_visual(rgba_visual)
        self.window.set_default_size(1400, 950)
        self.window.set_size_request(880, 660)
        self._rounded_shape_size = None
        self.window.connect("realize", self._apply_rounded_corners)
        self.window.connect("size-allocate", self._apply_rounded_corners)
        self.window.connect("destroy", self._on_close)
        self.window.connect("delete-event", self._on_delete)
        self._sensor_connecting = False

        # El visor usa solo capturas de la sesion seleccionada; no mezcla
        # nubes antiguas de data/ con el sector actual.
        self.base_path = ""
        self.updated_path = ""
        self.output_dir = str(PROJECT_ROOT / "output")

        self.sensor_connection = None
        self.capture_stop_event: threading.Event | None = None
        self.viewer: LiveViewer | None = None
        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.worker_thread: threading.Thread | None = None

        self._build_layout()
        self._set_sensor_status("Desconectado", COLOR_ERROR)
        GLib.timeout_add(150, self._poll_log_queue)

    # ------------------------------------------------------------------ UI

    def _apply_rounded_corners(self, *_args) -> None:
        """Clip the X11 client surface so the GTK window has visible rounded corners."""
        native = self.window.get_window()
        if native is None:
            return
        width, height = native.get_width(), native.get_height()
        if width <= 0 or height <= 0 or self._rounded_shape_size == (width, height):
            return
        radius = min(2, width // 2, height // 2)
        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, width, height)
        ctx = cairo.Context(surface)
        ctx.set_source_rgba(1, 1, 1, 1)
        ctx.move_to(radius, 0)
        ctx.line_to(width - radius, 0)
        ctx.arc(width - radius, radius, radius, -1.5707963267948966, 0)
        ctx.line_to(width, height - radius)
        ctx.arc(width - radius, height - radius, radius, 0, 1.5707963267948966)
        ctx.line_to(radius, height)
        ctx.arc(radius, height - radius, radius, 1.5707963267948966, 3.141592653589793)
        ctx.line_to(0, radius)
        ctx.arc(radius, radius, radius, 3.141592653589793, 4.71238898038469)
        ctx.close_path()
        ctx.fill()
        native.shape_combine_region(Gdk.cairo_region_create_from_surface(surface), 0, 0)
        self._rounded_shape_size = (width, height)

    def _build_layout(self) -> None:
        header_bar = Gtk.HeaderBar()
        header_bar.set_show_close_button(True)
        header_bar.set_title("")
        header_bar.set_subtitle("")
        self.window.set_titlebar(header_bar)

        root_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.window.add(root_box)

        content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        content_box.set_vexpand(True)
        root_box.pack_start(content_box, True, True, 0)

        self.stack = Gtk.Stack()
        self.stack.set_hhomogeneous(False)
        self.stack.set_vhomogeneous(False)
        self.stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self.stack.set_transition_duration(0)

        self.stack.add_titled(self._build_simple_workflow_page(), "rapido", "Medici\u00f3n r\u00e1pida")
        self.stack.add_titled(self._build_alignment_page(), "alineacion", "Alineacion")
        self.stack.add_titled(self._build_segmentation_page(), "segmentacion", "Segmentacion")
        self.stack.add_titled(self._build_processing_page(), "procesamiento", "Ajustes de analisis")
        self.stack.add_titled(self._build_visualization_page(), "visualizacion", "Vista 3D auxiliar")
        self.stack.add_titled(self._build_tools_page(), "herramientas", "Herramientas tecnicas")

        sidebar = self._build_sidebar()
        content_box.pack_start(sidebar, False, False, 0)
        content_box.pack_start(self.stack, True, True, 0)

        self.technical_log_frame = Gtk.Expander(label="Registro tecnico · detalles de captura y proceso")
        self.technical_log_frame.get_style_context().add_class("status-bar")
        self.technical_log_frame.set_no_show_all(True)
        self.technical_log_frame.set_expanded(False)
        log_scroller = Gtk.ScrolledWindow()
        log_scroller.set_min_content_height(110)
        log_scroller.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        self.log_view = Gtk.TextView()
        self.log_view.set_editable(False)
        self.log_view.set_monospace(True)
        self.log_view.set_left_margin(6)
        self.log_view.set_top_margin(6)
        self.log_buffer = self.log_view.get_buffer()
        log_scroller.add(self.log_view)
        self.technical_log_frame.add(log_scroller)
        root_box.pack_start(self.technical_log_frame, False, True, 4)

        self._update_page_chrome(self.stack.get_visible_child_name())

    def _build_sidebar(self) -> Gtk.Widget:
        """Navegacion por flujo de medicion y herramientas tecnicas."""
        listbox = Gtk.ListBox()
        listbox.set_selection_mode(Gtk.SelectionMode.SINGLE)
        listbox.get_style_context().add_class("sidebar")
        self._sidebar_rows: dict[str, Gtk.ListBoxRow] = {}

        def add_header(text: str) -> None:
            row = Gtk.ListBoxRow()
            row.set_selectable(False)
            row.set_activatable(False)
            row.set_can_focus(False)
            row.get_style_context().add_class("sidebar-heading")
            label = Gtk.Label(xalign=0)
            label.set_markup(f'<small><b>{GLib.markup_escape_text(text)}</b></small>')
            label.get_style_context().add_class("dim-label")
            label.set_margin_start(12)
            label.set_margin_top(12)
            label.set_margin_bottom(2)
            row.add(label)
            listbox.add(row)

        def add_page(title: str, stack_name: str) -> None:
            row = Gtk.ListBoxRow()
            label = Gtk.Label(label=title, xalign=0)
            label.set_margin_start(16)
            label.set_margin_end(12)
            label.set_margin_top(8)
            label.set_margin_bottom(8)
            row.add(label)
            row.stack_name = stack_name
            listbox.add(row)
            self._sidebar_rows[stack_name] = row

        add_header("Medicion")
        add_page("Medici\u00f3n r\u00e1pida", "rapido")

        add_header("Mas herramientas")
        add_page("Herramientas tecnicas", "herramientas")

        def on_row_selected(_listbox, row) -> None:
            if row is not None and getattr(row, "stack_name", None):
                self.stack.set_visible_child_name(row.stack_name)

        listbox.connect("row-selected", on_row_selected)
        listbox.select_row(self._sidebar_rows["rapido"])

        def on_stack_page_changed(stack: Gtk.Stack, _pspec) -> None:
            name = stack.get_visible_child_name()
            row = self._sidebar_rows.get(name)
            if row is not None and listbox.get_selected_row() is not row:
                listbox.select_row(row)
            self._update_page_chrome(name)

        self.stack.connect("notify::visible-child-name", on_stack_page_changed)

        scroller = Gtk.ScrolledWindow()
        scroller.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        scroller.set_size_request(200, -1)
        scroller.add(listbox)
        return scroller

    def _build_tools_page(self) -> Gtk.Widget:
        page = self._new_page()
        intro = Gtk.Label(
            label="Funciones de uso ocasional y herramientas de análisis. El flujo habitual está en Medición rápida.",
            xalign=0,
        )
        intro.set_line_wrap(True)
        page.pack_start(intro, False, False, 8)
        self.tools_nav_buttons = {}
        groups = (
            ("Inspeccion", (
                ("Vista 3D y celular", "Inspeccionar una nube o compartirla por la red local.", "visualizacion"),
            )),
            ("Herramientas de sesion", (
                ("Ajustes del analisis", "Parametros tecnicos del procesamiento.", "procesamiento"),
            )),
        )
        for title, items in groups:
            frame, box = self._section(title)
            for label, description, page_name in items:
                row = self._row(box)
                text = Gtk.Label(label=f"{label} — {description}", xalign=0)
                text.set_line_wrap(True)
                row.pack_start(text, True, True, 0)
                button = Gtk.Button(label="Abrir")
                button.connect("clicked", lambda _button, name=page_name: self.stack.set_visible_child_name(name))
                row.pack_end(button, False, False, 0)
                self.tools_nav_buttons[page_name] = button
            page.pack_start(frame, False, True, 0)
        return self._scrolled(page)

    def _update_page_chrome(self, page_name: str | None) -> None:
        """Keep technical logs with the technical tools hub."""
        self.technical_log_frame.set_visible(page_name == "herramientas")

    # -- Pagina: Captura ---------------------------------------------------------

    def _build_mobile_stream_section(self) -> Gtk.Widget:
        frame, box = self._section("Transmision al celular")
        note = Gtk.Label(
            label="Conecta el computador y el celular a la misma red WiFi. Escanea el codigo QR para abrir el visor.",
            xalign=0,
        )
        note.set_line_wrap(True)
        box.pack_start(note, False, False, 0)
        row = self._row(box)
        self.stream_toggle_button = Gtk.Button(label="Iniciar transmision")
        self.stream_toggle_button.connect("clicked", lambda _b: self._toggle_stream_clicked())
        row.pack_start(self.stream_toggle_button, False, False, 0)
        row = self._row(box)
        self.stream_qr_image = Gtk.Image()
        row.pack_start(self.stream_qr_image, False, False, 0)
        qr_side_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        self.stream_url_label = Gtk.Label(label="", xalign=0)
        self.stream_url_label.set_selectable(True)
        qr_side_box.pack_start(self.stream_url_label, False, False, 0)
        row.pack_start(qr_side_box, False, False, 0)

        return frame

    def _set_base_path(self, path: str) -> None:
        self.base_path = path
        self._update_alignment_pair_label()

    def _set_updated_path(self, path: str) -> None:
        self.updated_path = path
        self._update_alignment_pair_label()

    def _update_alignment_pair_label(self) -> None:
        label = getattr(self, "alignment_pair_label", None)
        if label is None:
            return
        base = Path(self.base_path).name if self.base_path and Path(self.base_path).exists() else "sin ANTES"
        updated = Path(self.updated_path).name if self.updated_path and Path(self.updated_path).exists() else "sin DESPUES"
        label.set_text(f"ANTES: {base}  ->  DESPUES: {updated}")

    # -- Pagina: Alineacion (Procrustes por puntos de referencia) ---------------

    def _build_alignment_page(self) -> Gtk.Widget:
        page = self._new_page()

        self.landmarks_base: list | None = None
        self.landmarks_updated: list | None = None
        self.alignment_rotation: np.ndarray | None = None
        self.alignment_translation: np.ndarray | None = None
        self._raw_updated_path: str | None = None

        info_frame, info_box = self._section("Alinear con puntos de referencia")
        self.alignment_pair_label = Gtk.Label(label="Selecciona una etapa DESPUES en Medicion rapida.", xalign=0)
        info_box.pack_start(self.alignment_pair_label, False, False, 0)
        note = Gtk.Label(
            label=(
                "Usa al menos 3 marcas rigidas visibles en ambas capturas, en el mismo orden y "
                "fuera del shotcrete. La alineacion lleva DESPUES al marco de ANTES; si las "
                "marcas no coinciden, no la apliques."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        info_box.pack_start(note, False, False, 0)
        page.pack_start(info_frame, False, True, 0)

        step_frame, step_box = self._section("Paso 1: elegir los mismos puntos en ambas nubes")

        row = self._row(step_box)
        self.pick_base_alignment_button = Gtk.Button(label="1. Elegir puntos en el tunel original...")
        self.pick_base_alignment_button.set_tooltip_text(
            "Shift+Click sobre cada punto de referencia, en orden, despues cerrar la ventana."
        )
        self.pick_base_alignment_button.connect("clicked", lambda _b: self._pick_alignment_points("base"))
        row.pack_start(self.pick_base_alignment_button, False, False, 0)
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
        self.alignment_back_button = Gtk.Button(label="Volver a Medicion rapida")
        self.alignment_back_button.connect("clicked", lambda _b: self.stack.set_visible_child_name("rapido"))
        row.pack_start(self.alignment_back_button, False, False, 0)
        self.alignment_result_label = Gtk.Label(label="")
        row.pack_start(self.alignment_result_label, False, False, 0)
        page.pack_start(apply_frame, False, True, 0)

        return self._scrolled(page)

    def _choose_path(
        self,
        title: str,
        action: Gtk.FileChooserAction,
        accept_label: str,
        *,
        initial_dir: Path | None = None,
        filter_name: str | None = None,
        filter_pattern: str | None = None,
    ) -> str | None:
        dialog = Gtk.FileChooserDialog(title=title, parent=self.window, action=action)
        dialog.add_buttons("_Cancelar", Gtk.ResponseType.CANCEL, accept_label, Gtk.ResponseType.OK)
        if filter_pattern:
            file_filter = Gtk.FileFilter()
            file_filter.set_name(filter_name or filter_pattern)
            file_filter.add_pattern(filter_pattern)
            dialog.add_filter(file_filter)
        if initial_dir is not None and initial_dir.is_dir():
            dialog.set_current_folder(str(initial_dir))
        try:
            return dialog.get_filename() if dialog.run() == Gtk.ResponseType.OK else None
        finally:
            dialog.destroy()

    def _pick_alignment_points(self, which: str) -> None:
        path = Path(self.base_path if which == "base" else self.updated_path)
        if not path.is_file():
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
            self._show_error("Error al elegir los puntos", str(exc))
            return

        if points is None:
            self._show_info("Sin seleccion", "Elige al menos 3 puntos antes de cerrar la ventana.")
            return

        method_label = "nube 3D"
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

        # Se guardan ya calculadas (y la ruta cruda, previa a esta alineacion) para que
        # la pestaña "Segmentacion" pueda ubicar un box en la nube con shotcrete sin
        # tener que transformar la nube completa.
        self.alignment_rotation = rotation
        self.alignment_translation = translation
        self._raw_updated_path = self.updated_path

        self.apply_alignment_button.set_sensitive(False)
        if self._session_alignment_target:
            self._session_importing = True
            self._refresh_quick_workflow_state()
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
        self._session_importing = False
        try:
            self._session_registration_done(aligned_path, rms)
        except Exception as exc:
            self._on_alignment_failed(str(exc))
            return
        self._refresh_quick_workflow_state()
        self.apply_alignment_button.set_sensitive(True)
        self.alignment_result_label.set_text(f"Error residual: {rms * 1000:.2f} mm — nube alineada guardada")
        comparison_path = Path(aligned_path)
        session = getattr(self, "measurement_session", None)
        stage_index = self.session_stage_combo.get_active() if session else -1
        if session and stage_index >= 0:
            registration = session.data["stages"][stage_index]["registration"]
            if registration:
                comparison_path = session.resolve(registration["path"])
        self.updated_path = str(comparison_path)
        self._show_info(
            "Alineacion aplicada",
            f"Error residual en los puntos de referencia: {rms * 1000:.2f} mm.\n\n"
            f"La etapa de la sesion se actualizo con el archivo alineado:\n{comparison_path}",
        )

    def _on_alignment_failed(self, message: str) -> None:
        self._session_importing = False
        self._refresh_quick_workflow_state()
        self.apply_alignment_button.set_sensitive(True)
        self.alignment_result_label.set_text("Error al aplicar la alineacion")
        self._show_error("Error", message)

    # -- Pagina: Segmentacion -----------------------------------------------

    def _build_segmentation_page(self) -> Gtk.Widget:
        page = self._new_page()

        self.segmentation_quad: np.ndarray | None = None

        info_frame, info_box = self._section("Definir la zona comun de la sesion")
        note = Gtk.Label(
            label=(
                "Elegi 4 puntos (Shift+Click), en orden alrededor del perimetro, sobre la "
                "etapa DESPUES ya alineada para marcar un tramo del tunel. La zona queda "
                "guardada en la sesion y se aplica a ANTES y a cada etapa al calcular. "
                "No se crean PLY recortados intermedios."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        info_box.pack_start(note, False, False, 0)
        page.pack_start(info_frame, False, True, 0)

        step_frame, step_box = self._section("Elegir la region y el ancho del box")
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

        apply_frame, apply_box = self._section("Guardar zona en la sesion")
        row = self._row(apply_box)
        self.apply_segmentation_button = Gtk.Button(label="Guardar zona comun")
        self.apply_segmentation_button.connect("clicked", lambda _b: self._apply_segmentation())
        row.pack_start(self.apply_segmentation_button, False, False, 0)

        self.segmentation_result_label = Gtk.Label(xalign=0)
        self.segmentation_result_label.set_line_wrap(True)
        apply_box.pack_start(self.segmentation_result_label, False, False, 0)
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
        session = self._session_roi_target
        if session is None:
            self._show_warning("Sesion requerida", "Alinea una etapa desde Medicion rapida para guardar una zona comun.")
            return
        width_m = self.segmentation_width_spin.get_value() / 100.0
        quad = self.segmentation_quad
        updated_source = Path(self.updated_path)

        self.apply_segmentation_button.set_sensitive(False)
        self._session_importing = True
        self._refresh_quick_workflow_state()
        self.segmentation_result_label.set_text("Validando y guardando la zona comun...")

        def worker():
            try:
                stage_cropped = crop_cloud_by_quad_box(load_point_cloud(updated_source), quad, width_m)
                if len(stage_cropped.points) < 3:
                    raise RuntimeError("La zona no contiene suficientes puntos en la etapa seleccionada.")
                session.set_roi(quad, width_m)
            except Exception as exc:
                self._ui(self._on_segmentation_failed, str(exc))
                return
            self._ui(self._on_segmentation_done, session)

        threading.Thread(target=worker, daemon=True).start()

    def _on_segmentation_done(self, session) -> None:
        self._session_importing = False
        self._session_roi_target = None
        self.apply_segmentation_button.set_sensitive(True)
        self.segmentation_result_label.set_text("Zona comun guardada y aplicada a la sesion.")
        self.quick_capture_status_label.set_text(f"Zona comun guardada · {session.path.parent}")
        self.quick_crop_expander.set_expanded(False)
        self._refresh_quick_workflow_state()
        self.stack.set_visible_child_name("rapido")

    def _on_segmentation_failed(self, message: str) -> None:
        self._session_importing = False
        self._refresh_quick_workflow_state()
        self.apply_segmentation_button.set_sensitive(True)
        self.segmentation_result_label.set_text("No se guardo la zona")
        self._show_error("Error", message)

    # -- Pagina: Ajustes de analisis --------------------------------------------

    def _build_processing_page(self) -> Gtk.Widget:
        page = self._new_page()
        intro = Gtk.Label(
            label=(
                "La zona comun se define desde Medicion rapida, despues de alinear una etapa, "
                "y queda guardada en la sesion. Aqui puedes ajustar el muestreo y los "
                "filtros del calculo."
            ),
            xalign=0,
        )
        intro.set_line_wrap(True)
        page.pack_start(intro, False, False, 8)
        advanced_frame, _advanced_box = self._build_advanced_measurement_options()
        advanced_frame.set_expanded(True)
        page.pack_start(advanced_frame, False, True, 0)
        return self._scrolled(page)

    # -- Opciones avanzadas de medicion ------------------------------------------

    def _build_advanced_measurement_options(self) -> tuple[Gtk.Widget, Gtk.Box]:
        expander = Gtk.Expander(label="Ajustes avanzados de medicion")
        self.analysis_options_expander = expander
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

        view_frame, view_box = self._section("Inspeccion 3D")
        note = Gtk.Label(
            label=(
                "Revisa la forma y cobertura de la nube ANTES o el sensor en vivo. Esta vista no calcula "
                "espesor; para ver una comparacion registrada, usa Medicion rapida."
            ),
            xalign=0,
        )
        note.set_line_wrap(True)
        view_box.pack_start(note, False, False, 0)
        row = self._row(view_box)
        self.show_updated_check = Gtk.CheckButton(label="Mostrar la nube actual (celeste)")
        self.show_updated_check.set_active(True)
        self.show_updated_check.connect("toggled", lambda _b: self._on_show_updated_changed())
        row.pack_start(self.show_updated_check, False, False, 0)

        row = self._row(view_box)
        self.source_static_rb = Gtk.RadioButton.new_with_label_from_widget(None, "Nube actual guardada")
        self.source_static_rb.connect("toggled", lambda _b: self._on_updated_source_changed())
        row.pack_start(self.source_static_rb, False, False, 0)
        self.source_live_rb = Gtk.RadioButton.new_with_label_from_widget(
            self.source_static_rb, "Sensor en vivo"
        )
        self.source_live_rb.connect("toggled", lambda _b: self._on_updated_source_changed())
        row.pack_start(self.source_live_rb, False, False, 0)

        row = self._row(view_box)
        self.open_viewer_button = Gtk.Button(label="Abrir vista 3D")
        self.open_viewer_button.connect("clicked", lambda _b: self._open_viewer())
        row.pack_start(self.open_viewer_button, False, False, 0)
        self.close_viewer_button = Gtk.Button(label="Cerrar vista 3D")
        self.close_viewer_button.set_sensitive(False)
        self.close_viewer_button.connect("clicked", lambda _b: self._close_viewer())
        row.pack_start(self.close_viewer_button, False, False, 0)
        page.pack_start(view_frame, False, True, 0)

        page.pack_start(self._build_mobile_stream_section(), False, True, 0)

        live_filters = Gtk.Expander(label="Filtros del sensor en vivo (opcionales)")
        live_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        live_box.set_margin_top(8)
        row = self._row(live_box)
        row.pack_start(Gtk.Label(label="Alcance maximo (m):"), False, False, 0)
        self.live_max_distance_entry = Gtk.Entry()
        self.live_max_distance_entry.set_placeholder_text("Sin limite")
        self.live_max_distance_entry.set_width_chars(8)
        row.pack_start(self.live_max_distance_entry, False, False, 0)
        row.pack_start(Gtk.Label(label="Angulo del cono (grados):"), False, False, 0)
        self.live_cone_angle_entry = Gtk.Entry()
        self.live_cone_angle_entry.set_placeholder_text("Campo completo")
        self.live_cone_angle_entry.set_width_chars(8)
        row.pack_start(self.live_cone_angle_entry, False, False, 0)
        row = self._row(live_box)
        row.pack_start(Gtk.Label(label="Eje frontal:"), False, False, 0)
        self.live_forward_axis_combo = Gtk.ComboBoxText()
        for axis in ("x", "y", "z"):
            self.live_forward_axis_combo.append_text(axis)
        self.live_forward_axis_combo.set_active(2)
        row.pack_start(self.live_forward_axis_combo, False, False, 0)
        self.live_invert_y_check = Gtk.CheckButton(label="Invertir Y")
        row.pack_start(self.live_invert_y_check, False, False, 0)
        self.live_invert_z_check = Gtk.CheckButton(label="Invertir Z")
        row.pack_start(self.live_invert_z_check, False, False, 0)
        live_filters.add(live_box)
        page.pack_start(live_filters, False, True, 0)
        return self._scrolled(page)

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

    def _run_open_file_dialog(self) -> str | None:
        return self._choose_path(
            "Seleccionar archivo .ply",
            Gtk.FileChooserAction.OPEN,
            "_Abrir",
            initial_dir=PROJECT_ROOT / "data",
            filter_name="Archivos PLY",
            filter_pattern="*.ply",
        )

    def _browse_output_dir(self) -> None:
        path = self._choose_path(
            "Seleccionar carpeta de salida",
            Gtk.FileChooserAction.SELECT_FOLDER,
            "_Seleccionar",
            initial_dir=Path(self.output_dir),
        )
        if path:
            self.output_dir = path
            self._set_path_label(self.output_dir_label, self.output_dir)

    def _show_message(self, title: str, message: str, message_type: Gtk.MessageType) -> None:
        dialog = Gtk.MessageDialog(
            transient_for=self.window,
            flags=0,
            message_type=message_type,
            buttons=Gtk.ButtonsType.OK,
            text=title,
        )
        try:
            dialog.format_secondary_text(message)
            dialog.run()
        finally:
            dialog.destroy()

    def _show_error(self, title: str, message: str) -> None:
        self._show_message(title, message, Gtk.MessageType.ERROR)

    def _show_warning(self, title: str, message: str) -> None:
        self._show_message(title, message, Gtk.MessageType.WARNING)

    def _show_info(self, title: str, message: str) -> None:
        self._show_message(title, message, Gtk.MessageType.INFO)

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

    # ------------------------------------------------------------ Sensor

    def _set_sensor_status(self, text: str, color: str) -> None:
        markup = f'<span foreground="{color}">●</span>  {GLib.markup_escape_text(text)}'
        label = getattr(self, "quick_sensor_status_label", None)
        if label is not None:
            label.set_markup(markup)

    def _set_connect_buttons(self, text: str, sensitive: bool) -> None:
        button = getattr(self, "quick_connect_button", None)
        if button is not None:
            button.set_label(text)
            button.set_sensitive(sensitive)

    def _quick_connect_clicked(self) -> None:
        address = self.quick_sensor_address_entry.get_text().strip()
        if not address:
            self._show_error("IP invalida", "Escribe la direccion IP del sensor Aurora.")
            return
        self._toggle_sensor_connection()

    def _toggle_sensor_connection(self) -> None:
        if self._session_busy():
            self._show_warning("Trabajo en curso", "Espera a que termine el trabajo antes de cambiar la conexion.")
            return
        if self.sensor_connection is not None:
            self._disconnect_sensor()
            return

        address = self.quick_sensor_address_entry.get_text().strip()
        if not address:
            self._show_warning("Direccion requerida", "Escribe la IP del sensor Aurora.")
            return
        self._set_connect_buttons("Conectando...", False)
        self._sensor_connecting = True
        self._set_sensor_status("Conectando...", COLOR_WARN)
        self._refresh_quick_workflow_state()

        def worker():
            try:
                connection = aurora_sensor.connect(address)
                self._ui(self._on_sensor_connected, connection)
            except Exception as exc:
                self._ui(self._on_sensor_connect_failed, exc)

        threading.Thread(target=worker, daemon=True).start()

    def _on_sensor_connected(self, connection) -> None:
        self._sensor_connecting = False
        self.sensor_connection = connection
        import uuid
        self.sensor_connection_id = uuid.uuid4().hex
        self._set_connect_buttons("Desconectar", True)
        self._set_sensor_status("Conectado", COLOR_OK)
        if self.viewer is not None and self.source_live_rb.get_active():
            self.viewer.set_live_sensor(connection)
            self._apply_viewer_settings()
        self._log(f"Conectado al sensor Aurora en {self.quick_sensor_address_entry.get_text()}.")
        self._refresh_quick_workflow_state()
        pending_target = self._pending_quick_capture_target
        self._pending_quick_capture_target = None
        if pending_target is not None:
            if pending_target == "base":
                try:
                    self._new_pair_session()
                except Exception as exc:
                    self._show_error("No se pudo iniciar la comparacion", str(exc))
                    return
            self._capture_clicked(pending_target)

    def _on_sensor_connect_failed(self, exc: Exception) -> None:
        pending_target = self._pending_quick_capture_target
        self._sensor_connecting = False
        self._pending_quick_capture_target = None
        self._set_connect_buttons("Conectar", True)
        self._set_sensor_status("Desconectado", COLOR_ERROR)
        if pending_target is not None:
            self.quick_capture_status_label.set_text("No se pudo conectar. Revisa la IP e intenta de nuevo.")
        self._refresh_quick_workflow_state()
        self._show_error("Error de conexion", str(exc))

    def _disconnect_sensor(self) -> None:
        if self.viewer is not None:
            self.viewer.set_live_sensor(None)
        try:
            aurora_sensor.disconnect(self.sensor_connection)
        except Exception as exc:
            self._log(f"Aviso al desconectar: {exc}")
        self.sensor_connection = None
        self._set_connect_buttons("Conectar", True)
        self._set_sensor_status("Desconectado", COLOR_ERROR)
        self.quick_stop_capture_button.set_sensitive(False)
        self._refresh_quick_workflow_state()

    def _capture_clicked(self, target: str) -> None:
        if not self.measurement_session or self._session_busy():
            self._show_warning("Sesion requerida", "Crea o abre una sesion y espera a que termine el trabajo actual.")
            return
        if target == "base" and self.measurement_session.data["base"]:
            self._show_warning("BASE fija", "Crea otra sesion para cambiar la referencia.")
            return
        if self.capture_stop_event is not None or (self.worker_thread and self.worker_thread.is_alive()):
            self._show_warning("Trabajo en curso", "Espera a que termine la lectura o comparacion.")
            return
        if self.sensor_connection is None:
            self._show_warning("Sensor no conectado", "Conecta el sensor antes de capturar.")
            return

        duration_s = 15.0
        self.quick_stop_capture_button.set_sensitive(True)
        self.capture_stop_event = threading.Event()
        label = "tunel original" if target == "base" else "tunel con shotcrete"
        self.quick_capture_status_label.set_text(f"Leyendo {label} ({duration_s:.0f} s)...")
        self._log(f"Capturando nube ({label}) durante {duration_s:.0f} s...")
        self._set_connect_buttons("Capturando...", False)
        self._refresh_quick_workflow_state()
        stop_event = self.capture_stop_event

        def worker():
            try:
                cloud = aurora_sensor.capture_snapshot(
                    self.sensor_connection,
                    duration_s=duration_s,
                    persistence_ratio=0.0,
                    stop_event=stop_event,
                    max_distance_m=None,
                    cone_angle_deg=None,
                    forward_axis="z",
                )
                if stop_event.is_set():
                    self._ui(self._on_capture_cancelled)
                    return
                try:
                    ref_pose = aurora_sensor.get_current_pose(self.sensor_connection)
                except Exception:
                    ref_pose = None
                if stop_event.is_set():
                    self._ui(self._on_capture_cancelled)
                    return
                self._ui(self._on_capture_done, target, cloud, ref_pose, stop_event)
            except Exception as exc:
                if stop_event.is_set():
                    self._ui(self._on_capture_cancelled)
                else:
                    self._ui(self._on_capture_failed, exc)

        threading.Thread(target=worker, daemon=True).start()

    def _stop_capture_clicked(self) -> None:
        if self.capture_stop_event is not None:
            self.capture_stop_event.set()
            self.quick_stop_capture_button.set_sensitive(False)
            self.quick_capture_status_label.set_text("Cancelando captura...")
            self._log("Captura cancelada; se descartaran los frames parciales.")

    def _on_capture_cancelled(self) -> None:
        self.quick_stop_capture_button.set_sensitive(False)
        self.capture_stop_event = None
        connected = self.sensor_connection is not None
        self._set_connect_buttons("Desconectar" if connected else "Conectar", connected)
        self.quick_capture_status_label.set_text("Captura cancelada. No se guardo ninguna nube.")
        self._refresh_quick_workflow_state()

    def _on_capture_done(self, target: str, cloud, ref_pose=None, stop_event=None) -> None:
        # The worker may have queued this callback just before the operator cancelled.
        # Recheck on the GTK thread before writing any files or adding a session capture.
        if stop_event is not None and stop_event.is_set():
            self._on_capture_cancelled()
            return
        self.quick_stop_capture_button.set_sensitive(False)
        self.capture_stop_event = None
        connected = self.sensor_connection is not None
        self._set_connect_buttons("Desconectar" if connected else "Conectar", connected)
        self._refresh_quick_workflow_state()

        import datetime
        import open3d as o3d

        session = self.measurement_session
        if session is None:
            self._show_error("No hay sesion activa", "Inicia una medicion antes de capturar.")
            return
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = session.path.parent / f".capture_{target}_{timestamp}.ply"
        path.parent.mkdir(parents=True, exist_ok=True)
        if not o3d.io.write_point_cloud(str(path), cloud):
            self._show_error("No se guardo la captura", f"No se pudo escribir {path}")
            return
        self._log(f"Captura guardada en: {path} ({len(cloud.points)} puntos)")

        if ref_pose is not None:
            try:
                save_reference_pose(path, ref_pose.position, ref_pose.rpy_deg)
            except Exception as exc:
                self._log(f"No se pudo guardar la pose SLAM: {exc}")

        self._session_add_source(path, target, ref_pose, captured=True)
        self.quick_capture_status_label.set_text("Captura recibida; guardando en la sesion...")
        if target == "updated":
            self._show_quick_result_message(
                "Etapa DESPUES capturada. Alineala con referencias estables antes de calcular la comparacion."
            )

    def _on_capture_failed(self, exc: Exception) -> None:
        self.quick_stop_capture_button.set_sensitive(False)
        self.capture_stop_event = None
        connected = self.sensor_connection is not None
        self._set_connect_buttons("Desconectar" if connected else "Conectar", connected)
        self.quick_capture_status_label.set_text("La lectura fallo.")
        self._refresh_quick_workflow_state()
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

    # ---------------------------------------------------------------- Crop

    # ---------------------------------------------------------- Pipeline

    def _build_params(self) -> PipelineParams:
        base_path = Path(self.base_path)
        updated_path = Path(self.updated_path)
        output_dir = Path(self.output_dir)

        return PipelineParams(
            base_path=base_path,
            updated_path=updated_path,
            output_dir=output_dir,
            voxel_size=float(self.voxel_entry.get_text() or 0.0),
            remove_outliers=self.remove_outliers_check.get_active(),
        )

    def _finish_pipeline_ui(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            GLib.timeout_add(50, self._finish_pipeline_ui)
            return
        self._refresh_quick_workflow_state()

    # --------------------------------------------------------------- Viewer

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

    def _open_viewer(self) -> None:
        base_file = Path(self.base_path) if self.base_path else None
        if base_file is None or not base_file.is_file():
            self._show_info(
                "Sin nube del sector",
                "Abre o crea una sesion con una captura ANTES desde Medicion rapida para inspeccionar su nube.",
            )
            return

        if self.viewer is not None and self.viewer.is_running():
            self._show_info("Vista 3D", "La vista 3D ya esta abierta.")
            return

        try:
            base_cloud = load_point_cloud(base_file)
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

    def _apply_viewer_settings(self) -> None:
        if self.viewer is None:
            return
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

    def _on_delete(self, *_args) -> bool:
        if self._session_busy():
            if self.capture_stop_event is not None:
                self._stop_capture_clicked()
            self._show_warning("Guardado en curso", "Espera a que termine el trabajo y se guarde la sesion antes de cerrar.")
            return True
        return False

    def _on_close(self, *_args) -> None:
        self._close_viewer()
        if self.sensor_connection is not None:
            try:
                aurora_sensor.disconnect(self.sensor_connection)
            except Exception:
                pass
        Gtk.main_quit()


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description="Aurora: sesiones exploratorias por sector")
    parser.add_argument("--session", type=Path, help="Abrir un sesion.json existente")
    args = parser.parse_args()
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
    app = AuroraGUI(window)
    if args.session:
        app._session_load(args.session)
    window.resize(1400, 950)
    window.show_all()
    window.present()
    Gtk.main()


if __name__ == "__main__":
    main()
