"""
GUI de escritorio (CustomTkinter, sobre tkinter) para comparar dos nubes de
puntos .ply sin usar la consola. Incluye recorte (crop) manual o
seleccionado visualmente en un visor 3D, captura desde el sensor Aurora,
coloreado por espesor (continuo o 3 niveles) y vista 3D en vivo o estatica.

Ejecutar con:
    python gui.py
"""

from __future__ import annotations

import csv
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox

import customtkinter as ctk
import numpy as np

import aurora_sensor
from live_stream_server import make_qr_image
from live_viewer import LiveViewer
from pointcloud_core import (
    PipelineParams,
    load_point_cloud,
    load_reference_pose,
    pick_crop_bounds,
    render_reference_photo_from_cloud,
    run_pipeline,
    save_reference_photo,
    save_reference_pose,
    visualize,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent

ctk.set_appearance_mode("System")
ctk.set_default_color_theme("blue")

FONT_TITLE = ("Segoe UI", 15, "bold")
FONT_SECTION = ("Segoe UI", 13, "bold")
FONT_BODY = ("Segoe UI", 12)
FONT_SMALL = ("Segoe UI", 10)

COLOR_OK = "#2fa84f"
COLOR_ERROR = "#d9534f"
COLOR_WARN = "#d9a53f"
COLOR_MUTED = "#8a8d93"


class AuroraGUI:
    QUICK_POSITION_TOLERANCE_CM = 0.5
    QUICK_ROTATION_TOLERANCE_DEG = 3.0

    def __init__(self, root: ctk.CTk) -> None:
        self.root = root
        self.root.title("Aurora — Analisis de espesor de shotcrete")
        self.root.geometry("1000x850")
        self.root.minsize(860, 700)

        self.base_path = tk.StringVar(value=str(PROJECT_ROOT / "data" / "base.ply"))
        self.updated_path = tk.StringVar(value=str(PROJECT_ROOT / "data" / "updated.ply"))
        self.output_dir = tk.StringVar(value=str(PROJECT_ROOT / "output"))
        self.voxel_size = tk.StringVar(value="0.0")
        self.remove_outliers = tk.BooleanVar(value=False)
        self.use_icp = tk.BooleanVar(value=False)
        self.icp_threshold = tk.StringVar(value="0.05")
        self.use_crop = tk.BooleanVar(value=False)
        self.crop_min = tk.StringVar(value="")
        self.crop_max = tk.StringVar(value="")
        self.crop_margin = tk.StringVar(value="0.08")
        self.max_distance = tk.StringVar(value="")

        self.color_mode = tk.StringVar(value="banded")
        self.band_low_mm = tk.StringVar(value="50")
        self.band_high_mm = tk.StringVar(value="100")

        self.show_updated = tk.BooleanVar(value=True)
        self.updated_source = tk.StringVar(value="static")  # "static" o "live"

        self.sensor_address = tk.StringVar(value="192.168.11.1")
        self.sensor_connection = None
        self.quick_base_path: str | None = None
        self.quick_updated_path: str | None = None
        self.quick_stage_results: list[tuple[str, float, float, float, int, str]] = []
        self.quick_pending_stage: str | None = None
        self.quick_session_summary_path: Path | None = None
        self.quick_capture_stop_event: threading.Event | None = None
        self.capture_in_progress = False

        self.result = None
        self.viewer: LiveViewer | None = None
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.worker_thread: threading.Thread | None = None

        self._build_layout()
        self.root.after(150, self._poll_log_queue)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ UI

    def _build_layout(self) -> None:
        header = ctk.CTkFrame(self.root, fg_color="transparent")
        header.pack(fill="x", padx=16, pady=(14, 4))
        ctk.CTkLabel(header, text="Aurora", font=("Segoe UI", 20, "bold")).pack(side="left")
        ctk.CTkLabel(
            header,
            text="  Analisis de espesor de shotcrete a partir de nubes de puntos",
            font=FONT_BODY,
            text_color=COLOR_MUTED,
        ).pack(side="left")

        self.tabview = ctk.CTkTabview(self.root, corner_radius=10)
        self.tabview.pack(fill="x", padx=16, pady=(4, 8))
        self.tabview.add("Medicion rapida")
        self.tabview.add("Datos y sensor")
        self.tabview.add("Procesamiento")
        self.tabview.add("Visualizacion")
        self.tabview.configure(height=330)

        self._build_quick_tab(self.tabview.tab("Medicion rapida"))
        self._build_files_tab(self.tabview.tab("Datos y sensor"))
        self._build_processing_tab(self.tabview.tab("Procesamiento"))
        self._build_visualization_tab(self.tabview.tab("Visualizacion"))
        self.tabview.set("Medicion rapida")

        actions_frame = ctk.CTkFrame(self.root, fg_color="transparent")
        actions_frame.pack(fill="x", padx=16, pady=(0, 6))
        self.run_button = ctk.CTkButton(
            actions_frame,
            text="▶  Ejecutar comparacion",
            command=self._run_pipeline_clicked,
            height=38,
            font=FONT_SECTION,
        )
        self.run_button.pack(side="left")
        self.view_heatmap_button = ctk.CTkButton(
            actions_frame,
            text="Ver heatmap 3D (ventana simple)",
            command=self._view_heatmap,
            height=38,
            fg_color="transparent",
            border_width=1,
            text_color=("gray10", "gray90"),
            state="disabled",
        )
        self.view_heatmap_button.pack(side="left", padx=8)

        log_section = self._section(self.root, "Resultado", fill="both", expand=True)
        self.log_text = ctk.CTkTextbox(log_section, height=180, font=("Consolas", 11))
        self.log_text.pack(fill="both", expand=True, padx=14, pady=(0, 14))

    # -- Tab: Medicion rapida -------------------------------------------------

    def _build_quick_tab(self, parent) -> None:
        content = ctk.CTkScrollableFrame(parent, fg_color="transparent", corner_radius=0)
        content.pack(fill="both", expand=True)
        intro = ctk.CTkLabel(
            content,
            text="Conecta el sensor, captura BASE antes del shotcrete y compara cada etapa DESPUES.",
            font=FONT_BODY,
            anchor="w",
            wraplength=880,
        )
        intro.pack(fill="x", padx=16, pady=(10, 4))

        connection_frame = self._section(content, "1. Conectar", fill="x")
        row = self._row(connection_frame)
        ctk.CTkLabel(row, text="IP del sensor:", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.sensor_address, width=150).pack(side="left", padx=8)
        self.quick_connect_button = ctk.CTkButton(
            row, text="Conectar", width=110, command=self._toggle_sensor_connection
        )
        self.quick_connect_button.pack(side="left", padx=8)
        self.quick_sensor_status = ctk.CTkLabel(row, text="●  Desconectado", font=FONT_BODY, text_color=COLOR_ERROR)
        self.quick_sensor_status.pack(side="left", padx=8)

        capture_frame = self._section(content, "2. Capturar las etapas", fill="x")
        ctk.CTkLabel(
            capture_frame,
            text=(
                "Lee BASE antes del shotcrete. Mantén el sensor encendido y vuelve a la misma "
                "posición y orientación antes de cada lectura DESPUES."
            ),
            font=FONT_SMALL,
            anchor="w",
            wraplength=880,
        ).pack(fill="x", padx=14, pady=(0, 6))
        pose_row = self._row(capture_frame, pady=(2, 4))
        self.quick_pose_button = ctk.CTkButton(
            pose_row,
            text="Ver distancia a BASE",
            command=self._show_quick_pose_delta,
            state="disabled",
            fg_color="transparent",
            border_width=1,
            text_color=("gray10", "gray90"),
        )
        self.quick_pose_button.pack(side="left")
        self.quick_pose_status = ctk.CTkLabel(
            pose_row,
            text="Captura la BASE para activar la guia IMU.",
            anchor="w",
            wraplength=700,
        )
        self.quick_pose_status.pack(side="left", padx=10)
        self.quick_capture_base_button = ctk.CTkButton(
            capture_frame,
            text="Leer BASE (antes)",
            command=lambda: self._capture_clicked("base", quick=True),
            state="disabled",
        )
        self.quick_capture_base_button.pack(anchor="w", padx=14, pady=3)
        self.quick_base_label = ctk.CTkLabel(capture_frame, text="Sin captura BASE", anchor="w")
        self.quick_base_label.pack(fill="x", padx=14, pady=(0, 4))
        self.quick_capture_updated_button = ctk.CTkButton(
            capture_frame,
            text="Leer DESPUES (shotcrete)",
            command=lambda: self._capture_clicked("updated", quick=True),
            state="disabled",
        )
        self.quick_capture_updated_button.pack(anchor="w", padx=14, pady=3)
        self.quick_updated_label = ctk.CTkLabel(capture_frame, text="Sin captura DESPUES", anchor="w")
        self.quick_updated_label.pack(fill="x", padx=14, pady=(0, 4))
        capture_actions = self._row(capture_frame, pady=(2, 8))
        self.quick_stop_button = ctk.CTkButton(
            capture_actions,
            text="Detener lectura",
            command=self._stop_quick_capture,
            state="disabled",
            fg_color="transparent",
            border_width=1,
            text_color=("gray10", "gray90"),
        )
        self.quick_stop_button.pack(side="left")
        self.quick_capture_status = ctk.CTkLabel(capture_actions, text="", anchor="w")
        self.quick_capture_status.pack(side="left", padx=10)
        ctk.CTkLabel(
            capture_frame,
            text="Valores automaticos: 15 s, persistencia 0, campo visual completo, eje Z.",
            font=FONT_SMALL,
            text_color=COLOR_MUTED,
            anchor="w",
        ).pack(fill="x", padx=14, pady=(0, 8))

        compare_frame = self._section(content, "3. Comparar espesor", fill="x")
        ctk.CTkLabel(
            compare_frame,
            text="Cada lectura DESPUES se compara de forma acumulada contra la misma BASE.",
            font=FONT_SMALL,
            anchor="w",
        ).pack(fill="x", padx=14, pady=(0, 6))
        self.quick_compare_button = ctk.CTkButton(
            compare_frame,
            text="Comparar BASE con DESPUES",
            command=self._quick_compare_clicked,
            state="disabled",
            height=36,
        )
        self.quick_compare_button.pack(anchor="w", padx=14, pady=3)
        self.quick_result_label = ctk.CTkLabel(
            compare_frame,
            text="Captura una BASE y una etapa DESPUES para comparar.",
            anchor="w",
            justify="left",
            wraplength=880,
        )
        self.quick_result_label.pack(fill="x", padx=14, pady=(3, 10))

    # -- Tab: Datos y sensor -------------------------------------------------

    def _build_files_tab(self, parent) -> None:
        files_frame = self._section(parent, "Archivos", fill="x")
        self._file_row(files_frame, "Nube base (original):", self.base_path)
        self._file_row(files_frame, "Nube actualizada (con shotcrete):", self.updated_path)
        self._dir_row(files_frame, "Carpeta de salida:", self.output_dir)

        sensor_frame = self._section(parent, "Sensor Aurora (captura de nubes)", fill="x")

        row = self._row(sensor_frame)
        ctk.CTkLabel(row, text="Direccion del sensor (IP):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.sensor_address, width=140).pack(side="left", padx=8)
        self.connect_button = ctk.CTkButton(row, text="Conectar", width=110, command=self._toggle_sensor_connection)
        self.connect_button.pack(side="left", padx=8)
        self.sensor_status_label = ctk.CTkLabel(
            row, text="●  Desconectado", font=FONT_BODY, text_color=COLOR_ERROR
        )
        self.sensor_status_label.pack(side="left", padx=8)

        row = self._row(sensor_frame, pady=(0, 12))
        self.capture_base_button = ctk.CTkButton(
            row, text="Capturar nube BASE", command=lambda: self._capture_clicked("base"), state="disabled"
        )
        self.capture_base_button.pack(side="left")
        self.capture_updated_button = ctk.CTkButton(
            row,
            text="Capturar nube ACTUALIZADA",
            command=lambda: self._capture_clicked("updated"),
            state="disabled",
        )
        self.capture_updated_button.pack(side="left", padx=8)

    # -- Tab: Procesamiento ---------------------------------------------------

    def _build_processing_tab(self, parent) -> None:
        options_frame = self._section(parent, "Opciones de procesamiento", fill="x")

        row = self._row(options_frame)
        ctk.CTkLabel(row, text="Tamano de voxel (m, 0 = sin downsample):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.voxel_size, width=80).pack(side="left", padx=8)

        row = self._row(options_frame)
        ctk.CTkCheckBox(row, text="Quitar outliers estadisticos", variable=self.remove_outliers).pack(side="left")

        row = self._row(options_frame, pady=(0, 12))
        ctk.CTkCheckBox(row, text="Alinear con ICP antes de medir", variable=self.use_icp).pack(side="left")
        ctk.CTkLabel(row, text="   Umbral ICP (m):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.icp_threshold, width=70).pack(side="left", padx=8)

        crop_frame = self._section(parent, "Recorte a region de interes (crop)", fill="x")
        ctk.CTkCheckBox(
            crop_frame, text="Aplicar recorte a ambas nubes antes de comparar", variable=self.use_crop
        ).pack(anchor="w", padx=14, pady=(0, 8))

        row = self._row(crop_frame)
        ctk.CTkLabel(row, text="Min (x y z):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.crop_min, width=220).pack(side="left", padx=8)
        ctk.CTkLabel(row, text="Max (x y z):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.crop_max, width=220).pack(side="left", padx=8)

        row = self._row(crop_frame, pady=(0, 12))
        ctk.CTkButton(row, text="Seleccionar recorte en visor 3D...", command=self._pick_crop_interactively).pack(
            side="left"
        )
        ctk.CTkLabel(row, text="  Margen extra (m):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.crop_margin, width=70).pack(side="left", padx=8)

    # -- Tab: Visualizacion ----------------------------------------------------

    def _build_visualization_tab(self, parent) -> None:
        color_frame = self._section(parent, "Color de espesor", fill="x")

        row = self._row(color_frame)
        ctk.CTkRadioButton(
            row, text="Continuo (heatmap azul -> rojo)", variable=self.color_mode, value="continuous"
        ).pack(side="left")
        ctk.CTkRadioButton(
            row, text="3 niveles (verde / amarillo / rojo)", variable=self.color_mode, value="banded"
        ).pack(side="left", padx=16)

        row = self._row(color_frame)
        ctk.CTkLabel(row, text="Umbral bajo (mm, verde <):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.band_low_mm, width=70).pack(side="left", padx=8)
        ctk.CTkLabel(row, text="Umbral alto (mm, rojo >=):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.band_high_mm, width=70).pack(side="left", padx=8)

        row = self._row(color_frame, pady=(0, 12))
        ctk.CTkLabel(row, text="Escala heatmap continuo, distancia max. (m, vacio = auto):", font=FONT_BODY).pack(
            side="left"
        )
        ctk.CTkEntry(row, textvariable=self.max_distance, width=80).pack(side="left", padx=8)

        view_frame = self._section(parent, "Vista 3D", fill="x")

        row = self._row(view_frame)
        ctk.CTkCheckBox(
            row,
            text="Mostrar nube actualizada (con shotcrete)",
            variable=self.show_updated,
            command=self._on_show_updated_changed,
        ).pack(side="left")

        row = self._row(view_frame)
        ctk.CTkLabel(row, text="Nube actualizada:", font=FONT_BODY).pack(side="left")
        ctk.CTkRadioButton(
            row,
            text="Estatica (archivo/captura)",
            variable=self.updated_source,
            value="static",
            command=self._on_updated_source_changed,
        ).pack(side="left", padx=(8, 0))
        ctk.CTkRadioButton(
            row,
            text="En tiempo real (sensor)",
            variable=self.updated_source,
            value="live",
            command=self._on_updated_source_changed,
        ).pack(side="left", padx=16)

        row = self._row(view_frame, pady=(0, 12))
        self.open_viewer_button = ctk.CTkButton(row, text="Abrir vista 3D", command=self._open_viewer)
        self.open_viewer_button.pack(side="left")
        self.close_viewer_button = ctk.CTkButton(
            row,
            text="Cerrar vista 3D",
            command=self._close_viewer,
            state="disabled",
            fg_color="transparent",
            border_width=1,
            text_color=("gray10", "gray90"),
        )
        self.close_viewer_button.pack(side="left", padx=8)

        stream_frame = self._section(parent, "Transmitir a un celular", fill="x")
        row = self._row(stream_frame)
        self.stream_toggle_button = ctk.CTkButton(row, text="Iniciar transmision", command=self._toggle_stream)
        self.stream_toggle_button.pack(side="left")
        self.stream_url_var = tk.StringVar(value="")
        ctk.CTkLabel(row, textvariable=self.stream_url_var, font=FONT_BODY).pack(side="left", padx=8)
        row = self._row(stream_frame, pady=(0, 12))
        self.stream_qr_label = ctk.CTkLabel(row, text="", image=None)
        self.stream_qr_label.pack(side="left")

    # -- Helpers de layout -----------------------------------------------------

    def _section(self, parent, title: str, fill: str = "x", expand: bool = False) -> ctk.CTkFrame:
        frame = ctk.CTkFrame(parent, corner_radius=10)
        frame.pack(fill=fill, expand=expand, padx=14 if parent is self.root else 4, pady=(10, 4))
        ctk.CTkLabel(frame, text=title, font=FONT_SECTION).pack(anchor="w", padx=14, pady=(10, 4))
        return frame

    def _row(self, parent, pady=(2, 6)) -> ctk.CTkFrame:
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=pady)
        return row

    def _file_row(self, parent, label: str, var: tk.StringVar) -> None:
        row = self._row(parent)
        ctk.CTkLabel(row, text=label, width=220, anchor="w", font=FONT_BODY).pack(side="left")
        ctk.CTkButton(row, text="Examinar...", width=100, command=lambda: self._browse_file(var)).pack(
            side="right"
        )
        ctk.CTkEntry(row, textvariable=var).pack(side="left", fill="x", expand=True, padx=8)

    def _dir_row(self, parent, label: str, var: tk.StringVar) -> None:
        row = self._row(parent, pady=(2, 12))
        ctk.CTkLabel(row, text=label, width=220, anchor="w", font=FONT_BODY).pack(side="left")
        ctk.CTkButton(row, text="Examinar...", width=100, command=lambda: self._browse_dir(var)).pack(side="right")
        ctk.CTkEntry(row, textvariable=var).pack(side="left", fill="x", expand=True, padx=8)

    def _browse_file(self, var: tk.StringVar) -> None:
        path = filedialog.askopenfilename(title="Seleccionar archivo .ply", filetypes=[("PLY files", "*.ply")])
        if path:
            var.set(path)

    def _browse_dir(self, var: tk.StringVar) -> None:
        path = filedialog.askdirectory(title="Seleccionar carpeta de salida")
        if path:
            var.set(path)

    def _log(self, message: str) -> None:
        self.log_queue.put(message)

    def _poll_log_queue(self) -> None:
        try:
            while True:
                message = self.log_queue.get_nowait()
                self.log_text.insert("end", message + "\n")
                self.log_text.see("end")
        except queue.Empty:
            pass
        self.root.after(150, self._poll_log_queue)

    def _parse_xyz(self, text: str, field_name: str) -> tuple[float, float, float]:
        parts = text.replace(",", " ").split()
        if len(parts) != 3:
            raise ValueError(f"{field_name} debe tener 3 numeros (x y z), separados por espacio o coma.")
        return tuple(float(p) for p in parts)

    # ------------------------------------------------------------ Sensor

    def _set_sensor_ui(self, label: str, status: str, color: str, enabled: bool) -> None:
        for button_name in ("connect_button", "quick_connect_button"):
            button = getattr(self, button_name, None)
            if button is not None:
                button.configure(text=label, state="normal" if enabled else "disabled")
        for status_name in ("sensor_status_label", "quick_sensor_status"):
            status_widget = getattr(self, status_name, None)
            if status_widget is not None:
                status_widget.configure(text=status, text_color=color)

    def _set_capture_ui(self, enabled: bool) -> None:
        can_capture = enabled and self.sensor_connection is not None
        for button_name in (
            "capture_base_button",
            "capture_updated_button",
            "quick_capture_base_button",
            "quick_capture_updated_button",
        ):
            button = getattr(self, button_name, None)
            if button is not None:
                button.configure(state="normal" if can_capture else "disabled")

    def _refresh_quick_state(self) -> None:
        base = Path(self.quick_base_path) if self.quick_base_path else None
        updated = Path(self.quick_updated_path) if self.quick_updated_path else None
        self.quick_base_label.configure(text=base.name if base and base.is_file() else "Sin captura BASE")
        self.quick_updated_label.configure(
            text=updated.name if updated and updated.is_file() else "Sin captura DESPUES"
        )
        worker_running = bool(self.worker_thread and self.worker_thread.is_alive())
        capturing = self.capture_in_progress
        self._set_capture_ui(self.sensor_connection is not None and not worker_running and not capturing)
        pose_available = bool(self.sensor_connection is not None and base and base.is_file())
        self.quick_pose_button.configure(state="normal" if pose_available and not capturing else "disabled")
        can_compare = bool(base and base.is_file() and updated and updated.is_file())
        self.quick_compare_button.configure(
            state="normal" if can_compare and not worker_running and not capturing else "disabled"
        )

    def _show_quick_pose_delta(self) -> None:
        if self.sensor_connection is None or not self.quick_base_path:
            self.quick_pose_status.configure(text="Conecta el sensor y captura la BASE primero.")
            return
        try:
            base_pose = load_reference_pose(Path(self.quick_base_path))
            current_pose = aurora_sensor.get_current_pose(self.sensor_connection)
        except Exception as exc:
            self.quick_pose_status.configure(text=f"No se pudo leer la pose IMU: {exc}")
            return
        if base_pose is None:
            self.quick_pose_status.configure(text="La captura BASE no tiene pose IMU guardada.")
            return
        delta_cm = (base_pose[0] - current_pose.position) * 100.0
        delta_rotation = ((base_pose[1] - current_pose.rpy_deg + 180.0) % 360.0) - 180.0
        within_position = bool(np.all(np.abs(delta_cm) <= self.QUICK_POSITION_TOLERANCE_CM))
        within_rotation = bool(np.all(np.abs(delta_rotation) <= self.QUICK_ROTATION_TOLERANCE_DEG))
        state = "LISTO" if within_position and within_rotation else "AJUSTAR"
        self.quick_pose_status.configure(
            text=(
                f"{state} · ΔX {delta_cm[0]:+.1f} cm, ΔY {delta_cm[1]:+.1f} cm, "
                f"ΔZ {delta_cm[2]:+.1f} cm · ΔR/P/Y "
                f"{delta_rotation[0]:+.1f}/{delta_rotation[1]:+.1f}/{delta_rotation[2]:+.1f}°"
            )
        )

    def _toggle_sensor_connection(self) -> None:
        if self.sensor_connection is not None:
            self._disconnect_sensor()
            return

        address = self.sensor_address.get().strip()
        self._set_sensor_ui("Conectando...", "●  Conectando...", COLOR_WARN, False)

        def worker():
            try:
                connection = aurora_sensor.connect(address)
                self.root.after(0, lambda: self._on_sensor_connected(connection))
            except Exception as exc:
                self.root.after(0, lambda exc=exc: self._on_sensor_connect_failed(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_sensor_connected(self, connection) -> None:
        self.sensor_connection = connection
        self._set_sensor_ui("Desconectar", "●  Conectado", COLOR_OK, True)
        self._set_capture_ui(True)
        if self.viewer is not None and self.updated_source.get() == "live":
            self.viewer.set_live_sensor(connection)
        self._log(f"Conectado al sensor Aurora en {self.sensor_address.get()}.")

    def _on_sensor_connect_failed(self, exc: Exception) -> None:
        self._set_sensor_ui("Conectar", "●  Desconectado", COLOR_ERROR, True)
        messagebox.showerror("Error de conexion", str(exc))

    def _disconnect_sensor(self) -> None:
        if self.viewer is not None:
            self.viewer.set_live_sensor(None)
        try:
            aurora_sensor.disconnect(self.sensor_connection)
        except Exception as exc:
            self._log(f"Aviso al desconectar: {exc}")
        self.sensor_connection = None
        self._set_sensor_ui("Conectar", "●  Desconectado", COLOR_ERROR, True)
        self._set_capture_ui(False)
        self._refresh_quick_state()

    def _capture_clicked(self, target: str, quick: bool = False) -> None:
        if self.capture_in_progress:
            messagebox.showwarning("Captura en curso", "Espera a que termine la lectura actual.")
            return
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("Analisis en curso", "Espera a que termine la comparacion antes de capturar.")
            return
        if self.sensor_connection is None:
            messagebox.showwarning("Sensor no conectado", "Conecta el sensor antes de capturar.")
            return

        self._set_capture_ui(False)
        self._set_sensor_ui("Capturando...", "●  Capturando...", COLOR_WARN, False)
        self.run_button.configure(state="disabled")
        stop_event = threading.Event() if quick else None
        self.quick_capture_stop_event = stop_event
        self.capture_in_progress = True
        self._refresh_quick_state()
        if quick:
            self.quick_stop_button.configure(state="normal")
            self.quick_capture_status.configure(text="Leyendo BASE..." if target == "base" else "Leyendo DESPUES...")
            self._refresh_quick_state()
        label = "BASE" if target == "base" else "DESPUES"
        self._log(f"Capturando nube {label}..." if not quick else f"Capturando nube {label} durante 15 s...")

        def worker():
            try:
                if quick:
                    cloud = aurora_sensor.capture_snapshot(
                        self.sensor_connection,
                        duration_s=15.0,
                        persistence_ratio=0.0,
                        stop_event=stop_event,
                        max_distance_m=None,
                        cone_angle_deg=None,
                        forward_axis="z",
                    )
                    try:
                        ref_image, ref_point_grid = aurora_sensor.capture_reference_frame(self.sensor_connection)
                    except Exception:
                        ref_image, ref_point_grid = None, None
                    try:
                        ref_pose = aurora_sensor.get_current_pose(self.sensor_connection)
                    except Exception:
                        ref_pose = None
                    self.root.after(
                        0,
                        lambda: self._on_capture_done(
                            target, cloud, quick=True, ref_image=ref_image, ref_point_grid=ref_point_grid, ref_pose=ref_pose
                        ),
                    )
                else:
                    cloud = aurora_sensor.capture_snapshot(self.sensor_connection)
                    self.root.after(0, lambda: self._on_capture_done(target, cloud))
            except Exception as exc:
                self.root.after(0, lambda exc=exc: self._on_capture_failed(exc, quick=quick))

        threading.Thread(target=worker, daemon=True).start()

    def _stop_quick_capture(self) -> None:
        if self.quick_capture_stop_event is not None:
            self.quick_capture_stop_event.set()
            self.quick_stop_button.configure(state="disabled")
            self.quick_capture_status.configure(text="Deteniendo lectura...")

    def _on_capture_done(
        self, target: str, cloud, quick: bool = False, ref_image=None, ref_point_grid=None, ref_pose=None
    ) -> None:
        self.quick_capture_stop_event = None
        self.capture_in_progress = False
        self.quick_stop_button.configure(state="disabled")
        self._set_sensor_ui(
            "Desconectar" if self.sensor_connection is not None else "Conectar",
            "●  Conectado" if self.sensor_connection is not None else "●  Desconectado",
            COLOR_OK if self.sensor_connection is not None else COLOR_ERROR,
            True,
        )
        self.run_button.configure(state="normal")
        self._set_capture_ui(self.sensor_connection is not None)

        import datetime

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        default_name = f"base_capturada_{timestamp}.ply" if target == "base" else f"updated_capturada_{timestamp}.ply"
        default_dir = PROJECT_ROOT / "data"
        default_dir.mkdir(parents=True, exist_ok=True)
        path = str(default_dir / default_name) if quick else filedialog.asksaveasfilename(
            title="Guardar captura como",
            initialdir=str(default_dir),
            initialfile=default_name,
            defaultextension=".ply",
            filetypes=[("PLY files", "*.ply")],
        )
        if not path:
            self._log("Captura descartada (no se eligio archivo de destino).")
            return

        import open3d as o3d

        o3d.io.write_point_cloud(path, cloud)
        self._log(f"Captura guardada en: {path} ({len(cloud.points)} puntos)")
        if quick:
            if ref_image is None or ref_point_grid is None:
                try:
                    ref_image, ref_point_grid = render_reference_photo_from_cloud(cloud)
                except Exception as exc:
                    self._log(f"No se pudo generar la foto de referencia: {exc}")
                    ref_image, ref_point_grid = None, None
            if ref_image is not None and ref_point_grid is not None:
                try:
                    save_reference_photo(Path(path), ref_image, ref_point_grid)
                except Exception as exc:
                    self._log(f"No se pudo guardar la foto de referencia: {exc}")
            if ref_pose is not None:
                try:
                    save_reference_pose(Path(path), ref_pose.position, ref_pose.rpy_deg)
                except Exception as exc:
                    self._log(f"No se pudo guardar la pose IMU: {exc}")
            else:
                self._log("No se pudo leer la pose IMU de esta captura.")
        if target == "base":
            self.base_path.set(path)
            if quick:
                self.quick_base_path = path
                self.updated_path.set("")
                self.quick_updated_path = None
                self.quick_stage_results.clear()
                self.quick_pending_stage = None
                self.quick_session_summary_path = (
                    Path(self.output_dir.get())
                    / "etapas"
                    / f"sesion_{Path(path).stem}"
                    / "resumen_etapas.csv"
                )
        else:
            self.updated_path.set(path)
            if quick:
                self.quick_updated_path = path
        self._refresh_quick_state()
        if quick:
            self.quick_capture_status.configure(text=f"Lectura guardada: {Path(path).name}")

    def _on_capture_failed(self, exc: Exception, quick: bool = False) -> None:
        self.quick_capture_stop_event = None
        self.capture_in_progress = False
        self.quick_stop_button.configure(state="disabled")
        self._set_sensor_ui(
            "Desconectar" if self.sensor_connection is not None else "Conectar",
            "●  Conectado" if self.sensor_connection is not None else "●  Desconectado",
            COLOR_OK if self.sensor_connection is not None else COLOR_ERROR,
            True,
        )
        self.run_button.configure(state="normal")
        self._set_capture_ui(self.sensor_connection is not None)
        if quick:
            self.quick_capture_status.configure(text="La lectura fallo.")
            self._refresh_quick_state()
        messagebox.showerror("Error de captura", str(exc))

    # ---------------------------------------------------------------- Crop

    def _quick_compare_clicked(self) -> None:
        if self.capture_in_progress:
            messagebox.showwarning("Captura en curso", "Espera a que termine la lectura antes de comparar.")
            return
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("Analisis en curso", "Espera a que termine la comparacion actual.")
            return
        if not self.quick_base_path or not self.quick_updated_path:
            messagebox.showwarning("Faltan capturas", "Captura una BASE nueva y al menos un DESPUES.")
            return
        base_path, updated_path = Path(self.quick_base_path), Path(self.quick_updated_path)
        if not base_path.is_file() or not updated_path.is_file():
            messagebox.showwarning("Faltan capturas", "No se encuentran los archivos BASE y DESPUES.")
            return
        try:
            base_pose = load_reference_pose(base_path)
            updated_pose = load_reference_pose(updated_path)
        except Exception as exc:
            messagebox.showwarning("Pose IMU invalida", f"No se pudieron leer las posiciones del sensor: {exc}")
            return
        if base_pose is None or updated_pose is None:
            messagebox.showwarning(
                "No se pudo verificar la posicion",
                "Falta la pose IMU en una captura. Vuelve a capturar con el sensor conectado.",
            )
            return
        position_error_cm = float(np.linalg.norm(base_pose[0] - updated_pose[0]) * 100.0)
        rotation_error = ((base_pose[1] - updated_pose[1] + 180.0) % 360.0) - 180.0
        rotation_error_deg = float(np.max(np.abs(rotation_error)))
        if (
            position_error_cm > self.QUICK_POSITION_TOLERANCE_CM
            or rotation_error_deg > self.QUICK_ROTATION_TOLERANCE_DEG
        ):
            self.quick_result_label.configure(
                text=(
                    f"No se comparo: DESPUES esta a {position_error_cm:.1f} cm y {rotation_error_deg:.1f} grados "
                    "de la pose BASE. Vuelve al mismo lugar y captura DESPUES otra vez."
                )
            )
            messagebox.showwarning(
                "Sensor fuera de posicion",
                "Vuelve el sensor a la posicion y orientacion BASE antes de capturar de nuevo. "
                f"Diferencia actual: {position_error_cm:.1f} cm, {rotation_error_deg:.1f} grados.",
            )
            return
        self.quick_pending_stage = self.quick_updated_path
        self._run_pipeline_clicked(quick=True)

    def _save_quick_session_summary(self) -> None:
        if self.quick_session_summary_path is None:
            return
        summary_path = self.quick_session_summary_path
        temp_path = summary_path.with_suffix(".tmp")
        try:
            summary_path.parent.mkdir(parents=True, exist_ok=True)
            with temp_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    [
                        "etapa",
                        "captura_despues",
                        "espesor_medio_cm",
                        "espesor_mediano_cm",
                        "percentil_95_cm",
                        "puntos",
                        "directorio_resultados",
                    ]
                )
                for index, (capture_path, mean_cm, median_cm, p95_cm, n_points, output_dir) in enumerate(
                    self.quick_stage_results, start=1
                ):
                    writer.writerow(
                        [index, capture_path, f"{mean_cm:.4f}", f"{median_cm:.4f}", f"{p95_cm:.4f}", n_points, output_dir]
                    )
            temp_path.replace(summary_path)
            self._log(f"Resumen de etapas guardado en: {summary_path}")
        except OSError as exc:
            self._log(f"No se pudo guardar el resumen de etapas: {exc}")
        finally:
            temp_path.unlink(missing_ok=True)

    def _pick_crop_interactively(self) -> None:
        base_file = Path(self.base_path.get())
        if not base_file.exists():
            messagebox.showerror("Error", f"No se encontro la nube base:\n{base_file}")
            return
        try:
            margin = float(self.crop_margin.get() or 0.08)
            cloud = load_point_cloud(base_file)
            bounds = pick_crop_bounds(cloud, margin=margin)
        except Exception as exc:
            messagebox.showerror("Error al abrir el visor 3D", str(exc))
            return

        if bounds is None:
            messagebox.showinfo(
                "Sin seleccion", "No se eligieron al menos 2 puntos. Repite usando Shift+Click sobre 2 esquinas."
            )
            return

        crop_min, crop_max = bounds
        self.crop_min.set(f"{crop_min[0]:.4f} {crop_min[1]:.4f} {crop_min[2]:.4f}")
        self.crop_max.set(f"{crop_max[0]:.4f} {crop_max[1]:.4f} {crop_max[2]:.4f}")
        self.use_crop.set(True)
        messagebox.showinfo("Recorte definido", "Se cargaron los limites min/max a partir de los puntos elegidos.")

    # ---------------------------------------------------------- Pipeline

    def _run_pipeline_clicked(self, quick: bool = False) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("En progreso", "Ya hay una comparacion en ejecucion.")
            return
        if self.capture_in_progress:
            messagebox.showwarning("Captura en curso", "Espera a que termine la lectura antes de comparar.")
            return

        try:
            params = self._build_params(quick=quick)
        except Exception as exc:
            messagebox.showerror("Parametros invalidos", str(exc))
            return

        self.log_text.delete("1.0", "end")
        self.run_button.configure(state="disabled")
        self.view_heatmap_button.configure(state="disabled")
        self._set_capture_ui(False)
        if quick:
            self.quick_compare_button.configure(state="disabled")
            self.quick_result_label.configure(text="Comparando; se asume la misma posicion y orientacion del sensor.")

        self.worker_thread = threading.Thread(target=self._run_pipeline_worker, args=(params, quick), daemon=True)
        self.worker_thread.start()
        self._refresh_quick_state()

    def _build_params(self, quick: bool = False) -> PipelineParams:
        base_path = Path(self.quick_base_path) if quick else Path(self.base_path.get())
        updated_path = Path(self.quick_updated_path) if quick else Path(self.updated_path.get())
        output_dir = Path(self.output_dir.get())

        if quick:
            return PipelineParams(
                base_path=base_path,
                updated_path=updated_path,
                output_dir=output_dir / "etapas" / updated_path.stem,
                voxel_size=0.0,
                remove_outliers=False,
                use_icp=False,
                icp_threshold=0.05,
                crop_min=None,
                crop_max=None,
                max_distance=None,
            )

        crop_min = crop_max = None
        if self.use_crop.get():
            crop_min = self._parse_xyz(self.crop_min.get(), "Min")
            crop_max = self._parse_xyz(self.crop_max.get(), "Max")

        max_distance = float(self.max_distance.get()) if self.max_distance.get().strip() else None

        return PipelineParams(
            base_path=base_path,
            updated_path=updated_path,
            output_dir=output_dir,
            voxel_size=float(self.voxel_size.get() or 0.0),
            remove_outliers=self.remove_outliers.get(),
            use_icp=self.use_icp.get(),
            icp_threshold=float(self.icp_threshold.get() or 0.05),
            crop_min=crop_min,
            crop_max=crop_max,
            max_distance=max_distance,
        )

    def _run_pipeline_worker(self, params: PipelineParams, quick: bool = False) -> None:
        try:
            self.result = run_pipeline(params, log=self._log)
            self._log("\nListo.")
            self.root.after(0, lambda: self.view_heatmap_button.configure(state="normal"))
            self.root.after(0, self._push_static_result_to_viewer)
            if quick:
                self.root.after(0, self._on_quick_pipeline_success)
        except Exception as exc:
            self._log(f"\nERROR: {exc}")
            if quick:
                self.root.after(0, lambda exc=exc: self._on_quick_pipeline_failure(str(exc)))
            else:
                self.root.after(0, lambda exc=exc: messagebox.showerror("Error durante el procesamiento", str(exc)))
        finally:
            self.root.after(0, self._finish_pipeline_ui)

    def _on_quick_pipeline_success(self) -> None:
        stats = self.result.stats
        if self.quick_pending_stage:
            stage = self.quick_pending_stage
            self.quick_stage_results = [row for row in self.quick_stage_results if row[0] != stage]
            self.quick_stage_results.append(
                (
                    stage,
                    stats.mean * 100.0,
                    stats.median * 100.0,
                    stats.p95 * 100.0,
                    stats.n_points,
                    str(self.result.csv_path.parent),
                )
            )
            lines = ["Espesor acumulado respecto de BASE:"]
            for index, (path, mean_cm, median_cm, _p95_cm, _n_points, _output_dir) in enumerate(
                self.quick_stage_results, start=1
            ):
                lines.append(f"{index}. {Path(path).name}: media {mean_cm:.2f} cm; mediana {median_cm:.2f} cm")
            self.quick_result_label.configure(text="\n".join(lines))
            self._save_quick_session_summary()
            self.quick_pending_stage = None

    def _on_quick_pipeline_failure(self, message: str) -> None:
        self.quick_pending_stage = None
        self.quick_result_label.configure(text=f"La comparacion fallo: {message}")
        messagebox.showerror("Error durante la comparacion", message)

    def _finish_pipeline_ui(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            self.root.after(50, self._finish_pipeline_ui)
            return
        self.run_button.configure(state="normal")
        self._refresh_quick_state()

    def _view_heatmap(self) -> None:
        if not self.result:
            return
        visualize(self.result.base_cloud, self.result.heatmap_cloud, show_overlay=True)

    # --------------------------------------------------------------- Viewer

    def _band_thresholds_m(self) -> tuple[float, float]:
        low_mm = float(self.band_low_mm.get() or 50)
        high_mm = float(self.band_high_mm.get() or 100)
        return low_mm / 1000.0, high_mm / 1000.0

    def _open_viewer(self) -> None:
        base_file = Path(self.base_path.get())
        if not base_file.exists():
            messagebox.showerror("Error", f"No se encontro la nube base:\n{base_file}")
            return

        if self.viewer is not None and self.viewer.is_running():
            messagebox.showinfo("Vista 3D", "La vista 3D ya esta abierta.")
            return

        try:
            base_cloud = self.result.base_cloud if self.result is not None else load_point_cloud(base_file)
        except Exception as exc:
            messagebox.showerror("Error", str(exc))
            return

        self.viewer = LiveViewer(base_cloud, log=self._log)
        self.viewer.start()
        self.close_viewer_button.configure(state="normal")
        self._apply_viewer_settings()
        self._log("Vista 3D abierta.")

    def _close_viewer(self) -> None:
        if self.viewer is not None:
            self.viewer.stop()
            self.viewer = None
        self.close_viewer_button.configure(state="disabled")
        self._clear_stream_ui()

    def _clear_stream_ui(self) -> None:
        self.stream_toggle_button.configure(text="Iniciar transmision")
        self.stream_url_var.set("")
        self.stream_qr_label.configure(image=None)
        self.stream_qr_label._qr_image_ref = None

    def _toggle_stream(self) -> None:
        if self.viewer is not None and self.viewer.is_streaming():
            self.viewer.stop_stream()
            self._clear_stream_ui()
            self._log("Transmision al celular detenida.")
            return

        if self.viewer is None or not self.viewer.is_running():
            self._open_viewer()
            if self.viewer is None:
                return

        try:
            url = self.viewer.start_stream()
        except Exception as exc:
            messagebox.showerror("Error al iniciar la transmision", str(exc))
            return

        self.stream_toggle_button.configure(text="Detener transmision")
        self.stream_url_var.set(f"Escanea con el celular (misma red WiFi): {url}")
        qr_img = make_qr_image(url).resize((160, 160))
        ctk_qr = ctk.CTkImage(light_image=qr_img, dark_image=qr_img, size=(160, 160))
        self.stream_qr_label.configure(image=ctk_qr, text="")
        self.stream_qr_label._qr_image_ref = ctk_qr  # evita que el GC recolecte la imagen
        self._log(f"Transmision al celular activa en: {url}")

    def _apply_viewer_settings(self) -> None:
        if self.viewer is None:
            return
        try:
            low_m, high_m = self._band_thresholds_m()
        except ValueError:
            low_m, high_m = 0.05, 0.10
        max_distance = float(self.max_distance.get()) if self.max_distance.get().strip() else None

        self.viewer.set_color_mode(self.color_mode.get(), low_m, high_m, max_distance)
        self.viewer.set_show_updated(self.show_updated.get())

        if self.updated_source.get() == "live":
            self.viewer.set_live_sensor(self.sensor_connection)
        else:
            self.viewer.set_live_sensor(None)
            self._push_static_result_to_viewer()

    def _push_static_result_to_viewer(self) -> None:
        if self.viewer is None or self.updated_source.get() != "static":
            return
        try:
            if self.result is not None:
                points = np.asarray(self.result.updated_cloud.points)
            else:
                updated_file = Path(self.updated_path.get())
                if not updated_file.exists():
                    return
                points = np.asarray(load_point_cloud(updated_file).points)
        except Exception as exc:
            self._log(f"No se pudo cargar la nube actualizada para la vista 3D: {exc}")
            return
        self.viewer.push_static_points(points)

    def _on_show_updated_changed(self) -> None:
        if self.viewer is not None:
            self.viewer.set_show_updated(self.show_updated.get())

    def _on_updated_source_changed(self) -> None:
        self._apply_viewer_settings()

    def _on_close(self) -> None:
        self._close_viewer()
        if self.sensor_connection is not None:
            try:
                aurora_sensor.disconnect(self.sensor_connection)
            except Exception:
                pass
        self.root.destroy()


def main() -> None:
    root = ctk.CTk()
    AuroraGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
