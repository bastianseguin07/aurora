"""
GUI de escritorio (CustomTkinter, sobre tkinter) para comparar dos nubes de
puntos .ply sin usar la consola. Incluye recorte (crop) manual o
seleccionado visualmente en un visor 3D, captura desde el sensor Aurora,
coloreado por espesor (continuo o 3 niveles) y vista 3D en vivo o estatica.

Ejecutar con:
    python gui.py
"""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox

import customtkinter as ctk
import numpy as np

import aurora_sensor
from live_viewer import LiveViewer
from pointcloud_core import PipelineParams, pick_crop_bounds, run_pipeline, visualize, load_point_cloud

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

        self.color_mode = tk.StringVar(value="continuous")
        self.band_low_mm = tk.StringVar(value="20")
        self.band_high_mm = tk.StringVar(value="50")

        self.show_updated = tk.BooleanVar(value=True)
        self.updated_source = tk.StringVar(value="static")  # "static" o "live"

        self.sensor_address = tk.StringVar(value="192.168.11.1")
        self.live_max_distance = tk.StringVar(value="5.0")
        self.live_cone_angle = tk.StringVar(value="90")
        self.live_forward_axis = tk.StringVar(value="z")
        self.live_invert_y = tk.BooleanVar(value=True)
        self.live_invert_z = tk.BooleanVar(value=True)
        self.sensor_connection = None

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
        self.tabview.add("Datos y sensor")
        self.tabview.add("Procesamiento")
        self.tabview.add("Visualizacion")
        self.tabview.configure(height=330)

        self._build_files_tab(self.tabview.tab("Datos y sensor"))
        self._build_processing_tab(self.tabview.tab("Procesamiento"))
        self._build_visualization_tab(self.tabview.tab("Visualizacion"))

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
        self.start_live_capture_button = ctk.CTkButton(
            row,
            text="Iniciar captura en tiempo real",
            command=self._start_live_capture_clicked,
            state="disabled",
        )
        self.start_live_capture_button.pack(side="left")
        self.capture_base_button = ctk.CTkButton(
            row, text="Capturar nube BASE", command=lambda: self._capture_clicked("base"), state="disabled"
        )
        self.capture_base_button.pack(side="left", padx=8)
        self.capture_updated_button = ctk.CTkButton(
            row,
            text="Capturar nube ACTUALIZADA",
            command=lambda: self._capture_clicked("updated"),
            state="disabled",
        )
        self.capture_updated_button.pack(side="left", padx=8)

        row = self._row(sensor_frame, pady=(0, 12))
        self.capture_live_baseline_button = ctk.CTkButton(
            row,
            text="Fijar BASE en vivo (MVP)",
            command=self._capture_live_baseline_clicked,
            state="disabled",
        )
        self.capture_live_baseline_button.pack(side="left", padx=8)
        self.clear_live_baseline_button = ctk.CTkButton(
            row,
            text="Quitar BASE en vivo",
            command=self._clear_live_baseline_clicked,
            state="disabled",
        )
        self.clear_live_baseline_button.pack(side="left", padx=8)
        self.save_live_clouds_button = ctk.CTkButton(
            row,
            text="Guardar nubes en vivo",
            command=self._save_live_clouds_clicked,
            state="disabled",
        )
        self.save_live_clouds_button.pack(side="left", padx=8)

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

        live_frame = self._section(parent, "Ajustes live (MVP)", fill="x")
        row = self._row(live_frame)
        ctk.CTkLabel(row, text="Distancia maxima (m):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.live_max_distance, width=80).pack(side="left", padx=(8, 16))
        ctk.CTkLabel(row, text="Cono (grados):", font=FONT_BODY).pack(side="left")
        ctk.CTkEntry(row, textvariable=self.live_cone_angle, width=80).pack(side="left", padx=8)

        row = self._row(live_frame, pady=(0, 12))
        ctk.CTkLabel(row, text="Eje frontal:", font=FONT_BODY).pack(side="left")
        ctk.CTkOptionMenu(row, values=["x", "y", "z"], variable=self.live_forward_axis).pack(side="left", padx=(8, 16))
        ctk.CTkCheckBox(row, text="Invertir Y", variable=self.live_invert_y).pack(side="left")
        ctk.CTkCheckBox(row, text="Invertir Z", variable=self.live_invert_z).pack(side="left", padx=12)

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

    def _toggle_sensor_connection(self) -> None:
        if self.sensor_connection is not None:
            self._disconnect_sensor()
            return

        address = self.sensor_address.get().strip()
        self.connect_button.configure(state="disabled")
        self.sensor_status_label.configure(text="●  Conectando...", text_color=COLOR_WARN)

        def worker():
            try:
                connection = aurora_sensor.connect(address)
                self.root.after(0, lambda: self._on_sensor_connected(connection))
            except Exception as exc:
                self.root.after(0, lambda exc=exc: self._on_sensor_connect_failed(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_sensor_connected(self, connection) -> None:
        self.sensor_connection = connection
        self.connect_button.configure(text="Desconectar", state="normal")
        self.sensor_status_label.configure(text="●  Conectado", text_color=COLOR_OK)
        self.capture_base_button.configure(state="normal")
        self.capture_updated_button.configure(state="normal")
        self.start_live_capture_button.configure(state="normal")
        self.capture_live_baseline_button.configure(state="normal")
        self.clear_live_baseline_button.configure(state="normal")
        self.save_live_clouds_button.configure(state="normal")
        if self.viewer is not None and self.updated_source.get() == "live":
            self.viewer.set_live_sensor(connection)
            self._apply_viewer_settings()
        self._log(f"Conectado al sensor Aurora en {self.sensor_address.get()}.")

    def _on_sensor_connect_failed(self, exc: Exception) -> None:
        self.connect_button.configure(state="normal")
        self.sensor_status_label.configure(text="●  Desconectado", text_color=COLOR_ERROR)
        messagebox.showerror("Error de conexion", str(exc))

    def _disconnect_sensor(self) -> None:
        if self.viewer is not None:
            self.viewer.set_live_sensor(None)
        try:
            aurora_sensor.disconnect(self.sensor_connection)
        except Exception as exc:
            self._log(f"Aviso al desconectar: {exc}")
        self.sensor_connection = None
        self.connect_button.configure(text="Conectar")
        self.sensor_status_label.configure(text="●  Desconectado", text_color=COLOR_ERROR)
        self.capture_base_button.configure(state="disabled")
        self.capture_updated_button.configure(state="disabled")
        self.start_live_capture_button.configure(state="disabled")
        self.capture_live_baseline_button.configure(state="disabled")
        self.clear_live_baseline_button.configure(state="disabled")
        self.save_live_clouds_button.configure(state="disabled")

    def _capture_clicked(self, target: str) -> None:
        if self.sensor_connection is None:
            messagebox.showwarning("Sensor no conectado", "Conecta el sensor antes de capturar.")
            return

        self.capture_base_button.configure(state="disabled")
        self.capture_updated_button.configure(state="disabled")
        self._log(f"Capturando nube ({'base' if target == 'base' else 'actualizada'})...")

        def worker():
            try:
                cloud = aurora_sensor.capture_snapshot(self.sensor_connection)
                self.root.after(0, lambda: self._on_capture_done(target, cloud))
            except Exception as exc:
                self.root.after(0, lambda exc=exc: self._on_capture_failed(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_capture_done(self, target: str, cloud) -> None:
        self.capture_base_button.configure(state="normal")
        self.capture_updated_button.configure(state="normal")

        default_name = "base_capturada.ply" if target == "base" else "updated_capturada.ply"
        default_dir = PROJECT_ROOT / "data"
        default_dir.mkdir(parents=True, exist_ok=True)
        path = filedialog.asksaveasfilename(
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
        if target == "base":
            self.base_path.set(path)
        else:
            self.updated_path.set(path)

    def _on_capture_failed(self, exc: Exception) -> None:
        self.capture_base_button.configure(state="normal")
        self.capture_updated_button.configure(state="normal")
        messagebox.showerror("Error de captura", str(exc))

    def _capture_live_baseline_clicked(self) -> None:
        if self.sensor_connection is None:
            messagebox.showwarning("Sensor no conectado", "Conecta el sensor antes de fijar la base en vivo.")
            return
        if not self._ensure_live_capture_running():
            return
        try:
            count = self.viewer.capture_live_baseline()
        except RuntimeError as exc:
            messagebox.showinfo("Frame aun no disponible", str(exc))
            return

        self._log(f"BASE en vivo fijada con {count} puntos. La comparacion continua se actualiza en tiempo real.")

    def _clear_live_baseline_clicked(self) -> None:
        if self.viewer is None or not self.viewer.is_running():
            messagebox.showinfo("Vista 3D cerrada", "Abre primero la vista 3D para quitar la base en vivo.")
            return
        self.viewer.clear_live_baseline()
        self._log("BASE en vivo quitada. Se restauro la nube base original.")

    def _start_live_capture_clicked(self) -> None:
        if self.sensor_connection is None:
            messagebox.showwarning("Sensor no conectado", "Conecta el sensor antes de iniciar captura en tiempo real.")
            return
        if not self._ensure_live_capture_running():
            return
        self._log("Captura en tiempo real activa en la vista 3D.")

    def _save_live_clouds_clicked(self) -> None:
        if not self._ensure_live_capture_running():
            return

        baseline_cloud, current_cloud, using_live_baseline = self.viewer.get_live_snapshot_clouds()
        if baseline_cloud is None or current_cloud is None:
            messagebox.showinfo("Sin frame disponible", "Aun no hay nube en vivo para guardar.")
            return

        target_dir = filedialog.askdirectory(title="Seleccionar carpeta para guardar nubes en vivo")
        if not target_dir:
            self._log("Guardado de nubes en vivo cancelado.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        baseline_name = "base_live_fijada" if using_live_baseline else "base_referencia_original"
        base_path = Path(target_dir) / f"{baseline_name}_{timestamp}.ply"
        updated_path = Path(target_dir) / f"updated_live_{timestamp}.ply"

        import open3d as o3d

        o3d.io.write_point_cloud(str(base_path), baseline_cloud)
        o3d.io.write_point_cloud(str(updated_path), current_cloud)
        self._log(f"Nubes en vivo guardadas en: {base_path} y {updated_path}")

    def _ensure_live_capture_running(self) -> bool:
        self.updated_source.set("live")
        if self.viewer is None or not self.viewer.is_running():
            self._open_viewer()
            if self.viewer is None:
                return False
        self._apply_viewer_settings()
        self.viewer.set_live_sensor(self.sensor_connection)
        return True

    # ---------------------------------------------------------------- Crop

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

    def _run_pipeline_clicked(self) -> None:
        if self.worker_thread and self.worker_thread.is_alive():
            messagebox.showwarning("En progreso", "Ya hay una comparacion en ejecucion.")
            return

        try:
            params = self._build_params()
        except Exception as exc:
            messagebox.showerror("Parametros invalidos", str(exc))
            return

        self.log_text.delete("1.0", "end")
        self.run_button.configure(state="disabled")
        self.view_heatmap_button.configure(state="disabled")

        self.worker_thread = threading.Thread(target=self._run_pipeline_worker, args=(params,), daemon=True)
        self.worker_thread.start()

    def _build_params(self) -> PipelineParams:
        base_path = Path(self.base_path.get())
        updated_path = Path(self.updated_path.get())
        output_dir = Path(self.output_dir.get())

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

    def _run_pipeline_worker(self, params: PipelineParams) -> None:
        try:
            self.result = run_pipeline(params, log=self._log)
            self._log("\nListo.")
            self.root.after(0, lambda: self.view_heatmap_button.configure(state="normal"))
            self.root.after(0, self._push_static_result_to_viewer)
        except Exception as exc:
            self._log(f"\nERROR: {exc}")
            self.root.after(0, lambda exc=exc: messagebox.showerror("Error durante el procesamiento", str(exc)))
        finally:
            self.root.after(0, lambda: self.run_button.configure(state="normal"))

    def _view_heatmap(self) -> None:
        if not self.result:
            return
        visualize(self.result.base_cloud, self.result.heatmap_cloud, show_overlay=True)

    # --------------------------------------------------------------- Viewer

    def _band_thresholds_m(self) -> tuple[float, float]:
        low_mm = float(self.band_low_mm.get() or 20)
        high_mm = float(self.band_high_mm.get() or 50)
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

        self.viewer = LiveViewer(base_cloud)
        self.viewer.start()
        self.close_viewer_button.configure(state="normal")
        self._apply_viewer_settings()
        self._log("Vista 3D abierta.")

    def _close_viewer(self) -> None:
        if self.viewer is not None:
            self.viewer.stop()
            self.viewer = None
        self.close_viewer_button.configure(state="disabled")

    def _apply_viewer_settings(self) -> None:
        if self.viewer is None:
            return
        try:
            low_m, high_m = self._band_thresholds_m()
        except ValueError:
            low_m, high_m = 0.02, 0.05
        max_distance = float(self.max_distance.get()) if self.max_distance.get().strip() else None

        self.viewer.set_color_mode(self.color_mode.get(), low_m, high_m, max_distance)
        self.viewer.set_show_updated(self.show_updated.get())
        try:
            live_max_distance, live_cone_angle = self._parse_live_filter_params()
        except ValueError as exc:
            self._log(f"Parametros live invalidos: {exc}. Se usan valores por defecto.")
            live_max_distance, live_cone_angle = None, None
        self.viewer.set_live_filters(
            max_distance_m=live_max_distance,
            cone_angle_deg=live_cone_angle,
            forward_axis=self.live_forward_axis.get(),
            invert_y=self.live_invert_y.get(),
            invert_z=self.live_invert_z.get(),
        )

        if self.updated_source.get() == "live":
            self.viewer.set_live_sensor(self.sensor_connection)
        else:
            self.viewer.set_live_sensor(None)
            self._push_static_result_to_viewer()

    def _parse_live_filter_params(self) -> tuple[float | None, float | None]:
        max_distance = None
        if self.live_max_distance.get().strip():
            max_distance = float(self.live_max_distance.get())
            if max_distance <= 0:
                raise ValueError("la distancia maxima debe ser > 0")

        cone_angle = None
        if self.live_cone_angle.get().strip():
            cone_angle = float(self.live_cone_angle.get())
            if cone_angle <= 0 or cone_angle > 180:
                raise ValueError("el cono debe estar en (0, 180]")

        return max_distance, cone_angle

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
