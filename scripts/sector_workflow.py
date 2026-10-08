"""GTK operator flow: one sector, immutable baseline, resumable stages."""
from __future__ import annotations

import threading
from pathlib import Path

import numpy as np
from gi.repository import Gtk, Pango

from measurement_session import MeasurementSession
from pointcloud_core import show_point_cloud


class SectorWorkflowMixin:
    def _build_simple_workflow_page(self):
        self.measurement_session = None
        self._session_importing = False
        self._session_alignment_target = None
        self._session_roi_target = None
        self.session_result = None
        page = self._new_page()
        page.get_style_context().add_class("sector-flow")
        intro = Gtk.Label(label="Compara dos capturas: una de ANTES y otra de DESPUES del shotcrete. Puedes cargar archivos o capturar con el sensor.", xalign=0)
        intro.set_line_wrap(True)
        page.pack_start(intro, False, False, 8)

        row = self._row(page)
        self.quick_demo_check = Gtk.CheckButton(label="Estoy usando datos de prueba")
        row.pack_start(self.quick_demo_check, False, False, 0)
        self.session_open_button = Gtk.Button(label="Abrir comparacion guardada...")
        self.session_open_button.connect("clicked", lambda _b: self._session_open())
        row.pack_end(self.session_open_button, False, False, 0)

        frame, box = self._section("Sensor")
        row = self._row(box)
        self.quick_sensor_address_entry = Gtk.Entry()
        self.quick_sensor_address_entry.set_text(self.sensor_address_entry.get_text())
        self.quick_sensor_address_entry.set_width_chars(16)
        row.pack_start(self.quick_sensor_address_entry, False, False, 0)
        self.quick_connect_button = Gtk.Button(label="Conectar")
        self.quick_connect_button.connect("clicked", lambda _b: self._quick_connect_clicked())
        row.pack_start(self.quick_connect_button, False, False, 0)
        self.quick_sensor_status_label = Gtk.Label(label="Desconectado", xalign=0)
        row.pack_start(self.quick_sensor_status_label, False, False, 0)
        self.quick_sensor_note = Gtk.Label(label="Cada captura dura 15 s. Mantén el sensor quieto durante la lectura.", xalign=0)
        box.pack_start(self.quick_sensor_note, False, False, 0)
        page.pack_start(frame, False, True, 0)

        pair = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        for target, title in (("base", "ANTES del shotcrete"), ("updated", "DESPUES del shotcrete")):
            frame, box = self._section(title)
            actions = self._row(box)
            button = Gtk.Button(label=f"Capturar {title.split()[0].lower()}")
            button.connect("clicked", lambda _b, target=target: self._quick_capture_clicked(target))
            setattr(self, f"quick_capture_{target}_button", button)
            actions.pack_start(button, True, True, 0)
            button = Gtk.Button(label=f"Cargar {title.split()[0].lower()}...")
            button.connect("clicked", lambda _b, target=target: self._session_import(target))
            setattr(self, f"session_import_{target}_button", button)
            actions.pack_start(button, True, True, 0)
            path_label = Gtk.Label(label="Sin archivo cargado", xalign=0)
            path_label.set_ellipsize(Pango.EllipsizeMode.MIDDLE)
            path_label.set_tooltip_text("Archivo seleccionado para esta captura")
            setattr(self, f"quick_{target}_path_label", path_label)
            box.pack_start(path_label, False, False, 0)
            pair.pack_start(frame, True, True, 0)
        page.pack_start(pair, False, True, 0)

        self.session_stage_combo = Gtk.ComboBoxText()
        self.session_stage_combo.connect("changed", lambda _c: self._session_select_stage())
        self.session_mode_combo = Gtk.ComboBoxText()
        self.session_mode_combo.append_text("Respecto de ANTES")
        self.session_mode_combo.append_text("Respecto de etapa anterior")
        self.session_mode_combo.set_active(0)
        row = self._row(page)
        self.quick_stop_capture_button = Gtk.Button(label="Detener captura")
        self.quick_stop_capture_button.set_no_show_all(True)
        self.quick_stop_capture_button.connect("clicked", lambda _b: self._stop_capture_clicked())
        row.pack_start(self.quick_stop_capture_button, False, False, 0)
        self.quick_capture_status_label = Gtk.Label(label="Las capturas se guardan automáticamente.", xalign=0)
        self.quick_capture_status_label.set_line_wrap(True)
        self.quick_capture_status_label.set_selectable(True)
        row.pack_start(self.quick_capture_status_label, True, True, 0)

        frame, box = self._section("Comparar")
        note = Gtk.Label(label="Si moviste el sensor entre capturas, alinea las nubes desde la seccion Alineacion antes de comparar.", xalign=0)
        note.set_line_wrap(True)
        box.pack_start(note, False, False, 0)
        self.session_align_button = Gtk.Button(label="Alinear etapa con referencias estables")
        self.session_align_button.connect("clicked", lambda _b: self._session_align_selected())
        row = self._row(box)
        row.pack_start(self.session_align_button, False, False, 0)
        self.quick_imu_button = Gtk.Button(label="Guia de pose (herramienta auxiliar)")
        self.quick_imu_button.connect("clicked", lambda _b: self._session_show_advanced("alineacion_imu"))
        self.session_surface_check = Gtk.CheckButton(label="Revise la misma zona en ambas nubes, sin maquinaria ni oclusiones visibles")
        self.session_surface_check.set_tooltip_text("Revision visual del operador; no calcula un porcentaje de cobertura ni certifica precision.")
        box.pack_start(self.session_surface_check, False, False, 0)
        self.quick_compare_button = Gtk.Button(label="Comparar antes y despues")
        self.quick_compare_button.get_style_context().add_class("suggested-action")
        self.quick_compare_button.connect("clicked", lambda _b: self._quick_compare_clicked())
        box.pack_start(self.quick_compare_button, False, False, 0)
        self.quick_result_panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.quick_result_panel.get_style_context().add_class("quick-result-panel")
        self.quick_result_label = Gtk.Label(label="Carga o captura las dos nubes para comenzar.", xalign=0)
        self.quick_result_label.set_line_wrap(True)
        self.quick_result_label.set_selectable(True)
        self.quick_result_panel.pack_start(self.quick_result_label, False, False, 0)
        self.quick_result_cards = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.quick_result_panel.pack_start(self.quick_result_cards, False, False, 0)
        box.pack_start(self.quick_result_panel, False, False, 0)
        row = self._row(box)
        self.session_view_button = Gtk.Button(label="Ver resultado 3D")
        self.session_view_button.connect("clicked", lambda _b: self._session_view())
        self.session_report_button = Gtk.Button(label="Abrir informe")
        self.session_report_button.connect("clicked", lambda _b: self._session_report())
        row.pack_start(self.session_view_button, False, False, 0)
        row.pack_start(self.session_report_button, False, False, 0)
        page.pack_start(frame, False, True, 0)
        self._refresh_quick_workflow_state()
        return self._scrolled(page)

    def _session_busy(self):
        return bool(self.capture_stop_event is not None or self._session_importing or getattr(self, "_sensor_connecting", False) or (self.worker_thread and self.worker_thread.is_alive()))

    def _session_new(self):
        if self._session_busy():
            return
        try:
            session = MeasurementSession.create(Path(self.output_dir) / "sesiones", "Comparacion rapida", "Par antes y despues", self.quick_demo_check.get_active())
            self._session_activate(session)
        except Exception as exc:
            self._show_error("No se pudo crear la sesion", str(exc))

    def _new_pair_session(self):
        session = MeasurementSession.create(
            Path(self.output_dir) / "sesiones", "Comparacion rapida",
            "Par antes y despues", self.quick_demo_check.get_active(),
        )
        self._session_activate(session)
        return session

    def _quick_capture_clicked(self, target):
        if self._session_busy():
            self._show_warning("Trabajo en curso", "Espera a que termine la captura o comparacion.")
            return
        if target == "base":
            try:
                self._new_pair_session()
            except Exception as exc:
                self._show_error("No se pudo iniciar la comparacion", str(exc))
                return
        elif not self.measurement_session or not self.measurement_session.data["base"]:
            self._show_warning("Falta el ANTES", "Primero captura o carga la nube de ANTES.")
            return
        self._capture_clicked(target, quick=True)

    def _session_open(self):
        if self._session_busy():
            return
        dialog = Gtk.FileChooserDialog(title="Abrir comparacion guardada", parent=self.window, action=Gtk.FileChooserAction.OPEN)
        dialog.add_buttons("Cancelar", Gtk.ResponseType.CANCEL, "Abrir", Gtk.ResponseType.OK)
        file_filter = Gtk.FileFilter()
        file_filter.add_pattern("*.json")
        file_filter.set_name("Comparacion Aurora (*.json)")
        dialog.add_filter(file_filter)
        path = dialog.get_filename() if dialog.run() == Gtk.ResponseType.OK else None
        dialog.destroy()
        if path:
            self._session_load(Path(path))

    def _session_load(self, path):
        try:
            self._session_activate(MeasurementSession.load(path))
        except Exception as exc:
            self._show_error("No se pudo abrir la comparacion", str(exc))

    def _session_activate(self, session):
        self.measurement_session = session
        self.session_result = None
        self._session_alignment_target = None
        self._session_roi_target = None
        self.session_surface_check.set_active(False)
        self.session_mode_combo.set_active(0)
        self.quick_demo_check.set_active(session.data["demo"])
        self.quick_capture_status_label.set_text(f"Comparacion guardada automaticamente · {session.path.parent}")
        self.session_stage_combo.remove_all()
        for index, capture in enumerate(session.data["stages"]):
            self.session_stage_combo.append_text(capture["source_name"])
        if session.data["stages"]:
            self.session_stage_combo.set_active(len(session.data["stages"]) - 1)
        self.quick_base_path_label.set_text(session.data["base"]["source_name"] if session.data["base"] else "Sin archivo cargado")
        selected = session.data["stages"][-1] if session.data["stages"] else None
        self.quick_updated_path_label.set_text(selected["source_name"] if selected else "Sin archivo cargado")
        self._session_select_stage()

    def _session_select_stage(self):
        session = self.measurement_session
        self._session_alignment_target = None
        self._session_roi_target = None
        self.session_surface_check.set_active(False)
        self.quick_base_path = str(session.resolve(session.data["base"]["path"])) if session and session.data["base"] else None
        index = self.session_stage_combo.get_active()
        self.quick_updated_path = str(session.resolve(session.data["stages"][index]["path"])) if session and index >= 0 else None
        if session and self.quick_base_path:
            self.base_path = self.quick_base_path
            self._set_path_label(self.base_path_label, self.base_path)
        if self.quick_updated_path:
            self.updated_path = self.quick_updated_path
            self._set_path_label(self.updated_path_label, self.updated_path)
        self.alignment_applied = False
        self._session_render_results()
        self._refresh_quick_workflow_state()

    def _refresh_quick_workflow_state(self):
        if not hasattr(self, "quick_compare_button"):
            return
        session = self.measurement_session
        busy = self._session_busy()
        has_base = bool(session and session.data["base"])
        has_stage = bool(session and self.session_stage_combo.get_active() >= 0)
        self._set_capture_buttons_sensitive(self.sensor_connection is not None and not busy)
        self.quick_capture_base_button.set_sensitive(self.sensor_connection is not None and not busy)
        self.quick_capture_updated_button.set_sensitive(has_base and self.sensor_connection is not None and not busy)
        self.session_import_base_button.set_sensitive(not busy)
        self.session_import_updated_button.set_sensitive(has_base and not busy)
        self.quick_compare_button.set_sensitive(has_stage and not busy)
        self.session_open_button.set_sensitive(not busy)
        selected = session.data["stages"][self.session_stage_combo.get_active()] if has_stage else None
        already_measured = bool(selected and any(selected["id"] in (r["stage_id"], r["reference_id"]) for r in session.data["results"]))
        self.session_align_button.set_sensitive(has_stage and not busy and not already_measured)
        self.session_surface_check.set_sensitive(has_stage and not busy)
        self.session_report_button.set_sensitive(bool(session) and not busy)
        self.session_view_button.set_sensitive(bool(session and session.data["results"]) and not busy)
        self.quick_stop_capture_button.set_sensitive(self.capture_stop_event is not None)
        self.quick_stop_capture_button.set_visible(self.capture_stop_event is not None)
        self.quick_demo_check.set_sensitive(not busy and not session)

    def _session_import(self, target):
        if self._session_busy():
            return
        if target == "updated" and (not self.measurement_session or not self.measurement_session.data["base"]):
            self._show_warning("Falta el ANTES", "Primero carga o captura la nube de ANTES.")
            return
        path = self._run_open_file_dialog()
        if path:
            if target == "base":
                try:
                    self._new_pair_session()
                except Exception as exc:
                    self._show_error("No se pudo iniciar la comparacion", str(exc))
                    return
            self._session_add_source(Path(path), target)

    def _session_add_source(self, path, target, ref_pose=None, captured=False):
        if self._session_busy() or not self.measurement_session:
            return
        session = self.measurement_session
        self._session_importing = True
        self.quick_capture_status_label.set_text("Verificando nube y guardando copia en la sesion...")
        self._refresh_quick_workflow_state()
        pose = None
        if ref_pose is not None:
            pose = {"position_m": np.asarray(ref_pose.position).tolist(), "rpy_deg": np.asarray(ref_pose.rpy_deg).tolist(),
                    "sensor_timestamp_ns": getattr(ref_pose, "timestamp_ns", None), "frame": "SLAM; nube en marco local de camara, extrinseca no aplicada"}
        settings = {"source": "sensor", "duration_s": 15, "persistence": 0, "forward_axis": "z", "motion_compensated": False} if captured else {"source": "archivo importado; parametros y fecha de captura desconocidos"}
        def worker():
            try:
                session.add_capture(path, base=target == "base", pose=pose,
                                    connection_id=getattr(self, "sensor_connection_id", None) if captured else None, settings=settings)
                self._ui(self._session_added, session)
            except Exception as exc:
                self._ui(self._session_import_failed, str(exc))
        threading.Thread(target=worker, daemon=True).start()

    def _session_added(self, session):
        self._session_importing = False
        self._session_activate(session)
        self.quick_capture_status_label.set_text(f"Archivo guardado automaticamente · {session.path.parent}")

    def _session_import_failed(self, message):
        self._session_importing = False
        self._refresh_quick_workflow_state()
        self._show_error("No se pudo guardar la captura", message)

    def _quick_compare_clicked(self):
        if self._session_busy():
            self._show_warning("Trabajo en curso", "Espera a que termine la captura o comparacion.")
            return
        session = self.measurement_session
        index = self.session_stage_combo.get_active()
        if not session or not session.data["base"] or index < 0:
            self._show_warning("Faltan capturas", "Carga o captura una nube de ANTES y otra de DESPUES.")
            return
        incremental = self.session_mode_combo.get_active() == 1
        surface_reviewed = self.session_surface_check.get_active()
        self._show_quick_result_message("Comparando distancias entre superficies...")
        def worker():
            try:
                record, result = session.compare(index, incremental=incremental, log=self._log, surface_reviewed=surface_reviewed)
                self._ui(self._session_comparison_done, session, record, result)
            except Exception as exc:
                self._ui(self._session_comparison_failed, str(exc))
        self.worker_thread = threading.Thread(target=worker, daemon=True)
        self.worker_thread.start()
        self._refresh_quick_workflow_state()

    def _session_comparison_done(self, session, record, result):
        try:
            session.record_result(record)
        except Exception as exc:
            self._session_comparison_failed(f"No se pudo completar el guardado: {exc}")
            return
        self._session_render_results()
        self._finish_pipeline_ui()

    def _session_comparison_failed(self, message):
        self._show_quick_result_message("Comparacion no aceptada: " + message)
        self._log(message)
        self._finish_pipeline_ui()

    def _session_render_results(self):
        session = self.measurement_session
        if not session or not session.data["results"]:
            self._show_quick_result_message("Listo para comparar cuando tengas las dos capturas.")
            return
        is_demo = session.data["demo"]
        lines = ["ENSAYO CON DATOS DE PRUEBA" if is_demo else "RESULTADO EXPLORATORIO; PRECISION DE CAMPO NO VALIDADA"]
        for child in self.quick_result_cards.get_children():
            self.quick_result_cards.remove(child)
        heading = Gtk.Label(label=lines[0], xalign=0.5)
        heading.get_style_context().add_class("quick-result-heading")
        self.quick_result_cards.pack_start(heading, False, False, 0)
        for record in session.data["results"]:
            stage = next(i + 1 for i, capture in enumerate(session.data["stages"]) if capture["id"] == record["stage_id"])
            stats = record["stats_m"]
            lines.append(f"DESPUES {stage}: media {stats['mean'] * 100:.2f} cm; mediana {stats['median'] * 100:.2f} cm")
            stage_card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
            stage_card.get_style_context().add_class("quick-result-stage")
            title = Gtk.Label(label=f"DESPUES {stage}", xalign=0)
            title.get_style_context().add_class("quick-result-stage-title")
            stage_card.pack_start(title, False, False, 0)
            metrics = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
            for caption, value in (("MEDIA", stats["mean"] * 100), ("MEDIANA", stats["median"] * 100)):
                metric = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
                metric.get_style_context().add_class("quick-result-metric")
                value_label = Gtk.Label(label=f"{value:.2f} cm")
                value_label.get_style_context().add_class("quick-result-value")
                caption_label = Gtk.Label(label=caption)
                caption_label.get_style_context().add_class("quick-result-caption")
                metric.pack_start(value_label, False, False, 0)
                metric.pack_start(caption_label, False, False, 0)
                metrics.pack_start(metric, True, True, 0)
            stage_card.pack_start(metrics, False, False, 0)
            self.quick_result_cards.pack_start(stage_card, False, False, 0)
        lines += ["La distancia no certifica espesor normal ni cobertura completa.", "CSV, mapa de colores e histograma guardados automaticamente."]
        self.quick_result_label.set_text("\n".join(lines))
        self.quick_result_label.hide()
        note = Gtk.Label(label="La distancia no certifica espesor normal ni cobertura completa.", xalign=0)
        note.set_line_wrap(True)
        note.get_style_context().add_class("quick-result-note")
        self.quick_result_cards.pack_start(note, False, False, 0)
        files = Gtk.Label(label="CSV, mapa de colores e histograma guardados automaticamente.", xalign=0)
        files.get_style_context().add_class("quick-result-files")
        self.quick_result_cards.pack_start(files, False, False, 0)
        self.quick_result_cards.show_all()

    def _show_quick_result_message(self, message):
        self.quick_result_label.set_text(message)
        self.quick_result_label.show()
        self.quick_result_cards.hide()

    def _session_show_advanced(self, page):
        self.stack.set_visible_child_name(page)

    def _session_align_selected(self):
        session = self.measurement_session
        index = self.session_stage_combo.get_active()
        if self._session_busy() or not session or index < 0:
            return
        capture = session.data["stages"][index]
        self._set_base_path(str(session.resolve(session.data["base"]["path"])))
        self._set_updated_path(str(session.resolve(capture["path"])))
        self.landmarks_base = self.landmarks_updated = None
        self.landmarks_base_label.set_text("(ninguno)")
        self.landmarks_updated_label.set_text("(ninguno)")
        self.session_stable_anchors_check.set_active(False)
        self._session_alignment_target = (session, capture)
        self.stack.set_visible_child_name("alineacion")

    def _session_registration_done(self, aligned_path, rms):
        if self._session_alignment_target is None:
            return
        session, capture = self._session_alignment_target
        if self.base_path != str(session.resolve(session.data["base"]["path"])) or self._raw_updated_path != str(session.resolve(capture["path"])):
            raise ValueError("Las capturas cambiaron durante la alineacion. Vuelve a seleccionar la etapa.")
        if not self.session_stable_anchors_check.get_active():
            raise ValueError("Confirma que las referencias fisicas no cambiaron ni fueron cubiertas por shotcrete.")
        session.register_stage(capture, Path(aligned_path), self.landmarks_base, self.landmarks_updated,
                               self.alignment_rotation, self.alignment_translation, rms)
        self._session_alignment_target = None
        self._log("Registro guardado en la sesion. Residual no equivale a precision validada de campo.")
        self.session_surface_check.set_active(False)

    def _session_crop_selected(self):
        session = self.measurement_session
        index = self.session_stage_combo.get_active()
        if self._session_busy() or not session or index < 0:
            return
        capture = session.data["stages"][index]
        if not session.data["demo"] and not capture["registration"]:
            self._show_warning("Falta alineacion", "Primero alinea la etapa para elegir la misma zona en ambas capturas.")
            return
        self._set_base_path(str(session.resolve(session.data["base"]["path"])))
        self._set_updated_path(str(session.resolve(capture["registration"]["path"] if capture["registration"] else capture["path"])))
        self._segmentation_source_base_path = None
        self._segmentation_source_updated_path = None
        self.segmentation_quad = None
        self._session_roi_target = session
        self.session_surface_check.set_active(False)
        self.stack.set_visible_child_name("segmentacion")

    def _session_view(self):
        if not self.measurement_session or not self.measurement_session.data["results"]:
            return
        from pointcloud_core import load_point_cloud
        artifact = self.measurement_session.data["results"][-1]["artifacts"]["heatmap"]
        path = self.measurement_session.resolve(artifact)
        def worker():
            try:
                show_point_cloud(load_point_cloud(path), "Aurora — mapa exploratorio de distancias")
            except Exception as exc:
                self._ui(self._show_error, "No se pudo abrir la vista", str(exc))
        threading.Thread(target=worker, daemon=True).start()

    def _session_view_pair(self):
        import copy
        import open3d as o3d
        from pointcloud_core import load_point_cloud, crop_cloud_by_quad_box
        session = self.measurement_session
        index = self.session_stage_combo.get_active()
        if not session or index < 0 or self._session_busy():
            return
        current = copy.deepcopy(session.data["stages"][index])
        previous = copy.deepcopy(session.data["stages"][index - 1] if self.session_mode_combo.get_active() == 1 and index > 0 else session.data["base"])
        roi = copy.deepcopy(session.data.get("roi"))
        def worker():
            try:
                clouds = []
                for capture in (previous, current):
                    session.verify(capture)
                    source = capture["registration"]["path"] if capture["registration"] else capture["path"]
                    cloud = load_point_cloud(session.resolve(source))
                    if roi:
                        cloud = crop_cloud_by_quad_box(cloud, np.asarray(roi["quad_base_frame_m"]), roi["width_m"])
                    clouds.append(cloud)
                clouds[0].paint_uniform_color([.55, .55, .55])
                clouds[1].paint_uniform_color([.34, .7, .78])
                o3d.visualization.draw_geometries(clouds, window_name="Aurora — referencia gris / etapa azul; registro y cobertura a revisar")
            except Exception as exc:
                self._ui(self._show_error, "No se pudo revisar el par", str(exc))
        threading.Thread(target=worker, daemon=True).start()

    def _session_report(self):
        if self.measurement_session and not self._session_busy():
            try:
                path = self.measurement_session.export_report()
                self._show_info("Informe exportado", f"Informe: {path}\nResumen CSV: {path.parent / 'resumen.csv'}")
            except Exception as exc:
                self._show_error("No se pudo exportar", str(exc))
