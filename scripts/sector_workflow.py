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
        self._pending_quick_capture_target = None
        page = self._new_page()
        page.get_style_context().add_class("sector-flow")
        intro = Gtk.Label(label="Una sesion por sector: captura o carga ANTES y cada etapa DESPUES. Puedes retirar el sensor entre etapas.", xalign=0)
        intro.set_line_wrap(True)
        page.pack_start(intro, False, False, 8)
        identity_expander = Gtk.Expander(label="Identificacion del lugar (opcional)")
        identity_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        identity_box.set_margin_top(8)
        identity_box.set_margin_bottom(4)
        identity_box.set_margin_start(4)
        self.session_tunnel_entry = Gtk.Entry()
        self.session_tunnel_entry.set_placeholder_text("Tunel")
        self.session_tunnel_entry.set_tooltip_text("Nombre del tunel para el informe de la sesion.")
        self.session_sector_entry = Gtk.Entry()
        self.session_sector_entry.set_placeholder_text("Sector o progresiva")
        self.session_sector_entry.set_tooltip_text("Sector o progresiva para el informe de la sesion.")
        identity_box.pack_start(self.session_tunnel_entry, True, True, 0)
        identity_box.pack_start(self.session_sector_entry, True, True, 0)
        identity_expander.add(identity_box)
        page.pack_start(identity_expander, False, False, 0)
        row = self._row(page)
        self.session_open_button = Gtk.Button(label="Abrir sesion guardada...")
        self.session_open_button.connect("clicked", lambda _b: self._session_open())
        row.pack_end(self.session_open_button, False, False, 0)

        frame, box = self._section("Sensor")
        row = self._row(box)
        self.quick_sensor_status_label = Gtk.Label(label="Desconectado", xalign=0)
        row.pack_start(self.quick_sensor_status_label, False, False, 0)
        self.quick_sensor_note = Gtk.Label(label="Cada captura dura 15 s. Manten el sensor quieto durante la lectura.", xalign=0)
        box.pack_start(self.quick_sensor_note, False, False, 0)
        page.pack_start(frame, False, True, 0)

        self.sensor_connection_expander = Gtk.Expander(label="IP y conexion manual (opcional)")
        connection_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        connection_box.set_margin_start(4)
        connection_box.set_margin_top(8)
        row = self._row(connection_box)
        row.pack_start(Gtk.Label(label="IP Aurora:"), False, False, 0)
        self.quick_sensor_address_entry = Gtk.Entry()
        self.quick_sensor_address_entry.set_text("192.168.11.1")
        self.quick_sensor_address_entry.set_width_chars(16)
        row.pack_start(self.quick_sensor_address_entry, False, False, 0)
        self.quick_connect_button = Gtk.Button(label="Conectar")
        self.quick_connect_button.connect("clicked", lambda _b: self._quick_connect_clicked())
        row.pack_start(self.quick_connect_button, False, False, 0)
        self.sensor_connection_expander.add(connection_box)
        page.pack_start(self.sensor_connection_expander, False, False, 0)


        pair = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        for target, title in (("base", "ANTES del shotcrete"), ("updated", "DESPUES del shotcrete")):
            frame, box = self._section(title)
            actions = self._row(box)
            button = Gtk.Button(label=f"Capturar {title.split()[0].lower()}")
            button.connect("clicked", lambda _b, target=target: self._quick_capture_clicked(target))
            setattr(self, f"quick_capture_{target}_button", button)
            actions.pack_start(button, True, True, 0)
            path_label = Gtk.Label(label="Sin archivo cargado", xalign=0)
            path_label.set_ellipsize(Pango.EllipsizeMode.MIDDLE)
            path_label.set_tooltip_text("Archivo seleccionado para esta captura")
            setattr(self, f"quick_{target}_path_label", path_label)
            box.pack_start(path_label, False, False, 0)
            pair.pack_start(frame, True, True, 0)
        page.pack_start(pair, False, True, 0)

        self.session_import_expander = Gtk.Expander(label="Cargar capturas desde archivo (opcional)")
        import_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        import_box.set_margin_start(4)
        import_box.set_margin_top(8)
        import_row = self._row(import_box)
        for target, label in (("base", "Cargar ANTES..."), ("updated", "Cargar DESPUES...")):
            button = Gtk.Button(label=label)
            button.set_tooltip_text("Usar una nube PLY ya capturada.")
            button.connect("clicked", lambda _b, target=target: self._session_import(target))
            setattr(self, f"session_import_{target}_button", button)
            import_row.pack_start(button, False, False, 0)
        self.session_import_expander.add(import_box)
        page.pack_start(self.session_import_expander, False, False, 0)

        self.session_stage_combo = Gtk.ComboBoxText()
        self.session_stage_combo.connect("changed", lambda _c: self._session_select_stage())
        self.session_mode_combo = Gtk.ComboBoxText()
        self.session_mode_combo.append_text("Respecto de ANTES")
        self.session_mode_combo.append_text("Respecto de etapa anterior")
        self.session_mode_combo.set_active(0)
        self.session_mode_combo.connect("changed", self._on_session_mode_changed)
        selection_row = self._row(page)
        selection_row.set_no_show_all(True)
        selection_row.pack_start(Gtk.Label(label="Etapa DESPUES:"), False, False, 0)
        selection_row.pack_start(self.session_stage_combo, True, True, 0)
        self.session_mode_label = Gtk.Label(label="Comparar:")
        self.session_mode_label.set_no_show_all(True)
        selection_row.pack_start(self.session_mode_label, False, False, 0)
        self.session_mode_combo.set_no_show_all(True)
        selection_row.pack_start(self.session_mode_combo, False, False, 0)
        self.session_selection_row = selection_row
        row = self._row(page)
        self.quick_stop_capture_button = Gtk.Button(label="Cancelar captura")
        self.quick_stop_capture_button.set_no_show_all(True)
        self.quick_stop_capture_button.connect("clicked", lambda _b: self._stop_capture_clicked())
        row.pack_start(self.quick_stop_capture_button, False, False, 0)
        self.quick_capture_status_label = Gtk.Label(label="Las capturas se guardan automáticamente.", xalign=0)
        self.quick_capture_status_label.set_line_wrap(True)
        self.quick_capture_status_label.set_selectable(True)
        row.pack_start(self.quick_capture_status_label, True, True, 0)

        frame, box = self._section("Alinear y comparar")
        self.session_align_note = Gtk.Label(
            label="Alinea cada etapa DESPUES con referencias estables antes de calcular.",
            xalign=0,
        )
        self.session_align_note.set_line_wrap(True)
        self.session_align_note.set_no_show_all(True)
        box.pack_start(self.session_align_note, False, False, 0)
        self.session_align_button = Gtk.Button(label="Alinear etapa con referencias estables")
        self.session_align_button.connect("clicked", lambda _b: self._session_align_selected())
        row = self._row(box)
        self.session_align_row = row
        self.session_align_row.set_no_show_all(True)
        row.pack_start(self.session_align_button, False, False, 0)
        self.quick_crop_expander = Gtk.Expander(label="Opciones de comparacion (opcional)")
        self.quick_crop_expander.set_no_show_all(True)
        crop_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        crop_box.set_margin_start(4)
        crop_box.set_margin_top(8)
        crop_row = self._row(crop_box)
        self.quick_crop_button = Gtk.Button(label="Definir zona comun")
        self.quick_crop_button.set_tooltip_text(
            "Alinea la etapa seleccionada y luego define una zona que la sesion aplicara a todas las etapas."
        )
        self.quick_crop_button.connect("clicked", lambda _b: self._session_crop_selected())
        crop_row.pack_start(self.quick_crop_button, False, False, 0)
        self.quick_crop_expander.add(crop_box)
        box.pack_start(self.quick_crop_expander, False, False, 0)
        self.quick_compare_button = Gtk.Button(label="Calcular comparacion")
        self.quick_compare_button.get_style_context().add_class("suggested-action")
        self.quick_compare_button.connect("clicked", lambda _b: self._quick_compare_clicked())
        box.pack_start(self.quick_compare_button, False, False, 0)
        self.quick_result_panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.quick_result_panel.get_style_context().add_class("quick-result-panel")
        self.quick_result_panel.set_no_show_all(True)
        self.quick_result_panel.hide()
        self.quick_result_label = Gtk.Label(label="", xalign=0)
        self.quick_result_label.set_line_wrap(True)
        self.quick_result_label.set_selectable(True)
        self.quick_result_panel.pack_start(self.quick_result_label, False, False, 0)
        self.quick_result_cards = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.quick_result_panel.pack_start(self.quick_result_cards, False, False, 0)
        box.pack_start(self.quick_result_panel, False, False, 0)
        self.session_result_actions_row = self._row(box)
        self.session_result_actions_row.set_no_show_all(True)
        self.session_view_button = Gtk.Button(label="Ver mapa del resultado 3D")
        self.session_view_button.set_no_show_all(True)
        self.session_view_button.connect("clicked", lambda _b: self._session_view())
        self.session_report_button = Gtk.Button(label="Abrir informe")
        self.session_report_button.set_no_show_all(True)
        self.session_report_button.connect("clicked", lambda _b: self._session_report())
        self.session_result_actions_row.pack_start(self.session_view_button, False, False, 0)
        self.session_result_actions_row.pack_start(self.session_report_button, False, False, 0)
        page.pack_start(frame, False, True, 0)
        self._refresh_quick_workflow_state()
        return self._scrolled(page)

    def _session_busy(self):
        return bool(self.capture_stop_event is not None or self._session_importing or getattr(self, "_sensor_connecting", False) or (self.worker_thread and self.worker_thread.is_alive()))

    def _new_pair_session(self):
        session = MeasurementSession.create(
            Path(self.output_dir) / "sesiones",
            self.session_tunnel_entry.get_text().strip() or "Tunel sin identificar",
            self.session_sector_entry.get_text().strip() or "Sector sin identificar",
        )
        self._session_activate(session)
        return session

    def _quick_capture_clicked(self, target):
        if self._session_busy():
            self._show_warning("Trabajo en curso", "Espera a que termine la captura o comparacion.")
            return
        if target != "base" and (not self.measurement_session or not self.measurement_session.data["base"]):
            self._show_warning("Falta el ANTES", "Primero captura o carga la nube de ANTES.")
            return
        if self.sensor_connection is None:
            address = self.quick_sensor_address_entry.get_text().strip()
            if not address:
                self._show_error("IP invalida", "Escribe la direccion IP del sensor Aurora.")
                return
            self._pending_quick_capture_target = target
            self.quick_capture_status_label.set_text("Conectando el sensor para iniciar la captura...")
            self._toggle_sensor_connection()
            return
        if target == "base":
            try:
                self._new_pair_session()
            except Exception as exc:
                self._show_error("No se pudo iniciar la comparacion", str(exc))
                return
        self._capture_clicked(target)

    def _session_picker_start_dir(self) -> Path:
        start_dir = Path(self.output_dir) / "sesiones"
        if start_dir.is_dir():
            manifests = [path for path in start_dir.glob("*/sesion.json") if path.is_file()]
            if manifests:
                return max(manifests, key=lambda path: path.stat().st_mtime).parent
            return start_dir
        return Path(self.output_dir)

    def _session_open(self):
        if self._session_busy():
            return
        path = self._choose_path(
            "Abrir sesion de medicion",
            Gtk.FileChooserAction.OPEN,
            "_Abrir",
            initial_dir=self._session_picker_start_dir(),
            filter_name="Sesion Aurora (*.json)",
            filter_pattern="*.json",
        )
        if path:
            self._session_load(Path(path))

    def _session_load(self, path):
        try:
            self._session_activate(MeasurementSession.load(path))
        except Exception as exc:
            self._show_error("No se pudo abrir la comparacion", str(exc))

    def _session_activate(self, session):
        self.measurement_session = session
        self.session_tunnel_entry.set_text(session.data["tunnel"])
        self.session_sector_entry.set_text(session.data["sector"])
        self._session_alignment_target = None
        self._session_roi_target = None
        self.session_mode_combo.set_active(0)
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
        base_capture = session.data["base"] if session else None
        if base_capture:
            self.base_path = str(session.resolve(base_capture["path"]))
        else:
            self.base_path = ""
        index = self.session_stage_combo.get_active()
        self.session_mode_combo.set_sensitive(index > 0)
        if index <= 0 and self.session_mode_combo.get_active() != 0:
            self.session_mode_combo.set_active(0)
        selected = session.data["stages"][index] if session and 0 <= index < len(session.data["stages"]) else None
        if selected:
            self.quick_updated_path_label.set_text(selected["source_name"])
            selected_path = selected["registration"]["path"] if selected["registration"] else selected["path"]
            self.updated_path = str(session.resolve(selected_path))
        else:
            self.quick_updated_path_label.set_text("Sin archivo cargado")
            self.updated_path = ""
        self._update_alignment_pair_label()
        self._session_render_results()
        self._refresh_quick_workflow_state()

    def _on_session_mode_changed(self, _combo):
        self._session_render_results()
        self._refresh_quick_workflow_state()

    def _refresh_quick_workflow_state(self):
        if not hasattr(self, "quick_compare_button"):
            return
        session = self.measurement_session
        busy = self._session_busy()
        has_base = bool(session and session.data["base"])
        has_stage = bool(session and self.session_stage_combo.get_active() >= 0)
        next_stage = len(session.data["stages"]) + 1 if session else 1
        if has_base:
            self.quick_capture_base_button.set_label("Nueva medicion · capturar ANTES")
            self.quick_capture_base_button.set_tooltip_text(
                "Inicia otra medicion y guarda la actual automaticamente."
            )
            self.session_import_base_button.set_label("Nueva medicion · cargar ANTES...")
            self.session_import_base_button.set_tooltip_text(
                "Crea otra medicion desde una nube; la sesion actual queda guardada."
            )
        else:
            self.quick_capture_base_button.set_label("Capturar ANTES")
            self.quick_capture_base_button.set_tooltip_text(
                "Inicia una sesion nueva con la nube original del sector."
            )
            self.session_import_base_button.set_label("Cargar ANTES...")
            self.session_import_base_button.set_tooltip_text(
                "Inicia una sesion nueva usando una nube PLY existente."
            )
        self.quick_capture_updated_button.set_label(f"Capturar DESPUES {next_stage}")
        self.session_import_updated_button.set_label(f"Cargar DESPUES {next_stage}...")
        self.session_stage_combo.set_sensitive(has_stage and not busy)
        self.session_selection_row.set_no_show_all(not has_stage)
        show_comparison_mode = has_stage and self.session_stage_combo.get_active() > 0
        self.session_mode_combo.set_sensitive(show_comparison_mode and not busy)
        if show_comparison_mode:
            self.session_mode_label.show()
            self.session_mode_combo.show()
        else:
            self.session_mode_label.hide()
            self.session_mode_combo.hide()
        if has_stage:
            self.session_selection_row.show_all()
            if not show_comparison_mode:
                self.session_mode_label.hide()
                self.session_mode_combo.hide()
        else:
            self.session_selection_row.hide()
        self.quick_capture_base_button.set_sensitive(not busy)
        self.quick_capture_updated_button.set_sensitive(has_base and not busy)
        self.session_import_base_button.set_sensitive(not busy)
        self.session_import_updated_button.set_sensitive(has_base and not busy)
        self.session_open_button.set_sensitive(not busy)
        selected = session.data["stages"][self.session_stage_combo.get_active()] if has_stage else None
        already_measured = bool(selected and any(selected["id"] in (r["stage_id"], r["reference_id"]) for r in session.data["results"]))
        registration = selected.get("registration") if selected else None
        has_registration = bool(
            registration
            and session.data["base"]
            and registration.get("base_id") == session.data["base"]["id"]
        )
        has_results = bool(session and session.data["results"])
        has_roi = bool(session and session.data.get("roi"))
        self.quick_crop_expander.set_visible(has_roi or (has_registration and not has_results))
        if has_roi:
            self.quick_crop_expander.set_label("Zona comun guardada · opciones")
        else:
            self.quick_crop_expander.set_label("Opciones de comparacion (opcional)")
        selected_result = self._session_selected_result()
        mode_valid = self.session_stage_combo.get_active() > 0 or self.session_mode_combo.get_active() == 0
        self.quick_compare_button.set_sensitive(has_registration and mode_valid and not busy)
        show_alignment_action = has_stage
        if already_measured:
            self.session_align_note.set_text(
                "Esta etapa ya tiene comparaciones guardadas. Puedes volver a alinearla; los resultados anteriores se conservan."
            )
            self.session_align_button.set_label("Rehacer alineacion de esta etapa")
            self.session_align_button.set_tooltip_text(
                "Guarda una nueva alineacion para futuras comparaciones. Los resultados existentes no cambian."
            )
        else:
            self.session_align_note.set_text("Alinea cada etapa DESPUES con referencias estables antes de calcular.")
            self.session_align_button.set_label("Alinear etapa con referencias estables")
            self.session_align_button.set_tooltip_text("Marca puntos fisicos estables que aparezcan en ambas nubes.")
        self.session_align_note.set_visible(show_alignment_action)
        self.session_align_row.set_visible(show_alignment_action)
        self.session_align_button.set_sensitive(has_stage and not busy)
        self.quick_crop_button.set_sensitive(has_registration and not busy and not has_results)
        if has_roi:
            if has_results:
                crop_label = "Zona comun fijada"
            elif has_registration:
                crop_label = "Cambiar zona comun"
            else:
                crop_label = "Zona comun guardada"
            self.quick_crop_button.set_label(crop_label)
        else:
            self.quick_crop_button.set_label("Definir zona comun")
        self.session_report_button.set_sensitive(bool(session and session.data["results"]) and not busy)
        self.session_view_button.set_sensitive(selected_result is not None and not busy)
        has_report = bool(session and session.data["results"])
        has_view = selected_result is not None
        self.session_report_button.set_visible(has_report)
        self.session_view_button.set_visible(has_view)
        self.session_result_actions_row.set_visible(has_report or has_view)
        self.quick_stop_capture_button.set_sensitive(self.capture_stop_event is not None)
        self.quick_stop_capture_button.set_visible(self.capture_stop_event is not None)

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
                # Sensor captures are staged inside the session so add_capture can
                # validate and copy them. Remove only our own staging files after
                # the durable session copy and manifest have been written.
                source = Path(path)
                try:
                    owned_temp = (captured and source.name.startswith(".capture_")
                                  and source.resolve().parent == session.path.parent.resolve())
                except OSError:
                    owned_temp = False
                if owned_temp:
                    for temporary in (source, source.with_name(source.stem + "_pose.npz"),
                                      source.with_name(source.stem + "_ref.npz")):
                        try:
                            temporary.unlink(missing_ok=True)
                        except OSError as exc:
                            self._log(f"No se pudo retirar temporal {temporary.name}: {exc}")
                self._ui(self._session_added, session)
            except Exception as exc:
                self._ui(self._session_import_failed, str(exc))
        threading.Thread(target=worker, daemon=True).start()

    def _session_added(self, session):
        self._session_importing = False
        self._session_activate(session)
        self.quick_capture_status_label.set_text(f"Captura guardada en la sesion · {session.path.parent}")

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
        if incremental and index == 0:
            self._show_quick_result_message("La primera etapa solo se puede comparar con ANTES.")
            return
        try:
            pipeline_options = self._build_params()
        except Exception as exc:
            self._show_error("Parametros invalidos", str(exc))
            return
        self._show_quick_result_message("Comparando distancias entre superficies...")
        def worker():
            try:
                record, _result = session.compare(
                    index, incremental=incremental, log=self._log,
                    pipeline_options=pipeline_options,
                )
                self._ui(self._session_comparison_done, session, record)
            except Exception as exc:
                self._ui(self._session_comparison_failed, str(exc))
        self.worker_thread = threading.Thread(target=worker, daemon=True)
        self.worker_thread.start()
        self._refresh_quick_workflow_state()

    def _session_comparison_done(self, session, record):
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
        record = self._session_selected_result()
        if not session or record is None:
            self.quick_result_panel.hide()
            return
        for child in self.quick_result_cards.get_children():
            self.quick_result_cards.remove(child)
        heading = Gtk.Label(label="RESULTADO EXPLORATORIO; PRECISION DE CAMPO NO VALIDADA", xalign=0.5)
        heading.get_style_context().add_class("quick-result-heading")
        self.quick_result_cards.pack_start(heading, False, False, 0)
        stage = next(i + 1 for i, capture in enumerate(session.data["stages"]) if capture["id"] == record["stage_id"])
        mode_label = "respecto de ANTES" if record["mode"] == "respecto_base" else "respecto de etapa anterior"
        stats = record["stats_m"]
        stage_card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        stage_card.get_style_context().add_class("quick-result-stage")
        title = Gtk.Label(label=f"DESPUES {stage} · {mode_label}", xalign=0)
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
        self.quick_result_label.hide()
        note = Gtk.Label(label="La distancia no certifica espesor normal ni cobertura completa.", xalign=0)
        note.set_line_wrap(True)
        note.get_style_context().add_class("quick-result-note")
        self.quick_result_cards.pack_start(note, False, False, 0)
        files = Gtk.Label(label="CSV, mapa de colores e histograma guardados automaticamente.", xalign=0)
        files.get_style_context().add_class("quick-result-files")
        self.quick_result_cards.pack_start(files, False, False, 0)
        self.quick_result_panel.show()
        self.quick_result_cards.show_all()

    def _show_quick_result_message(self, message):
        self.quick_result_panel.show()
        self.quick_result_label.set_text(message)
        self.quick_result_label.show()
        self.quick_result_cards.hide()

    def _session_align_selected(self):
        session = self.measurement_session
        index = self.session_stage_combo.get_active()
        if self._session_busy():
            return False
        if not session or index < 0:
            self._show_warning("Falta una etapa DESPUES", "Carga ANTES y DESPUES antes de alinear.")
            return False
        capture = session.data["stages"][index]
        self._set_base_path(str(session.resolve(session.data["base"]["path"])))
        self._set_updated_path(str(session.resolve(capture["path"])))
        base_id = session.data["base"]["id"]
        registration = capture.get("registration")
        if registration and registration.get("base_id") != base_id:
            registration = None
        reference_stage = capture if registration else next(
            (
                stage for stage in reversed(session.data["stages"])
                if stage.get("registration")
                and stage["registration"].get("base_id") == base_id
                and stage["registration"].get("base_points")
            ),
            None,
        )
        reference_registration = reference_stage.get("registration") if reference_stage else None
        self.landmarks_base = (
            np.asarray(reference_registration["base_points"], dtype=float).copy()
            if reference_registration else None
        )
        self.landmarks_updated = None
        if reference_registration:
            self.landmarks_base_label.set_text(
                f"{len(self.landmarks_base)} puntos reutilizados de {reference_stage['source_name']}"
            )
            self.pick_base_alignment_button.set_label("Cambiar puntos ANTES para esta etapa...")
            self.pick_base_alignment_button.set_tooltip_text(
                "La sesion reutiliza los puntos ANTES de una alineacion anterior. "
                "Puedes elegir otro conjunto para esta etapa si esas marcas ya no son visibles."
            )
        else:
            self.landmarks_base_label.set_text("(ninguno)")
            self.pick_base_alignment_button.set_label("1. Elegir puntos en el tunel original...")
            self.pick_base_alignment_button.set_tooltip_text(
                "Shift+Click sobre cada punto de referencia, en orden, despues cerrar la ventana."
            )
        self.landmarks_updated_label.set_text("(ninguno)")
        self._session_alignment_target = (session, capture)
        self.stack.set_visible_child_name("alineacion")
        return True

    def _session_registration_done(self, aligned_path, rms):
        if self._session_alignment_target is None:
            return
        session, capture = self._session_alignment_target
        if self.base_path != str(session.resolve(session.data["base"]["path"])) or self._raw_updated_path != str(session.resolve(capture["path"])):
            raise ValueError("Las capturas cambiaron durante la alineacion. Vuelve a seleccionar la etapa.")
        session.register_stage(capture, Path(aligned_path), self.landmarks_base, self.landmarks_updated,
                               self.alignment_rotation, self.alignment_translation, rms)
        self._session_alignment_target = None
        self._log("Registro guardado en la sesion. Residual no equivale a precision validada de campo.")

    def _session_crop_selected(self):
        session = self.measurement_session
        index = self.session_stage_combo.get_active()
        if self._session_busy() or not session or index < 0:
            return
        capture = session.data["stages"][index]
        if not capture["registration"]:
            self._show_warning("Falta alineacion", "Primero alinea la etapa para elegir la misma zona en ambas capturas.")
            return
        self._set_base_path(str(session.resolve(session.data["base"]["path"])))
        self._set_updated_path(str(session.resolve(capture["registration"]["path"] if capture["registration"] else capture["path"])))
        self.segmentation_quad = None
        self._session_roi_target = session
        self.stack.set_visible_child_name("segmentacion")

    def _session_selected_result(self):
        session = self.measurement_session
        index = self.session_stage_combo.get_active()
        if not session or index < 0 or index >= len(session.data["stages"]):
            return None
        stage_id = session.data["stages"][index]["id"]
        mode = "entre_etapas" if self.session_mode_combo.get_active() == 1 else "respecto_base"
        return next(
            (
                record for record in reversed(session.data["results"])
                if record["stage_id"] == stage_id and record["mode"] == mode
            ),
            None,
        )

    def _session_view(self):
        record = self._session_selected_result()
        if record is None:
            return
        from pointcloud_core import load_point_cloud
        path = self.measurement_session.resolve(record["artifacts"]["heatmap"])
        def worker():
            try:
                show_point_cloud(load_point_cloud(path), "Aurora — mapa exploratorio de distancias")
            except Exception as exc:
                self._ui(self._show_error, "No se pudo abrir la vista", str(exc))
        threading.Thread(target=worker, daemon=True).start()

    def _session_report(self):
        if not self.measurement_session or self._session_busy():
            return
        path = self.measurement_session.path.parent / "informe.md"
        if not path.is_file():
            self._show_error("No se encontro el informe", "Vuelve a calcular la comparacion para regenerar sus archivos.")
            return
        try:
            if not self._open_local_uri(path.as_uri()):
                self._show_error("No se pudo abrir el informe", f"Archivo disponible en: {path}")
        except Exception as exc:
            self._show_error("No se pudo abrir el informe", f"{exc}\nArchivo disponible en: {path}")

    def _open_local_uri(self, uri):
        return Gtk.show_uri_on_window(self.window, uri, Gdk.CURRENT_TIME)
