"""GTK integration: every page, synthetic captures, resume, reports, field gate."""
from __future__ import annotations
import tempfile
import os
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk
import numpy as np
import open3d as o3d
import gui_gtk as module
import pointcloud_core as core
import sector_workflow as workflow
from live_viewer import LiveViewer


def pump_until(predicate, description, timeout_s=30):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError(f"Timed out: {description}")


def main():
    provider = Gtk.CssProvider()
    provider.load_from_data(module.CSS)
    Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    with tempfile.TemporaryDirectory() as temp:
        root = Path(temp)
        stale_data = root / "data"
        stale_data.mkdir()
        (stale_data / "base_capturada_20261012_120000.ply").write_text("stale test capture")
        (stale_data / "updated_capturada_20261012_120030.ply").write_text("stale test capture")
        module.PROJECT_ROOT = root
        app = module.AuroraGUI(Gtk.Window())
        app.window.show_all()
        assert not hasattr(app, "session_surface_check")
        errors, warnings, infos = [], [], []
        app._show_error = lambda *args: errors.append(args)
        app._show_warning = lambda *args: warnings.append(args)
        app._show_info = lambda *args: infos.append(args)
        assert app.stack.get_visible_child_name() == "rapido"
        assert app.technical_log_frame.get_visible()
        assert isinstance(app.technical_log_frame, Gtk.Expander)
        assert app.technical_log_frame.get_label() == "Registro tecnico · detalles de captura y proceso"
        assert app.quick_sensor_status_label.get_use_markup()
        assert app.base_path == app.updated_path == ""
        app._open_viewer()
        assert app.viewer is None
        assert infos[-1][0] == "Sin nube del sector"
        assert not app.quick_result_panel.get_visible()
        assert not app.session_import_expander.get_expanded()
        assert not app.quick_crop_expander.get_expanded()
        assert not app.quick_crop_expander.get_visible()
        assert not app.session_align_note.get_visible()
        assert not app.session_align_row.get_visible()
        assert not app.sensor_connection_expander.get_expanded()
        assert app.session_open_button.get_label() == "Abrir sesion guardada..."
        assert app.session_import_base_button.get_label() == "Cargar ANTES..."
        assert app.quick_capture_base_button.get_label() == "Capturar ANTES"
        assert app.quick_capture_updated_button.get_label() == "Capturar DESPUES 1"
        assert app.session_import_updated_button.get_label() == "Cargar DESPUES 1..."
        session_dir = Path(app.output_dir) / "sesiones"
        older_dir, newer_dir = session_dir / "older", session_dir / "newer"
        older_dir.mkdir(parents=True)
        newer_dir.mkdir()
        older_manifest, newer_manifest = older_dir / "sesion.json", newer_dir / "sesion.json"
        older_manifest.write_text("{}")
        newer_manifest.write_text("{}")
        os.utime(older_manifest, ns=(1_000_000_000, 1_000_000_000))
        os.utime(newer_manifest, ns=(2_000_000_000, 2_000_000_000))
        assert app._session_picker_start_dir() == newer_dir
        assert set(app._sidebar_rows) == {"rapido", "alineacion", "segmentacion", "procesamiento", "visualizacion"}
        assert hasattr(app, "alignment_back_button")
        assert not hasattr(app, "run_button")
        assert not hasattr(app, "generate_report_button")
        assert not hasattr(app, "results_frame")
        assert "captura" not in app._sidebar_rows
        assert not hasattr(app, "use_crop_check")
        assert not hasattr(app, "crop_min_entry")
        assert not hasattr(app, "crop_max_entry")
        assert not hasattr(app, "_pick_crop_interactively")
        assert not hasattr(app, "use_icp_check")
        assert not hasattr(app, "icp_threshold_entry")
        assert not app._build_params().use_icp
        assert not hasattr(app, "sensor_address_entry")
        assert not hasattr(app, "capture_base_button")
        assert not hasattr(app, "capture_updated_button")
        assert not hasattr(app, "capture_live_baseline_button")
        assert not hasattr(app, "tools_nav_buttons")
        assert hasattr(app, "voxel_entry") and hasattr(app, "remove_outliers_check")
        assert app.analysis_options_expander.get_label() == "Ajustes avanzados de medicion"
        assert app.analysis_options_expander.get_expanded()
        app.voxel_entry.set_text("0.012")
        app.remove_outliers_check.set_active(True)
        advanced_params = app._build_params()
        assert advanced_params.voxel_size == .012 and advanced_params.remove_outliers
        app.voxel_entry.set_text("0.0")
        app.remove_outliers_check.set_active(False)
        assert not app.quick_crop_button.get_sensitive()
        assert not hasattr(app, "_clear_segmentation")
        assert not hasattr(app, "view_segmented_base_button")
        assert not hasattr(app, "start_live_capture_button")
        assert hasattr(app, "stream_toggle_button")
        assert not hasattr(app, "color_banded_rb")
        assert not hasattr(app, "band_low_entry")
        assert not hasattr(app, "max_distance_entry")
        assert app._build_params().max_distance is None
        assert app._parse_live_filter_params() == (None, None)
        viewer = LiveViewer(o3d.geometry.PointCloud(o3d.utility.Vector3dVector([[0.0, 0.0, 0.0]])))
        near_color = np.asarray(viewer._make_inspection_cloud(np.array([[0.0, 0.0, 0.01]])).colors)
        far_color = np.asarray(viewer._make_inspection_cloud(np.array([[0.0, 0.0, 10.0]])).colors)
        assert np.allclose(near_color, far_color)
        assert np.allclose(near_color[0], [0.22, 0.72, 0.82])
        assert not hasattr(app, "landmark_base_source_photo_rb")
        assert not hasattr(app, "load_base_photo_button")
        assert not hasattr(app, "_load_manual_reference_photo")
        for page in app._sidebar_rows:
            app.stack.set_visible_child_name(page)
            pump_until(lambda: not Gtk.events_pending(), "GTK navigation")
            assert app.stack.get_visible_child_name() == page
        page_names = [app.stack.child_get_property(child, "name") for child in app.stack.get_children()]
        assert not {"captura", "alineacion_imu", "comparacion", "prueba_embebida", "prueba_raycasting"} & set(page_names)
        assert {"alineacion", "segmentacion", "procesamiento", "visualizacion"} <= set(page_names)
        assert len(page_names) == 5
        for page in page_names:
            app.stack.set_visible_child_name(page)
            pump_until(lambda: not Gtk.events_pending(), "GTK page render")
        app.stack.set_visible_child_name("visualizacion")
        assert app.technical_log_frame.get_visible()
        app.stack.set_visible_child_name("rapido")
        assert app.technical_log_frame.get_visible()
        app.stack.set_visible_child_name("alineacion")
        app.alignment_back_button.emit("clicked")
        assert app.stack.get_visible_child_name() == "rapido"

        xy = np.array([(x, y) for x in np.linspace(0, 1, 12) for y in np.linspace(0, 1, 12)])
        import_before, import_after = root / "import_before.ply", root / "import_after.ply"
        clouds = [o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[xy, np.full(len(xy), z)])) for z in (0, .04)]
        o3d.io.write_point_cloud(str(import_before), clouds[0])
        o3d.io.write_point_cloud(str(import_after), clouds[1])
        app._new_pair_session()
        app._session_add_source(import_before, "base")
        pump_until(lambda: app.measurement_session.data["base"] and not app._session_busy(), "loaded BEFORE")
        assert import_before.is_file()
        assert app.quick_capture_base_button.get_label() == "Nueva medicion · capturar ANTES"
        assert app.session_import_base_button.get_label() == "Nueva medicion · cargar ANTES..."
        app._session_add_source(import_after, "updated")
        pump_until(lambda: len(app.measurement_session.data["stages"]) == 1 and not app._session_busy(), "loaded AFTER")
        assert import_after.is_file()
        assert app.quick_capture_updated_button.get_label() == "Capturar DESPUES 2"
        assert app.session_align_note.get_visible() and app.session_align_row.get_visible()
        assert not app.session_report_button.get_sensitive()
        assert not app.session_view_button.get_visible() and not app.session_report_button.get_visible()
        anchors = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=float)
        app._sidebar_listbox.select_row(app._sidebar_rows["alineacion"])
        assert app.stack.get_visible_child_name() == "alineacion"
        assert app._session_alignment_target is not None
        app.landmarks_base = anchors
        app.landmarks_updated = anchors.copy()
        app._apply_alignment()
        pump_until(lambda: app.measurement_session.data["stages"][0]["registration"] and not app._session_busy(), "imported pair alignment")
        assert app.quick_compare_button.get_sensitive(), (
            app.measurement_session.data["stages"][0]["registration"],
            app.session_stage_combo.get_active(),
            app._session_busy(),
        )
        assert app.stack.get_visible_child_name() == "rapido"
        app.voxel_entry.set_text("0.20")
        app._quick_compare_clicked()
        pump_until(lambda: len(app.measurement_session.data["results"]) == 1 and not app._session_busy(), "loaded pair comparison")
        assert abs(app.measurement_session.data["results"][0]["stats_m"]["mean"] - .04) < 1e-6
        assert app.session_align_note.get_visible() and app.session_align_row.get_visible()
        assert app.session_align_button.get_sensitive()
        assert app.session_align_button.get_label() == "Rehacer alineacion de esta etapa"
        assert app.measurement_session.data["results"][0]["stats_m"]["n_points"] < len(xy)
        app.voxel_entry.set_text("0.0")

        def fail_connect(_address):
            raise OSError("sensor desconectado")
        module.aurora_sensor.connect = fail_connect
        module.aurora_sensor.disconnect = lambda _: None
        reference_frame_calls = []
        module.aurora_sensor.capture_reference_frame = lambda *args, **kwargs: reference_frame_calls.append(True)
        module.aurora_sensor.get_current_pose = lambda _: SimpleNamespace(position=np.zeros(3), rpy_deg=np.zeros(3), timestamp_ns=1234)
        count = 0
        gate = threading.Event()
        gate.set()
        def capture(*args, **kwargs):
            nonlocal count
            gate.wait(5)
            height = count * .04
            count += 1
            return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[xy, np.full(len(xy), height)]))
        def register_selected_stage(expect_reused=False):
            app._session_align_selected()
            if expect_reused:
                assert np.allclose(app.landmarks_base, anchors)
                assert "reutilizados" in app.landmarks_base_label.get_text()
            else:
                app.landmarks_base = anchors
            app.landmarks_updated = anchors.copy()
            app._apply_alignment()
            index = app.session_stage_combo.get_active()
            pump_until(lambda: app.measurement_session.data["stages"][index]["registration"] and not app._session_busy(), "sensor stage alignment")
        module.aurora_sensor.capture_snapshot = capture
        assert app.sensor_connection is None and app.quick_capture_base_button.get_sensitive()
        app.session_tunnel_entry.set_text("Tunel de prueba")
        app.session_sector_entry.set_text("Sector norte")
        session_before_failed_connect = app.measurement_session
        app._quick_capture_clicked("base")
        pump_until(lambda: not app._sensor_connecting, "failed auto-connect")
        assert app.measurement_session is session_before_failed_connect
        assert app._pending_quick_capture_target is None
        assert "No se pudo conectar" in app.quick_capture_status_label.get_text()
        assert errors[-1][0] == "Error de conexion"
        errors.clear()
        module.aurora_sensor.connect = lambda _: object()
        app._quick_capture_clicked("base")
        pump_until(lambda: app.measurement_session.data["base"] and not app._session_busy(), "BASE")
        captured_base = app.measurement_session.data["base"]
        assert (app.measurement_session.path.parent / captured_base["path"]).is_file()
        assert not (app.measurement_session.path.parent / captured_base["source_name"]).exists()
        assert (stale_data / "base_capturada_20261012_120000.ply").is_file()
        assert (stale_data / "updated_capturada_20261012_120030.ply").is_file()
        assert app.sensor_connection is not None
        assert not reference_frame_calls
        assert app.measurement_session.data["tunnel"] == "Tunel de prueba"
        assert app.measurement_session.data["sector"] == "Sector norte"
        assert app.quick_capture_base_button.get_sensitive()
        for stage in (1, 2):
            app._quick_capture_clicked("updated")
            pump_until(lambda: len(app.measurement_session.data["stages"]) == stage and not app._session_busy(), "stage")
            assert app.quick_capture_updated_button.get_label() == f"Capturar DESPUES {stage + 1}"
            assert app.session_mode_combo.get_visible() == (stage > 1)
            register_selected_stage(expect_reused=(stage > 1))
            app._quick_compare_clicked()
            pump_until(lambda: len(app.measurement_session.data["results"]) == stage and not app._session_busy(), "comparison")
        assert np.allclose([r["stats_m"]["mean"] for r in app.measurement_session.data["results"]], [.04, .08])
        app.session_mode_combo.set_active(1)
        app._quick_compare_clicked()
        pump_until(lambda: len(app.measurement_session.data["results"]) == 3 and not app._session_busy(), "between stages")
        assert abs(app.measurement_session.data["results"][-1]["stats_m"]["mean"] - .04) < 1e-6
        session_path = app.measurement_session.path
        app._session_load(session_path)
        assert len(app.measurement_session.data["stages"]) == 2
        assert not app.quick_result_label.get_visible()
        result_children = app.quick_result_cards.get_children()
        assert app.quick_result_cards.get_visible() and len(result_children) == 4
        assert "RESULTADO EXPLORATORIO" in result_children[0].get_text()
        assert result_children[0].get_style_context().has_class("quick-result-heading")
        assert result_children[1].get_style_context().has_class("quick-result-stage")
        assert result_children[1].get_children()[0].get_text() == "DESPUES 2 · respecto de ANTES"
        app.session_mode_combo.set_active(1)
        assert app.quick_result_cards.get_children()[1].get_children()[0].get_text() == "DESPUES 2 · respecto de etapa anterior"
        app.session_mode_combo.set_active(0)
        selected_record = app._session_selected_result()
        assert selected_record["stage_id"] == app.measurement_session.data["stages"][1]["id"]
        assert selected_record["mode"] == "respecto_base"
        selected_capture = app.measurement_session.data["stages"][1]
        assert Path(app.base_path) == app.measurement_session.resolve(app.measurement_session.data["base"]["path"])
        assert Path(app.updated_path) == app.measurement_session.resolve(selected_capture["registration"]["path"])
        expected_heatmap = app.measurement_session.resolve(selected_record["artifacts"]["heatmap"])
        loaded_heatmaps = []
        original_load_cloud = core.load_point_cloud
        original_show_cloud = workflow.show_point_cloud
        core.load_point_cloud = lambda path: (loaded_heatmaps.append(Path(path)), original_load_cloud(path))[1]
        workflow.show_point_cloud = lambda *_args: None
        app._session_view()
        pump_until(lambda: bool(loaded_heatmaps), "selected result map")
        assert loaded_heatmaps[0] == expected_heatmap
        core.load_point_cloud = original_load_cloud
        workflow.show_point_cloud = original_show_cloud
        app.session_mode_combo.set_active(1)
        assert app._session_selected_result()["mode"] == "entre_etapas"
        app.session_stage_combo.set_active(0)
        assert app.session_mode_combo.get_active() == 0 and not app.session_mode_combo.get_sensitive()
        assert not app.session_mode_label.get_visible() and not app.session_mode_combo.get_visible()
        assert app._session_selected_result()["stage_id"] == app.measurement_session.data["stages"][0]["id"]
        first_capture = app.measurement_session.data["stages"][0]
        assert Path(app.updated_path) == app.measurement_session.resolve(first_capture["registration"]["path"])
        assert app.quick_updated_path_label.get_text() == first_capture["source_name"]
        app.session_stage_combo.set_active(1)
        assert app.session_mode_label.get_visible() and app.session_mode_combo.get_visible()
        assert app.quick_updated_path_label.get_text() == app.measurement_session.data["stages"][1]["source_name"]
        assert (session_path.parent / "informe.md").is_file()
        assert (session_path.parent / "resumen.csv").is_file()
        assert app.session_report_button.get_sensitive()
        assert not app._session_busy()
        opened_reports = []
        app._open_local_uri = lambda uri: opened_reports.append(uri) or True
        app._session_report()
        assert opened_reports == [(session_path.parent / "informe.md").as_uri()], (opened_reports, session_path.parent)
        assert app.measurement_session.data["base"]["pose"]["sensor_timestamp_ns"] == 1234

        # A resumed session should reuse its persisted BASE anchors for a new layer.
        app._quick_capture_clicked("updated")
        pump_until(lambda: len(app.measurement_session.data["stages"]) == 3 and not app._session_busy(), "resumed stage")
        assert not app.session_view_button.get_visible() and app.session_report_button.get_visible()
        app._session_align_selected()
        assert np.allclose(app.landmarks_base, anchors)
        assert "reutilizados" in app.landmarks_base_label.get_text()

        gate.clear()
        app._capture_clicked("updated")
        assert not app.quick_compare_button.get_sensitive()
        assert not app.session_open_button.get_sensitive()
        assert not app.session_stage_combo.get_sensitive()
        assert not app.session_mode_combo.get_sensitive()
        app._quick_compare_clicked()
        assert warnings[-1][0] == "Trabajo en curso"
        assert app._on_delete() is True
        assert warnings[-1][0] == "Guardado en curso"
        stages_before_cancel = len(app.measurement_session.data["stages"])
        gate.set()
        pump_until(lambda: app.capture_stop_event is None and not app._session_busy(), "discard cancelled capture")
        assert app.session_stage_combo.get_sensitive()
        assert app.session_mode_combo.get_sensitive()
        assert len(app.measurement_session.data["stages"]) == stages_before_cancel
        assert "No se guardo ninguna nube" in app.quick_capture_status_label.get_text()
        # Also cover the race where the worker queues a completed capture before
        # GTK processes the operator's cancellation click.
        delayed_cancel = threading.Event()
        delayed_cancel.set()
        app.capture_stop_event = delayed_cancel
        saved_clouds_before = set((root / "data").glob("*_capturada_*.ply"))
        app._on_capture_done("updated", clouds[1], stop_event=delayed_cancel)
        assert set((root / "data").glob("*_capturada_*.ply")) == saved_clouds_before
        assert len(app.measurement_session.data["stages"]) == stages_before_cancel
        app._quick_capture_clicked("base")
        pump_until(lambda: app.measurement_session.data["base"] and not app._session_busy(), "field BASE")
        app._quick_capture_clicked("updated")
        pump_until(lambda: len(app.measurement_session.data["stages"]) == 1 and not app._session_busy(), "field stage")
        assert not app.session_mode_combo.get_visible()
        app.session_mode_combo.set_active(0)
        app._quick_compare_clicked()
        pump_until(lambda: not app._session_busy(), "field guard")
        assert not app.measurement_session.data["results"]
        assert "Faltan referencias" in app.quick_result_label.get_text()
        register_selected_stage()
        assert app.quick_crop_expander.get_visible()
        assert app.quick_crop_button.get_sensitive()
        app.quick_crop_expander.set_expanded(True)
        app.quick_crop_button.emit("clicked")
        assert app.stack.get_visible_child_name() == "segmentacion"
        app.segmentation_quad = np.array([[.1, .1, .16], [.9, .1, .16], [.9, .9, .16], [.1, .9, .16]])
        app.segmentation_width_spin.set_value(100)
        app._apply_segmentation()
        pump_until(lambda: app.measurement_session.data.get("roi") and not app._session_busy(), "common sector crop")
        assert app.stack.get_visible_child_name() == "rapido"
        assert not app.quick_crop_expander.get_expanded()
        assert app.quick_crop_button.get_label() == "Cambiar zona comun"
        assert not list(app.measurement_session.path.parent.glob("capturas/*_segmento.ply"))
        app.stack.set_visible_child_name("rapido")
        app._quick_compare_clicked()
        pump_until(lambda: len(app.measurement_session.data["results"]) == 1 and not app._session_busy(), "registered sector comparison")
        assert app.measurement_session.data["results"][0]["status"] == "exploratorio_no_validado"
        assert abs(app.measurement_session.data["results"][0]["stats_m"]["mean"] - .04) < 1e-6
        report_text = (app.measurement_session.path.parent / "informe.md").read_text(encoding="utf-8")
        assert "Tunel de prueba" in report_text and "Sector norte" in report_text
        assert not errors, errors
        print(f"GTK passed: CSS, operator navigation, {len(page_names)} pages, BASE, multi-stage, resumed BASE-anchor reuse, reports, locks, registration gate, alignment, common ROI, no duplicate actions.")
        app.window.disconnect_by_func(app._on_close)
        app.window.destroy()


if __name__ == "__main__":
    main()
