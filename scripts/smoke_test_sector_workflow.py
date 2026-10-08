"""GTK integration: every page, synthetic captures, resume, reports, field gate."""
from __future__ import annotations
import tempfile
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
        module.PROJECT_ROOT = root
        app = module.AuroraGUI(Gtk.Window())
        app.window.show_all()
        errors, warnings = [], []
        app._show_error = lambda *args: errors.append(args)
        app._show_warning = lambda *args: warnings.append(args)
        app._show_info = lambda *args: None
        module.render_reference_photo_from_cloud = lambda _: (None, None)
        assert app.stack.get_visible_child_name() == "rapido"
        assert app.run_button.get_visible()
        assert app._sidebar_rows["captura"].get_visible()
        assert app._sidebar_rows["prueba_raycasting"].get_visible()
        for page in app._sidebar_rows:
            app.stack.set_visible_child_name(page)
            pump_until(lambda: not Gtk.events_pending(), "GTK navigation")
            assert app.stack.get_visible_child_name() == page
        app.stack.set_visible_child_name("rapido")

        app.quick_demo_check.set_active(True)
        xy = np.array([(x, y) for x in np.linspace(0, 1, 12) for y in np.linspace(0, 1, 12)])
        import_before, import_after = root / "import_before.ply", root / "import_after.ply"
        clouds = [o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[xy, np.full(len(xy), z)])) for z in (0, .04)]
        o3d.io.write_point_cloud(str(import_before), clouds[0])
        o3d.io.write_point_cloud(str(import_after), clouds[1])
        selected_files = iter((str(import_before), str(import_after)))
        app._run_open_file_dialog = lambda: next(selected_files)
        app._session_import("base")
        pump_until(lambda: app.measurement_session.data["base"] and not app._session_busy(), "loaded BEFORE")
        app._session_import("updated")
        pump_until(lambda: len(app.measurement_session.data["stages"]) == 1 and not app._session_busy(), "loaded AFTER")
        app._quick_compare_clicked()
        pump_until(lambda: len(app.measurement_session.data["results"]) == 1 and not app._session_busy(), "loaded pair comparison")
        assert abs(app.measurement_session.data["results"][0]["stats_m"]["mean"] - .04) < 1e-6

        module.aurora_sensor.connect = lambda _: object()
        module.aurora_sensor.disconnect = lambda _: None
        module.aurora_sensor.capture_reference_frame = lambda *args, **kwargs: (None, None)
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
        module.aurora_sensor.capture_snapshot = capture
        app._quick_connect_clicked()
        pump_until(lambda: app.sensor_connection is not None, "connect")
        app._quick_capture_clicked("base")
        pump_until(lambda: app.measurement_session.data["base"] and not app._session_busy(), "BASE")
        assert app.quick_capture_base_button.get_sensitive()
        for stage in (1, 2):
            app._quick_capture_clicked("updated")
            pump_until(lambda: len(app.measurement_session.data["stages"]) == stage and not app._session_busy(), "stage")
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
        assert "ENSAYO" in app.quick_result_label.get_text()
        assert (session_path.parent / "informe.md").is_file()
        assert (session_path.parent / "resumen.csv").is_file()
        assert app.measurement_session.data["base"]["pose"]["sensor_timestamp_ns"] == 1234

        gate.clear()
        app._capture_clicked("updated", quick=True)
        assert not app.quick_compare_button.get_sensitive()
        assert not app.session_open_button.get_sensitive()
        app._quick_compare_clicked()
        assert warnings[-1][0] == "Trabajo en curso"
        assert app._on_delete() is True
        assert warnings[-1][0] == "Guardado en curso"
        gate.set()
        pump_until(lambda: len(app.measurement_session.data["stages"]) == 3 and not app._session_busy(), "capture controls")
        app.quick_demo_check.set_active(False)
        app._quick_capture_clicked("base")
        pump_until(lambda: app.measurement_session.data["base"] and not app._session_busy(), "field BASE")
        app._quick_capture_clicked("updated")
        pump_until(lambda: len(app.measurement_session.data["stages"]) == 1 and not app._session_busy(), "field stage")
        app.session_mode_combo.set_active(0)
        app._quick_compare_clicked()
        pump_until(lambda: not app._session_busy(), "field guard")
        assert not app.measurement_session.data["results"]
        assert "Faltan referencias" in app.quick_result_label.get_text()
        app._session_align_selected()
        anchors = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=float)
        app.landmarks_base = anchors
        app.landmarks_updated = anchors.copy()
        app._apply_alignment()
        assert warnings[-1][0] == "Referencias sin confirmar"
        app.session_stable_anchors_check.set_active(True)
        app._apply_alignment()
        pump_until(lambda: app.measurement_session.data["stages"][0]["registration"] and not app._session_busy(), "manual anchor registration")
        app._session_crop_selected()
        app.segmentation_quad = np.array([[.1, .1, .16], [.9, .1, .16], [.9, .9, .16], [.1, .9, .16]])
        app.segmentation_width_spin.set_value(100)
        app._apply_segmentation()
        pump_until(lambda: app.measurement_session.data.get("roi") and not app._session_busy(), "common sector crop")
        app.stack.set_visible_child_name("rapido")
        app.session_surface_check.set_active(True)
        app._quick_compare_clicked()
        pump_until(lambda: len(app.measurement_session.data["results"]) == 1 and not app._session_busy(), "registered sector comparison")
        assert app.measurement_session.data["results"][0]["status"] == "exploratorio_no_validado"
        assert abs(app.measurement_session.data["results"][0]["stats_m"]["mean"] - .04) < 1e-6
        assert not errors, errors
        print("GTK passed: CSS, 10 pages, navigation, BASE, multi-stage, pairwise change, resume, artifacts, locks, field gate, manual alignment, common sector ROI.")
        app._stop_imu_poll()
        app.window.disconnect_by_func(app._on_close)
        app.window.destroy()


if __name__ == "__main__":
    main()
