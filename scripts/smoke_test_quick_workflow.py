"""Integration smoke test for GTK's quick flow using a simulated Aurora.

Run from a Linux desktop session with GTK3, PyGObject, NumPy, and Open3D
installed: ``python3 scripts/smoke_test_quick_workflow.py``.
"""

from __future__ import annotations

import csv
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

import numpy as np
import open3d as o3d

import gui_gtk as gui_module
from gui_gtk import AuroraGUI
from pointcloud_core import save_reference_pose


class FakeConnection:
    pass


def pump_until(predicate, description: str, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(f"Timed out waiting for {description}.")


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="aurora_quick_smoke_") as temp_dir:
        root = Path(temp_dir)
        gui_module.PROJECT_ROOT = root

        app = AuroraGUI(Gtk.Window())
        app.output_dir = str(root / "output")
        app.window.show_all()
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)

        errors: list[tuple] = []
        warnings: list[tuple] = []
        app._show_error = lambda *args: errors.append(args)
        app._show_warning = lambda *args: warnings.append(args)
        app._generate_alerts = lambda: None
        app._show_result_in_viewer = lambda: None
        gui_module.render_reference_photo_from_cloud = lambda _cloud: (None, None)

        gui_module.aurora_sensor.connect = lambda _address: FakeConnection()
        gui_module.aurora_sensor.disconnect = lambda _connection: None
        gui_module.aurora_sensor.capture_reference_frame = lambda *_args, **_kwargs: (None, None)
        gui_module.aurora_sensor.get_current_pose = lambda _connection: SimpleNamespace(
            position=np.zeros(3), rpy_deg=np.zeros(3)
        )

        xy = np.array([(x, y) for x in np.linspace(0, 1, 12) for y in np.linspace(0, 1, 12)])
        capture_count = 0
        capture_gate: threading.Event | None = None

        def fake_capture(_connection, **_kwargs):
            nonlocal capture_count
            if capture_gate is not None:
                capture_gate.wait(timeout=10)
            capture_count += 1
            z = 0.04 * (capture_count - 1)
            points = np.c_[xy, np.full(len(xy), z)]
            return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

        gui_module.aurora_sensor.capture_snapshot = fake_capture

        app.quick_sensor_address_entry.set_text("192.168.11.1")
        app._quick_connect_clicked()
        pump_until(lambda: app.sensor_connection is not None, "sensor connection")
        assert app.quick_capture_base_button.get_sensitive()

        app._capture_clicked("base", quick=True)
        pump_until(
            lambda: app.quick_base_path is not None
            and app.capture_stop_event is None
            and app.quick_capture_updated_button.get_sensitive(),
            "BASE capture",
        )

        for stage_number in (1, 2):
            app._capture_clicked("updated", quick=True)
            pump_until(
                lambda: app.quick_updated_path is not None
                and app.capture_stop_event is None
                and app.quick_capture_updated_button.get_sensitive(),
                f"DESPUES capture {stage_number}",
            )
            stage_path = Path(app.quick_updated_path)
            app._quick_compare_clicked()
            pump_until(
                lambda: len(app.quick_stage_results) == stage_number
                and (app.worker_thread is None or not app.worker_thread.is_alive())
                and app.quick_capture_updated_button.get_sensitive(),
                f"comparison {stage_number}",
            )
            assert (root / "output" / "etapas" / stage_path.stem / "thickness_per_point.csv").is_file()

        with app.quick_session_summary_path.open(encoding="utf-8", newline="") as stream:
            summary = list(csv.DictReader(stream))
        assert [row["etapa"] for row in summary] == ["1", "2"]
        measured_means = [float(row["espesor_medio_cm"]) for row in summary]
        assert np.allclose(measured_means, [4.0, 8.0], atol=1e-4), measured_means

        # A mismatched pose must stop the comparison before another result is added.
        save_reference_pose(Path(app.quick_updated_path), np.array([0.02, 0.0, 0.0]), np.zeros(3))
        app._quick_compare_clicked()
        assert warnings and warnings[-1][0] == "Sensor fuera de posicion"
        assert len(app.quick_stage_results) == 2

        # While capturing a replacement DESPUES, comparison and disconnect are locked.
        save_reference_pose(Path(app.quick_updated_path), np.zeros(3), np.zeros(3))
        capture_gate = threading.Event()
        app._capture_clicked("updated", quick=True)
        assert not app.quick_compare_button.get_sensitive()
        assert not app.quick_connect_button.get_sensitive()
        app._quick_compare_clicked()
        assert warnings[-1][0] == "Captura en curso"
        capture_gate.set()
        pump_until(
            lambda: app.capture_stop_event is None and app.quick_capture_updated_button.get_sensitive(),
            "capture controls restored",
        )

        assert not errors, errors
        print("Quick workflow smoke test passed: connect, BASE, two stages, pose guard, session CSV.")


if __name__ == "__main__":
    main()
