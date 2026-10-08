"""Integration smoke test for CustomTkinter using a simulated Aurora.

Run from a Windows desktop session after installing requirements:
``python scripts/smoke_test_quick_workflow_tk.py``.
"""

from __future__ import annotations

import csv
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import open3d as o3d

import gui as gui_module
from gui import AuroraGUI, ctk
from pointcloud_core import save_reference_pose


class FakeConnection:
    pass


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="aurora_tk_quick_smoke_") as temp_dir:
        root_dir = Path(temp_dir)
        gui_module.PROJECT_ROOT = root_dir
        root = ctk.CTk()
        app = AuroraGUI(root)
        app.output_dir.set(str(root_dir / "output"))
        errors: list[tuple] = []
        warnings: list[tuple] = []
        app._show_result_in_viewer = lambda: None
        app._push_static_result_to_viewer = lambda: None
        gui_module.render_reference_photo_from_cloud = lambda _cloud: (None, None)

        import tkinter.messagebox as messagebox

        messagebox.showerror = lambda *args: errors.append(args)
        messagebox.showwarning = lambda *args: warnings.append(args)
        gui_module.aurora_sensor.connect = lambda _address: FakeConnection()
        gui_module.aurora_sensor.disconnect = lambda _connection: None
        gui_module.aurora_sensor.capture_reference_frame = lambda *_args, **_kwargs: (None, None)
        current_position = np.zeros(3)
        gui_module.aurora_sensor.get_current_pose = lambda _connection: SimpleNamespace(
            position=current_position.copy(), rpy_deg=np.zeros(3)
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

        failures: list[BaseException] = []

        def wait_for(predicate, description: str, then) -> None:
            def poll() -> None:
                try:
                    if predicate():
                        then()
                    else:
                        root.after(20, poll)
                except BaseException as exc:
                    failures.append(AssertionError(f"{description}: {exc}"))
                    root.quit()

            root.after(20, poll)

        def finish() -> None:
            try:
                with app.quick_session_summary_path.open(encoding="utf-8", newline="") as stream:
                    summary = list(csv.DictReader(stream))
                assert [row["etapa"] for row in summary] == ["1", "2"]
                means = [float(row["espesor_medio_cm"]) for row in summary]
                assert np.allclose(means, [4.0, 8.0], atol=1e-4), means
                save_reference_pose(Path(app.quick_updated_path), np.array([0.02, 0.0, 0.0]), np.zeros(3))
                app._quick_compare_clicked()
                assert warnings and warnings[-1][0] == "Sensor fuera de posicion"
                assert len(app.quick_stage_results) == 2
                save_reference_pose(Path(app.quick_updated_path), np.zeros(3), np.zeros(3))
                nonlocal capture_gate
                capture_gate = threading.Event()
                app._capture_clicked("updated", quick=True)
                assert app.quick_compare_button.cget("state") == "disabled"
                assert app.quick_connect_button.cget("state") == "disabled"
                app._quick_compare_clicked()
                assert warnings[-1][0] == "Captura en curso"
                capture_gate.set()
                wait_for(
                    lambda: not app.capture_in_progress
                    and app.quick_capture_updated_button.cget("state") == "normal",
                    "capture controls restored",
                    verify,
                )
            except BaseException as exc:
                failures.append(exc)
                root.quit()

        def verify() -> None:
            if errors:
                failures.append(AssertionError(errors))
            if failures:
                root.quit()
                return
            print("CustomTkinter quick workflow smoke test passed: two stages, pose guard, session CSV.")
            root.quit()

        def capture_stage(stage_number: int) -> None:
            try:
                app._capture_clicked("updated", quick=True)
                wait_for(
                    lambda: app.quick_updated_path is not None
                    and not app.capture_in_progress
                    and app.quick_capture_updated_button.cget("state") == "normal",
                    f"DESPUES capture {stage_number}",
                    lambda: compare_stage(stage_number),
                )
            except BaseException as exc:
                failures.append(exc)
                root.quit()

        def compare_stage(stage_number: int) -> None:
            try:
                stage_path = Path(app.quick_updated_path)
                app._quick_compare_clicked()
                wait_for(
                    lambda: len(app.quick_stage_results) == stage_number
                    and (app.worker_thread is None or not app.worker_thread.is_alive())
                    and app.quick_capture_updated_button.cget("state") == "normal",
                    f"comparison {stage_number}",
                    lambda: next_stage(stage_number, stage_path),
                )
            except BaseException as exc:
                failures.append(exc)
                root.quit()

        def next_stage(stage_number: int, stage_path: Path) -> None:
            try:
                assert (root_dir / "output" / "etapas" / stage_path.stem / "thickness_per_point.csv").is_file()
                if stage_number < 2:
                    capture_stage(stage_number + 1)
                else:
                    finish()
            except BaseException as exc:
                failures.append(exc)
                root.quit()

        def after_base() -> None:
            try:
                assert app.quick_capture_base_button.cget("state") == "normal"
                app._show_quick_pose_delta()
                assert "LISTO" in app.quick_pose_status.cget("text")
                current_position[0] = 0.02
                app._show_quick_pose_delta()
                assert "AJUSTAR" in app.quick_pose_status.cget("text")
                current_position[0] = 0.0
                capture_stage(1)
            except BaseException as exc:
                failures.append(exc)
                root.quit()

        def after_connection() -> None:
            try:
                assert app.quick_capture_base_button.cget("state") == "normal"
                app._capture_clicked("base", quick=True)
                wait_for(
                    lambda: app.quick_base_path is not None
                    and not app.capture_in_progress
                    and app.quick_capture_updated_button.cget("state") == "normal",
                    "BASE capture",
                    after_base,
                )
            except BaseException as exc:
                failures.append(exc)
                root.quit()

        def start_connection() -> None:
            try:
                app._toggle_sensor_connection()
                wait_for(lambda: app.sensor_connection is not None, "sensor connection", after_connection)
            except BaseException as exc:
                failures.append(exc)
                root.quit()

        root.after(10, start_connection)
        root.mainloop()
        app._on_close()
        if failures:
            raise failures[0]


if __name__ == "__main__":
    main()
