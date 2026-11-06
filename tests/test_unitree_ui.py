import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from gui_gtk import AuroraGUI, Gtk
from pointcloud_core import (
    PipelineParams,
    RaycastPipelineParams,
    run_raycast_pipeline,
    run_pipeline,
    crop_point_cloud,
    estimate_wall_crop_bounds,
    compute_rigid_transform,
    apply_rigid_transform,
)
from viewer_window import _look_at_sensor_pov, _replace_live_geometry
from unitree_sensor import UnitreeConfig, UnitreeCloud
from test_unitree import frame


class SavedWorkflowTests(unittest.TestCase):
    def test_unitree_clouds_run_existing_crop_alignment_analysis_heatmap_and_pdf(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            yy, zz = np.meshgrid(np.linspace(-0.3, 0.3, 16), np.linspace(-0.3, 0.3, 16))
            xyz = np.column_stack([np.ones(yy.size) * 2, yy.ravel(), zz.ravel()])
            before = o3d.geometry.PointCloud()
            before.points = o3d.utility.Vector3dVector(xyz)
            after = o3d.geometry.PointCloud()
            after.points = o3d.utility.Vector3dVector(xyz + np.array([0.02, 0, 0]))
            before = crop_point_cloud(before, (1, -0.2, -0.2), (3, 0.2, 0.2))
            after = crop_point_cloud(after, (1, -0.2, -0.2), (3, 0.2, 0.2))
            rotation, translation = compute_rigid_transform(
                np.asarray(before.points)[[0, 1, -2, -1]],
                np.asarray(after.points)[[0, 1, -2, -1]],
            )
            aligned = apply_rigid_transform(after, rotation, translation)
            np.testing.assert_allclose(
                np.asarray(aligned.points), np.asarray(before.points), atol=1e-8
            )
            # Analyze unaligned clouds so the known 2 cm protrusion remains.
            paths = [root / "before.ply", root / "after.ply"]
            for path, cloud in zip(paths, (before, after)):
                self.assertTrue(o3d.io.write_point_cloud(str(path), cloud))
            result = run_pipeline(
                PipelineParams(paths[0], paths[1], root / "output"), log=lambda _: None
            )
            np.testing.assert_allclose(result.distances, 0.02, atol=1e-8)
            for path in (result.csv_path, result.histogram_path, result.heatmap_path):
                self.assertTrue(path.is_file())
                self.assertGreater(path.stat().st_size, 100)
            ui = Mock()
            ui._active_result.return_value = result
            ui._active_result_kind = "c2c"
            ui.output_dir = str(root / "output")
            ui.base_path, ui.updated_path = map(str, paths)
            ui._selected_color_legend.return_value = {
                "title": "Prueba",
                "continuous_max_cm": 5,
                "low_color": (0, 0, 1),
                "high_color": (1, 0, 0),
            }
            ui.color_banded_rb.get_active.return_value = False
            ui.color_six_bands_rb.get_active.return_value = False
            ui.color_continuous_rb.get_active.return_value = True
            with patch("gui_gtk.Gtk.show_uri_on_window", return_value=True):
                AuroraGUI._generate_report(ui)
            ui._show_error.assert_not_called()
            pdf = next((root / "output").glob("informe_aurora_*.pdf"))
            self.assertTrue(pdf.read_bytes().startswith(b"%PDF"))

    def test_existing_mesh_methods_accept_unitree_x_facing_wall_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            yy, zz = np.meshgrid(np.linspace(-0.3, 0.3, 20), np.linspace(-0.3, 0.3, 20))
            xyz = np.column_stack([np.full(yy.size, 2.0), yy.ravel(), zz.ravel()])
            before = o3d.geometry.PointCloud()
            before.points = o3d.utility.Vector3dVector(xyz)
            after = o3d.geometry.PointCloud()
            after.points = o3d.utility.Vector3dVector(xyz - np.array([0.02, 0, 0]))
            paths = [root / "before.ply", root / "after.ply"]
            for path, cloud in zip(paths, (before, after)):
                o3d.io.write_point_cloud(str(path), cloud)
            for method in ("delaunay", "poisson"):
                with self.subTest(method=method):
                    params = RaycastPipelineParams(
                        paths[0],
                        paths[1],
                        root / method,
                        mesh_method=method,
                        poisson_depth=5,
                        ray_method="normal",
                    )
                    result = run_raycast_pipeline(params, log=lambda _: None)
                    self.assertGreater(len(result.distances), 100)
                    self.assertAlmostEqual(
                        float(np.median(result.distances)), 0.02, delta=0.004
                    )
                    self.assertTrue(result.heatmap_path.is_file())
            np.testing.assert_array_equal(
                np.asarray(o3d.io.read_point_cloud(str(paths[0])).points), xyz
            )

    def test_empty_live_update_removes_previous_wall(self):
        scene = Mock()
        added = _replace_live_geometry(
            scene, frame().xyz, np.ones((4, 3)), object(), False
        )
        self.assertTrue(added)
        self.assertFalse(
            _replace_live_geometry(
                scene, np.empty((0, 3)), np.empty((0, 3)), object(), added
            )
        )
        scene.remove_geometry.assert_called_once_with("live")
        self.assertEqual(scene.add_geometry.call_count, 1)

    def test_wall_detection_and_camera_use_unitree_x_axis_without_moving_points(self):
        yy, zz = np.meshgrid(np.linspace(-0.2, 0.2, 20), np.linspace(-0.2, 0.2, 20))
        xyz = np.column_stack([np.ones(yy.size) * 2, yy.ravel(), zz.ravel()])
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)
        seed, bounds = estimate_wall_crop_bounds(cloud, cloud, forward_axis="x")
        self.assertEqual(np.argmin(np.array(bounds[1]) - bounds[0]), 0)
        self.assertAlmostEqual(seed[0], 2)
        np.testing.assert_array_equal(np.asarray(cloud.points), xyz)
        scene = Mock()
        _look_at_sensor_pov(scene, xyz, forward_axis="x")
        target, eye, up = scene.look_at.call_args.args
        self.assertLess(eye[0], target[0])
        np.testing.assert_array_equal(up, [0, 0, 1])


class GtkSelectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not Gtk.init_check([])[0]:
            raise unittest.SkipTest("Sin pantalla GTK")

    def test_switching_and_capture_save_preserve_native_fields(self):
        window = Gtk.Window()
        app = AuroraGUI(window)
        try:
            app.sensor_selector.set_active_id("unitree")
            self.assertEqual(app.capture_sensor_selector.get_active_id(), "unitree")
            self.assertEqual(app._cloud_forward_axis(), "x")
            self.assertFalse(app.quick_imu_compensate_check.get_active())
            self.assertIn("sin posicion SLAM", app.quick_imu_status_label.get_text())
            app._refresh_quick_status()
            self.assertIn("sin posicion SLAM", app.quick_imu_status_label.get_text())
            self.assertFalse(app.landmark_base_source_photo_rb.get_sensitive())
            self.assertFalse(app.load_base_photo_button.get_sensitive())
            app._show_sensor_notice("Sobrecarga de prueba: 2 nubes descartadas")
            self.assertIn("2 nubes", app.quick_sensor_notice_label.get_text())
            self.assertIn("2 nubes", app.live_notice_label.get_text())
            self.assertFalse(app.experimental_mobile_base_button.get_sensitive())
            self.assertEqual(app.capture_forward_axis_combo.get_active_text(), "x")
            config = app._unitree_config()
            self.assertEqual(config.cloud_scan_num, 18)
            self.assertEqual(app.live_decay_entry.get_text(), "1")
            cloud = UnitreeCloud([frame(2)], config)
            with tempfile.TemporaryDirectory() as d:
                path = Path(d) / "unitree.ply"
                with patch.object(app, "_save_file_dialog", return_value=str(path)):
                    app._on_capture_done("base", cloud)
                self.assertTrue(path.exists())
                with np.load(path.with_name("unitree_unitree.npz")) as saved:
                    np.testing.assert_array_equal(saved["fields"], cloud.unitree_fields)
                self.assertFalse(path.with_name("unitree_pose.npz").exists())
                self.assertFalse(path.with_name("unitree_ref.npz").exists())
            app.capture_sensor_selector.set_active_id("aurora")
            self.assertEqual(app.sensor_selector.get_active_id(), "aurora")
            self.assertEqual(app.capture_forward_axis_combo.get_active_text(), "z")
        finally:
            app._on_close()
            window.destroy()


if __name__ == "__main__":
    unittest.main()
