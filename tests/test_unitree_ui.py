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
    crop_pair_by_face_depth,
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

    def test_planar_method_can_be_selected_and_retained_from_both_pages(self):
        window = Gtk.Window()
        app = AuroraGUI(window)
        try:
            for previous in ('raycast', 'cloud_to_cloud'):
                app._set_calculation_method(previous)
                app.quick_distance_method_combo.set_active_id('plane')
                self.assertEqual(app.calculation_method, 'plane')
                self.assertTrue(app.result_method_plane_rb.get_active())
                self.assertFalse(app.quick_mesh_check.get_active())
                self.assertEqual(app._build_params().distance_method, 'plane')
            app.result_method_cloud_rb.set_active(True)
            self.assertEqual(app.quick_distance_method_combo.get_active_id(), 'cloud_to_cloud')
            app.result_method_plane_rb.set_active(True)
            self.assertEqual(app.quick_distance_method_combo.get_active_id(), 'plane')
        finally:
            app._on_close()
            window.destroy()

    def test_box_dimensions_keep_native_axes_across_calculation_methods(self):
        window = Gtk.Window()
        app = AuroraGUI(window)
        try:
            app.quick_crop_face_size_entry.set_text('20')
            app.quick_crop_height_entry.set_text('30')
            app.quick_crop_depth_entry.set_text('100')
            scene = o3d.geometry.PointCloud(o3d.utility.Vector3dVector([[1., 0., .2]]))
            with patch.object(app, '_preview_test_crop_bounds'):
                app._on_test_crop_wall_detected(np.array([1., 0., .2]),
                                               ((1., -.1, 0.), (1., .1, .4)), 'automático', None)
            original = np.array(app.test_crop_bounds)
            for method in ('cloud_to_cloud', 'plane', 'raycast', 'plane'):
                app.quick_distance_method_combo.set_active_id(method)
                np.testing.assert_array_equal(app.test_crop_bounds, original)
                self.assertEqual(app.quick_crop_width_label.get_text(), 'Ancho Y (cm):')
                self.assertEqual(app.quick_crop_height_label.get_text(), 'Alto Z (cm):')
                self.assertEqual(app.quick_crop_depth_label.get_text(), 'Profundidad X (cm):')
                _, _, _, box, axis, _ = crop_pair_by_face_depth(
                    scene, scene, app.test_crop_bounds, app._get_test_crop_depth_m(), app.test_crop_seed
                )
                self.assertEqual(axis, 0)
                np.testing.assert_allclose(np.subtract(box[1], box[0]), [1., .2, .3])
            app.quick_crop_height_entry.set_text('45')
            self.assertEqual(app.quick_crop_depth_entry.get_text(), '100')
            np.testing.assert_allclose(np.subtract(app.test_crop_bounds[1], app.test_crop_bounds[0]), [0., .2, .45])
        finally:
            app._on_close()
            window.destroy()

    def test_switching_and_capture_save_preserve_native_fields(self):
        window = Gtk.Window()
        app = AuroraGUI(window)
        try:
            app.sensor_selector.set_active_id("unitree")
            self.assertEqual(app.capture_sensor_selector.get_active_id(), "unitree")
            self.assertEqual(app._cloud_forward_axis(), "x")
            self.assertFalse(app.quick_imu_compensate_check.get_active())
            self.assertIn("Preparando el seguimiento", app.quick_imu_status_label.get_text())
            app._refresh_quick_status()
            self.assertIn("Preparando el seguimiento", app.quick_imu_status_label.get_text())
            self.assertFalse(app.landmark_base_source_photo_rb.get_sensitive())
            self.assertFalse(app.load_base_photo_button.get_sensitive())
            app._show_sensor_notice("Sobrecarga de prueba: 2 nubes descartadas")
            self.assertIn("2 nubes", app.quick_sensor_notice_label.get_text())
            self.assertIn("2 nubes", app.live_notice_label.get_text())
            self.assertFalse(app.experimental_mobile_base_button.get_sensitive())
            self.assertEqual(app.capture_forward_axis_combo.get_active_text(), "x")
            config = app._unitree_config()
            self.assertEqual(config.cloud_scan_num, 18)
            self.assertEqual(app.live_decay_entry.get_text(), "5")
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

    def test_persistent_scene_survives_navigation_and_first_pcd_capture_sets_origin(self):
        from unitree_pose import PoseHistory
        window = Gtk.Window()
        app = AuroraGUI(window)
        reference = PoseHistory._pose(dict(
            position=[1, 2, 3], quaternion_xyzw=[0, 0, 0, 1],
            stamp_ns=1, frame_id="unitree_lio_origin_ui_test",
        ))
        try:
            panel = app.scene_panel
            parent = panel.get_parent()
            for page in ("configuracion", "captura", "alineacion_imu", "resultados", "medicion_rapida"):
                app.stack.set_visible_child_name(page)
                self.assertIs(panel.get_parent(), parent)
                self.assertTrue(app.sidebar_revealer.get_reveal_child())
                self.assertIsNotNone(app.stack.get_child_by_name(page))
            cloud = UnitreeCloud([frame(2)], app._unitree_config())
            with tempfile.TemporaryDirectory() as directory:
                for index, delta in enumerate((0, 1)):
                    target = Path(directory) / f"capture_{index}.pcd"
                    later = PoseHistory._pose(dict(
                        position=[1 + delta, 2, 3], quaternion_xyzw=[0, 0, 0, 1],
                        stamp_ns=2, frame_id=reference.session_id,
                    ))
                    with patch.object(app, "_save_file_dialog", return_value=str(target)):
                        app._on_capture_done("base", cloud, ref_pose=later)
                    self.assertTrue(target.is_file())
                    np.testing.assert_allclose(np.asarray(o3d.io.read_point_cloud(str(target)).points),
                                               np.asarray(cloud.points), atol=1e-6)
                    with np.load(target.with_name(target.stem + "_unitree.npz")) as saved:
                        np.testing.assert_array_equal(saved["fields"], cloud.unitree_fields)
                    np.testing.assert_allclose(panel.feed.reference.position, later.position)
            for view in ("Órbita", "Superior", "Frontal", "Lateral"):
                panel.canvas.view(view)
                self.assertTrue(np.isfinite(panel.canvas._matrix(1.5)).all())
            previous = panel.canvas.distance
            panel.canvas.zoom(.8)
            self.assertLess(panel.canvas.distance, previous)
        finally:
            app._on_close()
            window.destroy()

    def test_before_keeps_visual_start_when_average_capture_pose_is_unavailable(self):
        from unitree_pose import PoseHistory
        from types import SimpleNamespace as NS
        window = Gtk.Window()
        app = AuroraGUI(window)
        start = PoseHistory._pose(dict(position=[1, 2, 3], quaternion_xyzw=[0, 0, 0, 1],
                                      stamp_ns=1, frame_id="unitree_lio_origin_visual_test"))
        moved = PoseHistory._pose(dict(position=[1.5, 2, 3], quaternion_xyzw=[0, 0, 0, 1],
                                      stamp_ns=2, frame_id=start.session_id))
        try:
            cloud = UnitreeCloud([frame(2)], app._unitree_config())
            with tempfile.TemporaryDirectory() as directory:
                target = Path(directory) / "before.pcd"
                app.scene_panel.pause_button.set_active(True)
                with patch.object(app, "_save_file_dialog", return_value=str(target)):
                    app._on_capture_done("base", cloud, ref_pose=None, visual_reference_pose=start)
                self.assertIs(app.scene_panel.feed.reference, start)
                self.assertFalse(app.scene_panel.pause_button.get_active())
                self.assertFalse(target.with_name("before_pose.npz").exists())
                self.assertEqual(app.imu_reference_pose.session_id, start.session_id)
                connection = NS(pose_history=NS(smoothed=lambda: moved), read_registered_batch=lambda: [])
                app.scene_panel.feed.set_connection(connection)
                app.scene_panel.feed._poll(connection)
                state = app.scene_panel.feed.snapshot()
                np.testing.assert_allclose(state["reference"], [0, 0, 0])
                np.testing.assert_allclose(state["position"], [.5, 0, 0])
                self.assertIs(app.scene_panel.feed.reference, start)
                next_target = Path(directory) / "next_before.pcd"
                with patch.object(app, "_save_file_dialog", return_value=str(next_target)):
                    app._on_capture_done("base", cloud, ref_pose=None, visual_reference_pose=moved)
                self.assertIs(app.scene_panel.feed.reference, moved)
                np.testing.assert_allclose(app.scene_panel.feed.snapshot()["position"], [0, 0, 0])
                np.testing.assert_allclose(app.imu_reference_pose[0], moved.position)
                app.scene_panel.feed.set_connection(None)
        finally:
            app._on_close()
            window.destroy()


if __name__ == "__main__":
    unittest.main()
