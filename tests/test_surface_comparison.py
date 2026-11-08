import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from surface_comparison import compare_surfaces, SurfaceParams
from pointcloud_core import (PipelineParams, run_pipeline, save_reference_pose,
                             ReferencePose, build_surface_view)


def face(x, width=.5, height=.5):
    y, z = np.meshgrid(np.linspace(-width/2, width/2, 30), np.linspace(-height/2, height/2, 30))
    return np.column_stack((np.full(y.size, x), y.ravel(), z.ravel()))


def tunnel(radius=1.5):
    theta, y = np.meshgrid(np.linspace(0, np.pi, 180), np.linspace(-.3, .3, 34))
    return theta, y, np.column_stack(((radius*np.cos(theta)).ravel(), y.ravel(), (radius*np.sin(theta)).ravel()))


def used(result, minimum=.01):
    return result.valid & (abs(result.signed) > result.variation) & (abs(result.signed) >= minimum)


class SurfaceComparisonTests(unittest.TestCase):
    def test_parallel_faces_with_floor_near_far_and_arbitrary_orientation(self):
        x, y = np.meshgrid(np.linspace(.2, 1.7, 60), np.linspace(-.35, .35, 32))
        floor = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.3)))
        for distance, angles in ((1.35, (0,0,0)), (.85, (0,0,0)), (1.35, (30,50,20))):
            rotation = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
            before = np.vstack((face(distance), floor)) @ rotation.T
            after = np.vstack((face(distance-.3), floor)) @ rotation.T
            originals = before.copy(), after.copy()
            comparison = compare_surfaces(before, after)
            mask = used(comparison)
            self.assertGreater(mask.sum(), 200)
            self.assertAlmostEqual(float(np.median(abs(comparison.signed[mask]))), .3, delta=.005)
            self.assertLess(mask[len(face(distance)):].sum(), 5)
            self.assertGreater((comparison.method[mask] == 1).sum(), 100)
            np.testing.assert_array_equal(before, originals[0]);np.testing.assert_array_equal(after, originals[1])

    def test_curved_tunnel_uses_local_normals_for_thirty_cm(self):
        _, _, before = tunnel(1.5)
        _, _, after = tunnel(1.2)
        comparison = compare_surfaces(before, after)
        mask = used(comparison)
        self.assertGreater(comparison.valid.mean(), .65)
        self.assertAlmostEqual(float(np.median(abs(comparison.signed[mask]))), .3, delta=.008)
        self.assertTrue(np.all(comparison.method[mask] == 2))

    def test_local_roof_change_is_not_diluted_by_unchanged_arc(self):
        theta, y, before = tunnel()
        radius = np.full(theta.shape, 1.5)
        roof = (abs(theta - np.pi/2) < .14) & (abs(y) < .12)
        radius[roof] -= .03
        after = np.column_stack(((radius*np.cos(theta)).ravel(), y.ravel(), (radius*np.sin(theta)).ravel()))
        comparison = compare_surfaces(before, after)
        mask = used(comparison)
        self.assertGreater(mask.sum(), 60)
        self.assertGreater(np.mean(roof.ravel()[mask]), .95)
        self.assertAlmostEqual(float(np.median(abs(comparison.signed[mask]))), .03, delta=.005)

    def test_small_wall_protrusion_is_measured_without_averaging_the_unchanged_wall(self):
        y, z = np.meshgrid(np.linspace(-.5, .5, 65), np.linspace(-.5, .5, 65))
        before = np.column_stack((np.full(y.size, 1.5), y.ravel(), z.ravel()))
        after = before.copy()
        patch = (abs(y.ravel()) < .15) & (abs(z.ravel()) < .15)
        after[patch, 0] -= .03
        comparison = compare_surfaces(before, after)
        mask = used(comparison)
        self.assertGreater(mask.sum(), 80)
        self.assertGreater(np.mean(patch[mask]), .98)
        self.assertAlmostEqual(float(np.median(abs(comparison.signed[mask]))), .03, delta=.004)

    def test_unchanged_noisy_scene_has_no_significant_change(self):
        rng = np.random.default_rng(55)
        before = face(1.3) + rng.normal(0, .002, (900,3))
        after = face(1.3) + rng.normal(0, .002, (900,3))
        comparison = compare_surfaces(before, after)
        self.assertGreater(comparison.valid.mean(), .6)
        self.assertLess(used(comparison).sum(), 5)

    def test_repeated_measurement_has_identical_surface_choices_and_results(self):
        rng = np.random.default_rng(41)
        before = face(1.3) + rng.normal(0,.003,(900,3))
        after = face(1.) + rng.normal(0,.003,(900,3))
        first = compare_surfaces(before,after)
        second = compare_surfaces(before,after)
        np.testing.assert_array_equal(first.valid,second.valid)
        np.testing.assert_array_equal(first.method,second.method)
        np.testing.assert_array_equal(first.quality,second.quality)
        np.testing.assert_allclose(first.signed,second.signed,atol=1e-12,equal_nan=True)

    def test_parallel_alternatives_are_ambiguous_and_unrelated_faces_gray(self):
        before = np.vstack((face(1.), face(1.3)))
        after = face(.85)
        comparison = compare_surfaces(before, after)
        self.assertLess(comparison.valid.mean(), .1)
        self.assertGreater(np.count_nonzero(comparison.reason == 2), 300)
        comparison = compare_surfaces(face(1.3), face(1.) + [0, 1., 0])
        self.assertFalse(comparison.valid.any())
        gray = build_surface_view(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(face(1.)+[0,1.,0])),
                                  comparison, np.zeros(900, bool), np.empty(0))
        np.testing.assert_allclose(np.asarray(gray.colors), np.tile([.58,.60,.63], (len(gray.points),1)))

    def test_holes_are_recorded_in_both_directions(self):
        complete = face(1.3, .7, .7)
        hole = (abs(complete[:,1]) < .12) & (abs(complete[:,2]) < .12)
        comparison = compare_surfaces(complete[~hole], complete)
        self.assertLess(comparison.valid[hole].mean(), .5)
        comparison = compare_surfaces(complete, complete[~hole])
        missing = comparison.unmatched_before
        self.assertGreater(np.count_nonzero((abs(missing[:,1]) < .08)&(abs(missing[:,2]) < .08)), 10)

    def test_invalid_parameters_are_rejected(self):
        for cfg in (SurfaceParams(search_depth_m=0), SurfaceParams(projection_radius_m=.01),
                    SurfaceParams(registration_allowance_m=-1)):
            with self.assertRaises(ValueError):
                compare_surfaces(face(1.3), face(1.), cfg)

    def test_pipeline_pose_alignment_exports_and_zero_state(self):
        before = face(1.3)
        rotation = Rotation.from_euler('xyz', [8, -12, 18], degrees=True)
        translation = np.array([.25, -.06, .04])
        native_after = (before - translation) @ rotation.as_matrix()
        base_pose = ReferencePose(np.zeros(3), np.zeros(3), np.eye(3), 'test', 'same_session')
        # Existing pose API uses intrinsic XYZ Euler angles.
        after_pose = ReferencePose(translation, rotation.as_euler('XYZ', degrees=True), np.eye(3), 'test', 'same_session')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = [root/'before.ply', root/'after.ply']
            for path, points, pose in zip(paths, (before, native_after), (base_pose, after_pose)):
                o3d.io.write_point_cloud(str(path), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
                save_reference_pose(path, *pose, source_pose=pose)
            original_files = [p.read_bytes() for p in paths]
            params = PipelineParams(*paths, root/'out', distance_method='auto', align_by_pose=True, minimum_thickness_m=.01)
            result = run_pipeline(params, log=lambda _: None)
            self.assertEqual(result.stats.n_points, 0)
            self.assertGreater(result.measurement_details['coverage_percent'], 60)
            self.assertEqual(result.stats.median, 0)
            self.assertGreater(len(result.heatmap_cloud.points), 0)
            self.assertTrue((params.output_dir/'surface_comparison.npz').is_file())
            with (params.output_dir/'surface_comparison_per_point.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(sum(row['source']=='after' for row in rows), len(native_after))
            self.assertTrue(any(row['status']=='within_variation' for row in rows))
            details = json.loads((params.output_dir/'measurement_details.json').read_text())
            self.assertTrue(details['pose_alignment_requested'])
            for path, data in zip(paths, original_files):self.assertEqual(path.read_bytes(),data)
            params.use_icp = True
            with self.assertRaisesRegex(ValueError, 'ICP'):run_pipeline(params, log=lambda _: None)
            params.use_icp = False
            incompatible = ReferencePose(translation, after_pose[1], np.eye(3), 'test', 'another_session')
            save_reference_pose(paths[1], *incompatible, source_pose=incompatible)
            with self.assertRaisesRegex(ValueError, 'sesiones'):run_pipeline(params, log=lambda _: None)


if __name__ == '__main__':unittest.main()
