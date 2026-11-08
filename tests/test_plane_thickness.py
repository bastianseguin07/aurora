import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from plane_thickness import planar_thickness
from pointcloud_core import PipelineParams, run_pipeline


def face(x):
    y, z = np.meshgrid(np.linspace(-.1, .1, 25), np.linspace(.03, .23, 25))
    return np.column_stack((np.full(y.size, x), y.ravel(), z.ravel()))


def floor_points():
    x, y = np.meshgrid(np.linspace(.95, 1.55, 25), np.linspace(-.12, .12, 7))
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))


class PlanarThicknessTests(unittest.TestCase):
    def test_floor_shortcuts_c2c_but_perpendicular_measurement_keeps_thirty_cm(self):
        before = np.vstack((face(1.35), floor_points()))
        after = np.vstack((face(1.05), floor_points()))
        original_before, original_after = before.copy(), after.copy()
        self.assertLess(np.median(cKDTree(before).query(face(1.05))[0]), .2)
        # Simulate the same target near and far, including walls/ceilings with
        # arbitrary orientation. No special X axis or distance correction.
        for angles in ((0, 0, 0), (30, 50, 70), (0, 90, 0)):
            rotation = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
            for offset in (np.zeros(3), np.array([4., 8., -2.])):
                with self.subTest(angles=angles, offset=offset):
                    indices, distances, details = planar_thickness(
                        before @ rotation.T + offset, after @ rotation.T + offset
                    )
                    np.testing.assert_allclose(distances, .3, atol=1e-8)
                    self.assertGreater(len(indices), 500)
                    self.assertGreater(details['rejected_geometry_points'], 0)
        near_indices, near, _ = planar_thickness(face(.85), face(.55))
        np.testing.assert_allclose(near, .3, atol=1e-8)
        np.testing.assert_array_equal(before, original_before)
        np.testing.assert_array_equal(after, original_after)

    def test_noise_is_preserved_in_per_point_distances(self):
        rng = np.random.default_rng(34)
        before, after = face(1.35), face(1.05)
        before += rng.normal(0, .003, before.shape)
        after += rng.normal(0, .003, after.shape)
        indices, distances, _ = planar_thickness(before, after)
        self.assertAlmostEqual(float(np.median(distances)), .3, delta=.005)
        self.assertGreater(np.std(distances), .001)
        self.assertLess(len(indices), len(after))  # no extrapolation at edges

    def test_rejects_unrelated_faces_and_insufficient_overlap(self):
        tilted = (face(1.05) - [1.05, 0, .13]) @ Rotation.from_euler('z', 30, degrees=True).as_matrix().T + [1.05, 0, .13]
        with self.assertRaisesRegex(ValueError, 'difieren'):
            planar_thickness(face(1.35), tilted)
        with self.assertRaisesRegex(ValueError, 'zona común'):
            planar_thickness(face(1.35), face(1.05) + [0, 2, 0])
        with self.assertRaisesRegex(ValueError, '30 puntos'):
            planar_thickness(face(1.35)[:20], face(1.05))
        mixed = np.vstack((face(1.35), face(2.), face(2.65)))
        with self.assertRaisesRegex(ValueError, 'dominante'):
            planar_thickness(mixed, face(1.05))

    def test_pipeline_exports_only_supported_face_points_and_records_method(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = [root / 'before.ply', root / 'after.ply']
            for path, points in zip(paths, (np.vstack((face(1.35), floor_points())),
                                           np.vstack((face(1.05), floor_points())))):
                cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
                o3d.io.write_point_cloud(str(path), cloud)
            params = PipelineParams(*paths, root / 'out', distance_method='plane', minimum_thickness_m=.01)
            result = run_pipeline(params, log=lambda _: None)
            self.assertEqual(result.distance_method, 'plane')
            self.assertAlmostEqual(result.stats.median, .3, places=7)
            self.assertEqual(len(result.distances), len(result.updated_cloud.points))
            self.assertEqual(len(result.distances), len(result.heatmap_cloud.points))
            csv = np.genfromtxt(result.csv_path, delimiter=',', names=True)
            np.testing.assert_allclose(csv['distance_m'], .3, atol=1e-6)
            details = json.loads((params.output_dir / 'measurement_details.json').read_text())
            self.assertEqual(details['distance_method'], 'plane')
            self.assertGreater(details['geometry']['rejected_geometry_points'], 0)
            params.use_icp = True
            with self.assertRaisesRegex(ValueError, 'ICP'):
                run_pipeline(params, log=lambda _: None)


if __name__ == '__main__':
    unittest.main()
