import sys
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from pointcloud_core import (estimate_wall_crop_bounds, rectangular_face_bounds,
                             crop_pair_by_face_depth, crop_cloud_by_face_depth)


def cloud(points):
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))


class CropAxesTests(unittest.TestCase):
    def test_frontal_wall_keeps_depth_axis_when_near_floor_dominates(self):
        y, z = np.meshgrid(np.linspace(-.2, .2, 30), np.linspace(.04, .4, 30))
        wall = np.column_stack((np.ones(y.size), y.ravel(), z.ravel()))
        x, y = np.meshgrid(np.linspace(.15, 1.6, 90), np.linspace(-.4, .4, 90))
        floor = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
        native = np.vstack((wall, floor))
        for forward, transform in (('x', np.eye(3)), ('z', np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]]))):
            with self.subTest(forward=forward):
                points = native @ transform.T
                original = points.copy()
                scene = cloud(points)
                for random_seed in (0, 4, 17):
                    o3d.utility.random.seed(random_seed)
                    seed, face = estimate_wall_crop_bounds(scene, scene, forward_axis=forward)
                    axis = 'xyz'.index(forward)
                    self.assertEqual(np.argmin(np.subtract(face[1], face[0])), axis)
                    self.assertAlmostEqual(seed[axis], 1., places=6)
                    face = rectangular_face_bounds(face, seed, .2, .3)
                    for _ in range(2):
                        _, _, face, bounds, actual_axis, _ = crop_pair_by_face_depth(scene, scene, face, 1., seed)
                        extent = np.subtract(bounds[1], bounds[0])
                        self.assertEqual(actual_axis, axis)
                        self.assertAlmostEqual(extent[axis], 1.)
                        np.testing.assert_allclose(np.delete(extent, axis), [.2, .3])
                np.testing.assert_array_equal(np.asarray(scene.points), original)

    def test_manual_ceiling_keeps_z_and_automatic_does_not_treat_it_as_front(self):
        x, y = np.meshgrid(np.linspace(.5, 1.5, 25), np.linspace(-.4, .4, 25))
        scene = cloud(np.column_stack((x.ravel(), y.ravel(), np.full(x.size, .9))))
        seed, bounds = estimate_wall_crop_bounds(scene, scene, np.array([1., 0, .9]), forward_axis='x')
        self.assertEqual(np.argmin(np.subtract(bounds[1], bounds[0])), 2)
        with self.assertRaisesRegex(ValueError, 'pared frontal'):
            estimate_wall_crop_bounds(scene, scene, forward_axis='x')

    def test_extrusion_does_not_mutate_face_arrays_or_swap_axis_on_repeat(self):
        bounds = np.array([[1., -.1, -.15], [1., .1, .15]])
        original = bounds.copy()
        scene = cloud([[1., 0, 0], [1.3, 0, 0], [1., .15, 0]])
        for _ in range(3):
            result = crop_cloud_by_face_depth(scene, bounds, 1.)
            self.assertEqual(len(result.points), 2)
            np.testing.assert_array_equal(bounds, original)


if __name__ == '__main__':
    unittest.main()
