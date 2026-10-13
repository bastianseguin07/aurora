"""Behavior tests: resumable sessions, registration gate, stage semantics, integrity."""
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np
import open3d as o3d

from measurement_session import MeasurementSession
from pointcloud_core import compute_rigid_transform, apply_rigid_transform


class Sessions(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        xy = np.array([(x, y) for x in np.linspace(0, 1, 10) for y in np.linspace(0, 1, 10)])
        self.paths = []
        for index, height in enumerate((0, .04, .08)):
            path = self.root / f"stage{index}.ply"
            o3d.io.write_point_cloud(str(path), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[xy, np.full(len(xy), height)])))
            self.paths.append(path)

    def tearDown(self):
        self.temp.cleanup()

    def make_session(self, register=True):
        session = MeasurementSession.create(self.root / "sessions", "Tunel 1", "Pared izquierda 2")
        session.add_capture(self.paths[0], base=True)
        stages = [session.add_capture(self.paths[1]), session.add_capture(self.paths[2])]
        if register:
            anchors = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=float)
            identity = np.eye(3)
            for stage, path in zip(stages, self.paths[1:]):
                session.register_stage(stage, path, anchors, anchors, identity, np.zeros(3), 0.)
        return session

    def test_resume_portable_and_pairwise_stages(self):
        session = self.make_session()
        self.assertNotIn("demo", session.data)
        for index, incremental, expected in ((0, False, .04), (1, False, .08), (1, True, .04)):
            record, _result = session.compare(index, incremental, log=lambda _: None)
            self.assertAlmostEqual(record["stats_m"]["mean"], expected, places=6)
            session.record_result(record)
        for path in self.paths:
            path.unlink()
        legacy_data = json.loads(session.path.read_text(encoding="utf-8"))
        legacy_data["demo"] = True
        session.path.write_text(json.dumps(legacy_data), encoding="utf-8")
        relocated = self.root / "moved"
        shutil.copytree(session.path.parent, relocated)
        restored = MeasurementSession.load(relocated / "sesion.json")
        self.assertNotIn("demo", restored.data)
        self.assertEqual(len(restored.data["stages"]), 2)
        self.assertEqual(len(restored.data["results"]), 3)
        self.assertTrue(restored.resolve(restored.data["results"][0]["artifacts"]["csv"]).is_file())
        self.assertIn("No se resta la media", restored.export_report().read_text(encoding="utf-8"))
        with self.assertRaises(ValueError):
            restored.add_capture(restored.resolve(restored.data["base"]["path"]), base=True)

    def test_field_requires_unchanged_anchors_not_pose(self):
        session = self.make_session(register=False)
        with self.assertRaisesRegex(ValueError, "Faltan referencias"):
            session.compare(0, log=lambda _: None)
        anchors = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]])
        offset = np.array([.3, .1, .2])
        stage = session.data["stages"][0]
        cloud = o3d.io.read_point_cloud(str(self.paths[1]))
        cloud.translate(offset)
        moved = self.root / "moved_stage.ply"
        o3d.io.write_point_cloud(str(moved), cloud)
        stage = session.add_capture(moved)
        rotation, translation = compute_rigid_transform(anchors, anchors + offset)
        aligned = apply_rigid_transform(cloud, rotation, translation)
        aligned_path = self.root / "aligned.ply"
        o3d.io.write_point_cloud(str(aligned_path), aligned)
        session.register_stage(stage, aligned_path, anchors, anchors + offset, rotation, translation, 0)
        result, _ = session.compare(2, log=lambda _: None)
        self.assertAlmostEqual(result["stats_m"]["mean"], .04, places=6)
        self.assertEqual(result["status"], "exploratorio_no_validado")
        with self.assertRaises(ValueError):
            session.compare(2, incremental=True, log=lambda _: None)  # previous stage unregistered
        with self.assertRaisesRegex(ValueError, "colineales"):
            session.register_stage(stage, aligned_path, np.zeros((3, 3)), np.zeros((3, 3)), rotation, translation, 0)

    def test_missing_modified_and_path_traversal_blocked(self):
        session = self.make_session()
        session.resolve(session.data["stages"][0]["path"]).write_bytes(b"changed")
        with self.assertRaisesRegex(ValueError, "modificada"):
            MeasurementSession.load(session.path)
        with self.assertRaises(ValueError):
            session.compare(0, log=lambda _: None)
        with self.assertRaises(ValueError):
            session.resolve("../../outside.ply")

    def test_fixed_roi_applies_to_every_stage(self):
        session = self.make_session()
        quad = np.array([[.1, .1, 0], [.9, .1, 0], [.9, .9, 0], [.1, .9, 0]])
        session.set_roi(quad, 1.)
        record, _ = session.compare(1, log=lambda _: None)
        self.assertAlmostEqual(record["stats_m"]["mean"], .08, places=6)
        self.assertLess(record["stats_m"]["n_points"], 100)
        session.record_result(record)
        with self.assertRaises(ValueError):
            session.set_roi(quad, .5)

    def test_curved_tunnel_known_offset_and_unsigned_ambiguity(self):
        """A synthetic cylinder checks geometry math, not physical sensor accuracy."""
        axial = np.linspace(0, 1, 24)
        angle = np.linspace(-np.pi, np.pi, 80, endpoint=False)
        xx, aa = np.meshgrid(axial, angle, indexing="ij")
        anchors = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]], dtype=float)

        def save_radius(path, radius):
            points = np.c_[xx.ravel(), radius * np.sin(aa).ravel(), radius * np.cos(aa).ravel()]
            cloud_path = self.root / path
            o3d.io.write_point_cloud(str(cloud_path), o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))
            return cloud_path

        base_path = save_radius("curved_before.ply", 2.0)
        inward_path = save_radius("curved_after_inward.ply", 1.96)
        outward_path = save_radius("curved_after_outward.ply", 2.04)
        session = MeasurementSession.create(self.root / "curved", "Tunel sintetico", "Boveda")
        session.add_capture(base_path, base=True)
        stages = [session.add_capture(path) for path in (inward_path, outward_path)]
        for stage, path in zip(stages, (inward_path, outward_path)):
            session.register_stage(stage, path, anchors, anchors, np.eye(3), np.zeros(3), 0.0)

        inward, _ = session.compare(0, log=lambda _: None)
        outward, _ = session.compare(1, log=lambda _: None)
        self.assertAlmostEqual(inward["stats_m"]["mean"], 0.04, places=6)
        self.assertAlmostEqual(outward["stats_m"]["mean"], 0.04, places=6)
        self.assertAlmostEqual(inward["stats_m"]["p95"], 0.04, places=6)
        self.assertAlmostEqual(outward["stats_m"]["p95"], 0.04, places=6)
        self.assertEqual(inward["method"], outward["method"])


if __name__ == "__main__":
    unittest.main()
