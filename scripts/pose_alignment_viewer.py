"""Launcher for the shared, decorated Open3D IMU pose viewer."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
from pathlib import Path

import numpy as np


class PoseAlignmentViewer:
    def __init__(self, reference_position, position_tolerance_m, log=print, reference_rpy_deg=None):
        self._reference_position = np.asarray(reference_position, dtype=float).reshape(3)
        self._reference_rpy = np.asarray(
            np.zeros(3) if reference_rpy_deg is None else reference_rpy_deg, dtype=float
        ).reshape(3)
        self._tolerance = float(position_tolerance_m)
        self._log = log
        self._directory: Path | None = None
        self._state_path: Path | None = None
        self._process: subprocess.Popen | None = None
        self._lock = threading.Lock()

    def start(self) -> None:
        if self.is_running():
            return
        self._directory = Path(tempfile.mkdtemp(prefix="aurora_imu_pose_"))
        self._state_path = self._directory / "pose.json"
        self.update_current_pose(self._reference_position, self._reference_rpy)
        root = Path(__file__).resolve().parent.parent
        command = [
            sys.executable, str(root / "scripts" / "viewer_host.py"), "pose",
            str(self._state_path), "--pose-reference-json",
            json.dumps([self._reference_position.tolist(), self._reference_rpy.tolist()]),
            "--pose-tolerance-m", str(self._tolerance), "--title", "Aurora - Guia de pose IMU",
        ]
        try:
            self._process = subprocess.Popen(command, cwd=root, start_new_session=True)
        except OSError:
            shutil.rmtree(self._directory, ignore_errors=True)
            self._directory = self._state_path = None
            raise
        self._log("Vista 3D de guia IMU abierta con metricas integradas.")

    def stop(self) -> None:
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, 15)
                process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, 9)
                except ProcessLookupError:
                    pass
        if self._directory is not None:
            shutil.rmtree(self._directory, ignore_errors=True)
        self._directory = self._state_path = None

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def update_current_pose(self, position, rpy_deg=None) -> None:
        if self._state_path is None:
            return
        pose = {
            "position": np.asarray(position, dtype=float).reshape(3).tolist(),
            "rpy_deg": np.asarray(
                self._reference_rpy if rpy_deg is None else rpy_deg, dtype=float
            ).reshape(3).tolist(),
        }
        with self._lock:
            temporary = self._state_path.with_suffix(".tmp")
            temporary.write_text(json.dumps(pose), encoding="utf-8")
            temporary.replace(self._state_path)
