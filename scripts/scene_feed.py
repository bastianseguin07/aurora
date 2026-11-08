"""Full-resolution scene feed. Native capture files and cursors are independent."""

from collections import deque
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

import sensor_backend
from cloud_display_colors import rviz_intensity_colors


EMPTY = np.empty((0, 3), dtype=np.float32)
MAX_SCENE_POINTS = 300_000


def scene_view(state, color_mode="intensity", capture_settings=None, bounds=None):
    """Apply reversible display settings without altering the acquired scene."""
    points = state["points"]
    position = state["position"]
    settings = capture_settings or {}
    distances = None
    local = points
    if settings.get("enabled") or color_mode == "distance":
        if state["registered"] and position is not None:
            local = np.einsum("ij,jk->ik", points - position, state["rotation"], optimize=False)
        distances = np.linalg.norm(local, axis=1)
    mask = None
    if settings.get("enabled"):
        mask = np.ones(len(points), dtype=bool)
        maximum = settings.get("max_distance_m")
        cone = settings.get("cone_angle_deg")
        lateral = settings.get("max_lateral_m")
        if maximum:
            mask &= distances < maximum
        if cone is not None and cone < 360:
            axis = "xyz".index(settings.get("forward_axis", "x"))
            mask &= local[:, axis] / np.maximum(distances, 1e-12) >= np.cos(np.radians(cone / 2))
        if lateral:
            mask &= np.abs(local[:, 1]) <= lateral
    intensity = state.get("intensity", np.empty(0))
    if color_mode == "height":
        values = points[:, 2]
    elif color_mode == "distance":
        values = distances
    else:
        values = intensity if len(intensity) == len(points) else np.mean(state["colors"], axis=1)
    if mask is not None:
        values = values[mask]
    finite = values[np.isfinite(values)]
    limits = bounds if bounds is not None else ((float(finite.min()), float(finite.max())) if len(finite) else None)
    visible_points = points[mask] if mask is not None else points
    if len(visible_points) > MAX_SCENE_POINTS:
        indices = np.linspace(0, len(visible_points) - 1, MAX_SCENE_POINTS, dtype=np.int64)
        visible_points, values = visible_points[indices], values[indices]
    return {**state, "points": visible_points, "colors": rviz_intensity_colors(values, limits),
            "color_limits": limits, "visible_count": len(visible_points)}


class SceneFeed:
    def __init__(self, start=True):
        self.lock = threading.RLock()
        self.connection = None
        self.decay_s = 5.0
        self.stop_event = threading.Event()
        self.frames = deque()
        self.pending_native = deque()
        self.path = deque()
        self.reference = None
        self.display_origin = None
        self.session = None
        self.position = None
        self.rotation = np.eye(3)
        self.mode = None
        self.rate_times = deque()
        self.last_cloud_at = None
        self.state = self._empty_state("Conecta el sensor para ver el entorno")
        self.thread = threading.Thread(target=self._run, daemon=True)
        if start:
            self.thread.start()

    @staticmethod
    def _empty_state(status):
        return dict(points=EMPTY, colors=EMPTY, path=EMPTY, position=None,
                    rotation=np.eye(3), reference=None, reference_rotation=np.eye(3),
                    pose_ok=False, registered=False, status=status, session=None,
                    capture_reference_ready=False,
                    intensity=np.empty(0, dtype=np.float32), rate_hz=0., cloud_age_s=None)

    def set_connection(self, connection):
        with self.lock:
            self.connection = connection
            self.frames.clear()
            self.pending_native.clear()
            self.rate_times.clear()
            self.last_cloud_at = None
            self.mode = None
            self.state = self._empty_state(
                "Esperando nubes…" if connection is not None else "Sensor desconectado"
            )

    def set_decay(self, seconds):
        seconds = float(seconds)
        if not np.isfinite(seconds) or not 0 <= seconds <= 60:
            raise ValueError("Decay time debe estar entre 0 y 60 s")
        with self.lock:
            self.decay_s = seconds

    def set_reference(self, pose, replace=False):
        """Keep the reference until an explicit new BEFORE capture replaces it."""
        if pose is None or not getattr(pose, "session_id", None):
            if replace:
                with self.lock:
                    # A new measurement without tracking must not inherit the
                    # last measurement's zero. The next valid pose rebases it.
                    self.reference = self.display_origin = self.position = None
                    self.path.clear()
                    self.frames.clear()
                    self.state = {**self.state, 'reference': None, 'position': None,
                                  'path': EMPTY, 'points': EMPTY, 'colors': EMPTY,
                                  'intensity': np.empty(0, dtype=np.float32),
                                  'capture_reference_ready': False, 'pose_ok': False}
            return False
        with self.lock:
            if self.session is not None and self.session != pose.session_id:
                return False
            if self.reference is not None and self.reference.session_id == pose.session_id and not replace:
                return False
            old_origin = (self.reference.position if self.reference is not None else self.display_origin
                          if self.display_origin is not None else np.zeros(3))
            self.reference = pose
            self.path.clear()
            self.path.append(np.asarray(pose.position).copy())
            if self.position is None:
                self.position = np.asarray(pose.position).copy()
                self.rotation = Rotation.from_quat(pose.quaternion_xyzw).as_matrix()
            elif np.linalg.norm(self.position - pose.position) >= .02:
                self.path.append(self.position.copy())
            if self.mode == "registered":
                self.state = {**self.state,
                              "points": self.state["points"] + old_origin - pose.position,
                              "reference": np.zeros(3),
                              "reference_rotation": Rotation.from_quat(pose.quaternion_xyzw).as_matrix(),
                              "position": self.position - pose.position,
                              "path": np.asarray(self.path) - pose.position,
                              "capture_reference_ready": True}
            return True

    def snapshot(self, blocking=True):
        if not self.lock.acquire(blocking=blocking):
            return None
        try:
            return self.state.copy()
        finally:
            self.lock.release()

    def clear_history(self):
        with self.lock:
            self.frames.clear()
            self.state = {**self.state, "points": EMPTY, "colors": EMPTY,
                          "intensity": np.empty(0, dtype=np.float32)}

    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=1)

    def _update_pose(self, pose):
        if self.session != pose.session_id:
            self.session = pose.session_id
            self.frames.clear()
            self.path.clear()
            self.position = None
            self.display_origin = np.asarray(pose.position).copy()
            if self.reference is not None and self.reference.session_id != self.session:
                self.reference = None
        p = np.asarray(pose.position)
        if self.display_origin is None:
            self.display_origin = p.copy()
        r = Rotation.from_quat(pose.quaternion_xyzw).as_matrix()
        # Visual deadband only; saved poses and measurement data remain untouched.
        if self.position is None or np.linalg.norm(p - self.position) >= 0.01:
            self.position = p.copy()
        angle = Rotation.from_matrix(self.rotation.T @ r).magnitude()
        if angle >= np.radians(1):
            self.rotation = r
        if not self.path or np.linalg.norm(self.position - self.path[-1]) >= 0.02:
            self.path.append(self.position.copy())

    def _append(self, points, colors, now, intensity=None):
        valid = np.isfinite(points).all(axis=1)
        intensity = np.mean(colors, axis=1) if intensity is None else np.asarray(intensity)
        points, colors = points[valid], colors[valid]
        intensity = intensity[valid]
        if not len(points):
            return
        if self.decay_s == 0:
            self.frames.clear()
        self.frames.append((now, points.astype(np.float32), colors.astype(np.float32), intensity.astype(np.float32)))
        self.last_cloud_at = now
        self.rate_times.append(now)

    def _poll(self, connection):
        now = time.monotonic()
        pose = None
        try:
            pose = connection.pose_history.smoothed()
        except (AttributeError, RuntimeError, ValueError):
            pass
        registered_frames = connection.read_registered_batch() if hasattr(connection, "read_registered_batch") else []
        native_mode = hasattr(connection, "pose_history") and hasattr(connection, "read_live_batch")
        native_frames = connection.read_live_batch(40) if native_mode else []

        samples = []
        # Once SLAM is active, retain its coordinate frame through short outages.
        if not native_mode and pose is None and self.mode != "registered":
            samples = sensor_backend.read_view_samples(connection, timeout_ms=40)
        with self.lock:
            if connection is not self.connection:
                return
            if pose is not None:
                self._update_pose(pose)
                if self.mode != "registered":
                    self.frames.clear()
                    self.mode = "registered"
            for frame in native_frames:
                xyz = frame.xyz
                valid = np.isfinite(xyz).all(axis=1) & np.any(xyz != 0, axis=1)
                if not np.all(valid):
                    frame = frame.subset(valid)
                if len(frame.fields):
                    self.pending_native.append(frame)
            while self.pending_native:
                frame = self.pending_native[0]
                if pose is None:
                    if self.mode != "registered":
                        self.mode = "local"
                        self._append(frame.xyz, np.full((len(frame.fields), 3), .65), frame.received_at, frame.fields["intensity"])
                else:
                    try:
                        points, session = connection.pose_history.points_to_world(frame)
                        if session == self.session:
                            self._append(points, np.full(points.shape, .65), frame.received_at, frame.fields["intensity"])
                    except (RuntimeError, ValueError):
                        if now - frame.received_at < 1.5:
                            break
                self.pending_native.popleft()
            for frame in ([] if native_mode else registered_frames):
                if pose is not None and frame.frame_id == self.session:
                    intensity = frame.fields["intensity"]
                    self._append(frame.xyz, np.full((len(intensity), 3), .65), frame.received_at, intensity)
            if samples:
                if self.mode != "local":
                    self.frames.clear()
                    self.mode = "local"
                for sample in samples:
                    colors = np.asarray(sample.colors)
                    if colors.shape != sample.points.shape:
                        colors = np.full(sample.points.shape, 0.65)
                    self._append(np.asarray(sample.points), colors, sample.received_at, sample.intensity)
            ttl = self.decay_s if self.decay_s > 0 else 0.75
            while self.frames and now - self.frames[0][0] > ttl:
                self.frames.popleft()
            points = np.concatenate([v[1] for v in self.frames]) if self.frames else EMPTY
            colors = np.concatenate([v[2] for v in self.frames]) if self.frames else EMPTY
            intensity = np.concatenate([v[3] for v in self.frames]) if self.frames else np.empty(0, dtype=np.float32)
            while self.rate_times and now - self.rate_times[0] > 2:
                self.rate_times.popleft()
            rate = ((len(self.rate_times) - 1) / (self.rate_times[-1] - self.rate_times[0])
                    if len(self.rate_times) > 1 and self.rate_times[-1] > self.rate_times[0] else 0.)
            world = self.mode == "registered"
            reference = self.reference if world else None
            origin = (reference.position if reference is not None else self.display_origin
                      if world and self.display_origin is not None else np.zeros(3))
            position = self.position - origin if world and self.position is not None else None
            path = np.asarray(self.path) - origin if world and self.path else EMPTY
            status = "Entorno completo · seguimiento activo" if world else "Nube local · preparando seguimiento"
            if world and pose is None:
                status = "Seguimiento sin actualizar · última posición"
            if not len(points):
                status += " · sin nubes recientes"
            self.state = dict(
                points=points - origin if world else points, colors=colors, path=path,
                position=position, rotation=self.rotation.copy(),
                reference=np.zeros(3) if reference is not None else None,
                reference_rotation=Rotation.from_quat(reference.quaternion_xyzw).as_matrix()
                if reference is not None else np.eye(3),
                pose_ok=pose is not None, registered=world, status=status, session=self.session,
                capture_reference_ready=reference is not None,
                intensity=intensity, rate_hz=rate,
                cloud_age_s=now - self.last_cloud_at if self.last_cloud_at is not None else None,
            )

    def _run(self):
        while not self.stop_event.is_set():
            with self.lock:
                connection = self.connection
            if connection is not None:
                try:
                    self._poll(connection)
                except Exception as exc:
                    with self.lock:
                        if connection is self.connection:
                            self.state = self._empty_state(f"Vista en espera: {exc}")
            self.stop_event.wait(0.15)
