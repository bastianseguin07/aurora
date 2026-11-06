"""Display-only temporal window. Never used by the saved capture path."""

from collections import deque
import numpy as np


class DecayBuffer:
    def __init__(self, decay_s=0.0, max_bytes=64 * 1024 * 1024, log=print):
        self.frames = deque()
        self.bytes = 0
        self.decay_s = float(decay_s)
        self.max_bytes = max_bytes
        self.log = log
        self.dropped = 0

    def clear(self):
        self.frames.clear()
        self.bytes = 0

    def configure(self, decay_s):
        if not np.isfinite(decay_s) or not 0 <= decay_s <= 60:
            raise ValueError("Decay time debe estar entre 0 y 60 segundos")
        self.decay_s = float(decay_s)

    def append(self, points, colors, now):
        if self.decay_s == 0:
            self.clear()
        points, colors = points.copy(), colors.copy()
        size = points.nbytes + colors.nbytes
        if size > self.max_bytes:
            self.dropped += 1
            self.log(
                "Sobrecarga del visor: nube supera memoria del buffer; se descarta solo de la vista"
            )
            return
        self.frames.append((now, points, colors, size))
        self.bytes += size
        dropped = 0
        while self.bytes > self.max_bytes:
            self.bytes -= self.frames.popleft()[3]
            dropped += 1
        if dropped:
            self.dropped += dropped
            self.log(
                f"Sobrecarga del visor: {dropped} nubes antiguas descartadas del buffer visual"
            )

    def snapshot(self, now):
        if self.decay_s > 0:
            while self.frames and now - self.frames[0][0] >= self.decay_s:
                self.bytes -= self.frames.popleft()[3]
        elif len(self.frames) > 1:
            latest = self.frames[-1]
            self.clear()
            self.frames.append(latest)
            self.bytes = latest[3]
        if not self.frames:
            return np.empty((0, 3)), np.empty((0, 3))
        return np.concatenate([f[1] for f in self.frames]), np.concatenate(
            [f[2] for f in self.frames]
        )
