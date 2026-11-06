"""Versioned full-cloud binary IPC, independent of ROS/SDK Python bindings."""

from __future__ import annotations
import struct
from dataclasses import dataclass
import numpy as np

HEADER = struct.Struct("<4sIdQIqH")
POINT_DTYPE = np.dtype(
    [(n, "<f8") for n in ("x", "y", "z", "intensity", "time")] + [("ring", "<u4")]
)
MAX_POINTS = 4_000_000


@dataclass(frozen=True)
class CloudFrame:
    fields: np.ndarray
    stamp: float
    cloud_id: int
    frame_id: str = "unilidar_lidar"
    received_at: float = 0.0
    stamp_ns: int = -1

    @property
    def xyz(self):
        return np.column_stack([self.fields[n] for n in ("x", "y", "z")])

    @property
    def nbytes(self):
        return self.fields.nbytes

    def subset(self, mask):
        return CloudFrame(
            self.fields[mask].copy(),
            self.stamp,
            self.cloud_id,
            self.frame_id,
            self.received_at,
            self.stamp_ns,
        )


def read_exact(stream, count):
    parts = bytearray()
    while len(parts) < count:
        chunk = stream.read(count - len(parts))
        if not chunk:
            if not parts:
                raise EOFError("Fin del flujo Unitree")
            raise ValueError("Nube Unitree truncada: se descarta el mensaje completo")
        parts.extend(chunk)
    return bytes(parts)


def read_frame(stream, received_at=0.0):
    magic, version, stamp, cloud_id, count, stamp_ns, frame_len = HEADER.unpack(
        read_exact(stream, HEADER.size)
    )
    if magic != b"UTL2" or version != 2 or count > MAX_POINTS or frame_len > 1024:
        raise ValueError("Cabecera Unitree invalida o nube demasiado grande")
    frame_id = read_exact(stream, frame_len).decode("utf-8") if frame_len else ""
    fields = (
        np.frombuffer(
            read_exact(stream, count * POINT_DTYPE.itemsize), dtype=POINT_DTYPE
        ).copy()
        if count
        else np.empty(0, dtype=POINT_DTYPE)
    )
    return CloudFrame(fields, stamp, cloud_id, frame_id, received_at, stamp_ns)


def encode_frame(frame):
    name = frame.frame_id.encode("utf-8")
    if len(frame.fields) > MAX_POINTS or len(name) > 1024:
        raise ValueError("Nube demasiado grande")
    return (
        HEADER.pack(
            b"UTL2",
            2,
            frame.stamp,
            frame.cloud_id,
            len(frame.fields),
            frame.stamp_ns,
            len(name),
        )
        + name
        + np.asarray(frame.fields, dtype=POINT_DTYPE).tobytes()
    )
