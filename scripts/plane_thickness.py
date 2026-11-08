"""Perpendicular thickness for a cropped, predominantly planar surface."""

import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull


def _face(points, label, tolerance_m):
    if len(points) < 30 or not np.isfinite(points).all():
        raise ValueError(f"{label}: se necesitan al menos 30 puntos finitos de la cara.")
    cloud = o3d.geometry.PointCloud()
    # Bound only the plane search; all original points are classified below.
    sample = points[np.linspace(0, len(points) - 1, min(len(points), 20_000), dtype=int)]
    cloud.points = o3d.utility.Vector3dVector(sample)
    coefficients, _ = cloud.segment_plane(
        distance_threshold=tolerance_m, ransac_n=3, num_iterations=1000,
    )
    normal = np.asarray(coefficients[:3])
    mask = np.abs(points @ normal + coefficients[3]) <= tolerance_m
    for _ in range(3):
        if np.count_nonzero(mask) < max(30, .6 * len(points)):
            raise ValueError(
                f"{label}: no hay una cara plana dominante (mínimo 60%). "
                "Recorta solo la cara de la pared o caja, excluyendo piso y laterales."
            )
        face = points[mask]
        center = np.mean(face, axis=0)
        _, singular, axes = np.linalg.svd(face - center, full_matrices=False)
        if singular[1] / np.sqrt(len(face)) < .005:
            raise ValueError(f"{label}: la cara es demasiado estrecha para definir un plano.")
        normal = axes[-1]
        mask = np.abs((points - center) @ normal) <= tolerance_m
    # Refit once to the final classification.
    face = points[mask]
    if len(face) < max(30, .6 * len(points)):
        raise ValueError(f"{label}: el ajuste no conserva una cara plana dominante. Reduce el recorte.")
    center = np.mean(face, axis=0)
    _, _, axes = np.linalg.svd(face - center, full_matrices=False)
    return mask, center, axes


def planar_thickness(before, after, tolerance_m=.015):
    """Return after indices, point-to-plane distances and fitting diagnostics.

    Both clouds must describe the same cropped face in a common frame. Reject
    multiple dominant surfaces, incompatible normals and poor overlap. Do not
    extrapolate the reference plane beyond the convex footprint of its samples.
    Distances are unsigned, like the existing C2C output.
    """
    before, after = np.asarray(before, dtype=float), np.asarray(after, dtype=float)
    if before.ndim != 2 or after.ndim != 2 or before.shape[1:] != (3,) or after.shape[1:] != (3,):
        raise ValueError("Las nubes deben tener puntos XYZ.")
    if not np.isfinite(tolerance_m) or tolerance_m <= 0:
        raise ValueError("La tolerancia del plano debe ser mayor que cero.")
    base_mask, center, axes = _face(before, "ANTES", tolerance_m)
    after_mask, _, after_axes = _face(after, "DESPUÉS", tolerance_m)
    angle = float(np.degrees(np.arccos(np.clip(abs(axes[-1] @ after_axes[-1]), 0, 1))))
    if angle > 10:
        raise ValueError(
            f"Las caras difieren {angle:.1f}°. Recorta la misma cara en ambas tomas "
            "y comprueba la alineación; Cara plana admite hasta 10°."
        )
    # Test convex footprint half-spaces; only measured overlap is supported.
    base_uv = (before[base_mask] - center) @ axes[:2].T
    after_uv = (after[after_mask] - center) @ axes[:2].T
    hull = ConvexHull(base_uv)
    inside = np.ones(len(after_uv), dtype=bool)
    for equation in hull.equations:
        inside &= after_uv @ equation[:2] + equation[2] <= 1e-9
    if inside.sum() < 30 or inside.mean() < .5:
        raise ValueError("Las caras no tienen suficiente zona común (mínimo 50%). Amplía o corrige el recorte.")
    indices = np.flatnonzero(after_mask)[inside]
    distances = np.abs((after[indices] - center) @ axes[-1])
    details = dict(
        tolerance_m=float(tolerance_m), normal_angle_deg=angle,
        base_face_points=int(base_mask.sum()), after_face_points=int(after_mask.sum()),
        overlap_points=len(indices), input_after_points=len(after),
        rejected_geometry_points=int(len(after) - len(indices)),
        base_residual_std_m=float(np.std((before[base_mask] - center) @ axes[-1])),
    )
    return indices, distances, details
