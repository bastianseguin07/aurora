"""Display-only rainbow colors for an individual cloud, matching RViz intensity."""

from pathlib import Path
import numpy as np
import open3d as o3d


def rviz_intensity_colors(intensity, bounds=None):
    """RViz's default, non-inverted rainbow with automatic intensity bounds.

    Equivalent to IntensityPCTransformer/getRainbowColor in ros2/rviz:
    low intensity is red, high intensity is violet. No geometry is changed.
    """
    intensity = np.asarray(intensity, dtype=float).reshape(-1)
    colors = np.full((len(intensity), 3), 0.55)
    valid = np.isfinite(intensity)
    if not valid.any():
        return colors
    values = intensity[valid]
    low, high = (values.min(), values.max()) if bounds is None else bounds
    span = high - low
    normalized = np.clip((values - low) / span, 0, 1) if span else np.zeros(len(values))
    stops = np.linspace(0, 1, 6)
    palette = np.array(
        [[1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 1, 1], [0, 0, 1], [1, 0, 1]]
    )
    colors[valid] = np.column_stack(
        [np.interp(normalized, stops, palette[:, i]) for i in range(3)]
    )
    return colors


def color_individual_cloud(path: Path, cloud):
    """Color a copy; captured PLY/PCD, native fields, and analyses stay intact."""
    path = Path(path)
    result = o3d.geometry.PointCloud(cloud)
    intensity = None
    sidecar = path.with_name(path.stem + "_unitree.npz")
    if sidecar.is_file():
        with np.load(sidecar) as data:
            fields = data["fields"]
            if len(fields) == len(cloud.points):
                xyz = np.column_stack([fields[k] for k in ("x", "y", "z")])
                if np.array_equal(xyz, np.asarray(cloud.points)):
                    intensity = fields["intensity"].copy()
    if intensity is None and path.suffix.lower() == ".pcd":
        tensor = o3d.t.io.read_point_cloud(str(path))
        for name in ("intensity", "intensities"):
            if name in tensor.point:
                values = tensor.point[name].numpy().reshape(-1)
                if len(values) == len(cloud.points):
                    intensity = values
                    break
    if intensity is None and cloud.has_colors():
        colors = np.asarray(cloud.colors)
        if np.allclose(colors[:, 0], colors[:, 1]) and np.allclose(
            colors[:, 1], colors[:, 2]
        ):
            # Older monochrome captures store normalized intensity as RGB gray.
            intensity = colors[:, 0]
        else:
            return result
    if intensity is None:
        # A file without intensity or RGB can still show its vertical structure.
        intensity = np.asarray(cloud.points)[:, 2]
    result.colors = o3d.utility.Vector3dVector(rviz_intensity_colors(intensity))
    return result
