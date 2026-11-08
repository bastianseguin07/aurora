"""
Logica central para comparar dos nubes de puntos .ply y estimar el espesor
de shotcrete (distancia Cloud-to-Cloud). La usan tanto la CLI
(compare_point_clouds.py) como la GUI (gui_gtk.py).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Tuple

import numpy as np
import open3d as o3d

Vec3 = Tuple[float, float, float]

# Aurora SLAM pose axes are X=forward, Y=left, Z=up. Depth-camera point grids
# use X=right, Y=down, Z=forward. Convert vectors between those frames before
# applying a pose transform to a depth point.
_POSE_TO_SENSOR_VIEW = np.array(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass
class DistanceStats:
    mean: float
    median: float
    std: float
    min: float
    max: float
    p95: float
    n_points: int
    input_n_points: int = 0
    excluded_n_points: int = 0
    minimum_thickness_m: float = 0.0

    def __str__(self) -> str:
        return (
            f"  Puntos analizados : {self.n_points}\n"
            f"  Espesor medio     : {self.mean * 100:.2f} cm\n"
            f"  Espesor mediano   : {self.median * 100:.2f} cm\n"
            f"  Desv. estandar    : {self.std * 100:.2f} cm"
            + (
                f"\n  Excluidos (< {self.minimum_thickness_m * 100:.2f} cm): "
                f"{self.excluded_n_points:,} de {self.input_n_points:,}"
                if self.minimum_thickness_m > 0
                else ""
            )
        )


@dataclass
class PipelineParams:
    base_path: Path
    updated_path: Path
    output_dir: Path
    voxel_size: float = 0.0
    remove_outliers: bool = False
    use_icp: bool = False
    icp_threshold: float = 0.05
    crop_min: Vec3 | None = None
    crop_max: Vec3 | None = None
    max_distance: float | None = None
    minimum_thickness_m: float = 0.0
    distance_method: str = "cloud_to_cloud"
    align_by_pose: bool = False
    surface_params: dict | None = None


@dataclass
class PipelineResult:
    stats: DistanceStats
    distances: np.ndarray
    base_cloud: o3d.geometry.PointCloud
    updated_cloud: o3d.geometry.PointCloud
    heatmap_cloud: o3d.geometry.PointCloud
    csv_path: Path
    histogram_path: Path
    heatmap_path: Path
    distance_method: str = "cloud_to_cloud"
    measurement_details: dict | None = None
    comparison: object | None = None
    comparison_cloud: o3d.geometry.PointCloud | None = None
    accepted_mask: np.ndarray | None = None


def load_point_cloud(path: Path) -> o3d.geometry.PointCloud:
    if not path.exists():
        raise FileNotFoundError(f"No se encontro el archivo: {path}")
    cloud = o3d.io.read_point_cloud(str(path))
    if len(cloud.points) == 0:
        raise ValueError(f"La nube de puntos '{path}' esta vacia o no se pudo leer.")
    return cloud


def crop_point_cloud(
    cloud: o3d.geometry.PointCloud, crop_min: Vec3, crop_max: Vec3
) -> o3d.geometry.PointCloud:
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=np.array(crop_min), max_bound=np.array(crop_max)
    )
    return cloud.crop(bbox)


def crop_cloud_by_face_depth(
    cloud: o3d.geometry.PointCloud,
    face_bounds: tuple[Vec3, Vec3],
    depth_m: float,
) -> o3d.geometry.PointCloud:
    """Extrude the selected face symmetrically along its thinnest axis."""
    if depth_m <= 0:
        raise ValueError("La profundidad debe ser mayor que cero.")
    low = np.asarray(face_bounds[0], dtype=np.float64).copy()
    high = np.asarray(face_bounds[1], dtype=np.float64).copy()
    if low.shape != (3,) or high.shape != (3,) or np.any(high < low):
        raise ValueError("Los limites de la cara no son validos.")
    depth_axis = int(np.argmin(high - low))
    center = (low[depth_axis] + high[depth_axis]) / 2.0
    low[depth_axis] = center - depth_m / 2.0
    high[depth_axis] = center + depth_m / 2.0
    return crop_point_cloud(cloud, tuple(low.tolist()), tuple(high.tolist()))


def crop_pair_by_face_depth(
    before: o3d.geometry.PointCloud,
    after: o3d.geometry.PointCloud,
    face_bounds: tuple[Vec3, Vec3],
    depth_m: float,
    seed: np.ndarray | None = None,
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, tuple[Vec3, Vec3], tuple[Vec3, Vec3], int, float]:
    """Apply the same face/depth box independently to both point clouds."""
    if depth_m <= 0:
        raise ValueError("La profundidad debe ser mayor que cero.")
    low = np.asarray(face_bounds[0], dtype=np.float64)
    high = np.asarray(face_bounds[1], dtype=np.float64)
    if low.shape != (3,) or high.shape != (3,) or np.any(high < low):
        raise ValueError("Los limites de la cara no son validos.")

    depth_axis = int(np.argmin(high - low))
    center = 0.5 * (low[depth_axis] + high[depth_axis])
    box_low, box_high = low.copy(), high.copy()
    box_low[depth_axis] = center - depth_m / 2.0
    box_high[depth_axis] = center + depth_m / 2.0
    before_cut = crop_point_cloud(before, tuple(box_low.tolist()), tuple(box_high.tolist()))
    after_cut = crop_point_cloud(after, tuple(box_low.tolist()), tuple(box_high.tolist()))
    if not len(before_cut.points) and not len(after_cut.points):
        raise ValueError("El box no contiene puntos. Aumenta el lado de la cara o la profundidad.")
    bounds = (tuple(low.tolist()), tuple(high.tolist()))
    box_bounds = (tuple(box_low.tolist()), tuple(box_high.tolist()))
    return before_cut, after_cut, bounds, box_bounds, depth_axis, 0.0


def rectangular_face_bounds(
    face_bounds: tuple[Vec3, Vec3], seed: np.ndarray, width_m: float, height_m: float
) -> tuple[Vec3, Vec3]:
    """Set an exact rectangular face centered on the selected wall point.

    The two in-plane axes (whichever two are not the detected depth axis)
    are assigned ``width_m``/``height_m`` in ascending axis order (e.g. for a
    depth axis of Z, X gets width_m and Y gets height_m).
    """
    if not np.isfinite(width_m) or width_m <= 0:
        raise ValueError("El ancho del box debe ser mayor que cero.")
    if not np.isfinite(height_m) or height_m <= 0:
        raise ValueError("El alto del box debe ser mayor que cero.")
    low = np.asarray(face_bounds[0], dtype=np.float64).copy()
    high = np.asarray(face_bounds[1], dtype=np.float64).copy()
    seed = np.asarray(seed, dtype=np.float64).reshape(3)
    if low.shape != (3,) or high.shape != (3,) or np.any(high < low):
        raise ValueError("Los limites de la cara no son validos.")
    depth_axis = int(np.argmin(high - low))
    low[depth_axis] = high[depth_axis] = seed[depth_axis]
    in_plane_axes = [axis for axis in range(3) if axis != depth_axis]
    for axis, size_m in zip(in_plane_axes, (width_m, height_m)):
        low[axis] = seed[axis] - size_m / 2.0
        high[axis] = seed[axis] + size_m / 2.0
    return tuple(low.tolist()), tuple(high.tolist())


def preprocess(
    cloud: o3d.geometry.PointCloud,
    voxel_size: float | None,
    remove_outliers: bool,
    crop_min: Vec3 | None,
    crop_max: Vec3 | None,
    cloud_label: str = "la nube",
) -> o3d.geometry.PointCloud:
    result = cloud
    if crop_min is not None and crop_max is not None:
        result = crop_point_cloud(result, crop_min, crop_max)
        if len(result.points) == 0:
            raise ValueError(
                f"El recorte (crop) no dejo ningun punto en {cloud_label}. "
                "Es probable que el objeto/zona se haya desplazado mas de lo que cubre "
                "el margen actual: agranda el margen (o los limites min/max) para que "
                "la region de interes quede incluida en ambas nubes, incluso desplazada."
            )
    if voxel_size and voxel_size > 0:
        result = result.voxel_down_sample(voxel_size)
    if remove_outliers:
        result, _ = result.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return result


def align_clouds(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    threshold: float,
) -> o3d.geometry.PointCloud:
    """
    Alinea 'source' contra 'target' con ICP punto-a-punto. Solo corrige
    pequenos errores de registro entre escaneos; no usar cuando el propio
    desplazamiento que se busca medir (p. ej. el espesor de shotcrete sobre
    TODA la pared) domina la nube, porque ICP podria "absorberlo" como si
    fuera error de alineacion. Es seguro usarlo cuando el fondo estatico
    domina en cantidad de puntos (p. ej. una escena con un objeto pequeno
    que se movio).
    """
    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    return source.transform(result.transformation)


def compute_rigid_transform(base_points: np.ndarray, moving_points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Calcula la rotacion (3x3) y traslacion (3,) optimas que llevan
    'moving_points' sobre 'base_points', dado un conjunto de puntos
    correspondientes en el mismo orden (algoritmo de Kabsch / analisis de
    Procrustes). Pensado para alinear usando puntos de referencia fijos
    (p. ej. cabezas de pernos de anclaje) en vez de ICP sobre toda la nube,
    para no confundir el espesor real de shotcrete con error de alineacion.

    Requiere al menos 3 puntos no colineales para que la rotacion quede
    bien determinada.
    """
    base_points = np.asarray(base_points, dtype=np.float64)
    moving_points = np.asarray(moving_points, dtype=np.float64)
    if (
        base_points.ndim != 2
        or moving_points.ndim != 2
        or base_points.shape != moving_points.shape
        or base_points.shape[0] < 3
        or base_points.shape[1] != 3
    ):
        raise ValueError("Se necesitan al menos 3 pares de puntos XYZ, en igual cantidad y orden.")
    if not np.all(np.isfinite(base_points)) or not np.all(np.isfinite(moving_points)):
        raise ValueError("Los puntos de referencia contienen valores no finitos.")

    # Tres puntos colineales no determinan una rotacion 3D unica. Rechazarlos
    # aqui evita que SVD elija un giro arbitrario alrededor de esa linea.
    for label, points in (("ANTES", base_points), ("DESPUES", moving_points)):
        singular_values = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
        if (
            singular_values[0] <= 1e-12
            or singular_values[1] <= max(singular_values[0] * 1e-6, 1e-12)
        ):
            raise ValueError(
                f"Los puntos de {label} estan repetidos o casi alineados. "
                "Elige al menos 3 referencias separadas y no colineales."
            )

    base_centroid = base_points.mean(axis=0)
    moving_centroid = moving_points.mean(axis=0)
    base_centered = base_points - base_centroid
    moving_centered = moving_points - moving_centroid

    h = moving_centered.T @ base_centered
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    correction = np.diag([1.0, 1.0, d])
    rotation = vt.T @ correction @ u.T
    translation = base_centroid - rotation @ moving_centroid
    return rotation, translation


def rigid_transform_rms_error(
    base_points: np.ndarray, moving_points: np.ndarray, rotation: np.ndarray, translation: np.ndarray
) -> float:
    """Error residual (RMS, en metros) tras aplicar la transformacion a los puntos de referencia."""
    transformed = (rotation @ np.asarray(moving_points).T).T + translation
    return float(np.sqrt(np.mean(np.sum((transformed - np.asarray(base_points)) ** 2, axis=1))))


def apply_rigid_transform(
    cloud: o3d.geometry.PointCloud, rotation: np.ndarray, translation: np.ndarray
) -> o3d.geometry.PointCloud:
    """Aplica la rotacion+traslacion (de compute_rigid_transform) a una copia de la nube."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    result = o3d.geometry.PointCloud(cloud)
    result.transform(matrix)
    return result


def _apply_sdk_render_style(vis, cloud: o3d.geometry.PointCloud) -> None:
    """Replica el estilo de visualizacion del demo oficial del SDK
    (examples/dense_point_cloud.py): fondo oscuro, puntos mas grandes,
    iluminacion desactivada para mostrar el color real, y evita el colormap
    arcoiris por defecto de Open3D (por altura Z) cuando la nube no trae
    colores propios (usa gris uniforme en ese caso)."""
    if not cloud.has_colors():
        cloud.paint_uniform_color([0.7, 0.7, 0.7])
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.1, 0.1, 0.1])
    opt.point_size = 2.0
    opt.point_color_option = o3d.visualization.PointColorOption.Color
    opt.light_on = False


def _start_at_sensor_pov(vis) -> None:
    """Posiciona la camara del visor en el origen del sensor mirando hacia
    +Z (los puntos ya vienen en su marco local: X=derecha, Y=abajo,
    Z=adelante), en vez de encuadrar toda la nube desde afuera."""
    try:
        ctr = vis.get_view_control()
        params = ctr.convert_to_pinhole_camera_parameters()
        params.extrinsic = np.eye(4)  # camara en el origen del sensor, mirando hacia +Z
        ctr.convert_from_pinhole_camera_parameters(params, allow_arbitrary=True)
    except Exception:
        pass  # si el visor no soporta esto, se queda con el encuadre por defecto


def show_point_cloud(
    cloud: o3d.geometry.PointCloud,
    window_name: str = "Aurora - Vista de la captura",
    show_coordinate_frame: bool = True,
) -> None:
    """Muestra una nube en el visor 3D comun, con ejes y cierre con Q."""
    show_geometries([cloud], window_name, show_coordinate_frame=show_coordinate_frame)


def _create_q_only_visualizer() -> o3d.visualization.VisualizerWithKeyCallback:
    """Create a legacy viewer that consumes keys except unmodified Q."""
    vis = o3d.visualization.VisualizerWithKeyCallback()

    def on_key_action(visualizer, key: int, action: int, mods: int) -> bool:
        if key == ord("Q") and action == 1 and mods == 0:
            visualizer.close()
        return False

    # GLFW key codes cover printable keys, Escape, navigation keys, and
    # modifiers. Registering each one prevents Open3D's default key actions.
    for key_code in range(32, 349):
        vis.register_key_action_callback(
            key_code,
            lambda visualizer, action, mods, key=key_code: on_key_action(
                visualizer, key, action, mods
            ),
        )
    return vis


def show_geometries(
    geometries,
    window_name: str = "Aurora - Vista 3D",
    show_coordinate_frame: bool = True,
    mesh_show_back_face: bool = False,
    style_cloud: o3d.geometry.PointCloud | None = None,
) -> None:
    """Abre geometrías 3D en una ventana Open3D común con estilo y controles consistentes."""
    vis = _create_q_only_visualizer()
    vis.create_window(window_name=window_name)
    for geometry in geometries:
        vis.add_geometry(geometry)
    if show_coordinate_frame:
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        vis.add_geometry(coord_frame)
    point_cloud = style_cloud
    if point_cloud is None:
        point_cloud = next(
            (geometry for geometry in geometries if isinstance(geometry, o3d.geometry.PointCloud)), None
        )
    if point_cloud is not None:
        _apply_sdk_render_style(vis, point_cloud)
    else:
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0.1, 0.1, 0.1])
        opt.mesh_show_back_face = mesh_show_back_face
    _start_at_sensor_pov(vis)
    vis.run()
    vis.destroy_window()


def show_point_cloud_pair(
    base_cloud: o3d.geometry.PointCloud,
    updated_cloud: o3d.geometry.PointCloud,
    window_name: str = "Aurora - Antes y despues (superpuestos)",
) -> None:
    """Display two unchanged clouds together, using colors to identify each stage."""
    base_view = o3d.geometry.PointCloud(base_cloud)
    updated_view = o3d.geometry.PointCloud(updated_cloud)
    base_view.paint_uniform_color((0.68, 0.72, 0.78))
    updated_view.paint_uniform_color((0.10, 0.72, 0.95))
    show_geometries(
        [base_view, updated_view],
        window_name=f"{window_name} - gris = ANTES, celeste = DESPUES",
        style_cloud=base_view,
    )


def _pick_points_in_shared_viewer(
    cloud: o3d.geometry.PointCloud,
    window_name: str,
    prompt: str,
    max_points: int = 0,
    context_clouds: list[o3d.geometry.PointCloud] | None = None,
    require_shift: bool = False,
    forward_axis: str = "z",
) -> np.ndarray:
    """Run the same decorated Open3D SceneWidget used by all inspection views."""
    with tempfile.TemporaryDirectory(prefix="aurora_point_picker_") as temp_dir:
        temp = Path(temp_dir)
        cloud_path = temp / "selection_cloud.ply"
        output_path = temp / "selected_points.npy"
        context_paths = []
        if not o3d.io.write_point_cloud(str(cloud_path), cloud):
            raise OSError("No se pudo preparar la nube para seleccionar puntos.")
        for index, context_cloud in enumerate(context_clouds or []):
            context_path = temp / f"context_cloud_{index}.ply"
            if not o3d.io.write_point_cloud(str(context_path), context_cloud):
                raise OSError("No se pudo preparar una nube de contexto para la selección.")
            context_paths.append(context_path)
        viewer = Path(__file__).resolve().with_name("viewer_host.py")
        command = [
            sys.executable,
            str(viewer),
            "pick",
            str(cloud_path),
            "--title",
            window_name,
            "--selection-output",
            str(output_path),
            "--pick-count",
            str(max_points),
            "--pick-prompt",
            prompt,
            "--forward-axis",
            forward_axis,
        ]
        for context_path in context_paths:
            command.extend(("--context-cloud", str(context_path)))
        if require_shift:
            command.append("--pick-requires-shift")
        subprocess.run(command, cwd=viewer.parent.parent, check=False)
        if not output_path.exists():
            return np.empty((0, 3), dtype=float)
        points = np.asarray(np.load(output_path), dtype=float).reshape((-1, 3))
        return points


def pick_landmark_points(cloud: o3d.geometry.PointCloud, window_name: str) -> np.ndarray | None:
    """
    Abre un visor 3D interactivo para elegir puntos de referencia (landmarks)
    con clic izquierdo sobre cada punto, en el mismo orden en
    que se van a elegir en la otra nube, despues cerrar la ventana (Q).
    Arranca en el punto de vista del sensor. Devuelve las coordenadas en el
    orden elegido, o None si se eligieron menos de 3 puntos.
    """
    picked_points = _pick_points_in_shared_viewer(
        cloud,
        window_name,
        "Clic izquierdo en los puntos de referencia en orden; Q para terminar",
    )
    if len(picked_points) < 3:
        return None
    return picked_points


def reference_photo_path_for(ply_path: Path) -> Path:
    """Convencion de nombre del sidecar con la foto de referencia (ver
    aurora_sensor.capture_reference_frame) asociada a una captura .ply."""
    return ply_path.with_name(ply_path.stem + "_ref.npz")


def save_reference_photo(ply_path: Path, image: np.ndarray, point_grid: np.ndarray) -> None:
    """Guarda la foto de referencia y su grid de puntos 3D junto a un .ply,
    para poder elegir puntos de alineacion sobre la foto mas adelante."""
    np.savez_compressed(reference_photo_path_for(ply_path), image=image, point_grid=point_grid)


def save_transformed_reference_photo(
    source_photo_path: Path,
    target_ply_path: Path,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> None:
    """Copy a reference photo sidecar while transforming its associated XYZ grid."""
    image, point_grid = load_reference_photo_file(source_photo_path)
    transformed_grid = np.asarray(point_grid, dtype=np.float64).copy()
    valid = np.all(np.isfinite(transformed_grid), axis=2)
    transformed_grid[valid] = (
        np.asarray(rotation, dtype=np.float64) @ transformed_grid[valid].T
    ).T + np.asarray(translation, dtype=np.float64)
    save_reference_photo(target_ply_path, image, transformed_grid)


def load_reference_photo_file(npz_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Carga una foto de referencia (imagen + grid de profundidad) desde una
    ruta .npz elegida directamente, sin asumir la convencion de nombre de
    reference_photo_path_for. Util cuando el .ply se movio/renombro a mano y
    el sidecar ya no queda junto a el con el nombre esperado."""
    data = np.load(npz_path)
    return data["image"], data["point_grid"]


def load_reference_photo(ply_path: Path) -> tuple[np.ndarray, np.ndarray] | None:
    """Carga la foto de referencia de un .ply si existe, o None si no se
    capturo ninguna para ese archivo (ej. .ply externo o sensor sin camara)."""
    path = reference_photo_path_for(ply_path)
    if not path.exists():
        return None
    return load_reference_photo_file(path)


def reference_pose_path_for(ply_path: Path) -> Path:
    """Convencion de nombre del sidecar con la pose del sensor (posicion +
    orientacion IMU/SLAM) al momento de una captura .ply — ver pestana
    'Alineacion IMU' en gui_gtk.py."""
    return ply_path.with_name(ply_path.stem + "_pose.npz")


class ReferencePose(tuple):
    """Keep the existing (position, rpy) API and carry its cloud axes/session."""
    def __new__(cls, position, rpy_deg, pose_to_cloud, frame_id, session_id):
        value = super().__new__(cls, (position, rpy_deg))
        value.pose_to_cloud = np.asarray(pose_to_cloud, dtype=float).reshape(3, 3)
        value.frame_id = str(frame_id)
        value.session_id = str(session_id)
        return value


def save_reference_pose(
    ply_path: Path,
    position: np.ndarray,
    rpy_deg: np.ndarray,
    alignment_version: int | None = None,
    source_pose=None,
) -> None:
    """Guarda la pose del sensor (posicion en metros, orientacion roll/pitch/yaw
    en grados) junto a un .ply, para poder guiar al usuario a devolver el
    sensor a esa misma posicion fisica antes de la siguiente captura."""
    values = {
        "position": np.asarray(position, dtype=np.float64),
        "rpy_deg": np.asarray(rpy_deg, dtype=np.float64),
    }
    if alignment_version is not None:
        values["alignment_version"] = np.asarray(int(alignment_version), dtype=np.int32)
    if source_pose is not None and hasattr(source_pose, "pose_to_cloud"):
        values.update(pose_to_cloud=source_pose.pose_to_cloud,
                      frame_id=source_pose.frame_id, session_id=source_pose.session_id)
        if hasattr(source_pose, "position_spread_m"):
            values["position_spread_m"] = source_pose.position_spread_m
        if hasattr(source_pose, "timestamp_ns"):
            values["timestamp_ns"] = np.asarray(source_pose.timestamp_ns, dtype=np.int64)
    np.savez_compressed(reference_pose_path_for(ply_path), **values)


def load_reference_pose(ply_path: Path) -> tuple[np.ndarray, np.ndarray] | None:
    """Carga la pose de referencia de un .ply si existe, o None si esa
    captura se hizo sin sensor conectado o antes de este cambio."""
    path = reference_pose_path_for(ply_path)
    if not path.exists():
        return None
    with np.load(path) as data:
        if "pose_to_cloud" in data.files:
            return ReferencePose(data["position"], data["rpy_deg"], data["pose_to_cloud"],
                                 data["frame_id"].item(), data["session_id"].item())
        return data["position"], data["rpy_deg"]


def reference_pose_alignment_version(ply_path: Path) -> int:
    """Version of the camera-frame pose transform recorded for an aligned PLY."""
    path = reference_pose_path_for(ply_path)
    if not path.exists():
        return 0
    with np.load(path) as data:
        if "alignment_version" not in data.files:
            return 0
        return int(np.asarray(data["alignment_version"]).item())


def compute_pose_relative_transform(
    base_pose: tuple[np.ndarray, np.ndarray],
    updated_pose: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """
    Calcula la rotacion (3x3) y traslacion (3,) necesarias para transformar puntos
    de la nube 'updated' al sistema de coordenadas de la nube 'base',
    usando las poses 6-DoF del sensor (SLAM/IMU).
    """
    base_pos, base_rpy = base_pose
    upd_pos, upd_rpy = updated_pose

    R_base = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(np.asarray(base_rpy, dtype=np.float64)))
    R_upd = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(np.asarray(upd_rpy, dtype=np.float64)))

    # Convert camera-local points to the Aurora pose frame, transform through
    # map coordinates, then convert into the base camera frame. Using the
    # pose rotation directly on camera XYZ swaps axes (forward is camera +Z).
    base_session = getattr(base_pose, "session_id", None)
    updated_session = getattr(updated_pose, "session_id", None)
    if base_session != updated_session:
        raise ValueError("Las poses pertenecen a sensores o sesiones de tracking distintas; vuelve a capturar ANTES y DESPUES sin reiniciar Point-LIO.")
    if base_session is not None and getattr(base_pose, "frame_id", None) != getattr(updated_pose, "frame_id", None):
        raise ValueError("Las poses no comparten marco de referencia.")
    pose_to_camera = getattr(base_pose, "pose_to_cloud", _POSE_TO_SENSOR_VIEW)
    if not np.allclose(pose_to_camera, getattr(updated_pose, "pose_to_cloud", _POSE_TO_SENSOR_VIEW)):
        raise ValueError("Las poses usan ejes de nube distintos.")
    R_rel = pose_to_camera @ R_base.T @ R_upd @ pose_to_camera.T
    t_rel = pose_to_camera @ R_base.T @ (
        np.asarray(upd_pos, dtype=np.float64) - np.asarray(base_pos, dtype=np.float64)
    )
    return R_rel, t_rel


def align_cloud_by_pose(
    cloud: o3d.geometry.PointCloud,
    base_pose: tuple[np.ndarray, np.ndarray],
    updated_pose: tuple[np.ndarray, np.ndarray],
) -> o3d.geometry.PointCloud:
    """Aplica la transformacion relativa de pose a una copia de la nube."""
    R_rel, t_rel = compute_pose_relative_transform(base_pose, updated_pose)
    return apply_rigid_transform(cloud, R_rel, t_rel)


def local_axes_from_rpy_deg(rpy_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Ejes propios del sensor (adelante/derecha/arriba) expresados en el
    sistema de coordenadas mundo/mapa, a partir de su orientacion
    roll/pitch/yaw (grados) en un instante dado. El marco del Aurora es
    X=adelante, Y=izquierda, Z=arriba; por eso derecha es -Y."""
    rotation = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(np.asarray(rpy_deg, dtype=np.float64)))
    forward = rotation @ np.array([1.0, 0.0, 0.0])
    right = rotation @ np.array([0.0, -1.0, 0.0])
    up = rotation @ np.array([0.0, 0.0, 1.0])
    return forward, right, up


def world_to_sensor_view_rotation(rpy_deg: np.ndarray) -> np.ndarray:
    """Transforma vectores del mapa a los ejes ópticos de la pose de referencia:
    X=derecha, Y=abajo, Z=adelante, como los frames XYZ de las nubes crudas."""
    pose_rotation = o3d.geometry.get_rotation_matrix_from_xyz(
        np.radians(np.asarray(rpy_deg, dtype=np.float64))
    )
    # En la pose Aurora: X=adelante, Y=izquierda, Z=arriba.
    # En el frame óptico: X=derecha, Y=abajo, Z=adelante.
    # La pose es base -> mundo. Convertimos primero mundo -> base con R.T y
    # luego Aurora base (X adelante, Y izquierda, Z arriba) -> nube óptica
    # (X derecha, Y abajo, Z adelante).
    return _POSE_TO_SENSOR_VIEW @ pose_rotation.T


def world_delta_to_sensor_view(delta_world: np.ndarray, reference_rpy_deg: np.ndarray) -> np.ndarray:
    """Expresa un desplazamiento del mapa en X=derecha, Y=abajo, Z=adelante
    respecto de la orientación guardada para la captura ANTES."""
    return world_to_sensor_view_rotation(reference_rpy_deg) @ np.asarray(
        delta_world, dtype=np.float64
    )


def direction_word_pairs_for_axes(rpy_deg: np.ndarray) -> dict[str, tuple[str, str]]:
    """Para cada eje del mundo (x, y, z), la palabra que corresponde a
    moverse en +eje y en -eje ('adelante'/'atras', 'derecha'/'izquierda' o
    'arriba'/'abajo') segun cual de los ejes propios del sensor (ver
    local_axes_from_rpy_deg) este mas alineado con ese eje del mundo."""
    forward, right, up = local_axes_from_rpy_deg(rpy_deg)
    candidates = [(forward, "adelante", "atras"), (right, "derecha", "izquierda"), (up, "arriba", "abajo")]

    pairs: dict[str, tuple[str, str]] = {}
    for index, axis in enumerate(("x", "y", "z")):
        world_axis = np.zeros(3)
        world_axis[index] = 1.0
        local_axis, positive_word, negative_word = max(
            candidates, key=lambda candidate: abs(float(np.dot(world_axis, candidate[0])))
        )
        projection = float(np.dot(world_axis, local_axis))
        pairs[axis] = (positive_word, negative_word) if projection >= 0 else (negative_word, positive_word)
    return pairs


def render_reference_photo_from_cloud(
    cloud: o3d.geometry.PointCloud,
    width: int = 960,
    height: int = 720,
    fov_deg: float = 140.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Genera una foto de referencia SINTETICA (no una foto de camara real) a
    partir de una nube de puntos, proyectandola con un modelo de camara
    pinhole simple desde el origen mirando hacia +Z (misma convencion que
    _start_at_sensor_pov: X=derecha, Y=abajo). Sirve de respaldo cuando la
    captura real (aurora_sensor.capture_reference_frame) no entrego un frame
    organizado o no habia imagen de camara disponible: el punto de
    referencia sigue siendo elegible por click en una imagen, aunque esa
    imagen no sea la foto real del sensor.

    Para cada punto se calcula su pixel proyectado; cuando varios puntos
    caen en el mismo pixel, gana el mas cercano a la camara (z minimo, tipo
    z-buffer). fov_deg=140 por defecto porque las capturas de tunel quedan
    muy cerca del sensor (pared a centimetros), un FOV de camara "normal"
    (~60-70 grados) dejaria la enorme mayoria de los puntos fuera de cuadro.

    Devuelve (image, point_grid) con el mismo contrato que
    load_reference_photo(): image uint8 (H,W,3), point_grid float64 (H,W,3)
    con NaN donde no se proyecto ningun punto.
    """
    points = np.asarray(cloud.points, dtype=np.float64)
    if points.shape[0] == 0:
        raise ValueError("La nube no tiene puntos.")

    if cloud.has_colors():
        colors = np.asarray(cloud.colors, dtype=np.float64)
    else:
        y = points[:, 1]
        y_norm = (y - y.min()) / max(y.max() - y.min(), 1e-9)
        colors = np.stack([y_norm, np.zeros_like(y_norm), 1.0 - y_norm], axis=1)

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    valid = z > 1e-6
    if not np.any(valid):
        raise ValueError("Ningun punto queda delante de la camara (z > 0).")
    x, y, z = x[valid], y[valid], z[valid]
    point_colors = colors[valid]
    point_xyz = points[valid]

    fov_rad = np.deg2rad(fov_deg)
    focal = (width / 2.0) / np.tan(fov_rad / 2.0)
    cx, cy = width / 2.0, height / 2.0

    u = (focal * x / z + cx).astype(np.int64)
    v = (focal * y / z + cy).astype(np.int64)

    in_frame = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    u, v, z_f = u[in_frame], v[in_frame], z[in_frame]
    point_colors = point_colors[in_frame]
    point_xyz = point_xyz[in_frame]

    image = np.zeros((height, width, 3), dtype=np.uint8)
    point_grid = np.full((height, width, 3), np.nan, dtype=np.float64)

    # Pintar en orden de profundidad decreciente: el ultimo write en cada
    # pixel es el punto mas cercano (mismo efecto que un z-buffer, sin
    # necesitar un renderer grafico).
    order = np.argsort(-z_f)
    u, v = u[order], v[order]
    point_colors = point_colors[order]
    point_xyz = point_xyz[order]

    image[v, u] = np.clip(point_colors * 255.0, 0, 255).astype(np.uint8)
    point_grid[v, u] = point_xyz

    if np.count_nonzero(~np.isnan(point_grid[..., 0])) == 0:
        raise ValueError("La proyeccion no genero ningun pixel valido; revisa fov_deg u orientacion de la nube.")

    return image, point_grid


def _nearest_valid_point(point_grid: np.ndarray, row: int, col: int, max_radius: int = 6) -> np.ndarray | None:
    """Busca el punto 3D valido mas cercano a (row, col) en un grid chico
    creciente, porque un objeto fino (ej. la punta de un perno) puede no
    tener profundidad valida exactamente en el pixel clickeado."""
    height, width = point_grid.shape[:2]
    for radius in range(max_radius + 1):
        r0, r1 = max(0, row - radius), min(height, row + radius + 1)
        c0, c1 = max(0, col - radius), min(width, col + radius + 1)
        patch = point_grid[r0:r1, c0:c1]
        valid = ~np.isnan(patch[..., 0])
        if valid.any():
            return patch[valid][0]
    return None


def pick_landmark_points_from_photo(
    image: np.ndarray, point_grid: np.ndarray, window_title: str, log=print
) -> np.ndarray | None:
    """
    Igual que pick_landmark_points, pero eligiendo los puntos sobre la foto
    de referencia (2D) en vez de la nube 3D cruda: util cuando el punto de
    referencia (ej. la punta de un perno tapado por shotcrete) se ve claro
    en la foto pero no como puntos 3D limpios. Cada click se traduce a la
    coordenada 3D real usando 'point_grid'. Click en orden, cerrar la
    ventana para terminar; devuelve None si se eligieron menos de 3 puntos.
    """
    import matplotlib.pyplot as plt

    depth_height, depth_width = point_grid.shape[:2]
    img_height, img_width = image.shape[:2]

    picked: list[np.ndarray] = []

    fig, ax = plt.subplots()
    fig.canvas.manager.set_window_title(window_title)
    ax.imshow(image)
    ax.set_axis_off()
    ax.set_title("Click en cada punto de referencia, en orden. Cerrar la ventana para terminar.")

    def on_click(event) -> None:
        if event.xdata is None or event.ydata is None or event.button != 1:
            return
        depth_col = int(event.xdata * depth_width / img_width)
        depth_row = int(event.ydata * depth_height / img_height)
        depth_row = min(max(depth_row, 0), depth_height - 1)
        depth_col = min(max(depth_col, 0), depth_width - 1)

        point = _nearest_valid_point(point_grid, depth_row, depth_col)
        if point is None:
            log("Ese punto no tiene profundidad valida cerca; elige otro.")
            return

        picked.append(point)
        ax.plot(event.xdata, event.ydata, "o", color="lime", markersize=8, markeredgecolor="black")
        ax.annotate(str(len(picked)), (event.xdata, event.ydata), color="black", fontsize=9, ha="center", va="center")
        fig.canvas.draw()

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.show()

    if len(picked) < 3:
        return None
    return np.asarray(picked)


def compute_c2c_distance(
    updated: o3d.geometry.PointCloud, base: o3d.geometry.PointCloud
) -> np.ndarray:
    distances = updated.compute_point_cloud_distance(base)
    return np.asarray(distances)


def summarize(distances: np.ndarray) -> DistanceStats:
    return summarize_filtered(distances)


def filter_measurements(
    points: np.ndarray, distances: np.ndarray, minimum_thickness_m: float = 0.0
) -> tuple[np.ndarray, np.ndarray, int]:
    """Exclude below-threshold measurements while keeping points aligned to distances."""
    points = np.asarray(points)
    distances = np.asarray(distances, dtype=np.float64)
    if not np.isfinite(minimum_thickness_m) or minimum_thickness_m < 0:
        raise ValueError("El umbral mínimo de espesor debe ser cero o mayor.")
    if len(points) != len(distances):
        raise ValueError("Puntos y distancias no tienen el mismo tamaño.")
    mask = distances >= minimum_thickness_m if minimum_thickness_m > 0 else np.ones(len(distances), dtype=bool)
    excluded = int(len(distances) - np.count_nonzero(mask))
    return points[mask], distances[mask], excluded


def summarize_filtered(
    distances: np.ndarray,
    input_n_points: int | None = None,
    excluded_n_points: int = 0,
    minimum_thickness_m: float = 0.0,
) -> DistanceStats:
    distances = np.asarray(distances, dtype=np.float64)
    if not distances.size:
        raise ValueError("El umbral mínimo excluyó todos los puntos; reduce el umbral.")
    return DistanceStats(
        mean=float(np.mean(distances)),
        median=float(np.median(distances)),
        std=float(np.std(distances)),
        min=float(np.min(distances)),
        max=float(np.max(distances)),
        p95=float(np.percentile(distances, 95)),
        n_points=int(distances.size),
        input_n_points=int(input_n_points if input_n_points is not None else distances.size),
        excluded_n_points=int(excluded_n_points),
        minimum_thickness_m=float(minimum_thickness_m),
    )


# Extremos de la escala continua de espesor (paleta industrial de alto contraste).
HEATMAP_COLOR_LOW = (0.0, 0.482, 1.0)  # azul #007AFF
HEATMAP_COLOR_HIGH = (1.0, 0.231, 0.188)  # rojo #FF3B30


def build_heatmap_cloud(
    updated: o3d.geometry.PointCloud,
    distances: np.ndarray,
    max_distance: float | None,
) -> o3d.geometry.PointCloud:
    clip_max = max_distance if max_distance else float(np.percentile(distances, 98))
    clip_max = max(clip_max, 1e-9)
    normalized = np.clip(distances / clip_max, 0.0, 1.0)[:, None]

    # Azul (bajo espesor) -> rojo (alto espesor), paleta "safety" de alto contraste.
    low_color = np.array(HEATMAP_COLOR_LOW)
    high_color = np.array(HEATMAP_COLOR_HIGH)
    colors = low_color * (1 - normalized) + high_color * normalized

    heatmap_cloud = o3d.geometry.PointCloud(updated)
    heatmap_cloud.colors = o3d.utility.Vector3dVector(colors)
    return heatmap_cloud


def build_subtle_overlay_cloud(
    updated: o3d.geometry.PointCloud,
    distances: np.ndarray,
    max_distance: float | None = None,
    base_gray: float = 0.72,
    highlight_color: tuple[float, float, float] = (0.85, 0.55, 0.20),
) -> o3d.geometry.PointCloud:
    """
    Colorea la nube 'actualizada' con un resaltado sutil (gris -> ambar tenue)
    proporcional a la diferencia con la base, pensado para superponerla sobre
    la nube base y ver la diferencia sin un heatmap llamativo tipo arcoiris.
    """
    clip_max = max_distance if max_distance else float(np.percentile(distances, 95))
    clip_max = max(clip_max, 1e-9)
    t = np.clip(distances / clip_max, 0.0, 1.0)[:, None]

    base_color = np.array([base_gray, base_gray, base_gray])
    highlight = np.array(highlight_color)
    colors = base_color * (1 - t) + highlight * t

    overlay_cloud = o3d.geometry.PointCloud(updated)
    overlay_cloud.colors = o3d.utility.Vector3dVector(colors)
    return overlay_cloud


# Verde = espesor bajo (dentro de spec / insuficiente), amarillo = medio, rojo = alto.
BAND_COLOR_LOW = (0.204, 0.780, 0.349)  # verde #34C759
BAND_COLOR_MEDIUM = (1.0, 0.8, 0.0)  # amarillo #FFCC00
BAND_COLOR_HIGH = (1.0, 0.231, 0.188)  # rojo #FF3B30


def build_heatmap_cloud_banded(
    updated: o3d.geometry.PointCloud,
    distances: np.ndarray,
    low_threshold: float,
    high_threshold: float,
) -> o3d.geometry.PointCloud:
    """
    Colorea la nube en 3 niveles discretos segun el espesor (en metros),
    en vez de un gradiente continuo: <low_threshold -> verde,
    [low_threshold, high_threshold) -> amarillo, >=high_threshold -> rojo.
    """
    colors = np.empty((distances.shape[0], 3))
    low_mask = distances < low_threshold
    high_mask = distances >= high_threshold
    medium_mask = ~low_mask & ~high_mask

    colors[low_mask] = BAND_COLOR_LOW
    colors[medium_mask] = BAND_COLOR_MEDIUM
    colors[high_mask] = BAND_COLOR_HIGH

    banded_cloud = o3d.geometry.PointCloud(updated)
    banded_cloud.colors = o3d.utility.Vector3dVector(colors)
    return banded_cloud


# Escala termica de 6 niveles equidistantes segun el espesor objetivo definido
# por el usuario (0 = frio/sin aplicar, tope = caliente/objetivo alcanzado).
SIX_BAND_LABELS = ("Muy Frio", "Frio", "Fresco", "Templado", "Calido", "Caliente")
SIX_BAND_COLORS = (
    (0.557, 0.267, 0.678),  # Muy Frio - Morado
    (0.0, 0.482, 1.0),  # Frio - Azul #007AFF
    (0.0, 0.780, 0.820),  # Fresco - Cian
    (0.204, 0.780, 0.349),  # Templado - Verde #34C759
    (1.0, 0.549, 0.0),  # Calido - Naranja #FF8C00
    (1.0, 0.231, 0.188),  # Caliente - Rojo #FF3B30
)


def build_heatmap_cloud_six_bands(
    updated: o3d.geometry.PointCloud,
    distances: np.ndarray,
    target_thickness_m: float,
) -> o3d.geometry.PointCloud:
    """
    Colorea la nube en 6 niveles equidistantes entre 0 y 'target_thickness_m'
    (el espesor objetivo definido por el usuario), con la escala termica
    Muy Frio/Frio/Fresco/Templado/Calido/Caliente (morado -> azul -> cian ->
    verde -> naranja -> rojo). Puntos por debajo de 0 o por encima del
    objetivo se recortan a la primera/ultima banda.
    """
    target_thickness_m = max(float(target_thickness_m), 1e-9)
    band_width = target_thickness_m / len(SIX_BAND_COLORS)
    band_idx = np.clip((distances / band_width).astype(np.int64), 0, len(SIX_BAND_COLORS) - 1)
    colors = np.asarray(SIX_BAND_COLORS)[band_idx]

    six_band_cloud = o3d.geometry.PointCloud(updated)
    six_band_cloud.colors = o3d.utility.Vector3dVector(colors)
    return six_band_cloud


def grayscale_point_cloud(cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Copy a cloud while preserving its native brightness as monochrome gray."""
    result = o3d.geometry.PointCloud(cloud)
    if not result.has_colors():
        result.paint_uniform_color((0.45, 0.45, 0.45))
        return result
    rgb = np.asarray(result.colors, dtype=np.float64)
    luminance = np.clip(rgb @ np.array([0.2126, 0.7152, 0.0722]), 0.0, 1.0)
    result.colors = o3d.utility.Vector3dVector(np.repeat(luminance[:, None], 3, axis=1))
    return result


def prepare_thickness_overlay(
    base: o3d.geometry.PointCloud,
    updated: o3d.geometry.PointCloud,
    color_spec: dict | None = None,
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud]:
    """Return native grayscale ANTES and a DESPUES cloud colored by C2C thickness."""
    spec = color_spec or {"mode": "continuous"}
    distances = compute_c2c_distance(updated, base)
    mode = spec.get("mode", "continuous")
    if mode == "six":
        colored = build_heatmap_cloud_six_bands(
            updated, distances, float(spec["target_m"])
        )
    elif mode == "three":
        colored = build_heatmap_cloud_banded(
            updated,
            distances,
            float(spec["low_m"]),
            float(spec["high_m"]),
        )
    else:
        colored = build_heatmap_cloud(
            updated, distances, spec.get("max_distance_m")
        )
    return grayscale_point_cloud(base), colored


@dataclass
class RaycastPipelineParams:
    base_path: Path
    updated_path: Path
    output_dir: Path
    poisson_depth: int = 9
    density_trim_quantile: float = 0.02
    max_distance: float | None = None
    minimum_thickness_m: float = 0.0
    # Origen del sensor a usar si el .ply no tiene sidecar de pose (p. ej.
    # nubes cargadas a mano, sin sensor conectado, para probar el flujo).
    # Si es None se exige el sidecar (ver run_raycast_pipeline). Solo se usa
    # si ray_method == "origin".
    base_origin: Vec3 | None = None
    updated_origin: Vec3 | None = None
    # "poisson" (mesh_from_point_cloud, pensado para nubes completas) o
    # "delaunay" (mesh_from_point_cloud_delaunay, pensado para un recorte
    # casi-plano, ver pestaña 'Preparar nubes').
    mesh_method: str = "poisson"
    # Preserve every captured point by default. Users can set a voxel when
    # they need faster reconstruction; an upper point-count guard remains.
    mesh_voxel_size: float = 0.0
    # "origin": raycast_thickness, un rayo por cada vertice desde la pose
    # guardada del sensor (ver base_origin/updated_origin arriba) — asume
    # que el sensor no se movio entre capturas.
    # "normal": raycast_thickness_along_normal, un rayo por cada vertice de
    # la malla base a lo largo de la normal del plano que mejor la ajusta
    # — no necesita la pose del sensor, pero SI necesita que las dos nubes
    # ya compartan sistema de coordenadas (alineacion IMU o por puntos de
    # referencia aplicada de antemano).
    ray_method: str = "origin"
    delaunay_max_edge_trim_factor: float = 4.0
    mesh_smoothing_iterations: int = 0
    color_spec: dict | None = None


@dataclass
class RaycastPipelineResult:
    stats: DistanceStats
    distances: np.ndarray
    hit_points: np.ndarray
    unmatched_points: np.ndarray
    base_mesh: o3d.geometry.TriangleMesh
    updated_mesh: o3d.geometry.TriangleMesh
    heatmap_cloud: o3d.geometry.PointCloud
    csv_path: Path
    histogram_path: Path
    heatmap_path: Path


def mesh_from_point_cloud(
    cloud: o3d.geometry.PointCloud,
    poisson_depth: int = 9,
    density_trim_quantile: float = 0.02,
) -> o3d.geometry.TriangleMesh:
    """
    Reconstruye una malla triangulada a partir de una nube de puntos via
    Poisson surface reconstruction (usada por el flujo experimental de
    raycasting, ver run_raycast_pipeline). Requiere normales orientadas de
    forma consistente (Poisson las necesita para saber que lado de la
    superficie es 'afuera'); si la nube no las trae, se estiman. Se recortan
    los triangulos de menor densidad (artefacto tipico de Poisson en zonas
    con pocos puntos, ej. bordes/huecos del escaneo) segun
    'density_trim_quantile'.
    """
    cloud = o3d.geometry.PointCloud(cloud)
    if not cloud.has_normals():
        cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        try:
            cloud.orient_normals_consistent_tangent_plane(30)
        except RuntimeError:
            # Open3D/Qhull puede fallar al orientar una nube perfectamente
            # plana o regular. En la captura Aurora los puntos estan en el
            # marco de la camara; orientar hacia el origen conserva una
            # orientacion consistente sin perturbar la geometria medida.
            cloud.orient_normals_towards_camera_location(np.zeros(3))

    if not np.all(np.isfinite(np.asarray(cloud.normals))):
        raise ValueError("No se pudieron estimar normales finitas para reconstruir la malla Poisson.")

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cloud, depth=poisson_depth)
    if density_trim_quantile > 0:
        densities = np.asarray(densities)
        threshold = np.quantile(densities, density_trim_quantile)
        mesh.remove_vertices_by_mask(densities < threshold)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    return mesh


def smooth_triangle_mesh(
    mesh: o3d.geometry.TriangleMesh,
    iterations: int = 0,
) -> o3d.geometry.TriangleMesh:
    """Apply optional Taubin smoothing without modifying the source mesh."""
    result = o3d.geometry.TriangleMesh(mesh)
    iterations = max(0, int(iterations))
    if iterations:
        result = result.filter_smooth_taubin(number_of_iterations=iterations)
        result.remove_degenerate_triangles()
        result.remove_unreferenced_vertices()
        result.compute_vertex_normals()
    return result


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Centroide y ejes (u, v, normal) del plano que mejor ajusta 'points' via
    PCA/SVD. El ajuste conserva la orientacion real del tramo en vez de forzar los ejes
    del plano a los ejes globales. Aca se usa la
    normal cruda del ajuste — la mejor aproximacion al plano real, sin
    importar su orientacion respecto a los ejes del mundo. El signo de la
    normal es arbitrario; quien llama decide hacia donde orientarla.
    """
    points = np.asarray(points, dtype=np.float64)
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
    u_axis, v_axis, normal = vt[0], vt[1], vt[2]
    return centroid, u_axis, v_axis, normal


def mesh_from_point_cloud_delaunay(
    cloud: o3d.geometry.PointCloud,
    max_edge_trim_factor: float = 4.0,
) -> o3d.geometry.TriangleMesh:
    """
    Reconstruye una malla triangulada a partir de una nube CASI PLANA via
    triangulacion de Delaunay 2D: ajusta un plano (fit_plane) y triangula
    la proyeccion (u, v) de cada punto sobre ese plano con
    'scipy.spatial.Delaunay', usando la posicion 3D real de cada punto para
    los vertices de la malla resultante. Pensada para un tramo de pared de
    tunel ya recortado (pestaña 'Preparar nubes'), no para una nube completa
    con curvatura fuerte — a diferencia de Poisson (mesh_from_point_cloud),
    no necesita una superficie cerrada/watertight ni normales orientadas,
    por lo que no genera artefactos en los bordes abiertos de un recorte.

    Delaunay 2D triangula todo el casco convexo de la proyeccion, lo que
    puede crear triangulos largos y finos puenteando huecos o bordes
    concavos de la nube; se descartan los triangulos con algun lado mas
    largo que 'max_edge_trim_factor' veces la mediana de longitud de lado
    de toda la triangulacion (mismo espiritu que 'density_trim_quantile' en
    Poisson, pero basado en longitud de arista en vez de densidad).
    """
    from scipy.spatial import Delaunay

    points = np.asarray(cloud.points, dtype=np.float64)
    if points.shape[0] < 3:
        raise ValueError("Se necesitan al menos 3 puntos para triangular (Delaunay).")

    centroid, u_axis, v_axis, _ = fit_plane(points)
    rel = points - centroid
    points_2d = np.column_stack([rel @ u_axis, rel @ v_axis])

    triangles = Delaunay(points_2d).simplices

    edge_ab = np.linalg.norm(points[triangles[:, 0]] - points[triangles[:, 1]], axis=1)
    edge_bc = np.linalg.norm(points[triangles[:, 1]] - points[triangles[:, 2]], axis=1)
    edge_ca = np.linalg.norm(points[triangles[:, 2]] - points[triangles[:, 0]], axis=1)
    max_edge = np.maximum(np.maximum(edge_ab, edge_bc), edge_ca)
    median_edge = float(np.median(np.concatenate([edge_ab, edge_bc, edge_ca])))
    triangles = triangles[max_edge <= max_edge_trim_factor * median_edge]

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(points)
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    mesh.remove_unreferenced_vertices()
    mesh.remove_degenerate_triangles()
    mesh.compute_vertex_normals()
    return mesh


def raycast_thickness_along_normal(
    base_mesh: o3d.geometry.TriangleMesh,
    updated_mesh: o3d.geometry.TriangleMesh,
    max_distance: float | None = None,
    sample_points: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Alternativa a 'raycast_thickness' que no depende de la pose del sensor:
    ajusta un plano a los vertices de 'base_mesh' (fit_plane) y tira, desde
    cada vertice, UN rayo en la direccion de la normal de ESE plano — la
    misma direccion para todos los vertices, no la normal local de cada
    triangulo (que seria ruidosa en una malla Delaunay por bordes
    irregulares) — hacia 'updated_mesh'. La normal se orienta
    automaticamente hacia el centroide de 'updated_mesh', para que el rayo
    viaje del lado donde se aplico el shotcrete.

    Requiere que ambas mallas YA COMPARTAN sistema de coordenadas
    (alineacion IMU o por puntos de referencia, pestaña 'Alineacion',
    aplicada de antemano) — a diferencia de 'raycast_thickness', este
    metodo no tiene forma de detectar ni corregir un desplazamiento del
    sensor entre capturas, lo mediria como si fuera espesor real.

    Devuelve distancias, puntos de impacto y vertices base cuyos rayos no
    impactaron la malla actualizada (o excedieron max_distance).
    """
    # Ray sampling follows the original cloud when supplied. Mesh vertices are
    # reconstruction artifacts (especially for Poisson) and can add or remove
    # samples independently of the captured point data.
    base_vertices = np.asarray(base_mesh.vertices) if sample_points is None else np.asarray(sample_points)
    if base_vertices.ndim != 2 or base_vertices.shape[1] != 3 or not len(base_vertices):
        raise ValueError("La nube base no contiene puntos validos para lanzar rayos.")
    centroid, _, _, normal = fit_plane(base_vertices)
    updated_centroid = np.asarray(updated_mesh.vertices).mean(axis=0)
    if np.dot(normal, updated_centroid - centroid) < 0:
        normal = -normal

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(updated_mesh))

    directions = np.tile(normal, (base_vertices.shape[0], 1))
    rays = o3d.core.Tensor(
        np.concatenate([base_vertices, directions], axis=1).astype(np.float32)
    )
    t_hit = scene.cast_rays(rays)["t_hit"].numpy()

    valid = np.isfinite(t_hit)
    if max_distance is not None:
        valid &= t_hit <= max_distance
    distances = t_hit[valid].astype(np.float64)
    hit_points = base_vertices[valid] + normal * distances[:, None]
    return distances, hit_points, base_vertices[~valid]


def raycast_thickness(
    base_mesh: o3d.geometry.TriangleMesh,
    updated_mesh: o3d.geometry.TriangleMesh,
    base_origin: np.ndarray,
    updated_origin: np.ndarray,
    sample_points: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Estima el espesor por raycasting en vez de vecino-mas-cercano (C2C):
    para cada vertice de la malla 'actualizada' (con shotcrete), tira un
    rayo desde el origen del sensor (posicion guardada al capturar, ver
    save_reference_pose) pasando por ese vertice, y el MISMO rayo (mismo
    origen+direccion) contra la malla 'base'. El espesor es la resta de las
    distancias de impacto (base - actualizada): positivo si la superficie
    con shotcrete quedo mas cerca del sensor que la original, como se
    espera fisicamente.

    Requiere que el sensor no se haya movido entre ambas capturas (o que se
    haya vuelto a la misma posicion via la pestaña 'Alineacion IMU') — si
    'base_origin' y 'updated_origin' difieren, las direcciones no apuntan
    al mismo punto fisico en ambas mallas y el resultado deja de ser
    valido; quien llama a esta funcion es responsable de avisar si el
    desfasaje es grande (ver run_raycast_pipeline).

    Devuelve distancias, impactos sobre la malla actualizada y vertices de
    esa malla cuyos rayos no impactaron ambas superficies.
    """
    scene_updated = o3d.t.geometry.RaycastingScene()
    scene_updated.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(updated_mesh))
    scene_base = o3d.t.geometry.RaycastingScene()
    scene_base.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(base_mesh))

    base_origin = np.asarray(base_origin, dtype=np.float64)
    updated_origin = np.asarray(updated_origin, dtype=np.float64)

    # Use captured DESPUES samples, not generated mesh vertices, as the ray
    # directions. This keeps the result tied to the original point cloud.
    vertices = np.asarray(updated_mesh.vertices) if sample_points is None else np.asarray(sample_points)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices):
        raise ValueError("La nube actualizada no contiene puntos validos para lanzar rayos.")
    offsets = vertices - updated_origin
    norms = np.linalg.norm(offsets, axis=1)
    valid_dir = norms > 1e-9
    directions = offsets[valid_dir] / norms[valid_dir, None]

    origins_updated = np.tile(updated_origin, (directions.shape[0], 1))
    origins_base = np.tile(base_origin, (directions.shape[0], 1))

    rays_updated = o3d.core.Tensor(
        np.concatenate([origins_updated, directions], axis=1).astype(np.float32)
    )
    rays_base = o3d.core.Tensor(
        np.concatenate([origins_base, directions], axis=1).astype(np.float32)
    )

    t_updated = scene_updated.cast_rays(rays_updated)["t_hit"].numpy()
    t_base = scene_base.cast_rays(rays_base)["t_hit"].numpy()

    valid_hit = np.isfinite(t_updated) & np.isfinite(t_base)
    distances = t_base[valid_hit] - t_updated[valid_hit]
    hit_points = origins_updated[valid_hit] + directions[valid_hit] * t_updated[valid_hit, None]
    valid_vertices = vertices[valid_dir]
    unmatched_points = np.concatenate((vertices[~valid_dir], valid_vertices[~valid_hit]), axis=0)
    return distances, hit_points, unmatched_points


def run_raycast_pipeline(params: RaycastPipelineParams, log=print) -> RaycastPipelineResult:
    """
    Flujo experimental alternativo a run_pipeline: en vez de Cloud-to-Cloud
    (vecino mas cercano), reconstruye una malla de cada nube
    (mesh_from_point_cloud via Poisson, o mesh_from_point_cloud_delaunay) y
    mide el espesor por raycasting, con uno de dos metodos
    (params.ray_method):

    - "origin": un rayo por punto de la nube DESPUES desde la pose guardada del sensor (ver
      raycast_thickness). Necesita el sidecar de pose ('_pose.npz') en
      AMBAS capturas, o un origen manual; asume que el sensor no se movio
      entre capturas.
    - "normal": un rayo por punto de la nube ANTES a lo largo de la
      normal del plano que mejor la ajusta (ver
      raycast_thickness_along_normal). No necesita pose del sensor, pero
      exige que las dos nubes ya compartan sistema de coordenadas
      (alineacion aplicada de antemano) — responsabilidad de quien llama.
    """
    params.output_dir.mkdir(parents=True, exist_ok=True)

    base_origin = updated_origin = None
    if params.ray_method == "origin":
        if params.base_origin is not None:
            base_origin = np.asarray(params.base_origin, dtype=np.float64)
            log(f"Origen del sensor (base) fijado a mano: {base_origin.tolist()}")
        else:
            base_pose = load_reference_pose(params.base_path)
            if base_pose is None:
                raise ValueError(
                    "No hay sidecar de pose ('_pose.npz') para la nube base y no se dio "
                    "un origen manual. Vuelve a capturar con el sensor conectado, o "
                    "completa el origen manual en la pestaña 'Raycasting (prueba)'."
                )
            base_origin = np.asarray(base_pose[0], dtype=np.float64)

        if params.updated_origin is not None:
            updated_origin = np.asarray(params.updated_origin, dtype=np.float64)
            log(f"Origen del sensor (actualizada) fijado a mano: {updated_origin.tolist()}")
        else:
            updated_pose = load_reference_pose(params.updated_path)
            if updated_pose is None:
                raise ValueError(
                    "No hay sidecar de pose ('_pose.npz') para la nube actualizada y no "
                    "se dio un origen manual. Vuelve a capturar con el sensor conectado, "
                    "o completa el origen manual en la pestaña 'Raycasting (prueba)'."
                )
            updated_origin = np.asarray(updated_pose[0], dtype=np.float64)
        origin_gap_cm = float(np.linalg.norm(updated_origin - base_origin)) * 100
        if origin_gap_cm > 2.0:
            log(
                f"Aviso: el sensor se movio {origin_gap_cm:.1f} cm entre capturas segun "
                "la pose guardada. El metodo de raycasting asume que el sensor no se "
                "movio (o volvio a la misma posicion via 'Alineacion IMU'); el "
                "resultado puede no ser fisicamente valido."
            )

    log("Cargando nubes de puntos...")
    base_source = load_point_cloud(params.base_path)
    updated_source = load_point_cloud(params.updated_path)
    base_cloud = base_source
    updated_cloud = updated_source
    log(f"  Base       : {len(base_cloud.points)} puntos")
    log(f"  Actualizada: {len(updated_cloud.points)} puntos")

    voxel_size = max(0.0, float(params.mesh_voxel_size))
    if voxel_size > 0:
        log(f"Reduciendo densidad de malla (voxel {voxel_size * 100:.2f} cm)...")
        base_cloud = base_cloud.voxel_down_sample(voxel_size)
        updated_cloud = updated_cloud.voxel_down_sample(voxel_size)

    # Keep Qhull/Poisson work bounded for full-resolution Aurora scans. Increase
    # the voxel only when the selected resolution still leaves an unusually
    # large cloud, and report the effective value in the calculation log.
    max_mesh_points = 350_000
    while max(len(base_cloud.points), len(updated_cloud.points)) > max_mesh_points:
        voxel_size = max(voxel_size * 1.25, 0.005)
        log(
            f"Ajustando resolucion para limitar la malla a {max_mesh_points:,} puntos "
            f"(voxel {voxel_size * 100:.2f} cm)..."
        )
        base_cloud = base_source.voxel_down_sample(voxel_size)
        updated_cloud = updated_source.voxel_down_sample(voxel_size)
    log(
        f"  Nubes para malla: {len(base_cloud.points):,} y {len(updated_cloud.points):,} "
        f"puntos (voxel efectivo {voxel_size * 100:.2f} cm)."
    )

    mesh_label = "Poisson" if params.mesh_method == "poisson" else "Delaunay"
    log(f"Reconstruyendo malla ({mesh_label}) de la nube base...")
    if params.mesh_method == "delaunay":
        base_mesh = mesh_from_point_cloud_delaunay(base_cloud, params.delaunay_max_edge_trim_factor)
    else:
        base_mesh = mesh_from_point_cloud(base_cloud, params.poisson_depth, params.density_trim_quantile)
    base_mesh = smooth_triangle_mesh(base_mesh, params.mesh_smoothing_iterations)
    log(f"  Malla base: {len(base_mesh.vertices)} vertices, {len(base_mesh.triangles)} triangulos")

    log(f"Reconstruyendo malla ({mesh_label}) de la nube actualizada...")
    if params.mesh_method == "delaunay":
        updated_mesh = mesh_from_point_cloud_delaunay(updated_cloud, params.delaunay_max_edge_trim_factor)
    else:
        updated_mesh = mesh_from_point_cloud(updated_cloud, params.poisson_depth, params.density_trim_quantile)
    updated_mesh = smooth_triangle_mesh(updated_mesh, params.mesh_smoothing_iterations)
    log(f"  Malla actualizada: {len(updated_mesh.vertices)} vertices, {len(updated_mesh.triangles)} triangulos")

    if params.ray_method == "normal":
        log("Tirando un rayo por punto original ANTES contra la malla DESPUES...")
        distances, hit_points, unmatched_points = raycast_thickness_along_normal(
            base_mesh, updated_mesh, params.max_distance, np.asarray(base_source.points)
        )
    else:
        log("Tirando un rayo por punto original DESPUES desde el origen del sensor...")
        distances, hit_points, unmatched_points = raycast_thickness(
            base_mesh, updated_mesh, base_origin, updated_origin, np.asarray(updated_source.points)
        )
    if distances.size == 0:
        raise ValueError(
            "Ningun rayo impacto ambas mallas. Revisa que las dos capturas cubran "
            "la misma zona del tunel."
        )
    input_n_points = len(distances)
    hit_count = len(distances)
    hit_points, distances, excluded_count = filter_measurements(
        hit_points, distances, params.minimum_thickness_m
    )
    stats = summarize_filtered(
        distances,
        input_n_points=input_n_points,
        excluded_n_points=excluded_count,
        minimum_thickness_m=params.minimum_thickness_m,
    )
    sample_total = len(base_source.points) if params.ray_method == "normal" else len(updated_source.points)
    unmatched_count = len(unmatched_points)
    coverage = 100.0 * hit_count / max(1, sample_total)
    log(
        f"Cobertura de rayos: {hit_count:,}/{sample_total:,} puntos originales "
        f"({coverage:.1f}%); sin coincidencia: {unmatched_count:,}."
    )
    if params.minimum_thickness_m > 0:
        log(
            f"Umbral mínimo {params.minimum_thickness_m * 100:.2f} cm: "
            f"{excluded_count:,}/{input_n_points:,} puntos excluidos."
        )
    log("")
    log("Resultados (raycasting):")
    log(str(stats))

    csv_path = params.output_dir / "thickness_raycast_per_point.csv"
    save_distances_csv(csv_path, hit_points, distances)
    log(f"CSV guardado en: {csv_path}")

    histogram_path = params.output_dir / "thickness_raycast_histogram.png"
    save_histogram(histogram_path, distances)
    log(f"Histograma guardado en: {histogram_path}")

    hit_cloud = o3d.geometry.PointCloud()
    hit_cloud.points = o3d.utility.Vector3dVector(hit_points)
    spec = params.color_spec or {"mode": "continuous", "max_distance_m": params.max_distance}
    if spec.get("mode") == "six":
        heatmap_cloud = build_heatmap_cloud_six_bands(hit_cloud, distances, float(spec["target_m"]))
    elif spec.get("mode") == "three":
        heatmap_cloud = build_heatmap_cloud_banded(hit_cloud, distances, float(spec["low_m"]), float(spec["high_m"]))
    else:
        heatmap_cloud = build_heatmap_cloud(hit_cloud, distances, spec.get("max_distance_m"))
    heatmap_path = params.output_dir / "thickness_raycast_heatmap.ply"
    o3d.io.write_point_cloud(str(heatmap_path), heatmap_cloud)
    log(f"Heatmap guardado en: {heatmap_path}")

    return RaycastPipelineResult(
        stats=stats,
        distances=distances,
        hit_points=hit_points,
        unmatched_points=unmatched_points,
        base_mesh=base_mesh,
        updated_mesh=updated_mesh,
        heatmap_cloud=heatmap_cloud,
        csv_path=csv_path,
        histogram_path=histogram_path,
        heatmap_path=heatmap_path,
    )


def save_distances_csv(path: Path, points: np.ndarray, distances: np.ndarray) -> None:
    header = "x,y,z,distance_m,distance_mm"
    data = np.column_stack([points, distances, distances * 1000])
    np.savetxt(path, data, delimiter=",", header=header, comments="", fmt="%.6f")


def save_histogram(path: Path, distances: np.ndarray, method_label: str = "Cloud-to-Cloud") -> None:
    # No usar pyplot/matplotlib.use("Agg") aca: eso cambia el backend
    # globalmente para todo el proceso (matplotlib es un singleton), y
    # rompe en silencio el picking interactivo sobre foto de referencia
    # (pick_landmark_points_from_photo, mas abajo) si se corre despues.
    # Figure + FigureCanvasAgg generan el PNG sin tocar el backend global.
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    fig = Figure(figsize=(8, 5))
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    ax.hist(distances * 100, bins=60, color="steelblue", edgecolor="black")
    ax.set_xlabel("Espesor de shotcrete (cm)")
    ax.set_ylabel("Cantidad de puntos")
    ax.set_title(f"Distribucion del espesor de shotcrete ({method_label})")
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def visualize(
    base: o3d.geometry.PointCloud,
    heatmap_cloud: o3d.geometry.PointCloud,
    show_overlay: bool,
) -> None:
    if show_overlay:
        base_copy = o3d.geometry.PointCloud(base)
        base_copy.paint_uniform_color([0.6, 0.6, 0.6])
        show_geometries(
            [base_copy, heatmap_cloud],
            window_name="Aurora - Superposicion (gris=base, heatmap=espesor)",
            style_cloud=base_copy,
        )
    else:
        show_point_cloud(heatmap_cloud, window_name="Aurora - Heatmap de espesor de shotcrete")


def pick_crop_bounds(
    cloud: o3d.geometry.PointCloud,
    margin: float = 0.08,
    exact_points: bool = False,
    window_name: str = "Aurora - Clic izquierdo para seleccionar, luego cerrar (Q)",
) -> tuple[Vec3, Vec3] | None:
    """
    Abre un visor 3D interactivo para elegir la region de interes (p. ej. la
    caja de prueba) sin escribir coordenadas a mano.

    Uso en el visor: clic izquierdo sobre los puntos y despues cerrar la
    ventana (tecla Q o el boton de cerrar). El margen opcional se agrega
    alrededor de los puntos elegidos.
    """
    picked_points = _pick_points_in_shared_viewer(
        cloud,
        window_name,
        "Shift + clic en 2 esquinas opuestas; Q para terminar",
        max_points=2 if exact_points else 0,
    )
    if len(picked_points) < 2 or (exact_points and len(picked_points) != 2):
        return None
    crop_min = tuple((picked_points.min(axis=0) - margin).tolist())
    crop_max = tuple((picked_points.max(axis=0) + margin).tolist())
    return crop_min, crop_max


def pick_crop_bounds_from_pair(
    before: o3d.geometry.PointCloud,
    after: o3d.geometry.PointCloud,
    color_spec: dict | None = None,
) -> tuple[Vec3, Vec3] | None:
    """Pick one wall point and return the estimated plane bounds around it."""
    seed = pick_crop_seed_from_pair(before, after, color_spec)
    if seed is None:
        return None
    _seed, bounds = estimate_wall_crop_bounds(before, after, seed)
    return bounds


def pick_crop_seed_from_pair(
    before: o3d.geometry.PointCloud,
    after: o3d.geometry.PointCloud,
    color_spec: dict | None = None,
    forward_axis: str = "z",
) -> np.ndarray | None:
    """Pick one DESPUES wall point; the same cloud drives automatic detection."""
    _before_view, after_view = prepare_thickness_overlay(before, after, color_spec)
    picked = _pick_points_in_shared_viewer(
        after_view,
        "Aurora - Shift + clic en un punto DESPUES; Q para confirmar",
        "Shift + clic en la pared DESPUES; otro Shift + clic reemplaza el punto. Q para confirmar.",
        max_points=1,
        context_clouds=[_before_view],
        require_shift=True,
        forward_axis=forward_axis,
    )
    return picked[0].copy() if len(picked) else None


def estimate_wall_crop_bounds(
    before: o3d.geometry.PointCloud,
    after: o3d.geometry.PointCloud,
    seed: np.ndarray | None = None,
    forward_axis: str = "z",
) -> tuple[np.ndarray, tuple[Vec3, Vec3]]:
    """Estimate an axis-aligned wall box from a clicked point or the center ray.

    The forward axis selects the center ray (Aurora +Z, Unitree +X).
    Plane fitting and cropping retain the original sensor coordinates.
    """
    after_points = np.asarray(after.points, dtype=np.float64)
    valid_after = np.isfinite(after_points).all(axis=1)
    after_points = after_points[valid_after]
    if len(after_points) < 3:
        raise ValueError("La nube DESPUES no tiene suficientes puntos válidos.")

    if forward_axis not in "xyz" or len(forward_axis) != 1:
        raise ValueError("Eje frontal invalido")
    axis = "xyz".index(forward_axis)
    automatic = seed is None
    if seed is None:
        forward = after_points[:, axis] > 0.0
        candidates = after_points[forward]
        if not len(candidates):
            raise ValueError("No hay puntos delante del sensor para detectar la pared.")
        radial = np.linalg.norm(np.delete(candidates, axis, axis=1), axis=1)
        angles = np.arctan2(radial, np.maximum(candidates[:, axis], 1e-9))
        central = angles <= np.deg2rad(8.0)
        if np.any(central):
            candidates = candidates[central]
        seed = candidates[int(np.argmin(candidates[:, axis]))].copy()
    else:
        seed = np.asarray(seed, dtype=np.float64).reshape(3)
    # Both modes use DESPUES in its native axes. Automatic selection searches
    # frontal surfaces; manual selection anchors the surface at the click.
    points = after_points

    radius = max(0.75, min(3.0, abs(float(seed[axis])) * 1.25))
    if automatic:
        # A close floor return must not limit the depth searched for the wall.
        radial = np.linalg.norm(np.delete(points, axis, axis=1), axis=1)
        local = points[(points[:, axis] > 0) &
                       (radial <= np.maximum(.25, points[:, axis] * np.tan(np.deg2rad(20))))]
    else:
        local = points[np.linalg.norm(points - seed, axis=1) <= radius]
    if len(local) < 30:
        nearest = np.argsort(np.linalg.norm(points - seed, axis=1))[: min(len(points), 1000)]
        local = points[nearest]
    if len(local) < 3:
        raise ValueError("No hay suficientes puntos cerca del punto elegido para ajustar la pared.")

    # A large nearby floor often wins the first RANSAC fit. Search distinct
    # surfaces instead of letting its normal silently turn an X box into Z.
    remaining = local
    wall_points = np.empty((0, 3), dtype=np.float64)
    best_score = float("inf")
    depth_axis = axis
    for _ in range(8):
        if len(remaining) < 10:
            break
        local_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(remaining))
        plane, inliers = local_cloud.segment_plane(
            distance_threshold=0.02, ransac_n=3, num_iterations=600
        )
        if len(inliers) < 10:
            break
        normal = np.asarray(plane[:3], dtype=np.float64)
        normal /= max(float(np.linalg.norm(normal)), 1e-12)
        # Restore support at intersections removed with an earlier plane.
        # Otherwise peeling the floor can erase the only center-ray samples
        # of a low box and make a farther wall appear to be the first target.
        face = local[np.abs(local @ normal + plane[3]) <= .02]
        candidate_axis = int(np.argmax(np.abs(normal)))
        distance_to_seed = abs(float(np.dot(normal, seed) + plane[3]))
        if automatic:
            # Automatic means frontal wall, in the native sensor axes. A floor
            # or ceiling belongs to manual selection, even if it has more points.
            radial = np.linalg.norm(np.delete(face, axis, axis=1), axis=1)
            central = (face[:, axis] > 0) & (radial <= np.maximum(face[:, axis], 0) * np.tan(np.deg2rad(8)))
            if candidate_axis == axis and np.count_nonzero(central) >= 3:
                score = float(np.median(face[central, axis]))
                if score < best_score:
                    best_score, wall_points, depth_axis = score, face, candidate_axis
        elif distance_to_seed <= 0.025 and distance_to_seed < best_score:
            # A clicked point anchors its surface, not another plane up to 8 cm
            # away. Manual ceiling/side selections keep their actual normal.
            best_score, wall_points, depth_axis = distance_to_seed, face, candidate_axis
        keep = np.ones(len(remaining), dtype=bool)
        keep[inliers] = False
        remaining = remaining[keep]

    if automatic and len(wall_points) < 10:
        raise ValueError("No pude identificar una pared frontal. Usa «Elegir 1 punto» sobre la superficie.")
    if automatic:
        # Center on an actual sample of the selected face, not on the floor
        # return which initially bounded the search neighborhood.
        center = np.median(wall_points, axis=0)
        seed = wall_points[np.argmin(np.linalg.norm(wall_points - center, axis=1))].copy()

    if len(wall_points) < 10:
        # Robust fallback for rough or incomplete surfaces: use the local
        # neighborhood around the seed and use its dominant view axis as depth.
        wall_points = local[np.linalg.norm(local - seed, axis=1) <= min(radius, 0.65)]
        depth_axis = int(np.argmax(np.abs(seed)))
    if len(wall_points) < 3:
        raise ValueError("No pude estimar una superficie de pared alrededor del punto.")

    low = np.min(wall_points, axis=0)
    high = np.max(wall_points, axis=0)
    low[depth_axis] = seed[depth_axis]
    high[depth_axis] = seed[depth_axis]
    for axis in range(3):
        if axis == depth_axis:
            continue
        if high[axis] - low[axis] < 0.20:
            low[axis] = seed[axis] - 0.10
            high[axis] = seed[axis] + 0.10
    lateral_axes = [axis for axis in range(3) if axis != depth_axis]
    side_length = max(float(high[axis] - low[axis]) for axis in lateral_axes)
    for axis in lateral_axes:
        face_center = 0.5 * (low[axis] + high[axis])
        low[axis] = face_center - side_length / 2.0
        high[axis] = face_center + side_length / 2.0
    return seed, (tuple(low.tolist()), tuple(high.tolist()))


def run_pipeline(params: PipelineParams, log=print) -> PipelineResult:
    if params.distance_method not in {"cloud_to_cloud", "plane", "auto"}:
        raise ValueError("Método de distancia desconocido.")
    if params.distance_method in {"plane", "auto"} and params.use_icp:
        raise ValueError("Desactiva ICP: alinear la superficie modificada podría borrar el espesor que quieres medir.")
    params.output_dir.mkdir(parents=True, exist_ok=True)

    log("Cargando nubes de puntos...")
    base_cloud = load_point_cloud(params.base_path)
    updated_cloud = load_point_cloud(params.updated_path)
    if params.align_by_pose:
        base_pose, updated_pose = load_reference_pose(params.base_path), load_reference_pose(params.updated_path)
        if base_pose is None or updated_pose is None:
            raise ValueError("Faltan poses para compensar el movimiento del sensor. Alinea las nubes o captura ambas con seguimiento.")
        log("Compensando IMU/SLAM de DESPUÉS en el marco de ANTES...")
        updated_cloud = align_cloud_by_pose(updated_cloud, base_pose, updated_pose)
    log(f"  Base       : {len(base_cloud.points)} puntos")
    log(f"  Actualizada: {len(updated_cloud.points)} puntos")

    log("Preparando nubes (recorte, reduccion y filtrado)...")
    log("Preparando nube ANTES...")
    base_cloud = preprocess(
        base_cloud, params.voxel_size, params.remove_outliers, params.crop_min, params.crop_max,
        cloud_label="la nube base",
    )
    log("Nube ANTES lista.")
    log("Preparando nube DESPUES...")
    updated_cloud = preprocess(
        updated_cloud, params.voxel_size, params.remove_outliers, params.crop_min, params.crop_max,
        cloud_label="la nube actualizada",
    )
    log("Nubes listas para medir.")
    if params.crop_min is not None:
        log(f"  Tras recorte -> base: {len(base_cloud.points)} puntos, actualizada: {len(updated_cloud.points)} puntos")

    if params.use_icp:
        log("Alineando nubes con ICP...")
        updated_cloud = align_clouds(updated_cloud, base_cloud, params.icp_threshold)

    measurement_details = comparison = comparison_cloud = accepted_mask = None
    if params.distance_method == "auto":
        from surface_comparison import compare_surfaces, SurfaceParams
        comparison_cloud = o3d.geometry.PointCloud(updated_cloud)
        comparison = compare_surfaces(np.asarray(base_cloud.points), np.asarray(updated_cloud.points),
                                      SurfaceParams(**(params.surface_params or {})), log=log)
        raw_distances = np.abs(comparison.signed)
        eligible = comparison.valid & (raw_distances > comparison.variation)
        measurement_details = dict(
            params=comparison.params, regions=comparison.regions,
            total_after_points=len(raw_distances), valid_points=int(comparison.valid.sum()),
            unmatched_after_points=int((~comparison.valid).sum()),
            unmatched_before_points=len(comparison.unmatched_before),
            within_variation_points=int((comparison.valid & ~eligible).sum()),
            plane_points=int(np.count_nonzero(comparison.method == 1)),
            local_normal_points=int(np.count_nonzero(comparison.method == 2)),
            high_quality_points=int(np.count_nonzero(comparison.quality == 2)),
            medium_quality_points=int(np.count_nonzero(comparison.quality == 1)),
            coverage_percent=float(100 * comparison.valid.mean()),
            codes=dict(method={0: "none", 1: "plane", 2: "local_normal"},
                       quality={0: "none", 1: "medium", 2: "high"},
                       reason={0: "valid", 1: "no_support", 2: "ambiguous", 3: "no_overlap", 4: "edge_or_noise"}),
            quality_note="Indicadores geométricos y de variación; no son probabilidades ni precisión absoluta certificada.",
        )
        log(f"Cobertura DESPUÉS: {measurement_details['valid_points']:,}/{len(raw_distances):,} puntos "
            f"({measurement_details['coverage_percent']:.1f}%); sin correspondencia: "
            f"{measurement_details['unmatched_after_points']:,}.")
    elif params.distance_method == "plane":
        from plane_thickness import planar_thickness
        log("Midiendo perpendicularmente a la cara plana de ANTES...")
        indices, raw_distances, measurement_details = planar_thickness(
            np.asarray(base_cloud.points), np.asarray(updated_cloud.points)
        )
        updated_cloud = updated_cloud.select_by_index(indices)
        log(f"Caras: {measurement_details['base_face_points']:,} puntos ANTES y "
            f"{measurement_details['after_face_points']:,} DESPUÉS; "
            f"{measurement_details['overlap_points']:,} en zona común; "
            f"ángulo {measurement_details['normal_angle_deg']:.1f}°.")
        log(f"Fuera de cara/zona común: {measurement_details['rejected_geometry_points']:,} puntos. "
            "Tolerancia del ajuste: 1.5 cm; solo se mide donde hay soporte en ANTES.")
    else:
        log("Calculando distancia Cloud-to-Cloud...")
        raw_distances = compute_c2c_distance(updated_cloud, base_cloud)
    if comparison is None:
        eligible = np.ones(len(raw_distances), dtype=bool)
    input_n_points = int(eligible.sum())
    if not np.isfinite(params.minimum_thickness_m) or params.minimum_thickness_m < 0:
        raise ValueError("El umbral mínimo de espesor debe ser cero o mayor.")
    minimum_mask = (
        raw_distances >= params.minimum_thickness_m
        if params.minimum_thickness_m > 0
        else np.ones(len(raw_distances), dtype=bool)
    )
    keep_mask = eligible & minimum_mask
    excluded_count = int(np.count_nonzero(eligible & ~minimum_mask))
    distances = raw_distances[keep_mask]
    updated_cloud = updated_cloud.select_by_index(np.flatnonzero(keep_mask))
    stats = summarize_filtered(
        distances,
        input_n_points=input_n_points,
        excluded_n_points=excluded_count,
        minimum_thickness_m=params.minimum_thickness_m,
    ) if len(distances) or comparison is None else DistanceStats(
        mean=0., median=0., std=0., min=0., max=0., p95=0., n_points=0,
        input_n_points=input_n_points, excluded_n_points=excluded_count,
        minimum_thickness_m=params.minimum_thickness_m,
    )
    if comparison is not None:
        accepted_mask = keep_mask
        measurement_details['used_points'] = int(keep_mask.sum())
        measurement_details['below_minimum_points'] = excluded_count
    log("")
    log("Resultados:")
    log(str(stats))

    points = np.asarray(updated_cloud.points)
    csv_path = params.output_dir / "thickness_per_point.csv"
    save_distances_csv(csv_path, points, distances)
    log(f"CSV guardado en: {csv_path}")

    histogram_path = params.output_dir / "thickness_histogram.png"
    save_histogram(histogram_path, distances,
                   {"plane": "Cara plana / perpendicular", "auto": "Automático por superficies"}.get(params.distance_method, "Cloud-to-Cloud"))
    log(f"Histograma guardado en: {histogram_path}")

    heatmap_cloud = (build_surface_view(comparison_cloud, comparison, accepted_mask, distances,
                                       color_spec={"max_distance_m": params.max_distance})
                     if comparison is not None else build_heatmap_cloud(updated_cloud, distances, params.max_distance))
    heatmap_path = params.output_dir / "thickness_heatmap.ply"
    o3d.io.write_point_cloud(str(heatmap_path), heatmap_cloud)
    log(f"Heatmap guardado en: {heatmap_path}")

    import json
    (params.output_dir / "measurement_details.json").write_text(json.dumps({
        "distance_method": params.distance_method,
        "base_path": str(params.base_path), "updated_path": str(params.updated_path),
        "minimum_thickness_m": params.minimum_thickness_m,
        "geometry": measurement_details,
        "pose_alignment_requested": params.align_by_pose,
        "median_m": stats.median, "mean_m": stats.mean, "n_points": stats.n_points,
    }, indent=2), encoding="utf-8")
    if comparison is not None:
        save_surface_diagnostics(params.output_dir, np.asarray(comparison_cloud.points), comparison, accepted_mask)

    return PipelineResult(
        stats=stats,
        distances=distances,
        base_cloud=base_cloud,
        updated_cloud=updated_cloud,
        heatmap_cloud=heatmap_cloud,
        csv_path=csv_path,
        histogram_path=histogram_path,
        heatmap_path=heatmap_path,
        distance_method=params.distance_method,
        measurement_details=measurement_details,
        comparison=comparison, comparison_cloud=comparison_cloud, accepted_mask=accepted_mask,
    )


def build_surface_view(cloud, comparison, accepted_mask, distances, view='thickness', color_spec=None):
    """Include unmeasurable regions in gray, without assigning them a thickness."""
    result = o3d.geometry.PointCloud(cloud)
    colors = np.tile([.58, .60, .63], (len(result.points), 1))
    if view == 'quality':
        colors[comparison.quality == 1] = [.95, .65, .1]
        colors[comparison.quality == 2] = [.2, .85, .4]
    elif view == 'method':
        colors[comparison.method == 1] = [.1, .75, .95]
        colors[comparison.method == 2] = [.75, .35, .95]
    else:
        colors[comparison.valid] = [.35, .6, .8]
        if np.any(accepted_mask):
            selected = result.select_by_index(np.flatnonzero(accepted_mask))
            spec = color_spec or {}
            if spec.get('mode') == 'six':
                colored = build_heatmap_cloud_six_bands(selected, distances, spec['target_m'])
            elif spec.get('mode') == 'three':
                colored = build_heatmap_cloud_banded(selected, distances, spec['low_m'], spec['high_m'])
            else:
                colored = build_heatmap_cloud(selected, distances, spec.get('max_distance_m'))
            colors[accepted_mask] = np.asarray(colored.colors)
    result.colors = o3d.utility.Vector3dVector(colors)
    if len(comparison.unmatched_before):
        missing = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(comparison.unmatched_before))
        missing.paint_uniform_color([.58, .60, .63])
        result += missing
    return result


def save_surface_diagnostics(output_dir, points, comparison, accepted):
    """Export every post-capture sample and missing reference support separately."""
    import csv
    np.savez_compressed(output_dir / 'surface_comparison.npz', points=points,
                        signed_m=comparison.signed, valid=comparison.valid,
                        method=comparison.method, quality=comparison.quality,
                        variation_m=comparison.variation, region=comparison.region,
                        reason=comparison.reason, accepted=accepted,
                        unmatched_before=comparison.unmatched_before)
    methods = ['none', 'plane', 'local_normal']
    quality = ['none', 'medium', 'high']
    reasons = ['valid', 'no_support', 'ambiguous', 'no_overlap', 'edge_or_noise']
    with (output_dir / 'surface_comparison_per_point.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source', 'x', 'y', 'z', 'signed_change_m', 'variation_m',
                         'method', 'quality', 'status', 'region', 'used_in_stats'])
        for i, point in enumerate(points):
            reason = reasons[comparison.reason[i]]
            if comparison.valid[i] and not accepted[i]:
                reason = ('within_variation' if abs(comparison.signed[i]) <= comparison.variation[i]
                          else 'below_minimum')
            writer.writerow(['after', *point, comparison.signed[i], comparison.variation[i],
                             methods[comparison.method[i]], quality[comparison.quality[i]],
                             reason, comparison.region[i], int(accepted[i])])
        for point in comparison.unmatched_before:
            writer.writerow(['before', *point, 'nan', 'nan', 'none', 'none', 'no_after_support', -1, 0])
