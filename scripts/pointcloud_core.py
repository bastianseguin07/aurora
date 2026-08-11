"""
Logica central para comparar dos nubes de puntos .ply y estimar el espesor
de shotcrete (distancia Cloud-to-Cloud). La usan tanto la CLI
(compare_point_clouds.py) como la GUI (gui.py).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
import open3d as o3d

Vec3 = Tuple[float, float, float]


@dataclass
class DistanceStats:
    mean: float
    median: float
    std: float
    min: float
    max: float
    p95: float
    n_points: int

    def __str__(self) -> str:
        return (
            f"  Puntos analizados : {self.n_points}\n"
            f"  Espesor medio     : {self.mean * 1000:.2f} mm\n"
            f"  Espesor mediano   : {self.median * 1000:.2f} mm\n"
            f"  Desv. estandar    : {self.std * 1000:.2f} mm\n"
            f"  Minimo            : {self.min * 1000:.2f} mm\n"
            f"  Maximo            : {self.max * 1000:.2f} mm\n"
            f"  Percentil 95      : {self.p95 * 1000:.2f} mm"
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
    if base_points.shape != moving_points.shape or base_points.shape[0] < 3:
        raise ValueError("Se necesitan al menos 3 puntos correspondientes, en igual cantidad en ambas nubes.")

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
    opt.point_size = 3.0
    opt.point_color_option = o3d.visualization.PointColorOption.Color
    opt.light_on = False


def show_point_cloud(cloud: o3d.geometry.PointCloud, window_name: str = "Aurora - Vista de la captura") -> None:
    """Abre una ventana 3D simple (no editable) mostrando la nube tal cual,
    con el mismo estilo del demo oficial del SDK, para verificar visualmente
    que la captura salio bien. Arranca posicionada en el punto de vista del
    sensor (los puntos ya vienen en su marco local, origen en el sensor,
    con X=derecha, Y=abajo, Z=adelante), en vez de encuadrar toda la nube."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name)
    vis.add_geometry(cloud)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(coord_frame)
    _apply_sdk_render_style(vis, cloud)
    try:
        ctr = vis.get_view_control()
        params = ctr.convert_to_pinhole_camera_parameters()
        params.extrinsic = np.eye(4)  # camara en el origen del sensor, mirando hacia +Z
        ctr.convert_from_pinhole_camera_parameters(params, allow_arbitrary=True)
    except Exception:
        pass  # si el visor no soporta esto, se queda con el encuadre por defecto
    vis.run()
    vis.destroy_window()


def transform_points_inverse(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """
    Aplica la transformacion inversa de 'compute_rigid_transform' a un array
    Nx3: lleva puntos ya expresados en el sistema de referencia 'base' de
    vuelta al sistema de coordenadas original de la nube 'moving' (previo a
    alinearla). Util para ubicar una region elegida en la nube alineada
    dentro de la nube cruda sin tener que transformar la nube completa.
    """
    points = np.asarray(points, dtype=np.float64)
    return (points - translation) @ rotation


def pick_landmark_points(cloud: o3d.geometry.PointCloud, window_name: str) -> np.ndarray | None:
    """
    Abre un visor 3D interactivo para elegir puntos de referencia (landmarks)
    en orden: Shift + click izquierdo sobre cada punto, en el mismo orden en
    que se van a elegir en la otra nube, despues cerrar la ventana (Q).
    Devuelve las coordenadas en el orden elegido, o None si se eligieron
    menos de 3 puntos.
    """
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=window_name)
    vis.add_geometry(cloud)
    _apply_sdk_render_style(vis, cloud)
    vis.run()
    vis.destroy_window()

    picked_indices = vis.get_picked_points()
    if len(picked_indices) < 3:
        return None

    points = np.asarray(cloud.points)
    return points[picked_indices]


def pick_quad_points(cloud: o3d.geometry.PointCloud, window_name: str) -> np.ndarray | None:
    """
    Abre un visor 3D interactivo para elegir los 4 puntos que definen una
    region cuadrada/rectangular sobre la superficie: Shift + click izquierdo
    en las 4 esquinas, en orden alrededor del perimetro, despues cerrar la
    ventana (Q). Devuelve None si no se eligieron exactamente 4 puntos.
    """
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=window_name)
    vis.add_geometry(cloud)
    _apply_sdk_render_style(vis, cloud)
    vis.run()
    vis.destroy_window()

    picked_indices = vis.get_picked_points()
    if len(picked_indices) != 4:
        return None

    points = np.asarray(cloud.points)
    return points[picked_indices]


def _points_in_polygon_2d(points_2d: np.ndarray, polygon_2d: np.ndarray) -> np.ndarray:
    """Ray casting vectorizado: True para los puntos que caen dentro del poligono."""
    x, y = points_2d[:, 0], points_2d[:, 1]
    n = polygon_2d.shape[0]
    inside = np.zeros(points_2d.shape[0], dtype=bool)
    j = n - 1
    for i in range(n):
        xi, yi = polygon_2d[i]
        xj, yj = polygon_2d[j]
        crosses = (yi > y) != (yj > y)
        x_intersect = (xj - xi) * (y - yi) / (yj - yi + 1e-15) + xi
        inside ^= crosses & (x < x_intersect)
        j = i
    return inside


def crop_cloud_by_quad_box(
    cloud: o3d.geometry.PointCloud,
    quad_points: np.ndarray,
    depth_m: float,
) -> o3d.geometry.PointCloud:
    """
    Recorta 'cloud' a un box 3D: la region delimitada por 'quad_points' (4
    esquinas, en orden alrededor del perimetro) extruida +/- depth_m/2 a lo
    largo de la normal del plano que mejor ajusta esos 4 puntos (ajuste por
    SVD, tolera que no sean perfectamente coplanares).
    """
    quad_points = np.asarray(quad_points, dtype=np.float64)
    if quad_points.shape[0] != 4:
        raise ValueError("Se necesitan exactamente 4 puntos para definir el box.")

    centroid = quad_points.mean(axis=0)
    _, _, vt = np.linalg.svd(quad_points - centroid)
    u_axis, v_axis, normal = vt[0], vt[1], vt[2]

    def to_local(points_3d: np.ndarray) -> np.ndarray:
        rel = points_3d - centroid
        return np.column_stack([rel @ u_axis, rel @ v_axis, rel @ normal])

    quad_local_2d = to_local(quad_points)[:, :2]

    points = np.asarray(cloud.points)
    local = to_local(points)

    inside_polygon = _points_in_polygon_2d(local[:, :2], quad_local_2d)
    inside_depth = np.abs(local[:, 2]) <= (depth_m / 2.0)
    mask = inside_polygon & inside_depth

    cropped = o3d.geometry.PointCloud()
    cropped.points = o3d.utility.Vector3dVector(points[mask])
    if cloud.has_colors():
        colors = np.asarray(cloud.colors)
        cropped.colors = o3d.utility.Vector3dVector(colors[mask])
    return cropped


def compute_c2c_distance(
    updated: o3d.geometry.PointCloud, base: o3d.geometry.PointCloud
) -> np.ndarray:
    distances = updated.compute_point_cloud_distance(base)
    return np.asarray(distances)


def summarize(distances: np.ndarray) -> DistanceStats:
    return DistanceStats(
        mean=float(np.mean(distances)),
        median=float(np.median(distances)),
        std=float(np.std(distances)),
        min=float(np.min(distances)),
        max=float(np.max(distances)),
        p95=float(np.percentile(distances, 95)),
        n_points=int(distances.size),
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


def save_distances_csv(path: Path, points: np.ndarray, distances: np.ndarray) -> None:
    header = "x,y,z,distance_m,distance_mm"
    data = np.column_stack([points, distances, distances * 1000])
    np.savetxt(path, data, delimiter=",", header=header, comments="", fmt="%.6f")


def save_histogram(path: Path, distances: np.ndarray) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.hist(distances * 1000, bins=60, color="steelblue", edgecolor="black")
    ax.set_xlabel("Espesor de shotcrete (mm)")
    ax.set_ylabel("Cantidad de puntos")
    ax.set_title("Distribucion del espesor de shotcrete (Cloud-to-Cloud)")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def visualize(
    base: o3d.geometry.PointCloud,
    heatmap_cloud: o3d.geometry.PointCloud,
    show_overlay: bool,
) -> None:
    if show_overlay:
        base_copy = o3d.geometry.PointCloud(base)
        base_copy.paint_uniform_color([0.6, 0.6, 0.6])
        o3d.visualization.draw_geometries(
            [base_copy, heatmap_cloud],
            window_name="Aurora - Superposicion (gris=base, heatmap=espesor)",
        )
    else:
        o3d.visualization.draw_geometries(
            [heatmap_cloud],
            window_name="Aurora - Heatmap de espesor de shotcrete",
        )


def pick_crop_bounds(cloud: o3d.geometry.PointCloud, margin: float = 0.08) -> tuple[Vec3, Vec3] | None:
    """
    Abre un visor 3D interactivo para elegir la region de interes (p. ej. la
    caja de prueba) sin escribir coordenadas a mano.

    Uso en el visor: Shift + click izquierdo sobre 2 o mas puntos que
    representen esquinas de la region deseada, despues cerrar la ventana
    (tecla Q o el boton de cerrar). El margen se agrega alrededor de los
    puntos elegidos para no cortar el borde real del objeto Y para que la
    region siga incluyendo al objeto aunque este se haya desplazado entre
    una nube y la otra (el margen debe ser mayor que el desplazamiento
    esperado, p. ej. si el objeto se movio 5 cm, usar un margen >= 8-10 cm).
    """
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="Aurora - Shift+Click en 2 esquinas, luego cerrar (Q)")
    vis.add_geometry(cloud)
    _apply_sdk_render_style(vis, cloud)
    vis.run()
    vis.destroy_window()

    picked_indices = vis.get_picked_points()
    if len(picked_indices) < 2:
        return None

    points = np.asarray(cloud.points)
    picked_points = points[picked_indices]
    crop_min = tuple((picked_points.min(axis=0) - margin).tolist())
    crop_max = tuple((picked_points.max(axis=0) + margin).tolist())
    return crop_min, crop_max


def run_pipeline(params: PipelineParams, log=print) -> PipelineResult:
    params.output_dir.mkdir(parents=True, exist_ok=True)

    log("Cargando nubes de puntos...")
    base_cloud = load_point_cloud(params.base_path)
    updated_cloud = load_point_cloud(params.updated_path)
    log(f"  Base       : {len(base_cloud.points)} puntos")
    log(f"  Actualizada: {len(updated_cloud.points)} puntos")

    base_cloud = preprocess(
        base_cloud, params.voxel_size, params.remove_outliers, params.crop_min, params.crop_max,
        cloud_label="la nube base",
    )
    updated_cloud = preprocess(
        updated_cloud, params.voxel_size, params.remove_outliers, params.crop_min, params.crop_max,
        cloud_label="la nube actualizada",
    )
    if params.crop_min is not None:
        log(f"  Tras recorte -> base: {len(base_cloud.points)} puntos, actualizada: {len(updated_cloud.points)} puntos")

    if params.use_icp:
        log("Alineando nubes con ICP...")
        updated_cloud = align_clouds(updated_cloud, base_cloud, params.icp_threshold)

    log("Calculando distancia Cloud-to-Cloud...")
    distances = compute_c2c_distance(updated_cloud, base_cloud)
    stats = summarize(distances)
    log("")
    log("Resultados:")
    log(str(stats))

    points = np.asarray(updated_cloud.points)
    csv_path = params.output_dir / "thickness_per_point.csv"
    save_distances_csv(csv_path, points, distances)
    log(f"CSV guardado en: {csv_path}")

    histogram_path = params.output_dir / "thickness_histogram.png"
    save_histogram(histogram_path, distances)
    log(f"Histograma guardado en: {histogram_path}")

    heatmap_cloud = build_heatmap_cloud(updated_cloud, distances, params.max_distance)
    heatmap_path = params.output_dir / "thickness_heatmap.ply"
    o3d.io.write_point_cloud(str(heatmap_path), heatmap_cloud)
    log(f"Heatmap guardado en: {heatmap_path}")

    return PipelineResult(
        stats=stats,
        distances=distances,
        base_cloud=base_cloud,
        updated_cloud=updated_cloud,
        heatmap_cloud=heatmap_cloud,
        csv_path=csv_path,
        histogram_path=histogram_path,
        heatmap_path=heatmap_path,
    )
