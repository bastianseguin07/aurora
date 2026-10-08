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
            f"  Espesor medio     : {self.mean * 100:.2f} cm\n"
            f"  Espesor mediano   : {self.median * 100:.2f} cm\n"
            f"  Desv. estandar    : {self.std * 100:.2f} cm\n"
            f"  Minimo            : {self.min * 100:.2f} cm\n"
            f"  Maximo            : {self.max * 100:.2f} cm\n"
            f"  Percentil 95      : {self.p95 * 100:.2f} cm"
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


def show_point_cloud(cloud: o3d.geometry.PointCloud, window_name: str = "Aurora - Vista de la captura") -> None:
    """Abre una ventana 3D simple (no editable) mostrando la nube tal cual,
    con el mismo estilo del demo oficial del SDK, para verificar visualmente
    que la captura salio bien. Arranca en el punto de vista del sensor."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name)
    vis.add_geometry(cloud)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(coord_frame)
    _apply_sdk_render_style(vis, cloud)
    _start_at_sensor_pov(vis)
    vis.run()
    vis.destroy_window()


def pick_landmark_points(cloud: o3d.geometry.PointCloud, window_name: str) -> np.ndarray | None:
    """
    Abre un visor 3D interactivo para elegir puntos de referencia (landmarks)
    en orden: Shift + click izquierdo sobre cada punto, en el mismo orden en
    que se van a elegir en la otra nube, despues cerrar la ventana (Q).
    Arranca en el punto de vista del sensor. Devuelve las coordenadas en el
    orden elegido, o None si se eligieron menos de 3 puntos.
    """
    vis = o3d.visualization.VisualizerWithEditing()
    created = vis.create_window(window_name=window_name)
    if not created or vis.get_render_option() is None:
        # Open3D's legacy GLFW viewer can fail to create an OpenGL context in
        # WSLg (for example, GLEW initialization fails). Do not continue into
        # _apply_sdk_render_style with a None render option: retain an
        # interactive landmark workflow using Matplotlib instead.
        try:
            if created:
                vis.destroy_window()
        except Exception:
            pass
        return _pick_landmark_points_matplotlib(cloud, window_name)

    try:
        vis.add_geometry(cloud)
        _apply_sdk_render_style(vis, cloud)
        _start_at_sensor_pov(vis)
        vis.run()
        picked_indices = vis.get_picked_points()
    finally:
        vis.destroy_window()

    if len(picked_indices) < 3:
        return None

    points = np.asarray(cloud.points)
    return points[picked_indices]


def _pick_landmark_points_matplotlib(
    cloud: o3d.geometry.PointCloud, window_name: str, display_limit: int = 120_000
) -> np.ndarray | None:
    """Interactive 3D fallback for systems where Open3D cannot create a GL window.

    The rendered cloud is sampled for responsiveness, while clicks are mapped
    against every original point so the returned landmarks retain full cloud
    precision. Rotate the view with the mouse, click three or more landmarks
    in order, then close the window.
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import proj3d

    points = np.asarray(cloud.points)
    if len(points) < 3:
        return None

    if len(points) > display_limit:
        sample_idx = np.linspace(0, len(points) - 1, display_limit, dtype=np.int64)
    else:
        sample_idx = np.arange(len(points))

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    if cloud.has_colors():
        colors = np.asarray(cloud.colors)[sample_idx]
    else:
        colors = "#aeb8c4"
    ax.scatter(*points[sample_idx].T, c=colors, s=1, depthshade=False, linewidths=0)
    ax.set_title(f"{window_name}\nClick en 3+ referencias en orden; arrastra para orbitar y cierra al terminar")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    selected: list[int] = []
    markers = []

    def on_click(event) -> None:
        if event.inaxes is not ax or event.button != 1 or event.x is None or event.y is None:
            return
        projected_x, projected_y, projected_z = proj3d.proj_transform(
            points[:, 0], points[:, 1], points[:, 2], ax.get_proj()
        )
        screen = ax.transData.transform(np.column_stack((projected_x, projected_y)))
        delta = screen - np.array([event.x, event.y])
        distance_sq = np.einsum("ij,ij->i", delta, delta)
        # In dense/overlapping projections, prefer the front-most close point.
        nearest = np.flatnonzero(distance_sq <= distance_sq.min() + 4.0)
        index = int(nearest[np.argmax(projected_z[nearest])])
        selected.append(index)
        x, y, z = points[index]
        marker = ax.scatter([x], [y], [z], c="#55d6be", s=55, edgecolors="black", depthshade=False)
        markers.append(marker)
        ax.text(x, y, z, str(len(selected)), color="#55d6be", fontsize=10)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.show()
    if len(selected) < 3:
        return None
    return points[np.asarray(selected, dtype=np.int64)]


def reference_photo_path_for(ply_path: Path) -> Path:
    """Convencion de nombre del sidecar con la foto de referencia (ver
    aurora_sensor.capture_reference_frame) asociada a una captura .ply."""
    return ply_path.with_name(ply_path.stem + "_ref.npz")


def save_reference_photo(ply_path: Path, image: np.ndarray, point_grid: np.ndarray) -> None:
    """Guarda la foto de referencia y su grid de puntos 3D junto a un .ply,
    para poder elegir puntos de alineacion sobre la foto mas adelante."""
    np.savez_compressed(reference_photo_path_for(ply_path), image=image, point_grid=point_grid)


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


def save_reference_pose(ply_path: Path, position: np.ndarray, rpy_deg: np.ndarray) -> None:
    """Guarda la pose del sensor (posicion en metros, orientacion roll/pitch/yaw
    en grados) junto a un .ply, para poder guiar al usuario a devolver el
    sensor a esa misma posicion fisica antes de la siguiente captura."""
    np.savez_compressed(
        reference_pose_path_for(ply_path),
        position=np.asarray(position, dtype=np.float64),
        rpy_deg=np.asarray(rpy_deg, dtype=np.float64),
    )


def load_reference_pose(ply_path: Path) -> tuple[np.ndarray, np.ndarray] | None:
    """Carga la pose de referencia de un .ply si existe, o None si esa
    captura se hizo sin sensor conectado o antes de este cambio."""
    path = reference_pose_path_for(ply_path)
    if not path.exists():
        return None
    data = np.load(path)
    return data["position"], data["rpy_deg"]


def local_axes_from_rpy_deg(rpy_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Ejes propios del sensor (adelante/derecha/arriba) expresados en el
    sistema de coordenadas mundo/mapa, a partir de su orientacion
    roll/pitch/yaw (grados) en un instante dado. Asume que en el marco
    propio del sensor 'adelante' (la cara con las camaras) es +Y, 'derecha'
    es +X y 'arriba' es +Z — ajustado a mano contra el sensor real (la
    primera version tenia 'adelante' y 'arriba' cruzados: lo que se
    mostraba como 'arriba'/'abajo' era en realidad 'adelante'/'atras')."""
    rotation = o3d.geometry.get_rotation_matrix_from_xyz(np.radians(np.asarray(rpy_deg, dtype=np.float64)))
    forward = rotation @ np.array([0.0, 1.0, 0.0])
    right = rotation @ np.array([1.0, 0.0, 0.0])
    up = rotation @ np.array([0.0, 0.0, 1.0])
    return forward, right, up


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


def pick_quad_points(cloud: o3d.geometry.PointCloud, window_name: str) -> np.ndarray | None:
    """
    Abre un visor 3D interactivo para elegir los 4 puntos que definen una
    region cuadrada/rectangular sobre la superficie: Shift + click izquierdo
    en las 4 esquinas, en orden alrededor del perimetro, despues cerrar la
    ventana (Q). Arranca en el punto de vista del sensor. Devuelve None si
    no se eligieron exactamente 4 puntos.
    """
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=window_name)
    vis.add_geometry(cloud)
    _apply_sdk_render_style(vis, cloud)
    _start_at_sensor_pov(vis)
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


def _quad_box_axes(quad_points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Centroide y ejes (u, v, normal) del box que representa la region.

    En vez de usar el plano que mejor ajusta los 4 puntos clickeados (que
    queda inclinado si el click no fue perfectamente parejo en algun eje),
    el box se fuerza a quedar recto: se ajusta al eje global mas cercano a
    la normal ajustada por SVD (X, Y o Z), y los otros dos ejes del box
    quedan exactamente sobre los otros dos ejes globales. El resultado es
    siempre un paralelepipedo alineado a los ejes, nunca inclinado, sin
    importar la imprecision al elegir los puntos.
    """
    quad_points = np.asarray(quad_points, dtype=np.float64)
    if quad_points.shape[0] != 4:
        raise ValueError("Se necesitan exactamente 4 puntos para definir el box.")
    centroid = quad_points.mean(axis=0)
    _, _, vt = np.linalg.svd(quad_points - centroid)
    raw_normal = vt[2]

    axis_idx = int(np.argmax(np.abs(raw_normal)))
    sign = 1.0 if raw_normal[axis_idx] >= 0 else -1.0
    normal = np.zeros(3)
    normal[axis_idx] = sign

    x_axis, y_axis, z_axis = np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    if axis_idx == 2:  # normal ~ Z (el sensor mirando de frente, caso tipico)
        u_axis, v_axis = x_axis, y_axis
    elif axis_idx == 1:  # normal ~ Y (sensor mirando hacia arriba/abajo)
        u_axis, v_axis = x_axis, z_axis
    else:  # normal ~ X (sensor mirando de costado)
        u_axis, v_axis = z_axis, y_axis

    return centroid, u_axis, v_axis, normal


def crop_cloud_by_quad_box(
    cloud: o3d.geometry.PointCloud,
    quad_points: np.ndarray,
    depth_m: float,
) -> o3d.geometry.PointCloud:
    """
    Recorta 'cloud' a un box 3D: la region delimitada por 'quad_points' (4
    esquinas, en orden alrededor del perimetro) extruida +/- depth_m/2 a lo
    largo de la normal del plano que mejor ajusta esos 4 puntos.
    """
    quad_points = np.asarray(quad_points, dtype=np.float64)
    centroid, u_axis, v_axis, normal = _quad_box_axes(quad_points)

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


def build_quad_box_wireframe(
    quad_points: np.ndarray,
    depth_m: float,
    color: tuple[float, float, float] = (1.0, 0.549, 0.0),
) -> o3d.geometry.LineSet:
    """
    Arma el contorno (wireframe) del box 3D que usaria 'crop_cloud_by_quad_box'
    con el mismo 'quad_points'/'depth_m', para previsualizar la region antes
    de aplicar el recorte.
    """
    quad_points = np.asarray(quad_points, dtype=np.float64)
    centroid, u_axis, v_axis, normal = _quad_box_axes(quad_points)
    rel = quad_points - centroid
    quad_local_2d = np.column_stack([rel @ u_axis, rel @ v_axis])
    half_depth = depth_m / 2.0

    corners = []
    for sign in (-half_depth, half_depth):
        for lu, lv in quad_local_2d:
            corners.append(centroid + lu * u_axis + lv * v_axis + sign * normal)
    corners = np.array(corners)

    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # cara del plano elegido
        (4, 5), (5, 6), (6, 7), (7, 4),  # cara opuesta (a +depth_m)
        (0, 4), (1, 5), (2, 6), (3, 7),  # aristas verticales
    ]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(np.array(edges))
    line_set.colors = o3d.utility.Vector3dVector(np.tile(np.array(color), (len(edges), 1)))
    return line_set


def show_quad_box_preview(
    cloud: o3d.geometry.PointCloud,
    quad_points: np.ndarray,
    depth_m: float,
    window_name: str = "Aurora - Region seleccionada (previsualizacion)",
) -> None:
    """Muestra la nube completa junto con el contorno del box 3D que se va a
    recortar (naranja), para confirmar visualmente la region antes de
    aplicar la segmentacion."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name)
    vis.add_geometry(cloud)
    vis.add_geometry(build_quad_box_wireframe(quad_points, depth_m))
    _apply_sdk_render_style(vis, cloud)
    _start_at_sensor_pov(vis)
    vis.run()
    vis.destroy_window()


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


@dataclass
class RaycastPipelineParams:
    base_path: Path
    updated_path: Path
    output_dir: Path
    poisson_depth: int = 9
    density_trim_quantile: float = 0.02
    max_distance: float | None = None
    # Origen del sensor a usar si el .ply no tiene sidecar de pose (p. ej.
    # nubes cargadas a mano, sin sensor conectado, para probar el flujo).
    # Si es None se exige el sidecar (ver run_raycast_pipeline). Solo se usa
    # si ray_method == "origin".
    base_origin: Vec3 | None = None
    updated_origin: Vec3 | None = None
    # "poisson" (mesh_from_point_cloud, pensado para nubes completas) o
    # "delaunay" (mesh_from_point_cloud_delaunay, pensado para un recorte
    # casi-plano, ver pestaña 'Segmentacion').
    mesh_method: str = "poisson"
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


@dataclass
class RaycastPipelineResult:
    stats: DistanceStats
    distances: np.ndarray
    hit_points: np.ndarray
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
        cloud.orient_normals_consistent_tangent_plane(30)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cloud, depth=poisson_depth)
    if density_trim_quantile > 0:
        densities = np.asarray(densities)
        threshold = np.quantile(densities, density_trim_quantile)
        mesh.remove_vertices_by_mask(densities < threshold)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    return mesh


def fit_plane(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Centroide y ejes (u, v, normal) del plano que mejor ajusta 'points' via
    PCA/SVD. A diferencia de '_quad_box_axes' (que fuerza la normal al eje
    global mas cercano para armar un box de recorte recto), aca se usa la
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
    tunel ya recortado (pestaña 'Segmentacion'), no para una nube completa
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
) -> tuple[np.ndarray, np.ndarray]:
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

    Devuelve (distancias, puntos de impacto sobre 'updated_mesh') solo para
    los rayos que impactaron.
    """
    base_vertices = np.asarray(base_mesh.vertices)
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
    return distances, hit_points


def raycast_thickness(
    base_mesh: o3d.geometry.TriangleMesh,
    updated_mesh: o3d.geometry.TriangleMesh,
    base_origin: np.ndarray,
    updated_origin: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
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

    Devuelve (distancias, puntos de impacto sobre la malla actualizada)
    solo para los rayos que impactaron ambas mallas.
    """
    scene_updated = o3d.t.geometry.RaycastingScene()
    scene_updated.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(updated_mesh))
    scene_base = o3d.t.geometry.RaycastingScene()
    scene_base.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(base_mesh))

    base_origin = np.asarray(base_origin, dtype=np.float64)
    updated_origin = np.asarray(updated_origin, dtype=np.float64)

    vertices = np.asarray(updated_mesh.vertices)
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
    return distances, hit_points


def run_raycast_pipeline(params: RaycastPipelineParams, log=print) -> RaycastPipelineResult:
    """
    Flujo experimental alternativo a run_pipeline: en vez de Cloud-to-Cloud
    (vecino mas cercano), reconstruye una malla de cada nube
    (mesh_from_point_cloud via Poisson, o mesh_from_point_cloud_delaunay) y
    mide el espesor por raycasting, con uno de dos metodos
    (params.ray_method):

    - "origin": un rayo por vertice desde la pose guardada del sensor (ver
      raycast_thickness). Necesita el sidecar de pose ('_pose.npz') en
      AMBAS capturas, o un origen manual; asume que el sensor no se movio
      entre capturas.
    - "normal": un rayo por vertice de la malla base a lo largo de la
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
    base_cloud = load_point_cloud(params.base_path)
    updated_cloud = load_point_cloud(params.updated_path)
    log(f"  Base       : {len(base_cloud.points)} puntos")
    log(f"  Actualizada: {len(updated_cloud.points)} puntos")

    mesh_label = "Poisson" if params.mesh_method == "poisson" else "Delaunay"
    log(f"Reconstruyendo malla ({mesh_label}) de la nube base...")
    if params.mesh_method == "delaunay":
        base_mesh = mesh_from_point_cloud_delaunay(base_cloud, params.delaunay_max_edge_trim_factor)
    else:
        base_mesh = mesh_from_point_cloud(base_cloud, params.poisson_depth, params.density_trim_quantile)
    log(f"  Malla base: {len(base_mesh.vertices)} vertices, {len(base_mesh.triangles)} triangulos")

    log(f"Reconstruyendo malla ({mesh_label}) de la nube actualizada...")
    if params.mesh_method == "delaunay":
        updated_mesh = mesh_from_point_cloud_delaunay(updated_cloud, params.delaunay_max_edge_trim_factor)
    else:
        updated_mesh = mesh_from_point_cloud(updated_cloud, params.poisson_depth, params.density_trim_quantile)
    log(f"  Malla actualizada: {len(updated_mesh.vertices)} vertices, {len(updated_mesh.triangles)} triangulos")

    if params.ray_method == "normal":
        log("Tirando rayos a lo largo de la normal de la malla base hacia la malla actualizada...")
        distances, hit_points = raycast_thickness_along_normal(base_mesh, updated_mesh, params.max_distance)
    else:
        log("Tirando rayos desde el origen del sensor hacia ambas mallas...")
        distances, hit_points = raycast_thickness(base_mesh, updated_mesh, base_origin, updated_origin)
    if distances.size == 0:
        raise ValueError(
            "Ningun rayo impacto ambas mallas. Revisa que las dos capturas cubran "
            "la misma zona del tunel."
        )
    stats = summarize(distances)
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
    heatmap_cloud = build_heatmap_cloud(hit_cloud, distances, params.max_distance)
    heatmap_path = params.output_dir / "thickness_raycast_heatmap.ply"
    o3d.io.write_point_cloud(str(heatmap_path), heatmap_cloud)
    log(f"Heatmap guardado en: {heatmap_path}")

    return RaycastPipelineResult(
        stats=stats,
        distances=distances,
        hit_points=hit_points,
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


def save_histogram(path: Path, distances: np.ndarray) -> None:
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
    ax.hist(distances * 1000, bins=60, color="steelblue", edgecolor="black")
    ax.set_xlabel("Espesor de shotcrete (mm)")
    ax.set_ylabel("Cantidad de puntos")
    ax.set_title("Distribucion del espesor de shotcrete (Cloud-to-Cloud)")
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
    _start_at_sensor_pov(vis)
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
