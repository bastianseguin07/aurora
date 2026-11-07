"""Open3D scene window shared by the GUI's static 3D inspection actions."""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import shutil
import time
from pathlib import Path

import numpy as np
import open3d as o3d
from open3d.visualization import gui, rendering
from cloud_display_colors import color_individual_cloud

from pointcloud_core import (
    compute_pose_relative_transform,
    load_point_cloud,
    prepare_thickness_overlay,
    grayscale_point_cloud,
    world_to_sensor_view_rotation,
)

logger = logging.getLogger(__name__)


def _rgb_image(color: tuple[float, float, float], width: int = 28, height: int = 18) -> o3d.geometry.Image:
    pixel = np.clip(np.asarray(color) * 255, 0, 255).astype(np.uint8)
    return o3d.geometry.Image(np.tile(pixel, (height, width, 1)))


def _legend_panel(spec: dict, em: int) -> gui.Vert:
    panel = gui.Vert(0.35 * em, gui.Margins(0.75 * em, 0.65 * em, 0.75 * em, 0.65 * em))
    # Let the native scene background show through the legend instead of
    # drawing a dark rectangular card over the 3D view.
    panel.background_color = gui.Color(0.0, 0.0, 0.0, 0.0)
    title = gui.Label(spec.get("title", "Espesor"))
    title.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
    panel.add_child(title)

    rows = spec.get("rows")
    if rows:
        for color, label, value in rows:
            row = gui.Horiz(0.45 * em)
            row.add_child(gui.ImageWidget(_rgb_image(tuple(color))))
            label_widget = gui.Label(str(label))
            label_widget.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
            row.add_child(label_widget)
            row.add_stretch()
            value_widget = gui.Label(str(value))
            value_widget.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
            row.add_child(value_widget)
            panel.add_child(row)
    else:
        low = np.asarray(spec.get("low_color", (0.0, 0.482, 1.0)))
        high = np.asarray(spec.get("high_color", (1.0, 0.231, 0.188)))
        ramp = np.linspace(low, high, 256)
        image = np.tile(np.clip(ramp * 255, 0, 255).astype(np.uint8)[None, :, :], (16, 1, 1))
        panel.add_child(gui.ImageWidget(o3d.geometry.Image(image)))
        labels = gui.Horiz()
        low_label = gui.Label("0 cm")
        low_label.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
        labels.add_child(low_label)
        labels.add_stretch()
        high_label = gui.Label(f">= {float(spec['continuous_max_cm']):.2f} cm")
        high_label.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
        labels.add_child(high_label)
        panel.add_child(labels)
    for color, label, value in spec.get("extra_rows", []):
        row = gui.Horiz(0.45 * em)
        row.add_child(gui.ImageWidget(_rgb_image(tuple(color))))
        label_widget = gui.Label(str(label))
        label_widget.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
        row.add_child(label_widget)
        row.add_stretch()
        value_widget = gui.Label(str(value))
        value_widget.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
        row.add_child(value_widget)
        panel.add_child(row)
    return panel


def _look_at_sensor_pov(scene_widget, points: np.ndarray, fov_deg: float = 60.0, forward_axis: str = "z") -> None:
    """Frame the main point mass; isolated bad depth returns must not zoom it away."""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return
    if len(points) > 250_000:
        indices = np.linspace(0, len(points) - 1, 250_000, dtype=np.int64)
        points = points[indices]
    low, high = np.percentile(points, (1.0, 99.0), axis=0)
    center = 0.5 * (low + high)
    radius = 0.5 * float(np.linalg.norm(high - low))
    axis = "xyz".index(forward_axis)
    off_axis = float(np.linalg.norm(np.delete(center, axis)))
    distance = (radius + off_axis) / max(math.sin(math.radians(fov_deg) / 2.0), 1e-6) * 1.1
    distance = max(distance, 0.5)
    target = np.zeros(3); target[axis] = center[axis]
    direction = np.zeros(3); direction[axis] = distance
    scene_widget.look_at(
        target,
        target - direction,
        np.array([0.0, 0.0, 1.0]) if forward_axis != "z" else np.array([0.0, -1.0, 0.0]),
    )


def _replace_live_geometry(scene, points, colors, material, already_added):
    """Apply an empty cloud as a real scene update, not as a missing message."""
    if already_added:
        scene.remove_geometry("live")
    if len(points) == 0:
        return False
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    if colors.shape == points.shape:
        cloud.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
    else:
        cloud.paint_uniform_color((0.10, 0.72, 0.95))
    scene.add_geometry("live", cloud, material)
    return True


def _read_geometries(
    kind: str, paths: list[Path], heatmap_spec: dict | None = None,
    empty_stages: set[str] | None = None,
    fallback_base_path: Path | None = None,
):
    material = rendering.MaterialRecord()
    material.shader = "defaultUnlit"
    material.point_size = 3.2 if kind in ("pair", "pair_box", "thickness") else 2.5

    if kind == "cloud":
        cloud = color_individual_cloud(paths[0], load_point_cloud(paths[0]))
        material.point_size = 3.0
        return [("nube", cloud, material)]
    if kind == "live":
        cloud = o3d.geometry.PointCloud()
        return [("live", cloud, material)]
    if kind == "pair_box":
        empty_stages = empty_stages or set()
        base = o3d.geometry.PointCloud() if "antes" in empty_stages else o3d.io.read_point_cloud(str(paths[0]))
        updated = o3d.geometry.PointCloud() if "despues" in empty_stages else o3d.io.read_point_cloud(str(paths[1]))
        if len(base.points) and len(updated.points):
            base, updated = prepare_thickness_overlay(base, updated, heatmap_spec)
        elif not len(base.points) and len(updated.points) and fallback_base_path is not None:
            # If the box contains no ANTES samples, keep the display clipped
            # but color DESPUES against the full ANTES cloud as a reference.
            fallback_base = load_point_cloud(fallback_base_path)
            _, updated = prepare_thickness_overlay(fallback_base, updated, heatmap_spec)
        else:
            if len(base.points):
                base = grayscale_point_cloud(base)
            if len(updated.points) and not updated.has_colors():
                updated.paint_uniform_color((0.10, 0.72, 0.95))
        return [("antes", base, material), ("despues", updated, material)]
    if kind == "pair":
        base = load_point_cloud(paths[0])
        updated = load_point_cloud(paths[1])
        base, updated = prepare_thickness_overlay(base, updated, heatmap_spec)
        return [("antes", base, material), ("despues", updated, material)]
    if kind == "thickness":
        base = load_point_cloud(paths[0])
        heatmap = load_point_cloud(paths[1])
        base = grayscale_point_cloud(base)
        if not heatmap.has_colors():
            heatmap.paint_uniform_color((0.10, 0.72, 0.95))
        return [("antes", base, material), ("despues", heatmap, material)]

    mesh_material = rendering.MaterialRecord()
    mesh_material.shader = "defaultLit"
    base_mesh = o3d.io.read_triangle_mesh(str(paths[0]))
    updated_mesh = o3d.io.read_triangle_mesh(str(paths[1]))
    if base_mesh.is_empty() or updated_mesh.is_empty():
        raise RuntimeError("No se pudieron cargar las dos mallas para inspeccionarlas.")
    if not base_mesh.has_vertex_colors():
        base_mesh.paint_uniform_color((0.55, 0.6, 0.65))
    if not updated_mesh.has_vertex_colors():
        updated_mesh.paint_uniform_color((1.0, 0.549, 0.0))
    base_mesh.compute_vertex_normals()
    updated_mesh.compute_vertex_normals()
    return [("antes", base_mesh, mesh_material), ("despues", updated_mesh, mesh_material)]


def _pose_grid(extent: float = 0.5, step: float = 0.1):
    points, lines = [], []
    for i in range(-round(extent / step), round(extent / step) + 1):
        value = i * step
        j = len(points)
        points.extend(((value, 0, -extent), (value, 0, extent)))
        lines.append((j, j + 1))
        j = len(points)
        points.extend(((-extent, 0, value), (extent, 0, value)))
        lines.append((j, j + 1))
    grid = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(np.asarray(points, dtype=float)),
        lines=o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32)),
    )
    grid.colors = o3d.utility.Vector3dVector(np.tile([[0.40, 0.42, 0.45]], (len(lines), 1)))
    return grid


def _pose_metrics_panel(em: int):
    status = gui.Label("Leyendo pose del sensor")
    status.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
    labels, titles = {}, {}
    for key, title, color in (
        ("x", "X", (0.88, 0.18, 0.14, 1.0)),
        ("y", "Y", (0.18, 0.62, 0.24, 1.0)),
        ("z", "Z", (0.12, 0.34, 0.92, 1.0)),
        ("roll", "Roll", (0.12, 0.14, 0.17, 1.0)),
        ("pitch", "Pitch", (0.12, 0.14, 0.17, 1.0)),
        ("yaw", "Yaw", (0.12, 0.14, 0.17, 1.0)),
    ):
        name = gui.Label(title)
        name.text_color = gui.Color(*color)
        value = gui.Label("—")
        value.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
        titles[key] = name
        labels[key] = value
    total = gui.Label("Distancia total: —")
    total.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
    widgets = [status, *titles.values(), *labels.values(), total]
    for widget in widgets:
        widget.background_color = gui.Color(0.0, 0.0, 0.0, 0.0)
    return status, titles, labels, total, widgets


def _rotation_from_forward(direction: np.ndarray) -> np.ndarray:
    """Rotate the mesh arrow's local +Z axis onto a normalized direction."""
    target = np.asarray(direction, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(target))
    if not np.isfinite(norm) or norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    target /= norm
    source = np.array([0.0, 0.0, 1.0])
    cross = np.cross(source, target)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if sine < 1e-12:
        if cosine >= 0.0:
            return np.eye(3, dtype=np.float64)
        return np.array(
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            dtype=np.float64,
        )
    x, y, z = cross / sine
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + skew * sine + (skew @ skew) * (1.0 - cosine)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "kind", choices=("cloud", "pair", "pair_box", "thickness", "meshes", "live", "pick", "pose")
    )
    parser.add_argument("paths", nargs="+")
    parser.add_argument("--forward-axis", choices=("x", "y", "z"), default="z")
    parser.add_argument("--title", default="Aurora - Vista 3D")
    parser.add_argument("--cleanup-dir")
    parser.add_argument("--legend-json")
    parser.add_argument("--box-bounds-json")
    parser.add_argument("--selection-output")
    parser.add_argument("--pick-count", type=int, default=0)
    parser.add_argument("--pick-prompt", default="Clic izquierdo para marcar puntos; Q para terminar")
    parser.add_argument("--pick-requires-shift", action="store_true")
    parser.add_argument("--context-cloud", action="append", default=[])
    parser.add_argument("--camera-reference", action="append", default=[])
    parser.add_argument("--empty-stage", choices=("antes", "despues"), action="append", default=[])
    parser.add_argument("--exit-status-file")
    parser.add_argument("--pose-reference-json")
    parser.add_argument("--pose-tolerance-m", type=float, default=0.02)
    parser.add_argument("--heatmap-spec-json")
    parser.add_argument("--thickness-stats-json")
    args = parser.parse_args()
    exit_reason: str | None = None

    try:
        paths = [Path(path) for path in args.paths]
        heatmap_spec = json.loads(args.heatmap_spec_json) if args.heatmap_spec_json else None
        geometries = (
            [] if args.kind == "pose"
            else _read_geometries(
                "cloud" if args.kind == "pick" else args.kind,
                paths,
                heatmap_spec,
                set(args.empty_stage),
                Path(args.camera_reference[0])
                if args.kind == "pair_box" and args.camera_reference
                else None,
            )
        )
        if args.kind == "pick":
            for index, context_path in enumerate(args.context_cloud):
                context_cloud = load_point_cloud(Path(context_path))
                geometries.append((f"context_{index}", context_cloud, geometries[0][2]))
        pose_reference = None
        if args.kind == "pose":
            if not args.pose_reference_json:
                raise ValueError("Falta la pose de referencia para la guia IMU.")
            ref_position, ref_rpy = json.loads(args.pose_reference_json)
            pose_reference = (np.asarray(ref_position, dtype=float), np.asarray(ref_rpy, dtype=float))
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.22)
            axes_material = rendering.MaterialRecord()
            axes_material.shader = "defaultUnlit"
            geometries = [
                ("reticula", _pose_grid(), rendering.MaterialRecord()),
                (
                    "marco_referencia",
                    o3d.geometry.TriangleMesh.create_sphere(radius=0.019),
                    rendering.MaterialRecord(),
                ),
                ("ejes_sensor", axes, axes_material),
            ]
            geometries[1][1].paint_uniform_color((0.20, 0.80, 0.35))
            geometries[1][1].compute_vertex_normals()

        if args.box_bounds_json:
            box_values = np.asarray(json.loads(args.box_bounds_json), dtype=np.float64)
            if box_values.shape == (8, 3):
                corners = box_values
            elif box_values.shape == (2, 3):
                low, high = box_values
                corners = np.asarray(
                    [
                        (low[0], low[1], low[2]),
                        (high[0], low[1], low[2]),
                        (high[0], high[1], low[2]),
                        (low[0], high[1], low[2]),
                        (low[0], low[1], high[2]),
                        (high[0], low[1], high[2]),
                        (high[0], high[1], high[2]),
                        (low[0], high[1], high[2]),
                    ],
                    dtype=np.float64,
                )
            else:
                raise ValueError("La caja debe tener dos limites XYZ o exactamente 8 vertices.")
            edges = np.asarray(
                [
                    (0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7),
                ],
                dtype=np.int32,
            )
            box = o3d.geometry.LineSet()
            box.points = o3d.utility.Vector3dVector(corners)
            box.lines = o3d.utility.Vector2iVector(edges)
            box.colors = o3d.utility.Vector3dVector(
                np.tile(np.asarray([[0.82, 0.02, 0.48]], dtype=np.float64), (len(edges), 1))
            )
            box_material = rendering.MaterialRecord()
            box_material.shader = "unlitLine"
            box_material.line_width = 4.0
            box_material.base_color = np.asarray([0.82, 0.02, 0.48, 1.0], dtype=np.float32)
            geometries.append(("caja_recorte", box, box_material))

        app = gui.Application.instance
        app.initialize()
        # Keep the native window hidden until its scene and camera are ready.
        window = app.create_window(args.title, 1280, 800, -1, -1, 1)
        scene_widget = gui.SceneWidget()
        scene_widget.scene = rendering.Open3DScene(window.renderer)
        if args.kind == "cloud":
            scene_widget.scene.set_background(np.array([48/255, 48/255, 48/255, 1.0]))

        for name, geometry, material in geometries:
            if args.kind != "live":
                if isinstance(geometry, o3d.geometry.PointCloud) and not len(geometry.points):
                    continue
                scene_widget.scene.add_geometry(name, geometry, material)
        if args.kind != "pose":
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
            axes_material = rendering.MaterialRecord()
            axes_material.shader = "defaultUnlit"
            scene_widget.scene.add_geometry("ejes", axes, axes_material)

        legend = json.loads(args.legend_json) if args.legend_json else None
        legend_widget = _legend_panel(legend, window.theme.font_size) if legend else None
        thickness_stats = json.loads(args.thickness_stats_json) if args.thickness_stats_json else None
        stats_widgets = []
        if thickness_stats:
            stats_title = gui.Label("ESTADÍSTICAS DE ESPESOR")
            stats_title.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
            stats_rows = (
                f"Media: {thickness_stats['mean_cm']:.2f} cm",
                f"Mediana: {thickness_stats['median_cm']:.2f} cm",
                f"Desviación estándar: {thickness_stats['std_cm']:.2f} cm",
            )
            stats_widgets = [stats_title]
            for row_text in stats_rows:
                row = gui.Label(row_text)
                row.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
                stats_widgets.append(row)
            for widget in stats_widgets:
                widget.background_color = gui.Color(0.0, 0.0, 0.0, 0.0)
        pair_controls = None
        if args.kind in ("pair", "pair_box", "thickness", "meshes"):
            pair_controls = gui.Horiz(
                0.5 * window.theme.font_size,
                gui.Margins(0.5 * window.theme.font_size, 0.35 * window.theme.font_size,
                            0.5 * window.theme.font_size, 0.35 * window.theme.font_size),
            )
            pair_controls.background_color = gui.Color(0.0, 0.0, 0.0, 0.0)
            before_toggle = gui.Checkbox("")
            after_toggle = gui.Checkbox("")
            before_toggle.checked = True
            after_toggle.checked = True
            before_toggle.set_on_checked(
                lambda visible: scene_widget.scene.show_geometry("antes", visible)
            )
            after_toggle.set_on_checked(
                lambda visible: scene_widget.scene.show_geometry("despues", visible)
            )
            for stage, toggle, label in (
                ("antes", before_toggle, "ANTES"), ("despues", after_toggle, "DESPUES")
            ):
                if stage in args.empty_stage:
                    toggle.enabled = False
                    label = f"{label} (sin puntos)"
                group = gui.Horiz(0.12 * window.theme.font_size)
                group.add_child(toggle)
                label_widget = gui.Label(label)
                label_widget.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
                label_line = gui.Vert(
                    0,
                    gui.Margins(0, 0.3 * window.theme.font_size, 0, 0),
                )
                label_line.add_child(label_widget)
                group.add_child(label_line)
                pair_controls.add_child(group)
        live_status = None
        pose_status = pose_titles = pose_labels = pose_total = pose_widgets = None
        pose_last_mtime = -1
        pose_last_poll = 0.0
        if args.kind == "pose":
            pose_status, pose_titles, pose_labels, pose_total, pose_widgets = _pose_metrics_panel(window.theme.font_size)
            current = o3d.geometry.TriangleMesh.create_sphere(radius=0.025, resolution=18)
            current.compute_vertex_normals()
            current.paint_uniform_color((0.90, 0.25, 0.25))
            scene_widget.scene.add_geometry("pose_actual", current, rendering.MaterialRecord())
        if args.kind == "live":
            live_status = gui.Label("Esperando el primer frame del sensor...")
            live_status.background_color = gui.Color(0.0, 0.0, 0.0, 0.0)
            live_status.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)
        pick_status = None
        picked_points: list[np.ndarray] = []
        picked_labels = []
        preview_widget = None
        cloud_points = np.asarray(geometries[0][1].points) if args.kind == "pick" else None
        if args.kind == "pick":
            pick_status = gui.Label(args.pick_prompt)
            # Keep picking instructions as quiet as the thickness legend:
            # transparent panel, dark readable text, and the same theme font size.
            pick_status.background_color = gui.Color(0.0, 0.0, 0.0, 0.0)
            pick_status.text_color = gui.Color(0.12, 0.14, 0.17, 1.0)

        def on_layout(layout_context):
            bounds = window.content_rect
            scene_widget.frame = bounds
            if preview_widget is not None:
                preview_widget.frame = bounds
            if pair_controls is not None:
                preferred = pair_controls.calc_preferred_size(
                    layout_context, gui.Widget.Constraints()
                )
                pair_controls.frame = gui.Rect(
                    bounds.x + window.theme.font_size,
                    bounds.y + window.theme.font_size,
                    preferred.width,
                    preferred.height,
                )
                if stats_widgets:
                    stats_x = bounds.x + window.theme.font_size
                    stats_y = bounds.y + window.theme.font_size + preferred.height + 6
                    stats_width = min(520, max(0, bounds.width - 2 * window.theme.font_size))
                    for index, widget in enumerate(stats_widgets):
                        widget.frame = gui.Rect(
                            stats_x,
                            stats_y + index * 23,
                            stats_width,
                            22,
                        )
            if legend_widget is not None:
                preferred = legend_widget.calc_preferred_size(
                    layout_context, gui.Widget.Constraints()
                )
                width = min(preferred.width, max(220, bounds.width - 2 * window.theme.font_size))
                height = min(preferred.height, bounds.height - 2 * window.theme.font_size)
                legend_widget.frame = gui.Rect(
                    bounds.x + bounds.width - width - window.theme.font_size,
                    bounds.y + window.theme.font_size,
                    width,
                    height,
                )
            if pick_status is not None:
                pref = pick_status.calc_preferred_size(layout_context, gui.Widget.Constraints())
                pick_status.frame = gui.Rect(
                    bounds.x + window.theme.font_size,
                    bounds.y + window.theme.font_size,
                    min(bounds.width - 2 * window.theme.font_size, pref.width + window.theme.font_size),
                    pref.height + 0.5 * window.theme.font_size,
                )
            if live_status is not None:
                pref = live_status.calc_preferred_size(layout_context, gui.Widget.Constraints())
                live_status.frame = gui.Rect(
                    bounds.x + window.theme.font_size,
                    bounds.y + window.theme.font_size,
                    min(bounds.width - 2 * window.theme.font_size, pref.width + window.theme.font_size),
                    pref.height + 0.5 * window.theme.font_size,
                )
            if pose_widgets is not None:
                panel_width = min(340, max(0, bounds.width - 2 * window.theme.font_size))
                panel_x = bounds.x + bounds.width - panel_width - window.theme.font_size
                top = bounds.y + window.theme.font_size
                pose_status.frame = gui.Rect(panel_x, top, panel_width, 28)
                for row, key in enumerate(("x", "y", "z", "roll", "pitch", "yaw")):
                    row_y = top + 34 + row * 29
                    pose_titles[key].frame = gui.Rect(panel_x, row_y, 68, 26)
                    pose_labels[key].frame = gui.Rect(panel_x + 74, row_y, max(0, panel_width - 74), 26)
                pose_total.frame = gui.Rect(panel_x, top + 34 + 6 * 29, panel_width, 28)

        window.add_child(scene_widget)
        if pose_widgets is not None:
            for widget in pose_widgets:
                window.add_child(widget)

        # Give SceneWidget a real viewport before configuring its camera.
        # Calling setup_camera() while its frame is still 0x0 leaves the first
        # visible swapchain frame black until a later layout pass repairs it.
        scene_widget.frame = window.content_rect
        bounds = scene_widget.scene.bounding_box
        center = bounds.get_center()
        camera_fov_deg = 60.0
        scene_widget.setup_camera(camera_fov_deg, bounds, center)
        # Aurora depth points use the sensor frame (+Z forward, +Y down).
        # Keep the initial optical axis parallel to +Z, but preserve the fit
        # distance that setup_camera calculated from the full scene bounds.
        # Putting the eye at (0, 0, 0) made the cloud look unnaturally close;
        # viewer zoom is framing and does not change the cloud's metric data.
        camera_geometries = geometries[:1] if args.kind == "pick" else geometries
        if args.camera_reference:
            point_sets = [
                np.asarray(cloud.points)
                for cloud in (o3d.io.read_point_cloud(path) for path in args.camera_reference)
                if len(cloud.points)
            ]
        else:
            point_sets = [
                np.asarray(geometry.points)
                for _name, geometry, _material in camera_geometries
                if isinstance(geometry, o3d.geometry.PointCloud) and len(geometry.points)
            ]
        if args.kind == "pose":
            scene_widget.look_at(
                np.zeros(3), np.array([0.0, -0.65, -1.7]), np.array([0.0, -1.0, 0.0])
            )
        elif point_sets:
            _look_at_sensor_pov(scene_widget, np.concatenate(point_sets, axis=0), camera_fov_deg, args.forward_axis)
        else:
            fitted_eye = np.asarray(scene_widget.scene.camera.get_model_matrix())[:3, 3]
            view_distance = float(np.linalg.norm(fitted_eye - np.asarray(center)))
            if not np.isfinite(view_distance) or view_distance < 1e-6:
                view_distance = max(float(bounds.get_max_extent()) * 1.5, 0.5)
            radius = 0.5 * float(np.linalg.norm(bounds.get_extent()))
            off_axis = float(np.linalg.norm(np.asarray(center)[:2]))
            sensor_axis_fit = (radius + off_axis) / max(
                math.sin(math.radians(camera_fov_deg) / 2.0), 1e-6
            ) * 1.1
            view_distance = max(view_distance, sensor_axis_fit)
            axis_target = np.array([0.0, 0.0, float(center[2])])
            scene_widget.look_at(
                axis_target,
                axis_target - np.array([0.0, 0.0, view_distance]),
                np.array([0.0, -1.0, 0.0]),
            )
        # Render the scene before mapping the native window. Show this prepared
        # frame as a temporary image while the interactive SceneWidget draws
        # its first on-screen frames behind it.
        if args.kind != "pose":
            warmup_image = app.render_to_image(
                scene_widget.scene,
                max(1, scene_widget.frame.width),
                max(1, scene_widget.frame.height),
            )
            preview_widget = gui.ImageWidget(warmup_image)
            preview_widget.frame = window.content_rect
            window.add_child(preview_widget)
        else:
            warmup_image = None
        if pair_controls is not None:
            window.add_child(pair_controls)
        for widget in stats_widgets:
            window.add_child(widget)
        if legend_widget is not None:
            window.add_child(legend_widget)
        if pick_status is not None:
            window.add_child(pick_status)
        if live_status is not None:
            window.add_child(live_status)
        window.set_on_layout(on_layout)
        del warmup_image

        modifier_keys = {
            gui.KeyName.LEFT_SHIFT,
            gui.KeyName.RIGHT_SHIFT,
            gui.KeyName.LEFT_CONTROL,
            gui.KeyName.RIGHT_CONTROL,
            gui.KeyName.ALT,
            gui.KeyName.META,
        }
        pressed_modifiers: set[int] = set()
        last_key_event_at = 0.0

        def handle_key_event(event) -> None:
            nonlocal last_key_event_at, exit_reason
            last_key_event_at = time.monotonic()
            key = event.key
            if event.type == gui.KeyEvent.Type.DOWN:
                if key in modifier_keys:
                    pressed_modifiers.add(key)
                elif key == gui.KeyName.Q and not pressed_modifiers:
                    if args.selection_output:
                        np.save(
                            args.selection_output,
                            np.asarray(picked_points, dtype=float).reshape((-1, 3)),
                        )
                    exit_reason = "q"
                    app.quit()
                else:
                    exit_reason = "key"
            elif event.type == gui.KeyEvent.Type.UP:
                pressed_modifiers.discard(key)
                exit_reason = "key"

        def on_window_key(event) -> bool:
            try:
                handle_key_event(event)
            except Exception:
                # Never let a Python callback exception unwind into Open3D's
                # event loop, where it can terminate the entire viewer.
                logger.exception("No se pudo procesar una tecla en la vista 3D")
            # Consume both phases before Open3D's default key handling.
            return True

        def on_scene_key(event):
            try:
                handle_key_event(event)
            except Exception:
                logger.exception("No se pudo procesar una tecla en la escena 3D")
            # Always consume the event so default handling cannot dismiss the window.
            return gui.Widget.CONSUMED

        window.set_on_key(on_window_key)
        scene_widget.set_on_key(on_scene_key)

        def on_close() -> bool:
            nonlocal exit_reason
            # Open3D/GLFW may still turn a key event into a native close request
            # after widget dispatch. Reject that request briefly after any key;
            # the titlebar X remains available, and unmodified Q exits above.
            allow_close = time.monotonic() - last_key_event_at > 0.35
            if allow_close:
                exit_reason = "window"
            return allow_close

        window.set_on_close(on_close)

        # Shift+left-drag pans across the cloud; the wheel remains the zoom
        # gesture. Wheel events while a button is held are consumed.
        drag_origin: tuple[float, float] | None = None
        drag_left_button = False
        drag_pick_shift = False
        last_drag_y: float | None = None
        last_drag_x: float | None = None

        def rotate_vector(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
            axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
            return (
                vector * math.cos(angle)
                + np.cross(axis, vector) * math.sin(angle)
                + axis * float(np.dot(axis, vector)) * (1.0 - math.cos(angle))
            )

        def on_mouse(event):
            nonlocal drag_origin, drag_left_button, drag_pick_shift, last_drag_x, last_drag_y
            left_down = event.is_button_down(gui.MouseButton.LEFT)
            shift_down = event.is_modifier_down(gui.KeyModifier.SHIFT) or any(
                key in pressed_modifiers
                for key in (gui.KeyName.LEFT_SHIFT, gui.KeyName.RIGHT_SHIFT)
            )
            if event.type == gui.MouseEvent.Type.BUTTON_DOWN:
                drag_origin = (event.x, event.y)
                drag_left_button = left_down
                drag_pick_shift = shift_down
                last_drag_x = event.x
                last_drag_y = event.y
                # Prevent a press/release click from reaching Open3D's default
                # camera handler. Camera motion is applied only from DRAG below.
                return gui.Widget.CONSUMED
            if event.type == gui.MouseEvent.Type.WHEEL:
                # int(...) matters: comparing the MouseButton enum directly
                # with "!=" is always True in this Open3D build, which made
                # every wheel event look like "button held" and get dropped
                # below without ever reaching the zoom code.
                if int(event.buttons) != int(gui.MouseButton.NONE):
                    return gui.Widget.CONSUMED
                # set_on_mouse() replaces Open3D's built-in camera handling
                # entirely (returning IGNORED does not fall back to it), so
                # the wheel-to-zoom gesture has to be implemented here too,
                # the same way drag-to-rotate/pan already is above.
                camera = scene_widget.scene.camera
                model = np.asarray(camera.get_model_matrix())
                eye = model[:3, 3]
                center = np.asarray(scene_widget.center_of_rotation)
                up = model[:3, 1]
                direction = eye - center
                distance = float(np.linalg.norm(direction))
                if distance > 1e-9:
                    zoom_factor = math.exp(-float(event.wheel_dy) * 0.1)
                    new_distance = max(distance * zoom_factor, 0.01)
                    new_eye = center + direction / distance * new_distance
                    camera.look_at(center, new_eye, up)
                return gui.Widget.CONSUMED
            if event.type == gui.MouseEvent.Type.DRAG and left_down and last_drag_y is not None and last_drag_x is not None:
                delta_x = event.x - last_drag_x
                delta_y = event.y - last_drag_y
                last_drag_x = event.x
                last_drag_y = event.y
                if drag_origin is not None and math.hypot(
                    event.x - drag_origin[0], event.y - drag_origin[1]
                ) < 5:
                    return gui.Widget.CONSUMED
                if shift_down and (abs(delta_x) > 0 or abs(delta_y) > 0):
                    camera = scene_widget.scene.camera
                    model = np.asarray(camera.get_model_matrix())
                    eye = model[:3, 3]
                    center = np.asarray(scene_widget.center_of_rotation)
                    distance = float(np.linalg.norm(eye - center))
                    world_per_pixel = (
                        2.0 * distance * math.tan(math.radians(camera_fov_deg) / 2.0)
                        / max(float(scene_widget.frame.height), 1.0)
                    )
                    right = model[:3, 0]
                    up = model[:3, 1]
                    translation = (
                        right * (-float(delta_x) * world_per_pixel)
                        + up * (float(delta_y) * world_per_pixel)
                    )
                    new_center = center + translation
                    camera.look_at(new_center, eye + translation, up)
                    scene_widget.center_of_rotation = new_center
                elif not shift_down and (abs(delta_x) > 0 or abs(delta_y) > 0):
                    camera = scene_widget.scene.camera
                    model = np.asarray(camera.get_model_matrix())
                    center = np.asarray(scene_widget.center_of_rotation)
                    offset = model[:3, 3] - center
                    up = model[:3, 1]
                    right = model[:3, 0]
                    yaw = -float(delta_x) * 0.006
                    pitch = -float(delta_y) * 0.006
                    offset = rotate_vector(offset, up, yaw)
                    right = rotate_vector(right, up, yaw)
                    up = rotate_vector(up, up, yaw)
                    offset = rotate_vector(offset, right, pitch)
                    up = rotate_vector(up, right, pitch)
                    camera.look_at(center, center + offset, up)
                return gui.Widget.CONSUMED
            if event.type == gui.MouseEvent.Type.BUTTON_UP and drag_origin is not None:
                start = drag_origin
                was_left_click = drag_left_button
                was_shift_click = drag_pick_shift
                drag_origin = None
                drag_left_button = False
                drag_pick_shift = False
                last_drag_x = None
                last_drag_y = None
                moved = math.hypot(event.x - start[0], event.y - start[1])
                can_pick = args.pick_count == 1 or args.pick_count <= 0 or len(picked_points) < args.pick_count
                correct_modifier = not args.pick_requires_shift or was_shift_click
                if args.kind == "pick" and was_left_click and moved < 5 and can_pick and correct_modifier:
                    x, y = int(event.x - scene_widget.frame.x), int(event.y - scene_widget.frame.y)

                    if x < 0 or y < 0 or x >= scene_widget.frame.width or y >= scene_widget.frame.height:
                        return gui.Widget.CONSUMED
                    camera = scene_widget.scene.camera
                    near = np.asarray(camera.unproject(float(x), float(y), 0.0, scene_widget.frame.width, scene_widget.frame.height)).reshape(3)
                    ray_probe = np.asarray(
                        camera.unproject(
                            float(x), float(y), 0.5,
                            scene_widget.frame.width, scene_widget.frame.height,
                        )
                    ).reshape(3)
                    # Open3D's perspective projection has an infinite far
                    # plane; unprojecting depth=1 returns infinities. A finite
                    # mid-depth sample gives the same click-ray direction.
                    ray = ray_probe - near
                    ray_length = float(np.linalg.norm(ray))
                    if not np.all(np.isfinite(ray)) or ray_length <= 1e-9:
                        return gui.Widget.CONSUMED
                    ray /= ray_length
                    # Project all cloud points onto the exact click ray. The
                    # former fixed 700-sample ray skipped dense points whenever
                    # the camera's far plane was large, so clicks often missed.
                    offsets = cloud_points - near
                    along = offsets @ ray
                    in_depth = along >= 0.0
                    if not np.any(in_depth):
                        return gui.Widget.CONSUMED
                    perpendicular = offsets - along[:, None] * ray
                    perpendicular_sq = np.einsum("ij,ij->i", perpendicular, perpendicular)
                    world_per_pixel = np.maximum(along, 1e-6) * (
                        2.0 * math.tan(math.radians(camera_fov_deg) / 2.0)
                        / max(float(scene_widget.frame.height), 1.0)
                    )
                    screen_distance_px = np.sqrt(perpendicular_sq) / np.maximum(world_per_pixel, 1e-9)
                    screen_distance_px[~in_depth] = np.inf
                    best_index = int(np.argmin(screen_distance_px))
                    if screen_distance_px[best_index] <= 12.0:
                        point = cloud_points[best_index]
                        replacing_single = args.pick_count == 1
                        if replacing_single:
                            if picked_points:
                                scene_widget.scene.remove_geometry("picked_1")
                            if picked_labels:
                                scene_widget.remove_3d_label(picked_labels.pop())
                            picked_points[:] = [point.copy()]
                        else:
                            picked_points.append(point.copy())
                        marker_radius = max(float(world_per_pixel[best_index]) * 7.0, 0.006)
                        marker = o3d.geometry.TriangleMesh.create_sphere(radius=marker_radius)
                        marker.translate(point)
                        marker.paint_uniform_color((1.0, 0.34, 0.08))
                        marker_material = rendering.MaterialRecord()
                        marker_material.shader = "defaultUnlit"
                        marker_name = "picked_1" if replacing_single else f"picked_{len(picked_points)}"
                        scene_widget.scene.add_geometry(marker_name, marker, marker_material)
                        picked_labels.append(
                            scene_widget.add_3d_label(point, str(len(picked_points)))
                        )
                        if pick_status is not None:
                            limit = f"/{args.pick_count}" if args.pick_count > 0 else ""
                            if replacing_single:
                                pick_status.text = f"{args.pick_prompt}   ·   Punto 1 marcado; otro clic lo reemplaza"
                            else:
                                pick_status.text = f"{args.pick_prompt}   ·   Puntos: {len(picked_points)}{limit}"
                        if args.selection_output:
                            np.save(args.selection_output, np.asarray(picked_points, dtype=float).reshape((-1, 3)))
                    elif pick_status is not None:
                        pick_status.text = (
                            "No encontre un punto bajo el clic; acerca el zoom y vuelve a hacer clic."
                        )

                    # Camera/ray picking is synchronous; keeping it inside the
                    # event callback avoids races with closing the window.
                    return gui.Widget.CONSUMED
                if (
                    args.kind == "pick" and args.pick_requires_shift and was_left_click
                    and moved < 5 and not was_shift_click and pick_status is not None
                ):
                    pick_status.text = "Mantén Shift y haz clic sobre la pared para marcar el punto."
                    return gui.Widget.CONSUMED
            return gui.Widget.IGNORED

        preview_frames = 0
        live_added = False
        last_live_mtime_ns = -1
        live_camera_initialized = False
        last_live_poll = 0.0
        live_count_history: list[int] = []

        def refresh_pose() -> bool:
            nonlocal pose_last_mtime, pose_last_poll
            now = time.monotonic()
            if now - pose_last_poll < 0.08:
                return False
            pose_last_poll = now
            try:
                mtime = paths[0].stat().st_mtime_ns
                if mtime == pose_last_mtime:
                    return False
                state = json.loads(paths[0].read_text(encoding="utf-8"))
                position = np.asarray(state["position"], dtype=float)
                rpy = np.asarray(state["rpy_deg"], dtype=float)
                ref_position, ref_rpy = pose_reference
                offset = world_to_sensor_view_rotation(ref_rpy) @ (position - ref_position)
                current_rotation, _ = compute_pose_relative_transform(
                    pose_reference, (position, rpy)
                )
                forward = current_rotation @ np.array([0.0, 0.0, 1.0])
                forward /= max(float(np.linalg.norm(forward)), 1e-12)
                rotation_error = ((ref_rpy - rpy + 180.0) % 360.0) - 180.0
                distance = float(np.linalg.norm(offset))
                scene_widget.scene.remove_geometry("pose_actual")
                marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.032, resolution=20)
                marker.compute_vertex_normals()
                marker.paint_uniform_color((0.90, 0.25, 0.25))
                marker.translate(offset)
                scene_widget.scene.add_geometry("pose_actual", marker, rendering.MaterialRecord())
                scene_widget.scene.remove_geometry("pose_actual_front")
                front_arrow = o3d.geometry.TriangleMesh.create_arrow(
                    cylinder_radius=0.006,
                    cone_radius=0.016,
                    cylinder_height=0.17,
                    cone_height=0.06,
                    resolution=16,
                )
                front_rotation = _rotation_from_forward(forward)
                front_arrow.rotate(front_rotation, center=np.zeros(3))
                front_arrow.translate(offset + forward * 0.025)
                front_arrow.paint_uniform_color((0.05, 0.24, 1.0))
                front_arrow.compute_vertex_normals()
                front_material = rendering.MaterialRecord()
                front_material.shader = "defaultUnlit"
                scene_widget.scene.add_geometry(
                    "pose_actual_front", front_arrow, front_material
                )
                active = next((i for i, axis in enumerate(("x", "y", "z")) if abs(offset[i]) > args.pose_tolerance_m), None)
                if active is None:
                    pose_status.text = "Sensor en posición"
                    pose_status.text_color = gui.Color(0.12, 0.55, 0.25, 1.0)
                else:
                    words = (("derecha", "izquierda"), ("abajo", "arriba"), ("adelante", "atrás"))
                    correction = -offset[active]
                    word = words[active][0 if correction >= 0 else 1]
                    pose_status.text = f"Mover hacia {word.upper()}"
                    pose_status.text_color = gui.Color(0.72, 0.25, 0.12, 1.0)
                direction_words = (("derecha", "izquierda"), ("abajo", "arriba"), ("adelante", "atrás"))
                for i, key in enumerate(("x", "y", "z")):
                    correction = -offset[i]
                    word = direction_words[i][0 if correction >= 0 else 1]
                    pose_labels[key].text = f"{word} {abs(correction) * 100:.1f} cm"
                for key, value in zip(("roll", "pitch", "yaw"), rotation_error):
                    pose_labels[key].text = f"{value:+.1f}°"
                pose_total.text = f"Distancia total: {distance * 100:.1f} cm"
                pose_last_mtime = mtime
                scene_widget.force_redraw()
                return True
            except FileNotFoundError:
                return False
            except Exception:
                logger.exception("No se pudo actualizar la guia de pose IMU")
                return False

        def refresh_live_frame() -> bool:
            nonlocal live_added, last_live_mtime_ns, last_live_poll, live_camera_initialized
            now = time.monotonic()
            if now - last_live_poll < 0.08:
                return False
            last_live_poll = now
            try:
                mtime_ns = paths[0].stat().st_mtime_ns
                if mtime_ns == last_live_mtime_ns:
                    return False
                with np.load(paths[0], allow_pickle=False) as frame:
                    points = np.asarray(frame["points"], dtype=np.float64)
                    colors = np.asarray(frame["colors"], dtype=np.float64)
                if points.ndim != 2 or points.shape[1] != 3:
                    return False
                if len(points) == 0:
                    live_added = _replace_live_geometry(scene_widget.scene, points, colors, geometries[0][2], live_added)
                    live_status.text = "En vivo · sin puntos recientes"
                    live_count_history.clear()
                    last_live_mtime_ns = mtime_ns
                    scene_widget.force_redraw()
                    return True
                live_added = _replace_live_geometry(scene_widget.scene, points, colors, geometries[0][2], live_added)
                live_count_history.append(len(points))
                del live_count_history[:-5]
                count_estimate = int(np.median(live_count_history))
                live_status.text = f"En vivo · aprox. {round(count_estimate / 1000):,} mil puntos"
                if not live_camera_initialized:
                    bounds = scene_widget.scene.bounding_box
                    center = bounds.get_center()
                    scene_widget.setup_camera(60.0, bounds, center)
                    _look_at_sensor_pov(scene_widget, points, forward_axis=args.forward_axis)
                    live_camera_initialized = True
                last_live_mtime_ns = mtime_ns
                scene_widget.force_redraw()
                return True
            except FileNotFoundError:
                return False
            except Exception:
                logger.exception("No se pudo actualizar el frame del sensor en vivo")
                return False

        def on_tick() -> bool:
            nonlocal preview_frames
            changed = False
            if args.kind == "pose":
                changed = refresh_pose()
            if args.kind == "live":
                changed = refresh_live_frame()
                if preview_widget is None or not preview_widget.visible:
                    return changed
            if preview_widget is None or not preview_widget.visible:
                return changed
            preview_frames += 1
            if preview_frames >= 12:
                preview_widget.visible = False
            # Keep drawing the SceneWidget underneath the prepared image until
            # its first interactive frames have reached the visible window.
            return True

        window.set_on_tick_event(on_tick)
        if preview_widget is not None and args.kind == "pick":
            preview_widget.set_on_mouse(on_mouse)
        scene_widget.set_on_mouse(on_mouse)
        scene_widget.force_redraw()
        window.show(True)
        app.run()
    finally:
        if args.exit_status_file:
            try:
                Path(args.exit_status_file).write_text(exit_reason or "unexpected", encoding="utf-8")
            except OSError:
                logger.exception("No se pudo guardar el motivo de salida del visor")
        if args.cleanup_dir and not os.environ.get("AURORA_VIEWER_SUPERVISED"):
            shutil.rmtree(args.cleanup_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
