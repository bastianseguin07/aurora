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

from pointcloud_core import load_point_cloud

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


def _read_geometries(kind: str, paths: list[Path]):
    material = rendering.MaterialRecord()
    material.shader = "defaultUnlit"
    material.point_size = 2.5

    if kind == "cloud":
        cloud = load_point_cloud(paths[0])
        if not cloud.has_colors():
            cloud.paint_uniform_color((0.7, 0.72, 0.76))
        return [("nube", cloud, material)]
    if kind == "pair":
        base = load_point_cloud(paths[0])
        updated = load_point_cloud(paths[1])
        base.paint_uniform_color((0.68, 0.72, 0.78))
        updated.paint_uniform_color((0.10, 0.72, 0.95))
        return [("antes", base, material), ("despues", updated, material)]
    if kind == "thickness":
        base = load_point_cloud(paths[0])
        heatmap = load_point_cloud(paths[1])
        base.paint_uniform_color((0.68, 0.72, 0.78))
        if not heatmap.has_colors():
            heatmap.paint_uniform_color((0.10, 0.72, 0.95))
        return [("antes", base, material), ("despues", heatmap, material)]

    mesh_material = rendering.MaterialRecord()
    mesh_material.shader = "defaultLit"
    base_mesh = o3d.io.read_triangle_mesh(str(paths[0]))
    updated_mesh = o3d.io.read_triangle_mesh(str(paths[1]))
    if base_mesh.is_empty() or updated_mesh.is_empty():
        raise RuntimeError("No se pudieron cargar las dos mallas para inspeccionarlas.")
    base_mesh.paint_uniform_color((0.55, 0.6, 0.65))
    updated_mesh.paint_uniform_color((1.0, 0.549, 0.0))
    base_mesh.compute_vertex_normals()
    updated_mesh.compute_vertex_normals()
    return [("antes", base_mesh, mesh_material), ("despues", updated_mesh, mesh_material)]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("kind", choices=("cloud", "pair", "thickness", "meshes", "pick"))
    parser.add_argument("paths", nargs="+")
    parser.add_argument("--title", default="Aurora - Vista 3D")
    parser.add_argument("--cleanup-dir")
    parser.add_argument("--legend-json")
    parser.add_argument("--selection-output")
    parser.add_argument("--pick-count", type=int, default=0)
    parser.add_argument("--pick-prompt", default="Clic izquierdo para marcar puntos; Q para terminar")
    parser.add_argument("--exit-status-file")
    args = parser.parse_args()
    exit_reason: str | None = None

    try:
        paths = [Path(path) for path in args.paths]
        geometries = _read_geometries("cloud" if args.kind == "pick" else args.kind, paths)

        app = gui.Application.instance
        app.initialize()
        # Keep the native window hidden until its scene and camera are ready.
        window = app.create_window(args.title, 1280, 800, -1, -1, 1)
        scene_widget = gui.SceneWidget()
        scene_widget.scene = rendering.Open3DScene(window.renderer)

        for name, geometry, material in geometries:
            scene_widget.scene.add_geometry(name, geometry, material)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        axes_material = rendering.MaterialRecord()
        axes_material.shader = "defaultUnlit"
        scene_widget.scene.add_geometry("ejes", axes, axes_material)

        legend = json.loads(args.legend_json) if args.legend_json else None
        legend_widget = _legend_panel(legend, window.theme.font_size) if legend else None
        pair_controls = None
        if args.kind in ("pair", "thickness", "meshes"):
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
            for toggle, label in ((before_toggle, "ANTES"), (after_toggle, "DESPUES")):
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
        pick_status = None
        picked_points: list[np.ndarray] = []
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

        window.set_on_layout(on_layout)
        window.add_child(scene_widget)

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
        warmup_image = app.render_to_image(
            scene_widget.scene,
            max(1, scene_widget.frame.width),
            max(1, scene_widget.frame.height),
        )
        preview_widget = gui.ImageWidget(warmup_image)
        preview_widget.frame = window.content_rect
        window.add_child(preview_widget)
        if pair_controls is not None:
            window.add_child(pair_controls)
        if legend_widget is not None:
            window.add_child(legend_widget)
        if pick_status is not None:
            window.add_child(pick_status)
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
            nonlocal drag_origin, drag_left_button, last_drag_x, last_drag_y
            left_down = event.is_button_down(gui.MouseButton.LEFT)
            shift_down = event.is_modifier_down(gui.KeyModifier.SHIFT)
            if event.type == gui.MouseEvent.Type.BUTTON_DOWN:
                drag_origin = (event.x, event.y)
                drag_left_button = left_down
                last_drag_x = event.x
                last_drag_y = event.y
                # Prevent a press/release click from reaching Open3D's default
                # camera handler. Camera motion is applied only from DRAG below.
                return gui.Widget.CONSUMED
            if event.type == gui.MouseEvent.Type.WHEEL and event.buttons != gui.MouseButton.NONE:
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
                drag_origin = None
                drag_left_button = False
                last_drag_x = None
                last_drag_y = None
                moved = math.hypot(event.x - start[0], event.y - start[1])
                # The landmark picker is a dedicated selection window, so a
                # left click always selects. Shift+click remains supported;
                # this also works on desktops that do not report Shift in the
                # Open3D mouse event reliably.
                if args.kind == "pick" and was_left_click and moved < 5 and (args.pick_count <= 0 or len(picked_points) < args.pick_count):
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
                        picked_points.append(point.copy())
                        marker_radius = max(float(world_per_pixel[best_index]) * 2.5, 0.003)
                        marker = o3d.geometry.TriangleMesh.create_sphere(radius=marker_radius)
                        marker.translate(point)
                        marker.paint_uniform_color((1.0, 0.25, 0.18))
                        marker_material = rendering.MaterialRecord()
                        marker_material.shader = "defaultUnlit"
                        scene_widget.scene.add_geometry(f"picked_{len(picked_points)}", marker, marker_material)
                        scene_widget.add_3d_label(point, str(len(picked_points)))
                        if pick_status is not None:
                            limit = f"/{args.pick_count}" if args.pick_count > 0 else ""
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
            return gui.Widget.IGNORED

        preview_frames = 0

        def on_tick() -> bool:
            nonlocal preview_frames
            if preview_widget is None or not preview_widget.visible:
                return False
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
