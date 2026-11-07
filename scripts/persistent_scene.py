"""GPU point scene embedded in GTK, with RViz-like orbit and session markers."""

import ctypes
import math

import gi
gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, GLib, Gtk
import numpy as np
from OpenGL import GL
from OpenGL.GL.shaders import compileProgram, compileShader
from OpenGL.raw.GL.VERSION.GL_2_0 import glVertexAttribPointer

from scene_feed import SceneFeed, scene_view


def _vertices(points, color):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    return np.column_stack((points, np.tile(color, (len(points), 1)))).astype(np.float32)


def _axes(position, rotation, size):
    lines = []
    for i, color in enumerate(((1, .2, .2), (.2, .9, .3), (.25, .5, 1))):
        lines.append(_vertices([position, position + rotation[:, i] * size], color))
    return np.concatenate(lines)


class SceneCanvas(Gtk.GLArea):
    def __init__(self, on_error):
        super().__init__()
        self.set_required_version(3, 3)
        self.set_has_depth_buffer(True)
        self.set_auto_render(False)
        self.set_hexpand(True)
        self.set_vexpand(True)
        self.set_size_request(320, 280)
        self.program = self.vao = self.vbo = None
        self.cloud_vao = self.cloud_vbo = None
        self._uploaded_cloud = None
        self.on_error = on_error
        self.state = SceneFeed._empty_state("")
        self.center = np.zeros(3)
        self.distance, self.yaw, self.elevation = 8.0, -1.0, .55
        self.drag = None
        self.layers = dict(cloud=True, grid=True, sensor=True, path=True)
        self.point_size = 2.0
        self.follow = False
        self.fitted_session = None
        self.set_events(self.get_events() | Gdk.EventMask.BUTTON_PRESS_MASK
                        | Gdk.EventMask.BUTTON_RELEASE_MASK | Gdk.EventMask.POINTER_MOTION_MASK
                        | Gdk.EventMask.SCROLL_MASK | Gdk.EventMask.SMOOTH_SCROLL_MASK)
        self.connect("realize", self._realize)
        self.connect("unrealize", self._unrealize)
        self.connect("render", self._render)
        self.connect("button-press-event", self._press)
        self.connect("button-release-event", lambda *_: self._release())
        self.connect("motion-notify-event", self._motion)
        self.connect("scroll-event", self._scroll)

    def _realize(self, *_):
        self.make_current()
        if self.get_error():
            self.on_error(str(self.get_error()))
            return
        try:
            self.program = compileProgram(
                compileShader("""#version 330 core
                layout(location=0) in vec3 position;
                layout(location=1) in vec3 color;
                uniform mat4 mvp;
                uniform float point_size;
                out vec3 vertex_color;
                void main(){ gl_Position=mvp*vec4(position,1.0);
                             gl_PointSize=point_size; vertex_color=color; }
                """, GL.GL_VERTEX_SHADER),
                compileShader("""#version 330 core
                in vec3 vertex_color; out vec4 fragment;
                void main(){ fragment=vec4(vertex_color,1.0); }
                """, GL.GL_FRAGMENT_SHADER),
            )
            self.vao = GL.glGenVertexArrays(1)
            self.vbo = GL.glGenBuffers(1)
            self.cloud_vao = GL.glGenVertexArrays(1)
            self.cloud_vbo = GL.glGenBuffers(1)
            for vao, vbo in ((self.vao, self.vbo), (self.cloud_vao, self.cloud_vbo)):
                GL.glBindVertexArray(vao)
                GL.glBindBuffer(GL.GL_ARRAY_BUFFER, vbo)
                for index, offset in ((0, 0), (1, 12)):
                    GL.glEnableVertexAttribArray(index)
                    glVertexAttribPointer(index, 3, GL.GL_FLOAT, False, 24, ctypes.c_void_p(offset))
            self._uploaded_cloud = None
            GL.glBindVertexArray(0)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            self.on_error(f"No se pudo iniciar el visor 3D: {exc}")
            self.program = None

    def _unrealize(self, *_):
        self.make_current()
        if not self.get_error() and self.program is not None:
            GL.glDeleteBuffers(1, [self.vbo])
            GL.glDeleteVertexArrays(1, [self.vao])
            GL.glDeleteBuffers(1, [self.cloud_vbo])
            GL.glDeleteVertexArrays(1, [self.cloud_vao])
            GL.glDeleteProgram(self.program)
        self.program = None

    def _matrix(self, aspect):
        direction = np.array([math.cos(self.elevation) * math.cos(self.yaw),
                              math.cos(self.elevation) * math.sin(self.yaw),
                              math.sin(self.elevation)])
        eye = self.center + direction * self.distance
        forward = (self.center - eye) / self.distance
        right = np.cross(forward, [0., 0., 1.])
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        view = np.eye(4)
        view[:3, :3] = [right, up, -forward]
        view[:3, 3] = -view[:3, :3] @ eye
        f, near, far = 1 / math.tan(math.radians(55) / 2), .01, max(1000., self.distance * 20)
        projection = np.zeros((4, 4))
        projection[0, 0], projection[1, 1] = f / aspect, f
        projection[2, 2] = (far + near) / (near - far)
        projection[2, 3] = 2 * far * near / (near - far)
        projection[3, 2] = -1
        return np.asarray(projection @ view, dtype=np.float32)

    def _draw_vertices(self, vertices, mode, size=2):
        if not len(vertices):
            return
        data = np.ascontiguousarray(vertices, dtype=np.float32)
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.vbo)
        GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_DYNAMIC_DRAW)
        GL.glUniform1f(GL.glGetUniformLocation(self.program, "point_size"), size)
        GL.glDrawArrays(mode, 0, len(data))

    def _draw_cloud(self, points, colors, size):
        GL.glBindVertexArray(self.cloud_vao)
        key = (id(points), id(colors))
        if key != self._uploaded_cloud:
            data = np.empty((len(points), 6), dtype=np.float32)
            data[:, :3], data[:, 3:] = points, colors
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.cloud_vbo)
            GL.glBufferData(GL.GL_ARRAY_BUFFER, data.nbytes, data, GL.GL_STREAM_DRAW)
            self._uploaded_cloud = key
        GL.glUniform1f(GL.glGetUniformLocation(self.program, "point_size"), size)
        GL.glDrawArrays(GL.GL_POINTS, 0, len(points))
        GL.glBindVertexArray(self.vao)

    def _render(self, *_):
        GL.glClearColor(48 / 255, 48 / 255, 48 / 255, 1.)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        if self.program is None:
            return True
        scale = self.get_scale_factor()
        width, height = self.get_allocated_width(), max(1, self.get_allocated_height())
        GL.glViewport(0, 0, width * scale, height * scale)
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glDepthFunc(GL.GL_LEQUAL)
        GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
        GL.glUseProgram(self.program)
        GL.glBindVertexArray(self.vao)
        GL.glUniformMatrix4fv(GL.glGetUniformLocation(self.program, "mvp"), 1, True, self._matrix(width / height))
        state = self.state
        if self.layers["grid"]:
            grid = []
            extent = max(10, min(100, math.ceil(self.distance)))
            cx, cy = np.round(self.center[:2]).astype(int)
            for offset in range(-extent, extent + 1):
                grid.extend(((cx + offset, cy - extent, 0), (cx + offset, cy + extent, 0),
                             (cx - extent, cy + offset, 0), (cx + extent, cy + offset, 0)))
            self._draw_vertices(_vertices(grid, (.36, .36, .38)), GL.GL_LINES)
            self._draw_vertices(_axes(np.zeros(3), np.eye(3), 1), GL.GL_LINES)
        if self.layers["cloud"] and len(state["points"]):
            self._draw_cloud(state["points"], state["colors"], self.point_size * scale)
        if self.layers["path"] and len(state["path"]) > 1:
            self._draw_vertices(_vertices(state["path"], (.8, .5, 1.)), GL.GL_LINE_STRIP)
        if self.layers["sensor"]:
            if state["reference"] is not None:
                self._draw_vertices(_axes(state["reference"], state["reference_rotation"], .6), GL.GL_LINES)
                self._draw_vertices(_vertices([state["reference"]], (1., 1., 1.)), GL.GL_POINTS, 30 * scale)
            p = state["position"]
            if p is None and not state["registered"]:
                p = np.zeros(3)
            if p is not None:
                if state["reference"] is not None:
                    self._draw_vertices(_vertices([state["reference"], p], (.85, .55, 1.)), GL.GL_LINES)
                self._draw_vertices(_vertices([p, p + state["rotation"][:, 0] * .6], (1., .2, .2)), GL.GL_LINES)
                color = (1, .8, .1) if state["pose_ok"] else (.6, .6, .6)
                self._draw_vertices(_vertices([p], color), GL.GL_POINTS, 14 * scale)
        GL.glBindVertexArray(0)
        GL.glUseProgram(0)
        return True

    def set_state(self, state):
        self.state = state
        if self.follow and state["position"] is not None:
            self.center = state["position"].copy()
        identity = (state["registered"], state["session"], state["reference"] is not None)
        if len(state["points"]) and identity != self.fitted_session:
            self.fit()
            self.fitted_session = identity
        self.queue_render()

    def fit(self):
        points = self.state["points"]
        if len(points):
            lo, hi = np.percentile(points, [5, 95], axis=0)
            self.center = (lo + hi) / 2
            # Fit the visible point mass to this viewport, without distant
            # returns making the useful scene look tiny at every opening.
            direction = np.array([math.cos(self.elevation) * math.cos(self.yaw),
                                  math.cos(self.elevation) * math.sin(self.yaw),
                                  math.sin(self.elevation)])
            right = np.cross(-direction, [0., 0., 1.])
            right /= np.linalg.norm(right)
            up = np.cross(right, -direction)
            offsets = points - self.center
            aspect = max(1, self.get_allocated_width()) / max(1, self.get_allocated_height())
            tangent = math.tan(math.radians(55) / 2)
            required_distance = np.einsum("ij,j->i", offsets, direction, optimize=False) + np.maximum(
                np.abs(np.einsum("ij,j->i", offsets, right, optimize=False)) / (tangent * aspect),
                np.abs(np.einsum("ij,j->i", offsets, up, optimize=False)) / tangent,
            )
            self.distance = max(.5, float(np.percentile(required_distance, 95)) * 1.05)
        else:
            self.center = np.zeros(3)
            self.distance = 8.
        self.queue_render()

    def view(self, name):
        self.yaw, self.elevation = {"Órbita": (-1., .55), "Superior": (-math.pi / 2, math.pi / 2 - .001),
                                   "Frontal": (math.pi, 0.), "Lateral": (-math.pi / 2, 0.)}[name]
        self.queue_render()

    def center_sensor(self):
        position = self.state["position"]
        self.center = np.zeros(3) if position is None else position.copy()
        self.queue_render()

    def zoom(self, factor):
        self.distance = float(np.clip(self.distance * factor, .1, 1000))
        self.queue_render()

    def _press(self, _widget, event):
        self.drag = (event.x, event.y, event.button)
        return True

    def _release(self):
        self.drag = None
        return True

    def _motion(self, _widget, event):
        if self.drag is None:
            return False
        x, y, button = self.drag
        dx, dy = event.x - x, event.y - y
        self.drag = (event.x, event.y, button)
        if button == 2 or event.state & Gdk.ModifierType.SHIFT_MASK:
            right = np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.])
            up = np.array([-math.sin(self.elevation) * math.cos(self.yaw),
                           -math.sin(self.elevation) * math.sin(self.yaw), math.cos(self.elevation)])
            self.center += (-right * dx + up * dy) * self.distance / max(1, self.get_allocated_height())
        else:
            self.yaw -= dx * .008
            self.elevation = float(np.clip(self.elevation + dy * .008, -1.56, 1.56))
        self.queue_render()
        return True

    def _scroll(self, _widget, event):
        if event.direction == Gdk.ScrollDirection.SMOOTH:
            _, _, delta = event.get_scroll_deltas()
        else:
            delta = -1 if event.direction == Gdk.ScrollDirection.UP else 1
        self.zoom(math.exp(float(delta) * .12))
        return True


class PersistentScenePanel(Gtk.Box):
    def __init__(self, on_decay_changed=None):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.set_name("persistent-scene")
        self.set_size_request(360, -1)
        self.set_margin_start(8)
        self.set_margin_end(8)
        self.set_margin_top(12)
        self.set_margin_bottom(12)
        self.feed = SceneFeed()
        self.error = None
        self.on_decay_changed = on_decay_changed
        self.capture_settings = {}
        self.paused_state = None
        self._last_source_state = None
        self._view_key = None
        self._view_geometry = None
        title = Gtk.Label(label="ENTORNO 3D", xalign=0)
        title.set_tooltip_text("Entorno completo para orientarte. Las capturas conservan sus filtros configurados.")
        title.get_style_context().add_class("section-title")
        self.pack_start(title, False, False, 0)
        self.status = Gtk.Label(label="Conecta el sensor para ver el entorno", xalign=0)
        self.status.set_line_wrap(True)
        self.status.set_max_width_chars(80)
        self.pack_start(self.status, False, False, 0)
        self.canvas = SceneCanvas(self._on_error)
        self.pack_start(self.canvas, True, True, 0)
        row = Gtk.Box(spacing=6)
        row.pack_start(Gtk.Label(label="Decay (s):"), False, False, 0)
        self.decay_spin = Gtk.SpinButton.new_with_range(0, 60, .5)
        self.decay_spin.set_value(10)
        self.decay_spin.set_digits(1)
        self.decay_spin.set_tooltip_text("Segundos de historia visual. 0 muestra la última nube. No cambia la duración de captura.")
        self.decay_spin.connect("value-changed", self._decay_changed)
        row.pack_start(self.decay_spin, False, False, 0)
        row.pack_start(Gtk.Label(label="Punto (px):"), False, False, 0)
        self.point_spin = Gtk.SpinButton.new_with_range(.5, 12, .5)
        self.point_spin.set_value(2)
        self.point_spin.set_digits(1)
        self.point_spin.connect("value-changed", lambda b: self._point_size(b.get_value()))
        row.pack_start(self.point_spin, False, False, 0)
        self.color_combo = Gtk.ComboBoxText()
        for key, text in (("intensity", "Intensidad"), ("height", "Altura Z"), ("distance", "Distancia al sensor")):
            self.color_combo.append(key, text)
        self.color_combo.set_active_id("intensity")
        self.color_combo.connect("changed", lambda *_: self._refresh())
        row.pack_start(self.color_combo, True, True, 0)
        self.pack_start(row, False, False, 0)
        row = Gtk.Box(spacing=6)
        self.pause_button = Gtk.ToggleButton(label="Pausar vista")
        self.pause_button.set_tooltip_text("Congela la imagen para inspeccionarla; la recepción y las capturas continúan.")
        self.pause_button.connect("toggled", self._pause)
        row.pack_start(self.pause_button, True, True, 0)
        self.capture_toggle = Gtk.ToggleButton(label="Vista completa")
        self.capture_toggle.set_tooltip_text("Alterna el entorno completo y la zona que capturarán ANTES y DESPUÉS.")
        self.capture_toggle.connect("toggled", lambda *_: self._refresh())
        row.pack_start(self.capture_toggle, True, True, 0)
        self.clear_button = Gtk.Button(label="Limpiar nube")
        self.clear_button.set_tooltip_text("Vacía la historia visual; conserva las capturas, el inicio y el seguimiento.")
        self.clear_button.connect("clicked", self._clear)
        row.pack_start(self.clear_button, True, True, 0)
        self.pack_start(row, False, False, 0)
        expander = Gtk.Expander(label="Rango de color (rojo: menor · magenta: mayor)")
        row = Gtk.Box(spacing=6)
        self.auto_color = Gtk.CheckButton(label="Automático")
        self.auto_color.set_active(True)
        row.pack_start(self.auto_color, False, False, 0)
        self.color_min = Gtk.SpinButton.new_with_range(-100000, 100000, .1)
        self.color_min.set_value(0)
        self.color_max = Gtk.SpinButton.new_with_range(-100000, 100000, .1)
        self.color_max.set_value(255)
        for label, spin in (("Mín:", self.color_min), ("Máx:", self.color_max)):
            spin.set_digits(1)
            spin.set_sensitive(False)
            spin.connect("value-changed", lambda *_: self._refresh())
            row.pack_start(Gtk.Label(label=label), False, False, 0)
            row.pack_start(spin, False, False, 0)
        self.auto_color.connect("toggled", self._auto_color_changed)
        expander.add(row)
        self.pack_start(expander, False, False, 0)
        self.color_label = Gtk.Label(xalign=0)
        self.pack_start(self.color_label, False, False, 0)
        row = Gtk.Box(spacing=8)
        for key, label in (("cloud", "Nube"), ("grid", "Cuadrícula"), ("sensor", "Sensor"), ("path", "Recorrido")):
            check = Gtk.CheckButton(label=label)
            check.set_active(True)
            check.connect("toggled", lambda b, layer=key: self._layer(layer, b.get_active()))
            row.pack_start(check, False, False, 0)
        self.pack_start(row, False, False, 0)
        self.follow = Gtk.CheckButton(label="Seguir sensor con la cámara")
        self.follow.connect("toggled", lambda b: setattr(self.canvas, "follow", b.get_active()))
        self.pack_start(self.follow, False, False, 0)
        self.position_label = Gtk.Label(label="Captura por primera vez para fijar el inicio de la sesión.", xalign=0)
        self.position_label.set_line_wrap(True)
        self.position_label.set_max_width_chars(48)
        self.pack_start(self.position_label, False, False, 0)
        hint = Gtk.Label(label="Blanco: inicio · amarillo: sensor · violeta: desplazamiento\nArrastrar: orbitar · central/Shift: desplazar · rueda: zoom", xalign=0)
        hint.set_line_wrap(True)
        hint.get_style_context().add_class("dim-label")
        self.pack_start(hint, False, False, 0)
        self.timer = GLib.timeout_add(180, self._refresh)
        self.connect("destroy", lambda *_: self.stop())

    def _on_error(self, text):
        self.error = text
        self.status.set_text(text)

    def _layer(self, key, enabled):
        self.canvas.layers[key] = enabled
        self.canvas.queue_render()

    def set_decay_time(self, seconds):
        self.feed.set_decay(seconds)
        if self.decay_spin.get_value() != seconds:
            self.decay_spin.set_value(seconds)

    def _decay_changed(self, spin):
        self.feed.set_decay(spin.get_value())
        if self.on_decay_changed is not None:
            self.on_decay_changed(spin.get_value())

    def set_capture_settings(self, settings):
        self.capture_settings = dict(settings)
        self._view_key = None

    def _point_size(self, size):
        self.canvas.point_size = size
        self.canvas.queue_render()

    def _pause(self, button):
        self.paused_state = self._last_source_state if button.get_active() else None
        button.set_label("Reanudar vista" if button.get_active() else "Pausar vista")
        self._refresh()

    def _clear(self, *_):
        self.feed.clear_history()
        self.pause_button.set_active(False)
        self.paused_state = None
        self._view_key = None
        self._refresh()

    def _auto_color_changed(self, button):
        self.color_min.set_sensitive(not button.get_active())
        self.color_max.set_sensitive(not button.get_active())
        self._refresh()

    def _refresh(self):
        state = self.paused_state if self.paused_state is not None else self.feed.snapshot(blocking=False)
        if state is None:
            return True
        self._last_source_state = state
        mode = self.color_combo.get_active_id() or "intensity"
        bounds = None if self.auto_color.get_active() else (self.color_min.get_value(), self.color_max.get_value())
        if bounds is not None and bounds[1] <= bounds[0]:
            self.color_label.set_text("El máximo de color debe ser mayor que el mínimo.")
            return True
        settings = self.capture_settings if self.capture_toggle.get_active() else None
        self.capture_toggle.set_label(self.capture_settings.get("label", "Zona de captura")
                                      if self.capture_toggle.get_active() else "Vista completa")
        position_key = state["position"].tobytes() if state["position"] is not None else None
        key = (id(state["points"]), id(state.get("intensity")), position_key,
               state["rotation"].tobytes(), mode, bounds, str(settings))
        if key != self._view_key:
            self._view_geometry = scene_view(state, mode, settings, bounds)
            self._view_key = key
        view = {**state, "points": self._view_geometry["points"], "colors": self._view_geometry["colors"]}
        self.canvas.set_state(view)
        count = f"{len(state['points']):,}".replace(",", ".")
        visible = f"{len(view['points']):,}".replace(",", ".")
        count_label = f"{visible} de {count}" if len(view["points"]) != len(state["points"]) else count
        rate = state.get("rate_hz", 0.)
        prefix = "Vista pausada" if self.paused_state is not None else state["status"]
        self.status.set_text(self.error or f"{prefix}\n{count_label} puntos · {rate:.1f} nubes/s")
        self.status.set_tooltip_text("Hasta 300.000 puntos dibujados, distribuidos por toda la historia del decay. Las capturas y los cálculos conservan sus datos completos.")
        limits = self._view_geometry["color_limits"]
        unit = "u." if mode == "intensity" else "m"
        title = {"intensity": "Intensidad", "height": "Altura Z", "distance": "Distancia al sensor"}[mode]
        self.color_label.set_text(f"{title}: {limits[0]:.1f} a {limits[1]:.1f} {unit}" if limits else "Sin puntos para colorear")
        if state["reference"] is None:
            if state["pose_ok"] and state["position"] is not None:
                distance = float(np.linalg.norm(state["position"])) * 100
                text = (f"Desde esta apertura: {0.0 if distance < 1.5 else distance:.1f} cm · sin referencia de captura.\n"
                        "Captura ANTES para fijar el inicio de la medición.")
            else:
                text = "Sin referencia de captura. Captura ANTES para fijar el inicio de la medición."
        elif not state["pose_ok"] or state["position"] is None:
            text = "Inicio guardado · esperando una pose reciente de la misma sesión."
        else:
            position = state["position"].copy()
            position[np.abs(position) < .01] = 0
            text = (f"Desde la primera captura: X {position[0] * 100:+.1f} cm · "
                    f"Y {position[1] * 100:+.1f} cm · Z {position[2] * 100:+.1f} cm\n"
                    f"Distancia al inicio: {np.linalg.norm(position) * 100:.1f} cm")
        self.position_label.set_text(text)
        return True

    def stop(self):
        if self.timer is not None:
            GLib.source_remove(self.timer)
            self.timer = None
        self.feed.stop()
