"""Render each GTK page and validate callback method references without sensor calls."""
import argparse
import ast
import json
import time
from pathlib import Path
import gi
gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gtk
import gui_gtk


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path("output/auditoria/gui"))
    parser.add_argument("--session", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    Gtk.Settings.get_default().set_property("gtk-application-prefer-dark-theme", True)
    provider = Gtk.CssProvider()
    provider.load_from_data(gui_gtk.CSS)
    Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    app = gui_gtk.AuroraGUI(Gtk.Window())
    app.window.resize(1280, 960)
    app.window.show_all()
    app.quick_demo_check.set_active(True)
    if args.session:
        app._session_load(args.session)
    methods = set()
    for filename in ("gui_gtk.py", "sector_workflow.py"):
        tree = ast.parse(Path(__file__).with_name(filename).read_text(encoding="utf-8-sig"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name) and node.func.value.id == "self" and node.func.attr.startswith("_"):
                methods.add(node.func.attr)
    missing = [name for name in methods if not hasattr(app, name)]
    assert not missing, missing
    pages = []
    for page in app._sidebar_rows:
        app.stack.set_visible_child_name(page)
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            while Gtk.events_pending():
                Gtk.main_iteration_do(False)
            time.sleep(.01)
        window = app.window.get_window()
        snapshot = Gdk.pixbuf_get_from_window(window, 0, 0, window.get_width(), window.get_height())
        if snapshot is not None:
            assert len(set(snapshot.get_pixels())) > 16, f"Window capture is blank on {page}; use X11 for screenshot verification."
            snapshot.savev(str(args.output_dir / f"{page}.png"), "png", [], [])
        pages.append({"page": page, "width": window.get_width(), "height": window.get_height(), "snapshot": snapshot is not None})
    (args.output_dir / "verification.json").write_text(json.dumps({"pages": pages, "callback_methods_checked": len(methods), "missing_methods": missing}, indent=2), encoding="utf-8")
    app._stop_imu_poll()
    app.window.disconnect_by_func(app._on_close)
    app.window.destroy()
    print(f"Rendered {len(pages)} GTK pages; {len(methods)} callback/helper references resolved.")


if __name__ == "__main__":
    main()
