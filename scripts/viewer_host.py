"""Keep an Open3D viewer open if its native event loop exits unexpectedly."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
VIEWER = Path(__file__).resolve().with_name("viewer_window.py")
MAX_RAPID_RESTARTS = 8
STABLE_RUN_SECONDS = 10.0
RESTART_DELAY_SECONDS = 0.25


def main() -> int:
    viewer_args = sys.argv[1:]
    cleanup_dir = None
    if "--cleanup-dir" in viewer_args:
        index = viewer_args.index("--cleanup-dir")
        if index + 1 < len(viewer_args):
            cleanup_dir = Path(viewer_args[index + 1])

    rapid_restarts = 0
    while True:
        with tempfile.NamedTemporaryFile(prefix="aurora_viewer_exit_", delete=False) as marker_file:
            marker = Path(marker_file.name)
        marker.unlink(missing_ok=True)
        command = [
            sys.executable,
            str(VIEWER),
            *viewer_args,
            "--exit-status-file",
            str(marker),
        ]
        environment = os.environ.copy()
        environment["AURORA_VIEWER_SUPERVISED"] = "1"
        started_at = time.monotonic()
        try:
            result = subprocess.run(command, cwd=PROJECT_ROOT, env=environment, check=False)
            reason = marker.read_text(encoding="utf-8").strip() if marker.exists() else "unexpected"
        finally:
            marker.unlink(missing_ok=True)

        if reason in {"q", "window"}:
            if cleanup_dir is not None:
                shutil.rmtree(cleanup_dir, ignore_errors=True)
            return result.returncode

        if time.monotonic() - started_at >= STABLE_RUN_SECONDS:
            rapid_restarts = 0
        rapid_restarts += 1
        if rapid_restarts > MAX_RAPID_RESTARTS:
            print(
                "La vista 3D terminó repetidamente sin una orden de cierre; "
                "se detuvo el reinicio automático.",
                file=sys.stderr,
                flush=True,
            )
            if cleanup_dir is not None:
                shutil.rmtree(cleanup_dir, ignore_errors=True)
            return result.returncode or 1

        print(
            "La vista 3D terminó sin Q ni cierre de ventana; reabriendo.",
            file=sys.stderr,
            flush=True,
        )
        time.sleep(RESTART_DELAY_SECONDS)

        if cleanup_dir is not None and not cleanup_dir.exists():
            print(
                f"No se puede reabrir la vista; falta {cleanup_dir}.",
                file=sys.stderr,
                flush=True,
            )
            return result.returncode or 1


if __name__ == "__main__":
    raise SystemExit(main())
