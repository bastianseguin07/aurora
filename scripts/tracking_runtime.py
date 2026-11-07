"""Start or recover the project's Point-LIO process without touching the LiDAR driver."""

import os
from pathlib import Path
import signal
import subprocess
import time


def _processes():
    processes = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            command = (entry / "cmdline").read_bytes().decode().strip("\0").split("\0")
            stat = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            processes[int(entry.name)] = (int(stat[1]), command)
        except (OSError, ValueError, UnicodeError):
            continue
    return processes


def ensure_tracking(root, connection, log=print):
    root = Path(root)
    processes = _processes()
    launches = [pid for pid, (_, cmd) in processes.items()
                if "launch" in cmd and "point_lio" in cmd and "aurora_l2.launch.py" in cmd]
    # A newly connected subscriber needs a short discovery/initialization window.
    deadline = time.monotonic() + (3 if launches else 0)
    while time.monotonic() < deadline:
        if connection.stop.is_set():
            return None
        try:
            connection.pose_history.latest()
            return None
        except RuntimeError:
            time.sleep(.1)
    if connection.stop.is_set():
        return None
    for launch in launches:
        children = {launch}
        for _ in range(8):
            children |= {pid for pid, (parent, _) in processes.items() if parent in children}
        # Only recover launches whose children execute this workspace's binary.
        if not any(str(root / "build/ros2_lio_ws") in " ".join(processes[pid][1]) for pid in children):
            continue
        log("Recuperando seguimiento sin posición reciente; se creará una sesión nueva.")
        try:
            os.kill(launch, signal.SIGINT)
        except ProcessLookupError:
            continue
        deadline = time.monotonic() + 2
        while Path(f"/proc/{launch}").exists() and time.monotonic() < deadline:
            time.sleep(.1)
        for pid in children:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(.2)
        for pid in children:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    if connection.stop.is_set():
        return None
    with (root / "build/point_lio.log").open("a") as stream:
        process = subprocess.Popen([str(root / "run_unitree_lio.sh"), "rviz:=false"],
                                   cwd=root, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
    log("Preparando seguimiento del sensor.")
    return process
