"""Portable sector sessions. C2C results remain exploratory, never certified thickness."""
from __future__ import annotations

import copy
import hashlib
import json
import shutil
import uuid
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from pointcloud_core import PipelineParams, crop_cloud_by_quad_box, load_point_cloud, load_reference_pose, run_pipeline


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def digest(path: Path) -> str:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def cloud_diagnostics(cloud) -> dict:
    points = np.asarray(cloud.points)
    if len(points) < 3 or not np.isfinite(points).all():
        raise ValueError("La nube necesita al menos 3 puntos finitos; no se descartan datos invalidos en silencio.")
    if np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
        raise ValueError("La nube no contiene una superficie: puntos coincidentes o colineales.")
    spacing = np.asarray(cloud.compute_nearest_neighbor_distance())
    return {
        "points": len(points), "min_m": points.min(axis=0).tolist(),
        "max_m": points.max(axis=0).tolist(),
        "spacing_median_m": float(np.median(spacing)),
        "spacing_p95_m": float(np.percentile(spacing, 95)),
        "colors": bool(cloud.has_colors()),
    }


class MeasurementSession:
    VERSION = 1

    def __init__(self, path: Path, data: dict):
        self.path, self.data = path.resolve(), data

    @classmethod
    def create(cls, directory: Path, tunnel: str, sector: str, demo: bool):
        if not tunnel.strip() or not sector.strip():
            raise ValueError("Identifica el tunel y el sector antes de crear la sesion.")
        identifier = uuid.uuid4().hex
        session = cls(directory / identifier / "sesion.json", {
            "version": cls.VERSION, "id": identifier, "tunnel": tunnel.strip(),
            "sector": sector.strip(), "demo": bool(demo), "created_at": now(),
            "base": None, "stages": [], "results": [], "roi": None,
        })
        session.save()
        return session

    @classmethod
    def load(cls, path: Path):
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("version") != cls.VERSION:
            raise ValueError("Version de sesion no compatible.")
        for field in ("id", "tunnel", "sector", "demo", "created_at", "base", "stages", "results"):
            if field not in data:
                raise ValueError(f"Sesion incompleta: falta {field}.")
        session = cls(path, data)
        for capture in session.captures():
            session.verify(capture)
        for result in data["results"]:
            for artifact in result["artifacts"].values():
                if not session.resolve(artifact).is_file():
                    raise ValueError(f"Falta un resultado de la sesion: {artifact}")
        return session

    def resolve(self, relative: str) -> Path:
        path = (self.path.parent / relative).resolve()
        if not path.is_relative_to(self.path.parent):
            raise ValueError("La sesion referencia un archivo fuera de su carpeta.")
        return path

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        temporary.replace(self.path)

    def captures(self):
        return ([self.data["base"]] if self.data["base"] else []) + self.data["stages"]

    def verify(self, capture):
        for relative, expected in capture["files"].items():
            path = self.resolve(relative)
            if not path.is_file() or digest(path) != expected:
                raise ValueError(f"Captura ausente o modificada: {relative}")

    def add_capture(self, source: Path, base=False, pose=None, connection_id=None, settings=None):
        if base and self.data["base"]:
            raise ValueError("La BASE es fija. Crea otra sesion para cambiar de sector o referencia.")
        if not base and not self.data["base"]:
            raise ValueError("Primero registra la BASE del sector.")
        diagnostics = cloud_diagnostics(load_point_cloud(source))
        if pose is None:
            stored_pose = load_reference_pose(source)
            if stored_pose is not None:
                pose = {"position_m": stored_pose[0].tolist(), "rpy_deg": stored_pose[1].tolist(),
                        "sensor_timestamp_ns": None, "frame": "sidecar SLAM; extrinseca a camara no aplicada"}
        identifier = uuid.uuid4().hex
        relative = f"capturas/{identifier}.ply"
        destination = self.resolve(relative)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        files = {relative: digest(destination)}
        for suffix in ("_pose.npz", "_ref.npz"):
            sidecar = source.with_name(source.stem + suffix)
            if sidecar.is_file():
                copied = destination.with_name(destination.stem + suffix)
                shutil.copy2(sidecar, copied)
                files[copied.relative_to(self.path.parent).as_posix()] = digest(copied)
        capture = {
            "id": identifier, "path": relative, "source_name": source.name,
            "registered_at": now(), "pose": pose, "connection_id": connection_id,
            "settings": settings or {}, "diagnostics": diagnostics, "files": files,
            "registration": None,
        }
        before = copy.deepcopy(self.data)
        if base:
            self.data["base"] = capture
        else:
            self.data["stages"].append(capture)
        try:
            self.save()
        except Exception:
            self.data = before
            raise
        return capture

    def register_stage(self, capture, aligned_path: Path, base_points, moving_points, rotation, translation, rms):
        """Record manual unchanged anchors; never automatically trust ICP or a pose match."""
        if any(capture["id"] in (record["stage_id"], record["reference_id"]) for record in self.data["results"]):
            raise ValueError("Esta etapa ya tiene resultados. Agrega otra captura para conservar su trazabilidad.")
        base_points, moving_points = np.asarray(base_points), np.asarray(moving_points)
        if len(base_points) < 3 or base_points.shape != moving_points.shape:
            raise ValueError("Se necesitan al menos 3 correspondencias en el mismo orden.")
        for points in (base_points, moving_points):
            if not np.isfinite(points).all() or np.linalg.matrix_rank(points - points.mean(axis=0)) < 2:
                raise ValueError("Las referencias deben ser finitas y no colineales.")
        self.verify(self.data["base"])
        self.verify(capture)
        cloud_diagnostics(load_point_cloud(aligned_path))
        relative = f"capturas/{capture['id']}_registro_{uuid.uuid4().hex}.ply"
        shutil.copy2(aligned_path, self.resolve(relative))
        before = copy.deepcopy(capture)
        capture["files"][relative] = digest(self.resolve(relative))
        capture["registration"] = {
            "method": "manual_unchanged_anchors", "base_id": self.data["base"]["id"],
            "path": relative, "base_points": base_points.tolist(), "moving_points": moving_points.tolist(),
            "rotation": np.asarray(rotation).tolist(), "translation": np.asarray(translation).tolist(),
            "residual_m": float(rms), "registered_at": now(),
            "quality": "Residual interno; precision y estabilidad fisica sin validacion de campo.",
        }
        try:
            self.save()
        except Exception:
            capture.clear()
            capture.update(before)
            raise

    def set_roi(self, quad, width_m):
        if self.data["results"]:
            raise ValueError("La zona es fija despues de comparar. Crea otra sesion para cambiarla.")
        quad = np.asarray(quad)
        if quad.shape != (4, 3) or not np.isfinite(quad).all() or not np.isfinite(width_m) or width_m <= 0:
            raise ValueError("La zona requiere 4 puntos finitos y ancho positivo.")
        cropped = crop_cloud_by_quad_box(load_point_cloud(self.resolve(self.data["base"]["path"])), quad, width_m)
        cloud_diagnostics(cropped)
        before = self.data.get("roi")
        self.data["roi"] = {"quad_base_frame_m": quad.tolist(), "width_m": float(width_m)}
        try:
            self.save()
        except Exception:
            self.data["roi"] = before
            raise

    def compare(self, stage_index: int, incremental=False, log=print, surface_reviewed=False):
        base = self.data["base"]
        current = self.data["stages"][stage_index]
        previous = self.data["stages"][stage_index - 1] if incremental and stage_index > 0 else base
        if incremental and stage_index == 0:
            raise ValueError("La primera etapa solo se puede comparar con BASE.")
        for capture in (base, previous, current):
            self.verify(capture)
        if not self.data["demo"]:
            for capture in (previous, current):
                if capture is base:
                    continue
                registration = capture["registration"]
                if not registration or registration["base_id"] != base["id"]:
                    raise ValueError("Faltan referencias estables para esta etapa. Usa Alinear etapa. La coincidencia IMU no verifica el registro.")
            if not surface_reviewed:
                raise ValueError("Falta revisar visualmente que el par cubra la misma zona y no incluya maquinaria u oclusiones. Esa revision es manual, no una prueba automatica de cobertura.")
        def working_path(capture):
            registration = capture["registration"]
            return self.resolve(registration["path"] if registration else capture["path"])
        mode = "entre_etapas" if incremental else "respecto_base"
        # Unique run dirs preserve all prior artifacts and avoid collisions between sectors/modes.
        directory = self.path.parent / "resultados" / current["id"] / mode / uuid.uuid4().hex
        first_path, second_path = working_path(previous), working_path(current)
        roi = self.data.get("roi")
        if roi:
            import open3d as o3d
            directory.mkdir(parents=True, exist_ok=True)
            paths = []
            for name, source in (("referencia_roi", first_path), ("etapa_roi", second_path)):
                cropped = crop_cloud_by_quad_box(load_point_cloud(source), np.asarray(roi["quad_base_frame_m"]), roi["width_m"])
                cloud_diagnostics(cropped)
                destination = directory / f"{name}.ply"
                if not o3d.io.write_point_cloud(str(destination), cropped):
                    raise OSError(f"No se pudo guardar el sector recortado: {destination}")
                paths.append(destination)
            first_path, second_path = paths
        result = run_pipeline(PipelineParams(first_path, second_path, directory), log=log)
        first = cloud_diagnostics(result.base_cloud)
        second = cloud_diagnostics(result.updated_cloud)
        reverse = np.asarray(result.base_cloud.compute_point_cloud_distance(result.updated_cloud))
        span = np.minimum(first["max_m"], second["max_m"]) - np.maximum(first["min_m"], second["min_m"])
        diagnostic = {
            "reference": first, "current": second,
            "reverse_c2c_median_m": float(np.median(reverse)),
            "reverse_c2c_p95_m": float(np.percentile(reverse, 95)),
            "shared_bbox_span_m": np.maximum(span, 0).tolist(),
            "coverage": "No cuantificada: los limites y C2C no prueban visibilidad ni solape de superficie.",
        }
        record = {
            "stage_id": current["id"], "reference_id": previous["id"], "mode": mode,
            "created_at": now(), "status": "ensayo" if self.data["demo"] else "exploratorio_no_validado",
            "method": "C2C euclidiana sin signo; no es espesor normal certificado",
            "surface_review": {"operator_confirmed": bool(surface_reviewed), "automated_coverage_validated": False},
            "roi": copy.deepcopy(roi),
            "stats_m": {key: getattr(result.stats, key) for key in ("mean", "median", "std", "min", "max", "p95", "n_points")},
            "diagnostics": diagnostic,
            "artifacts": {name: path.relative_to(self.path.parent).as_posix() for name, path in {
                "csv": result.csv_path, "histogram": result.histogram_path, "heatmap": result.heatmap_path,
            }.items()},
        }
        return record, result

    def record_result(self, record):
        self.data["results"].append(record)
        try:
            self.save()
        except Exception:
            self.data["results"].pop()
            raise
        self.export_report()

    def export_report(self) -> Path:
        import csv
        path = self.path.parent / "informe.md"
        lines = ["# Aurora — informe exploratorio", f"Tunel: {self.data['tunnel']}", f"Sector: {self.data['sector']}",
                 f"Sesion: {self.data['id']}", f"Creada: {self.data['created_at']}",
                 "ENSAYO CON DATOS DE PRUEBA" if self.data["demo"] else "TERRENO: RESULTADOS SIN CERTIFICACION",
                 "", "Distancias C2C sin signo: incluyen muestreo, rugosidad, registro y oclusiones. No prueban espesor normal ni falta/exceso de shotcrete.",
                 "No se resta la media acumulada para calcular una capa: entre etapas se compara directamente el par de nubes.", "",
                 f"Zona comun: {self.data.get('roi') or 'nubes completas; cobertura del sector sin cuantificar'}", "",
                 "## Capturas"]
        for index, capture in enumerate(self.captures()):
            lines += [f"- {'BASE' if index == 0 else 'Etapa ' + str(index)}: {capture['source_name']} ({capture['path']}), {capture['registered_at']}",
                      f"  Puntos: {capture['diagnostics']['points']}; pose: {capture['pose']}; conexion: {capture['connection_id']}"]
            if capture["registration"]:
                lines.append(f"  Referencias manuales; residual: {capture['registration']['residual_m'] * 1000:.3f} mm. No es precision de campo.")
        lines += ["", "## Comparaciones"]
        with (self.path.parent / "resumen.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["etapa", "referencia", "modo", "estado", "media_cm", "mediana_cm", "p95_cm", "puntos", "fecha", "csv"])
            for record in self.data["results"]:
                stats = record["stats_m"]
                writer.writerow([record["stage_id"], record["reference_id"], record["mode"], record["status"],
                                 stats["mean"] * 100, stats["median"] * 100, stats["p95"] * 100, stats["n_points"], record["created_at"], record["artifacts"]["csv"]])
                lines += [f"- {record['mode']}: {record['reference_id']} → {record['stage_id']}",
                          f"  Media C2C {stats['mean'] * 100:.3f} cm; mediana {stats['median'] * 100:.3f} cm; P95 {stats['p95'] * 100:.3f} cm. Estado: {record['status']}.",
                          f"  Revision manual de zona: {record.get('surface_review')}; no valida cobertura automatica.",
                          f"  Diagnosticos: {json.dumps(record['diagnostics'], ensure_ascii=False)}", f"  CSV: {record['artifacts']['csv']}"]
        path.write_text("\n\n".join(lines) + "\n", encoding="utf-8")
        return path
