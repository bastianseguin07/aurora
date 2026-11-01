"""Run the expensive raycasting pipeline outside the GTK process."""

from __future__ import annotations

import json
import sys
import traceback
from dataclasses import asdict
from pathlib import Path

import open3d as o3d

from pointcloud_core import RaycastPipelineParams, run_raycast_pipeline


def main() -> int:
    payload = json.loads(sys.argv[1])
    params = RaycastPipelineParams(
        base_path=Path(payload["base_path"]),
        updated_path=Path(payload["updated_path"]),
        output_dir=Path(payload["output_dir"]),
        max_distance=payload["max_distance"],
        base_origin=tuple(payload["base_origin"]) if payload["base_origin"] is not None else None,
        updated_origin=tuple(payload["updated_origin"]) if payload["updated_origin"] is not None else None,
        mesh_method=payload["mesh_method"],
        mesh_voxel_size=payload["mesh_voxel_size"],
        ray_method=payload["ray_method"],
        density_trim_quantile=payload["density_trim_quantile"],
        poisson_depth=payload["poisson_depth"],
        mesh_smoothing_iterations=payload["mesh_smoothing_iterations"],
        delaunay_max_edge_trim_factor=payload["delaunay_max_edge_trim_factor"],
    )
    try:
        result = run_raycast_pipeline(params, log=lambda message: print(message, flush=True))
        base_mesh_path = params.output_dir / "raycast_base_mesh.ply"
        updated_mesh_path = params.output_dir / "raycast_updated_mesh.ply"
        print("Guardando mallas reconstruidas...", flush=True)
        if not o3d.io.write_triangle_mesh(str(base_mesh_path), result.base_mesh):
            raise RuntimeError(f"No se pudo guardar la malla base: {base_mesh_path}")
        if not o3d.io.write_triangle_mesh(str(updated_mesh_path), result.updated_mesh):
            raise RuntimeError(f"No se pudo guardar la malla actualizada: {updated_mesh_path}")
        stats_path = params.output_dir / "raycast_stats.json"
        stats_path.write_text(json.dumps(asdict(result.stats)), encoding="utf-8")
        print("__AURORA_RESULT__", flush=True)
        return 0
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
