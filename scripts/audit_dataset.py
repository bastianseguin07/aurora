"""Repeatable raw-pair diagnostic; it does not create an operator session."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from measurement_session import cloud_diagnostics
from pointcloud_core import PipelineParams, run_pipeline


def main():
    parser = argparse.ArgumentParser(description="Diagnostico C2C sin registro; no valida espesor ni trabajo en mina.")
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--updated", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("output/auditoria"))
    args = parser.parse_args()
    result = run_pipeline(PipelineParams(args.base, args.updated, args.output_dir), log=lambda _message: None)
    base = cloud_diagnostics(result.base_cloud)
    updated = cloud_diagnostics(result.updated_cloud)
    reverse = np.asarray(result.base_cloud.compute_point_cloud_distance(result.updated_cloud))
    output = {
        "status": "diagnostico_sin_registro_no_apto_para_medicion",
        "base": str(args.base.resolve()),
        "updated": str(args.updated.resolve()),
        "stats_m": {key: getattr(result.stats, key) for key in ("mean", "median", "std", "min", "max", "p95", "n_points")},
        "diagnostics": {
            "base": base,
            "updated": updated,
            "reverse_c2c_median_m": float(np.median(reverse)),
            "reverse_c2c_p95_m": float(np.percentile(reverse, 95)),
        },
        "limitation": "Nubes comparadas en sus coordenadas originales, sin referencias, espesor fisico conocido ni validacion de sensor. Solo diagnostico del pipeline.",
        "artifacts": {"csv": str(result.csv_path), "histogram": str(result.histogram_path), "heatmap": str(result.heatmap_path)},
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "diagnostico_dataset.json").write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
