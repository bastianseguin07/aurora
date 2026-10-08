"""
Comparacion de nubes de puntos 3D (.ply) para medir el espesor de shotcrete.

Calcula la distancia Cloud-to-Cloud (C2C) desde cada punto de la nube
"actualizada" (tunel con shotcrete) hacia su vecino mas cercano en la
nube "base" (tunel original). Esa distancia es la estimacion del
espesor de la capa de hormigon proyectado en cada punto.

Uso basico:
    python compare_point_clouds.py --base ../data/base.ply --updated ../data/updated.ply

Para pruebas controladas (recorte a una region de interes), usar
--crop-min / --crop-max con las coordenadas x y z minimas y maximas de la
region a analizar (en metros, mismo sistema de referencia que el .ply):
    python compare_point_clouds.py --base ../data/base.ply --updated ../data/updated.ply \
        --crop-min -0.3 -0.3 0.5 --crop-max 0.3 0.3 1.5

Para elegir el recorte visualmente en vez de escribir coordenadas, usar la GUI
(gui.py), que incluye un selector interactivo en 3D.

Ver `python compare_point_clouds.py --help` para todas las opciones.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from pointcloud_core import PipelineParams, run_pipeline, visualize


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compara dos nubes de puntos .ply para estimar el espesor de shotcrete."
    )
    parser.add_argument("--base", required=True, type=Path, help="Ruta a la nube base (tunel original).")
    parser.add_argument("--updated", required=True, type=Path, help="Ruta a la nube actualizada (con shotcrete).")
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Tamano de voxel (m) para downsample previo. 0 desactiva el downsample.",
    )
    parser.add_argument(
        "--remove-outliers",
        action="store_true",
        help="Aplica un filtro estadistico de outliers antes de comparar.",
    )
    parser.add_argument(
        "--icp",
        action="store_true",
        help="Aplica ICP para realinear la nube actualizada contra la base antes de medir distancias.",
    )
    parser.add_argument(
        "--icp-threshold",
        type=float,
        default=0.05,
        help="Distancia maxima de correspondencia para ICP, en metros (default: 0.05).",
    )
    parser.add_argument(
        "--crop-min",
        type=float,
        nargs=3,
        metavar=("X_MIN", "Y_MIN", "Z_MIN"),
        default=None,
        help="Esquina minima (x y z, en metros) de la region de interes a recortar en ambas nubes.",
    )
    parser.add_argument(
        "--crop-max",
        type=float,
        nargs=3,
        metavar=("X_MAX", "Y_MAX", "Z_MAX"),
        default=None,
        help="Esquina maxima (x y z, en metros) de la region de interes a recortar en ambas nubes.",
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=None,
        help="Distancia maxima (m) usada para saturar la escala de color del heatmap.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "output",
        help="Carpeta donde se guardan resultados (CSV, PLY con heatmap, histograma).",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Abre una ventana interactiva de Open3D con el heatmap.",
    )
    parser.add_argument(
        "--overlay",
        action="store_true",
        help="En la visualizacion, superpone la nube base (gris) junto con el heatmap.",
    )
    args = parser.parse_args()

    if (args.crop_min is None) != (args.crop_max is None):
        parser.error("--crop-min y --crop-max deben usarse juntos.")
    return args


def main() -> int:
    args = parse_args()

    params = PipelineParams(
        base_path=args.base,
        updated_path=args.updated,
        output_dir=args.output_dir,
        voxel_size=args.voxel_size,
        remove_outliers=args.remove_outliers,
        use_icp=args.icp,
        icp_threshold=args.icp_threshold,
        crop_min=tuple(args.crop_min) if args.crop_min else None,
        crop_max=tuple(args.crop_max) if args.crop_max else None,
        max_distance=args.max_distance,
    )

    result = run_pipeline(params)

    if args.visualize:
        visualize(result.base_cloud, result.heatmap_cloud, args.overlay)

    return 0


if __name__ == "__main__":
    sys.exit(main())
