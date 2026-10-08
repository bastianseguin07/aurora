"""Repeatable software-only diagnostic for a pair of test point clouds."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from measurement_session import MeasurementSession


def main():
    parser = argparse.ArgumentParser(description="Ensayo exploratorio C2C; no valida espesor ni trabajo en mina.")
    parser.add_argument("--base", required=True, type=Path)
    parser.add_argument("--updated", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("output/auditoria"))
    args = parser.parse_args()
    session = MeasurementSession.create(args.output_dir, "Ensayo de cajas", "pcd — escena completa sin registro", True)
    session.add_capture(args.base, base=True)
    session.add_capture(args.updated)
    record, _result = session.compare(0, log=lambda _message: None)
    session.record_result(record)
    output = {"session": str(session.path), "stats_m": record["stats_m"], "diagnostics": record["diagnostics"],
              "limitation": "Capturas de cajas sin referencias ni espesor fisico conocido; no valida mina, iluminacion ni metrologia."}
    (args.output_dir / "pcd_diagnostico.json").write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
