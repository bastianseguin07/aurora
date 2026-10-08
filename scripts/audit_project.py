"""Inventory every project file except Git internals; inspect Python dependency structure."""
from __future__ import annotations
import ast
import csv
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent.parent
    directory = root / "docs"
    directory.mkdir(exist_ok=True)
    output = directory / "inventario.csv"
    rows = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or ".git" in path.relative_to(root).parts or path == output:
            continue
        relative = path.relative_to(root).as_posix()
        imports, definitions = [], []
        if path.suffix == ".py":
            tree = ast.parse(path.read_text(encoding="utf-8-sig"))
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imports.extend(item.name for item in node.names)
                if isinstance(node, ast.ImportFrom):
                    imports.append("." * node.level + (node.module or ""))
            definitions = [node.name for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]
        if "__pycache__" in relative or path.suffix == ".pyc":
            category, recommendation = "cache regenerable", "PRESCINDIBLE; no eliminado"
        elif relative.startswith(("data/", "output/")) and path.name != ".gitkeep":
            category, recommendation = "captura/resultado generado", "CONSERVAR; revisar archivado, no borrar automaticamente"
        elif relative.startswith("pcd/"):
            category, recommendation = "datos de prueba del usuario", "CONSERVAR; no valida mina"
        elif relative.startswith("python_bindings/"):
            category, recommendation = "SDK vendorizado/fallback", "CONSERVAR paquete; imports internos, binario externo requerido"
        elif path.name in ("gui.py", "smoke_test_quick_workflow_tk.py", "setup.ps1", "setup.bat"):
            category, recommendation = "Windows heredado", "CONSERVAR; sin paridad con sesiones GTK"
        elif path.name == "embedded_viewer.py":
            category, recommendation = "visor alternativo experimental", "LABORATORIO; candidato a eliminar tras elegir visor"
        elif path.suffix in (".md", ".txt"):
            category, recommendation = "documentacion/configuracion", "CONSERVAR; plan intacto, docs de flujo actualizadas"
        else:
            category, recommendation = "codigo/configuracion/pruebas", "CONSERVAR; ver matriz de auditoria"
        rows.append([relative, path.stat().st_size, category, recommendation, "; ".join(sorted(set(imports))), "; ".join(definitions)])
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["archivo", "bytes", "categoria", "decision", "imports", "definiciones_principales"])
        writer.writerows(rows)
    print(f"Inventario: {len(rows)} archivos, {sum(row[1] for row in rows):,} bytes. {output}")


if __name__ == "__main__":
    main()
