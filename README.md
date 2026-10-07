# Aurora — medición de espesor de shotcrete

Aplicación de escritorio GTK3 para capturar y comparar nubes de puntos `.ply`
de un túnel antes y después de aplicar shotcrete. Incluye análisis por
consola, alineación por referencias, visualización 3D y transmisión a un
navegador móvil.

## Plataforma

La interfaz gráfica de esta rama es GTK3 (`scripts/gui_gtk.py`). El setup
automático está preparado para Linux (Ubuntu) y macOS. En Windows no hay un
setup GTK probado; si se necesita ejecutar la GUI desde Windows, usar un
entorno Linux con escritorio, por ejemplo Ubuntu con WSLg, y validar Open3D y
la conexión al sensor en ese entorno.

La integración con el Aurora Slamtec requiere instalar su SDK nativo aparte.
Sin el SDK se pueden analizar archivos `.ply` existentes.

## Inicio rápido

En Ubuntu:

```bash
./setup.sh
./venv/bin/python3 scripts/gui_gtk.py
```

En macOS, instala [Homebrew](https://brew.sh) y ejecuta:

```bash
./setup.sh
./venv/bin/python3 scripts/gui_gtk.py
```

El setup instala GTK3/PyGObject como dependencias del sistema y las demás
dependencias Python dentro de `venv`. El venv hereda los bindings del sistema
para que `gi` esté disponible.

## Flujo de trabajo por pasadas

1. Captura o carga una nube BASE.
2. Guarda cada escaneo posterior con un nombre distinto para conservar todas
   las pasadas.
3. Selecciona la BASE y la pasada que quieres analizar. Si se movió el Aurora,
   abre **Alineación** y selecciona al menos tres referencias fijas en ambas
   nubes, en el mismo orden. La aplicación guarda una copia alineada e indica
   el error RMS de las referencias.
4. Calcula y revisa el mapa de espesor. El CSV, histograma y mapa PLY se
   guardan en la carpeta de salida configurada. Cambia esa carpeta entre
   pasadas si quieres conservar cada juego de resultados. El informe se
   exporta como PDF con fecha y hora.

BASE contra cada pasada mide el cambio acumulado desde la BASE. Para estimar
lo añadido entre dos etapas, selecciona la captura anterior como BASE. Las
referencias deben quedar visibles y fijas. ICP puede absorber el espesor si
ajusta una pared completa, por lo que no se recomienda para registrar
superficies que cambiaron.

Los porcentajes del informe representan puntos del escaneo, no porcentaje de
superficie. El cálculo C2C usa distancias no firmadas; su exactitud depende de
la alineación, densidad y calidad de las capturas. Valida con espesores
conocidos antes de usarlo como medición contractual.

## Captura con el sensor Aurora

El SDK de Python Slamtec no se instala desde PyPI. Sigue las instrucciones
oficiales de [`py_aurora_remote`](https://github.com/Slamtec/py_aurora_remote)
para compilar e instalar el wheel compatible con tu sistema. Conecta el
equipo a la WiFi del sensor o a su red y usa la IP configurada en la pestaña
**Captura** (por defecto `192.168.11.1`). La integración debe verificarse con
el hardware real antes de usarla en terreno.

## Línea de comandos

```bash
./venv/bin/python3 scripts/compare_point_clouds.py \
  --base data/base.ply \
  --updated data/updated.ply \
  --visualize
```

Opciones útiles: `--voxel-size`, `--remove-outliers`, `--icp`,
`--crop-min/--crop-max`, `--max-distance`, `--output-dir` y `--overlay`.
Consulta `scripts/compare_point_clouds.py --help` para la lista completa.

## Estructura principal

```text
scripts/pointcloud_core.py      procesamiento y medición
scripts/aurora_sensor.py        integración con el sensor
scripts/gui_gtk.py              interfaz GTK3
scripts/live_viewer.py          visor 3D
scripts/live_stream_server.py   transmisión a navegador móvil
scripts/web_static/viewer.html  cliente WebGL
scripts/compare_point_clouds.py CLI
setup.sh                        instalación en Linux/macOS
requirements.txt                dependencias Python
MANUAL_USO.md                   guía de la interfaz
```
