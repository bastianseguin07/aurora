# Aurora — Medicion de espesor de shotcrete a partir de nubes de puntos

La interfaz de escritorio de Aurora es GTK3 (`scripts/gui_gtk.py`) tanto en Ubuntu como en Windows. Cada sistema usa su propio entorno de Python.

## Compatibilidad por sistema operativo

Ubuntu instala GTK desde `apt`; Windows usa un entorno Pixi aislado con GTK3, PyGObject y Open3D de conda-forge. Ambos ejecutan el mismo archivo `scripts/gui_gtk.py` y comparten los modulos de captura, alineacion y comparacion.

| Sistema | Interfaz GTK3 | Entorno |
|---|---|---|
| Ubuntu 20.04+ / WSLg | `scripts/gui_gtk.py` | `setup.sh` y paquetes GTK del sistema |
| Windows 10/11 | `scripts/gui_gtk.py` | Pixi y `pixi.lock` (GTK3 + Open3D en el mismo entorno) |
| macOS | `scripts/gui_gtk.py` | Homebrew; no validado en este proyecto |

En Windows, la GUI GTK se ha iniciado con el entorno Pixi y se verificaron GTK3 y Open3D. La conexion con el sensor real sigue dependiendo de instalar el SDK Slamtec compilado para Windows; la ruta de captura con archivos y los visores se ejecutan dentro del mismo entorno.

## Inicio rapido

Clona el repo, entra a la carpeta `Aurora`, y corre el script de setup segun
tu sistema. Crea el entorno virtual e instala las dependencias
automaticamente (puede tardar unos minutos por Open3D).

### Linux (Ubuntu 20.04+)

```bash
./setup.sh
```

Instala los paquetes de sistema necesarios (`python3-venv`,
`libgl1-mesa-glx`, `libgomp1`, `python3-gi`, `gir1.2-gtk-3.0`) via `apt`
(pide `sudo` la primera vez), y crea el entorno virtual con
`--system-site-packages` para que herede los bindings de GTK3 instalados a
nivel de sistema. Despues instala el resto de las dependencias (Open3D,
numpy, etc.) dentro del venv con `pip`, normalmente.

```bash
./venv/bin/python3 scripts/gui_gtk.py
```

### macOS

Requiere [Homebrew](https://brew.sh) instalado primero. Despues:

```bash
./setup.sh
```

El script detecta que estas en macOS y corre `brew install python gtk+3
pygobject3`, y crea el entorno virtual usando el Python de Homebrew (no el
Python del sistema/Xcode) con `--system-site-packages`, por la misma razon
que en Linux: GTK3 vive a nivel de sistema (Homebrew), no via pip.

```bash
./venv/bin/python3 scripts/gui_gtk.py
```

> Si el `pip install open3d` del setup falla, es probablemente porque tu Mac
> tiene una version de Python muy nueva o muy vieja para el wheel de Open3D
> disponible en ese momento — instala una version de Python 3.9-3.12 via
> `brew install python@3.11` (por ejemplo) y volve a crear el venv apuntando
> a ese binario especifico.

### Windows

Para usar la misma interfaz GTK3 que en Ubuntu, instala [Pixi](https://pixi.sh/latest/#installation) y ejecuta en PowerShell:

```powershell
.\setup_gtk_windows.ps1
.\run_gtk_windows.ps1
```

Pixi crea el entorno `.pixi/` con las versiones fijadas en `pixi.lock`. Para transferir cambios entre equipos, usa la misma rama de Git.

## Estructura

```
Aurora/
├── venv/                          <- entorno virtual de Ubuntu/macOS (lo crea setup.sh)
├── data/
│   ├── base.ply                   <- nube del tunel original (coloca aqui tu archivo)
│   └── updated.ply                <- nube del tunel con shotcrete (coloca aqui tu archivo)
├── output/                        <- resultados generados por el script
│   ├── thickness_per_point.csv
│   ├── thickness_histogram.png
│   └── thickness_heatmap.ply
├── scripts/
│   ├── pointcloud_core.py         <- logica central (comparacion, crop, alineacion Procrustes)
│   ├── aurora_sensor.py           <- integracion con el sensor Slamtec Aurora
│   ├── live_viewer.py             <- visor 3D independiente (estatico o en vivo), ventana aparte
│   ├── embedded_viewer.py         <- visor 3D embebido en la ventana (seccion experimental)
│   ├── compare_point_clouds.py    <- linea de comandos
│   ├── gui_gtk.py                 <- unica interfaz grafica GTK3 (Ubuntu y Windows)
├── setup_gtk_windows.ps1          <- setup GTK3 para Windows mediante Pixi
|-- run_gtk_windows.ps1            <- lanza la GUI GTK3 en Windows
├── setup.sh                       <- setup automatico (Linux y macOS) -> gui_gtk.py
├── requirements.txt
└── README.md
```

## 1. Crear y activar el entorno virtual manualmente (alternativa a los scripts de setup)

En Ubuntu/macOS, si preferis no usar `setup.sh` o necesitas ajustar la version
de Python, podes crear el entorno a mano. En Windows, usa Pixi como se explica arriba.

**Linux:**
```bash
cd /ruta/a/Aurora
python3 -m venv --system-site-packages venv   # hereda los bindings GTK instalados por apt
source venv/bin/activate
```

**macOS:**
```bash
cd /ruta/a/Aurora
$(brew --prefix)/bin/python3 -m venv --system-site-packages venv   # idem, Python de Homebrew
source venv/bin/activate
```

> **Nota sobre la version de Python:** Open3D publica wheels para versiones
> especificas de Python (tipicamente 3.9-3.12 segun la version de Open3D). Si
> `pip install open3d` falla en tu version actual, instala una version
> compatible (por ejemplo Python 3.11) y crea el entorno virtual con esa
> version especifica.

## 2. Instalar dependencias (si no usaste el script de setup)

Con el entorno virtual activado (deberias ver `(venv)` al inicio del prompt):

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Esto instala las dependencias Python de medicion y visualizacion (`open3d`, `numpy`, `scipy` y `matplotlib`) con versiones compatibles entre si. Los bindings de GTK3
(`gi`/PyGObject, para `gui_gtk.py`) **no** estan en `requirements.txt` porque
no se instalan con pip — vienen del sistema (`apt`/`brew`), y el venv los
hereda solo si se creo con `--system-site-packages` (ver seccion 1).

## 3. Colocar tus nubes de puntos

Copia tus dos archivos `.ply` dentro de `Aurora/data/`:

- `data/base.ply` — tunel original, antes del shotcrete.
- `data/updated.ply` — mismo tunel, despues de aplicar el shotcrete.

Alternativa: capturar las nubes directamente desde el sensor con la GUI
(pestaña "Captura"), sin necesidad de tener los `.ply` de antemano.

## 3.1 Instalar el SDK del sensor Slamtec Aurora (opcional, solo si vas a capturar en vivo)

El dispositivo con SSID `SLAMWARE-Aurora-XXXX` es un **Slamtec Aurora**. Su SDK
de Python no esta en PyPI: hay que clonarlo y compilar un wheel una vez, con
el entorno virtual del proyecto activado.

**Windows:**
```powershell
git clone --recursive https://github.com/Slamtec/py_aurora_remote.git aurora_sdk_src
cd aurora_sdk_src
pip install -r requirements-dev.txt
python tools/build_package.py --platforms win64
pip install wheels\slamtec_aurora_python_sdk_win64-2.1.1-py3-none-any.whl
cd ..
```

**Linux:** igual, pero `--platforms linux_x86_64` y el wheel `..._linux_x86_64-...whl`.

**macOS:** igual, pero `--platforms macos_arm64` (Apple Silicon) o
`macos_x86_64` (Intel), y el wheel correspondiente.

Conecta la PC a la red WiFi del sensor (`SLAMWARE-Aurora-XXXX`) o a la misma
red que el sensor. La IP por defecto del dispositivo suele ser `192.168.11.1`
(configurable en el campo "Direccion del sensor" de la GUI).

> Si no instalas este SDK, el resto de la aplicacion (comparar `.ply` ya
> existentes, GUI, CLI, crop, alineacion, heatmap) funciona igual — la GUI
> simplemente mostrara un error claro al intentar conectar el sensor.

> **Nota:** esta integracion se escribio siguiendo exactamente el patron del
> ejemplo oficial del SDK (`examples/dense_point_cloud.py`), pero no pudo
> probarse contra el sensor fisico real durante el desarrollo. Probala primero
> con el sensor a mano; si algo falla, el mensaje de error indicara en que
> paso ocurrio (conexion, camara de profundidad no soportada, suscripcion, o
> captura de frames).

## 4. Usar la GUI

La aplicacion GTK3 se inicia con `./venv/bin/python3 scripts/gui_gtk.py` en Ubuntu o `.\run_gtk_windows.ps1` en Windows. El flujo de operador y las instrucciones detalladas estan en [MANUAL_USO.md](MANUAL_USO.md); la revision de proposito, limites y decisiones de interfaz esta en [AUDITORIA_GUI.md](AUDITORIA_GUI.md).

La barra lateral organiza las secciones en **Flujo de trabajo** (Captura, Guia de pose IMU, Alineacion por puntos, Preparar nubes), **Configuracion** (Archivos ANTES/DESPUES, Ajustes de analisis y Visualizacion 3D) y **Resultados**. **Medicion rapida** es la entrada principal; ahi se eligen o capturan ambas nubes y se calcula el espesor. Preparar nubes permite recortar un PLY grande a una copia mas pequena para aislar cajas u otros objetos. La guia IMU ayuda a repetir una pose bajo el mismo tracking, pero no registra las nubes por software. Si las poses difieren, usa la alineacion por puntos antes de calcular.

Cloud-to-Cloud es el metodo inicial. Poisson y Delaunay son opciones de reconstruccion disponibles en Medicion rapida; Delaunay suele requerir un recorte local preparado con **Preparar nubes**. La seccion **Resultados** concentra el estado, las estadisticas, los detalles tecnicos y el boton para exportar el informe PDF. Las funciones de captura y transmision en vivo son herramientas opcionales; valida el comportamiento con el sensor y el entorno reales antes de usarlas para mediciones de produccion.
## 5. Ejecutar la comparacion por linea de comandos (alternativa sin GUI)

```bash
python scripts/compare_point_clouds.py --base data/base.ply --updated data/updated.ply --visualize
```

Esto imprime en consola las estadisticas de espesor (media, mediana, desvio,
min/max, percentil 95) y genera en `output/`:

- `thickness_per_point.csv` — distancia (espesor) por cada punto, en metros y mm.
- `thickness_histogram.png` — histograma de la distribucion de espesores.
- `thickness_heatmap.ply` — la nube "actualizada" coloreada como heatmap
  (azul = poco espesor, rojo = mucho espesor) para abrir en Open3D, CloudCompare, etc.

Con `--visualize` se abre ademas una ventana interactiva de Open3D mostrando
el heatmap. Agregando `--overlay` se superpone tambien la nube base en gris.

### Opciones utiles

| Flag | Para que sirve |
|---|---|
| `--voxel-size 0.01` | Downsample previo (en metros) para nubes muy densas; acelera el calculo. |
| `--remove-outliers` | Filtra puntos ruidosos/aislados antes de comparar. |
| `--icp` | Realinea la nube actualizada contra la base con ICP antes de medir. Usar solo si sospechas de un error de registro entre escaneos, no para corregir el espesor real (para eso, mejor usar la alineacion por puntos de referencia desde la GUI). |
| `--crop-min X Y Z --crop-max X Y Z` | Recorta ambas nubes a una caja delimitadora (en metros) antes de comparar. |
| `--max-distance 0.08` | Satura la escala de color del heatmap a un espesor maximo esperado (en metros). |
| `--output-dir otra_carpeta` | Cambia donde se guardan los resultados (por defecto `Aurora/output`). |

Ejemplo mas completo:

```bash
python scripts/compare_point_clouds.py \
    --base data/base.ply \
    --updated data/updated.ply \
    --voxel-size 0.01 \
    --remove-outliers \
    --crop-min -0.3 -0.3 0.5 --crop-max 0.3 0.3 1.5 \
    --max-distance 0.10 \
    --visualize --overlay
```

## 6. Desactivar el entorno virtual

```bash
deactivate
```

## Notas tecnicas

- El script calcula la **distancia Cloud-to-Cloud (C2C)**: para cada punto de
  la nube actualizada busca su vecino mas cercano en la nube base (via KD-Tree,
  `Open3D.compute_point_cloud_distance`). Esa distancia euclidiana es la
  estimacion del espesor de shotcrete en ese punto. Es una distancia **no
  dirigida/no firmada** (siempre positiva).
- Para que esa distancia sea una buena estimacion del espesor real, ambas
  nubes deben estar en el mismo sistema de referencia. Si no lo estan, hay dos
  formas de corregirlo:
  - **Alineacion por puntos de referencia (Procrustes/Kabsch, GUI, pestaña
    "Alineacion")**: usa solo puntos fijos elegidos a mano (que no cambiaron
    entre capturas). Recomendado cuando hay puntos de referencia identificables
    (pernos, marcas).
  - **ICP (`--icp` / checkbox "Corregir alineacion")**: ajusta usando toda la
    superficie. Mas rapido/automatico, pero si el shotcrete cambia TODA la
    pared, puede confundir el espesor real con error de alineacion — usar con
    criterio, e idealmente solo cuando el fondo estatico domina en cantidad de
    puntos (por ejemplo, una escena con un objeto pequeno que se movio).
