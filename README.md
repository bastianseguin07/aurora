# Aurora — Comparacion de nubes de puntos LiDAR (espesor de shotcrete)

Proyecto autocontenido: todo (entorno virtual, scripts y archivos `.ply`) vive
dentro de esta carpeta `Aurora`.

## Inicio rapido

Cloná el repo, entrá a la carpeta `Aurora`, y corré el script de setup segun tu
sistema. Crea el entorno virtual e instala todas las dependencias
automaticamente (puede tardar unos minutos por Open3D).

**Windows (PowerShell):**

```powershell
.\setup.ps1
```

Si PowerShell bloquea la ejecucion por politica de scripts, ver la nota en la
seccion 1 mas abajo — es un permiso que se habilita una sola vez.

**Windows (CMD / simbolo del sistema):**

```bat
setup.bat
```

(`setup.ps1` es un script de PowerShell; `setup.bat` es un wrapper para poder
correrlo igual desde CMD o con doble clic).

**Linux (Ubuntu 20.04):**

```bash
./setup.sh
```

Instala tambien los paquetes de sistema necesarios (`python3-venv`,
`python3-tk`, `libgl1-mesa-glx`, `libgomp1`) via `apt`, pidiendo `sudo` la
primera vez. Este script no se pudo probar contra una maquina Ubuntu real
durante el desarrollo (el entorno de desarrollo fue Windows) — los nombres de
paquete y comandos son los estandar de Ubuntu 20.04, pero si algo falla al
instalar, avisa en que paso para ajustarlo.

Al terminar cualquiera de los 3, abrí la interfaz grafica:

```powershell
# Windows
.\venv\Scripts\python.exe scripts\gui.py
```
```bash
# Linux
./venv/bin/python3 scripts/gui.py
```

## Estructura

```
Aurora/
├── venv/                          <- entorno virtual (lo crea setup.ps1 / setup.sh)
├── data/
│   ├── base.ply                   <- nube del tunel original (coloca aqui tu archivo)
│   └── updated.ply                <- nube del tunel con shotcrete (coloca aqui tu archivo)
├── output/                        <- resultados generados por el script
│   ├── thickness_per_point.csv
│   ├── thickness_histogram.png
│   └── thickness_heatmap.ply
├── scripts/
│   ├── pointcloud_core.py         <- logica central de comparacion (usada por CLI y GUI)
│   ├── aurora_sensor.py           <- integracion con el sensor Slamtec Aurora
│   ├── live_viewer.py             <- visor 3D independiente (estatico o en vivo)
│   ├── compare_point_clouds.py    <- linea de comandos
│   └── gui.py                     <- interfaz grafica (recomendada)
├── setup.ps1                      <- setup automatico (Windows, PowerShell)
├── setup.bat                      <- wrapper de setup.ps1 para Windows CMD / doble clic
├── setup.sh                       <- setup automatico (Linux / Ubuntu 20.04)
├── requirements.txt
└── README.md
```

## 1. Crear y activar el entorno virtual (manual, alternativa a los scripts de setup)

Todos los comandos se ejecutan **desde dentro de la carpeta `Aurora`** en PowerShell.

```powershell
cd C:\Users\basti\OneDrive\Escritorio\Aurora

# Crear el entorno virtual
python -m venv venv

# Activar el entorno virtual
.\venv\Scripts\Activate.ps1
```

> Si PowerShell bloquea la activacion por la politica de ejecucion de scripts,
> corre una vez (en una consola con permisos de usuario, no hace falta admin):
> `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`

> **Nota sobre la version de Python:** Open3D publica wheels para versiones
> especificas de Python. Si `pip install open3d` falla en tu version actual,
> instala Python 3.10 o 3.11 (https://www.python.org/downloads/) y crea el
> entorno virtual con esa version, por ejemplo:
> `py -3.11 -m venv venv`

## 2. Instalar dependencias

Con el entorno virtual activado (deberias ver `(venv)` al inicio del prompt):

```powershell
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Esto instala `open3d`, `numpy`, `scipy`, `matplotlib` y `customtkinter` (para
la interfaz grafica) con versiones compatibles entre si (evita el conflicto
conocido de Open3D con NumPy 2.x).

## 3. Colocar tus nubes de puntos

Copia tus dos archivos `.ply` dentro de `Aurora\data\`:

- `data\base.ply` — tunel original, antes del shotcrete.
- `data\updated.ply` — mismo tunel, despues de aplicar el shotcrete.

Alternativa: capturar las nubes directamente desde el sensor con la GUI (ver
seccion "Sensor Aurora" mas abajo), sin necesidad de tener los `.ply` de antemano.

## 3.1 Instalar el SDK del sensor Slamtec Aurora (opcional, solo si vas a capturar en vivo)

El dispositivo con SSID `SLAMWARE-Aurora-XXXX` es un **Slamtec Aurora**. Su SDK
de Python no esta en PyPI: hay que clonarlo y compilar un wheel una vez.
Con el entorno virtual de Aurora activado:

```powershell
cd C:\Users\basti\OneDrive\Escritorio\Aurora
git clone --recursive https://github.com/Slamtec/py_aurora_remote.git aurora_sdk_src
cd aurora_sdk_src
pip install -r requirements-dev.txt
python tools/build_package.py --platforms win64
pip install wheels\slamtec_aurora_python_sdk_win64-2.1.1-py3-none-any.whl
cd ..
```

En Linux, el mismo procedimiento pero con `--platforms linux_x86_64` y
`pip install` sobre el wheel `..._linux_x86_64-...whl` generado.

Conecta la PC a la red WiFi del sensor (`SLAMWARE-Aurora-XXXX`) o a la misma
red que el sensor. La IP por defecto del dispositivo suele ser `192.168.11.1`
(configurable en el campo "Direccion del sensor" de la GUI).

> Si no instalas este SDK, el resto de la aplicacion (comparar `.ply` ya
> existentes, GUI, CLI, crop, heatmap) funciona igual — la GUI simplemente
> mostrara un error claro al intentar conectar el sensor, indicando que falta
> el paquete `slamtec_aurora_sdk`.

> **Nota:** esta integracion se escribio siguiendo exactamente el patron del
> ejemplo oficial del SDK (`examples/dense_point_cloud.py`), pero no pudo
> probarse contra el sensor fisico real durante el desarrollo (no hay hardware
> conectado en este entorno). Probala primero con el sensor a mano; si algo
> falla, el mensaje de error indicara en que paso ocurrio (conexion, camara de
> profundidad no soportada, suscripcion, o captura de frames).

## 4. Usar la GUI (recomendado, sin consola)

En vez de escribir comandos, se puede usar la interfaz grafica:

```powershell
python scripts\gui.py
```

(o, sin activar el entorno: `.\venv\Scripts\python.exe scripts\gui.py`)

La ventana esta organizada en 3 pestañas ("Datos y sensor", "Procesamiento",
"Visualizacion") para no saturar la pantalla, con el boton principal
"Ejecutar comparacion" y el registro de resultados siempre visibles debajo.
Ahi podes:

- Elegir los archivos `.ply` base y actualizado con botones "Examinar...".
- Marcar casillas para quitar outliers o alinear con ICP, y escribir el
  tamano de voxel o el umbral de ICP si hace falta.
- **Recortar (crop) a una region de interes** antes de comparar — util para
  aislar, por ejemplo, solo la zona de una caja de prueba o un tramo puntual
  del tunel, en vez de que la estadistica se diluya con el resto de la
  escena que no cambio. Hay dos formas de definir el recorte:
  - Escribiendo manualmente las coordenadas minimas y maximas (x y z, en metros).
  - Con el boton **"Seleccionar recorte en visor 3D..."**: se abre la nube
    base en una ventana 3D, se hace **Shift + click izquierdo** sobre 2
    puntos que marquen esquinas opuestas de la zona de interes, y se cierra
    la ventana (tecla `Q` o el boton de cerrar). Los limites se completan
    solos en los campos Min/Max.
- Presionar **"Ejecutar comparacion"** — corre en segundo plano sin trabar la
  ventana, y el resultado (estadisticas, rutas de los archivos generados) se
  muestra en el cuadro de texto inferior.

### Sensor Aurora (capturar nubes en vivo)

En la seccion **"Sensor Aurora"**:

- Escribi la IP del sensor (por defecto `192.168.11.1`) y presiona **"Conectar"**.
- Con el sensor conectado, **"Capturar nube BASE"** / **"Capturar nube ACTUALIZADA"**
  toman una foto fija (acumulan ~15 frames del sensor para reducir ruido) y piden
  donde guardarla como `.ply` — el campo de archivo correspondiente se actualiza solo.
- Esto requiere el SDK del sensor instalado (ver seccion 3.1); si no esta instalado,
  la GUI muestra un mensaje de error explicando que falta y donde instalarlo.

### Color de espesor

En la seccion **"Color de espesor"** se elige entre:

- **Continuo (heatmap azul->rojo)** — gradiente proporcional al espesor (como antes).
- **3 niveles (verde/amarillo/rojo)** — clasifica cada punto en 3 bandas segun
  dos umbrales configurables en milimetros: verde si el espesor es menor al
  "umbral bajo", rojo si es mayor o igual al "umbral alto", amarillo en el medio.
  Util para ver de un vistazo donde el shotcrete quedo dentro de especificacion,
  insuficiente o excesivo.

### Vista 3D (estatica o en vivo)

La seccion **"Vista 3D"** abre una ventana de Open3D separada (no bloquea la GUI)
que muestra siempre la **nube base en gris** (el tunel original, sin shotcrete) y,
opcionalmente, la **nube actualizada coloreada por espesor**:

- **"Mostrar nube actualizada"** — tilda/destilda para mostrar u ocultar esa capa
  sin cerrar la ventana.
- **Estatica (archivo/captura)** — usa la nube actualizada cargada desde archivo
  o la ultima capturada/comparada.
- **En tiempo real (sensor)** — con el sensor conectado, la nube actualizada se
  redibuja continuamente con los frames en vivo del sensor, recalculando el
  espesor contra la nube base en cada frame.
- **"Abrir vista 3D"** / **"Cerrar vista 3D"** controlan la ventana. Cambiar el
  modo de color, los umbrales, o la fuente (estatica/vivo) mientras la vista esta
  abierta se aplica al instante.

## 5. Ejecutar la comparacion por linea de comandos (alternativa)

```powershell
python scripts\compare_point_clouds.py --base data\base.ply --updated data\updated.ply --visualize
```

Esto imprime en consola las estadisticas de espesor (media, mediana, desvio,
min/max, percentil 95) y genera en `output\`:

- `thickness_per_point.csv` — distancia (espesor) por cada punto, en metros y mm.
- `thickness_histogram.png` — histograma de la distribucion de espesores.
- `thickness_heatmap.ply` — la nube "actualizada" coloreada como heatmap
  (azul = poco espesor, rojo = mucho espesor) para abrir en Open3D, CloudCompare, etc.

Con `--visualize` se abre ademas una ventana interactiva de Open3D mostrando
el heatmap. Agregando `--overlay` se superpone tambien la nube base en gris
para comparar visualmente ambas superficies.

### Opciones utiles

| Flag | Para que sirve |
|---|---|
| `--voxel-size 0.01` | Downsample previo (en metros) para nubes muy densas; acelera el calculo. |
| `--remove-outliers` | Filtra puntos ruidosos/aislados antes de comparar. |
| `--icp` | Realinea la nube actualizada contra la base con ICP antes de medir. Usar solo si sospechas de un error de registro entre escaneos, no para corregir el espesor real. |
| `--crop-min X Y Z --crop-max X Y Z` | Recorta ambas nubes a una caja delimitadora (en metros) antes de comparar. Ideal para aislar una region de interes y evitar que el resto de la escena diluya la estadistica. |
| `--max-distance 0.08` | Satura la escala de color del heatmap a un espesor maximo esperado (en metros), util si hay outliers que "aplastan" la escala de colores. |
| `--output-dir otra_carpeta` | Cambia donde se guardan los resultados (por defecto `Aurora\output`). |

Ejemplo mas completo:

```powershell
python scripts\compare_point_clouds.py `
    --base data\base.ply `
    --updated data\updated.ply `
    --voxel-size 0.01 `
    --remove-outliers `
    --crop-min -0.3 -0.3 0.5 --crop-max 0.3 0.3 1.5 `
    --max-distance 0.10 `
    --visualize --overlay
```

## 6. Desactivar el entorno virtual

```powershell
deactivate
```

## Notas tecnicas

- El script calcula la **distancia Cloud-to-Cloud (C2C)**: para cada punto de
  la nube actualizada busca su vecino mas cercano en la nube base (via KD-Tree,
  `Open3D.compute_point_cloud_distance`). Esa distancia euclidiana es la
  estimacion del espesor de shotcrete en ese punto.
- Es una distancia **no dirigida/no firmada** (siempre positiva). Para la
  mayoria de los casos de shotcrete sobre pared de tunel esto es una buena
  aproximacion del espesor real, siempre que ambas nubes esten en el mismo
  sistema de referencia (mismo origen de escaneo/geo-referenciacion).
- Si las dos nubes no comparten exactamente el mismo sistema de coordenadas
  (por ejemplo, escaneos independientes sin geo-referenciar), usa `--icp` para
  alinear antes de medir. Ojo: si el propio shotcrete desplaza mucho la
  superficie, un ICP demasiado agresivo puede "corregir" parte del espesor
  real como si fuera error de alineacion — usarlo con criterio.
