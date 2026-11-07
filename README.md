# Aurora — Medicion de espesor de shotcrete a partir de nubes de puntos

La interfaz de escritorio de Aurora usa GTK3 (`scripts/gui_gtk.py`) y está preparada para Ubuntu 20.04 o posterior. El entorno Python se instala localmente en `venv/`.

## Sensor Unitree LiDAR L2 (rama feat/unitree-l2)

Esta rama conserva la GUI y los análisis y añade selección de sensor Aurora/Unitree,
SDK UDP directo, puente ROS opcional y decay time separado de las capturas. Consulta
[UNITREE_L2.md](UNITREE_L2.md) para instalación, red, captura nativa/acumulada,
campos guardados y pruebas pendientes con hardware.

Para ROS 2 Lyrical y RViz en Ubuntu 26.04, consulta
[ROS2_LYRICAL.md](ROS2_LYRICAL.md). Después de compilar el driver puedes abrir
solo el visor con `./open_rviz.sh`, o driver y visor con `./run_unitree_ros2.sh`.
La guia de pose y la compensacion con seguimiento Unitree se describen en
[UNITREE_POSE.md](UNITREE_POSE.md).

```bash
./setup.sh --with-unitree-sdk
./run_gui.sh
```

## Instalación e inicio en Ubuntu

Ejecuta desde la carpeta del proyecto:

```bash
./setup.sh
./run_gui.sh
```

El setup instala los paquetes GTK del sistema si hacen falta y crea el entorno virtual con las dependencias Python. Para abrir la GUI directamente también puedes usar `./venv/bin/python3 scripts/gui_gtk.py`.

## Estructura

```
Aurora/
├── venv/                          <- entorno virtual de Ubuntu (lo crea setup.sh)
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
│   ├── gui_gtk.py                 <- interfaz grafica GTK3 para Ubuntu
├── setup.sh                       <- setup automatico para Ubuntu
├── requirements.txt
└── README.md
```

## 1. Crear y activar el entorno virtual manualmente (alternativa)

Si prefieres no usar `setup.sh`, en Ubuntu puedes crear el entorno así:

```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
```

## 2. Instalar dependencias (si no usaste el script de setup)

Con el entorno virtual activado (deberias ver `(venv)` al inicio del prompt):

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Esto instala las dependencias Python de medicion y visualizacion (`open3d`, `numpy`, `scipy` y `matplotlib`) con versiones compatibles entre si. Los bindings de GTK3
(`gi`/PyGObject, para `gui_gtk.py`) **no** estan en `requirements.txt` porque
no se instalan con pip — vienen del sistema (`apt`), y el venv los
hereda solo si se creo con `--system-site-packages` (ver seccion 1).

## 3. Colocar tus nubes de puntos

Copia tus dos archivos `.ply` dentro de `Aurora/data/`:

- `data/base.ply` — tunel original, antes del shotcrete.
- `data/updated.ply` — mismo tunel, despues de aplicar el shotcrete.

Alternativa: capturar las nubes directamente desde el sensor con la GUI
(pestaña "Captura"), sin necesidad de tener los `.ply` de antemano.

## 3.1 Instalar el SDK del sensor Slamtec Aurora (opcional, solo si vas a capturar en vivo)

El dispositivo con SSID `SLAMWARE-Aurora-XXXX` es un **Slamtec Aurora**. El SDK
se obtiene del repositorio oficial y se prepara para la arquitectura de Ubuntu.
El clon se guarda localmente en `aurora_sdk_src/` (ignorado por Git), no se
incluye ni se redistribuye dentro del repositorio de Aurora.

**Ubuntu:**
```bash
# Instalar GUI, dependencias y SDK del sensor en un solo paso:
./setup.sh --with-sensor-sdk

# Si Aurora ya esta instalado, basta con preparar el SDK:
./install_sensor_sdk.sh
```

El instalador usa el tag oficial `2.1.1-rtm`, incluye los submodulos nativos y
construye el wheel para Linux x86_64 o ARM64. Requiere conexion a Internet.

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

La aplicacion GTK3 se inicia con `./run_gui.sh` en Ubuntu. El flujo de operador y las instrucciones detalladas estan en [MANUAL_USO.md](MANUAL_USO.md).

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
