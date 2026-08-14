# Aurora — Medicion de espesor de shotcrete a partir de nubes de puntos

Proyecto autocontenido: todo (entorno virtual, scripts y archivos `.ply`) vive
dentro de esta carpeta `Aurora`. Incluye la interfaz gráfica GTK3 (`scripts/gui_gtk.py`),
pensada para mostrar la app a clientes con un aspecto nativo de escritorio Linux/Mac.

## Compatibilidad real por sistema operativo

Antes de instalar nada, es importante saber que **no los 3 sistemas operativos
quedan igual de bien soportados** con la interfaz GTK — esto no es una
limitacion arbitraria, es una limitacion real de como se distribuyen los
bindings de GTK3 para Python en cada sistema:

| Sistema | GUI recomendada | Estado |
|---|---|---|
| **Linux (Ubuntu 20.04+)** | `scripts/gui_gtk.py` | **Probado y verificado en Ubuntu Linux**: GTK3 se instala como paquete de sistema (`apt`) y Open3D se instala normal via `pip`, ambos conviven en el mismo entorno virtual. |
| **macOS** | `scripts/gui_gtk.py` | Debería funcionar via Homebrew (`brew install gtk+3 pygobject3`), siguiendo el mismo patron que Linux. Instrucciones basadas en la practica estandar documentada de PyGObject/Homebrew — **no se pudo probar en una Mac real** (no hay una disponible en el entorno de desarrollo). |
| **Windows** | `scripts/gui.py` (CustomTkinter) | GTK3 **no tiene un camino simple** en Windows: no existe un wheel de pip que lo instale, la unica forma es MSYS2 (un Python separado del de Windows) o compilar GTK desde cero con `gvsbuild` (un proceso de horas, no apto para un setup de "un comando"). Se probo exhaustivamente en este desarrollo: **MSYS2 permite correr GTK3, pero ese mismo Python no puede instalar Open3D** (no hay wheel compatible). Por eso, en Windows se recomienda usar `gui.py`, una interfaz equivalente hecha con CustomTkinter que instala con un simple `pip install` y funciona con Open3D sin problemas. |

Todo lo demas (CLI, `pointcloud_core.py`, captura por sensor, alineacion,
crop, etc.) funciona igual sin importar que GUI uses — la unica diferencia es
la interfaz grafica en si.

## Inicio rapido

Clona el repo, entra a la carpeta `Aurora`, y corre el script de setup segun
tu sistema. Crea el entorno virtual e instala las dependencias
automaticamente (puede tardar unos minutos por Open3D).

### Linux (Ubuntu 20.04+)

```bash
./setup.sh
```

Instala los paquetes de sistema necesarios (`python3-venv`, `python3-tk`,
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

GTK3 no tiene un camino simple en Windows (ver tabla de arriba), asi que en
Windows se usa `gui.py` (CustomTkinter) en vez de `gui_gtk.py`:

**PowerShell:**
```powershell
.\setup.ps1
```
Si PowerShell bloquea la ejecucion por politica de scripts, correr una vez
(no requiere admin):
```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

**CMD / doble clic:**
```bat
setup.bat
```
(wrapper que invoca `setup.ps1` via PowerShell, para quienes no usan
PowerShell directamente)

Al terminar:
```powershell
.\venv\Scripts\python.exe scripts\gui.py
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
│   ├── pointcloud_core.py         <- logica central (comparacion, crop, alineacion Procrustes)
│   ├── aurora_sensor.py           <- integracion con el sensor Slamtec Aurora
│   ├── live_viewer.py             <- visor 3D independiente (estatico o en vivo), ventana aparte
│   ├── embedded_viewer.py         <- visor 3D embebido en la ventana (seccion experimental)
│   ├── compare_point_clouds.py    <- linea de comandos
│   ├── gui_gtk.py                 <- interfaz grafica GTK3 (recomendada en Linux/Mac, demos)
│   └── gui.py                     <- interfaz grafica CustomTkinter (recomendada en Windows)
├── setup.ps1                      <- setup automatico (Windows, PowerShell) -> gui.py
├── setup.bat                      <- wrapper de setup.ps1 para Windows CMD / doble clic
├── setup.sh                       <- setup automatico (Linux y macOS) -> gui_gtk.py
├── requirements.txt
└── README.md
```

## 1. Crear y activar el entorno virtual manualmente (alternativa a los scripts de setup)

Si preferis no usar `setup.ps1`/`setup.sh`, o necesitas ajustar algo (version
de Python especifica, etc.), podes hacerlo a mano.

**Windows (PowerShell):**
```powershell
cd C:\ruta\a\Aurora
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**Linux:**
```bash
cd /ruta/a/Aurora
python3 -m venv --system-site-packages venv   # --system-site-packages solo si vas a usar gui_gtk.py
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

Esto instala `open3d`, `numpy`, `scipy`, `matplotlib` y `customtkinter` (para
`gui.py`) con versiones compatibles entre si. Los bindings de GTK3
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

```bash
# Linux / macOS
./venv/bin/python3 scripts/gui_gtk.py
```
```powershell
# Windows
.\venv\Scripts\python.exe scripts\gui.py
```

La ventana esta organizada en pestañas con una barra lateral, con el boton
principal **"Calcular espesor"** siempre visible arriba y el panel de
resultado siempre visible abajo. Los controles poco frecuentes/tecnicos estan
escondidos detras de un desplegable **"Opciones avanzadas"**, y los campos
que necesitan explicacion tienen un icono ⓘ al lado (`gui_gtk.py`).

**Flujo tipico (un solo click):** elegis los dos archivos `.ply` en la
pestaña "Comparacion" (o los capturas primero en "Captura"), presionas
**"Calcular espesor"**, y la app corre el analisis, muestra los resultados
como tarjetas con los numeros principales, cambia sola a la pestaña
"Visualizacion 3D" y abre la vista 3D coloreada por espesor. El boton
**"Generar informe"** (arriba, junto a "Calcular espesor") exporta un reporte
en Markdown con las estadisticas y el estado (dentro/fuera de los umbrales).

### Pestaña "Captura"

Todo lo relacionado al sensor Aurora, separado de la comparacion: IP del
sensor, boton Conectar/Desconectar con indicador de estado (●
verde/rojo/amarillo), y botones **"Capturar tunel original"** / **"Capturar
tunel con shotcrete"** (toman una foto fija acumulando ~15 frames del sensor
para reducir ruido). Al guardar, la app cambia sola a "Comparacion" mostrando
el archivo ya cargado.

**Captura en tiempo real (MVP)** — en la misma pestaña:

- **"Iniciar captura en tiempo real"** — abre la vista 3D mostrando en vivo lo
  que el sensor esta viendo en este momento.
- **"Fijar BASE en vivo"** — usa el frame actual del sensor como referencia
  para medir espesor en vivo, sin necesidad de guardar un `.ply` antes (util
  para monitoreo continuo: apuntas el sensor, fijas la base, y segui viendo
  el espesor actualizarse en tiempo real a medida que aplicas shotcrete).
  **"Quitar BASE en vivo"** vuelve a usar la nube base original cargada.
- **"Guardar nubes en vivo"** — vuelca a disco, como dos `.ply`, la base de
  referencia actual y el frame en vivo mas reciente.

### Pestaña "Comparacion"

**"Tunel original"** / **"Tunel con shotcrete"** — cada seccion muestra el
nombre del archivo elegido (no la ruta completa, que aparece como tooltip)
con un boton **"Elegir archivo..."**.

### Pestaña "Alineacion" (Procrustes con puntos de referencia)

Si las dos capturas no comparten exactamente la misma posicion (el sensor se
reubico entre una y otra), esta pestaña permite alinearlas eligiendo
manualmente 3 o mas **puntos de referencia fijos** — por ejemplo cabezas de
pernos de anclaje, marcas o esquinas rigidas — en vez de usar ICP sobre toda
la superficie (que puede confundir el espesor real con error de alineacion).

1. **"1. Elegir puntos en el tunel original..."** — abre un visor 3D con la
   nube base. Shift+Click sobre cada punto de referencia, en un orden que vos
   elijas, despues cerrar la ventana (tecla `Q`).
2. **"2. Elegir los MISMOS puntos en el tunel con shotcrete..."** — mismo
   proceso sobre la otra nube, marcando los **mismos puntos fisicos en el
   mismo orden** (la correspondencia entre nubes es solo por orden de
   seleccion, no automatica).
3. **"Calcular alineacion y aplicar"** — calcula la rotacion y traslacion
   optimas (algoritmo de Kabsch / analisis de Procrustes) usando esos puntos,
   muestra el **error residual en mm** (que tan bien calzaron los puntos
   elegidos — un valor alto indica que se marcaron mal o en distinto orden),
   aplica la transformacion a toda la nube con shotcrete, la guarda como
   `<nombre>_alineado.ply`, y actualiza la pestaña "Comparacion" para usar ese
   archivo.

### Pestaña "Segmentacion" (recorte por box, opcional)

Va despues de "Alineacion" porque necesita la transformacion ya calculada
ahi (rotacion + traslacion): la nube original y la nube con shotcrete no
comparten sistema de coordenadas (el sensor se reposiciona entre captura y
captura), asi que un mismo box no selecciona la misma region fisica en las
dos a menos que se sepa como convertir entre ambos sistemas.

En vez de transformar la nube con shotcrete COMPLETA y recien despues
recortarla, esta pestaña hace lo inverso — mucho mas barato cuando el box
final es una fraccion chica de la nube: ubica el box en el sistema de
coordenadas ORIGINAL (crudo) de la nube con shotcrete aplicando la
transformacion inversa solo a las 4 esquinas del box, recorta ahi, y recien
transforma el resultado ya chico. La nube original nunca se transforma (es
el sistema de referencia).

1. **"Elegir 4 puntos en el tunel alineado..."** — abre un visor 3D con la
   nube base. Shift+Click en las 4 esquinas de la region deseada (por
   ejemplo, un tramo especifico del tunel), en orden alrededor del
   perimetro, despues cerrar la ventana (`Q`).
2. **"Ancho del box (cm)"** — profundidad del recorte a lo largo de la
   normal del plano que mejor ajusta esos 4 puntos, centrada en ese plano
   (10 cm por defecto, +/- 5 cm a cada lado). Tiene que ser mayor que el
   espesor de shotcrete esperado, para no cortar la superficie con
   shotcrete (que queda mas cerca del sensor que la superficie original).
3. **"Aplicar segmentacion"** — requiere haber calculado la alineacion
   primero (pestaña "Alineacion"); recorta las dos nubes a los puntos que
   caen dentro del box, guarda `<nombre>_segmento.ply` para cada una, y
   actualiza la pestaña "Comparacion" para usar esos archivos recortados en
   el resto del analisis. **"Quitar segmentacion"** vuelve a usar las nubes
   completas.

### Pestaña "Ajustes de analisis"

**"Analizar solo una zona"** (opcional) — checkbox + boton **"Seleccionar
zona en el visor 3D..."**: Shift+Click sobre 2 o mas puntos que marquen la
zona de interes (por ejemplo, un tramo del tunel), util para que la
estadistica no se diluya con el resto de una escena que no cambio.
**"Opciones avanzadas"** (colapsado) — voxel, filtro de ruido, ICP, coordenadas
manuales de zona, y carpeta de resultados.

### Pestaña "Visualizacion 3D"

**Color del espesor** — escala continua (degrade azul→rojo) o 3 niveles de
color (verde/amarillo/rojo segun umbrales en mm, los mismos que se usan para
la alerta y el informe). **Vista 3D en pantalla** — mostrar/ocultar el
resultado, y elegir si la vista es estatica (ultima captura/archivo) o en
vivo (sensor conectado, redibuja continuamente). Botones **"Abrir vista 3D"**
/ **"Cerrar vista 3D"** (ventana de Open3D aparte).

**Ajustes de captura en vivo (MVP)** — distancia maxima de lectura (m),
angulo de cono/FOV (grados) centrado en un eje frontal (`x`/`y`/`z`), e
inversion de ejes `Y`/`Z` para adaptar la orientacion al montaje fisico del
sensor. Se aplican tanto a la vista en vivo como a "Fijar BASE en vivo".

### Alertas e informe (RF5/RF6)

Al terminar un analisis ("Calcular espesor"), si el espesor medio queda por
debajo del umbral bajo o por encima del umbral alto (configurados en "Color
del espesor" → 3 niveles), aparece una alerta indicando falta o exceso de
shotcrete. El boton **"Generar informe"** (en la barra superior, se habilita
despues de un analisis exitoso) exporta un reporte en Markdown a la carpeta
de resultados, con fecha, archivos analizados, estado (dentro de
parametro/falta/exceso), tabla de estadisticas, y referencias al histograma,
CSV y heatmap generados.

### Pestaña "Comparacion (prueba)" — seccion experimental

Visor 3D **embebido directamente en la ventana** (a diferencia de
"Visualizacion 3D", que abre una ventana de Open3D aparte). Muestra ambas
nubes superpuestas con un resaltado sutil (gris → ambar tenue) donde
difieren, en vez de un heatmap tipo arcoiris. Se controla con el mouse
(arrastrar = orbitar, rueda = zoom). Todavia no esta decidido si esto queda
en la version final — usa la API "clasica" de Open3D (`Visualizer` con
`visible=False`) en vez del renderizador nuevo (`OffscreenRenderer`), porque
este ultimo no soporta modo headless en Windows; en Linux/Mac deberia
funcionar igual con cualquiera de los dos, pero se eligio la clasica por
compatibilidad.

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
