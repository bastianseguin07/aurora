# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Que es este proyecto

Aurora mide el **espesor de shotcrete** (concreto proyectado) sobre la pared
de un tunel, comparando dos nubes de puntos 3D `.ply`: una **base** (tunel
antes del shotcrete) y una **actualizada** (mismo tunel despues). Para cada
punto de la nube actualizada se calcula la distancia Cloud-to-Cloud (C2C, via
KD-Tree de Open3D) a su vecino mas cercano en la base; esa distancia
euclidiana no dirigida es la estimacion de espesor. Las nubes pueden venir de
archivos `.ply` existentes o capturarse en vivo con un sensor **Slamtec
Aurora** (LIDAR/SLAM) por WiFi.

Hay dos GUIs de escritorio equivalentes porque GTK3 + Open3D no conviven en
Windows (no hay wheel de GTK3 para pip en Windows y el Python de MSYS2 que si
tiene GTK3 no puede instalar Open3D):

- **Linux/macOS** -> `scripts/gui_gtk.py` (GTK3, interfaz recomendada/demo a clientes).
- **Windows** -> `scripts/gui.py` (CustomTkinter, misma logica, otro toolkit).

Ambas GUIs son solo la capa de interfaz; toda la geometria/pipeline vive en
`scripts/pointcloud_core.py`, compartido tambien por el CLI.

## Comandos

No hay tests, linter ni build configurados en el repo.

```bash
# Setup (crea venv + instala dependencias; Linux/macOS instala tambien GTK3 via apt/brew)
./setup.sh          # Linux/macOS
./setup.ps1          # Windows PowerShell

# Correr la GUI
./venv/bin/python3 scripts/gui_gtk.py     # Linux/macOS
.\venv\Scripts\python.exe scripts\gui.py   # Windows

# CLI sin GUI (mismo pipeline, run_pipeline())
python scripts/compare_point_clouds.py --base data/base.ply --updated data/updated.ply --visualize
# flags utiles: --voxel-size 0.01  --remove-outliers  --icp
#               --crop-min X Y Z --crop-max X Y Z  --max-distance 0.08  --output-dir DIR --overlay
```

`requirements.txt`: `open3d`, `numpy<2.0`, `scipy`, `matplotlib`,
`customtkinter` (solo `gui.py`), `reportlab` (informe PDF), `websockets`
(transmision en vivo), `qrcode[pil]` (QR de la transmision). Los bindings
GTK3 (`gi`/PyGObject) NO estan en requirements.txt — vienen del sistema
(`apt`/`brew`); el venv los hereda solo si se creo con `--system-site-packages`.

## Arquitectura

```
scripts/pointcloud_core.py   <- TODA la logica geometrica (sin GUI), usada por CLI y ambas GUIs
scripts/aurora_sensor.py     <- wrapper del SDK del sensor Slamtec Aurora
scripts/compare_point_clouds.py  <- CLI
scripts/gui_gtk.py           <- GUI GTK3 (clase AuroraGUI, ~3000 lineas)
scripts/gui.py               <- GUI CustomTkinter (equivalente para Windows)
scripts/live_viewer.py       <- ventana Open3D aparte (vista 3D estatica o en vivo)
scripts/pose_alignment_viewer.py <- ventana Open3D "gizmo" para reposicionar el sensor via IMU/SLAM
scripts/embedded_viewer.py   <- visor 3D embebido en la ventana GTK (Gtk.DrawingArea), experimental
scripts/live_stream_server.py <- servidor HTTP+WebSocket para ver la vista en vivo desde un celular
scripts/web_static/viewer.html <- pagina que abre el celular (WebGL propio, sin CDN)
```

### `pointcloud_core.py` — pipeline central

Punto de entrada unico: `run_pipeline(params: PipelineParams, log=print) ->
PipelineResult`. Llamado tanto por el CLI como por `_run_pipeline_worker` en
`gui_gtk.py`. Dentro:

- `load_point_cloud` / `crop_point_cloud` / `preprocess` (voxel downsample +
  remocion de outliers) / `align_clouds` (ICP, opcional).
- **Alineacion por puntos de referencia (Procrustes/Kabsch)** — alternativa a
  ICP: `compute_rigid_transform`, `rigid_transform_rms_error`,
  `apply_rigid_transform`. Usa solo puntos fijos elegidos a mano (pernos,
  marcas), preferible a ICP cuando el shotcrete cubre TODA la superficie
  (ICP en ese caso confundiria espesor real con error de alineacion).
- **Picking interactivo** en un visor Open3D (Shift+Click, cerrar con `Q`):
  `pick_landmark_points` (N puntos sueltos, para alineacion),
  `pick_quad_points` (4 puntos para un box de recorte), `pick_crop_bounds`.
- **Picking de puntos de referencia sobre una foto** (para cuando el punto
  fisico queda tapado por el shotcrete, ver seccion "Foto de referencia..."
  mas abajo): `pick_landmark_points_from_photo` (ventana `matplotlib`, click
  normal en vez de Shift+Click, mismo contrato de retorno que
  `pick_landmark_points`), `save_reference_photo`/`load_reference_photo`/
  `reference_photo_path_for` (sidecar `<nombre>_ref.npz` junto al `.ply`),
  `_nearest_valid_point` (busca el punto 3D valido mas cercano al pixel
  clickeado, porque un objeto fino como la punta de un perno puede no tener
  profundidad exactamente en ese pixel).
- **Pose de referencia del sensor (IMU/SLAM)** — alternativa a la
  alineacion por puntos cuando el sensor no perdio tracking entre capturas
  (ver pestaña "Alineacion IMU" mas abajo): `reference_pose_path_for`,
  `save_reference_pose`/`load_reference_pose` (sidecar `<nombre>_pose.npz`,
  posicion + roll/pitch/yaw en el instante de la captura),
  `local_axes_from_rpy_deg` (ejes propios del sensor — adelante/derecha/
  arriba — expresados en el sistema de coordenadas mundo, a partir de su
  orientacion), `direction_word_pairs_for_axes` (que palabra
  'adelante'/'atras'/'izquierda'/'derecha'/'arriba'/'abajo' le corresponde
  a moverse en +X/-X, +Y/-Y, +Z/-Z del mundo, segun hacia donde miraba el
  sensor al capturar la base). La convencion de ejes propios del sensor
  (adelante=+Y, derecha=+X, arriba=+Z en su marco local) se ajusto a mano
  contra el hardware real para adelante/atras; izquierda/derecha y
  arriba/abajo son la mejor aproximacion disponible pero no estan
  confirmados.
- **Segmentacion por box orientado**: `_quad_box_axes`,
  `crop_cloud_by_quad_box`, `build_quad_box_wireframe`,
  `show_quad_box_preview` — ajusta un plano a 4 puntos elegidos y recorta una
  region con un ancho (cm) a lo largo de la normal. Aplica la transformacion
  **inversa** solo a las 4 esquinas del box (para ubicarlo en el sistema de
  coordenadas crudo de la nube con shotcrete), recorta ahi, y recien
  transforma el resultado ya chico — evita transformar la nube completa.
- **Medicion**: `compute_c2c_distance`, `summarize` -> `DistanceStats`.
- **Coloreado**: `build_heatmap_cloud` (degrade continuo azul->rojo),
  `build_heatmap_cloud_banded` (3 niveles verde/amarillo/rojo por umbrales),
  `build_heatmap_cloud_six_bands` (escala termica de 6 niveles),
  `build_subtle_overlay_cloud` (resaltado gris->ambar, para el visor
  embebido).
- **Salida**: `save_distances_csv`, `save_histogram`.

### `live_stream_server.py` — transmision de la vista en vivo a un celular

Permite ver la vista 3D en vivo desde un celular en la misma red WiFi, con
orbita/zoom real controlados con el dedo (no es un video, es la nube de
puntos real renderizada en el navegador). Dos servidores en threads propios:

- **HTTP** (`http.server` stdlib) sirve `web_static/viewer.html` una vez,
  inyectando el puerto del websocket (`{{WS_PORT}}` -> numero real).
- **WebSocket** (libreria `websockets`, loop de asyncio propio) transmite la
  nube base (una vez, o cuando cambia) y la nube en vivo (cada
  `LIVE_STREAM_INTERVAL_S` = 0.2s) como frames binarios: header de **8 bytes**
  (dos `uint32`: tipo de mensaje, cantidad de puntos — tiene que ser
  multiplo de 4 o el navegador rompe al construir el `Float32Array` del lado
  del cliente) + posiciones float32 + colores float32.
- Downsample independiente por tipo de frame antes de enviar:
  `MAX_STREAM_POINTS_BASE` (40k, se manda pocas veces) vs.
  `MAX_STREAM_POINTS_LIVE` (12k, se repite varias veces por segundo — tiene
  que ser chico para no saturar el WiFi ni el telefono).
- `make_qr_image(url)` genera un QR (via `qrcode` + Pillow) para escanear con
  la camara en vez de tipear la URL a mano.
- `LiveViewer` (`live_viewer.py`) es quien realmente llama a
  `broadcast_base()`/`broadcast_live()` dentro de su loop de render
  (`_run()`), con los mismos puntos/colores que ya calculo para su propia
  ventana Open3D — el stream nunca hace su propio calculo de espesor.
- El HTML (`web_static/viewer.html`) es un renderer WebGL **propio, sin
  three.js ni ninguna CDN**: el sensor Aurora suele estar en su propia red
  WiFi (`SLAMWARE-Aurora-XXXX`) sin salida a internet, asi que cualquier
  dependencia externa rompe en terreno. Tiene un panel de opciones (⚙️,
  arriba a la derecha) para color de fondo, tamaño de punto por nube,
  mostrar/ocultar cada nube y centrar camara — todo del lado del cliente,
  persistido en `localStorage` del navegador (no toca el servidor Python).
- GUI: boton **"Iniciar transmision"** vive en la pestaña **Captura**
  (`gui_gtk.py`, junto a "Iniciar captura en tiempo real"), no en
  "Visualizacion 3D" — a proposito, para que quede al lado de donde
  realmente se abre la vista en vivo. Si hay sensor conectado, arranca la
  vista en modo vivo automaticamente; si no, transmite lo que ya este
  mostrando el visor (modo estatico).

### `aurora_sensor.py` — sensor

SDK oficial no esta en PyPI (se compila a mano desde `py_aurora_remote`, ver
README seccion 3.1). `connect`/`disconnect` (IP, default `192.168.11.1`),
`read_frame_points_and_colors` (un frame, color de camara RGB o por altura si
no hay imagen), `capture_snapshot` (acumula ~15 frames para reducir ruido),
`stream_frames` (generador para vivo). Si el SDK no esta instalado se lanza
`AuroraNotAvailable` con mensaje claro; el resto de la app sigue funcionando
sin sensor (comparar `.ply` ya existentes, GUI, CLI).

`capture_reference_frame(connection, timeout_ms=300, max_distance_m=None)` —
pide un unico frame **organizado** (`len(points) == width*height`) del
sensor y devuelve `(imagen_uint8 HxWx3, point_grid HxWx3)`: la foto de
camara tal cual, mas un grid 3D con el punto correspondiente a cada pixel
(`NaN` donde no hay profundidad valida, mismo criterio de validez que
`read_frame_points`). Es la base de la alineacion por foto (ver
`pointcloud_core.pick_landmark_points_from_photo` y la pestaña "Alineacion"
mas abajo). Devuelve `(None, None)` si el frame no es organizado o no hay
imagen de camara disponible — best-effort, nunca rompe la captura principal
(`capture_snapshot`) que corre en paralelo.

`get_current_pose(connection) -> SensorPose` — pose actual del sensor
(posicion en metros + roll/pitch/yaw en grados) segun su tracking
visual-inercial (SLAM+IMU), en el mismo sistema de coordenadas que las
nubes crudas. Es la base de la pestaña "Alineacion IMU": mientras el
sensor queda encendido y no pierde tracking, esta pose sirve para guiar al
usuario de vuelta a la posicion donde capturo el "Tunel original", sin
necesidad de alinear las nubes por software despues.

### `gui_gtk.py` — GUI GTK3 (clase `AuroraGUI`, ~3000 lineas)

Layout (`_build_layout`, L304): boton **"Calcular espesor"** siempre visible
arriba, panel de resultado siempre visible abajo, tema "Dark Industrial".

**Barra lateral** (`_build_sidebar`, L400) — sidebar propia (reemplaza el
`Gtk.StackSidebar` automatico de GTK) para poder agrupar las pestañas por
proposito en vez de listarlas todas igual, ya que no todas son "pasos" del
flujo:

- **Flujo de trabajo**: Captura, Alineacion IMU (reposicionar sensor,
  opcional pero recomendado si el sensor sigue conectado y con tracking),
  Comparacion, Alineacion (opcional, respaldo por software), Segmentacion
  (opcional) — pasos secuenciales.
- **Configuracion**: Ajustes de analisis, Visualizacion 3D — no son pasos,
  ajustan como se calcula o se ve el resultado, se puede llegar a
  "Calcular espesor" sin pasar por ahi.
- **Experimental**: Comparacion (prueba).

Sincroniza en ambos sentidos con `self.stack` (click en la sidebar cambia de
pagina; un salto de pagina programatico, ej. tras calcular, resalta la fila
correcta en la sidebar) via la señal `notify::visible-child-name`.

Cada pestaña tiene su `_build_*_page()`:

1. **Captura** (`_build_capture_page`, L471) — Todo lo del sensor, separado
   de la comparacion:
   - IP del sensor + boton Conectar/Desconectar con indicador (● verde =
     conectado / rojo = desconectado / amarillo = conectando).
   - **"Capturar tunel original"** / **"Capturar tunel con shotcrete"**
     (`_capture_clicked`, L1761) — snapshot acumulando ~15 frames para
     reducir ruido; al guardar cambia sola a "Comparacion" con el archivo ya
     cargado. Ademas del snapshot principal, intenta (best-effort, via
     `aurora_sensor.capture_reference_frame`) capturar una **foto de
     referencia** y guardarla junto al `.ply` (`_on_capture_done`, L1830) —
     ver "Foto de referencia..." en el paso 3 (Alineacion) mas abajo. Si el
     sensor no entrega un frame organizado o no hay imagen de camara, sigue
     sin foto (no rompe la captura principal).
   - **Captura en tiempo real (MVP)**: "Iniciar captura en tiempo real"
     (`_start_live_capture_clicked`, L1910) abre la vista 3D mostrando en
     vivo lo que ve el sensor. "Fijar BASE en vivo"
     (`_capture_live_baseline_clicked`, L1919) usa el frame actual como
     referencia para medir espesor en vivo sin guardar un `.ply` antes
     (monitoreo continuo mientras se aplica shotcrete); "Quitar BASE en
     vivo" vuelve a la base original cargada. "Guardar nubes en vivo" vuelca
     a disco la base de referencia actual y el frame en vivo mas reciente
     como dos `.ply`.
   - Limites de distancia maxima (m) y angulo de FOV (grados, eje x/y/z) +
     inversion de ejes Y/Z para adaptar al montaje fisico del sensor — se
     aplican tanto a la vista en vivo como a "Fijar BASE en vivo".
   - **"Iniciar transmision"** (`_toggle_stream_clicked`, L2383;
     `_clear_stream_ui`/`_set_stream_qr`, L2368-2381) — arranca
     `live_stream_server.py` y muestra URL + QR (`Gtk.Image` desde un PIL via
     `GdkPixbuf.Pixbuf.new_from_data`) para ver la vista en vivo desde un
     celular en la misma WiFi. Ver seccion `live_stream_server.py` arriba.

2. **Alineacion IMU** (`_build_imu_alignment_page`, L709) — Alternativa a la
   pestaña "Alineacion" (punto 4 mas abajo) que evita tener que alinear las
   nubes por software: en vez de corregir el desajuste despues, guia al
   usuario para que el sensor **nunca pierda tracking** entre las dos
   capturas, devolviendolo a mano a la posicion fisica exacta de la base
   antes de capturar el shotcrete. Solo tiene sentido si el sensor sigue
   conectado y encendido desde la captura de la base (si se apago o perdio
   tracking, hay que usar la pestaña "Alineacion" con puntos de referencia
   en su lugar).
   - Al capturar "Tunel original" con el sensor conectado (pestaña
     Captura), ademas del `.ply` y la foto de referencia se guarda
     automaticamente un sidecar `<nombre>_pose.npz` con la posicion +
     orientacion del sensor en ese instante
     (`aurora_sensor.get_current_pose`, `pointcloud_core.save_reference_pose`).
   - **"Posicion de referencia"** — muestra desde que archivo se cargo esa
     pose, o un aviso si esa captura no tiene sidecar (se hizo sin sensor,
     o es anterior a esta funcion) — en ese caso no hay guia disponible y
     hay que usar "Alineacion" por software.
   - **"Guia en vivo"** (`_imu_poll_tick`, L956, cada 300ms via
     `GLib.timeout_add`) — compara la pose actual del sensor contra la de
     referencia y corrige **un eje a la vez** (X, despues Y, despues Z),
     con tolerancia `IMU_POSITION_TOLERANCE_CM = 0.5` cm por eje (y
     `IMU_ROTATION_TOLERANCE_DEG = 3.0`grados de rotacion). La correccion
     se muestra en palabras ("mover hacia ADELANTE: 12.0 cm"), no en
     ejes X/Y/Z crudos — `pointcloud_core.direction_word_pairs_for_axes`
     traduce cada eje del mundo a la palabra que le corresponde segun
     hacia donde miraba el sensor al capturar la base. Una linea grande
     arriba de todo muestra solo la instruccion del eje que falta corregir
     (el mismo que resalta la flecha de la ventana 3D, ver abajo); cuando
     los tres ejes y la rotacion estan dentro de tolerancia, se marca
     "✓ Sensor en posicion, listo para capturar".
   - **"Ver posiciones en 3D"** (`_open_imu_pose_viewer`, L814) — abre una
     ventana Open3D aparte (`pose_alignment_viewer.py`, clase
     `PoseAlignmentViewer`), deliberadamente **sin la nube de puntos**
     (mostrarla completa no ayuda a ver "hacia donde moverse", solo
     satura la vista): un piso de referencia (grilla), una esfera que pasa
     de rojo a ambar a verde segun la distancia total al objetivo, y un
     **indicador fijo en la esquina superior izquierda** de la ventana —
     una flecha por eje activo (cian=X, magenta=Y, amarillo=Z, mismos
     colores que los cuadraditos ■ junto a "Eje X/Y/Z" en el panel de
     abajo) que se recalcula cada frame proyectando un pixel fijo con la
     intrinseca/extrinseca real de la camara (`_hud_frame`), para que
     quede pegada a esa esquina sin importar si el usuario orbita o hace
     zoom con el mouse — se convierte en un punto verde cuando los tres
     ejes ya estan alineados. Junto a esa ventana se abre tambien un
     **panel flotante GTK** (`_build_imu_hud_window`, L840, siempre
     encima) con los mismos numeros en letra grande, para poder leerlos
     sin volver a mirar la pestaña principal mientras se mueve el sensor
     con las dos manos.
   - Los ejes propios del sensor (que definen que es "adelante") se
     ajustaron a mano contra el hardware real para adelante/atras;
     izquierda/derecha y arriba/abajo son la mejor aproximacion disponible
     pero no estan confirmados — si una palabra no coincide con el
     movimiento fisico real, hay que confiar en el numero (cm) y el color
     en vez de la palabra.

3. **Comparacion** (`_build_comparison_page`, L664) — Solo selecciona los dos
   archivos `.ply` ("Tunel original" / "Tunel con shotcrete"), no calcula
   nada; el resto de la app lee/escribe estos dos paths (`self.base_path`,
   `self.updated_path`). Se actualiza sola tras capturar, alinear o
   segmentar. Cambiar cualquiera de los dos a mano invalida la alineacion ya
   calculada (`self.alignment_applied = False` en `_set_base_path`/
   `_set_updated_path`), porque corresponderia a otro par de archivos.

4. **Alineacion** (`_build_alignment_page`, L699; `_pick_alignment_points`,
   L783; `_apply_alignment`, L837) — Procrustes/Kabsch con puntos de
   referencia manuales, para cuando el sensor se reubico entre capturas y
   ICP no es confiable:
   1. "1. Elegir puntos en el tunel original..." — Shift+Click en cada punto
      de referencia (pernos, marcas, esquinas rigidas) en el orden que se
      quiera, cerrar con `Q`.
   2. "2. Elegir los MISMOS puntos en el tunel con shotcrete..." — mismo
      proceso en la otra nube; la correspondencia entre nubes es **solo por
      orden de seleccion**, no automatica — hay que marcar los mismos puntos
      fisicos en el mismo orden.
   3. "Calcular alineacion y aplicar" — calcula rotacion+traslacion optimas
      (Kabsch) con esos puntos, muestra el **error residual en mm** (alto =
      puntos mal marcados o en distinto orden), aplica la transformacion a
      toda la nube con shotcrete, guarda `<nombre>_alineado.ply`, y
      actualiza "Comparacion" para usar ese archivo.

   **Foto de referencia para puntos tapados por el shotcrete** — cada uno de
   los dos pasos de picking tiene un selector "Nube 3D" / "Foto de
   referencia" (`self.landmark_base_source_photo_rb` /
   `self.landmark_updated_source_photo_rb`). Problema que resuelve: un perno
   de anclaje se ve completo (base y punta) en la nube **antes** del
   shotcrete, pero **despues** solo sobresale la punta, y esa punta es
   demasiado fina para que el sensor de profundidad la resuelva como puntos
   3D limpios — antes era imposible marcar el mismo punto fisico en el
   "despues". La punta si se distingue en la foto de camara que el sensor
   captura junto con la nube. Con "Foto de referencia" activo,
   `_pick_alignment_points` (L783) carga el sidecar `<nombre>_ref.npz` (via
   `pointcloud_core.load_reference_photo`) y abre
   `pick_landmark_points_from_photo` (ventana `matplotlib`, click normal en
   vez de Shift+Click) en vez del visor Open3D; cada click se traduce a la
   coordenada 3D real via el grid de profundidad guardado en el sidecar,
   buscando el punto valido mas cercano si el pixel exacto no tiene
   profundidad. Si no hay sidecar para ese archivo (esa captura se hizo sin
   sensor, o antes de este cambio), muestra un aviso y hay que usar "Nube
   3D". El sidecar se genera solo al capturar con el sensor conectado (ver
   paso 1, Captura); no sigue al `.ply` si se renombra/mueve a mano fuera de
   la app. Las etiquetas de resultado indican el metodo usado, ej.
   `"4 puntos elegidos (foto) ✓"`.

   Se descarto a proposito la alternativa de reconstruir una malla desde la
   nube y aplicarle la foto como textura UV (picking en 3D sobre esa
   superficie texturizada): mas compleja, y depende de que la reconstruccion
   de superficie (Poisson/ball-pivoting) salga bien en un escaneo de tunel
   ruidoso — puede fallar o generar artefactos justo en el punto fino que se
   quiere marcar. Click en foto 2D -> punto 3D es mas simple y no depende de
   esa reconstruccion.

5. **Segmentacion** (`_build_segmentation_page`, L903;
   `_apply_segmentation`, L1031) — Recorte por box, opcional, va despues de
   Alineacion porque necesita esa transformacion (rotacion+traslacion): las
   dos nubes no comparten sistema de coordenadas hasta que se calcula.
   1. "Elegir 4 puntos en el tunel alineado..." — Shift+Click en las 4
      esquinas de la region deseada, en orden alrededor del perimetro,
      cerrar con `Q`.
   2. "Ancho del box (cm)" — profundidad del recorte a lo largo de la normal
      del plano ajustado a esos 4 puntos, centrado en el plano (default 10cm,
      +/-5cm). Debe ser mayor al espesor de shotcrete esperado para no
      cortar la superficie con shotcrete (mas cerca del sensor que la
      original).
   3. "Aplicar segmentacion" — requiere alineacion calculada antes; recorta
      ambas nubes a los puntos dentro del box, guarda `<nombre>_segmento.ply`
      para cada una, y actualiza "Comparacion" para usarlas. "Quitar
      segmentacion" vuelve a las nubes completas.

6. **Ajustes de analisis** (`_build_processing_page`, L1137;
   avanzado en `_build_advanced_expander_page2`, L1172) —
   - "Analizar solo una zona" (checkbox) + "Seleccionar zona en el visor
     3D..." — Shift+Click sobre 2+ puntos que delimitan la zona de interes,
     para que la estadistica no se diluya con el resto de la escena.
   - "Opciones avanzadas" (colapsado): tamaño de voxel, filtro de ruido/
     outliers, ICP (checkbox "Corregir alineacion"), coordenadas manuales de
     zona, carpeta de resultados de salida.

7. **Visualizacion 3D** (`_build_visualization_page`, L1275) —
   - "Color del espesor": escala continua (degrade azul->rojo), **3 niveles**
     (verde/amarillo/rojo, **modo por defecto**) o 6 niveles (escala
     termica), segun umbrales en mm — default **50mm/100mm** (5cm/10cm) —
     los mismos umbrales disparan la alerta y aparecen en el informe.
     `self.color_banded_rb.set_active(True)` se llama recien despues de
     crear `band_revealer`/`six_bands_revealer` (si se llama antes, crashea:
     dispara `_on_color_mode_changed` sobre widgets que todavia no existen).
   - "Vista 3D en pantalla": mostrar/ocultar resultado; estatica (ultima
     captura/archivo) o en vivo (sensor conectado, redibuja continuo).
     Botones "Abrir vista 3D" / "Cerrar vista 3D" (ventana Open3D aparte, via
     `live_viewer.py`).
   - Mismos ajustes de captura en vivo que la pestaña Captura (distancia,
     FOV, inversion de ejes) — comparten estado. El boton de transmision al
     celular NO esta aca (esta en "Captura", ver arriba).

8. **Comparacion (prueba)** (`_build_embedded_test_page`, L1445) — Seccion
   **experimental**, no decidido si queda en la version final. Visor 3D
   **embebido directamente en la ventana** (`embedded_viewer.py`,
   `EmbeddedComparisonViewer(Gtk.DrawingArea)`), a diferencia de
   "Visualizacion 3D" que abre ventana Open3D aparte. Muestra ambas nubes
   superpuestas con resaltado sutil (gris->ambar tenue, fondo blanco) en vez
   de heatmap arcoiris; se controla con mouse (arrastrar = orbitar, rueda =
   zoom). Usa la API "clasica" de Open3D (`Visualizer`, `visible=False`) en
   vez de `OffscreenRenderer`, porque este ultimo no soporta headless en
   Windows.

**Pipeline** (`_run_pipeline_clicked` L2036, `_build_params` L2060,
`_run_pipeline_worker` L2086) — el boton "Calcular espesor" arma un
`PipelineParams` desde el estado actual de todas las pestañas y corre
`run_pipeline` en un worker thread; al terminar, cambia sola a "Visualizacion
3D" y muestra el resultado coloreado.

**Alertas e informe** (`_generate_report`, L2143) — Si el espesor medio queda
fuera de los umbrales bajo/alto (los de "Color del espesor"), se muestra una
alerta de falta/exceso de shotcrete. "Generar informe" — habilitado tras un
analisis exitoso — exporta un **PDF formateado** (via `reportlab`, no
Markdown plano) a la carpeta de resultados: fecha, archivos analizados,
estado (dentro/falta/exceso), tabla de estadisticas, referencias al
histograma/CSV/heatmap generados.

**Vista en vivo — nube base con color original** (`live_viewer.py`) — la
nube base (gris antes) ahora se muestra con su color real (RGB de camara o el
que traiga el `.ply`); ya no se fuerza `paint_uniform_color`.

## Convenciones importantes de este proyecto

- **Nunca alinear/transformar (SLAM, pose transform) una captura cruda en
  silencio.** La alineacion es siempre una accion explicita del usuario
  (pestaña "Alineacion", Procrustes manual con puntos elegidos a mano) — no
  algo que se dispare automaticamente al capturar o cargar una nube.
- Distincion clave a mantener al tocar el pipeline: **Alineacion por puntos
  de referencia** (Procrustes/Kabsch, manual, confiable cuando el shotcrete
  cubre toda la pared) vs. **ICP** (automatico sobre toda la superficie, solo
  recomendable si el fondo estatico domina en cantidad de puntos). No
  reemplazar uno por otro sin que el usuario lo pida.
- **Tres formas de resolver la alineacion entre capturas, no una sola.**
  Orden de preferencia real de uso:
  1. **Alineacion IMU** (pestaña, fisica) — el sensor nunca se apaga entre
     capturas, se lo devuelve a mano a la posicion original guiado por la
     app; si funciona, las dos nubes ya comparten sistema de coordenadas y
     **no hace falta transformar nada por software**.
  2. **Alineacion por puntos de referencia** (Procrustes/Kabsch, pestaña
     "Alineacion") — respaldo cuando el sensor se apago, perdio tracking, o
     la captura es de una sesion anterior sin pose guardada.
  3. **ICP** (checkbox en Ajustes de analisis) — ultimo recurso, solo si el
     fondo estatico domina en cantidad de puntos.

  Las tres coexisten a proposito, no se reemplazan entre si. Si el usuario
  pide "ya no necesito alinear" tras usar Alineacion IMU, es porque esa
  captura en particular no lo necesito (sensor sin cortes de tracking) — no
  es una señal para eliminar o deprecar la pestaña "Alineacion", que sigue
  siendo necesaria como respaldo.
- `gui.py` (Windows/CustomTkinter) y `gui_gtk.py` (Linux-macOS/GTK3) deben
  mantenerse funcionalmente equivalentes: un cambio de comportamiento en el
  pipeline o en una pestaña generalmente aplica a ambas GUIs. Excepcion
  actual conocida: `gui.py` todavia no tiene las pestañas "Alineacion" ni
  "Alineacion IMU" (solo el checkbox de ICP), asi que el picking sobre foto
  de referencia y todo el flujo de reposicionamiento por IMU/SLAM (ver
  secciones `gui_gtk.py` arriba) solo existen en `gui_gtk.py` por ahora —
  no es una regresion, es un gap de paridad preexistente.
- El repo es autocontenido: venv, scripts y `.ply` viven todos dentro de esta
  carpeta `Aurora`. Al capturar con el sensor conectado, cada `.ply` puede
  venir acompañado de un sidecar `<nombre>_ref.npz` (foto de referencia +
  grid de profundidad, ver `pointcloud_core.save_reference_photo`) y de un
  sidecar `<nombre>_pose.npz` (posicion/orientacion del sensor, solo para
  la captura de la base — ver `pointcloud_core.save_reference_pose`) — si
  se copia o comparte un `.ply` capturado, hay que llevarse tambien esos
  sidecars para no perder la opcion de alinear sobre la foto o de usar la
  guia IMU.
- **Header binario del stream (`live_stream_server.py`/`viewer.html`) debe
  quedar en multiplos de 4 bytes.** Un `Float32Array` en JavaScript exige que
  su offset dentro del `ArrayBuffer` sea multiplo de 4, o tira `RangeError`
  — y ese error queda atrapado dentro de `ws.onmessage`, sin romper la
  conexion ni mostrar nada visible mas que en la consola del navegador (bug
  real que costo bastante diagnosticar). Si se agrega un campo al header,
  mantenerlo alineado a 4 bytes en ambos lados (Python `struct.pack` y el
  `DataView`/offsets del JS).
