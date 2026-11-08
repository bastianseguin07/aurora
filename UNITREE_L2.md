# Unitree LiDAR L2 en Aurora

Esta integración vive en la rama `feat/unitree-l2`. El worktree de desarrollo es
`/home/nkz/Downloads/aurora-unitree`; `/home/nkz/Downloads/aurora` conserva `main`.
La GUI GTK, los análisis por archivos, los visores y los informes son los mismos.
En **Medición rápida** o **Captura** se puede elegir **Slamtec Aurora** o
**Unitree LiDAR L2**. Cambiar de sensor requiere desconectarlo primero.

## Instalación y compilación

Desde el directorio de esta rama:

```bash
./setup.sh --with-unitree-sdk
./run_gui.sh
```

`setup.sh` prepara Python/GTK y sus dependencias. Los paquetes del sistema
requieren `sudo` cuando faltan. NumPy se selecciona según la versión de Python:
1.x para Python anterior a 3.13 y 2.x para las versiones posteriores. ROS no es
una dependencia de la GUI ni del modo UDP.

Si ya tienes el entorno Python, compila solamente el SDK:

```bash
./install_unitree_sdk.sh
# O indica un clon existente:
./install_unitree_sdk.sh /ruta/al/Proyecto-24CVEK-264744
```

El instalador clona la referencia en `../unitree-reference` si falta, usando un
checkout parcial de `py/`, SDK y paquetes ROS. No descarga las capturas PLY de la
referencia. Usa `g++`, C++17 y la biblioteca estática del SDK para `x86_64` o
`aarch64`; no necesita CMake para estos dos ejecutables. Genera:

- `build/unitree/unitree_bridge`: adquisición binaria usada por Aurora.
- `build/unitree/example_lidar_udp`: ejemplo original compilado como referencia.
- `build/unitree/reference_commit.txt`: revisión del SDK utilizada.
- `build/unitree/LICENSE.unitree`: licencia del código externo enlazado.

Los binarios no se versionan. El ejemplo original conserva IPs fijas y salida de
texto; la aplicación usa el nuevo puente configurable. No ejecutes ambos a la vez.

## Conexión UDP directa

1. Conecta el LiDAR por Ethernet y comprueba su alimentación.
2. Configura la interfaz de red de la PC en la subred del sensor mediante el
   administrador de red. La aplicación no cambia las interfaces ni el firmware.
3. Selecciona **Unitree LiDAR L2**. En **Captura → Conexión Unitree**, selecciona
   **SDK UDP directo** y revisa los datos de tu instalación.
4. Pulsa **Conectar sensor**. Se considera conectado cuando llega una nube
   completa; si no llega en cinco segundos, se cierra el proceso y se muestra el
   motivo y los últimos mensajes del driver.
5. Captura ANTES y DESPUÉS y revisa las nubes antes de calcular.

Al cerrar Aurora o desconectar la sesión, se libera el socket UDP de la PC, pero
no se envía `stopLidarRotation`: el Unitree sigue girando y escaneando. Para
detenerlo, corta su alimentación o usa su control externo.

Valores de referencia encontrados en el repositorio, que debes confirmar:

| Parámetro | Valor inicial |
| --- | --- |
| IP del sensor | `192.168.1.62` |
| Puerto del sensor | `6101` |
| IP local de la PC | `192.168.1.2` |
| Puerto local | `6201` |
| Barridos SDK por nube (`cloud_scan_num`) | `18` |
| Frame | `unilidar_lidar` |
| Timestamps SDK | reloj del sistema |

La IP local debe existir en la PC. El puente comprueba que su IP/puerto UDP estén
libres antes de abrir el SDK y usa un bloqueo para evitar otro puente Aurora en
el mismo puerto. No inicia ROS: mientras usas UDP directo, detén cualquier driver
ROS u otra aplicación que abra ese puerto. En modo ROS, Aurora solo se suscribe
al driver existente y no abre otra conexión UDP al LiDAR.

## Qué se guarda

En el extremo izquierdo de la barra **Captura fija**, **Nativo: una nube** espera
la primera nube completa nueva recibida después de pulsar capturar. Una nube
nativa es un `PointCloudUnitree` completo devuelto por `getPointCloud()`, con la
agrupación `cloud_scan_num` del SDK. No es una línea de stdout, un punto ni un
paquete UDP arbitrario. El valor 18 configura barridos agrupados por el SDK; no
significa 18 segundos ni garantiza una cobertura angular específica.

De **1 a 60 segundos**, **Acumular X s** conserva y concatena los retornos de las
nubes completas recibidas durante ese intervalo. El tiempo se mide con el reloj
monótono de recepción de la aplicación. También procesa las nubes de ese
intervalo que ya estaban en su cola al terminar el tiempo. La delimitación no
corta una nube por la mitad. Sensor, trípode y escena deben permanecer quietos.
No se aplica el consenso por píxel de Aurora, ni mediana, promediado espacial,
voxel, ICP, normalización de coordenadas o reconstrucción de malla al capturar.

Solo se descartan coordenadas inválidas: NaN, infinito o el vector cero. En la GUI,
los filtros de captura parten activos: apertura de 360° y límite lateral Unitree
de ±0,5 m en Y. Así se conservan puntos hacia delante (+X), detrás (-X) y arriba
(+Z), mientras se recortan retornos laterales más lejanos. La distancia máxima
radial queda en 0 (sin límite) y puede cambiarse. La persistencia por píxel no se
aplica a Unitree: debe estar en cero. La antigua “persistencia” medía la duración
de la toma, no la persistencia de cada punto.

Cada captura produce un `.ply` con XYZ y colores de intensidad y un archivo
`<nombre>_unitree.npz` con:

- `fields`: XYZ, intensidad, tiempo relativo y `ring`, conservando los valores.
- `cloud_offsets`: límites de cada nube dentro de la captura acumulada.
- `cloud_stamps` y `cloud_ids`: timestamp y secuencia de cada nube.
  `cloud_stamps_ns` conserva ademas nanosegundos enteros del Header ROS cuando
  existen; -1 indica que la fuente solo entrego un timestamp flotante.
- Frame, unidades, transporte, fuente temporal y agrupación SDK cuando se conoce.

El formato intermedio transporta los escalares de punto como float64 y ring como
uint32, sin redondear la representación textual. Los valores float32 originales
son representables exactamente en float64. Los colores son una representación
de intensidad; no modifican la geometría. Conserva el NPZ junto al PLY para
mantener los campos que Open3D no escribe en su PLY estándar. Al reutilizar un
nombre, los auxiliares incompatibles del otro sensor se archivan con el sufijo
`_superseded_...npz` para no asociar una pose/foto antigua a la nueva nube. El PLY sigue siendo
compatible con el análisis existente.

En ROS se conserva un mensaje `PointCloud2` completo por nube. El puente respeta
endianness, offsets, stride por fila y tipos numéricos. Los campos opcionales no
publicados se marcan como NaN (`intensity`, `time`) o `4294967295` (`ring`). Para
el driver de referencia `time` expresa segundos relativos al timestamp de nube.
En ROS 2, sin un identificador de nube en Header, se usa la secuencia local de
mensajes recibidos. `cloud_scan_num=-1` en el NPZ ROS significa que la agrupación
la controla el driver externo y no fue consultada por la GUI.

En **Medición rápida**, **Cancelar captura** aparece sobre la barra de progreso
solo durante una captura, sin desplazar los controles. Descarta la captura en
curso de ANTES o DESPUÉS: no guarda archivos ni cambia las capturas, los
resultados o la referencia IMU existentes. Se puede volver a capturar enseguida.

**Detener captura** conserva las nubes completas recibidas antes de detener, si
hay alguna. Sin datos válidos se informa el error. La captura tiene un límite de
256 MiB de campos recibidos; si lo supera, falla explícitamente para evitar una
captura incompleta presentada como completa. Reduce la duración en ese caso.

## Tiempo real y decay time

Abre **Herramientas de vista en vivo** o **Visualización 3D**. En **Ajustes de
captura en vivo**, `Decay time` controla exclusivamente los puntos mostrados:

- `0`: muestra solo la última nube recibida; no acumula historia ilimitada.
- `0 < valor <= 60`: conserva las nubes de los últimos X segundos. El valor
  inicial es 5 s en Aurora.
- Con un valor positivo, la escena se vacía cuando expira la última nube aunque
  no lleguen mensajes nuevos. Al desconectar se limpia la vista.

Se usa el instante de recepción, separado del timestamp del sensor y del tiempo
de captura. La ventana visual tiene un límite de 64 MiB; cualquier descarte por
ese límite se informa en el registro. La recepción compartida tiene un máximo de
128 MiB y 4096 mensajes, incluidos mensajes vacios. Captura y visor tienen cursores independientes: mirar en vivo no roba
nubes a la captura. Una captura que se quede atrás y pierda nubes por sobrecarga
falla expresamente. El visor informa pérdidas y continúa.

**Fijar BASE en vivo** toma la última nube nativa, no la unión visual del decay.
Al guardar las nubes en vivo se guardan también sus campos nativos cuando están
disponibles. Cambiar el decay no cambia las capturas ni los archivos de entrada.

Las colas del SDK, UDP y ROS también tienen límites externos a la aplicación:
esto no demuestra ausencia de pérdidas físicas o del middleware. Hay que medir
la tasa real y comprobar el rendimiento con el sensor antes de usarlo en obra.

### Panel 3D persistente

El panel derecho permanece visible al cambiar de página; el separador permite
ajustar su ancho manteniendo la navegación y los controles originales. Muestra
una cuadrícula de 1 m y controles de decay, tamaño de punto, color por intensidad,
altura o distancia, rango de color automático/manual, pausa y limpieza visual.
El ratón conserva órbita, desplazamiento y zoom.

Con Unitree por ROS 2 dibuja todos los puntos de `/unilidar/cloud`, ubicados en
el mapa usando las poses de Point-LIO interpoladas para el tiempo de cada retorno.
No usa la nube reducida por vóxeles de Point-LIO. El dibujo tiene un límite de
300.000 puntos, distribuido por la zona seleccionada y toda la historia del decay;
las capturas y los cálculos conservan sus datos completos.
Cada nueva captura ANTES guardada con pose válida fija el inicio. Al abrir la
GUI se usa un cero visual nuevo en la posición inicial del sensor,
sin cargar una referencia de captura anterior ni reiniciar Point-LIO.
La referencia se renueva al guardar ANTES; DESPUÉS conserva esa referencia.
El origen visual de ANTES también conserva la pose válida al iniciar la captura;
si no se valida una pose promedio para la medición, ese origen sigue disponible
en la sesión para el círculo blanco y la guía. Esto no crea una pose promedio
ficticia para compensar la nube. El contorno blanco se dibuja sobre el amarillo
cuando ambos casi coinciden, manteniendo el mismo diámetro.
Un círculo blanco marca el inicio y un círculo amarillo del mismo tamaño la posición
actual, con solo su eje X, y una línea violeta los une. El panel informa X/Y/Z
y distancia al inicio en centímetros.
Los dos círculos se dibujan sobre la nube para mantenerlos visibles; el recorrido
histórico es opcional y aparece desactivado al iniciar.
Capturar DESPUÉS conserva ese mismo inicio. Un nuevo ANTES reinicia el cero
visual y la guía con su pose recién tomada, manteniendo el mapa de Point-LIO. Al
reiniciarse Point-LIO, cambia el identificador de sesión y se descarta la
referencia anterior; la siguiente captura válida fija el nuevo inicio. Las poses
caducadas se muestran como última posición, sin indicarlas como seguimiento activo.

Sin Point-LIO disponible muestra la nube local del sensor, sin inventar una
posición global. El entorno muestra la escena completa para orientarse; las
capturas siguen aplicando sus filtros propios. La historia visual respeta el
decay configurado. El botón de zona alterna entre la escena completa y los mismos
filtros utilizados al capturar. En Medición rápida se puede seleccionar Arco de
túnel (predeterminado, +X/−X/techo con límite lateral) o Solo pared frontal
(cono de 60° en +X, sin techo vertical ni puntos detrás del sensor).
El seguimiento arranca y se recupera desde la GUI; una captura espera su
inicialización para poder guardar una referencia válida. La guía aplica una
tolerancia visual de 1,5 cm a la posición, sin modificar las poses guardadas.
Estas copias visuales no modifican las nubes PLY/PCD, campos nativos, sidecars,
recortes, cálculo de espesor ni exportaciones existentes.

### Automático por superficies

Medición rápida inicia en **Automático por superficies**. Los tres métodos
manuales siguen disponibles. Cambiar el método conserva el punto elegido y
las dimensiones y ejes del box. Sirve tanto con el perfil Arco como con Pared
frontal; recortar una región sigue siendo útil para elegir qué cambio medir.

El flujo aplica primero la pose IMU/Point-LIO si está activada y ambas capturas
tienen poses compatibles. También corrige movimientos inferiores a la tolerancia
visual de la guía. La transformación se aplica a una copia en memoria; no
reescribe las capturas ni aplica ICP a la superficie modificada. Poses de
sesiones diferentes se rechazan. Sin poses, las nubes deben estar previamente
alineadas en el mismo marco.

Para detectar superficies se usan muestras reales de vóxeles de 1,5 cm, sin
promediar esquinas, y normales PCA a varias escalas. Un plano dominante robusto
puede estabilizar las normales de una cara pequeña. Se forman regiones por
continuidad espacial y orientación compatible, separando esquinas y capas.
Las regiones planas se comparan mediante distancia perpendicular a la cara de
ANTES. Para regiones curvas o rugosas se usa una comparación local inspirada en
M3C2: búsqueda en un cilindro orientado por la normal, soporte común y diferencia
de medianas proyectadas. Es una implementación propia, no el plugin M3C2 ni
una certificación equivalente al método publicado.

Los valores iniciales, editables en Resultados → Parámetros avanzados de
superficies, son radio normal 7 cm, radio de comparación 6 cm, búsqueda ±60 cm
y margen estimado de alineación 1 cm. Se buscan regiones locales a intervalos
de 4 cm, con al menos ocho muestras geométricas por lado y normales compatibles
hasta 20°. Las muestras originales se clasifican y conservan en el diagnóstico;
el muestreo sirve para estimar superficies y no reduce los archivos capturados.
El límite visual de 300.000 puntos del entorno persistente es independiente.

No hay fallback automático a C2C si la asociación falla. Capas alternativas,
bordes, falta de soporte y regiones sin cobertura suficiente se dejan sin
medición. Se comprueba la huella medida de las caras y el soporte local para
evitar extrapolar una superficie hacia zonas sin datos. En superficies curvas
se restringe la comparación a las huellas locales comunes. Las regiones de
ANTES que no pudieron compararse también se conservan como puntos grises.

El cambio firmado es positivo hacia el origen del marco ANTES; las estadísticas
de espesor usan su módulo, como los métodos anteriores. La asociación es
geométrica: no demuestra que un cambio sea material añadido ni identifica
semánticamente una máquina. Dos superficies distintas pero geométricamente
compatibles pueden requerir selección de región o revisión del operador.

La calidad Alta/Media depende del soporte, ángulo y dispersión. El indicador de
variación suma el margen de alineación, la mayor dispersión robusta local y dos
errores estándar combinados de las muestras geométricas. No es un intervalo de
confianza calibrado ni incluye automáticamente todo el error absoluto de rango
o la deriva del mapa. Solo se usan cambios que superan esa variación y, después,
el umbral mínimo de espesor (1 cm por defecto). Las capturas no se reescalan para
forzar un desplazamiento conocido.

Resultados permite elegir mapas de **Espesor**, **Calidad geométrica** y
**Método usado por zona**. Gris significa sin medición confiable; azul tenue,
sin cambio detectable sobre el umbral o la variación estimada. La cobertura,
cantidad de cambios usados y puntos descartados se muestran aparte. Una escena
sin cambios y una escena sin correspondencias tienen estados diferentes; no se
presenta espesor cero como una medición de regiones que no pudieron compararse.

Además del CSV de espesores aceptados, histograma, mapa PLY y PDF, se guardan:

- `surface_comparison_per_point.csv`: todos los puntos DESPUÉS, método, calidad,
  cambio firmado, variación, región, motivo y uso en estadísticas; incluye los
  puntos ANTES sin correspondencia.
- `surface_comparison.npz`: los mismos vectores y máscaras para reprocesamiento.
- `measurement_details.json`: parámetros, cobertura, regiones, correspondencias,
  diccionario de códigos y estadísticas.

Un nuevo ANTES renueva el cero visual y la guía y quita el DESPUÉS anterior de
la selección. Si esa captura no tiene pose, se elimina la referencia previa en
vez de reutilizarla. Cerrar y abrir la GUI limpia capturas seleccionadas,
referencias y resultados; el próximo seguimiento establece un nuevo cero visual.
El sensor y el mapa de Point-LIO pueden seguir funcionando en segundo plano.
Esto no borra los archivos guardados ni resetea físicamente la IMU.

La validación incluye caja con separación conocida cerca/lejos, piso dominante,
paredes rotadas, arco curvo, sobresalientes locales en pared y techo, ruido,
alternativas ambiguas, huecos en ambas capturas, movimiento del sensor con
compensación de pose y reinicios del estado de medición. Las capturas reales
del 8 de noviembre se reprocesaron sin modificar los originales; el script y
los resultados locales están en `output/validacion_superficies/`.

### Medir una cara plana sin atajos al piso o los bordes

La detección automática del box busca una pared frontal: si el piso domina el
primer ajuste, busca otros planos en lugar de usar su normal como profundidad.
Con Unitree la profundidad frontal se mantiene en X, el ancho en Y y el alto
en Z. Los campos muestran su eje después de detectar la superficie. Elegir
Cara plana, Cloud-to-Cloud o Raycasting conserva esa orientación, el punto y
los valores del box; no intercambia coordenadas de las nubes. La selección
manual permite otras superficies: para un techo horizontal la profundidad es
Z y la cara ocupa X/Y. Cambiar de superficie puede cambiar los ejes; cambiar
el método de cálculo no los cambia.

En Medición rápida y Resultados se puede elegir **Cara plana (perpendicular)**
para una región recortada de una pared o de un objeto plano. Cloud-to-Cloud
mide la distancia al punto más cercano de ANTES, que puede pertenecer al piso
o a un lateral. Esa distancia puede ser mucho menor que la separación entre
las dos caras frontales; aumentar los puntos o reiniciar la IMU no corrige esa
elección de superficie.

Cara plana detecta por separado una cara dominante en ANTES y DESPUÉS con
RANSAC y la refina por mínimos cuadrados. Usa 1,5 cm de tolerancia geométrica,
exige al menos 30 puntos y un 60% de soporte por cara, normales compatibles
(hasta 10°) y un 50% de zona común como mínimo. Solo mide puntos de la cara
DESPUÉS que estén dentro de la huella convexa medida de ANTES: no extrapola
fuera del soporte. Las distancias son absolutas, perpendiculares al plano de
ANTES; no son el módulo del desplazamiento libre de un objeto. Los puntos del
piso/laterales y los que no tienen soporte se cuentan aparte del umbral mínimo
de espesor. Las capturas originales se conservan.

Esta opción requiere seleccionar la misma cara en ambas tomas y alinear las
nubes si se movió el sensor. No se debe aplicar como plano único a un arco de
túnel curvo. ICP se rechaza porque podría alinear las dos caras y borrar el
espesor. Las estadísticas, el CSV, el mapa y el PDF usan los mismos puntos y
distancias; `measurement_details.json` registra método, archivos, controles
geométricos y estadísticas. La dispersión del plano no es una certificación de
exactitud absoluta del sensor.

En tres pares guardados de la prueba de caja del 8 de noviembre, usando el
mismo umbral mínimo de 1 cm en ambos métodos, las medianas fueron:

| Capturas ANTES / DESPUÉS | C2C | Cara plana |
| --- | ---: | ---: |
| 003924 / 003952, lejos | 14,9 cm | 27,7 cm |
| 004801 / 004824, cerca | 28,4 cm | 29,9 cm |
| 005012 / 005043, lejos | 13,6 cm | 28,5 cm |

Las poses originales de estos pares difieren entre 0,26 y 0,38 cm. El efecto
principal se reproduce con caras separadas exactamente 30 cm más un piso:
C2C toma el atajo, mientras la medición perpendicular recupera 30 cm tanto al
rotar la escena como al trasladarla. Los 27,7–28,5 cm de los datos reales no se
reescalan a 30 cm: queda una diferencia respecto al desplazamiento manual que
exige evaluar colocación, orientación y error de rango mediante una referencia
física antes de atribuirle una causa única.

## ROS opcional

La máquina de desarrollo tiene **Ubuntu 26.04.1 y Python 3.14**. Se instalaron
ROS 2 Lyrical y RViz en `/opt/ros/lyrical`, y se compiló una adaptación del driver
de referencia en `build/ros2_ws`. Consulta [ROS2_LYRICAL.md](ROS2_LYRICAL.md)
para instalar, compilar y arrancar. ROS 1 no está instalado. El procesamiento de
archivos y la GUI también funcionan sin ROS.

Para usar un entorno ROS que ya pueda ejecutar el driver de referencia:

1. Compila y arranca el driver en ese entorno, siguiendo sus README. Para ROS 1,
   configura `unitree_lidar_ros/src/unitree_lidar_ros/config/config.yaml` y usa
   `roslaunch unitree_lidar_ros run_without_rviz.launch`. El workspace debe estar
   compilado y su `setup.bash` cargado.
2. Para ROS 2, usa el paquete `unitree_lidar_ros2` y su launch en un entorno donde
   sus dependencias estén instaladas. Su launch original también abre RViz; el
   visor propio de Aurora no necesita RViz.
3. Selecciona **ROS 1: driver externo** o **ROS 2: driver externo** en Aurora.
4. En **Python del entorno ROS**, indica el ejecutable de ese entorno, o un
   wrapper ejecutable que cargue su `setup.bash` y ejecute Python. El script
   `scripts/ros_unitree_bridge.py` debe ser accesible desde ese entorno.
5. Revisa el topic de nube, el topic IMU y el frame esperado. En la referencia:
   `/unilidar/cloud`, `/unilidar/imu`, `unilidar_lidar`.
6. Pulsa Conectar. El puente se suscribe al driver; no inicia otro SDK.

Ejemplo de wrapper local, adaptando la ruta y el Python a tu entorno real:

```bash
#!/usr/bin/env bash
source /ruta/al/entorno_ros/setup.bash
exec /ruta/al/python_ros "$@"
```

El puente usa solo biblioteca estándar para empaquetar mensajes; `rospy` o
`rclpy` y `sensor_msgs` son necesarios únicamente en ese Python externo. La GUI
mantiene su propio entorno. Si utilizas un contenedor, monta el proyecto y adapta
el wrapper, las rutas y la red; no basta con apuntar a un Python que no ve ROS.

La IMU ROS se lee como diagnóstico de orientación, con su timestamp y frame,
sin inventar una posición. No activa la compensación SLAM. Un frame recibido
distinto del esperado produce un error: no se aplica una transformación implícita.

## Coordenadas y funciones disponibles

Al abrir una captura individual ANTES o DESPUES, el visor usa fondo oscuro y
el arcoiris de intensidad de RViz. Lee la intensidad nativa del sidecar Unitree
cuando corresponde a los mismos XYZ, o el campo intensity de un PCD. Para
archivos antiguos monocromos utiliza su brillo; si no hay intensidad ni RGB,
colorea por altura. Los RGB de un archivo que ya tiene color se conservan.
El color se aplica a una copia visual: no reescribe la captura ni se usa como
espesor. Las comparaciones de dos nubes conservan su escala de espesor.

Unitree entrega XYZ en metros en un marco derecho: +X se opone a la salida del
cable inferior, +Y está 90° antihorario respecto a +X, +Z sale de la base hacia
arriba. La IMU tiene un desplazamiento respecto al origen LiDAR, aunque sus ejes
son paralelos. Aurora usa X derecha, Y abajo, Z adelante. No se igualan esos
marcos al capturar. El visor y la detección frontal de pared reciben el eje
apropiado; el recorte trabaja en las coordenadas originales.

El LiDAR L2 y una IMU requieren un algoritmo de odometria para estimar traslacion.
La guia de posicion y compensacion entre capturas se habilitan con Point-LIO
por ROS 2; consulta [UNITREE_POSE.md](UNITREE_POSE.md). Sin ese seguimiento,
usa un trípode fijo entre tomas o la alineación
manual por referencias estáticas. No se guardan fotos de profundidad ni poses
ficticias. La selección por píxel de cámara se marca como no disponible; sigue
operativa la selección de puntos 3D, el recorte, la comparación y el PDF.

## Verificación realizada y prueba física pendiente

Ejecuta la suite desde esta rama:

```bash
./venv/bin/python3 -m unittest discover -s tests -v
bash -n setup.sh install_unitree_sdk.sh run_gui.sh
```

Las pruebas cubren protocolo binario, mensajes truncados/fragmentados, campos y
límites de nube, PointCloud2 con stride y endian distintos, PLY y NPZ, captura
nativa y acumulada, cancelación, cierre del proceso sin datos, sobrecarga, ROS
ausente, decay sin mensajes nuevos, independencia de la captura y capacidades.
También se prueba el selector GTK cuando hay pantalla, el eje X del LiDAR y un
flujo de archivos con recorte, alineación, distancia conocida de 2 cm, heatmap y
PDF, ademas de raycasting con Delaunay y Poisson sobre una pared orientada al
eje X del LiDAR. Las pruebas sin pantalla omiten solamente la de widgets GTK. Los puentes ROS 1
y ROS 2 tambien se ejecutan contra modulos simulados. Además se probó el runtime
ROS 2 Lyrical real con un publicador sintético: conexión del puente, XYZ, ring
uint32, timestamp en nanosegundos y acumulación. Esto no sustituye una prueba
con el dispositivo físico.

Se compilaron el ejemplo original y el puente contra la biblioteca real x86_64,
y se ejecutó el puente real en loopback sin sensor para comprobar el timeout y
su cierre. Los mensajes simulados y estas pruebas no acreditan precisión física.

Con el equipo conectado falta comprobar: IPs y firmware reales, tasa de puntos,
agrupación efectiva por nube, intensidad/ring/timestamps del dispositivo, pérdida
de datos bajo carga, ROS real si se usa y resolución del relieve a las distancias
de trabajo. Repite la prueba del cartón a varias distancias y compara las nubes
nativas antes de evaluar la medición de shotcrete.

Referencia inspeccionada: repositorio `iNakzu/Proyecto-24CVEK-264744`, revisión
`7ec53a5` (versión del SDK descrita como 2.0.10). Se reutiliza su SDK externo y se
adapta el flujo de adquisición de `py/tab_captura.py`; no se traslada su GUI Qt ni
sus filtros predeterminados. La configuración de ROS/RViz y los ejes se verificaron
en el código y README de esa revisión.


En esta copia de desarrollo se preparó un venv local que reutiliza, en modo
lectura mediante un archivo `.pth`, los paquetes Python ya instalados en el venv
de Aurora. El puente Cairo faltante se extrajo localmente en `build/gtk-deps`;
no se modificó el entorno del proyecto original. ROS 2 se instaló posteriormente
como paquete del sistema con autorización del usuario. Si mueves o
eliminas el proyecto original, prepara un entorno independiente con `setup.sh`.
