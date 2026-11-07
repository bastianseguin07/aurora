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

Solo se descartan coordenadas inválidas: NaN, infinito o el vector cero. Distancia
y cono son recortes opcionales, apagados inicialmente. La persistencia por píxel
no se aplica a Unitree: debe estar en cero. No se heredan el límite de 1,5 m ni
la eliminación estadística de puntos de la GUI antigua. Su antigua “persistencia”
medía la duración de la toma, no la persistencia de cada punto.

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

**Detener captura** conserva las nubes completas recibidas antes de detener, si
hay alguna. Sin datos válidos se informa el error. La captura tiene un límite de
256 MiB de campos recibidos; si lo supera, falla explícitamente para evitar una
captura incompleta presentada como completa. Reduce la duración en ese caso.

## Tiempo real y decay time

Abre **Herramientas de vista en vivo** o **Visualización 3D**. En **Ajustes de
captura en vivo**, `Decay time` controla exclusivamente los puntos mostrados:

- `0`: muestra solo la última nube recibida; no acumula historia ilimitada.
- `0 < valor <= 60`: conserva las nubes de los últimos X segundos. El valor
  inicial es 1 s, como el RViz de referencia.
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
