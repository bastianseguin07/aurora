# ROS 2 Lyrical en Ubuntu 26.04

ROS 2 Lyrical tiene soporte nativo para Ubuntu 26.04 amd64/arm64. ROS 1 Noetic,
utilizado habitualmente en Ubuntu 20.04, corresponde a otro entorno. Este setup
prepara ROS 2 con RViz para el Unitree L2.

## Instalación

```bash
pkexec ./install_ros2_system.sh
./setup_unitree_ros2.sh
```

La primera orden instala el repositorio oficial ROS y paquetes del sistema;
requiere autenticación de administrador de Ubuntu en una ventana del sistema.
No introduce contraseñas en archivos ni en el chat. La segunda compila como
usuario normal una copia adaptada del driver en `build/ros2_ws`. El clon externo
`../unitree-reference` se conserva sin modificar.

El driver se adapta a C++20 y las cabeceras de ROS 2 Lyrical. No se ejecutan a la
vez el driver ROS y el SDK UDP directo: el script de arranque usa el mismo bloqueo
de puerto que nuestro puente nativo.

## Arranque con RViz

Confirma primero las IPs de tu red y configura la IP local en la interfaz Ethernet:

```bash
./run_unitree_ros2.sh lidar_ip:=192.168.1.62 local_ip:=192.168.1.2
```

Abre RViz con `/unilidar/cloud`, frame `unilidar_lidar`, intensidad y decay de 1 s.
La herramienta **Move Camera** permite girar arrastrando con el botón izquierdo,
desplazarse con el botón central y acercar/alejar con la rueda. El panel **Views**
usa el controlador **Orbit**. La configuración incluye las herramientas para que
los eventos del mouse lleguen al controlador de cámara.
Para ejecutar el driver sin abrir RViz:

```bash
./run_unitree_ros2.sh lidar_ip:=192.168.1.62 local_ip:=192.168.1.2 rviz:=false
```

También admite argumentos `lidar_port:=6101`, `local_port:=6201` y
`cloud_scan_num:=18`. Mantén el sensor quieto si acumulas nubes.

En otra terminal puedes usar:

```bash
source /opt/ros/lyrical/setup.bash
ros2 topic list
ros2 topic hz /unilidar/cloud
ros2 topic info /unilidar/imu
rviz2
```

## Conexión con la GUI

1. Arranca el driver con `run_unitree_ros2.sh`.
2. En Aurora, selecciona Unitree y **ROS 2: driver externo**.
3. En Python del entorno ROS indica el wrapper absoluto
   `/home/nkz/Downloads/aurora-unitree/scripts/ros2_python.sh`.
4. Deja topic `/unilidar/cloud`, IMU `/unilidar/imu` y frame
   `unilidar_lidar`, salvo que hayas cambiado los nombres del driver.
5. Pulsa Conectar. La GUI se suscribe; no abre otra conexión UDP.

El wrapper carga ROS únicamente para el proceso puente. No mezcla el entorno
Python de la GUI con el de ROS. La IMU proporciona orientación; esto no equivale
a una posición SLAM validada y no activa compensación automática de traslación.

## Verificación en esta máquina

Se instalaron ROS 2 Lyrical y RViz y se compiló `unitree_lidar_ros2` en Ubuntu
26.04.1. RViz abrió con OpenGL 4.6 y la suscripción `/unilidar/cloud`. Pasaron las
43 pruebas del proyecto y una prueba con el middleware ROS real y un publicador
sintético en el dominio DDS aislado 137: conexión del puente, XYZ, ring uint32,
timestamps en nanosegundos y acumulación. Puedes repetirla con:

```bash
source /opt/ros/lyrical/setup.bash
./venv/bin/python3 scripts/check_ros2_unitree.py
```

El launch acepta los argumentos documentados y arranca el ejecutable del driver.
En loopback sin sensor permaneció en la inicialización del SDK; no se validó la
publicación de nubes del dispositivo. Esa prueba se detuvo después de 8 segundos.

Referencias oficiales:
- https://docs.ros.org/en/lyrical/Releases/lyrical/supported-platforms.html
- https://github.com/ros-infrastructure/ros-apt-source/releases/tag/1.3.0

## Primera conexión con el sensor físico

Se configuró el perfil NetworkManager `Unitree-L2` en `enp0s31f6`, con IP local
`192.168.1.2/32` y ruta directa `192.168.1.62/32`. Así el Wi-Fi conserva su ruta
de Internet aunque también use la red 192.168.1.x. El sensor respondió al ping
y envió UDP desde `192.168.1.62:6101` a `192.168.1.2:6201`.

El driver publicó nubes a aproximadamente 11.9 Hz. El puente de la GUI recibió
61 nubes y 329269 puntos durante una comprobación de 5 s, con IMU continua.
Se corrigió la conversión del indicador de orientación de NumPy a `bool` de
Python para evitar que el diagnóstico JSON cerrase el puente al recibir IMU real.
La prueba sintética también publica IMU y comprueba la continuidad del proceso.
Queda pendiente validar precisión geométrica y medición de espesor.

Si RViz ya usa el driver, selecciona **ROS 2: driver externo** en la GUI. El modo
**SDK UDP directo** necesita que el driver ROS esté detenido; el bloqueo de
puerto impide abrir dos lectores UDP sobre el mismo sensor.
