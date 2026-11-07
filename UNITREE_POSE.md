# Guia de pose Unitree y compensacion entre capturas

La guia IMU de la GUI utiliza ahora la odometria LiDAR-IMU de Point-LIO para
Unitree. La IMU sola no aporta traslacion. Se necesita el driver ROS 2 activo,
la nube `/unilidar/cloud` y la IMU `/unilidar/imu`.

## Preparar y usar

```bash
./setup_unitree_lio.sh
./run_unitree_ros2.sh lidar_ip:=192.168.1.62 local_ip:=192.168.1.2 rviz:=false
./run_unitree_lio.sh rviz:=false
./run_gui.sh --sensor unitree --transport ros2 --pose-guide
```

Si el driver ya esta activo, conserva ese proceso: no abras otro lector UDP.
En la pagina **Guia de pose IMU** tambien puedes pulsar **Iniciar seguimiento
Unitree LiDAR-IMU**. Manten quieta la carcasa durante la inicializacion; el giro
interno del LiDAR es normal. Point-LIO publica `/unitree_lio/odometry` y la GUI
se habilita cuando recibe posicion reciente. Un seguimiento detenido o con
retraso superior a 0.75 s deja de alimentar la guia.

Captura un nuevo ANTES con el seguimiento activo. Las capturas anteriores sin
pose no pueden recuperar retrospectivamente su posicion. Mantén el sensor quieto
durante cada captura; puedes reubicarlo entre ANTES y DESPUES. La guia muestra
el movimiento necesario para volver a la referencia y el visor 3D conserva su
origen en esa referencia. Con las dos poses guardadas puedes aplicar
**Compensar movimiento automaticamente por pose** o la compensacion de Medicion
rapida. Se genera otro archivo alineado y se conservan los originales.

No reinicies Point-LIO entre las dos capturas: cada arranque tiene un identificador
de sesion distinto y la aplicacion rechaza compensaciones entre sesiones. Esta
integracion usa odometria local; no incluye cierre de bucle ni relocalizacion
entre sesiones. Tampoco habilita la captura movil experimental del Aurora.

## Promedio de pose y coordenadas

La guia se actualiza a 10 Hz y promedia hasta 0.25 s de poses cercanas; cuando detecta un cambio mayor
de 3 cm o 1 grado dentro de esa ventana, muestra la pose reciente. La ventana
puede introducir retardo en movimientos pequenos. Durante una captura fija se
promedian las poses de su intervalo: posicion media y media de cuaterniones.
Si hay cambio sostenido superior a 1 cm o 1 grado, o dispersion de posicion
superior a 1.5 cm, la nube se conserva sin pose utilizable para compensacion.

El promedio no modifica los XYZ ni elimina deriva sistematica. Se guarda la
dispersion observada en `position_spread_m`; es variacion, no una garantia de
exactitud absoluta. Se debe validar el registro contra referencias fisicas
antes de usarlo para estimar espesores pequenos.

Point-LIO estima la pose de la IMU. El adaptador aplica el brazo IMU->LiDAR
`[0.007698, 0.014655, -0.00667]` m, interpola poses que encierran los timestamps
de captura y guarda el resultado en el marco XYZ nativo del LiDAR. Nunca
extrapola una pose faltante. El sidecar `_pose.npz` incluye sesion, marco,
conversion de ejes, timestamp y variacion. Las poses antiguas del Aurora siguen
utilizando su conversion de ejes de camara.

## Implementacion y comprobaciones

El setup usa `dfloreaa/point_lio_ros2` en la revision
`a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7`, basada en Point-LIO para Unitree.
El clon externo no se modifica; se adapta una copia en `build/ros2_lio_ws`.
Las licencias del codigo externo se conservan en ese workspace.

La adaptacion actualiza las cabeceras y CMake para Lyrical, usa ring uint32,
limita colas y longitud del recorrido, corrige el acceso a vectores que solo
reservaban memoria y exige correspondencias geometricas antes de publicar pose.
Se conserva un mapa de 1000 m de lado para un alcance de busqueda de 100 m:
el algoritmo exige lado mayor que tres veces el alcance para no descartar
sus referencias al desplazar el mapa local. El voxel de 0.2 m se aplica solo
al seguimiento, no a las capturas de la GUI.

```bash
./venv/bin/python3 -m unittest discover -s tests -v
source /opt/ros/lyrical/setup.bash
./venv/bin/python3 scripts/check_ros2_unitree.py
```

Las pruebas comprueban interpolacion, brazo IMU-LiDAR, poses caducadas, promedio
sin ocultar un desplazamiento grande, conversion de ejes, compensacion de un
movimiento conocido y rechazo de sesiones distintas. El smoke test ROS publica
nubes, IMU y odometria sinteticas en un dominio DDS aislado.

Fuentes:
- https://github.com/unitreerobotics/point_lio_unilidar
- https://github.com/dfloreaa/point_lio_ros2
- https://github.com/unitreerobotics/unilidar_sdk2
