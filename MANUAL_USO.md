# Manual de uso — Aurora

Guia de la interfaz GTK3 para medir diferencias entre nubes del tunel antes y despues de aplicar shotcrete. La GUI es la misma en Windows y Ubuntu. Consulta [README.md](README.md) para instalarla y [AUDITORIA_GUI.md](AUDITORIA_GUI.md) para los limites y decisiones del proyecto.

## Medicion rapida

1. Abre **Medicion rapida**. Carga dos archivos `.ply` o captura las etapas ANTES y DESPUES desde los botones de esta pagina. La captura abre **Captura** y al guardarla queda asignada a la etapa elegida.
2. Para capturas del Aurora, manten el sensor encendido y el mismo tracking entre tomas. La **Guia de pose IMU** ayuda a volver a la pose ANTES; solo compara la pose guardada, no registra ni transforma las nubes.
3. Si las poses difieren, o no hay pose IMU y sabes que moviste el sensor, alinea DESPUES con **Alineacion por puntos**. Selecciona los mismos puntos fijos en ambas nubes y en el mismo orden. Si las dos nubes no comparten coordenadas y no hay alineacion, no calcules el espesor.
4. Usa **Cloud-to-Cloud** para empezar. La reconstruccion **Poisson** o **Delaunay** es opcional y crea una superficie interpolada; no es una referencia independiente de verdad. Delaunay suele requerir un recorte local en **Segmentacion**.
5. Pulsa **Calcular espesor**. Al terminar, **Resultados** muestra las estadisticas, el registro tecnico y las acciones de visualizacion. Desde ahi puedes generar el informe PDF.

La GUI no precarga automaticamente capturas antiguas. Comprueba que los nombres de ANTES y DESPUES sean del mismo tramo y sesion.

## Resultados

Al iniciar un calculo, la app abre **Resultados**. Alli aparecen el estado, los puntos analizados, los espesores medio/mediano, minimo, maximo y P95, junto con las acciones de visualizacion disponibles y **Generar informe PDF**. Abre **Detalles tecnicos** dentro de **Registro del calculo** para revisar el progreso y los mensajes del metodo usado. Si cambias ANTES o DESPUES, las estadisticas anteriores se limpian y el informe vuelve a quedar desactivado hasta completar otra medicion.

La interfaz calcula un par ANTES/DESPUES por vez y no mantiene un historial de capas. Si se aplican varias capas de shotcrete, elige manualmente las dos capturas que correspondan a cada comparacion y guarda los resultados con nombres distintos.

## Captura

Conecta la PC a la WiFi del Aurora y usa la direccion del sensor (por defecto `192.168.11.1`). Pulsa **Conectar** y espera el indicador conectado. **Capturar tunel original** asigna la toma a ANTES; **Capturar tunel con shotcrete** la asigna a DESPUES. Ambas capturan durante 15 segundos por defecto y luego piden donde guardar el `.ply`. **Detener captura** termina antes con los frames acumulados.

En **Ajustes de captura (opcional)**, la persistencia minima por defecto es `0` (sin filtro de persistencia); valores mayores conservan puntos presentes en mas frames y pueden eliminar puntos reales si el sensor se mueve. El limite de campo de vision y la distancia maxima tambien descartan puntos: dejalos desactivados salvo que tengas claro que la zona de pared medida queda dentro del filtro. Revisa la toma con **Ver ultima captura**.

La captura puede guardar datos auxiliares de pose y referencia junto al archivo. Si no estan disponibles, el `.ply` se puede cargar y alinear manualmente por puntos 3D; la guia IMU requerira pose guardada.

**Herramientas de vista en vivo (opcional)** permite abrir la nube actual, fijar una base en vivo, guardar dos nubes y transmitir la vista a un celular en la misma red WiFi. Es un flujo de monitoreo separado del calculo principal de pares guardados; valida su comportamiento con el sensor real antes de usarlo para decisiones de espesor.

## Guia de pose IMU

Esta pagina sirve antes de capturar DESPUES: muestra la diferencia entre la pose guardada con ANTES y la pose actual del Aurora para ayudar a regresar al mismo lugar y orientacion. Debes mantener el sensor y su tracking activos; si se reinician, dos valores parecidos no garantizan que las nubes compartan el mismo mapa. La guia no mueve puntos ni alinea nubes en software.

Si ya tienes las dos capturas y sus poses son distintas, usa **Alineacion por puntos**. No intentes corregir una toma existente moviendo el sensor: eso requiere volver a capturar.

## Alineacion por puntos (respaldo)

Se usa si el sensor se movio entre las capturas o si no comparten coordenadas. Elige al menos tres puntos fijos comunes en la nube ANTES y en DESPUES, en el mismo orden. Pueden ser marcas o pernos que no cambiaron con el shotcrete. La foto auxiliar puede ayudar cuando un punto queda tapado; si no existe o no corresponde al `.ply`, selecciona **Nube 3D**.

**Calcular alineacion y aplicar** calcula una transformacion rigida, guarda una nube DESPUES alineada y actualiza el archivo activo. El residual en milimetros indica el ajuste de los puntos elegidos, no garantiza por si mismo que toda la pared haya quedado bien registrada. Comprueba visualmente el resultado y no uses referencias cubiertas, movidas o mal identificadas.

## Segmentacion (opcional)

Recorta una zona del tunel para analizar solo un tramo o preparar una region local para Delaunay. La GUI muestra arriba los archivos activos: selecciona cuatro esquinas en la nube DESPUES y define el ancho del box; la misma region se aplica a ANTES. No hay un selector independiente en esta seccion para evitar que use un par distinto al de Medicion rapida. Ambas nubes deben compartir coordenadas; si el sensor se movio, primero aplica una alineacion valida. El recorte escribe nubes derivadas; **Quitar segmentacion** vuelve a las nubes completas. Si cambias cualquiera de los archivos despues de recortar, hay que elegir de nuevo la region.

## Preparar nube de prueba

Los nombres de archivos seleccionados se muestran en amarillo en esta sección, en **Medición rápida** y en **Archivos ANTES / DESPUÉS** para distinguirlos de las etiquetas.

**Preparar nube de prueba** permite cargar ANTES y DESPUÉS, abrir cada nube por separado o ver el par superpuesto (gris = ANTES, celeste = DESPUÉS). Para recortar ambas, confirma que compartan coordenadas, pulsa **Ver ambas y elegir dos esquinas**, y marca con Shift+clic izquierdo dos esquinas opuestas de la misma cara. Aurora toma esa región de la vista combinada y aplica el mismo box a las dos nubes. Configura la profundidad y revisa el par recortado antes de guardarlo. Las dos copias se guardan en la carpeta elegida con nombres separados ANTES/DESPUÉS; los originales se conservan. **Usar el par recortado en Medición rápida** carga ambas copias juntas.

La cara seleccionada debe estar aproximadamente alineada con los ejes XYZ; el método de dos puntos no calcula la orientación de una cara inclinada. La profundidad debe incluir el objeto deseado sin alcanzar objetos vecinos. Este flujo no alinea las capturas; si el sensor cambió de lugar, alinea el par antes de recortarlo.

## Configuracion

**Archivos ANTES / DESPUES** permite elegir manualmente los `.ply` activos. Cambiar una ruta invalida la alineacion aplicada; vuelve a alinear si las capturas no comparten coordenadas.

**Ajustes de analisis** contiene la seleccion de zona, tamano de voxel, eliminacion de puntos aislados, ICP, limites manuales y carpeta de salida. Cloud-to-Cloud mide distancia al vecino mas cercano desde DESPUES hacia ANTES; no determina por si solo si las capturas estan en el mismo marco. ICP puede absorber parte del cambio real de la pared, asi que no lo uses como sustituto automatico de referencias estables.

## Visualizacion 3D

Permite abrir o cerrar la ventana Open3D, superponer el resultado y seleccionar origen fijo o en vivo. Configura el color continuo o los niveles de espesor. Los umbrales de color tambien se usan para las alertas e informe. Los filtros de cada frame en vivo (distancia, cono, eje y orientacion) son independientes de los filtros de captura fija.

## Resultado e informe

El resultado incluye cantidad de puntos y estadisticas (media, mediana, minimo, maximo y percentil 95), ademas de archivos de salida como CSV, histograma y nube heatmap. **Resultados** concentra estas estadisticas, las acciones 3D, el registro tecnico plegable y el boton **Generar informe PDF**. El registro muestra las operaciones y errores para diagnostico.

Las paginas de comparacion embebida y raycasting se conservan como prototipos internos y no estan en la navegacion del operador.

## Alternativa por linea de comandos

```bash
python scripts/compare_point_clouds.py --base data/base.ply --updated data/updated.ply --visualize
```

Consulta las opciones CLI en [README.md](README.md). La alineacion por puntos y la segmentacion por box estan disponibles en la GUI, no como opciones equivalentes de ese comando.
