# Auditoría de la GUI GTK

Revisión de la interfaz en `scripts/gui_gtk.py` para el propósito principal de Aurora: capturar o cargar nubes del túnel antes y después del shotcrete, alinear las referencias cuando sea necesario, calcular el espesor y revisar/exportar el resultado.

## Decisiones por sección

| Sección actual | Decisión | Motivo y ajuste |
| --- | --- | --- |
| Medición rápida | Mantener como entrada principal | Reúne selección/captura, alineación y cálculo. Cloud-to-Cloud queda como opción normal; Poisson y Delaunay siguen siendo alternativas de reconstrucción. Ahora sus vistas del resultado se pueden abrir desde esta sección. |
| Captura | Mantener y reducir el ruido visual | Conectar, capturar antes/después y revisar la última toma son controles centrales. Duración, persistencia y filtros pasan a “Ajustes de captura”; captura en vivo y transmisión pasan a “Herramientas de vista en vivo”. |
| Guía de pose IMU | Mantener con límites explícitos | Guía al operador para volver a una pose mientras el Aurora conserva el tracking. Compara las poses guardadas; no aplica una transformación a las nubes. La comparación solo es válida dentro del mismo tracking. |
| Alineación por puntos | Mantener como respaldo | Sirve cuando no se conserva el tracking o el sensor cambió de lugar. Requiere seleccionar los mismos puntos fijos en el mismo orden; sigue siendo un flujo avanzado. |
| Preparar nubes | Mantener en Flujo de trabajo | Permite revisar ANTES y DESPUES, seleccionar una region comun sobre la vista superpuesta, guardar copias recortadas y cargarlas en Medicion rapida cuando se necesiten. Conserva intactos los originales. |
| Archivos ANTES / DESPUÉS | Mantener bajo Configuración | El selector duplicaba los botones de Medición rápida; queda como acceso secundario para administrar archivos ya existentes. |
| Ajustes de análisis | Mantener bajo Configuración | La selección de zona, reducción de densidad, filtros e ICP afectan el cálculo, pero no hacen falta en el recorrido normal. ICP conserva su advertencia porque puede confundir el espesor con un error de registro. |
| Visualización 3D | Mantener | Es la revisión visual del espesor y permite configurar umbrales e inspeccionar resultados. Los filtros de captura de un archivo y los filtros por frame de la vista en vivo son controles distintos. |
| Comparación embebida y página avanzada de raycasting | Sacar de la navegación de operador | La primera es un prototipo declarado como no final. Las opciones de malla ya se ofrecen plegadas en Medición rápida; sus resultados también se pueden inspeccionar ahí. El código experimental se conserva, pero ya no distrae en la barra lateral. |
| Resultado, informe y detalles técnicos | Mantener y unificar visualmente | Los resultados, el PDF y el registro ayudan a verificar y diagnosticar una medición. El registro tenía un fondo casi negro ajeno a las tarjetas; ahora usa la superficie oscura del tema. El tooltip del informe se corrigió para indicar PDF. |

## Riesgos de funcionamiento que la interfaz debe dejar claros

- La GUI conserva una sola ruta ANTES y una DESPUÉS. No lleva historial de capas: para varias aplicaciones de shotcrete se deben elegir y guardar los pares manualmente; falta definir si el producto debe reportar espesor incremental por capa o acumulado desde la roca original.
- Los sidecars de pose IMU no contienen un identificador de sesión SLAM. Dos poses numéricamente próximas no prueban por sí solas que las nubes compartan mapa; el operador debe conservar encendido el sensor y el tracking.
- La IMU ayuda a repetir la pose, pero no soluciona por sí sola que el túnel o la zona escaneada cambien por el movimiento de maquinaria. Hay que verificar que ambas nubes cubran una zona común y estable; la GUI no demuestra automáticamente que sean exactamente el mismo tramo físico.
- La tolerancia de 0,5 cm y 3 grados es exigente para reposicionar el sensor a mano. Se conserva por ahora para no relajar silenciosamente la calidad; debe validarse en terreno antes de tratarla como una tolerancia operativa definitiva.
- Poisson y Delaunay producen superficies reconstruidas, no una verdad de espesor. Se mantienen como opciones y no reemplazan Cloud-to-Cloud por defecto. Delaunay está pensado para un recorte local casi plano; Poisson puede cerrar bordes abiertos.
- Las capturas anteriores ya no se precargan al iniciar. El operador elige el par de esta medición y evita mezclar automáticamente sesiones distintas.

## Cambios de esta auditoría

- Paneles de registro con fondo, borde, márgenes y tipografía que siguen el tema oscuro.
- Captura avanzada y herramientas en vivo agrupadas en paneles plegables.
- Comparación de archivos movida a Configuración; las páginas experimentales retiradas de la navegación principal.
- Acciones para abrir el mapa de espesor y las mallas después de una medición rápida con malla.
- Rutas personales fijas retiradas de la selección inicial y de la foto de referencia de alineación.
- Tooltip del botón de informe corregido: el archivo generado es PDF.
- La pose IMU se nombra como guía/verificación y, si detecta poses distintas en dos capturas existentes, dirige a alineación por puntos.
- Las opciones de reconstrucción de malla quedan plegadas por defecto para reducir ruido en el flujo inicial.
- README y manual actualizados para reflejar la navegación, el PDF y el flujo actual de captura/alineación.
- El recorte en **Preparar nubes** guarda copias independientes y conserva los archivos originales; el operador carga explicitamente el par recortado en Medicion rapida.
- Al iniciar, los campos ANTES y DESPUÉS quedan vacíos y muestran “Sin seleccionar”; ya no aparentan tener cargados `base.ply` y `updated.ply` inexistentes.
- En la barra superior, el menú queda a la izquierda y “Generar informe” a la derecha, separados para darles jerarquía visual. Se conserva la fuente global.
- Los umbrales de color se validan antes de generar alertas o PDF; entradas no numéricas, infinitas o con orden inválido muestran un aviso en vez de interrumpir el callback. Se aceptan decimales con coma.
- **Preparar nubes** permite cargar cada etapa, inspeccionarlas por separado o superpuestas, marcar dos esquinas en el par y guardar ambas nubes recortadas con el mismo box, sin tocar los originales.
- Estado, estadísticas, acciones de inspección, registro técnico e informe PDF se trasladaron desde la franja inferior a la sección **Resultados**. Los cálculos estándar y de malla muestran sus salidas ahí; cambiar los archivos invalida las estadísticas anteriores y desactiva el informe para evitar mezclar resultados y rutas.

## Prueba con las nubes de Downloads

Se probaron `base_capturada_20261006_143616.ply` y `updated_capturada_20261006_143645.ply` (1.004.555 y 1.086.045 puntos). El flujo Medición rápida de la GUI mostró la confirmación de alineación faltante y, al continuar, completó el cálculo con voxel de 2 cm. El resultado fue media 2,11 cm, mediana 1,15 cm, P95 7,57 cm y máximo 43,85 cm. Los productos (CSV, histograma y heatmap PLY) se escribieron en una carpeta temporal, sin modificar los originales.

Estos archivos no tienen sidecars de pose junto a ellos, por lo que no se pudo verificar si comparten sistema de coordenadas. La cola alta puede incluir falta de alineación, zonas no comunes, geometría de la caja o correspondencias de vecino más cercano; este resultado valida que el flujo corre, no que la medición sea correcta. La estimación Cloud-to-Cloud debe interpretarse sobre una zona común correctamente alineada y su resultado debe contrastarse con una medida conocida.

También se abrió la GUI nativa de Windows para revisar la composición: la barra superior separa menú e informe, los nombres de archivos no simulan una carga inicial y el registro técnico está dentro de la superficie oscura del tema.

## Verificación pendiente en terreno

La GUI se puede comprobar en escritorio, pero la precisión de la guía IMU, la continuidad del tracking y la utilidad de cada reconstrucción Poisson/Delaunay dependen del sensor y de la geometría real del túnel. Esos puntos requieren capturas reales antes de declarar la medición lista para producción.
