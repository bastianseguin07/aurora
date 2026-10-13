# Auditoría y versión por sector — 7 octubre 2026

Se revisaron `plan.txt`, instrucciones del repositorio, ambos frontends, todos los módulos propios, los bindings locales del SDK, instalación, documentación, pruebas y datos. No había `AGENTS.md` en el proyecto ni en su carpeta inmediata. Se conservaron los cambios de estilo existentes, `plan.txt` y las dos nubes no versionadas. No se hizo commit ni push ni se eliminaron datos.

## Decisión de producto

Una sesión por túnel y sector/estación, con una BASE inmutable y tantas etapas posteriores como se necesiten. Se puede retirar el sensor entre etapas y retomar la sesión. Cada captura de 15 s debe hacerse con el sensor quieto: `capture_snapshot` acumula coordenadas locales sin compensación de movimiento.

Para volver desde otra posición se necesitan referencias físicas estables visibles en ambas capturas; se conserva la alineación Kabsch existente. La GUI no exige permanecer en el mismo punto durante la aplicación de shotcrete. Las comparaciones de terreno requieren ese registro explícito; una pose parecida no lo sustituye. Hasta caracterizar errores del sensor y registro, el resultado se presenta como distancia exploratoria entre superficies. No se emite una certificación de espesor ni una instrucción de aplicar material basada solamente en C2C.

Las nubes de prueba y las capturas mineras usan el mismo flujo y los mismos requisitos de registro. La GUI asume que la zona esta despejada; no pregunta ni valida oclusiones. No hay un modo de prueba que omita controles. Los originales se copian junto a sus sidecars, se verifican por SHA-256 y se guardan en un paquete de sesion portable. Las capturas del sensor se escriben temporalmente dentro de la sesion y el original temporal se retira despues de validar la copia; los PLY importados no se alteran. No se cambia el algoritmo geometrico existente ni el formato de las salidas C2C.

## Auditoría de cada sección

| Sección | Necesidad / operador | Recorrido de lógica y salidas | Dependencias y evidencia | Decisión |
|---|---|---|---|---|
| Medición rápida → Sesión por sector | Operador: capturar, interrumpir, retomar, comparar etapas | `SectorWorkflowMixin` → `MeasurementSession` → `run_pipeline`; JSON, copias PLY/NPZ, informe MD, resumen CSV y artefactos por comparación | Sustituye el flujo que solo conservaba paths en memoria y un CSV sin reabrir. Pruebas de persistencia, etapas, bloqueo y registro | SIMPLIFICAR; única entrada principal |
| Captura / vista en vivo | Operador captura dentro de sesion; tecnico inspecciona o comparte la vista | Captura ANTES/DESPUES queda en Medicion rapida; la copia durable y su manifiesto viven en la sesion, sin duplicado permanente en `data/`; inspeccion 3D y transmision celular se agruparon en una herramienta | Evita pares de captura fuera de sesion y C2C en vivo sin registro ni validacion | SIMPLIFICAR; captura trazable y herramienta auxiliar combinada |
| Guia de pose SLAM / IMU | Intentaba devolver el sensor a la pose anterior; no registraba las nubes de profundidad | La pose SLAM y la nube densa usan marcos distintos y no garantizan continuidad | Se retiro la pagina, sus ayudas y el visor asociado; el flujo de terreno usa puntos fisicos estables | ELIMINAR DE LA GUI; sidecar solo permanece para raycasting experimental |
| Comparación manual | Elegir dos PLY fuera de una sesión | Se retiró la página manual; ANTES y DESPUÉS se cargan dentro de la sesión por sector | Evita un segundo cálculo sin historial ni artefactos de sesión; el CLI sigue disponible para diagnóstico técnico | ELIMINAR DE LA GUI; una sola entrada de comparación |
| Alineación por referencias | Necesaria para registrar cada etapa | `_pick_alignment_points` selecciona puntos sobre 3D; `_apply_alignment` → Kabsch + RMS → nube alineada | La sesión guarda correspondencias, rotación, traslación, residual y vínculo a BASE. Al reabrir o añadir capas reutiliza las referencias de BASE guardadas; operador revisa que sigan visibles en cada DESPUÉS. Rechaza referencias colineales/no finitas | CONSERVAR; reutilizar BASE, confirmar DESPUÉS |
| Selección de referencias sobre foto | Puede ayudar a ubicar objetos finos si hay imagen de cámara alineada con profundidad | El flujo GTK ahora selecciona puntos directamente en la nube 3D y deja de capturar/generar sidecars de imagen | La profundidad debe existir en cada punto; una imagen sintética podía parecer una foto real y la pantalla duplicaba el método de selección | RETIRAR DE GTK; compatibilidad de sidecars conservada para el frontend heredado |
| Segmentación | Aislar una zona comparable y excluir fondo/maquinaria | Acción en Medición rápida tras alinear → `_pick_segmentation_quad` → `MeasurementSession.set_roi`; `compare` aplica el box persistido a cada par | Aproxima un box local a partir de cuatro puntos; no reconstruye una pared curva. No escribe PLY intermedios ni ejecuta un recorte paralelo. Una zona común editable antes del primer resultado; luego queda fija | CONSERVAR, en flujo principal; zona común por sesión |
| Ajustes de análisis | Técnico: muestreo y filtros | `_build_params` → `preprocess`; voxel y eliminación de aislados. La GUI deja la escala de color en automático | El recorte paralelo por coordenadas y el control ICP fueron retirados de GTK: el recorte común se define una vez desde Medición rápida y el registro usa puntos estables explícitos. Voxel puede perder capas finas; ICP se conserva en el núcleo/CLI para investigación, porque puede absorber shotcrete como desplazamiento | CONSERVAR COMO AVANZADA |
| Inspección 3D y transmisión celular/tablet | Inspección ocasional de nube de la sesión o sensor en vivo y consulta compartida | Una página agrupa `LiveViewer` y `LiveStreamServer` HTTP+WS → `web_static/viewer.html`, QR; nube actual celeste, sin cálculo C2C | Usa la captura ANTES de la sesión abierta, no la última nube global de `data/`; no se alinea ni mide espesor. Stream WebGL local, no AR. Puertos pendientes de probar en terreno | CONSERVAR COMO HERRAMIENTA AUXILIAR; controles unificados |
| Comparación 3D embebida | Repetía la visualización 3D principal y volvía a calcular C2C | Se retiró su página, renderizador GTK/Cairo y coloreado exclusivo; el mapa 3D del flujo de sesión es la vista única | No añadía una medida ni una salida necesaria. Cairo sigue requerido por el recorte de ventana GTK | ELIMINAR; no duplica el visor |
| Raycasting | Investigación de métrica direccional | El algoritmo queda en `pointcloud_core.py`; se retiraron sus controles de la GUI | La normal única no representa normales locales de túnel curvo. Delaunay 2D supone parche casi plano; Poisson puede cerrar huecos. No tiene validación para shotcrete | ELIMINAR DE LA GUI; conservar solo como código de investigación |
| Alertas avanzadas | Visualizar comparación con umbrales | Se retiraron las alertas y la paleta por bandas; el mapa continuo queda solo para resultados de comparación registrada | La media de toda escena no demuestra falta/exceso local ni indica cuánto material aplicar | ELIMINAR; no sugerir conformidad con colores |
| Informe | Trazabilidad para operador / revisión técnica | La sesión exporta MD + CSV automáticamente, con fuentes, modo, fecha, pose, diagnóstico y artefactos | Se retiró el botón PDF duplicado; los nombres `thickness_*` se mantienen por compatibilidad, con significado C2C explicitado | CONSERVAR; informe de sector en inicio |

La comprobación de GUI recorre las páginas activas y comprueba las referencias a métodos/callbacks. No sustituye la ejecución con sensor físico, picking manual real, stream de red o precisión de campo. El render verifica GTK3/X11 en Ubuntu 24.04 mediante WSLg; no valida escalas distintas. La prueba de geometría incluye una bóveda cilíndrica sintética con desplazamientos radiales conocidos de 4 cm, tanto hacia dentro como hacia fuera; valida la distancia C2C en esa geometría ideal y deja visible que la salida sin signo no distingue ambos casos. Esto no valida captura, calibración, rugosidad, oclusión ni exactitud física del Aurora.

## Geometría y calidad: qué se mide hoy

`compute_c2c_distance(updated, base)` usa el vecino más cercano euclidiano. Es una distancia positiva, no dirigida, dependiente del muestreo: un sensor desplazado, huecos, roca rugosa, maquinaria o menor densidad pueden generar distancias sin shotcrete. No recupera automáticamente una normal física ni distingue crecimiento de pérdida de material. Una nube casi idéntica con fondo grande puede diluir la media del objeto.

Respecto de BASE: se compara etapa N con la captura original. Es la distancia acumulada exploratoria. Entre etapas: se compara directamente etapa N con N−1, ambas en marco BASE; no se restan promedios. La diferencia de dos medias sobre conjuntos de puntos distintos no es espesor de una capa.

La sesión comprueba archivos, hashes, superficie no vacía/degenerada, valores finitos, registro de referencias para terreno y zona común cuando se define. Registra cantidad de puntos, límites, mediana/P95 de espaciamiento, C2C inversa y extensión común de cajas. Esos diagnósticos **no miden cobertura de superficie ni demuestran solape**; no se etiqueta un porcentaje falso como confianza. Si falta registro en terreno, se bloquea. Incluso después del registro, el resultado sigue sin validación metrológica. Rechazar referencias colineales o menos de tres puntos es una condición estructural, no una tolerancia de precisión.

No hay evidencia suficiente para fijar distancia máxima, densidad mínima, porcentaje de cobertura, RMS admisible o precisión de espesor universal. Los umbrales deben derivarse de validación por distancia, incidencia, rugosidad, polvo, iluminación, vibración y resolución, con referencia física independiente.

## Evidencia de `pcd`

Los archivos son `.ply`, pese al nombre de la carpeta. No hay sidecars de pose/foto ni espesores físicos de referencia en esa carpeta. No se infiere pose ni unidad física conocida del nombre. Se procesaron como ensayo sin ICP ni registro, preservando originales.

| Dato | Antes `base_capturada_20261006_143616.ply` | Después `updated_capturada_20261006_143645.ply` |
|---|---:|---:|
| Puntos | 1.004.555 | 1.086.045 |
| X mín./máx. (unidades del PLY, interpretadas por pipeline como m) | −4,33883 / 0,60389 | −4,58453 / 0,61578 |
| Y mín./máx. | −2,35202 / 0,17843 | −2,30467 / 0,14342 |
| Z mín./máx. | 0,11914 / 3,51651 | 0,11697 / 3,80622 |
| Espaciamiento NN mediano | 4,028 mm | 4,210 mm |
| Espaciamiento NN P95 | 12,103 mm | 12,628 mm |
| RGB | Sí | Sí |

En la escena completa: C2C media 1,057891 cm; mediana 0,320781 cm; P95 4,664124 cm; máximo 43,845015 cm. En sentido inverso, mediana 0,279979 cm y P95 3,677655 cm. Son estadísticas de toda la escena; no una medida validada de la caja superior. No hay una región objetivo identificada ni altura medida con instrumento externo. La caja solo verifica lectura, procesamiento, exportación y presentación del ensayo.

Reproducción:

```bash
python3 scripts/audit_dataset.py --base pcd/base_capturada_20261006_143616.ply --updated pcd/updated_capturada_20261006_143645.ply --output-dir output/auditoria
```

El diagnostico `output/auditoria/diagnostico_dataset.json` conserva estadisticas y rutas de CSV, histograma y mapa generados para el par original. Corre fuera de las sesiones del operador, sin registro; por diseno no produce una comparacion de terreno ni omite los controles de la aplicacion.

## Sensor, oscuridad y marcos

El código se suscribe a `ENHANCED_IMAGE_TYPE_DEPTH` y obtiene `DEPTHCAM_FRAME_TYPE_POINT3D`. Esa ruta corresponde a cámara de profundidad, no al scan LiDAR 2D. El ejemplo oficial declara coordenadas locales de cámara (Z hacia adelante, X derecha, Y abajo). `capture_snapshot` acumula esas coordenadas sin aplicar pose por frame; la pose guardada es SLAM. Se corrigió la documentación interna que afirmaba que ambos marcos eran el mismo. [SDK: nube de profundidad](https://github.com/Slamtec/aurora_remote_sdk_demo/blob/main/demo/depthcam_view/README.md).

Aurora y Aurora S son modelos diferentes. El primero combina LiDAR, cámaras e IMU; el segundo describe profundidad estéreo con LiDAR opcional. No se identificó el modelo/firmware físico conectado en este entorno. La resolución del mapa 2D tampoco es exactitud del espesor 3D. [Aurora](https://www.slamtec.com/en/aurora/spec), [Aurora S](https://www.slamtec.com/en/aurora-s/spec).

El fabricante anuncia funcionamiento de mapeo/localización Aurora en oscuridad, pero eso no proporciona una especificación de error de profundidad densa bajo las condiciones de esta mina. No hay prueba aquí de precisión estéreo en oscuridad total, polvo, superficies mojadas o roca sin textura. Es necesario ensayar iluminación auxiliar y medir error de profundidad con el modelo exacto; no asumir que la nube densa es LiDAR independiente de luz. Esta conclusión sobre lo que no queda probado es una inferencia del código y del alcance de las especificaciones. [Descripción del fabricante](https://www.slamtec.com/en/aurora).

## Comparación de métodos de trabajo

| Método | Captura / uso | Volver al mismo punto | Maquinaria / oclusiones | Complejidad y confianza |
|---|---|---|---|---|
| Soporte fijo retirado y repuesto en estación | Simple; quieto en cada snapshot | Facilita repetir, sin dejar sensor instalado permanentemente | Se retira al liberar acceso; pérdidas de campo visual requieren repetir sector | Baja, pero repetibilidad mecánica y metrología deben ensayarse; pose sola insuficiente |
| Sectores con referencias protegidas | BASE y etapas cuando cada sector esté accesible | No exige posición exacta; registra ambas capturas en marco común | Tolera retiro; no tolera referencias tapadas ni nube de otra zona | Recomendado para piloto: moderada complejidad, ya reutiliza Kabsch; residual interno no certifica error |
| Barrido completo antes/después | Requiere mosaico/registro de frames y cobertura | Puede cambiar posición si mapa y extrínsecas son coherentes | Maquinaria provoca huecos y accesos incompletos | Alta; snapshot actual no compensa movimiento, no se implementó una falsa captura 360° |
| Transformación automática por SLAM | Potencialmente rápida tras calibración | No exige repetir estación | Requiere continuidad/relocalización probada en escena cambiante | Extrínseca cámara→sensor, sincronía y calidad/continuidad de mapa faltan; no habilitada |
| ICP de toda pared | Fácil de invocar | Tolera desplazamientos pequeños | Material nuevo y maquinaria alteran correspondencias | Puede eliminar espesor al registrar; excluido del flujo principal |
| ICP restringido a referencias sin cambios | Posible refinamiento futuro | No exige repetición exacta | Necesita máscara estable y suficiente solape | La implementación actual no separa máscara de referencia y zona medida; no se declara validada |

No se impone medir siempre pared izquierda antes que techo. Se divide por sectores identificables y se trabaja en el orden permitido por acceso seguro y visibilidad de referencias. La bóveda requiere revisar ángulos de incidencia y oclusiones; un plano único o una zona muy ancha no representa toda la curvatura. Usar varios sectores/puntos de vista es una recomendación de piloto, no un procedimiento minero aprobado.

## Relación con `plan.txt`

| Requisito / hito | Estado observado |
|---|---|
| RF1 espesor preciso usando LiDAR | Pendiente de validación; cálculo actual C2C sobre profundidad de cámara, no metrología LiDAR 3D |
| RF2 mapa de colores en vivo | Retirado del visor auxiliar porque mostraba distancias sin registro; queda pendiente de rediseño y validación |
| RF3 AR | Visor 3D en browser/tablet no es AR: no registra cámara ni pose del dispositivo |
| RF4 reconocimiento de puntos/pernos | Selección manual; SDK contiene módulos de semántica, sin integración específica ni modelo entrenado |
| RF5 falta/exceso | Umbrales exploratorios, no clasificación metrológica validada |
| RF6 interfaz intuitiva e informes | Flujo por sector implementado, sesiones retomables e informe; requiere evaluación con operadores |
| Hito fuente LiDAR | Se identificó la ruta real de datos y el conflicto entre objetivo y profundidad estéreo; falta caracterización de hardware |
| Hito RA / referencias | Alineación manual disponible; RA con dispositivo móvil pendiente |
| Hito mapa y tablet | Visor 3D local y stream disponibles; despliegue tablet/red del sitio pendiente |

Se mantiene la restricción de `plan.txt` de no usar visores sobre los ojos del trabajador. No se afirmó cumplimiento legal ni se añadió hardware de ese tipo. No se elimina AR o reconocimiento del plan por simplificar menús; quedan como objetivos no cumplidos y no como funciones simuladas.

## Inventario y archivos prescindibles

`inventario.csv` enumera cada archivo fuera de `.git`, tamaño, categoría, imports/definiciones y recomendación. Se inspeccionó el código propio y el árbol de dependencias de los bindings. Las carpetas de caché y resultados se inspeccionan por función/formato y metadatos; no se usa el contenido de millones de filas como evidencia de campo.

| Archivo / grupo | Uso / evidencia | Decisión |
|---|---|---|
| `scripts/gui_gtk.py`, `sector_workflow.py`, `measurement_session.py` | Frontend principal, sesiones y trazabilidad | CONSERVAR y simplificar responsabilidades |
| `pointcloud_core.py` | Geometría compartida por GTK, Tk y CLI | CONSERVAR; fórmulas sin modificar; raycasting experimental |
| `aurora_sensor.py` | Único wrapper de conexión/depth/snapshot/pose | CONSERVAR; documentación de marcos corregida |
| `live_viewer.py` | Vista 3D estatica y en vivo | CONSERVAR COMO AVANZADO |
| `pose_alignment_viewer.py` | Gizmo para aproximar la pose SLAM; no registra las nubes | ELIMINADO junto con la pagina IMU |
| `live_stream_server.py`, `web_static/viewer.html` | Stream y visor móvil offline | CONSERVAR COMO AVANZADOS; no AR |
| `embedded_viewer.py` | Renderizador GTK/Cairo que duplicaba el mapa de distancias Open3D | ELIMINADO; el mapa de sesión usa el visor principal |
| Ayudantes sin referencias internas (`stream_frames`, `get_stream_url`, `_save_file_dialog`, `build_heatmap_cloud_banded`) | Bucle de frames duplicado, getter no consultado, selector de guardado reemplazado por guardado automático y paleta retirada | ELIMINADOS tras búsqueda en `scripts/`; se conservan los overrides requeridos del servidor HTTP |
| `gui.py`, `smoke_test_quick_workflow_tk.py`, `setup.ps1`, `setup.bat` | Frontend Windows previo, comprobaciones y setup correspondiente | CONSERVAR como alternativa heredada; no equivalente al flujo por sector GTK |
| `compare_point_clouds.py` | CLI reproducible del pipeline; útil para diagnóstico | CONSERVAR COMO AVANZADO |
| `audit_dataset.py`, `verify_gui_pages.py`, `test_measurement_session.py`, `smoke_test_sector_workflow.py` | Evidencia reproducible de datos, páginas, sesiones y comportamiento | CONSERVAR |
| `smoke_test_quick_workflow.py` | Nombre de comando previo, delega a prueba nueva | CONSERVAR por compatibilidad; eliminar solo al actualizar automatizaciones |
| `python_bindings/setup.py` | Packaging del SDK vendorizado | CONSERVAR; no instala por sí solo el binario ausente |
| `python_bindings/slamtec_aurora_sdk/` (todos los módulos detallados en inventario) | Fallback explícito de `_import_sdk`; módulos se importan desde fachada SDK | CONSERVAR paquete completo. No borrar semántica/recorder/transform/otros por no usarse directamente: pueden ser dependencias internas. Faltan `.so`/`.dll` en copia local; requiere wheel oficial con binario |
| `requirements.txt`, `setup.sh`, `run_gui.sh`, `.gitignore` | Instalación y arranque | CONSERVAR; `setup.sh` corregido para Cairo/Ubuntu 24.04 |
| `README.md`, `MANUAL_USO.md`, `CLAUDE.md` | Entrada/documentación; contenían equivalencias y flujo anterior | ACTUALIZAR; se conserva referencia avanzada con aviso |
| `plan.txt` | Alcance CORFO, RF/hitos | CONSERVAR sin modificar |
| `pcd/*.ply` | Únicos datos disponibles, ensayo de cajas | CONSERVAR sin modificar, no pruebas de mina |
| `data/.gitkeep`, `output/.gitkeep` | Estructura de carpetas versionada | CONSERVAR |
| `__pycache__`, `*.pyc` | Cachés regenerables | PRESCINDIBLES; no se eliminaron |
| `output/**`, `data/*.ply`, `data/*_ref.npz`, `data/*_pose.npz` generados | Capturas/informes/historial; pueden ser evidencia del usuario | Revisar archivado; no borrar automáticamente |
| Paths fijos `/home/miguel/Desktop/gui_gtkV2/...` en GUI | Enlazaban otra sesión de otra máquina si existía | ELIMINADOS del código: selección de sesión reemplaza el default específico |

## Evidencia reproducible y límites de entrega

```bash
python3 scripts/test_measurement_session.py
python3 scripts/smoke_test_quick_workflow.py
GDK_BACKEND=x11 python3 scripts/verify_gui_pages.py
python3 scripts/audit_project.py
python3 -m compileall -q scripts python_bindings
git diff --check
```

Se verificaron 5 pruebas de modelo (portabilidad/integridad, bloqueo de registro, distancia por etapas y zona fija), integración GTK con sensor simulado (captura, varias etapas, reapertura, bloqueos, alineación y recorte), CSS y render de 6 páginas activas. No se verificó conexión al sensor físico, precisión de espesor, detección de tracking perdido, red/tablet real ni trabajo en mina.

Se detectó y corrigió `python3-gi-cairo` faltante al renderizar DrawingArea; `libgl1-mesa-glx` no tiene candidato en Ubuntu 24.04, se reemplazó por `libgl1`. Se normalizaron `setup.sh` y `run_gui.sh` a LF y se agregó `.gitattributes` para conservarlo en checkouts Windows. Se corrigió el manejo de error de `run_gui.sh` que quedaba anulado por `set -e`, y se verificó sintaxis con Bash. WSLg/Wayland produjo capturas de ventana negras; los renders inspeccionados se obtuvieron con X11. El fallback X11 es de ejecución, no modifica la lógica de cálculo.

Próxima validación necesaria: confirmar modelo/firmware y marcos/extrínseca; medir escalones conocidos y roca curva con referencia independiente, repeticiones quietas y recolocadas, distancias/incidencias distintas, oscuridad con/sin iluminación, polvo/agua y oclusiones. Reservar referencias que no se utilicen para ajustar y medir su error después del registro. Derivar de esos datos límites aceptables con el responsable de metrología del proyecto. Si no se logra exactitud útil a escala del shotcrete, cambiar el método/sensor antes de presentar una certificación.
