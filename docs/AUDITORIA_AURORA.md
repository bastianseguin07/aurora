# Auditoría y versión por sector — 7 octubre 2026

Se revisaron `plan.txt`, instrucciones del repositorio, ambos frontends, todos los módulos propios, los bindings locales del SDK, instalación, documentación, pruebas y datos. No había `AGENTS.md` en el proyecto ni en su carpeta inmediata. Se conservaron los cambios de estilo existentes, `plan.txt` y las dos nubes no versionadas. No se hizo commit ni push ni se eliminaron datos.

## Decisión de producto

Una sesión por túnel y sector/estación, con una BASE inmutable y tantas etapas posteriores como se necesiten. Se puede retirar el sensor entre etapas y retomar la sesión. Cada captura de 15 s debe hacerse con el sensor quieto: `capture_snapshot` acumula coordenadas locales sin compensación de movimiento.

Para volver desde otra posición se necesitan referencias físicas estables visibles en ambas capturas; se conserva la alineación Kabsch existente. La GUI no exige permanecer en el mismo punto durante la aplicación de shotcrete. Las comparaciones de terreno requieren ese registro explícito; una pose parecida no lo sustituye. Hasta caracterizar errores del sensor y registro, el resultado se presenta como distancia exploratoria entre superficies. No se emite una certificación de espesor ni una instrucción de aplicar material basada solamente en C2C.

El modo Ensayo admite archivos sin referencias, siempre identificado como prueba. Los originales se copian junto a sus sidecars, se verifican por SHA-256 y se guardan en un paquete de sesión portable. No se cambia el algoritmo geométrico existente ni el formato de las salidas C2C.

## Auditoría de cada sección

| Sección | Necesidad / operador | Recorrido de lógica y salidas | Dependencias y evidencia | Decisión |
|---|---|---|---|---|
| Medición rápida → Sesión por sector | Operador: capturar, interrumpir, retomar, comparar etapas | `SectorWorkflowMixin` → `MeasurementSession` → `run_pipeline`; JSON, copias PLY/NPZ, informe MD, resumen CSV y artefactos por comparación | Sustituye el flujo que solo conservaba paths en memoria y un CSV sin reabrir. Pruebas de persistencia, etapas, bloqueo y registro | SIMPLIFICAR; única entrada principal |
| Captura | Técnico: conectar, snapshots, filtros y vista en vivo | `_toggle_sensor_connection` → SDK; `_capture_clicked` → `capture_snapshot` → PLY, foto/grid y pose best effort. Vivo → `LiveViewer` | Captura densa vía `DEPTHCAM_FRAME_TYPE_POINT3D`, no lectura de escaneo LiDAR. Frames se acumulan sin pose por frame. La GUI nueva reutiliza ese flujo | CONSERVAR como avanzada; conexión y captura simples en inicio |
| Alineación IMU | Auxiliar para acercarse a una pose anterior | `_refresh_imu_reference` / `_imu_poll_tick` → pose SLAM, HUD y `PoseAlignmentViewer` | La pose no demuestra continuidad del mapa, sincronía ni calidad de nube. 0,5 cm / 3° son umbrales heredados de guía, no tolerancias metrológicas | OCULTAR COMO AVANZADA; no habilita medición de terreno |
| Comparación | Técnico: pares arbitrarios de archivos | `_set_base_path`, `_set_updated_path` → parámetros del pipeline | Selección de paths no guarda historial. Cambiar par invalida alineación. Sustituida para operadores por archivos vinculados a sesión | OCULTAR COMO AVANZADA |
| Alineación por referencias | Necesaria al mover sensor | `_pick_alignment_points` → 3D/foto; `_apply_alignment` → Kabsch + RMS → nube alineada | No requiere que la pared completa siga igual. Sesión guarda puntos, rotación, traslación, residual y vínculo a BASE. Rechaza referencias colineales/no finitas y exige confirmación de estabilidad | CONSERVAR, acceso directo desde la etapa |
| Foto de referencia | Facilitar selección visible | NPZ imagen + grid 3D; picking busca profundidad y puede acudir a píxel vecino | No ve a través de material ni recupera una referencia tapada. No confundir foto sintética con evidencia fotográfica real. Puede seleccionar otra superficie | CONSERVAR COMO AVANZADA; texto corregido |
| Segmentación | Aislar una zona comparable y excluir fondo/maquinaria | `_pick_segmentation_quad` / `_apply_segmentation` → `crop_cloud_by_quad_box` → PLY recortados | Aproxima un box local a partir de cuatro puntos; no reconstruye una pared curva. Sesión conserva una única zona BASE y aplica el mismo recorte a todas las etapas alineadas | CONSERVAR, acceso directo; zona fija tras comparar |
| Ajustes de análisis | Técnico: muestreo, ruido y recorte | `_build_params` → `preprocess`; parámetros avanzados, ICP | Voxel puede perder capas finas. ICP global puede absorber shotcrete como desplazamiento. No está activado en sesiones de operador | OCULTAR COMO AVANZADA |
| Visualización 3D | Revisar nubes/mapa; técnico ajusta colores | `LiveViewer` → C2C, colores, Open3D, fuente estática/viva | Cambiar escala de colores no cambia geometría. Marco fijo requerido en vivo; sin control de movimiento/registro en ese prototipo | CONSERVAR; vista del resultado en inicio, ajustes avanzados |
| Transmisión celular/tablet | Consulta compartida sin accesorios en ojos | `LiveStreamServer` HTTP+WS → `web_static/viewer.html`, QR | WebGL local sin CDN, útil con WiFi sin internet. Es visor 3D; no AR con cámara/pose de dispositivo. Puertos y acceso de red pendientes de probar en terreno | CONSERVAR COMO AVANZADA |
| Comparación embebida | Alternativa de visualización | `_load_embedded_comparison` → C2C + overlay → `EmbeddedComparisonViewer` | Duplica cálculo/vista y combina GTK/Cairo con Open3D. Se encontró dependencia `python3-gi-cairo` faltante; se corrigió setup | OCULTAR EN LABORATORIO; no borrar ahora |
| Raycasting | Investigación de métrica direccional | `_run_raycast_clicked` → Poisson/Delaunay → rayo desde origen o normal global → salidas propias | Normal única del plano no representa normales locales de túnel curvo. Delaunay 2D supone parche casi plano; Poisson puede cerrar huecos. Origen de pose SLAM no se puede usar directamente con nubes locales de cámara | OCULTAR EN LABORATORIO; no adoptar como espesor final |
| Alertas avanzadas | Visualizar comparación con umbrales | `_generate_alerts` → media C2C y umbrales de usuario | Media de toda escena no prueba falta/exceso local. Se corrigieron textos para no afirmar necesidad de material. El flujo de sesiones no usa estas alertas | SIMPLIFICAR y calificar; no diagnóstico de obra |
| Informe | Trazabilidad para operador / revisión técnica | Sesión exporta MD + CSV automáticamente, con fuentes, modo, fecha, pose, diagnóstico y artefactos; PDF avanzado preservado | Resultados originales con nombres `thickness_*` permanecen por compatibilidad, pero significado C2C explicitado. PDF también identifica resultado exploratorio | CONSERVAR; informe de sector en inicio |

La comprobación de GUI recorre las 10 páginas y comprueba las referencias a métodos/callbacks. No sustituye la ejecución con sensor físico, picking manual real, stream de red o precisión de campo. Las capturas de pantalla verifican presentación bajo GTK3/X11 en Ubuntu 24.04 mediante WSLg, con escalas 1× y 2×.

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

El JSON `output/auditoria/pcd_diagnostico.json` apunta a una sesión de ensayo completa con copias, CSV por punto, histograma, mapa e informe. Cada nueva ejecución crea otra sesión y conserva resultados anteriores.

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
| RF2 mapa de colores en vivo | Prototipo disponible; campo/movimiento/registro sin validar |
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
| `live_viewer.py`, `pose_alignment_viewer.py` | Vista 3D y guía de pose avanzada | CONSERVAR COMO AVANZADOS |
| `live_stream_server.py`, `web_static/viewer.html` | Stream y visor móvil offline | CONSERVAR COMO AVANZADOS; no AR |
| `embedded_viewer.py` | Visor alternativo que duplica la vista Open3D | CANDIDATO A ELIMINAR tras elegir un visor; conservar laboratorio ahora |
| `gui.py`, `smoke_test_quick_workflow_tk.py`, `setup.ps1`, `setup.bat` | Frontend Windows previo, comprobaciones y setup correspondiente | CONSERVAR como alternativa heredada; ya no equivalente al flujo por sector GTK |
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

Se verificaron 4 pruebas de modelo (portabilidad/integridad, bloqueo de registro, distancia por etapas y zona fija), integración GTK con sensor simulado (captura, varias etapas, reapertura, bloqueos, alineación y recorte), CSS y render de 10 páginas; el par `pcd` se ejecutó con el pipeline real. No se verificó conexión al sensor físico, precisión de espesor, detección de tracking perdido, red/tablet real ni trabajo en mina.

Se detectó y corrigió `python3-gi-cairo` faltante al renderizar DrawingArea; `libgl1-mesa-glx` no tiene candidato en Ubuntu 24.04, se reemplazó por `libgl1`. Se normalizaron `setup.sh` y `run_gui.sh` a LF y se agregó `.gitattributes` para conservarlo en checkouts Windows. Se corrigió el manejo de error de `run_gui.sh` que quedaba anulado por `set -e`, y se verificó sintaxis con Bash. WSLg/Wayland produjo capturas de ventana negras; los renders inspeccionados se obtuvieron con X11. El fallback X11 es de ejecución, no modifica la lógica de cálculo.

Próxima validación necesaria: confirmar modelo/firmware y marcos/extrínseca; medir escalones conocidos y roca curva con referencia independiente, repeticiones quietas y recolocadas, distancias/incidencias distintas, oscuridad con/sin iluminación, polvo/agua y oclusiones. Reservar referencias que no se utilicen para ajustar y medir su error después del registro. Derivar de esos datos límites aceptables con el responsable de metrología del proyecto. Si no se logra exactitud útil a escala del shotcrete, cambiar el método/sensor antes de presentar una certificación.
