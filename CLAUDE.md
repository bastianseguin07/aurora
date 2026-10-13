# Instrucciones del proyecto Aurora

## Estado vigente

La interfaz principal es GTK en Ubuntu (`scripts/gui_gtk.py`). La interfaz `scripts/gui.py` es un frontend anterior de Windows: comparte partes del pipeline, pero no la sesion por sector ni el flujo de captura/registro actual. No describirlas como equivalentes.

El operador trabaja en una sesion por sector. La sesion tiene una nube ANTES inmutable, una o mas etapas DESPUES, registro por referencias cuando cambia la posicion, resultados por etapa y un informe persistente. Las nubes de cajas de `pcd/` usan el mismo flujo y no constituyen validacion minera.

## Invariantes tecnicos

- `capture_snapshot` acumula puntos en coordenadas locales de camara durante 15 s y no compensa movimiento. Mantener el sensor quieto durante la captura.
- La pose SLAM no es el marco de la nube densa y no sustituye la alineacion entre etapas. Si el sensor se mueve, usar referencias fisicas estables visibles en ambas capturas.
- C2C es distancia positiva al vecino mas cercano, sensible a muestreo, huecos y desalineacion. Llamarla distancia exploratoria; nunca afirmar espesor normal certificado o recomendar aplicar/aceptar material basandose solo en ella.
- No habilitar comparaciones de terreno sin registro requerido. No asumir porcentaje de cobertura a partir de cantidad de puntos o cajas envolventes.
- No borrar capturas, sesiones ni resultados existentes durante limpieza. Son evidencia potencial del usuario.
- `pointcloud_core.py` comparte geometria con GTK, Tk y CLI. Cambiar formulas requiere validacion especifica; la simplificacion visual no debe alterar el calculo de medicion.

## Estructura actual

- `scripts/sector_workflow.py`: captura/importacion, sesiones, etapas y recorrido del operador.
- `scripts/measurement_session.py`: almacenamiento portable, hashes, registro, comparaciones e informes.
- `scripts/gui_gtk.py`: frontend principal; ajustes de medicion en el flujo por sector e inspeccion 3D auxiliar.
- `scripts/pointcloud_core.py`: pipeline de nube y exportacion.
- `scripts/gui.py`: frontend Tk anterior.
- `scripts/live_viewer.py`, `live_stream_server.py`, `web_static/`: visualizacion y transmision auxiliares.
- `scripts/audit_dataset.py`: ensayo tecnico con `pcd/`; no es camino alternativo de campo.

Mantener la navegacion principal breve. Los ajustes tecnicos del calculo se encuentran dentro de Medicion rapida y la inspeccion 3D tiene acceso directo desde la barra lateral. Evitar duplicar importacion, captura, ejecucion de pipeline o visualizacion. Los metodos experimentales deben etiquetarse como tales y no mezclarse con el resultado por sector.

## Validacion

En Ubuntu, desde la raiz:

```bash
python3 scripts/test_measurement_session.py
python3 scripts/smoke_test_quick_workflow.py
GDK_BACKEND=x11 python3 scripts/verify_gui_pages.py
python3 scripts/audit_project.py
```

La GUI verificada con WSLg/X11 no acredita funcionamiento con hardware real ni precision de campo. Toda afirmacion sobre exactitud requiere contrastar mediciones repetidas con una referencia fisica independiente y el modelo/firmware exacto del sensor.
