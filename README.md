# Aurora: comparacion exploratoria de superficies por sector

Aurora captura o importa nubes PLY para comparar una superficie ANTES y una o mas etapas DESPUES de aplicar shotcrete. La interfaz principal es GTK en Ubuntu. Cada comparacion queda dentro de una sesion retomable con copias de las fuentes, registro, resultados e informe.

La distancia Cloud-to-Cloud (C2C) actual es exploratoria y no dirigida: no es espesor normal certificado, no detecta por si sola perdida o falta de material y no debe usarse para aceptar shotcrete ni tomar decisiones de seguridad.

## Flujo de trabajo

1. En **Medicion rapida**, conecta el Aurora y captura ANTES, o carga un PLY. Una captura dura 15 s; manten el sensor quieto durante ese intervalo.
2. Cuando el sector vuelva a estar accesible, captura o carga DESPUES. Puedes guardar varias etapas en la misma sesion y retirar el sensor entre ellas.
3. Registra cada etapa DESPUES con puntos fisicos estables visibles en ambas nubes. La sesion reutiliza la seleccion de puntos ANTES despues de la primera etapa; marca los mismos puntos en DESPUES y confirma que sigan visibles. La pose SLAM no reemplaza este paso.
4. Compara con ANTES para el cambio acumulado o con la etapa anterior para el cambio entre capas. Revisa el mapa y conserva la carpeta de sesion.

El flujo asume que la zona esta despejada. No detecta oclusiones ni demuestra cobertura de toda la seccion. La alineacion por puntos y la seleccion de una zona comun son herramientas disponibles; el residual de registro tampoco certifica precision fisica.

## Interfaz y compatibilidad

| Entorno | Interfaz | Estado |
|---|---|---|
| Ubuntu 24.04 | `scripts/gui_gtk.py` | Camino principal; GTK, sensor y flujo por sesiones disponibles. Verificado en WSLg/X11, no en una instalacion Ubuntu de campo. |
| Windows | `scripts/gui.py` | Interfaz anterior CustomTkinter. Comparte el pipeline geometrico, pero no ofrece sesiones por sector ni paridad de flujo; no usar como camino operativo equivalente. |
| macOS | Sin camino validado | No se declara soportado para uso de campo. |

## Instalar y abrir en Ubuntu

```bash
./setup.sh
./run_gui.sh
```

Para abrir manualmente despues de instalar:

```bash
./venv/bin/python3 scripts/gui_gtk.py
```

El sensor debe ser accesible por la red indicada en la pantalla. Para evaluar archivos PLY, usa los controles **Cargar** de ANTES y DESPUES: pasan por las mismas verificaciones y guardado de sesion que una captura real.

## Datos y herramientas

Las sesiones se guardan en `output/sesiones/`. Cada carpeta contiene el manifiesto, copias de las capturas, los resultados de comparacion y el informe Markdown/CSV. Conserva esas carpetas como evidencia y respaldo.

- `scripts/compare_point_clouds.py`: pipeline CLI para diagnostico tecnico de pares PLY.
- **Medicion rapida** ofrece la zona comun opcional despues de alinear. Los ajustes avanzados del pipeline estan dentro de Medicion rapida; la inspeccion 3D y la transmision al celular tienen acceso directo desde la barra lateral.
- `pcd/`: datos de cajas para verificar lectura y funcionamiento; no representan ni validan condiciones mineras.

## Comprobaciones

En Ubuntu con las dependencias instaladas:

```bash
python3 scripts/test_measurement_session.py
python3 scripts/smoke_test_quick_workflow.py
GDK_BACKEND=x11 python3 scripts/verify_gui_pages.py
python3 scripts/audit_project.py
```

Estas comprobaciones cubren persistencia, etapas, registro, recorte, navegacion y render GTK. No prueban el sensor fisico, la precision de profundidad en oscuridad, la repetibilidad en terreno ni la exactitud metrologica. Esas validaciones siguen pendientes antes de usar resultados como medicion aceptada.
