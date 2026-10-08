# Uso del flujo por sector

1. Instala con `bash setup.sh` y abre con `bash run_gui.sh` (o `venv/bin/python3 scripts/gui_gtk.py`) en Ubuntu. La página inicial es **Sesión por sector**.
2. Escribe túnel y sector/estación (por ejemplo pared izquierda del tramo 2). Para cajas de `pcd`, activa **Ensayo con datos de prueba**. Pulsa **Crear sesión**. El modo y la identificación pertenecen a esa sesión; editar los campos después solo prepara una sesión nueva.
3. Conecta la IP del sensor y pulsa **Capturar BASE**, o **Importar BASE**. La BASE queda fija. Mantén el sensor quieto durante cada captura: no hace un barrido con compensación de movimiento.
4. Agrega una etapa DESPUÉS cuando el sector esté accesible. Las capturas y sidecars se copian a la sesión. Puedes cerrar la app y retirarte; para continuar usa **Abrir sesión** y selecciona `sesion.json`. Conserva/copia toda su carpeta, no solo ese JSON.
5. Elige una etapa en la lista. En terreno usa **Alinear etapa con referencias estables**: marca al menos tres puntos no colineales, en el mismo orden, visibles fuera del área cambiada por shotcrete. Confirma que no cambiaron, aplica y revisa el residual. Un residual pequeño no demuestra precisión física. Si las referencias se taparon, no reutilices una foto anterior como sustituto.
6. Si hay fondo/maquinaria en la escena, usa **Elegir zona común del sector** después de alinear. Elige cuatro esquinas de un parche local en el marco BASE y el ancho del box. Revisa que ambas nubes tengan superficie dentro. Esa zona se aplica a todas las etapas y queda fija tras la primera comparación. Para otra zona o BASE crea otra sesión.
7. Selecciona **Respecto de BASE** para distancia acumulada exploratoria o **Respecto de etapa anterior** para comparar el par de etapas consecutivas. Abre **Revisar par en 3D**: referencia gris, actual azul. En terreno confirma manualmente misma zona y ausencia de maquinaria/oclusiones visibles. Esa confirmación no calcula cobertura ni certifica calidad. Ambas etapas usadas necesitan registro. La primera etapa se compara con BASE. No se calcula una capa restando medias acumuladas.
8. Pulsa **Comparar etapa seleccionada**. Se guardan CSV por punto, histograma y mapa de colores en una carpeta única por comparación. Repetir una comparación conserva resultados anteriores. Puedes abrir el último mapa 3D y exportar el informe del sector.

Todo resultado es exploratorio: distancia C2C sin signo, no espesor normal certificado ni diagnóstico automático de falta/exceso. La versión conserva datos de pose, referencias y diagnósticos, pero cobertura de superficie, solape y exactitud en mina siguen pendientes de validación.

Los archivos de la sesión están bajo `output/sesiones/<id>/`: `sesion.json`, `capturas/`, `resultados/`, `informe.md` y `resumen.csv`. La app detecta capturas faltantes/modificadas al abrir o comparar. Si una etapa ya participó en resultados, agrega otra captura para cambiar su registro y conservar trazabilidad.

Las opciones **Herramientas avanzadas** y **Laboratorio** de la barra lateral despliegan las páginas técnicas. El visor móvil es una vista 3D, no AR registrada sobre la cámara. La GUI Windows `gui.py` permanece como alternativa anterior y no contiene este flujo retomable.

En Windows con WSLg, si Wayland no muestra la ventana, puede probarse `GDK_BACKEND=x11 python3 scripts/gui_gtk.py` dentro de Ubuntu. Esto no valida hardware ni corrige la medición. Para los errores `[WARN: COPY MODE]`, revisar WSLg en el equipo.

Para detalle técnico, decisiones de simplificación, datos de las cajas y pendientes de `plan.txt`, consulta [la auditoría](AUDITORIA_AURORA.md).
