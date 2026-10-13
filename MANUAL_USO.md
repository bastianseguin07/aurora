# Manual de uso - Aurora

Este manual describe el flujo actual de la GUI GTK en Ubuntu. La interfaz Windows (`scripts/gui.py`) es un frontend heredado y aun no tiene paridad con las sesiones de sector. Consulta `docs/AUDITORIA_AURORA.md` para ver las diferencias y los limites tecnicos.

## Medicion por sector

1. Abre **Medicion rapida**. Si el sensor esta desconectado, **Capturar antes** lo conecta automaticamente. Si necesitas cambiar la IP o conectarlo manualmente, expande **IP y conexion manual (opcional)**. Para usar PLY existentes, expande **Cargar capturas desde archivo (opcional)**. Si quieres incluir tunel y sector en el informe, expande **Identificacion del lugar (opcional)** y escribe esos nombres.
2. Captura o carga **ANTES**. Una captura de sensor dura 15 segundos; mantenlo inmovil durante toda la lectura. Si cancelas antes, la lectura parcial se descarta. Se crea una sesion nueva y la nube completa queda guardada ahi.
3. Cuando el sector vuelva a estar accesible, captura o carga **DESPUES**. Puedes agregar varias etapas; el sensor se puede retirar mientras trabaja la maquinaria.
4. Alinea cada etapa con al menos tres referencias fisicas estables, no colineales y visibles en ambas nubes. Tras la primera etapa, la sesion reutiliza los puntos elegidos en ANTES; marca los puntos correspondientes en DESPUES y confirma que sigan visibles. Si no se ven, no fuerces el registro.
5. Pulsa **Calcular comparacion**. Desde la segunda etapa puedes comparar con ANTES para cambio acumulado o con la etapa anterior para cambio entre capas. **Ver mapa del resultado 3D** sigue la etapa y el modo seleccionados.
6. Conserva la carpeta de sesion: guarda las nubes originales, el registro, los resultados y el informe. Puedes reabrirla desde **Abrir sesion guardada**.

La app asume que las capturas cubren la misma zona y que esta despejada; no detecta oclusiones ni valida cobertura. La salida es distancia C2C exploratoria, no espesor normal certificado ni criterio para aceptar shotcrete. Para el procedimiento de campo y advertencias, consulta [docs/FLUJO_SECTOR.md](docs/FLUJO_SECTOR.md).

## Secciones de la interfaz

- **Medicion rapida:** captura/importa ANTES y DESPUES, alinea cada etapa desde el boton del flujo, calcula y abre el informe. La alineacion vuelve a esta pantalla con **Volver a Medicion rapida**.
- **Ajustes avanzados de medicion:** dentro de Medicion rapida, permite cambiar voxel, filtro de puntos aislados y carpeta de salida.
- **Vista 3D y celular:** desde la barra lateral inspecciona la nube ANTES de la sesion o el sensor en vivo y comparte la vista por la red local. No registra ni calcula espesor, y no es realidad aumentada; el mapa de comparacion se abre desde Medicion rapida.
- **Zona comun:** despues de alinear una etapa, aparece dentro de **Opciones de comparacion** en Medicion rapida. Es opcional y, una vez definida, se aplica a toda la sesion.

La GUI GTK se inicia en Ubuntu con:

```bash
./venv/bin/python3 scripts/gui_gtk.py
```

La interfaz Windows heredada se inicia con:

```powershell
.\venv\Scripts\python.exe scripts\gui.py
```

---

## Alternativa sin GUI: linea de comandos

Para automatizar o correr en un servidor sin interfaz grafica:

```bash
python scripts/compare_point_clouds.py --base data/base.ply --updated data/updated.ply --visualize
```

Ver `README.md` seccion 5 para la lista completa de flags (`--voxel-size`,
`--remove-outliers`, `--icp`, `--crop-min/--crop-max`, `--max-distance`,
`--output-dir`, `--overlay`). La segmentacion por box orientado es exclusiva
de `gui_gtk.py` por ahora y no tiene flag equivalente en el CLI.
