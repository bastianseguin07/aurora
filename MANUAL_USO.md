# Manual de uso — Aurora

El flujo actual del operador está en [Sesión por sector](docs/FLUJO_SECTOR.md): crear/abrir sesión, BASE fija, varias etapas, referencias estables, zona común, comparar y exportar. El resto de este archivo se conserva como referencia de herramientas avanzadas previas, no como flujo principal.

Los resultados actuales son distancias exploratorias C2C, no espesores certificados. Las menciones heredadas a falta/exceso, puntos tapados, AR o equivalencia GTK/Windows deben interpretarse con las correcciones y límites de la [auditoría](docs/AUDITORIA_AURORA.md). Un visor móvil 3D no es AR registrada; una foto anterior no recupera un punto físico cubierto.

Guia de uso de cada seccion de la aplicacion, pensada para el operador que va
a usar la GUI en terreno o en oficina. Para instalacion y setup, ver
`README.md`. Para detalles tecnicos internos del codigo, ver `CLAUDE.md`.

Hay dos versiones de la interfaz, con las mismas ideas de fondo pero **no
identicas** en pestañas disponibles:

- **`scripts/gui_gtk.py`** (Linux/macOS) — version completa, 7 pestañas.
  Todo este manual describe esta version salvo que se indique lo contrario.
- **`scripts/gui.py`** (Windows) — version equivalente pero mas simple, 3
  pestañas. Al final de cada seccion hay una nota "**En Windows (`gui.py`)**"
  cuando esa parte falta o funciona distinto ahi.

## Flujo general (un solo click)

1. Conseguir las dos nubes `.ply` — capturandolas con el sensor (pestaña
   **Captura**) o eligiendo archivos ya existentes (pestaña **Comparacion**).
2. (Opcional) **Alineacion** si el sensor se reubico entre una captura y
   otra.
3. (Opcional) **Segmentacion** si solo interesa medir un tramo del tunel.
4. (Opcional) **Ajustes de analisis** / **Visualizacion 3D** para afinar como
   se calcula o se ve el resultado.
5. Boton **"▶ Calcular espesor"** (arriba, siempre visible) — corre el
   analisis, muestra las estadisticas como tarjetas, y abre solo la vista 3D
   coloreada por espesor.
6. Boton **"Generar informe"** (al lado, se habilita despues de calcular) —
   exporta un PDF con el resultado.

---

## Pestaña "Captura"

Todo lo relacionado al sensor Slamtec Aurora.

### Conectar el sensor

1. Conecta la PC a la red WiFi del sensor (`SLAMWARE-Aurora-XXXX`).
2. Campo **"Direccion del sensor"** — IP del dispositivo (por defecto
   `192.168.11.1`, normalmente no hace falta cambiarla).
3. Boton **"Conectar"** — el indicador de estado cambia: 🟡 amarillo
   (conectando) → 🟢 verde (conectado) o 🔴 rojo (desconectado/error).

### Capturar una nube fija

- **"Capturar tunel original"** — antes de aplicar shotcrete.
- **"Capturar tunel con shotcrete"** — despues de aplicar shotcrete.

Ambos botones acumulan frames del sensor durante la **"Duracion de la
captura (s)"** (15s por defecto) para reducir ruido, y filtran puntos segun
la **"Persistencia minima de puntos"**: un punto debe aparecer en al menos
esa fraccion de los frames capturados para conservarse (valores altos,
0.6-0.8, descartan mejor el polvo en el aire, que no aparece siempre en el
mismo lugar; un valor muy alto puede descartar tambien puntos reales si el
sensor tiembla). **"Detener captura"** corta antes de tiempo y procesa lo
acumulado hasta ese momento.

Opcional — **"Limitar campo de vision"**: descarta puntos fuera de un cono
(en grados) centrado en un eje frontal (x/y/z), util para capturar solo una
zona de prueba sin lo que hay detras o a los costados. **"Distancia maxima
(m)"** descarta puntos mas lejos que ese valor (vacio = sin limite).

Al terminar, se abre un dialogo para elegir donde guardar el `.ply`; la app
cambia sola a la pestaña "Comparacion" con ese archivo ya cargado.

**"Ver ultima captura: ..."** — abre en una ventana 3D aparte lo que se
acaba de capturar, para chequear que salio bien antes de seguir.

> **Foto de referencia automatica:** al capturar con el sensor conectado, la
> app tambien intenta guardar una foto de camara junto al `.ply` (archivo
> `<nombre>_ref.npz` en la misma carpeta). Esta foto se usa despues en la
> pestaña "Alineacion" para marcar puntos de referencia que quedan tapados
> por el shotcrete (ver mas abajo). No hace falta hacer nada especial para
> esto — pasa solo, y si no se puede (sensor sin camara disponible en ese
> momento), la captura principal sigue igual, solo que sin esa opcion extra
> mas adelante.

### Captura en tiempo real (MVP)

Para monitorear el espesor mientras se aplica el shotcrete, sin tener que
guardar archivos `.ply` primero:

1. **"Iniciar captura en tiempo real"** — abre una vista 3D que se redibuja
   continuamente con lo que el sensor esta viendo.
2. **"Fijar BASE en vivo (MVP)"** — usa el frame actual como referencia
   ("antes"); a partir de ahi, la vista muestra el espesor actualizandose en
   vivo a medida que se rocia shotcrete. **"Quitar BASE en vivo"** vuelve a
   usar la nube base original cargada (si habia una).
3. **"Guardar nubes en vivo"** — si en algun momento se quiere conservar un
   par de nubes para el flujo normal (analisis, informe), vuelca a disco la
   base de referencia actual y el frame en vivo mas reciente como dos
   `.ply`.

Los campos **"Limitar campo de vision"** / **"Distancia maxima"** de mas
arriba tambien aplican a la vista en vivo.

### Transmitir la vista a un celular

**"Iniciar transmision"** abre un servidor local en la PC y muestra una URL
+ codigo QR. Con el celular conectado a la **misma red WiFi**, escanealo con
la camara (o entra la URL a mano en el navegador): se puede orbitar y hacer
zoom con el dedo, viendo la nube de puntos real (no un video). Si la vista
en vivo todavia no estaba abierta, se abre sola. **"Detener transmision"**
la corta.

> **En Windows (`gui.py`):** esta seccion existe igual, en la pestaña "Datos
> y sensor". La captura en tiempo real (MVP) con "Fijar BASE en vivo" **no**
> esta disponible ahi — solo "Capturar nube BASE"/"Capturar nube
> ACTUALIZADA" (snapshots), conectar/desconectar, y la transmision al
> celular.

---

## Pestaña "Comparacion"

Solo selecciona los dos archivos que se van a comparar — no calcula nada
por si sola:

- **"Tunel original"** — nube base (antes del shotcrete).
- **"Tunel con shotcrete"** — nube actualizada (despues).

Cada uno tiene un boton **"Elegir archivo..."**; el nombre del archivo
elegido se muestra ahi (la ruta completa aparece como tooltip al pasar el
mouse). Esta pestaña se actualiza sola despues de capturar, alinear o
segmentar — normalmente no hace falta tocarla a mano salvo para elegir
archivos `.ply` ya existentes. Cambiar cualquiera de los dos archivos a mano
invalida una alineacion ya calculada (hay que rehacerla).

> **En Windows (`gui.py`):** no es una pestaña aparte, esta junta con
> "Captura" en la pestaña "Datos y sensor" (seccion "Archivos").

---

## Pestaña "Alineacion" (opcional)

Solo hace falta si el sensor **se reubico** entre la captura original y la
de shotcrete (no quedaron en la misma posicion exacta). Alinea eligiendo a
mano 3 o mas **puntos de referencia fijos** — por ejemplo cabezas de pernos
de anclaje, marcas, esquinas rigidas — que no se movieron entre capturas.
Distinto de ICP: ICP ajusta automaticamente toda la superficie, lo que
puede confundir el espesor real con error de alineacion si el shotcrete
cubre TODA la pared; esto usa solo los puntos fijos que elijas.

### Paso 1: elegir los mismos puntos en ambas nubes

1. **"1. Elegir puntos en el tunel original..."** — Shift+Click sobre cada
   punto de referencia, en el orden que quieras, despues cerrar la ventana
   (tecla `Q`).
2. **"2. Elegir los MISMOS puntos en el tunel con shotcrete..."** — mismo
   proceso, marcando los **mismos puntos fisicos en el mismo orden** (la
   correspondencia entre nubes es solo por orden de seleccion, no
   automatica — si te salteas el orden, la alineacion sale mal).

Junto a cada uno de estos dos botones hay un selector **"Nube 3D" / "Foto de
referencia"** — ver la seccion siguiente.

### Foto de referencia: para puntos tapados por el shotcrete

Un perno de anclaje se ve completo (base y punta) en la nube **antes** del
shotcrete. Pero **despues**, el shotcrete tapa la base y solo sobresale la
punta — y esa punta es demasiado fina para que el sensor de profundidad la
resuelva como puntos 3D limpios, asi que antes era imposible marcar el mismo
punto fisico en el "despues".

La solucion: elegir "**Foto de referencia**" en vez de "Nube 3D" para ese
paso. En vez de abrirse un visor 3D, se abre la **foto real** que el sensor
capturo junto con esa nube (donde la punta del perno SI se distingue a
simple vista). Se hace click normal (no Shift+Click) sobre cada punto, en
orden, y se cierra la ventana para terminar — la app traduce cada click a
la coordenada 3D real por debajo.

Requisitos:

- Esa nube tiene que haberse **capturado con el sensor conectado** (ver nota
  de "Foto de referencia automatica" en la pestaña Captura). Si el archivo
  es un `.ply` externo, o se capturo antes de tener esta funcion, no va a
  existir la foto y la app avisa — en ese caso usa "Nube 3D" para ese paso.
- Si el `.ply` se copia o renombra **a mano** fuera de la app, hay que
  llevarse tambien su archivo `<nombre>_ref.npz` de al lado, o se pierde la
  opcion de foto para ese archivo.

Recomendacion practica: usar "Nube 3D" para el paso 1 (antes del shotcrete,
donde el perno completo ya se ve bien en 3D) y "Foto de referencia" para el
paso 2 (con shotcrete, donde solo la punta es visible). La etiqueta junto a
cada boton indica que metodo se uso, ej. `"4 puntos elegidos (foto) ✓"`.

### Paso 2: calcular y aplicar

**"Calcular alineacion y aplicar"** — calcula la rotacion y traslacion
optimas (algoritmo de Kabsch) usando los puntos elegidos, y muestra el
**error residual en mm** (un valor alto indica que se marcaron puntos mal o
en distinto orden entre los dos pasos — conviene rehacer la seleccion).
Aplica la transformacion a la nube con shotcrete, la guarda como
`<nombre>_alineado.ply`, y actualiza la pestaña "Comparacion" para usar ese
archivo de ahi en adelante.

> **En Windows (`gui.py`):** esta pestaña **no existe todavia**. La unica
> forma de corregir desalineacion ahi es el checkbox "Alinear con ICP antes
> de medir" (pestaña "Procesamiento"), con las limitaciones de ICP ya
> explicadas arriba.

---

## Pestaña "Segmentacion" (opcional)

Recorta ambas nubes a una sola zona del tunel (por ejemplo, un tramo
especifico). Va **despues** de Alineacion porque necesita esa transformacion
ya calculada: la nube original y la de shotcrete no comparten sistema de
coordenadas hasta ese momento (el sensor se reposiciona entre captura y
captura), asi que un mismo box no selecciona la misma region fisica en las
dos si no se sabe convertir entre ambos sistemas.

1. **"Elegir 4 puntos en el tunel alineado..."** — Shift+Click en las 4
   esquinas de la region deseada, en orden alrededor del perimetro, cerrar
   con `Q`.
2. **"Ancho del box (cm)"** — profundidad del recorte a lo largo de la
   normal del plano que mejor ajusta esos 4 puntos, centrada en ese plano
   (10cm por defecto, +/-5cm). Tiene que ser **mayor** al espesor de
   shotcrete esperado, para no cortar la superficie con shotcrete (que
   queda mas cerca del sensor que la original).
3. **"Aplicar segmentacion"** — requiere alineacion calculada antes; recorta
   ambas nubes a los puntos dentro del box, guarda `<nombre>_segmento.ply`
   para cada una, y actualiza "Comparacion" para usar esos archivos
   recortados en el resto del analisis. **"Quitar segmentacion"** vuelve a
   usar las nubes completas.

> **En Windows (`gui.py`):** esta pestaña **no existe**. El recorte
> disponible ahi es el "crop" simple de la pestaña "Procesamiento"
> (coordenadas min/max en linea recta, sin box orientado a un plano).

---

## Pestaña "Ajustes de analisis"

- **"Analizar solo la zona seleccionada"** (checkbox) + **"Seleccionar zona
  en el visor 3D..."** — Shift+Click sobre 2 o mas puntos que delimiten la
  zona de interes, para que la estadistica no se diluya con el resto de una
  escena que no cambio.
- **"Opciones avanzadas"** (colapsado, click para desplegar):
  - **Tamaño de voxel (m)** — 0 = sin reducir; un valor como 0.01 (1cm)
    acelera el analisis en nubes muy densas, promediando puntos cercanos.
  - **"Quitar ruido / puntos aislados"** — descarta puntos anormalmente
    lejos de sus vecinos.
  - **"Corregir alineacion entre capturas (ICP)"** + umbral (m) — ver
    diferencia con la alineacion por puntos de referencia en la seccion de
    esa pestaña, mas arriba.
  - Coordenadas manuales de la zona (alternativa a elegirla en 3D).
  - **"Margen extra al seleccionar en 3D (m)"** — tiene que ser mayor al
    desplazamiento esperado entre capturas, o la zona puede quedar vacia en
    la nube con shotcrete.
  - Carpeta donde se guardan los resultados (CSV, histograma, heatmap,
    informe).

> **En Windows (`gui.py`):** pestaña "Procesamiento" — mismas opciones de
> voxel/outliers/ICP, mas un crop simple por coordenadas min/max (no hay
> seleccion de "solo una zona" por Shift+Click con el mismo flujo).

---

## Pestaña "Visualizacion 3D"

### Color del espesor

Tres modos, elegibles con radiobuttons:

- **Escala continua** — degrade azul (poco espesor) → rojo (mucho espesor),
  proporcional al valor medido.
- **3 niveles de color** (modo por defecto) — clasifica cada punto en
  verde/amarillo/rojo segun dos umbrales en mm que vos definis: **"Hasta
  este espesor = verde"** y **"Desde este espesor = rojo"** (50mm/100mm por
  defecto). Estos mismos umbrales son los que disparan la alerta de
  falta/exceso y aparecen en el informe.
- **6 niveles (espesor objetivo)** — divide el **"Espesor objetivo (cm)"**
  que definas en 6 tramos iguales con una escala termica: Muy Frio (morado)
  → Frio (azul) → Fresco (cian) → Templado (verde) → Calido (naranja) →
  Caliente (rojo = objetivo alcanzado o superado). Pensado para ver de un
  vistazo el progreso de aplicacion durante la operacion (por ejemplo, en la
  vista en vivo).

"Opciones avanzadas" (dentro de esta seccion): tope de la barra de color (m)
para la escala continua — vacio = se ajusta solo segun el maximo encontrado.

### Vista 3D en pantalla

- **"Mostrar resultado sobre el tunel original"** — muestra/oculta la nube
  con shotcrete coloreada, superpuesta a la base.
- **"Origen de la vista"** — "Ultima captura / archivo" (resultado fijo) o
  "En vivo" (redibuja continuo, necesita sensor conectado).
- **"Abrir vista 3D"** / **"Cerrar vista 3D"** — ventana Open3D aparte.

### Ajustes de captura en vivo (MVP)

Mismos parametros que en la pestaña Captura (distancia maxima, cono/FOV, eje
frontal), mas **inversion de ejes Y/Z** para adaptar la orientacion al
montaje fisico del sensor si quedo dado vuelta. Comparten estado con la
pestaña Captura — cambiar uno cambia el otro.

> **En Windows (`gui.py`):** pestaña "Visualizacion". Solo hay 2 modos de
> color (continuo y 3 niveles, sin el modo de 6 niveles), y no hay ajustes
> de captura en vivo (no tiene el MVP de captura en tiempo real).

---

## Pestaña "Comparacion (prueba)" — experimental

Visor 3D **embebido directamente en la ventana** (a diferencia de
"Visualizacion 3D", que abre una ventana de Open3D aparte). Muestra ambas
nubes superpuestas con un resaltado sutil (gris → ambar tenue, fondo blanco)
donde difieren, en vez de un heatmap tipo arcoiris. Se controla con el mouse
(arrastrar = orbitar, rueda = zoom). Todavia no esta decidido si esto queda
en la version final de la app.

> **En Windows (`gui.py`):** no existe esta pestaña.

---

## Botones superiores (siempre visibles)

- **"▶ Calcular espesor"** — corre el analisis completo con el estado actual
  de todas las pestañas (archivos, alineacion/segmentacion si se aplicaron,
  ajustes, color elegido) y muestra el resultado: tarjetas con puntos
  analizados, espesor medio/mediano/minimo/maximo y percentil 95, y cambia
  sola a "Visualizacion 3D" con la vista coloreada.
- **"Generar informe"** — se habilita despues de un analisis exitoso.
  Exporta un **PDF** (`informe_aurora_<fecha>_<hora>.pdf`, en la carpeta de
  resultados) con: fecha y hora, archivos analizados, estado (DENTRO DE
  PARAMETRO / FALTA SHOTCRETE / EXCESO DE SHOTCRETE segun los umbrales de "3
  niveles de color"), tabla de estadisticas en cm, y referencias al
  histograma/CSV/heatmap generados junto con el analisis.
- **"Detalles tecnicos"** (desplegable, abajo del todo) — log con lo que fue
  haciendo la app paso a paso; util para diagnosticar un error.

### Alertas

Si el espesor medio queda por debajo del umbral "verde" o por encima del
umbral "rojo" (los de "3 niveles de color" en Visualizacion 3D), aparece una
alerta de **falta** o **exceso** de shotcrete despues de calcular.

---

## Alternativa sin GUI: linea de comandos

Para automatizar o correr en un servidor sin interfaz grafica:

```bash
python scripts/compare_point_clouds.py --base data/base.ply --updated data/updated.ply --visualize
```

Ver `README.md` seccion 5 para la lista completa de flags (`--voxel-size`,
`--remove-outliers`, `--icp`, `--crop-min/--crop-max`, `--max-distance`,
`--output-dir`, `--overlay`). La alineacion por foto de referencia y la
segmentacion por box orientado son funciones exclusivas de `gui_gtk.py` por
ahora, no tienen flag equivalente en el CLI.
