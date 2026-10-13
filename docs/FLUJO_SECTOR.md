# Medicion rapida por tunel y sector

La misma ruta sirve para nubes PLY de cajas, otras pruebas y capturas mineras. No hay modo especial de cajas ni archivos de muestra cargados automaticamente.

## Flujo corto

1. En **Medicion rapida**, pulsa **Capturar antes** o **Cargar antes...**. Al capturar, conecta el sensor desde esta misma pagina y mantenlo quieto durante los 15 segundos de lectura. La nube queda guardada en una sesion nueva.
2. Cuando el lugar vuelva a estar accesible, pulsa **Capturar despues** o **Cargar despues...**. Puedes repetirlo para cada etapa de shotcrete; la lista de etapas aparece al cargar la primera.
3. Si moviste el sensor, pulsa **Alinear etapa con referencias estables**. Marca al menos tres puntos fijos, no alineados en una sola recta, visibles en ambas nubes y fuera de la zona modificada. Usa el mismo orden para ANTES y DESPUES. Esa alineacion es necesaria para comparar posiciones distintas; la app no puede asumir que dos capturas corresponden al mismo sitio solo porque son del mismo tunel.
4. Pulsa **Calcular comparacion**. Antes de calcular, confirma que ambas nubes muestran la misma zona util y que no dominan la escena maquinaria u oclusiones. Para varias capas, elige si quieres comparar contra ANTES o contra la etapa inmediatamente anterior.
5. Conserva la carpeta que muestra la app. Incluye las capturas originales, el registro, los resultados y el informe. No guardes solo el CSV.

## En el tunel

Antes del shotcrete, acuerda un tramo concreto y coloca o identifica referencias fisicas que permanezcan fijas y visibles en todas las visitas. Deben quedar fuera de la superficie que recibira material. Registra manualmente tunel, progresiva/estacion, lado o zona, fecha, etapa y operador: la interfaz actual no guarda todos esos datos de terreno.

Toma el ANTES con el sensor quieto. Retiralo mientras trabaja la maquinaria. Cuando el sector este libre y seguro, vuelve a capturar DESPUES desde una posicion que muestre la misma zona y las referencias. Si las referencias quedaron cubiertas, cambiaron o no se ven, no fuerces esa comparacion: elige referencias que sigan visibles o captura otra pareja con referencias validas. En un tunel curvo, evita usar puntos que pertenezcan a la superficie recubierta como anclas.

La salida actual es distancia C2C exploratoria entre nubes; no equivale a espesor normal certificado, no cuantifica cobertura y no debe usarse para aceptar shotcrete ni decidir seguridad. El programa captura una ventana de 15 segundos acumulada sin compensacion de movimiento, asi que el sensor debe permanecer inmovil. Valida primero con supervision tecnica y contrastes independientes en terreno.

Para instalar y abrir en Ubuntu usa `bash setup.sh` y `bash run_gui.sh`. Para diagnosticos y limites del algoritmo consulta [la auditoria](AUDITORIA_AURORA.md).
