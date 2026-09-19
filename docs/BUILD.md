# Compilación — savia_c

Notas de build de la estación Savia en C. **Un mismo código, dos placas**: lo que
cambia es la combinación `PICO_BOARD` + `SAVIA_ON_DEVICE_INFERENCE`.

## Requisitos

- **Pico SDK** (≥ 2.0, soporta RP2040 y RP2350). Exportar `PICO_SDK_PATH`.
- `arm-none-eabi-gcc`, `cmake` (≥ 3.13), `ninja` o `make`.
- Solo para inferencia on-device: **pico-tflmicro** como submódulo en `lib/pico-tflmicro`.

## Matriz de compilación

| # | Placa física | `PICO_BOARD` | `SAVIA_ON_DEVICE_INFERENCE` | Inferencia | Binario |
|---|---|---|---|---|---|
| 1 | **Pico WH** (RP2040) | `pico_w` | `OFF` | la hace el móvil | `savia_c-pico_w-mloffdevice.uf2` |
| 2 | **Pico 2 W** (RP2350) | `pico2_w` | `ON` | LSTM int8 on-device | `savia_c-pico2_w-mlondevice.uf2` |

> **Por qué la diferencia:** el RP2040 (Pico WH) tiene **264 KB de RAM y sin FPU**; el
> LSTM (arena ~200 KB en int8) **no cabe junto a la pila BLE** (~90 KB). El RP2350
> (Pico 2 W) tiene **520 KB + FPU** y sí lo aloja con holgura. Medición que lo
> sustenta: ver `project_pending_hw_tests` en las notas del proyecto.

## Comandos

> ⚠️ **`SAVIA_ENABLE_BLE` está OFF por defecto.** Para un firmware que la app pueda
> ver hay que añadir **`-DSAVIA_ENABLE_BLE=ON`** (enlaza BTstack + CYW43 y genera el
> ATT DB). Sin él el build es mínimo y arranca sin BLE.

### Pico WH (RP2040) — inferencia off-device

```sh
cmake -S . -B build-pico_w -G Ninja \
  -DPICO_BOARD=pico_w \
  -DSAVIA_ENABLE_BLE=ON \
  -DSAVIA_ON_DEVICE_INFERENCE=OFF
ninja -C build-pico_w
# -> build-pico_w/savia_c-pico_w-mloffdevice.uf2
```

### Pico 2 W (RP2350) — inferencia on-device

```sh
cmake -S . -B build-pico2_w -G Ninja \
  -DPICO_BOARD=pico2_w \
  -DSAVIA_ENABLE_BLE=ON \
  -DSAVIA_ON_DEVICE_INFERENCE=ON
ninja -C build-pico2_w
# -> build-pico2_w/savia_c-pico2_w-mlondevice.uf2
```

## Build MOCK: humedad simulada con datos reales (desarrollo)

`make build MOCK=ON` (o `make flash MOCK=ON`, en cualquier placa) genera una imagen de
desarrollo que **arranca con el mock encendido**: la sonda es una **réplica de humedad
medida** hasta que se apague desde TerraLink. HS10 y HS30 del nodo 4 de
`dataset_master_hourly.csv`, del 1 al 29 de septiembre de 2020 (año que el LSTM no vio al
entrenar). El binario lleva el sufijo `-mock` (`savia_c-pico2_w-mlondevice-mock.uf2`).
`make build --mock` no es posible: `make` trata cualquier `--xxx` como opción propia y
aborta con `unrecognized option`.

- **Sólo humedad**, en el puerto 1. La temperatura del aire no se simula: llega como en
  campo, por el downlink LoRa (Open-Meteo del backend) o por la característica BLE
  `weather`. Sin ella la inferencia se niega con «no air-temperature forecast cached».
- **48 h hacia atrás + 24 h hacia delante.** En cuanto hay hora (LoRa o `time_sync` BLE)
  la estación completa las 48 h que necesita el LSTM, terminando en la hora en curso, y
  las 24 h siguientes, que son el «real» con el que se puntúa el pronóstico («Predicción
  vs real»). Cada ciclo añade la hora nueva; la inferencia bajo demanda rellena antes de
  correr.
- **Determinista.** El valor de una hora depende sólo de su hora local
  (`utc_offset_min`): el ritmo del dataset (riegos hacia las 10–11 h) cae en las mismas
  horas locales, y un reinicio reconstruye exactamente los mismos datos. Los 29 días se
  repiten en bucle. Una hora que ya tiene lectura (p. ej. un `ingest` de la app) se respeta.
- **El mock se apaga desde la app, sin reflashear.** `MOCK=ON` sólo cambia el valor de
  fábrica: una placa sin config guardada arranca replicando, y el interruptor de
  TerraLink manda a partir de ahí (la decisión se guarda y sobrevive a los reinicios).
  Si al flashear sobrevive una config que decía mock apagado, enciéndelo desde la app.
- **No mezcla réplica y medidas.** Activar o desactivar el mock vacía el anillo, en
  cualquier build: la réplica y lo que mida la sonda nunca comparten ventana.
- **Auto-test al arrancar** (sólo Pico 2 W): corre el LSTM sobre una ventana real embebida
  (con su propia TA) y deja en los logs la desviación frente a la predicción del host con
  el mismo modelo y el MAE frente al HS30 medido (`selftest: ...`).
- **Coste:** ~7 KB de flash (la réplica la usan también los builds normales, para el mock
  de TerraLink); RAM sin cambios.

> ⚠️ No es para producción: con LoRa activo en modo FORWARD, la réplica sube al backend
> como si la hubiera medido la sonda.

Los datos están en `include/savia/mock_soil_data.h`, generado por
`tools/gen_mock_soil.py` (otro tramo, o si cambia el modelo embebido):

```sh
../../.venv-tflite/bin/python tools/gen_mock_soil.py --start "2020-09-01 00:00" --days 29
```

La predicción de referencia del host usa los bytes del modelo embebido, el mismo escalado
en float32 y la misma cuantización que el firmware, con los kernels de TFLite **sin**
XNNPACK: con XNNPACK la salida int8 de este modelo se mueve hasta 0,005.

## Flashear

Mantener pulsado **BOOTSEL**, conectar el USB, y arrastrar el `.uf2` a la unidad
`RPI-RP2` (RP2040) o `RP2350` que aparece. Los logs salen por USB-serie
(`pico_enable_stdio_usb`); abrir con `minicom`/`screen` a 115200.

## Notas

- **`pico_w` vs `pico2_w`** seleccionan chip y librerías correctas del SDK; el
  binario NO es intercambiable entre placas (distinta arquitectura: Cortex-M0+
  vs M33). El **fuente sí** es el mismo.
- El BLE usa el **mismo CYW43439** en ambas placas → el código BLE es idéntico.
- `SAVIA_ON_DEVICE_INFERENCE=ON` en una Pico WH **compilaría pero no cabría** en
  RAM; por eso la combinación recomendada para el RP2040 es `OFF`.
- TFLM (`lib/pico-tflmicro`) solo se enlaza con `SAVIA_ON_DEVICE_INFERENCE=ON`.

## Inferencia on-device: qué queda por cablear (RP2350)

El pipeline de datos ya está **completo y probado en host** (`scaler.c`, `lstm_input.c`,
`inference_run_daily()` en `inference.c`, test `test_inference.c`). Lo que falta son
los tres pasos que dependen de la placa/artefactos y no se pueden compilar sin ellos:

1. **Añadir la lib** `lib/pico-tflmicro` (submódulo). El `CMakeLists.txt` ya la enlaza
   condicionalmente cuando `SAVIA_ON_DEVICE_INFERENCE=ON`.
2. **Embeber el modelo**: `sh tools/embed_model.sh` genera
   `include/savia/lstm_hs30_int8_model.h` con `g_lstm_hs30_model[]` desde
   `lstm_hs30_int8.tflite`.
3. **Rellenar `inference_run()`** en `src/system/inference.c` con el `MicroInterpreter`
   (la receta exacta —set de tensores, cuantización int8, `Invoke()`, dequant— está en
   los comentarios de esa función). Arena medida ≈200 KB.

El **disparador está comentado a propósito** en `src/main.c` (`// inference_run_daily(now_ms);`):
descoméntalo cuando los tres pasos anteriores estén hechos para correrlo en el ciclo diario.

## Tests de host (sin SDK, sin placa)

La lógica pura (config, y a futuro codecs/parsers/agregación) se compila y testea
**nativamente en el PC**, sin el Pico SDK ni hardware:

```sh
sh test/run_host_tests.sh
```

Usa el `cc` del sistema con `-Iinclude`. Es el equivalente a la suite `pytest` de
`savia_py` para la parte de lógica que no depende del hardware.

## Preparar el cross-compile (una vez)

Compilar el `.uf2` del firmware necesita el toolchain ARM + el Pico SDK (no la placa):

```sh
sh tools/setup_pico_sdk.sh        # arm-none-eabi-gcc + ninja + clona pico-sdk
export PICO_SDK_PATH=$HOME/pico-sdk
```

Después, los comandos de compilación de arriba. **Compilar no requiere el Pico**;
la placa solo hace falta para flashear y ejecutar.

### Dónde busca el Makefile el SDK y el toolchain

`make build` pasa siempre a cmake `PICO_SDK_PATH` y `PICO_TOOLCHAIN_PATH`, así que
no dependen de lo que haya en el `PATH` ni de una caché anterior:

- `PICO_SDK_PATH`: por defecto `~/pico-sdk`.
- `PICO_TOOLCHAIN_PATH`: por defecto la versión más alta de
  `~/arm-gnu-toolchain/arm-gnu-toolchain-*-arm-none-eabi/bin` (toolchain
  oficial de ARM descomprimido ahí). Si no hay ninguna, cmake busca
  `arm-none-eabi-gcc` en el `PATH`.

Ambas se pueden sobreescribir por entorno o en la línea de comandos
(`make build PICO_TOOLCHAIN_PATH=/ruta/al/bin`).

> El `arm-none-eabi-gcc` de la **fórmula** de Homebrew no incluye newlib y el
> enlace falla con `nosys.specs`; usa el toolchain oficial de ARM (o el cask
> `gcc-arm-embedded`).

Si se mueve o borra el toolchain con el que se configuró un `build-*/`, cmake no
lo vuelve a buscar (queda grabado en `CMakeCache.txt`); el Makefile lo detecta y
regenera el directorio de build automáticamente. A mano: `make clean`.
