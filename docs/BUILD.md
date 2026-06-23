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
