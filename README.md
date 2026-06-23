# savia_c

Firmware en **C** de la estación **Savia** para **Raspberry Pi Pico** (RP2040 / RP2350),
usando el **Pico SDK**. Es la implementación para microcontrolador del firmware que
en Linux vive en `savia_py` (Raspberry Pi Zero 2 W).

> Estado: **funcional**. Build, **BLE** (GATT + auth + config + ingest + logs +
> pinmap), almacenamiento en flash, scheduler y deep-sleep implementados y con
> tests de host. Quedan como stubs `TODO(hw)` la lectura **SDI-12** (PIO), **LoRa**
> (Wio-E5) y la inferencia on-device (TFLM) en Pico 2 W.

## Un solo código, dos placas

La estación corre sobre dos variantes de Pico, seleccionadas **en tiempo de
compilación**. El código fuente es el mismo; cambian un par de flags:

| Placa | Chip | RAM / FPU | Inferencia LSTM | Build |
|---|---|---|---|---|
| **Pico WH** | RP2040 | 264 KB / no | **off-device** (la hace la app) | `-DPICO_BOARD=pico_w -DSAVIA_ON_DEVICE_INFERENCE=OFF` |
| **Pico 2 W** | RP2350 | 520 KB / sí | **on-device** (LSTM int8 / TFLM) | `-DPICO_BOARD=pico2_w -DSAVIA_ON_DEVICE_INFERENCE=ON` |

El modelo LSTM solo cabe en la RAM del RP2350; en el RP2040 se sirve el dato y la
inferencia la hace el móvil. La **ubicación de la inferencia es la única pieza
modular** entre ambas placas. Detalle y comandos exactos en [`docs/BUILD.md`](docs/BUILD.md).

## ▶️ Compilar y flashear

Lo fácil — desde `development/savia_c/` hay un **`Makefile`** que envuelve cmake + ninja:

```sh
make build        # compila el .uf2 (Pico W, BLE ON)  -> build-pico_w/
make flash        # build + copia el .uf2 a la Pico en BOOTSEL (RPI-RP2)
make test         # tests de host (lógica pura, sin SDK ni placa)
make clean
```

Para la otra placa o variantes: `make build BOARD=pico2_w INFER=ON` (Pico 2 W con
LSTM on-device), o `make build BLE=OFF` (build mínimo sin radio).

> **BLE va ON por defecto** (la app lo necesita para conectar). Ya no hace falta
> pasar ningún flag; solo `make build`. Se puede desactivar con `BLE=OFF`.

**Una vez** — toolchain ARM + Pico SDK (no hace falta la placa para compilar; los
tests de host tampoco lo necesitan):

```sh
sh tools/setup_pico_sdk.sh
export PICO_SDK_PATH=$HOME/pico-sdk
```

**Flashear** — mantén pulsado **BOOTSEL** mientras conectas el USB (aparece la unidad
`RPI-RP2`); `make flash` copia el `.uf2` y la Pico se reinicia sola. Logs por
USB-serie a 115200 (`screen`/`minicom`).

Por debajo, `make build` equivale a:

```sh
cmake -S . -B build-pico_w -G Ninja -DPICO_BOARD=pico_w -DSAVIA_ENABLE_BLE=ON
ninja -C build-pico_w
```

Matriz completa de placas y flags en [`docs/BUILD.md`](docs/BUILD.md).

## Qué hace (las cuatro responsabilidades)

1. **Adquirir** — leer la sonda SDI-12 (pin configurable, multi-sensor).
2. **Agregar** — media horaria, guardada en un anillo en flash (≥48 h).
3. **Inferir** — pronóstico de humedad 24 h (on-device en Pico 2 W; en la app en Pico WH).
4. **Servir** — periférico BLE GATT (mismo contrato que `savia_py` → TerraLink + app de Tobías).

Más: canal **LoRa** (Wio-E5), y **deep sleep con despertar por botón** + tiempo de
sueño parametrizable desde la app (requisitos de la era-Pico).

## Estructura

```
savia_c/
├── CMakeLists.txt          # build: PICO_BOARD + SAVIA_ON_DEVICE_INFERENCE
├── pico_sdk_import.cmake    # import estándar del Pico SDK
├── docs/BUILD.md            # matriz de compilación y el porqué
├── include/
│   ├── btstack_config.h     # config mínima de BTstack (BLE)
│   └── savia/               # interfaces de cada módulo
└── src/                     # implementación (lógica real + stubs hw)
```

## Contrato BLE

Idéntico al de `savia_py` (documentado en `docs/integracion_ble_savia_tobias.md`
del repo TFM). La app no distingue si el periférico es una Pi o un Pico.
