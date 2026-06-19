# savia_c

Firmware en **C** de la estación **Savia** para **Raspberry Pi Pico** (RP2040 / RP2350),
usando el **Pico SDK**. Es la implementación para microcontrolador del firmware que
en Linux vive en `savia_py` (Raspberry Pi Zero 2 W).

> Estado: **scaffold inicial**. Estructura, sistema de build y contratos listos;
> las piezas que dependen de hardware (BLE, SDI-12, LoRa, TFLM) están como
> interfaces + stubs marcados con `TODO(hw)`. Se rellenan con la placa delante.

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
