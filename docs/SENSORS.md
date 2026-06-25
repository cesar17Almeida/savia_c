# Sensores — catálogo, pines configurables y cadencia

Cómo Savia soporta "conectar cualquier sensor" de forma honesta, cómo la app
(TerraLink) asigna pines libres, y cómo se parametriza la frecuencia de lectura.

## 1. Un sensor no es "un pin"

Leer un sensor desconocido enchufándolo a un GPIO es físicamente imposible: un
sensor es **interfaz eléctrica + protocolo + decodificación**. Para sacar °C o VWC,
el firmware tiene que saber de antemano una de tres cosas. Por eso la app **no
programa un pin en crudo**: elige un **tipo soportado** del catálogo y lo asigna a
un **pin libre y capaz**. "Cualquier sensor" se cumple en tres moldes:

| Molde | Cómo funciona | Tipo(s) en el firmware |
|---|---|---|
| **Bus auto-descriptivo** | El sensor se anuncia (`?!`, `aI!`) y devuelve sus valores con comandos estándar. Plug-and-play real. | `sdi12_aquacheck`, `sdi12_generic` |
| **Sensor analógico simple** | El instalador introduce la recta del datasheet `valor = escala·raw + offset` + unidades. | `analog_linear` |
| **Digital barato auto-ID** | Termómetro 1-Wire que se identifica por ROM y devuelve °C directo. | `onewire_ds18b20` |

Lo que **no** se puede prometer: un digital propietario nuevo con mapa de registros
propio solo enchufándolo. Eso siempre requiere un driver nuevo en firmware → reflash.

## 2. El catálogo (`savia_sensor_type_t`)

| `type` (wire) | Interfaz | Pin (capacidad) | Mapeo que aporta el instalador |
|---|---|---|---|
| `sdi12_aquacheck` | SDI-12 | PIO (cualquier GPIO) | nada — layout fijo (10/30 cm…) |
| `sdi12_generic` | SDI-12 | PIO | `chan[]`: valor-índice → `{kind, depth_cm}` |
| `analog_linear` | ADC | **ADC: solo GP26-28** | `kind`, `depth_cm`, `scale`, `offset` |
| `onewire_ds18b20` | 1-Wire | PIO | opcional `kind`/`depth_cm` (def. `soil_temperature`) |

### DS18B20 (la "tercera familia" de interfaz)

El DS18B20 es el termómetro digital de ~1 € omnipresente en campo. Se eligió como
ejemplo de modularidad porque demuestra una interfaz distinta (1-Wire) con coste de
firmware mínimo:

- **Auto-identificación:** ROM de 64 bits (family code `0x28`); no necesita driver
  por modelo. Varias sondas pueden colgar del mismo bus (selección por ROM match).
- **Eléctrico:** un solo pin de datos (push-pull con pull-up de 4.7 kΩ), 3.3 V —
  **sin level-shifter**. En el RP2040/RP2350 el timing 1-Wire lo lleva un programa
  PIO (igual que SDI-12), por eso su capacidad requerida es `PIO`.
- **Lectura:** reset + `Convert T` (0x44) + leer scratchpad → °C directo. El
  instalador no mapea nada; como mucho ajusta `kind`/`depth_cm` si se entierra.
- **Estado:** decodificación real `TODO(hw)` en `src/drivers/sensor_sdi12.c`
  (`measure_ds18b20`); con `mock_enabled` el pipeline adquirir→agregar→servir ya
  es ejercitable.

## 3. Modelo de datos (`savia_sensor_slot_t`)

Un slot = pin + tipo + la decodificación que el firmware no puede inferir. Una
`union` mantiene el slot compacto (toda la config cabe en una página de flash de
256 B): AquaCheck/1-Wire no gastan la union; analógico usa `scale/offset`; SDI-12
genérico usa el array de canales (máx. `SAVIA_SDI12_MAX_CHANNELS` = 4).

```c
typedef struct {
    savia_sensor_type_t type;
    uint8_t gpio;        // pin de datos (analógico = pin ADC)
    char    address;     // dirección SDI-12 (ignorada en analógico/1-Wire)
    uint8_t kind;        // savia_reading_kind_t del valor analógico/1-Wire
    uint8_t depth_cm;    // profundidad de ese valor (0 = ninguna)
    union {
        struct { float scale, offset; } analog;                       // analog_linear
        struct { uint8_t count; savia_channel_t ch[4]; } sdi12;       // sdi12_generic
    } map;
} savia_sensor_slot_t;
```

## 4. Inventario de pines (`pinmap`, característico BLE `…0015`)

`pinmap.c` (SDK-free, host-testado) modela las **capacidades estáticas** por pin
(`DIGITAL/PIO/PWM/I2C/SPI/UART` muxables en todo GPIO de usuario; **ADC solo
GP26-28**), el **estado vivo** (`free`/`in_use`/`reserved`) derivado de la config, y
las **reservas de sistema** (CYW43 `GP23/24/25/29`, botón de wake, UART de LoRa
`GP4/5` cuando LoRa está activo). La app lee `…0015` para ofrecer **solo** pines
asignables y pintarlos (ver `terralink_app/.../ui/components/PicoPinout.kt`).

- `pinmap_caps_for_sensor(type)` → capacidad que el tipo necesita en su pin
  (SDI-12/1-Wire → `PIO`; analógico → `ADC`).
- `pinmap_check_assign(cfg, gpio, need, exclude_slot)` → valida un pin suelto.
- `pinmap_check_sensors(base, slots, n, &bad)` → **valida la tabla entera de forma
  atómica**: capacidades + reservas + colisiones intra-lote (dos slots en el mismo
  pin). Devuelve el primer fallo y su índice; no aplica nada.

## 5. Edición de sensores por BLE (config `…0013`)

La app manda un **patch** (CBOR, campos omitidos = sin cambio). `sensors` es un
**reemplazo completo** de la tabla, validado en el firmware antes de commitear
(atómico: o entra toda o ninguna). En error → `config_err` con
`"sensor <i>: <motivo>"` (motivo de `pinmap_assign_str`).

**Entrada de sensor en el patch / snapshot:**

```jsonc
{
  "gpio": 26, "type": "analog_linear",
  "addr": "",                       // SDI-12: '0'..'9'; vacío en analógico/1-Wire
  "kind": "soil_temperature",       // analógico / 1-Wire
  "depth_cm": 30,
  "scale": 0.1, "offset": -40.0,    // solo analog_linear
  "chan": [                          // solo sdi12_generic
    { "kind": "soil_moisture", "depth_cm": 10 },
    { "kind": "soil_moisture", "depth_cm": 30 }
  ]
}
```

El **snapshot** (lectura de `…0013`) emite, para AquaCheck, solo
`{port,gpio,type,addr}` (layout fijo) — así el contrato existente con la app de
Tobías no cambia; los demás tipos añaden sus campos.

## 6. Frecuencia de lectura

Es **global** (`capture_interval_s`, campo ya existente, suelo `SAVIA_CAPTURE_MIN_S`
= 60 s por el AquaCheck), no por-sensor: el LSTM consume medias horarias, todos los
sensores se muestrean juntos y una cadencia por-sensor complicaría la agregación y
el planificador de deep-sleep sin ganancia agronómica. La app la edita en el patch
`…0013` (`capture_s`). El `scheduler` ya la consume y la respeta sobre el
`sleep_seconds` (despertar obligatorio para capturar).

## 7. Tests (host, sin SDK/HW)

- `test/test_pinmap.c` — capacidades de los 4 tipos + `pinmap_check_sensors`
  (set válido, analógico fuera de ADC → `INCAPABLE`, colisión intra-lote →
  `OCCUPIED`, pin reservado → `RESERVED`).
- `test/test_sensors.c` — construye un patch `sensors[]` con los 4 moldes, lo
  parsea (`ble_parse_config_patch`), valida atómicamente y serializa el snapshot.
- Cross-check CBOR contra `savia_py` (`crosscheck_ble_codec.py`) sigue verde: el
  snapshot por defecto no cambió en el wire.

Ejecutar: `make test` (o `sh test/run_host_tests.sh`).
