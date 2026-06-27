# AquaCheck SDI-12 — respuestas reales de la sonda

Capturado el 2026-06-27 con `tools/sdi12_bringup` (probe de bring-up) sobre **GP18**,
alimentando la sonda a 3.3 V, decodificación por **sobremuestreo** (~8×, sin deriva).
La sonda estaba en ambiente (aire/mesa), no enterrada.

## Identificación de esta sonda

| Dato | Valor |
|---|---|
| Dirección SDI-12 | `0` (de fábrica) |
| Protocolo | SDI-12 v1.3 |
| Fabricante / modelo | `AquaChck` / `ACHSDI` (versión calibrada 0–100 %, ±1 %) |
| Firmware | `043` (≈ v4.3) |
| Nº de serie | `S257588` |
| **Nº de sensores** | **4** ⚠️ (no 2) |

## Respuestas por comando

| Comando | Respuesta (ASCII) | Significado |
|---|---|---|
| `?!`   | `0` | dirección de la sonda |
| `0!`   | `0` | ack activo (presente) |
| `0I!`  | `013AquaChckACHSDI043S257588` | identificación completa |
| `0R0!` | `0+043.0+04+004` | versión 43.0, **04 sensores**, longitud (código 04) |
| `0X#!` | `0#04L04` | **4 sensores, código de longitud L04 = 40 cm** |
| `0M!`  | `00024` | inicia medición humedad: `ttt=002` s, `n=4` valores |
| `0D0!` | `0+016.9562+025.1937+002.3312` | humedad sensores 1-3 (SFU) |
| `0D1!` | `0+002.8218` | humedad sensor 4 (SFU) |
| `0M1!` | `00004` | inicia medición temperatura: `ttt=000` s, `n=4` |
| `0D0!` | `0+25.937+26.125+26.062` | temperatura sensores 1-3 (°C) |
| `0D1!` | `0+26.562` | temperatura sensor 4 (°C) |
| `0C!`  | `000004` | medición concurrente: `nn=04` valores |

### Valores de esta lectura (en aire)
- **Humedad (SFU, rango −5..+120):** 16.96, 25.19, 2.33, 2.82
- **Temperatura (°C):** 25.94, 26.12, 26.06, 26.56

## Notas para implementar el driver SDI-12 real (PIO) en el firmware

1. **Trama**: 1200 baud, 7 bits datos, paridad par, 1 stop, **invertido** (idle/marking = LOW;
   start/spacing = HIGH; logic-1 = LOW). Half-duplex en un solo hilo (GP18).
2. **Secuencia de medición**: `aM!` → respuesta `atttn` → esperar `ttt` s → `aD0!`, `aD1!`…
   hasta recoger `n` valores. Temperatura igual con `aM1!`.
3. **Esta sonda devuelve 4 valores** repartidos: `D0!` trae 3, `D1!` trae 1. No asumir un
   número fijo: leer `n` de la respuesta a `M!` y pedir `D0..Dk` hasta juntar `n`.
4. **Unidades**: el valor de humedad es **SFU (scaled frequency unit)**, NO VWC. La conversión
   SFU→VWC necesita calibración de suelo (documento "AquaCheck soil calibrations").
5. **Mapeo de profundidades (RESUELTO)**: `aX#!`=`0#04L04` → **4 sensores, longitud 40 cm**
   = SKU **1120-0404**. Según la tabla del datasheet (fila 2, marcas en 100/200/300/400 mm),
   las profundidades son **10 / 20 / 30 / 40 cm**. ✅ Incluye 10 cm y 30 cm (los que pide el LSTM).
6. **Orden de los valores CONFIRMADO (test de inmersión 2026-06-27)**: metiendo la **punta**
   (40 cm) en agua, el **4º valor** (el de `0D1!`) saltó de ~2.8 a ~72 SFU; los otros 3 no
   cambiaron. Luego el orden es **top→bottom (superficial primero)**:
   `valor[0]`=10 cm, `valor[1]`=20 cm, `valor[2]`=30 cm, `valor[3]`=40 cm.
   → en el driver: **HS10 = valor[0]** (1º de `0D0!`), **HS30 = valor[2]** (3º de `0D0!`).
   Mismo orden para las 4 temperaturas de `0M1!`.
6. **No enviar `aAb!`** (cambia la dirección) ni comandos de escritura: la sonda no tiene
   valores ajustables salvo la dirección.

> Capturado con el firmware de diagnóstico `tools/sdi12_bringup` (no es el firmware de la
> estación). La lectura SDI-12 real en `src/drivers/sensor_sdi12.c` sigue siendo `TODO(hw)`.
