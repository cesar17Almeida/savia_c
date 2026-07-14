# BLE Weather Parsing & GATT Long Write Fixes

This document summarizes the changes made to the `savia_c` firmware to fully support receiving weather temperatures over Bluetooth Low Energy (BLE) from a mobile application (e.g., Flutter). These changes allow the IoT Station to successfully gather the historical and forecasted temperatures required by the LSTM model when LoRa is unavailable.

## 1. CBOR Float16 Support (`src/codec/cbor.c`)
- **Issue**: Modern CBOR encoders (like the one used in Dart/Flutter) automatically optimize floating-point numbers like `20.0` or `20.5` by encoding them as **Half-Precision Floats** (Float16, tag `0xf9`). The custom C parser previously only supported Float32 (`0xfa`) and Float64 (`0xfb`), returning an error on Float16 and triggering a "bad weather" log.
- **Change**: Added explicit parsing logic in `cbor_r_double` to detect `0xf9`, decode the 16-bit float (extracting the sign, exponent, and fraction), and cast it securely to a standard `double` for the firmware to use.

## 2. Weather Array Extraction (`include/savia/ble_codec.h` & `src/ble/ble_codec.c`)
- **Issue**: Previously, the `ble_parse_weather` function verified the outer structure of the weather payload but explicitly skipped the `data` key, relying instead on LoRa to fetch actual temperatures.
- **Change**: 
  - Modified the signature of `ble_parse_weather()` to accept output arrays: `float *past_ta`, `uint8_t *n_past`, `float *future_ta`, and `uint8_t *n_future`.
  - Upgraded the CBOR map parsing loop to actively step into the `data` object, identify the `past_ta_hourly` and `future_ta_hourly` arrays, and iterate through them to extract the float values using the newly updated `cbor_r_double`.

## 3. GATT Long Write Buffering (`src/ble/ble_gatt.c`)
- **Issue**: The `H_WEATHER` payload containing 72 floats (approx. 200+ bytes) exceeds standard BLE MTU sizes (e.g., iOS limits). This forces the mobile OS to chunk the payload using GATT "Prepare Write" requests. The firmware's `att_write_cb` was ignoring the `offset` parameter, attempting to parse each broken chunk as a complete CBOR map, which constantly failed.
- **Change**: 
  - Included `"savia/weather.h"`.
  - Implemented a static 512-byte reassembly buffer (`weather_buf`) and a length tracker specifically for the `H_WEATHER` characteristic inside `att_write_cb`.
  - The firmware now listens to BTstack's `ATT_TRANSACTION_MODE_ACTIVE` to securely buffer incoming chunks at the correct `offset`.
  - Upon receiving `ATT_TRANSACTION_MODE_EXECUTE`, it passes the fully assembled payload to `ble_parse_weather()` and subsequently commits the data to the device's cache via `weather_set()`.
  - Removed the `(void) tx_mode;` and `(void) offset;` ignores at the top of the callback.

## 4. Test Suite Adaptations (`test/test_ble_codec.c`)
- **Issue**: The test harness broke due to the updated `ble_parse_weather` signature.
- **Change**: 
  - Included `"savia/weather.h"`.
  - Initialized mock arrays and length counters (`pta`, `fta`, `np`, `nf`) to correctly call the updated `ble_parse_weather` testing suite assertions.
