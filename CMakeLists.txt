# savia_c — firmware C de la estación Savia para Raspberry Pi Pico (RP2040 / RP2350)
#
# UN SOLO código, DOS placas. Se selecciona en tiempo de compilación:
#
#   Pico WH  (RP2040, 264 KB, sin FPU)  -> inferencia OFF-device (la hace el móvil)
#       cmake -DPICO_BOARD=pico_w  -DSAVIA_ON_DEVICE_INFERENCE=OFF ..
#
#   Pico 2 W (RP2350, 520 KB, con FPU)  -> inferencia ON-device (LSTM int8 / TFLM)
#       cmake -DPICO_BOARD=pico2_w -DSAVIA_ON_DEVICE_INFERENCE=ON  ..
#
# Matriz completa y el porqué en docs/BUILD.md.

cmake_minimum_required(VERSION 3.13)

# Placa destino por defecto (se puede sobreescribir con -DPICO_BOARD=...).
if(NOT DEFINED PICO_BOARD)
  set(PICO_BOARD pico2_w CACHE STRING "Target board: pico_w (RP2040) | pico2_w (RP2350)")
endif()

include(pico_sdk_import.cmake)

project(savia_c C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# --- Opción: inferencia LSTM on-device --------------------------------------
# Solo cabe en RP2350 (Pico 2 W). En RP2040 (Pico WH) se deja OFF y la inferencia
# la hace la app móvil con el pronóstico que servimos por BLE.
option(SAVIA_ON_DEVICE_INFERENCE "Run the LSTM on-device (needs RP2350 RAM/FPU)" OFF)

# El stack BLE (BTstack + CYW43) se enlaza por defecto: la app lo necesita para
# conectar. Se puede desactivar con -DSAVIA_ENABLE_BLE=OFF para un build mínimo
# sin radio (bring-up / pruebas de lógica en placa).
option(SAVIA_ENABLE_BLE "Link the BLE stack (BTstack + CYW43)" ON)

pico_sdk_init()

# --- Ejecutable -------------------------------------------------------------
add_executable(savia_c
  src/main.c
  # system: app lifecycle, config + persistence, identity, clock, logging, ML hook
  src/system/config.c
  src/system/config_store.c
  src/system/pinmap.c
  src/system/device.c
  src/system/status_led.c    # onboard status LED (paired / BLE on / operating)
  src/system/clock.c
  src/system/clock_store.c   # persist the sync ring (outage recovery) in flash
  src/system/actuator.c      # digital-actuator slot state (BLE op "act")
  src/system/log.c
  src/system/inference.c
  src/system/weather.c       # TA forecast cache (LoRa downlink -> LSTM input)
  # power: deep sleep + schedule
  src/power/power.c
  src/power/scheduler.c
  # drivers: hardware peripherals
  src/drivers/sensor_sdi12.c
  src/drivers/lora_wioe5.c
  # storage: flash-backed ring
  src/storage/storage_flash.c
  # codec: reusable encoders (also linked by the host tests)
  src/codec/cbor.c
  src/codec/sha256.c
  src/codec/lora_codec.c     # LoRaWAN node payloads, wire v2 (typed up/downlinks + config TLV)
  src/codec/sdi12_parse.c    # pure SDI-12 reply parsing (host-tested)
  # ble: peripheral, wire codec, auth
  src/ble/auth.c
  src/ble/ble_codec.c
  src/ble/ble_gatt.c
)

target_include_directories(savia_c PRIVATE include)

# Nivel de log: -DSAVIA_LOG_LEVEL=0 (DEBUG, vuelca bytes BLE) | 1 (INFO, def) | 2 (WARN).
if(NOT DEFINED SAVIA_LOG_LEVEL)
  set(SAVIA_LOG_LEVEL 1)
endif()
target_compile_definitions(savia_c PRIVATE SAVIA_LOG_LEVEL=${SAVIA_LOG_LEVEL})

# Credenciales OTAA de TTN: NUNCA en el fuente. Llegan del .env local via Makefile
# (-DSAVIA_LORA_APP_KEY=...). Si no se pasan, lora_wioe5.c usa placeholders a cero.
foreach(_cred SAVIA_LORA_DEV_EUI SAVIA_LORA_APP_EUI SAVIA_LORA_APP_KEY)
  if(${_cred})
    target_compile_definitions(savia_c PRIVATE ${_cred}="${${_cred}}")
  endif()
endforeach()

# Nombre del binario según placa + ubicación de la inferencia, para no mezclarlos.
if(SAVIA_ON_DEVICE_INFERENCE)
  target_compile_definitions(savia_c PRIVATE SAVIA_ON_DEVICE_INFERENCE=1)
  # LSTM input prep (scaler + tensor build) only ships on the board that runs the
  # model; off-device (RP2040) the app builds the tensors, so we skip these to
  # save flash/RAM. Both are pure C and host-tested via test/run_host_tests.sh.
  target_sources(savia_c PRIVATE
    src/system/scaler.c          # StandardScaler constants (LSTM in/out)
    src/system/lstm_input.c      # gather + build the LSTM tensors
    src/system/inference_tflm.cpp # TFLM MicroInterpreter (C++), needs lib/pico-tflmicro
  )
  set(SAVIA_VARIANT "${PICO_BOARD}-mlondevice")
  message(STATUS "savia_c: inferencia ON-device (TFLM) — requiere lib/pico-tflmicro")
else()
  target_compile_definitions(savia_c PRIVATE SAVIA_ON_DEVICE_INFERENCE=0)
  set(SAVIA_VARIANT "${PICO_BOARD}-mloffdevice")
  message(STATUS "savia_c: inferencia OFF-device (la hace la app)")
endif()
set_target_properties(savia_c PROPERTIES OUTPUT_NAME "savia_c-${SAVIA_VARIANT}")

# --- Librerías del Pico SDK -------------------------------------------------
target_link_libraries(savia_c
  pico_stdlib
  hardware_flash            # anillo de almacenamiento + config en flash
  pico_flash                # flash_safe_execute (escritura segura con cyw43/BTstack)
  pico_rand                 # get_rand_32 -> nonce del handshake auth
  hardware_uart             # LoRa Wio-E5 (comandos AT)
  hardware_pio              # timing de SDI-12
  hardware_adc              # sensores analógicos (analog_linear, GP26-28)
)

# BLE peripheral vía BTstack sobre el CYW43439 (mismo chip en Pico WH y Pico 2 W).
# Se enlaza solo con SAVIA_ENABLE_BLE=ON (cuando el módulo BLE esté implementado).
if(SAVIA_ENABLE_BLE)
  # CYW43_LWIP=0: BLE-only, sin WiFi/lwIP (evita el include de lwip/netif.h).
  target_compile_definitions(savia_c PRIVATE SAVIA_ENABLE_BLE=1 CYW43_LWIP=0)
  # Genera el ATT DB (savia_ble.h) desde el .gatt (mismo contrato que savia_py).
  pico_btstack_make_gatt_header(savia_c PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src/ble/savia_ble.gatt)
  target_link_libraries(savia_c
    pico_cyw43_arch_threadsafe_background   # BLE corre en background; el supervisor sigue
    pico_btstack_ble                        # GATT server
    pico_btstack_cyw43
  )
endif()

# TFLite Micro solo se enlaza con inferencia on-device (Pico 2 W).
# Añadir pico-tflmicro como submódulo en lib/ (ver docs/BUILD.md):
if(SAVIA_ON_DEVICE_INFERENCE)
  if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/pico-tflmicro/CMakeLists.txt)
    add_subdirectory(lib/pico-tflmicro)
    target_link_libraries(savia_c pico-tflmicro)
    # pico-tflmicro drags in pico_multicore, which flips flash_safe_execute into
    # "core-1 lockout required" mode -- and we never launch core 1, so every flash
    # save fails with NOT_PERMITTED (-4). We are single-core: declare core 1 safe.
    target_compile_definitions(savia_c PRIVATE PICO_FLASH_ASSUME_CORE1_SAFE=1)
    # GCC 15 (arm-gnu-toolchain 15.x) dropped transitive <cstdint> includes, which
    # the vendored flatbuffers headers rely on ('uint32_t does not name a type').
    # Force-include stdint.h -- but ONLY for C/C++ TUs, never the .S assembly files
    # the lib pulls from the SDK (the assembler chokes on C typedefs).
    target_compile_options(pico-tflmicro PRIVATE
      "$<$<COMPILE_LANGUAGE:C,CXX>:SHELL:-include stdint.h>")
  else()
    message(WARNING "SAVIA_ON_DEVICE_INFERENCE=ON pero falta lib/pico-tflmicro (ver docs/BUILD.md)")
  endif()
endif()

# stdio por USB-serie para logs durante el bring-up.
pico_enable_stdio_usb(savia_c 1)
pico_enable_stdio_uart(savia_c 0)

# Genera .uf2 / .elf / .bin / .map
pico_add_extra_outputs(savia_c)
