# savia_c — wrapper fino sobre cmake + ninja para no recordar los flags.
#
#   make build           # compila el .uf2 (Pico 2 W, BLE ON, LSTM on-device) -> build-pico2_w/
#   make flash           # build + flash a la Pico en BOOTSEL. AVISA: reinicia el
#                        # micro (pierde las lecturas) y resetea la config guardada.
#                        # Salta la confirmación con: make flash YES=1
#   make test            # tests de host (lógica pura, sin SDK ni placa)
#   make clean           # borra el build dir
#
# Variables (sobreescribibles):
#   make build BOARD=pico_w INFER=OFF    # Pico W (RP2040): sin LSTM on-device
#   make build BLE=OFF                   # build mínimo sin radio
#   make build MOCK=ON                   # dev: soil moisture replayed from the real dataset
#
# Compilar necesita PICO_SDK_PATH + arm-none-eabi-gcc (ver tools/setup_pico_sdk.sh);
# `make test` no necesita nada de eso.

# pico2_w (RP2350) por defecto | pico_w (RP2040) requiere INFER=OFF
BOARD ?= pico2_w
# BTstack + CYW43 (la app lo necesita)
BLE ?= ON
# LSTM on-device por defecto (disponible en la app); ON solo cabe en pico2_w (RP2350).
# Para la Pico W (RP2040): make build BOARD=pico_w INFER=OFF
INFER ?= ON
# Dev image: the probe is replaced by the dataset replay, 48 h back + 24 h ahead
# (never for production). make can't take "--mock": its options are its own.
MOCK ?= OFF
override MOCK := $(if $(filter ON on 1 YES yes TRUE true,$(MOCK)),ON,OFF)

# Credenciales OTAA locales (ver .env.example). Fichero ignorado por git: si no
# existe, las variables quedan vacias y el firmware usa placeholders a cero.
-include .env

# Pico SDK y toolchain ARM. Sobreescribibles por entorno o en la línea de
# comandos; si no, se buscan en sus ubicaciones habituales. El arm-none-eabi-gcc
# de la fórmula de Homebrew NO trae newlib (el enlace falla con 'nosys.specs'):
# se prefiere el toolchain oficial de ARM descomprimido en ~/arm-gnu-toolchain/
# (se toma la versión más alta presente).
PICO_SDK_PATH ?= $(HOME)/pico-sdk
PICO_TOOLCHAIN_PATH ?= $(lastword $(sort $(wildcard \
  $(HOME)/arm-gnu-toolchain/arm-gnu-toolchain-*-arm-none-eabi/bin)))

BUILD := build-$(BOARD)
INFER_TAG := $(if $(filter ON,$(INFER)),on,off)
MOCK_TAG := $(if $(filter ON,$(MOCK)),-mock)
UF2 := $(BUILD)/savia_c-$(BOARD)-ml$(INFER_TAG)device$(MOCK_TAG).uf2

# What the flash warning says about mock data (one quoted shell word per line).
ifeq ($(MOCK),ON)
FLASH_MOCK_NOTE := "* build MOCK: arranca con la humedad replicada del dataset real" \
  "  (48 h atrás + 24 h adelante) en vez de la sonda; se apaga desde TerraLink" \
  "* no es para producción: con LoRa en FORWARD la réplica sube al backend"
else
FLASH_MOCK_NOTE := "* mock: conserva el valor guardado (de fábrica OFF; se cambia desde TerraLink)"
endif

.DEFAULT_GOAL := build
.PHONY: build flash test clean help

# Reconfigura siempre (cmake es idempotente y barato) para que un cambio de
# BOARD/BLE/INFER se aplique, y luego construye.
build:
	@# cmake no vuelve a buscar el compilador si ya está en la caché: si el que
	@# hay grabado ya no existe (toolchain movido/borrado), se regenera el build dir.
	@cc=$$(sed -n 's/^CMAKE_C_COMPILER:[A-Z]*=//p' $(BUILD)/CMakeCache.txt 2>/dev/null); \
	if [ -n "$$cc" ] && [ ! -x "$$cc" ]; then \
	  echo "==> el compilador cacheado ya no existe ($$cc): regenerando $(BUILD)"; \
	  rm -rf $(BUILD); \
	fi
	cmake -S . -B $(BUILD) -G Ninja \
	  -DPICO_SDK_PATH=$(PICO_SDK_PATH) \
	  $(if $(PICO_TOOLCHAIN_PATH),-DPICO_TOOLCHAIN_PATH=$(PICO_TOOLCHAIN_PATH)) \
	  -DPICO_BOARD=$(BOARD) -DSAVIA_ENABLE_BLE=$(BLE) -DSAVIA_ON_DEVICE_INFERENCE=$(INFER) \
	  -DSAVIA_MOCK_DATA=$(MOCK) \
	  -DSAVIA_LORA_DEV_EUI=$(LORA_DEV_EUI) \
	  -DSAVIA_LORA_APP_EUI=$(LORA_APP_EUI) \
	  -DSAVIA_LORA_APP_KEY=$(LORA_APP_KEY)
	ninja -C $(BUILD)
	@echo "==> $(UF2)"

# BOOTSEL monta RPI-RP2 en RP2040 (pico_w) y RP2350 en RP2350 (pico2_w).
flash: build
	@vol=""; \
	for v in /Volumes/RPI-RP2 /Volumes/RP2350; do \
	  test -d "$$v" && vol="$$v" && break; \
	done; \
	test -n "$$vol" || { \
	  echo "BOOTSEL no montada (busqué RPI-RP2 y RP2350). Pon la Pico en BOOTSEL (mantén el botón al conectar el USB)."; \
	  exit 1; }; \
	echo ""; \
	echo "  !! ADVERTENCIA -- flashear $$vol reinicia el micro:"; \
	echo "     * las lecturas almacenadas se PIERDEN (viven en RAM)"; \
	echo "     * la config guardada (clave BLE, coords, pines, LoRa) se RESETEA a"; \
	echo "       los valores por defecto (salvo mismo layout/versión de firmware)"; \
	for line in $(FLASH_MOCK_NOTE); do echo "     $$line"; done; \
	echo ""; \
	if [ "$(YES)" != "1" ]; then \
	  printf "  Escribe 'y' para continuar [y/N]: "; read ans; \
	  case "$$ans" in [yY]*) ;; *) echo "  Cancelado."; exit 1;; esac; \
	fi; \
	echo "Copiando $(UF2) -> $$vol/"; \
	cp $(UF2) "$$vol/"; \
	echo "==> Flasheado. La Pico se reinicia sola."

test:
	sh test/run_host_tests.sh

clean:
	rm -rf $(BUILD)

help:
	@sed -n '1,20p' Makefile
