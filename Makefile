# savia_c — wrapper fino sobre cmake + ninja para no recordar los flags.
#
#   make build           # compila el .uf2 (Pico W, BLE ON)  -> build-pico_w/
#   make flash           # build + copia el .uf2 a la Pico en BOOTSEL (RPI-RP2)
#   make test            # tests de host (lógica pura, sin SDK ni placa)
#   make clean           # borra el build dir
#
# Variables (sobreescribibles):
#   make build BOARD=pico2_w INFER=ON    # Pico 2 W con LSTM on-device
#   make build BLE=OFF                   # build mínimo sin radio
#
# Compilar necesita PICO_SDK_PATH + arm-none-eabi-gcc (ver tools/setup_pico_sdk.sh);
# `make test` no necesita nada de eso.

# pico_w (RP2040) | pico2_w (RP2350)
BOARD ?= pico_w
# BTstack + CYW43 (la app lo necesita)
BLE ?= ON
# LSTM on-device; ON solo cabe en pico2_w (RP2350)
INFER ?= OFF

BUILD := build-$(BOARD)
INFER_TAG := $(if $(filter ON,$(INFER)),on,off)
UF2 := $(BUILD)/savia_c-$(BOARD)-ml$(INFER_TAG)device.uf2

.DEFAULT_GOAL := build
.PHONY: build flash test clean help

# Reconfigura siempre (cmake es idempotente y barato) para que un cambio de
# BOARD/BLE/INFER se aplique, y luego construye.
build:
	cmake -S . -B $(BUILD) -G Ninja \
	  -DPICO_BOARD=$(BOARD) -DSAVIA_ENABLE_BLE=$(BLE) -DSAVIA_ON_DEVICE_INFERENCE=$(INFER)
	ninja -C $(BUILD)
	@echo "==> $(UF2)"

flash: build
	@test -d /Volumes/RPI-RP2 || { \
	  echo "RPI-RP2 no montada. Pon la Pico en BOOTSEL (mantén el botón al conectar el USB)."; \
	  exit 1; }
	cp $(UF2) /Volumes/RPI-RP2/
	@echo "==> Flasheado. La Pico se reinicia sola."

test:
	sh test/run_host_tests.sh

clean:
	rm -rf $(BUILD)

help:
	@sed -n '1,20p' Makefile
