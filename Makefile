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
	echo "     * mock arranca OFF; sólo TerraLink puede reactivarlo"; \
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
