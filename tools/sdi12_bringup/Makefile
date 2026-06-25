# Standalone SDI-12 bring-up test (separate from the main firmware).
#   make build   # compile build/sdi12_bringup.uf2
#   make flash   # build + copy to the Pico in BOOTSEL (RPI-RP2)
#   make clean
# Needs PICO_SDK_PATH + arm-none-eabi-gcc on PATH (same env as the root project).
BOARD ?= pico_w
UF2 := build/sdi12_bringup.uf2

.DEFAULT_GOAL := build
.PHONY: build flash clean

build:
	cmake -S . -B build -G Ninja -DPICO_BOARD=$(BOARD)
	ninja -C build
	@echo "==> $(UF2)"

flash: build
	@test -d /Volumes/RPI-RP2 || { \
	  echo "RPI-RP2 no montada. Pon la Pico en BOOTSEL (mantén el botón al conectar el USB)."; \
	  exit 1; }
	cp $(UF2) /Volumes/RPI-RP2/
	@echo "==> Flasheado. La Pico se reinicia sola."

clean:
	rm -rf build
