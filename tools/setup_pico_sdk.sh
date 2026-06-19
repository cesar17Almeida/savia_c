#!/usr/bin/env sh
# One-time setup to CROSS-COMPILE the firmware (.uf2) on this machine.
#
# Compiling needs NO Pico hardware -- you build the .uf2 on the PC and only need
# a physical Pico to flash/run it. This installs the ARM cross-compiler + build
# tools and clones the Pico SDK (with its submodules: BTstack, CYW43, TinyUSB).
set -e

echo "== 1/3 build tools (Homebrew) =="
brew install cmake ninja
# Official Arm embedded toolchain (provides arm-none-eabi-gcc):
brew install --cask gcc-arm-embedded || brew install arm-none-eabi-gcc

echo "== 2/3 Pico SDK (shallow clone + submodules) =="
SDK_DIR="${PICO_SDK_PATH:-$HOME/pico-sdk}"
if [ ! -d "$SDK_DIR/.git" ]; then
  git clone -b master --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/raspberrypi/pico-sdk "$SDK_DIR"
else
  echo "  ya existe: $SDK_DIR"
fi

echo "== 3/3 listo =="
echo
echo "Añade esto a tu shell (~/.zshrc):"
echo "    export PICO_SDK_PATH=$SDK_DIR"
echo
echo "Luego compila con los comandos de docs/BUILD.md (no necesitas la placa)."
