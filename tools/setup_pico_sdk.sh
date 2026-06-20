#!/usr/bin/env sh
# One-time setup to CROSS-COMPILE the firmware (.uf2) on this machine.
#
# Compiling needs NO Pico hardware -- you build the .uf2 on the PC and only need
# a physical Pico to flash/run it. Installs the ARM cross-compiler + build tools
# and clones the Pico SDK (with submodules: BTstack, CYW43, TinyUSB, lwIP).
set -e
export HOMEBREW_NO_AUTO_UPDATE=1

echo "== 1/3 build tools (Homebrew, sin sudo) =="
brew install cmake ninja

echo "== 2/3 ARM toolchain (formula, sin sudo) =="
if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  echo "  ya presente: $(arm-none-eabi-gcc --version | head -1)"
else
  brew install arm-none-eabi-gcc || {
    echo "  !! Formula no disponible. Instala el toolchain oficial de ARM a mano:"
    echo "       brew install --cask gcc-arm-embedded   # puede pedir tu contraseña de macOS"
    exit 2
  }
fi

echo "== 3/3 Pico SDK (shallow clone + submodules) =="
SDK_DIR="${PICO_SDK_PATH:-$HOME/pico-sdk}"
if [ -d "$SDK_DIR/.git" ]; then
  echo "  ya existe: $SDK_DIR"
else
  git clone -b master --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/raspberrypi/pico-sdk "$SDK_DIR"
fi

echo
echo "== LISTO =="
arm-none-eabi-gcc --version | head -1
echo "PICO_SDK_PATH=$SDK_DIR"
echo "Añade a ~/.zshrc:  export PICO_SDK_PATH=$SDK_DIR"
