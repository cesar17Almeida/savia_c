#!/usr/bin/env sh
# Host unit tests: compile the SDK-free firmware logic NATIVELY (no Pico SDK,
# no hardware) and run assert-based checks. The BLE codec is cross-checked
# against savia_py's OWN codec, proving TerraLink-compatible output.
#
# Firmware cross-compile (the full .uf2) is a separate path -- see docs/BUILD.md.
set -e
cd "$(dirname "$0")/.."
CC="${CC:-cc}"
CFLAGS="-std=c11 -Wall -Wextra -Iinclude"

echo "== 1) config test =="
"$CC" $CFLAGS test/test_config.c src/system/config.c -o /tmp/savia_test_config
/tmp/savia_test_config

echo ""
echo "== 1b) pinmap test =="
"$CC" $CFLAGS test/test_pinmap.c src/system/pinmap.c src/system/config.c -o /tmp/savia_test_pinmap
/tmp/savia_test_pinmap

echo ""
echo "== 2) storage + clock test =="
"$CC" $CFLAGS test/test_storage.c src/storage/storage_flash.c src/system/clock.c -o /tmp/savia_test_storage
/tmp/savia_test_storage

echo ""
echo "== 2b) scheduler test =="
"$CC" $CFLAGS test/test_scheduler.c src/power/scheduler.c -o /tmp/savia_test_sched
/tmp/savia_test_sched

echo ""
echo "== 2c) sha256 + auth test =="
"$CC" $CFLAGS test/test_auth.c src/codec/sha256.c src/ble/auth.c -o /tmp/savia_test_auth
/tmp/savia_test_auth

echo ""
echo "== 2d) LoRa codec test =="
"$CC" $CFLAGS test/test_lora_codec.c src/codec/lora_codec.c -o /tmp/savia_test_lora
/tmp/savia_test_lora

echo ""
echo "== 3) BLE codec test + cross-check vs savia_py =="
PYTHON="../savia_py/.venv/bin/python"
if [ ! -x "$PYTHON" ]; then
  echo "  !! falta $PYTHON (venv de savia_py con cbor2); salto el cross-check"
else
  "$PYTHON" test/crosscheck_ble_codec.py gen
  "$CC" $CFLAGS test/test_ble_codec.c src/ble/ble_codec.c src/codec/cbor.c src/system/config.c src/system/pinmap.c -o /tmp/savia_test_ble
  /tmp/savia_test_ble
  "$PYTHON" test/crosscheck_ble_codec.py check
fi

echo ""
echo "ALL HOST TESTS PASSED"
