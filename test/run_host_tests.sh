#!/usr/bin/env sh
# Host unit tests: compile the SDK-free firmware logic NATIVELY (no Pico SDK,
# no hardware) and run assert-based checks. Catches logic/type errors on the PC.
#
# Firmware cross-compile (the full .uf2) is a separate path that needs the Pico
# SDK -- see docs/BUILD.md.
set -e
cd "$(dirname "$0")/.."
CC="${CC:-cc}"

echo "== building host tests with $CC =="
"$CC" -std=c11 -Wall -Wextra -Iinclude \
    test/test_config.c src/config.c -o /tmp/savia_test_config

echo "== running =="
/tmp/savia_test_config

echo "ALL HOST TESTS PASSED"
