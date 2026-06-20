#!/usr/bin/env python3
"""Cross-check the C BLE codec against savia_py's OWN codec.

If savia_py (cbor2 + its chunked_decode -- the contract TerraLink mirrors)
decodes/reassembles what the C firmware produced, the wire format matches and
TerraLink will read it unchanged.

Usage: crosscheck_ble_codec.py gen|check  (run via savia_py's venv python)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SAVIA_PY = os.path.normpath(os.path.join(HERE, "..", "..", "savia_py"))

import cbor2  # from savia_py's venv


def gen():
    req = {"v": 1, "op": "get", "kind": "raw", "from": 1000, "to": 2000, "limit": 50}
    with open("/tmp/savia_req.cbor", "wb") as f:
        f.write(cbor2.dumps(req))
    print("gen: wrote /tmp/savia_req.cbor (data_request for C to parse)")


def check():
    sys.path.insert(0, SAVIA_PY)
    from savia.ble.codec import chunked_decode  # the exact savia_py reassembler

    # 1) Readings decode with cbor2 (== what TerraLink's CBOR decoder sees).
    data = open("/tmp/savia_readings.cbor", "rb").read()
    rows = cbor2.loads(data)
    assert isinstance(rows, list) and len(rows) == 2, rows
    r0, r1 = rows
    assert set(r0.keys()) == {"ts_ms", "port", "kind", "value", "depth_cm"}, r0.keys()
    assert r0["ts_ms"] == 1700000000000, r0
    assert r0["port"] == 1 and r0["depth_cm"] == 10
    assert r0["kind"] == "soil_moisture", r0
    assert abs(r0["value"] - 0.75) < 1e-6, r0
    assert r1["depth_cm"] == 30 and abs(r1["value"] - 0.77) < 1e-6, r1
    print("check: readings decode OK ->", rows)

    # 2) Frames reassemble via savia_py's chunked_decode.
    raw = open("/tmp/savia_frames.bin", "rb").read()
    frames, i = [], 0
    while i < len(raw):
        ln = (raw[i] << 8) | raw[i + 1]
        i += 2
        frames.append(raw[i:i + ln])
        i += ln
    reassembled = chunked_decode(frames)
    assert reassembled == data, "chunked_decode result != original readings CBOR"
    print(f"check: {len(frames)} frames reassembled by savia_py.chunked_decode -> "
          f"byte-identical ({len(reassembled)} B)")

    print("CROSSCHECK OK")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "gen":
        gen()
    elif mode == "check":
        check()
    else:
        sys.exit("usage: crosscheck_ble_codec.py gen|check")
