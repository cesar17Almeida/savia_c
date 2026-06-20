#!/usr/bin/env python3
"""Cross-check the C BLE codec against savia_py's OWN codec.

If savia_py (cbor2 + its chunked_decode -- the contract TerraLink mirrors)
decodes/reassembles what the C firmware produced, the wire format matches and
TerraLink reads it unchanged.

Usage: crosscheck_ble_codec.py gen|check   (run via savia_py's venv python)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SAVIA_PY = os.path.normpath(os.path.join(HERE, "..", "..", "savia_py"))

import cbor2  # from savia_py's venv


def gen():
    # Inputs for the C parsers to read.
    with open("/tmp/savia_req.cbor", "wb") as f:
        f.write(cbor2.dumps({"v": 1, "op": "get", "kind": "raw",
                             "from": 1000, "to": 2000, "limit": 50}))
    with open("/tmp/savia_timesync.cbor", "wb") as f:
        f.write(cbor2.dumps({"v": 1, "op": "set", "ms": 1700000000000}))
    with open("/tmp/savia_weather.cbor", "wb") as f:
        f.write(cbor2.dumps({"v": 1, "op": "upd",
                             "data": {"past_ta_hourly": [20.0] * 4,
                                      "future_ta_hourly": [22.0] * 2}}))
    print("gen: wrote request / time_sync / weather inputs")


def check():
    sys.path.insert(0, SAVIA_PY)
    from savia.ble.codec import chunked_decode  # the exact savia_py reassembler

    def load(name):
        return cbor2.loads(open(f"/tmp/savia_{name}.cbor", "rb").read())

    # 1) readings
    data = open("/tmp/savia_readings.cbor", "rb").read()
    rows = cbor2.loads(data)
    assert len(rows) == 2
    assert set(rows[0].keys()) == {"ts_ms", "port", "kind", "value", "depth_cm"}
    assert rows[0]["ts_ms"] == 1700000000000 and rows[0]["kind"] == "soil_moisture"
    assert rows[0]["depth_cm"] == 10 and abs(rows[0]["value"] - 0.75) < 1e-6
    print("check: readings OK")

    # 2) chunk frames reassembled by savia_py's chunked_decode
    raw = open("/tmp/savia_frames.bin", "rb").read()
    frames, i = [], 0
    while i < len(raw):
        ln = (raw[i] << 8) | raw[i + 1]; i += 2
        frames.append(raw[i:i + ln]); i += ln
    assert chunked_decode(frames) == data
    print(f"check: {len(frames)} frames reassembled byte-identical")

    # 3) aggregations
    aggs = load("aggs")
    assert len(aggs) == 1
    a = aggs[0]
    assert set(a.keys()) == {"hour_ms", "port", "kind", "count", "mean", "min", "max", "depth_cm"}
    assert a["count"] == 3 and a["kind"] == "soil_moisture" and a["depth_cm"] == 10
    assert abs(a["mean"] - 0.6) < 1e-6 and abs(a["min"] - 0.5) < 1e-6 and abs(a["max"] - 0.7) < 1e-6
    print("check: aggregations OK")

    # 4) predictions (null port/confidence on the second)
    preds = load("preds")
    assert len(preds) == 2
    assert set(preds[0].keys()) == {"ts_ms", "model", "kind", "port", "value", "confidence"}
    assert preds[0]["model"] == "lstm-hs30" and preds[0]["kind"] == "hs30_forecast"
    assert preds[0]["port"] == 1 and abs(preds[0]["confidence"] - 0.9) < 1e-6
    assert preds[1]["port"] is None and preds[1]["confidence"] is None
    print("check: predictions OK (incl. CBOR null)")

    # 5) status
    st = load("status")
    assert set(st.keys()) == {"v", "fw", "uptime_s", "last_sync_ms", "weather_updated_ms"}
    assert st["v"] == 1 and st["fw"] == "0.1.0-c" and st["uptime_s"] == 12345
    assert st["last_sync_ms"] == 1700000000000 and st["weather_updated_ms"] is None
    print("check: status OK")

    # 6) count
    assert load("count") == {"count": 42}
    print("check: count OK")

    print("CROSSCHECK OK")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "gen":
        gen()
    elif mode == "check":
        check()
    else:
        sys.exit("usage: crosscheck_ble_codec.py gen|check")
