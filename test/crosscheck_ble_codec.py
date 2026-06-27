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
    with open("/tmp/savia_config_patch.cbor", "wb") as f:
        f.write(cbor2.dumps({"v": 1, "op": "set", "name": "Huerta-1", "sleep_s": 300,
                             "deep_sleep": True, "capture_s": 120, "daily_hour": 6,
                             "mock": True, "log_level": 0}))
    # ingest: timestamped points (mix of air_temperature + soil_moisture w/ depth)
    with open("/tmp/savia_ingest.cbor", "wb") as f:
        f.write(cbor2.dumps({"v": 1, "op": "ingest", "data": [
            {"ts_ms": 1781996400000, "kind": "air_temperature", "value": 22.5},
            {"ts_ms": 1782000000000, "kind": "air_temperature", "value": 24.1},
            {"ts_ms": 1782000000000, "kind": "soil_moisture", "value": 0.42, "depth_cm": 30},
        ]}))
    print("gen: wrote request / time_sync / weather / config_patch / ingest inputs")


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

    # 5) status (incl. the LoRa link block)
    st = load("status")
    assert set(st.keys()) == {"v", "fw", "uptime_s", "last_sync_ms", "weather_updated_ms", "lora"}
    assert st["v"] == 1 and st["fw"] == "0.1.0-c" and st["uptime_s"] == 12345
    assert st["last_sync_ms"] == 1700000000000 and st["weather_updated_ms"] is None
    lora = st["lora"]
    assert set(lora.keys()) == {"inited", "joined", "rssi", "snr", "last_ms", "module", "seq"}
    assert lora["inited"] is True and lora["joined"] is True
    assert lora["rssi"] == -106 and abs(lora["snr"] - 7.0) < 1e-6
    assert lora["last_ms"] == 1700000000000
    assert lora["module"] == "v4.0.11" and lora["seq"] == 3
    print("check: status OK (incl. lora block)")

    # 6) count
    assert load("count") == {"count": 42}
    print("check: count OK")

    # 7) config snapshot (device card + sleep + schedule + sensors)
    cfg = load("config")
    assert set(cfg.keys()) == {"v", "device", "name", "sleep_s", "deep_sleep", "capture_s",
                               "daily_hour", "mock", "log_level", "wake_gpio", "sensors"}
    assert cfg["v"] == 1 and cfg["name"] == "Savia" and cfg["sleep_s"] == 3600 and cfg["wake_gpio"] == 15
    assert cfg["deep_sleep"] is False   # default OFF
    assert cfg["capture_s"] == 3600 and cfg["daily_hour"] == 20
    assert cfg["mock"] is True and cfg["log_level"] == 1   # dev defaults
    dev = cfg["device"]
    assert set(dev.keys()) == {"model", "mcu", "fw"}   # liveness lives in status; app maps image by model
    assert dev["model"] == "Raspberry Pi Pico WH" and dev["mcu"] == "RP2040"
    assert len(cfg["sensors"]) == 1
    s0 = cfg["sensors"][0]
    # interval_s = 0 -> the sensor follows the global capture_s (default config).
    assert s0 == {"port": 1, "gpio": 2, "type": "sdi12_aquacheck", "addr": "0", "interval_s": 0}
    print("check: config snapshot OK (device card + schedule + sensors)")

    # 8) config ack
    ack = load("config_ack")
    assert ack == {"v": 1, "op": "config_ok", "sleep_s": 300, "deep_sleep": True}
    print("check: config ack OK")

    # 9) logs (array of text lines)
    logs = cbor2.loads(open("/tmp/savia_logs.cbor", "rb").read())
    assert logs == ["boot ok", "BLE: central connected", "sched: daily cycle (hour=20)"]
    print("check: logs OK")

    # 10) ingest: C parsed the 3 timestamped points back into readings
    ing = load("ingest_parsed")
    assert len(ing) == 3
    assert ing[0]["kind"] == "air_temperature" and ing[0]["ts_ms"] == 1781996400000
    assert ing[0]["depth_cm"] == 0 and ing[0]["port"] == 1 and abs(ing[0]["value"] - 22.5) < 1e-4
    assert ing[2]["kind"] == "soil_moisture" and ing[2]["depth_cm"] == 30
    assert abs(ing[2]["value"] - 0.42) < 1e-4
    assert load("ingest_ok") == {"v": 1, "op": "ingest_ok", "created": 3, "updated": 1}
    print("check: ingest OK")

    # 11) pinmap snapshot (GPIO inventory: free / in_use / reserved + caps bitmask)
    pm = load("pinmap")
    assert set(pm.keys()) == {"v", "pins"}
    assert pm["v"] == 1 and len(pm["pins"]) == 30
    by = {p["gpio"]: p for p in pm["pins"]}
    assert by[2]["state"] == "in_use" and by[2]["reason"] == "sensor" and by[2]["port"] == 1
    assert by[15]["state"] == "reserved" and by[15]["reason"] == "wake_btn"
    assert by[23]["state"] == "reserved" and by[23]["reason"] == "wireless"
    assert by[29]["reason"] == "wireless"
    assert by[6]["state"] == "free" and by[6]["reason"] == ""
    assert "port" not in by[6] and "port" in by[2]   # port only on sensor pins
    assert by[26]["caps"] & 0x08 and not (by[0]["caps"] & 0x08)   # ADC (bit 3) only on GP26..28
    print("check: pinmap OK (GPIO inventory: free/in_use/reserved + caps)")

    print("CROSSCHECK OK")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "gen":
        gen()
    elif mode == "check":
        check()
    else:
        sys.exit("usage: crosscheck_ble_codec.py gen|check")
