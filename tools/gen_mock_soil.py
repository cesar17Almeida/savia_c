#!/usr/bin/env python3
"""Generate include/savia/mock_soil_data.h: the soil series the mock probe replays.

The mock probe replays measured hourly soil moisture (HS10, HS30) from node 4 of
the LSTM dataset. The default stretch is September 2020: a season held out of the
model's training (2018/19/21/22) and the same time of year as the demos.

It also embeds one 72 h window for the boot self-test: that window's own air
temperature, and the forecast the host computes for it with the exact model
bytes the firmware embeds, using the firmware's float32 scaling and int8
quantization. On the board, any difference from that forecast comes from TFLM,
not from the data. The host runs TFLite's builtin kernels WITHOUT the default
XNNPACK delegate: XNNPACK's int8 path moves this model's output by up to 0.005,
and the reference resolver cannot load its ELU op.

Usage (TensorFlow is needed for the host forecast: TFM/.venv-tflite):
    ../../.venv-tflite/bin/python tools/gen_mock_soil.py
    ../../.venv-tflite/bin/python tools/gen_mock_soil.py --start "2020-09-01 00:00" --days 29
"""
import argparse
import csv
import hashlib
import json
import os
import re
import sys
from datetime import datetime, timedelta

import numpy as np

PAST, FUTURE = 48, 24
TEST_YEARS = (2020, 2023)   # node 4's years held out of the LSTM's training

HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(FIRMWARE, "..", ".."))
DATASET = os.path.join(ROOT, "docs/sensor_documentation/model/dataset_lstm/dataset_master_hourly.csv")
SCALER = os.path.join(ROOT, "docs/sensor_documentation/model/scaler/scaler_params.json")
MODEL_HEADER = os.path.join(FIRMWARE, "include/savia/lstm_hs30_int8_model.h")
OUT = os.path.join(FIRMWARE, "include/savia/mock_soil_data.h")


def load_stretch(node, start, hours):
    """Consecutive hourly rows [start, start + hours); fails on any gap or blank."""
    want = {start + timedelta(hours=k): k for k in range(hours)}
    rows = [None] * hours
    with open(DATASET) as fh:
        for r in csv.DictReader(fh):
            if r["id_nodo"] != str(node):
                continue
            t = datetime.strptime(r["instante"], "%Y-%m-%d %H:%M:%S")
            k = want.get(t)
            if k is None:
                continue
            try:
                rows[k] = {"t": t, "hs10": float(r["HS10_mean"]),
                           "hs30": float(r["HS30_mean"]), "ta": float(r["TA_mean"])}
            except ValueError:
                sys.exit(f"blank value at {t}: pick another stretch")
    missing = [start + timedelta(hours=k) for k, r in enumerate(rows) if r is None]
    if missing:
        sys.exit(f"{len(missing)} missing hours, first {missing[0]}: pick another stretch")
    return rows


def f32(x):
    return np.float32(x)


class Firmware:
    """The on-device pipeline, reproduced in float32 (scaler.c, lstm_input.c,
    inference_tflm.cpp) around TFLite's builtin int8 kernels."""

    def __init__(self):
        import tensorflow as tf
        sc = json.load(open(SCALER))
        order = sc["feature_order_scaler"]
        self.mu = {k: f32(v) for k, v in zip(order, sc["mean"])}
        self.sd = {k: f32(v) for k, v in zip(order, sc["scale_std"])}
        self.model = embedded_model()
        resolver = tf.lite.experimental.OpResolverType.BUILTIN_WITHOUT_DEFAULT_DELEGATES
        self.interp = tf.lite.Interpreter(model_content=self.model,
                                          experimental_op_resolver_type=resolver)
        self.interp.allocate_tensors()
        self.ins = self.interp.get_input_details()
        self.out = self.interp.get_output_details()[0]

    def scale(self, x, name):
        return (f32(x) - self.mu[name]) / self.sd[name]

    @staticmethod
    def quantize(x, detail):
        if detail["dtype"] != np.int8:
            return x.astype(np.float32)
        s, zp = detail["quantization"]
        v = (x / f32(s)).astype(np.float64)
        q = np.sign(v) * np.floor(np.abs(v) + 0.5)          # lroundf: half away from zero
        return np.clip(q + zp, -128, 127).astype(np.int8)

    def forecast(self, past_rows, future_ta):
        past = np.zeros((1, PAST, 3), dtype=np.float32)        # model order [TA, HS10, HS30]
        for t, r in enumerate(past_rows):
            past[0, t] = (self.scale(r["ta"], "TA_mean"), self.scale(r["hs10"], "HS10_mean"),
                          self.scale(r["hs30"], "HS30_mean"))
        fut = np.array([[[self.scale(v, "TA_mean")] for v in future_ta]], dtype=np.float32)
        for d in self.ins:                                      # bound by size, as the shim does
            src = past if int(np.prod(d["shape"][1:])) == PAST * 3 else fut
            self.interp.set_tensor(d["index"], self.quantize(src, d))
        self.interp.invoke()
        y = self.interp.get_tensor(self.out["index"]).reshape(-1)
        if self.out["dtype"] == np.int8:
            s, zp = self.out["quantization"]
            y = (y.astype(np.int32) - zp).astype(np.float32) * f32(s)
        return y.astype(np.float32) * self.sd["HS30_mean"] + self.mu["HS30_mean"]


def embedded_model():
    """The .tflite bytes compiled into the firmware, read back from its header."""
    text = open(MODEL_HEADER).read()
    body = text[text.index("{") + 1:text.index("};")]
    return bytes(int(b, 16) for b in re.findall(r"0x([0-9a-fA-F]{2})", body))


def pick_window(rows, fw, index):
    """The self-test window: given, or the replay tool's rule -- truth that moves,
    error nearest the stretch's median, so it is neither flattering nor an outlier."""
    scored = []
    for i in range(len(rows) - PAST - FUTURE + 1):
        future = rows[i + PAST:i + PAST + FUTURE]
        pred = fw.forecast(rows[i:i + PAST], [r["ta"] for r in future])
        truth = np.array([f32(r["hs30"]) for r in future])
        scored.append((i, float(np.abs(pred - truth).mean()), float(truth.max() - truth.min())))
    if index is not None:
        return index, scored
    med = float(np.median([s[1] for s in scored]))
    moving = [s for s in scored if s[2] > 0.01] or scored
    return min(moving, key=lambda s: abs(s[1] - med))[0], scored


def c_float(v):
    return np.format_float_positional(f32(v), unique=True, trim="0") + "f"


def c_array(name, values, per_line=6):
    lines = []
    for k in range(0, len(values), per_line):
        lines.append("    " + ", ".join(c_float(v) for v in values[k:k + per_line]) + ",")
    return f"static const float {name}[{len(values)}] = {{\n" + "\n".join(lines) + "\n};\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", type=int, default=4)
    ap.add_argument("--start", default="2020-09-01 00:00", help="local midnight the replay starts at")
    ap.add_argument("--days", type=int, default=29)
    ap.add_argument("--selftest-index", type=int, default=None, help="first past row of the self-test window")
    args = ap.parse_args()

    start = datetime.strptime(args.start, "%Y-%m-%d %H:%M")
    if start.hour or start.minute:
        sys.exit("--start must be a midnight: the firmware maps row % 24 to the local hour")
    hours = args.days * 24
    rows = load_stretch(args.node, start, hours)
    if hours < PAST + FUTURE:
        sys.exit("the stretch must hold at least one 72 h window")

    fw = Firmware()
    sel, scored = pick_window(rows, fw, args.selftest_index)
    if not 0 <= sel <= hours - PAST - FUTURE:
        sys.exit("self-test window must fit inside the stretch")
    win = rows[sel:sel + PAST + FUTURE]
    host = fw.forecast(win[:PAST], [r["ta"] for r in win[PAST:]])
    truth = np.array([f32(r["hs30"]) for r in win[PAST:]])
    mae = float(np.abs(host - truth).mean())
    all_mae = float(np.mean([s[1] for s in scored]))
    end = rows[-1]["t"]
    sha = hashlib.sha256(fw.model).hexdigest()[:16]

    out = []
    out.append("// Auto-generated by tools/gen_mock_soil.py -- do not edit.\n")
    out.append("//\n")
    out.append(f"// Soil moisture the MOCK probe replays: node {args.node} of dataset_master_hourly.csv,\n")
    held = "held out of training" if start.year in TEST_YEARS else "a TRAINING year"
    out.append(f"// {start:%Y-%m-%d %H:%M} to {end:%Y-%m-%d %H:%M} local ({hours} h; {start.year} is {held}).\n")
    out.append(f"// Self-test window: past from {win[0]['t']:%Y-%m-%d %H:%M}; host forecast with the\n")
    out.append(f"// embedded model (sha256 {sha}...), MAE {mae:.4f} vs measured\n")
    out.append(f"// (stretch average {all_mae:.4f} over {len(scored)} windows).\n")
    out.append("#ifndef SAVIA_MOCK_SOIL_DATA_H\n#define SAVIA_MOCK_SOIL_DATA_H\n\n")
    out.append(f"#define MOCK_SOIL_ROWS         {hours}   // whole days: row % 24 is the local hour\n")
    out.append(f"#define MOCK_SELFTEST_ROW      {sel}   // first past row of the self-test window\n\n")
    out.append(c_array("MOCK_SOIL_HS10", [r["hs10"] for r in rows]) + "\n")
    out.append(c_array("MOCK_SOIL_HS30", [r["hs30"] for r in rows]) + "\n")
    out.append("// Air temperature of the self-test window (48 past + 24 future, degC). The\n")
    out.append("// self-test's only: the station's TA always comes from the weather cache.\n")
    out.append(c_array("MOCK_SELFTEST_TA", [r["ta"] for r in win]) + "\n")
    out.append("// What the host predicted for that window (HS30, VWC 0..1).\n")
    out.append(c_array("MOCK_SELFTEST_HOST", list(host)) + "\n")
    out.append("#endif // SAVIA_MOCK_SOIL_DATA_H\n")
    text = "".join(out)

    # Every literal must parse back to the float32 the host computed with.
    for name, want in (("MOCK_SOIL_HS10", [r["hs10"] for r in rows]),
                       ("MOCK_SOIL_HS30", [r["hs30"] for r in rows]),
                       ("MOCK_SELFTEST_HOST", list(host))):
        body = text[text.index(name):]
        body = body[body.index("{") + 1:body.index("}")]
        got = [f32(float(v.rstrip("f"))) for v in re.findall(r"[-0-9.eE]+f", body)]
        assert got == [f32(v) for v in want], name

    with open(OUT, "w") as fh:
        fh.write(text)
    print(f"{OUT}: {hours} h from {start:%Y-%m-%d}, self-test row {sel} "
          f"({win[0]['t']:%Y-%m-%d %H:%M}), host MAE {mae:.4f}, stretch MAE {all_mae:.4f}")


if __name__ == "__main__":
    main()
