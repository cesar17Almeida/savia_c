#!/usr/bin/env python3
"""Replay the training dataset through the LSTM the firmware embeds.

Two jobs, one pass over the data:

  1. Quantify. The .h5 -> .tflite conversion was checked for fidelity to itself
     (0.04 % int8 vs float), but that says nothing about how well the model
     predicts. This scores the int8 model the Pico actually runs against the
     ground truth in `dataset_master_hourly.csv`: MAE / RMSE / bias overall and
     per horizon, over as many windows as the dataset holds.

  2. Export one window for the board test. TerraLink replays a real 48 h window
     into the station over BLE, so it needs one -- picked here rather than by
     hand, with the model's own forecast and the measured truth alongside it, so
     what the phone shows can be checked against what the host computed.

Usage:
    python tools/replay_lstm_dataset.py --node 4 --limit 400
    python tools/replay_lstm_dataset.py --export ../terralink_app/composeApp/src/commonMain/composeResources/files/lstm_replay_window.csv
"""
import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime

import numpy as np

PAST_STEPS = 48
FUTURE_STEPS = 24

# The notebook's evaluation protocol (03_01_LSTM_Modeling.ipynb), reproduced so
# the number here is comparable to the one the model was signed off with.
#
# The split is temporal: node 4's 2018/19/21/22 trained it, 2020 and 2023 are
# held out. And the model is only ever evaluated on DRY-DOWN windows -- a jump up
# inside the 24 h being predicted means it rained or the field was irrigated, and
# the model was never given a rain or irrigation input, so scoring it there scores
# something it cannot know. Both thresholds are the notebook's.
TEST_YEARS = (2020, 2023)
TOLERANCE_JUMP = 0.005     # max hour-to-hour rise inside the target window
TOLERANCE_GLOBAL = 0.01    # max net rise from first to last hour

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DATASET = os.path.join(ROOT, "docs/sensor_documentation/model/dataset_lstm/dataset_master_hourly.csv")
SCALER = os.path.join(ROOT, "docs/sensor_documentation/model/scaler/scaler_params.json")
MODEL_INT8 = os.path.join(ROOT, "docs/sensor_documentation/model/modelo_lstm/lstm_hs30_int8.tflite")
MODEL_FLOAT = os.path.join(ROOT, "docs/sensor_documentation/model/modelo_lstm/lstm_hs30.tflite")


def load_scaler():
    """mu/sigma keyed by variable. The scaler's own column order is
    [HS30, TA, HS10]; the tensors want [TA, HS10, HS30], so scale by name and
    reorder afterwards -- never by position."""
    j = json.load(open(SCALER))
    order = j["feature_order_scaler"]
    mu = dict(zip(order, j["mean"]))
    sd = dict(zip(order, j["scale_std"]))
    return mu, sd


def is_drying(rows, i):
    """The notebook's physical filter, applied to the 24 h being predicted."""
    fut = [rows[i + PAST_STEPS + k]["HS30"] for k in range(FUTURE_STEPS)]
    for k in range(1, FUTURE_STEPS):
        if fut[k] - fut[k - 1] > TOLERANCE_JUMP:
            return False
    return (fut[-1] - fut[0]) <= TOLERANCE_GLOBAL


def load_rows(node, years=None):
    out = []
    with open(DATASET) as fh:
        for r in csv.DictReader(fh):
            if node is not None and r["id_nodo"] != str(node):
                continue
            if years and not r["instante"].startswith(tuple(str(y) for y in years)):
                continue
            try:
                t = datetime.strptime(r["instante"], "%Y-%m-%d %H:%M:%S")
                out.append({
                    "t": t,
                    "HS10": float(r["HS10_mean"]),
                    "HS20": float(r["HS20_mean"]),
                    "HS30": float(r["HS30_mean"]),
                    "TA": float(r["TA_mean"]),
                })
            except (ValueError, KeyError):
                continue        # a gap in the record is not a window we can use
    out.sort(key=lambda r: r["t"])
    return out


def contiguous_windows(rows):
    """Index of every window whose 72 rows are consecutive whole hours.

    The dataset has gaps, and a window that silently skips six hours would score
    the model on a history it never had."""
    span = PAST_STEPS + FUTURE_STEPS
    for i in range(len(rows) - span + 1):
        ok = True
        for k in range(1, span):
            if (rows[i + k]["t"] - rows[i + k - 1]["t"]).total_seconds() != 3600:
                ok = False
                break
        if ok:
            yield i


def build_tensors(rows, i, mu, sd):
    past = rows[i:i + PAST_STEPS]
    fut = rows[i + PAST_STEPS:i + PAST_STEPS + FUTURE_STEPS]
    x_past = np.zeros((1, PAST_STEPS, 3), dtype=np.float32)
    for k, r in enumerate(past):
        x_past[0, k, 0] = (r["TA"] - mu["TA_mean"]) / sd["TA_mean"]
        x_past[0, k, 1] = (r["HS10"] - mu["HS10_mean"]) / sd["HS10_mean"]
        x_past[0, k, 2] = (r["HS30"] - mu["HS30_mean"]) / sd["HS30_mean"]
    x_fut = np.zeros((1, FUTURE_STEPS, 1), dtype=np.float32)
    for k, r in enumerate(fut):
        x_fut[0, k, 0] = (r["TA"] - mu["TA_mean"]) / sd["TA_mean"]
    truth = np.array([r["HS30"] for r in fut], dtype=np.float32)
    return x_past, x_fut, truth


def make_runner(model_path):
    import tensorflow as tf
    interp = tf.lite.Interpreter(model_path=model_path)
    interp.allocate_tensors()
    ins = interp.get_input_details()
    out = interp.get_output_details()[0]
    # Input order is not guaranteed to be (past, future): bind by shape.
    i_past = next(d for d in ins if tuple(d["shape"][1:]) == (PAST_STEPS, 3))
    i_fut = next(d for d in ins if tuple(d["shape"][1:]) == (FUTURE_STEPS, 1))

    def run(x_past, x_fut):
        interp.set_tensor(i_past["index"], x_past)
        interp.set_tensor(i_fut["index"], x_fut)
        interp.invoke()
        return interp.get_tensor(out["index"])[0].astype(np.float32)

    return run


def unscale(y, mu, sd):
    return y * sd["HS30_mean"] + mu["HS30_mean"]


def report(name, errors):
    e = np.concatenate([e for e in errors])
    mae = float(np.abs(e).mean())
    rmse = float(math.sqrt((e ** 2).mean()))
    bias = float(e.mean())
    print(f"  {name:<12} MAE {mae:.5f}   RMSE {rmse:.5f}   sesgo {bias:+.5f}   n={e.size}")
    return mae, rmse, bias


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", type=int, default=4)
    ap.add_argument("--limit", type=int, default=0, help="max windows (0 = all)")
    ap.add_argument("--stride", type=int, default=1, help="hours between windows")
    ap.add_argument("--years", default=",".join(str(y) for y in TEST_YEARS),
                    help="years to evaluate; empty string = the whole record")
    ap.add_argument("--all-windows", action="store_true",
                    help="skip the drying filter and score every window (the station's real duty cycle)")
    ap.add_argument("--model", default=MODEL_INT8)
    ap.add_argument("--compare-float", action="store_true",
                    help="also run the un-quantized model, to separate model error from int8 error")
    ap.add_argument("--export", default=None, help="write the chosen window as CSV for TerraLink")
    ap.add_argument("--export-index", type=int, default=None,
                    help="window index to export (default: the most illustrative)")
    args = ap.parse_args()

    mu, sd = load_scaler()
    years = tuple(int(y) for y in args.years.split(",") if y.strip()) or None
    rows = load_rows(args.node, years)
    idx = list(contiguous_windows(rows))
    n_contig = len(idx)
    if not args.all_windows:
        idx = [i for i in idx if is_drying(rows, i)]
    if args.stride > 1:
        idx = idx[::args.stride]
    if args.limit:
        idx = idx[:args.limit]
    scope = "todas las ventanas" if args.all_windows else "solo secado (protocolo del notebook)"
    print(f"nodo {args.node} · años {years or 'todos'} · {scope}")
    print(f"{len(rows)} horas, {n_contig} ventanas de 72 h contiguas, {len(idx)} evaluadas")
    if not idx:
        sys.exit("sin ventanas contiguas")

    run_int8 = make_runner(args.model)
    run_float = make_runner(MODEL_FLOAT) if args.compare_float else None

    per_window = []
    errs_int8, errs_float, errs_persist = [], [], []
    by_h_int8 = [[] for _ in range(FUTURE_STEPS)]

    for i in idx:
        x_past, x_fut, truth = build_tensors(rows, i, mu, sd)
        pred = unscale(run_int8(x_past, x_fut), mu, sd)
        e = pred - truth
        errs_int8.append(e)
        for h in range(FUTURE_STEPS):
            by_h_int8[h].append(abs(float(e[h])))
        # Persistence baseline: "tomorrow looks like the last hour measured".
        # Without it, an MAE has no scale -- a model has to beat doing nothing.
        last = rows[i + PAST_STEPS - 1]["HS30"]
        errs_persist.append(np.full(FUTURE_STEPS, last, dtype=np.float32) - truth)
        if run_float:
            errs_float.append(unscale(run_float(x_past, x_fut), mu, sd) - truth)
        per_window.append((i, float(np.abs(e).mean()), float(truth.max() - truth.min())))

    print("\nError frente al HS30 medido (VWC 0..1):")
    report("int8 (placa)", errs_int8)
    if run_float:
        report("float", errs_float)
    report("persistencia", errs_persist)

    print("\nMAE por horizonte:")
    for h in range(0, FUTURE_STEPS, 4):
        chunk = [f"H+{h + k + 1}:{np.mean(by_h_int8[h + k]):.4f}" for k in range(4) if h + k < FUTURE_STEPS]
        print("  " + "  ".join(chunk))

    # A window worth showing on a phone: enough movement in the truth that the
    # forecast is not a flat line, and an error near the model's own median so it
    # is representative rather than flattering.
    med = float(np.median([w[1] for w in per_window]))
    moving = [w for w in per_window if w[2] > 0.01] or per_window
    chosen = min(moving, key=lambda w: abs(w[1] - med))
    ci = args.export_index if args.export_index is not None else chosen[0]
    print(f"\nVentana elegida: idx={ci}  inicio={rows[ci]['t']}  MAE={chosen[1]:.5f}  recorrido={chosen[2]:.4f}")

    if args.export:
        x_past, x_fut, truth = build_tensors(rows, ci, mu, sd)
        pred = unscale(run_int8(x_past, x_fut), mu, sd)
        os.makedirs(os.path.dirname(args.export), exist_ok=True)
        with open(args.export, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["# nodo", args.node, "inicio", rows[ci]["t"].isoformat(),
                        "mae_host", f"{float(np.abs(pred - truth).mean()):.6f}"])
            w.writerow(["seccion", "h", "hs10", "hs20", "hs30", "ta"])
            for k in range(PAST_STEPS):
                r = rows[ci + k]
                w.writerow(["past", k - (PAST_STEPS - 1), f"{r['HS10']:.6f}",
                            f"{r['HS20']:.6f}", f"{r['HS30']:.6f}", f"{r['TA']:.4f}"])
            for k in range(FUTURE_STEPS):
                r = rows[ci + PAST_STEPS + k]
                w.writerow(["future", k + 1, "", "", f"{r['HS30']:.6f}", f"{r['TA']:.4f}"])
            for k in range(FUTURE_STEPS):
                w.writerow(["host_pred", k + 1, "", "", f"{float(pred[k]):.6f}", ""])
        print(f"exportado -> {args.export}")


if __name__ == "__main__":
    main()
