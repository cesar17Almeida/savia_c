#!/usr/bin/env python3
"""Convert the HS30 LSTM to FULL-INTEGER int8 for TFLite Micro (RP2350 / Pico 2 W).

The existing dynamic-range int8 model (weights int8, activations float32) runs on
the Pi's tflite-runtime but NOT on TFLM: its hybrid FULLY_CONNECTED fails to
prepare (kernel_util.cpp:406, "false was not true"). This does post-training
FULL-integer quantization with a representative dataset built from the real
training data, so every op becomes int8. We keep FLOAT I/O (QUANTIZE/DEQUANTIZE at
the boundary) so the firmware shim (inference_tflm.cpp) still feeds/reads floats.

Run in .venv-tflite (full TensorFlow), NEVER on the Pi:
    .venv-tflite/bin/python development/savia_c/tools/convert_lstm_tflm_int8.py
"""
from __future__ import annotations
import argparse, csv, json, sys
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]                      # tools/savia_c/development/TFM -> TFM
MODEL_DIR = REPO / "docs/sensor_documentation/model"
H5 = MODEL_DIR / "modelo_lstm/modelo_indep_HS30_mean_seed_562.h5"
CSV = MODEL_DIR / "dataset_lstm/dataset_master_hourly.csv"
SCALER = MODEL_DIR / "scaler/scaler_params.json"
OUT = MODEL_DIR / "modelo_lstm/lstm_hs30_int8_full.tflite"

PAST_STEPS, FUTURE_STEPS, OUTPUT_LEN = 48, 24, 24
SEED = 562


def load_scaler() -> dict:
    d = json.loads(SCALER.read_text())
    return {name: (float(d["mean"][i]), float(d["scale_std"][i]))
            for i, name in enumerate(d["feature_order_scaler"])}


def load_windows(n_windows: int) -> list[tuple[np.ndarray, np.ndarray]]:
    """Real 48h-past + 24h-future windows from the master dataset, StandardScaler
    space, past order [TA,HS10,HS30] / future [TA] -- exactly what the model sees."""
    sc = load_scaler()
    rows_by_node: dict[str, list[dict]] = {}
    with open(CSV) as f:
        for row in csv.DictReader(f):
            rows_by_node.setdefault(row["id_nodo"], []).append(row)

    span = PAST_STEPS + FUTURE_STEPS
    cands = [(nid, s) for nid, rows in rows_by_node.items()
             for s in range(0, len(rows) - span)]
    rng = np.random.default_rng(SEED)
    rng.shuffle(cands)

    def col(rows, name):
        return np.array([float(r[name]) for r in rows], dtype=np.float32)

    def scale(name, x):
        m, s = sc[name]
        return (x - m) / s

    windows = []
    for nid, start in cands:
        if len(windows) >= n_windows:
            break
        rows = rows_by_node[nid][start:start + span]
        try:
            ta, hs10, hs30 = col(rows, "TA_mean"), col(rows, "HS10_mean"), col(rows, "HS30_mean")
        except (ValueError, KeyError):
            continue
        if not (np.isfinite(ta).all() and np.isfinite(hs10).all() and np.isfinite(hs30).all()):
            continue
        ta_s, hs10_s, hs30_s = scale("TA_mean", ta), scale("HS10_mean", hs10), scale("HS30_mean", hs30)
        past = np.stack([ta_s[:PAST_STEPS], hs10_s[:PAST_STEPS], hs30_s[:PAST_STEPS]], axis=-1)  # (48,3)
        future = ta_s[PAST_STEPS:span].reshape(FUTURE_STEPS, 1)                                   # (24,1)
        windows.append((past.astype(np.float32), future.astype(np.float32)))

    if not windows:
        sys.exit("ERROR: no valid windows built from the CSV")
    print(f"==> built {len(windows)} representative windows from real data")
    return windows


def input_order(model) -> list[str]:
    return ["past" if int(inp.shape[1]) == PAST_STEPS else "future" for inp in model.inputs]


def build_unrolled(model):
    import keras
    def clone_fn(layer):
        if isinstance(layer, keras.layers.LSTM):
            cfg = layer.get_config(); cfg["unroll"] = True
            return keras.layers.LSTM.from_config(cfg)
        return layer.__class__.from_config(layer.get_config())
    print("==> rebuilding with unroll=True on LSTM layers...")
    twin = keras.models.clone_model(model, clone_function=clone_fn)
    twin.set_weights(model.get_weights())
    return twin


def hs30_mean_scale() -> tuple[float, float]:
    m, s = load_scaler()["HS30_mean"]
    return m, s


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--h5", type=Path, default=H5)
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--n", type=int, default=300, help="representative windows")
    ap.add_argument("--io", choices=["float", "int8"], default="float",
                    help="model input/output dtype: float (QUANTIZE/DEQUANTIZE wrap) "
                         "or int8 (forces the tail to quantize -- no float FC)")
    args = ap.parse_args()

    import keras, tensorflow as tf
    if not args.h5.exists():
        sys.exit(f"ERROR: model not found: {args.h5}")
    print(f"==> loading {args.h5.name} (keras {keras.__version__})")
    model = keras.models.load_model(args.h5, compile=False)
    order = input_order(model)
    print(f"    inputs {[tuple(i.shape) for i in model.inputs]} order={order}, "
          f"outputs {[tuple(o.shape) for o in model.outputs]}")

    unrolled = build_unrolled(model)
    windows = load_windows(args.n)

    def repr_gen():
        for past, future in windows:
            feed = {"past": past[None, ...], "future": future[None, ...]}
            yield [feed[name] for name in order]

    print("==> converting FULL-INTEGER int8 (representative dataset, float I/O)...")
    conv = tf.lite.TFLiteConverter.from_keras_model(unrolled)
    conv.optimizations = [tf.lite.Optimize.DEFAULT]
    conv.representative_dataset = repr_gen
    conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    # Per-TENSOR (not per-channel) quantization for dense layers: the unrolled graph
    # has 242 FCs and TFLM allocates per-channel multiplier/shift arrays for each at
    # Prepare (~1 KB x 242 = ~250 KB of arena!). Per-tensor kills that overhead.
    conv._experimental_disable_per_channel_quantization_for_dense_layers = True  # noqa: SLF001
    if args.io == "int8":
        # Force int8 I/O: the converter must quantize the whole graph incl. the
        # output tail (no float FC left for TFLM to choke on). Shim then quantizes
        # inputs / dequantizes output using the tensors' scale+zero_point.
        conv.inference_input_type = tf.int8
        conv.inference_output_type = tf.int8
    tflite_model = conv.convert()

    # Op sanity: no Flex, and report the op set.
    interp = tf.lite.Interpreter(model_content=tflite_model); interp.allocate_tensors()
    ops = sorted({o["op_name"] for o in interp._get_ops_details()})
    flex = [o for o in ops if o.lower().startswith("flex")]
    print(f"    ops: {ops}")
    if flex:
        print(f"    !!! FLEX ops present: {flex} -- TFLM can't run these")
    import collections
    td_all = interp.get_tensor_details()
    dtc = collections.Counter(np.dtype(t["dtype"]).name for t in td_all)
    fc_bad = sum(1 for o in interp._get_ops_details() if o["op_name"] == "FULLY_CONNECTED"
                 and np.dtype(td_all[o["outputs"][0]]["dtype"]).name != "int8")
    print(f"    tensor dtypes: {dict(dtc)}  | FC with non-int8 output: {fc_bad} "
          f"(must be 0 for TFLM)")

    # Write the artifact FIRST so a flaky verify never loses it.
    args.out.write_bytes(tflite_model)
    in_det = interp.get_input_details()
    out_det = interp.get_output_details()[0]
    print(f"==> wrote {args.out} ({args.out.stat().st_size/1024:.0f} KB), "
          f"I/O dtype {in_det[0]['dtype'].__name__}/{out_det['dtype'].__name__}")

    # Accuracy vs the float Keras model on REAL windows (HS30 humidity units). Handles
    # both float and int8 I/O: quantize inputs / dequantize output when the model is int8.
    idx = {("past" if int(d["shape"][1]) == PAST_STEPS else "future"): d for d in in_det}
    mean, scl = hs30_mean_scale()

    def q(x, det):
        if det["dtype"] == np.int8:
            s, z = det["quantization"]
            return np.clip(np.round(x / s) + z, -128, 127).astype(np.int8)
        return x.astype(np.float32)

    def dq(v, det):
        if det["dtype"] == np.int8:
            s, z = det["quantization"]
            return (v.astype(np.float32) - z) * s
        return v.astype(np.float32)

    errs = []
    for past, future in windows[:64]:
        k = np.asarray(model.predict([{"past": past[None], "future": future[None]}[n] for n in order],
                                     verbose=0)).reshape(OUTPUT_LEN)
        interp.set_tensor(idx["past"]["index"], q(past[None], idx["past"]))
        interp.set_tensor(idx["future"]["index"], q(future[None], idx["future"]))
        interp.invoke()
        t = dq(interp.get_tensor(out_det["index"]).reshape(OUTPUT_LEN), out_det)
        errs.append(np.max(np.abs((k - t) * scl)))            # abs error in humidity units
    print(f"    accuracy vs .h5 (real HS30 units, 64 real windows): "
          f"max abs err = {max(errs):.5f}, mean abs err = {np.mean(errs):.5f} (HS30~{mean:.2f})")
    return 1 if flex else 0


if __name__ == "__main__":
    raise SystemExit(main())
