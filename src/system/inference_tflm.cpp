// TFLite Micro interpreter for the on-device (RP2350) LSTM. C++ because TFLM is
// C++; exposes inference_run() with C linkage so the C pipeline in inference.c can
// call it. Only compiled/linked when SAVIA_ON_DEVICE_INFERENCE=1 (Pico 2 W).
//
// The embedded model (lstm_hs30_int8.tflite) has FLOAT32 inputs/output with internal
// QUANTIZE/DEQUANTIZE ops (int8 weights), so we feed/read floats directly -- no manual
// quantization. Values are in StandardScaler space; inference_run_daily un-scales.
#include "savia/inference.h"
#include "savia/lstm_input.h"   // LSTM_PAST_STEPS/FEATURES, LSTM_FUTURE_STEPS, LSTM_OUTPUT_STEPS
#include "savia/log.h"

#if SAVIA_ON_DEVICE_INFERENCE

#include "savia/lstm_hs30_int8_model.h"   // g_lstm_hs30_model[], g_lstm_hs30_model_len
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "pico/time.h"                    // time_us_32() for Invoke latency
#include <cstring>
#include <cmath>

namespace {

// Tensor arena in RP2350 SRAM, next to BTstack. On-board numbers (2026-07-04,
// per-tensor int8 model): steady-state arena_used = 162 KB, Invoke = 269 ms. Do
// NOT size from arena_used: AllocateTensors needs a large TRANSIENT peak for the
// greedy planner's scratch (~40 B x ~2k buffers of this unrolled graph) -- 200 KB
// fails ("temp memory"), 224 KB fails ("Too many buffers"); 350 KB is validated.
// The exact peak sits in (224, 350] -- not worth more flash cycles to bisect.
// NOTE: per-CHANNEL dense quantization does NOT fit at all (242 FCs x ~1 KB of
// Prepare multipliers) -- keep the converter's per-tensor flag.
constexpr int kArenaBytes = 350 * 1024;
alignas(16) uint8_t g_arena[kArenaBytes];

tflite::MicroInterpreter *g_interp = nullptr;
bool g_ready = false;
bool g_failed = false;   // stop retrying a known-bad setup on every call

bool ensure_ready() {
    if (g_ready) return true;
    if (g_failed) return false;

    const tflite::Model *model = tflite::GetModel(g_lstm_hs30_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        LOG_WARN("TFLM: model schema %lu != supported %d\n",
                 (unsigned long) model->version(), TFLITE_SCHEMA_VERSION);
        g_failed = true;
        return false;
    }

    // The unrolled int8 LSTM uses exactly these 16 builtin ops (from model introspection).
    static tflite::MicroMutableOpResolver<16> resolver;
    resolver.AddQuantize();       resolver.AddDequantize();
    resolver.AddFullyConnected(); resolver.AddAdd();
    resolver.AddMul();            resolver.AddSplit();
    resolver.AddLogistic();       resolver.AddTanh();
    resolver.AddConcatenation();  resolver.AddPack();
    resolver.AddUnpack();         resolver.AddFill();
    resolver.AddShape();          resolver.AddStridedSlice();
    resolver.AddTranspose();      resolver.AddElu();

    static tflite::MicroInterpreter interp(model, resolver, g_arena, kArenaBytes);
    if (interp.AllocateTensors() != kTfLiteOk) {
        // Could be a too-small arena OR a kernel that rejects the model at Prepare
        // (see the TFLM error just above -- e.g. a FULLY_CONNECTED with a float
        // output means the model isn't cleanly int8 for TFLM). Arena is %d KB.
        LOG_WARN("TFLM: AllocateTensors FAILED (arena %d KB; see the TFLM error above)\n",
                 kArenaBytes / 1024);
        g_failed = true;
        return false;
    }
    g_interp = &interp;
    g_ready = true;
    LOG_INFO("TFLM ready: arena_used=%u B (%u KB) of %u KB, inputs=%u outputs=%u\n",
             (unsigned) interp.arena_used_bytes(),
             (unsigned) (interp.arena_used_bytes() / 1024), (unsigned) (kArenaBytes / 1024),
             (unsigned) interp.inputs_size(), (unsigned) interp.outputs_size());
    return true;
}

}  // namespace

// Element count of a tensor from its byte size + dtype (int8 = 1 B, float = 4 B).
static size_t tensor_elems(const TfLiteTensor *t) {
    return (t->type == kTfLiteInt8) ? t->bytes : t->bytes / sizeof(float);
}

extern "C" int inference_run(const float *past48x3, const float *future24x1, float *out24) {
    if (!ensure_ready()) return -1;

    // Feed both inputs, mapped by element count (order-independent): 144 -> past
    // [t][TA,HS10,HS30], 24 -> future TA. Model I/O may be float32 (copy) or int8
    // (quantize with the tensor's scale+zero_point) -- handle both.
    for (size_t i = 0; i < g_interp->inputs_size(); i++) {
        TfLiteTensor *in = g_interp->input(i);
        size_t n = tensor_elems(in);
        const float *src = (n == LSTM_PAST_STEPS * LSTM_PAST_FEATURES) ? past48x3
                         : (n == LSTM_FUTURE_STEPS)                    ? future24x1
                                                                       : nullptr;
        if (!src) { LOG_WARN("TFLM: unexpected input %u elems=%u\n", (unsigned) i, (unsigned) n); return -3; }
        if (in->type == kTfLiteInt8) {
            const float s = in->params.scale;
            const int zp = in->params.zero_point;
            for (size_t k = 0; k < n; k++) {
                int q = (int) lroundf(src[k] / s) + zp;
                in->data.int8[k] = (int8_t) (q < -128 ? -128 : q > 127 ? 127 : q);
            }
        } else {
            std::memcpy(in->data.f, src, n * sizeof(float));
        }
    }

    uint32_t t0 = time_us_32();
    if (g_interp->Invoke() != kTfLiteOk) { LOG_WARN("TFLM: Invoke FAILED\n"); return -2; }
    uint32_t dt_us = time_us_32() - t0;

    TfLiteTensor *out = g_interp->output(0);
    if (tensor_elems(out) != LSTM_OUTPUT_STEPS) {
        LOG_WARN("TFLM: output has %u elems (want %d)\n", (unsigned) tensor_elems(out), LSTM_OUTPUT_STEPS);
        return -4;
    }
    if (out->type == kTfLiteInt8) {
        const float s = out->params.scale;
        const int zp = out->params.zero_point;
        for (size_t k = 0; k < LSTM_OUTPUT_STEPS; k++)
            out24[k] = ((int) out->data.int8[k] - zp) * s;
    } else {
        std::memcpy(out24, out->data.f, LSTM_OUTPUT_STEPS * sizeof(float));
    }

    LOG_INFO("TFLM Invoke OK: %u.%03u ms, arena_used=%u B\n",
             (unsigned) (dt_us / 1000), (unsigned) (dt_us % 1000),
             (unsigned) g_interp->arena_used_bytes());
    return 0;
}

#endif  // SAVIA_ON_DEVICE_INFERENCE
