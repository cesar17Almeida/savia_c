#include "savia/inference.h"
#include "savia/lstm_input.h"
#include "savia/storage.h"
#include "savia/types.h"
#include "savia/log.h"
#include <string.h>

// This file is the ONE place the two boards diverge. The build flag
// SAVIA_ON_DEVICE_INFERENCE (set by CMake from the board choice) picks the path.

#define HOUR_MS 3600000ULL

#if SAVIA_ON_DEVICE_INFERENCE

// ===== Pico 2 W (RP2350): inference ON-device, int8 LSTM via TFLite Micro =====
// inference_run() (the TFLM MicroInterpreter) lives in inference_tflm.cpp -- TFLM is
// C++. This file keeps the board-agnostic C pipeline (gather -> build -> run ->
// unscale -> store). The embedded model has FLOAT32 I/O (internal QUANTIZE ops), so
// the shim feeds/reads floats directly -- no manual int8 quantization here.

bool inference_on_device(void) { return true; }

int inference_run_daily(uint64_t now_ms) {
    lstm_raw_inputs_t raw;
    lstm_input_status_t st = lstm_gather_inputs(now_ms, &raw);
    if (st != LSTM_INPUT_OK) {
        LOG_WARN("inference: inputs not ready (status=%d) -- skipping\n", (int) st);
        return (int) st;
    }

    // Static buffers: the tensors are ~600 floats -- keep them off the stack.
    static float past[LSTM_PAST_STEPS * LSTM_PAST_FEATURES];
    static float future[LSTM_FUTURE_STEPS];
    static float out_scaled[LSTM_OUTPUT_STEPS];
    lstm_build_tensors(&raw, past, future);

    int rc = inference_run(past, future, out_scaled);
    if (rc != 0) {
        LOG_WARN("inference: model unavailable (rc=%d)\n", rc);
        return rc;
    }

    float hs30[LSTM_OUTPUT_STEPS];
    lstm_unscale_output(out_scaled, hs30);

    // Persist the 24 h forecast (one row per hour, next 24 h) + log the min, which
    // is what the LoRa uplink / downstream irrigation logic cares about.
    uint64_t latest_hour = now_ms - (now_ms % HOUR_MS);
    float min = hs30[0];
    storage_clear_predictions();                       // fresh curve replaces yesterday's
    for (size_t t = 0; t < LSTM_OUTPUT_STEPS; t++) {
        if (hs30[t] < min) min = hs30[t];
        savia_prediction_t p;
        memset(&p, 0, sizeof(p));
        p.ts_ms = latest_hour + (uint64_t)(t + 1) * HOUR_MS;
        strcpy(p.model, "lstm-hs30");
        strcpy(p.kind, "hs30_forecast");
        p.value = hs30[t];
        storage_append_prediction(&p);
    }
    LOG_INFO("inference: HS30 24h forecast stored (min=%.3f)\n", (double) min);
    return 0;
}

#else

// ===== Pico WH (RP2040): inference OFF-device =====
// The model does not fit RP2040 RAM, so the station only serves the forecast
// inputs over BLE and the app runs the LSTM. These are no-ops by design.

bool inference_on_device(void) { return false; }

int inference_run(const float *past48x3, const float *future24x1, float *out24) {
    (void)past48x3; (void)future24x1; (void)out24;
    return -1;  // by design: inference happens in the app on this board
}

int inference_run_daily(uint64_t now_ms) {
    (void)now_ms;
    return -1;  // by design: the app runs the model from the served forecast
}

#endif
