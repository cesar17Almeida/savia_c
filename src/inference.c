#include "savia/inference.h"

// This file is the ONE place the two boards diverge. The build flag
// SAVIA_ON_DEVICE_INFERENCE (set by CMake from the board choice) picks the path.

#if SAVIA_ON_DEVICE_INFERENCE

// ===== Pico 2 W (RP2350): inference ON-device, int8 LSTM via TFLite Micro =====
// Needs the RP2350's RAM (~200 KB tensor arena, measured) and FPU. Requires the
// pico-tflmicro library (see docs/BUILD.md) and the int8 model embedded as a C
// array (xxd -i lstm_hs30_int8.tflite). TODO(hw): wire MicroInterpreter.

bool inference_on_device(void) { return true; }

int inference_run(const float *past48x3, const float *future24x1, float *out24) {
    (void)past48x3; (void)future24x1; (void)out24;
    // TODO(hw): set the two input tensors, Invoke(), read the 24 outputs.
    return -1;  // not wired yet
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

#endif
