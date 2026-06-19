// ML inference -- the ONE piece that differs between the two boards.
//
//   SAVIA_ON_DEVICE_INFERENCE=1 (Pico 2 W / RP2350): run the int8 LSTM via
//     TFLite Micro on-device.
//   SAVIA_ON_DEVICE_INFERENCE=0 (Pico WH / RP2040): no local inference; the
//     station just serves the forecast inputs and the app runs the model.
//
// The contract mirrors savia_py: past (48x3) + future (24x1) in StandardScaler
// space -> 24-hour HS30 forecast (still scaled; caller un-scales).
#ifndef SAVIA_INFERENCE_H
#define SAVIA_INFERENCE_H

#include <stdbool.h>

// True if this build runs the model on-device.
bool inference_on_device(void);

// Run the LSTM. `past48x3` has 48*3 floats, `future24x1` 24 floats, `out24`
// receives 24 floats. Returns 0 on success, <0 if unavailable (off-device build
// or model not loaded).
int inference_run(const float *past48x3, const float *future24x1, float *out24);

#endif // SAVIA_INFERENCE_H
