// Builds the LSTM's two input tensors from station data and un-scales its output.
// Pure logic (storage + weather are SDK-free), so it host-tests. Contract:
// docs/plan_implementacion_modelo.md + the model's scaler_params.json.
//
//   past   : 48 h x 3 features, StandardScaler space, model order [TA, HS10, HS30]
//   future : 24 h x 1 feature (TA), StandardScaler space
//   output : 24 h HS30 forecast, still scaled -> lstm_unscale_output -> VWC 0..1
//
// Sources (see lstm_gather_inputs): HS10 / HS30 from the hourly soil-probe
// aggregates; TA (past + future) from the weather cache (Open-Meteo via LoRa/BLE),
// because the probe does not measure air temperature.
#ifndef SAVIA_LSTM_INPUT_H
#define SAVIA_LSTM_INPUT_H

#include <stdint.h>

#define LSTM_PAST_STEPS     48
#define LSTM_PAST_FEATURES  3
#define LSTM_FUTURE_STEPS   24
#define LSTM_OUTPUT_STEPS   24

// Feature layout of one past-tensor row (model input order, NOT scaler order).
enum { LSTM_FEAT_TA = 0, LSTM_FEAT_HS10 = 1, LSTM_FEAT_HS30 = 2 };

// Raw (real-unit) inputs, oldest -> newest, before scaling.
typedef struct {
    float ta[LSTM_PAST_STEPS];          // air temperature, degC (past 48 h)
    float hs10[LSTM_PAST_STEPS];        // soil moisture 10 cm, VWC 0..1
    float hs30[LSTM_PAST_STEPS];        // soil moisture 30 cm, VWC 0..1
    float future_ta[LSTM_FUTURE_STEPS]; // forecast air temperature, degC (next 24 h)
} lstm_raw_inputs_t;

typedef enum {
    LSTM_INPUT_OK                   = 0,
    LSTM_INPUT_INSUFFICIENT_HISTORY = -1,  // no HS10/HS30 history to build the window
    LSTM_INPUT_NO_FORECAST          = -2,  // TA past/future window not available
} lstm_input_status_t;

// Assemble the raw window ending at the hour containing `now_ms`. Interior gaps
// in a soil series are filled last-observation-carried-forward; a leading gap is
// back-filled from the first known sample. Returns LSTM_INPUT_OK on success.
lstm_input_status_t lstm_gather_inputs(uint64_t now_ms, lstm_raw_inputs_t *out);

// Scale + reorder the raw window into the model's two input tensors.
//   past_out   : LSTM_PAST_STEPS * LSTM_PAST_FEATURES floats, row-major [t][feat]
//   future_out : LSTM_FUTURE_STEPS floats
void lstm_build_tensors(const lstm_raw_inputs_t *in, float *past_out, float *future_out);

// Un-scale the model's 24 scaled HS30 outputs back to VWC 0..1.
void lstm_unscale_output(const float *scaled, float *hs30_real);

#endif // SAVIA_LSTM_INPUT_H
