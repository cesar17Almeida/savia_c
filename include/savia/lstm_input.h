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

// Real hourly buckets a soil series needs INSIDE the 48 h window before the gaps
// around them are worth filling. The LOCF/back-fill below is meant to bridge a
// missed hour, not to manufacture two days of history out of one sample: under
// this floor the window would be mostly fabricated, so gathering fails instead.
#define LSTM_MIN_PAST_HOURS 24

// How old the newest REAL bucket of a soil series may be before the window stops
// being trustworthy, in hours. 0 = the hour being inferred must contain a real
// reading; nothing in the newest step may be a LOCF copy.
//
// Zero is deliberate. The copy-forward fill cannot represent a discrete event: if
// it rained or the field was irrigated inside the copied span, the window still
// says the soil is where it was, and the forecast starts from soil that no longer
// exists. At one hour of tolerance that is already a plausible miss, so we take
// none. What makes zero workable is that the supervisor SAMPLES BEFORE IT INFERS
// (see scheduler_tick: a daily tick marks every input slot due, and the on-demand
// path captures first) -- without that, the newest bucket would usually belong to
// the previous hour and inference would never run.
//
// To grant tolerance later: raise this to N and the newest real bucket may be up
// to N hours old, the missing steps being copies of it. Weigh it as "how long a
// blind spot am I willing to forecast across" -- an irrigation cycle or a shower
// fits in one hour. Nothing else needs to change; the guard is one comparison in
// lstm_gather_inputs. Note that a capture_interval_s above N hours puts every
// window past the bound and stops LOCAL inference, which is intended: the model
// was trained on hourly series.
#define LSTM_MAX_STALE_HOURS 0
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
    LSTM_INPUT_STALE_HISTORY        = -3,  // soil history is there but stops too far back
} lstm_input_status_t;

// Human-readable reason, for the logs the app reads over BLE ("status=-3" tells
// an installer nothing). Never NULL.
const char *lstm_input_status_str(lstm_input_status_t st);

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
