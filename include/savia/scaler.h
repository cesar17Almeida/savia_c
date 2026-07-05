// StandardScaler for the LSTM HS30 model. Constants are baked from
// docs/sensor_documentation/model/scaler/scaler_params.json (StandardScaler,
// sklearn 1.6.1). Forward: (x - mean) / std. Inverse: x * std + mean.
//
// Index order MATCHES scaler_params.json "feature_order_scaler":
//   0 = HS30_mean, 1 = TA_mean, 2 = HS10_mean.
// NOTE: this is the SCALER order, NOT the model's past-tensor order
// ([TA, HS10, HS30]); lstm_input.c reorders after scaling. Pure logic, host-tested.
#ifndef SAVIA_SCALER_H
#define SAVIA_SCALER_H

typedef enum {
    SCALER_HS30 = 0,
    SCALER_TA   = 1,
    SCALER_HS10 = 2,
    SCALER_N    = 3,
} savia_scaler_feature_t;

// Forward-scale one value of the given feature: (x - mean) / std.
float scaler_transform(float x, savia_scaler_feature_t f);

// Inverse-scale an HS30 value (the model target) back to VWC 0..1.
float scaler_inverse_hs30(float x_scaled);

#endif // SAVIA_SCALER_H
