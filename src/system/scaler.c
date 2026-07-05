#include "savia/scaler.h"

// From scaler_params.json (StandardScaler, n_features=3, sklearn 1.6.1).
// Index order = feature_order_scaler: [HS30_mean, TA_mean, HS10_mean].
static const float SCALER_MEAN[SCALER_N] = {
    0.7712527688624472f,   // HS30_mean (VWC 0..1)
    24.38746466771412f,    // TA_mean   (degC)
    0.7902161245631403f,   // HS10_mean (VWC 0..1)
};
static const float SCALER_STD[SCALER_N] = {
    0.042382551037717174f, // HS30_mean
    5.069760003904925f,    // TA_mean
    0.04550010247318015f,  // HS10_mean
};

float scaler_transform(float x, savia_scaler_feature_t f) {
    return (x - SCALER_MEAN[f]) / SCALER_STD[f];
}

float scaler_inverse_hs30(float x_scaled) {
    return x_scaled * SCALER_STD[SCALER_HS30] + SCALER_MEAN[SCALER_HS30];
}
