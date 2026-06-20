// Shared data types for the station firmware.
#ifndef SAVIA_TYPES_H
#define SAVIA_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    READING_SOIL_MOISTURE = 0,    // VWC 0..1
    READING_SOIL_TEMPERATURE = 1, // degrees C
} savia_reading_kind_t;

// A single sensor reading (mirrors savia_py's Reading).
typedef struct {
    uint64_t ts_ms;     // epoch ms UTC
    uint8_t  port;      // logical station port (1..6)
    uint8_t  depth_cm;  // sensor depth, e.g. 10 / 30
    uint8_t  kind;      // savia_reading_kind_t
    float    value;     // VWC 0..1 or degrees C
} savia_reading_t;

// One hourly aggregate (mirrors savia_py serialize_aggregations).
typedef struct {
    uint64_t hour_ms;   // hour start, epoch ms UTC
    uint8_t  port;
    uint8_t  kind;
    uint8_t  depth_cm;
    uint32_t count;
    float    mean;
    float    min;
    float    max;
} savia_aggregate_t;

// One model output row (mirrors savia_py serialize_predictions). On the Pico WH
// inference is off-device, so the prediction store stays empty here.
typedef struct {
    uint64_t ts_ms;
    char     model[16];     // e.g. "lstm-hs30"
    char     kind[20];      // e.g. "hs30_forecast"
    bool     has_port;
    uint8_t  port;
    float    value;
    bool     has_confidence;
    float    confidence;
} savia_prediction_t;

#endif // SAVIA_TYPES_H
