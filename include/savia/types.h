// Shared data types for the station firmware.
#ifndef SAVIA_TYPES_H
#define SAVIA_TYPES_H

#include <stdint.h>

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

#endif // SAVIA_TYPES_H
