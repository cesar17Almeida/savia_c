// Metadata half of the sensor catalog: the rows without their measure functions,
// so this compiles natively for the host tests (the driver needs the Pico SDK).
#include "savia/sensor_catalog.h"
#include <string.h>

typedef struct {
    savia_sensor_type_t type;
    const char *token;
    uint8_t caps;
    uint8_t pins;
    savia_slot_role_t role;
    uint8_t extra;
} catalog_row_t;

#define X(type, token, caps, pins, role, extra, measure) { type, token, caps, pins, role, extra },
static const catalog_row_t ROWS[] = { SAVIA_SENSOR_CATALOG(X) };
#undef X

#define ROW_COUNT (sizeof(ROWS) / sizeof(ROWS[0]))

// A type added to the enum without its catalog row would silently lose its pin
// rules and its wire token; fail the build instead. (-1: SENSOR_NONE has no row.)
_Static_assert(ROW_COUNT == SAVIA_SENSOR_TYPE_COUNT - 1,
               "every savia_sensor_type_t needs exactly one SAVIA_SENSOR_CATALOG row");

static const catalog_row_t *row_of(savia_sensor_type_t type) {
    for (size_t i = 0; i < ROW_COUNT; i++)
        if (ROWS[i].type == type) return &ROWS[i];
    return NULL;
}

const char *sensor_type_token(savia_sensor_type_t type) {
    const catalog_row_t *r = row_of(type);
    return r ? r->token : "none";
}

savia_sensor_type_t sensor_type_from_token(const char *s, size_t n) {
    for (size_t i = 0; i < ROW_COUNT; i++) {
        const char *t = ROWS[i].token;
        if (strlen(t) == n && memcmp(s, t, n) == 0) return ROWS[i].type;
    }
    return SENSOR_NONE;
}

uint8_t sensor_type_caps(savia_sensor_type_t type) {
    const catalog_row_t *r = row_of(type);
    return r ? r->caps : 0;
}

uint8_t sensor_type_pins(savia_sensor_type_t type) {
    const catalog_row_t *r = row_of(type);
    return r ? r->pins : 1;
}

uint8_t sensor_type_extra(savia_sensor_type_t type) {
    const catalog_row_t *r = row_of(type);
    return r ? r->extra : 0;
}

bool sensor_type_is_output(savia_sensor_type_t type) {
    const catalog_row_t *r = row_of(type);
    return r && r->role == SAVIA_SLOT_OUTPUT;
}
