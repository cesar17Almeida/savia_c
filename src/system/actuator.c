#include "savia/actuator.h"
#include "savia/config.h"

static uint8_t s_on_mask;   // bit (port-1) set = ON; boot value 0 = all OFF

void actuator_set(uint8_t port, bool on) {
    if (port < 1 || port > SAVIA_MAX_SENSORS) return;
    uint8_t bit = (uint8_t)(1u << (port - 1));
    if (on) s_on_mask |= bit; else s_on_mask &= (uint8_t) ~bit;
}

bool actuator_is_on(uint8_t port) {
    if (port < 1 || port > SAVIA_MAX_SENSORS) return false;
    return (s_on_mask >> (port - 1)) & 1u;
}
