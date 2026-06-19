#include "savia/storage.h"
// #include "hardware/flash.h"
// #include "hardware/sync.h"

// Circular log of fixed-size records in on-chip flash (replaces SQLite). The
// data is tiny -- hourly aggregates over >=48 h -- so a wear-levelled ring at
// the top of flash is plenty. The BLE handlers query this for raw/agg/pred.

void storage_init(void) {
    // TODO(hw): locate the ring region (reserved at link time), validate the
    // head/tail markers, or format if empty. Consider LittleFS for wear level.
}

bool storage_append_reading(const savia_reading_t *r) {
    (void)r;
    // TODO(hw): write one record at the ring head; erase-ahead a sector when
    // crossing a flash page boundary (flash_range_erase/_program under a lock,
    // both cores parked).
    return true;
}

int storage_query_raw(uint64_t from_ms, uint64_t to_ms,
                      savia_reading_t *out, int max) {
    (void)from_ms; (void)to_ms; (void)out; (void)max;
    // TODO(hw): scan the ring tail->head, emit records with ts in [from, to).
    return 0;
}
