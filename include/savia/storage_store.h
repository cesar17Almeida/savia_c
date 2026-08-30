// Persist the measurement state (readings ring + TA window) across resets, the
// way clock_store.c persists the sync ring. Kept out of storage_flash.c so the
// query/aggregation paths stay SDK-free and host-testable.
#ifndef SAVIA_STORAGE_STORE_H
#define SAVIA_STORAGE_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Restore the readings from the newest valid image. Call right after
 * storage_init(): readings carry absolute timestamps, so they need no clock.
 * Returns how many were restored.
 *
 * The TA window travels in the same record but is NOT adopted here -- judging it
 * needs a clock, which at this point in the boot the station usually does not
 * have yet. See storage_store_adopt_weather().
 */
size_t storage_store_load(void);

/**
 * Adopt the TA window from the image loaded above, once the clock is known.
 *
 * Two things can refuse it: a window too old to still describe the current hour
 * (see WEATHER_RESTORE_MAX_AGE_MS -- the window is positional, so a stale one
 * would silently feed the model temperatures for the wrong hours), and a cache
 * that a LoRa downlink has already filled during bring-up, which is by
 * definition fresher than anything in flash.
 *
 * Returns true if the stored window was adopted.
 */
bool storage_store_adopt_weather(uint64_t now_ms);

/** Write an image of the current state to the spare bank. Cheap to call when
 *  nothing changed -- see storage_take_dirty(). */
void storage_store_save(void);

#endif // SAVIA_STORAGE_STORE_H
