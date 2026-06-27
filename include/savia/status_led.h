#ifndef SAVIA_STATUS_LED_H
#define SAVIA_STATUS_LED_H

// Onboard-LED status indicator (Pico W: the LED hangs off the CYW43 chip):
//   solid           = a central is connected (paired)
//   blink every 1 s = advertising (Bluetooth on, discoverable)
//   blink every 3 s = operating with the radio idle (Bluetooth off)
// No-op when BLE is compiled out (no CYW43 radio -> no onboard LED).
void status_led_init(void);

#endif // SAVIA_STATUS_LED_H
