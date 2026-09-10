#include <Arduino.h>

/*
 * Hand control back to the boot selector on the next boot.
 *
 * The selector lives in the factory partition and only regains control because
 * neither firmware marks its OTA slot valid: with rollback enabled, an app that
 * never calls esp_ota_mark_app_valid_cancel_rollback() is rolled back, landing
 * in factory again. Arduino-ESP32 asks the sketch whether to defer that through
 * this weak hook.
 *
 * Guarded, because a single-firmware build must NOT define it: there is no
 * factory partition to fall back to and deferring would send it somewhere
 * unbootable.
 */
#ifdef MC_LOADER_ROLLBACK
extern "C" bool verifyRollbackLater() { return true; }
#endif
