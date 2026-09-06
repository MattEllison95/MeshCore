#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>
#include "MyMesh.h"

#ifdef MACMESH_UART_DIAGNOSTIC
extern volatile uint32_t macmesh_uart_diag_rx_bytes;
extern volatile uint32_t macmesh_uart_diag_tx_frames;
#endif

// Believe it or not, this std C function is busted on some platforms!
static uint32_t _atoi(const char* sp) {
  uint32_t n = 0;
  while (*sp && *sp >= '0' && *sp <= '9') {
    n *= 10;
    n += (*sp++ - '0');
  }
  return n;
}

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  #include <InternalFileSystem.h>
  #if defined(QSPIFLASH)
    #include <CustomLFS_QSPIFlash.h>
    DataStore store(InternalFS, QSPIFlash, rtc_clock);
  #else
  #if defined(EXTRAFS)
    #include <CustomLFS.h>
    CustomLFS ExtraFS(0xD4000, 0x19000, 128);
    DataStore store(InternalFS, ExtraFS, rtc_clock);
  #else
    DataStore store(InternalFS, rtc_clock);
  #endif
  #endif
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
  DataStore store(LittleFS, rtc_clock);
#elif defined(ESP32)
  #include <SPIFFS.h>
  DataStore store(SPIFFS, rtc_clock);
#endif

#ifdef ESP32
  #ifdef WIFI_SSID
    #include <helpers/esp32/SerialWifiInterface.h>
    SerialWifiInterface serial_interface;
    #ifndef TCP_PORT
      #define TCP_PORT 5000
    #endif
  #elif defined(BLE_PIN_CODE) && defined(SERIAL_RX)
    /* Both transports at once: the Macintosh on the header UART and a phone or
       web app over BLE. MyMesh holds one interface pointer, so a fan-out
       stands in for it and forwards to both. */
    #include <helpers/esp32/SerialBLEInterface.h>
    #include <helpers/ArduinoSerialInterface.h>
    #include <helpers/MultiSerialInterface.h>
    SerialBLEInterface ble_interface;
    ArduinoSerialInterface uart_interface;
    MultiSerialInterface serial_interface;
    HardwareSerial companion_serial(1);
    #define COMPANION_UART_AND_BLE 1
  #elif defined(BLE_PIN_CODE)
    #include <helpers/esp32/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #elif defined(SERIAL_RX)
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
    HardwareSerial companion_serial(1);
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(RP2040_PLATFORM)
  //#ifdef WIFI_SSID
  //  #include <helpers/rp2040/SerialWifiInterface.h>
  //  SerialWifiInterface serial_interface;
  //  #ifndef TCP_PORT
  //    #define TCP_PORT 5000
  //  #endif
  // #elif defined(BLE_PIN_CODE)
  //   #include <helpers/rp2040/SerialBLEInterface.h>
  //   SerialBLEInterface serial_interface;
  #if defined(SERIAL_RX)
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
    HardwareSerial companion_serial(1);
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(NRF52_PLATFORM)
  #ifdef BLE_PIN_CODE
    #include <helpers/nrf52/SerialBLEInterface.h>
    SerialBLEInterface serial_interface;
  #else
    #include <helpers/ArduinoSerialInterface.h>
    ArduinoSerialInterface serial_interface;
  #endif
#elif defined(STM32_PLATFORM)
  #include <helpers/ArduinoSerialInterface.h>
  ArduinoSerialInterface serial_interface;
#else
  #error "need to define a serial interface"
#endif

/* GLOBAL OBJECTS */
#ifdef DISPLAY_CLASS
  #include "UITask.h"
  UITask ui_task(&board, &serial_interface);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables, store
   #ifdef DISPLAY_CLASS
      , &ui_task
   #endif
);

/* END GLOBAL OBJECTS */

void halt() {
  while (1) ;
}

/* WIFI RECONNECT TRACKERS */
#if defined(ESP32) && defined(WIFI_SSID)
  bool wifi_needs_reconnect = false;
  unsigned long last_wifi_reconnect_attempt = 0;
#endif

/*
 * WIFI PURELY AS A CLOCK SOURCE (MacMesh)
 *
 * MeshCore has no NTP, so a node with no GPS and no phone ever attached has no
 * source of real time. It cold-boots to 2024, gets clamped to the firmware
 * build date, and is then raised only by adverts from neighbours running the
 * same bootstrap -- a consensus that lags real time by days and corroborates
 * itself, so it never converges.
 *
 * This brings WiFi up for SNTP and nothing else. It deliberately does NOT use
 * WIFI_SSID: that macro is the companion-TRANSPORT selector, and defining it
 * would move the companion protocol to TCP and take the serial port away from
 * the Macintosh. The serial link is the whole point of this carrier, so the
 * feature gets its own flag and leaves the transport chain untouched.
 *
 * Credentials live in NodePrefs, set over the companion protocol and persisted,
 * so a node can be pointed at a network without a reflash. No SSID means the
 * radio is never brought up.
 */
#if defined(ESP32) && defined(MACMESH_WIFI_TIME)
  #include <WiFi.h>
  #include <time.h>
  #include <esp_sntp.h>
  #ifndef MACMESH_NTP_SERVER
    #define MACMESH_NTP_SERVER "pool.ntp.org"
  #endif

  /*
   * Do NOT infer "SNTP has answered" from time(). ESP32RTCClock implements
   * MeshCore's clock with settimeofday()/time(), so time() returns the node's
   * OWN mesh-bootstrapped clock -- not an unset epoch. Reading it back and
   * feeding it to applyExternalTime() sets the clock to the value it already
   * had, reports success, and marks it authoritative, freezing the wrong time
   * and locking the mesh out of ever correcting it. That is a bug this file
   * shipped with once; the sync callback is the only honest signal.
   */
  static volatile bool macmesh_ntp_have = false;
  static volatile uint32_t macmesh_ntp_secs = 0;
  static bool macmesh_ntp_started = false;
  static unsigned long macmesh_ntp_next_poll = 0;

  static void macmeshSntpSynced(struct timeval *tv) {
    // SNTP task context: record only, and apply from loop().
    macmesh_ntp_secs = (uint32_t)tv->tv_sec;
    macmesh_ntp_have = true;
#ifdef MACMESH_WIFI_TIME_DEBUG
    Serial.printf("[macmesh] SNTP SYNCED %lu\n", (unsigned long)tv->tv_sec);
#endif
  }

  static void macmeshStartWifiTime() {
    const NodePrefs* prefs = the_mesh.getNodePrefs();
#ifdef MACMESH_WIFI_TIME_DEBUG
    // The ssid only. Never print the psk.
    Serial.printf("[macmesh] ssid=\"%s\" len=%u psk_len=%u\n",
                  prefs->wifi_ssid, (unsigned)strlen(prefs->wifi_ssid),
                  (unsigned)strlen(prefs->wifi_psk));
#endif
    if (prefs->wifi_ssid[0] == 0) return;   // never configured

    board.setInhibitSleep(true);   // a sleeping node never finishes the handshake
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(prefs->wifi_ssid, prefs->wifi_psk);
  }

  static void macmeshPollWifiTime() {
    if (millis() < macmesh_ntp_next_poll) return;
    macmesh_ntp_next_poll = millis() + 2000;
#ifdef MACMESH_WIFI_TIME_DEBUG
    {
      static unsigned long last_report = 0;
      if (millis() - last_report > 5000) {
        last_report = millis();
        Serial.printf("[macmesh] wifi=%d ip=%s sntp_started=%d sync=%d clock=%lu\n",
                      (int)WiFi.status(), WiFi.localIP().toString().c_str(),
                      (int)macmesh_ntp_started, (int)sntp_get_sync_status(),
                      (unsigned long)time(NULL));
      }
    }
#endif

    if (macmesh_ntp_have) {
      macmesh_ntp_have = false;
      /* Applied on every sync, not just the first: SNTP re-checks periodically,
         and a node left running should follow it rather than drift. */
      the_mesh.applyExternalTime(macmesh_ntp_secs);
      return;
    }

    if (WiFi.status() != WL_CONNECTED) return;
    if (!macmesh_ntp_started) {
      /* Started only once there is an address -- the server name has to
         resolve, and this also covers a node that boots out of range and
         associates later. */
      sntp_set_time_sync_notification_cb(macmeshSntpSynced);
      // UTC with no DST rules: MeshCore timestamps are epoch seconds throughout.
      configTime(0, 0, MACMESH_NTP_SERVER);
      macmesh_ntp_started = true;
    }
  }
#endif

#ifndef SERIAL_BAUD
  #define SERIAL_BAUD 115200
#endif

#if defined(MACMESH_SERIAL_TEST_RESPONDER) && defined(SERIAL_RX)
static const uint8_t MACMESH_SERIAL_TEST_REQUEST[] = "MMTEST1?\r\n";
static const uint8_t MACMESH_SERIAL_TEST_REPLY[] = "MMTEST1!\r\n";
static size_t macmesh_serial_test_matched = 0;

static void macmesh_run_serial_test_responder() {
  while (companion_serial.available()) {
    int value = companion_serial.read();
    if (value < 0) {
      break;
    }

    uint8_t byte = (uint8_t)value;
    if (byte == MACMESH_SERIAL_TEST_REQUEST[macmesh_serial_test_matched]) {
      macmesh_serial_test_matched++;
    } else {
      // Preserve the one-byte prefix overlap in "...M" followed by the next
      // request. No arbitrary traffic is echoed back to the Macintosh.
      macmesh_serial_test_matched =
          byte == MACMESH_SERIAL_TEST_REQUEST[0] ? 1 : 0;
    }

    if (macmesh_serial_test_matched ==
        sizeof(MACMESH_SERIAL_TEST_REQUEST) - 1) {
      companion_serial.write(MACMESH_SERIAL_TEST_REPLY,
                             sizeof(MACMESH_SERIAL_TEST_REPLY) - 1);
      companion_serial.flush();
      macmesh_serial_test_matched = 0;
    }
  }
}
#endif

void setup() {
  Serial.begin(115200);


  board.begin();

#ifdef DISPLAY_CLASS
  DisplayDriver* disp = NULL;
  if (display.begin()) {
    disp = &display;
    disp->startFrame();
  #ifdef ST7789
    disp->setTextSize(2);
  #endif
    disp->drawTextCentered(disp->width() / 2, 28, "Loading...");
    disp->endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  #if defined(QSPIFLASH)
    if (!QSPIFlash.begin()) {
      // debug output might not be available at this point, might be too early. maybe should fall back to InternalFS here?
      MESH_DEBUG_PRINTLN("CustomLFS_QSPIFlash: failed to initialize");
    } else {
      MESH_DEBUG_PRINTLN("CustomLFS_QSPIFlash: initialized successfully");
    }
  #else
  #if defined(EXTRAFS)
      ExtraFS.begin();
  #endif
  #endif
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

#ifdef BLE_PIN_CODE
  serial_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name, the_mesh.getBLEPin());
#else
  serial_interface.begin(Serial);
#endif
  the_mesh.startInterface(serial_interface);
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

  //#ifdef WIFI_SSID
  //  WiFi.begin(WIFI_SSID, WIFI_PWD);
  //  serial_interface.begin(TCP_PORT);
  // #elif defined(BLE_PIN_CODE)
  //   char dev_name[32+16];
  //   sprintf(dev_name, "%s%s", BLE_NAME_PREFIX, the_mesh.getNodeName());
  //   serial_interface.begin(dev_name, the_mesh.getBLEPin());
  #if defined(COMPANION_UART_AND_BLE)
    companion_serial.setPins(SERIAL_RX, SERIAL_TX);
    companion_serial.begin(SERIAL_BAUD);
    uart_interface.begin(companion_serial);
    ble_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name,
                        the_mesh.getBLEPin());
    serial_interface.add(&uart_interface);
    serial_interface.add(&ble_interface);
  #elif defined(SERIAL_RX)
    companion_serial.setPins(SERIAL_RX, SERIAL_TX);
    companion_serial.begin(SERIAL_BAUD);
    serial_interface.begin(companion_serial);
  #else
    serial_interface.begin(Serial);
  #endif
    the_mesh.startInterface(serial_interface);
#elif defined(ESP32)
  SPIFFS.begin(true);
  store.begin();
  the_mesh.begin(
    #ifdef DISPLAY_CLASS
        disp != NULL
    #else
        false
    #endif
  );

#ifdef WIFI_SSID
  board.setInhibitSleep(true);   // prevent sleep when WiFi is active
  WiFi.setAutoReconnect(true);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
      if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
          WIFI_DEBUG_PRINTLN("WiFi disconnected. Flagging for reconnect...");
          wifi_needs_reconnect = true;
      } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
          WIFI_DEBUG_PRINTLN("WiFi connected successfully!");
          wifi_needs_reconnect = false;
      }
  });

  WiFi.begin(WIFI_SSID, WIFI_PWD);
  serial_interface.begin(TCP_PORT);
#elif defined(COMPANION_UART_AND_BLE)
  companion_serial.setPins(SERIAL_RX, SERIAL_TX);
  companion_serial.begin(SERIAL_BAUD);
  uart_interface.begin(companion_serial);
  ble_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name,
                      the_mesh.getBLEPin());
  serial_interface.add(&uart_interface);
  serial_interface.add(&ble_interface);
#elif defined(BLE_PIN_CODE)
  serial_interface.begin(BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name, the_mesh.getBLEPin());
#elif defined(SERIAL_RX)
  companion_serial.setPins(SERIAL_RX, SERIAL_TX);
  companion_serial.begin(SERIAL_BAUD);
#ifndef MACMESH_SERIAL_TEST_RESPONDER
  serial_interface.begin(companion_serial);
#endif
#else
  serial_interface.begin(Serial);
#endif
#ifndef MACMESH_SERIAL_TEST_RESPONDER
  the_mesh.startInterface(serial_interface);
#endif
#if defined(MACMESH_WIFI_TIME)
  macmeshStartWifiTime();
#endif
#else
  #error "need to define filesystem"
#endif

  sensors.begin();

#if ENV_INCLUDE_GPS == 1
  the_mesh.applyGpsPrefs();
#endif

#ifdef DISPLAY_CLASS
  ui_task.begin(disp, &sensors, the_mesh.getNodePrefs());  // still want to pass this in as dependency, as prefs might be moved
#endif

  board.onBootComplete();
}

void loop() {
#if defined(MACMESH_SERIAL_TEST_RESPONDER) && defined(SERIAL_RX)
  macmesh_run_serial_test_responder();
#endif
  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#ifdef MACMESH_UART_DIAGNOSTIC
  static uint32_t next_diag_frame = 0;
  if (millis() >= next_diag_frame) {
    char rx_text[24];
    char tx_text[24];
    next_diag_frame = millis() + 250;
    snprintf(rx_text, sizeof(rx_text), "Mac RX: %lu", (unsigned long)macmesh_uart_diag_rx_bytes);
    snprintf(tx_text, sizeof(tx_text), "Mac TX: %lu", (unsigned long)macmesh_uart_diag_tx_frames);
    display.startFrame();
    display.drawTextCentered(display.width() / 2, 18, "MacMesh UART test");
    display.drawTextCentered(display.width() / 2, 36, rx_text);
    display.drawTextCentered(display.width() / 2, 50, tx_text);
    display.endFrame();
  }
#endif
#endif
  rtc_clock.tick();

  if (!the_mesh.hasPendingWork()) {
#if defined(NRF52_PLATFORM)
    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
#endif
  }

#if defined(ESP32) && defined(WIFI_SSID)
  // Safely attempt to reconnect every 10 seconds if flagged
  if (wifi_needs_reconnect && (millis() - last_wifi_reconnect_attempt > 10000)) {
    WIFI_DEBUG_PRINTLN("Attempting manual WiFi reconnect...");
    WiFi.disconnect();
    WiFi.reconnect();
    last_wifi_reconnect_attempt = millis();
  }
#endif

#if defined(ESP32) && defined(MACMESH_WIFI_TIME)
  macmeshPollWifiTime();
#endif
}
