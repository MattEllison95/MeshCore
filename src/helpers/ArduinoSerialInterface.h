#pragma once

#include "BaseSerialInterface.h"
#include "SerialFrameParser.h"
#include <Arduino.h>

#ifndef ARDUINO_SERIAL_FRAME_TIMEOUT_MS
#define ARDUINO_SERIAL_FRAME_TIMEOUT_MS 1000
#endif

class ArduinoSerialInterface : public BaseSerialInterface {
  bool _isEnabled;
  Stream* _serial;
  SerialFrameParser<MAX_FRAME_SIZE> _parser;

public:
  ArduinoSerialInterface()
      : _isEnabled(false),
        _serial(NULL),
        _parser('<', ARDUINO_SERIAL_FRAME_TIMEOUT_MS) {}

  void begin(Stream& serial) { 
    _serial = &serial; 
  #ifdef RAK_4631
    pinMode(WB_IO2, OUTPUT);
  #endif  
  }

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  bool isConnected() const override;

  bool isWriteBusy() const override;
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;
};
