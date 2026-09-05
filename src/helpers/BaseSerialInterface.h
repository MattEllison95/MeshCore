#pragma once

#include <Arduino.h>

#define MAX_FRAME_SIZE  176   // +4 for transport codes (region scoping)

class BaseSerialInterface {
protected:
  BaseSerialInterface() { }

public:
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual bool isEnabled() const = 0;

  virtual bool isConnected() const = 0;

  virtual bool isWriteBusy() const = 0;
  virtual size_t writeFrame(const uint8_t src[], size_t len) = 0;
  virtual size_t checkRecvFrame(uint8_t dest[]) = 0;

  /*
   * Send to every attached client EXCEPT whichever one issued the command
   * being handled. Used to tell the other clients about something this node
   * did on one client's behalf -- they have no other way to learn about it.
   *
   * A single-transport build has no other client, so doing nothing is right.
   */
  virtual size_t writeFrameToOthers(const uint8_t src[], size_t len) {
    (void)src; (void)len;
    return 0;
  }
};
