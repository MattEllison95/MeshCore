#pragma once

#include "BaseSerialInterface.h"


/*
 * Fans one companion protocol out across several transports at once -- for
 * example the header UART and BLE together, so a Macintosh on the wire and a
 * phone or web app can both reach the node.
 *
 * Upstream MyMesh holds a single BaseSerialInterface*, so a build picks exactly
 * one of USB, BLE, WiFi or UART. This is itself a BaseSerialInterface, so it
 * drops into that same pointer and none of MyMesh's ~70 call sites change.
 *
 * Fanning out at the interface layer rather than at the byte-stream layer
 * matters: each child keeps its own framing state, so two clients talking at
 * the same time cannot interleave into one corrupted frame.
 *
 * Replies are broadcast to every connected child. A client therefore sees
 * responses to commands it did not send, which is harmless -- the companion
 * protocol is already a push protocol, and clients ignore what they did not ask
 * for.
 */
#define MAX_SERIAL_CHILDREN 3

class MultiSerialInterface : public BaseSerialInterface {
  BaseSerialInterface *_children[MAX_SERIAL_CHILDREN];
  bool _pairable[MAX_SERIAL_CHILDREN];
  int _count;
  int _next;            /* round-robin, so one busy child cannot starve another */
  int _last_src;        /* child that supplied the frame being handled, -1 = none */

public:
  MultiSerialInterface() : _count(0), _next(0), _last_src(-1) { }

  /*
   * pairable marks a transport a user joins deliberately -- BLE, which needs a
   * PIN. A UART is not pairable: it is connected because the cable exists.
   */
  bool add(BaseSerialInterface *child, bool pairable = false) {
    if (child == NULL || _count >= MAX_SERIAL_CHILDREN) {
      return false;
    }
    _pairable[_count] = pairable;
    _children[_count++] = child;
    return true;
  }

  void enable() override {
    for (int i = 0; i < _count; i++) {
      _children[i]->enable();
    }
  }

  void disable() override {
    for (int i = 0; i < _count; i++) {
      _children[i]->disable();
    }
  }

  bool isEnabled() const override {
    for (int i = 0; i < _count; i++) {
      if (_children[i]->isEnabled()) return true;
    }
    return false;
  }

  /* Connected if ANY transport has a client. MyMesh gates unsolicited pushes
     on this, so it must not go false just because BLE is unpaired. */
  bool isPairedConnection() const override {
    for (int i = 0; i < _count; i++) {
      if (_pairable[i] && _children[i]->isConnected()) return true;
    }
    return false;
  }

  bool isConnected() const override {
    for (int i = 0; i < _count; i++) {
      if (_children[i]->isConnected()) return true;
    }
    return false;
  }

  /* Busy only if every connected child is busy; otherwise progress is possible. */
  bool isWriteBusy() const override {
    bool any_connected = false;
    for (int i = 0; i < _count; i++) {
      if (_children[i]->isConnected()) {
        any_connected = true;
        if (!_children[i]->isWriteBusy()) return false;
      }
    }
    return any_connected;
  }

  size_t writeFrame(const uint8_t src[], size_t len) override {
    size_t written = 0;
    for (int i = 0; i < _count; i++) {
      /* Only write to children with a client attached. Writing into an
         unattached transport can block or silently fill a queue. */
      if (_children[i]->isConnected()) {
        size_t n = _children[i]->writeFrame(src, len);
        if (n > written) written = n;
      }
    }
    return written;
  }

  /*
   * Everyone except whoever sent the command being handled. Upstream never
   * echoes a sent message back to a client -- it assumes the one client that
   * sent it will display it itself. With two clients on one node that leaves
   * the second one blind to anything the first did, so the node has to tell
   * it. The sender is excluded or it would show its own message twice.
   */
  size_t writeFrameToOthers(const uint8_t src[], size_t len) override {
    size_t written = 0;
    for (int i = 0; i < _count; i++) {
      if (i == _last_src) continue;
      if (_children[i]->isConnected()) {
        size_t n = _children[i]->writeFrame(src, len);
        if (n > written) written = n;
      }
    }
    return written;
  }

  /* Return the first frame found, resuming after the child that supplied the
     last one so a chatty transport cannot monopolise the poll. */
  size_t checkRecvFrame(uint8_t dest[]) override {
    for (int i = 0; i < _count; i++) {
      int idx = (_next + i) % _count;
      size_t len = _children[idx]->checkRecvFrame(dest);
      if (len > 0) {
        _next = (idx + 1) % _count;
        _last_src = idx;      /* whose command we are about to handle */
        return len;
      }
    }
    return 0;
  }
};
