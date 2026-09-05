#include "ArduinoSerialInterface.h"

#ifdef MACMESH_UART_DIAGNOSTIC
volatile uint32_t macmesh_uart_diag_rx_bytes = 0;
volatile uint32_t macmesh_uart_diag_tx_frames = 0;
#endif

void ArduinoSerialInterface::enable() { 
  _isEnabled = true;
  _parser.reset();
}
void ArduinoSerialInterface::disable() {
  _isEnabled = false;
  _parser.reset();
}

bool ArduinoSerialInterface::isConnected() const { 
  return true;   // no way of knowing, so assume yes
}

bool ArduinoSerialInterface::isWriteBusy() const {
  return false;
}

size_t ArduinoSerialInterface::writeFrame(const uint8_t src[], size_t len) {
  if (!_isEnabled || _serial == NULL || src == NULL ||
      len == 0 || len > MAX_FRAME_SIZE) {
    // frame is too big!
    return 0;
  }

  uint8_t hdr[3];
  hdr[0] = '>';
  hdr[1] = (len & 0xFF);  // LSB
  hdr[2] = (len >> 8);    // MSB

  if (_serial->write(hdr, 3) != 3) {
    return 0;
  }
#ifdef MACMESH_UART_DIAGNOSTIC
  macmesh_uart_diag_tx_frames++;
#endif
  return _serial->write(src, len);
}

size_t ArduinoSerialInterface::checkRecvFrame(uint8_t dest[]) {
  if (!_isEnabled || _serial == NULL || dest == NULL) {
    return 0;
  }

  _parser.expire(millis());
  while (_serial->available()) {
    int c = _serial->read();
    if (c < 0) break;

#ifdef MACMESH_UART_DIAGNOSTIC
    macmesh_uart_diag_rx_bytes++;
#endif

    int result = _parser.feed((uint8_t)c, millis());
    if (result == SerialFrameParser<MAX_FRAME_SIZE>::FRAME_READY) {
      size_t frame_len = _parser.frameLength();
      memcpy(dest, _parser.frame(), frame_len);
      return frame_len;
    }
  }
  return 0;
}
