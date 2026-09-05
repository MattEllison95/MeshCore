#pragma once

#include <stddef.h>
#include <stdint.h>

// Incremental parser for the three-byte MeshCore serial envelope:
// marker, uint16 little-endian payload length, payload.
//
// The parser deliberately only treats a marker as framing while waiting for a
// header (or after a timed-out/invalid frame). Markers are legal payload bytes,
// so resynchronizing in the middle of a valid payload would corrupt messages.
template <size_t Capacity>
class SerialFrameParser {
public:
  enum Result {
    INVALID_FRAME = -1,
    NEED_MORE = 0,
    FRAME_READY = 1,
  };

  SerialFrameParser(uint8_t marker, uint32_t inter_byte_timeout_ms)
      : _marker(marker), _timeout_ms(inter_byte_timeout_ms) {
    reset();
  }

  void reset() {
    _state = WAIT_MARKER;
    _expected_length = 0;
    _used = 0;
    _frame_length = 0;
    _has_timestamp = false;
  }

  bool expire(uint32_t now_ms) {
    if (_state == WAIT_MARKER || !_has_timestamp || _timeout_ms == 0) {
      return false;
    }

    if ((uint32_t)(now_ms - _last_byte_at_ms) < _timeout_ms) {
      return false;
    }

    resetPartial();
    return true;
  }

  int feed(uint8_t byte, uint32_t now_ms) {
    bool expired = expire(now_ms);
    int result = expired ? INVALID_FRAME : NEED_MORE;

    switch (_state) {
    case WAIT_MARKER:
      if (byte == _marker) {
        _state = WAIT_LENGTH_LOW;
        _frame_length = 0;
        noteByte(now_ms);
      }
      return result;

    case WAIT_LENGTH_LOW:
      _length_low = byte;
      _state = WAIT_LENGTH_HIGH;
      noteByte(now_ms);
      return result;

    case WAIT_LENGTH_HIGH: {
      uint16_t length = (uint16_t)_length_low |
                        ((uint16_t)byte << 8);
      if (length == 0 || length > Capacity) {
        // If the byte that made this header invalid is itself a marker, retain
        // it as the start of the next header instead of discarding it.
        bool marker_starts_next_frame = (byte == _marker);
        resetPartial();
        if (marker_starts_next_frame) {
          _state = WAIT_LENGTH_LOW;
          noteByte(now_ms);
        }
        return INVALID_FRAME;
      }

      _expected_length = length;
      _used = 0;
      _state = WAIT_PAYLOAD;
      noteByte(now_ms);
      return result;
    }

    case WAIT_PAYLOAD:
      _buffer[_used++] = byte;
      noteByte(now_ms);
      if (_used == _expected_length) {
        _frame_length = _expected_length;
        resetPartial();
        return FRAME_READY;
      }
      return result;

    default:
      reset();
      return INVALID_FRAME;
    }
  }

  const uint8_t *frame() const { return _buffer; }
  size_t frameLength() const { return _frame_length; }

private:
  enum State {
    WAIT_MARKER,
    WAIT_LENGTH_LOW,
    WAIT_LENGTH_HIGH,
    WAIT_PAYLOAD,
  };

  void noteByte(uint32_t now_ms) {
    _last_byte_at_ms = now_ms;
    _has_timestamp = true;
  }

  void resetPartial() {
    _state = WAIT_MARKER;
    _expected_length = 0;
    _used = 0;
    _has_timestamp = false;
  }

  uint8_t _buffer[Capacity];
  uint8_t _marker;
  uint8_t _length_low;
  State _state;
  uint16_t _expected_length;
  uint16_t _used;
  uint16_t _frame_length;
  uint32_t _timeout_ms;
  uint32_t _last_byte_at_ms;
  bool _has_timestamp;
};
