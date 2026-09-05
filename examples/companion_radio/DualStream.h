#pragma once

#include <Arduino.h>

/*
 * Presents USB CDC and a hardware UART as one Stream, so the companion
 * protocol is reachable from both: the Macintosh on the header UART, and the
 * MeshCore web app over USB for configuration such as setting the clock.
 * Upstream selects one transport at build time with #if defined(SERIAL_RX).
 *
 * Reads never consult the CDC's "connected" flag. That flag tracks the DTR
 * line state, and a host can be reading and writing without it being set --
 * gating reads on it makes USB look dead even while bytes are arriving.
 *
 * Writes are protected differently: ESP32 USB CDC blocks when its buffer fills
 * and nothing is draining it, which would wreck the UART's timing for the Mac.
 * setTxTimeoutMs(0) at startup makes those writes return immediately instead,
 * so an unattached USB port costs nothing.
 *
 * Two clients issuing commands at once would interleave frames, so use one at
 * a time. Replies go to both, which is harmless.
 */
class DualStream : public Stream {
    Stream *_uart;
    USBCDC *_usb;

public:
    DualStream(Stream *uart, USBCDC *usb) : _uart(uart), _usb(usb) { }

    int available() override {
        int n = _uart ? _uart->available() : 0;
        if (_usb) {
            n += _usb->available();
        }
        return n;
    }

    int read() override {
        if (_uart && _uart->available()) {
            return _uart->read();
        }
        if (_usb && _usb->available()) {
            return _usb->read();
        }
        return -1;
    }

    int peek() override {
        if (_uart && _uart->available()) {
            return _uart->peek();
        }
        if (_usb && _usb->available()) {
            return _usb->peek();
        }
        return -1;
    }

    void flush() override {
        if (_uart) {
            _uart->flush();
        }
    }

    size_t write(uint8_t b) override {
        size_t n = _uart ? _uart->write(b) : 1;
        if (_usb) {
            _usb->write(b);
        }
        return n;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        size_t n = _uart ? _uart->write(buffer, size) : size;
        if (_usb) {
            _usb->write(buffer, size);
        }
        return n;
    }
};
