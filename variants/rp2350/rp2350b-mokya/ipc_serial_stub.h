// Phase 2 M1.0 spike stub — IpcSerialStream class declaration.
//
// The class body has to live in a header so every Meshtastic TU that touches
// `Serial` can see the full interface (SerialConsole.cpp, main.cpp, etc.).
// variant.h pulls this in after configuration.h has already included
// <Arduino.h>, so Stream and friends are guaranteed to be declared.
//
// M1.0 behaviour is intentionally inert: writes are discarded, reads return
// empty. The real shared-SRAM ring plumbing happens in M1.3.
//
// License: MIT.

#pragma once

#include <Arduino.h>

class IpcSerialStream : public Stream {
  public:
    IpcSerialStream() = default;

    // Stream / Print virtual interface — all no-ops for the spike.
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t * /*buf*/, size_t len) override { return len; }
    using Print::write;

    // Meshtastic calls Serial.begin()/end() from main boot code.
    void begin(unsigned long /*baud*/ = 115200) {}
    void begin(unsigned long /*baud*/, uint16_t /*config*/) {}
    void end() {}

    // `if (Serial)` keeps the framework draining into our ring. Reporting
    // "connected" is safe here because TX is a silent drop for M1.0.
    operator bool() const { return true; }
};

extern IpcSerialStream Serial;
