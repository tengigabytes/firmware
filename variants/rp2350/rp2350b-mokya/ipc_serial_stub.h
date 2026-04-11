// Phase 2 M1.1 — IpcSerialStream: Meshtastic <-> shared-SRAM SPSC ring.
//
// The class has to live in a header so every Meshtastic TU that touches
// `Serial` can see the full interface (SerialConsole.cpp, main.cpp, etc.).
// variant.h pulls this in after configuration.h has already included
// <Arduino.h>, so Stream and friends are guaranteed to be declared.
//
// M1.0 was a discard-everything stub. M1.1 turns this into a real byte
// bridge: write() chunks the outgoing buffer into IPC_MSG_SERIAL_BYTES
// messages pushed to g_ipc_shared.c0_to_c1_ctrl/slots; read() drains the
// c1_to_c0 ring into a small local rx buffer and serves byte-at-a-time to
// match Arduino Stream semantics.
//
// License: MIT.

#pragma once

#include <Arduino.h>
#include <stdint.h>

class IpcSerialStream : public Stream {
  public:
    IpcSerialStream() = default;

    // Stream / Print virtual interface.
    int available() override;
    int read() override;
    int peek() override;
    void flush() override {}
    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buf, size_t len) override;
    using Print::write;

    // Meshtastic calls Serial.begin()/end() from main boot code. Both are
    // no-ops here — the ring is brought up in initVariant() before
    // setup() runs.
    void begin(unsigned long /*baud*/ = 115200) {}
    void begin(unsigned long /*baud*/, uint16_t /*config*/) {}
    void end() {}

    // `if (Serial)` must report connected so Meshtastic keeps flushing to
    // the ring even before Core 1 brings USB up.
    operator bool() const { return true; }

  private:
    // Local RX staging buffer. We pop one IPC_MSG_SERIAL_BYTES message at a
    // time from c1_to_c0 and serve its bytes via read()/peek()/available().
    uint8_t  rx_buf_[256];
    uint16_t rx_len_  = 0;   // valid bytes in rx_buf_
    uint16_t rx_pos_  = 0;   // next byte to serve
    uint8_t  tx_seq_  = 0;   // rolling sequence number for push()

    // Pull the next message off c1_to_c0 into rx_buf_ (if any). Returns
    // true if rx_buf_ gained bytes.
    bool refill_rx_();
};

extern IpcSerialStream Serial;
