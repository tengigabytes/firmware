/* ipc_gps_stream.h — Arduino Stream adapter over IpcGpsBuf (M3.5)
 *
 * Core 1 writes one NMEA sentence per second into IpcGpsBuf in shared SRAM
 * and atomically flips write_idx. This Stream lets Meshtastic's GPS class
 * consume those sentences as if they came from a UART, without us having to
 * stand up a real Serial port on Core 0.
 *
 * Selected at build time via -DMOKYA_IPC_GPS_STREAM=1; when defined,
 * GPS::_serial_gps is typed as Stream* (see GPS.h patch) and initVariant()
 * wires it to the singleton instance returned by ipc_gps_stream::instance().
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <Arduino.h>

class IpcGpsStream : public Stream {
  public:
    IpcGpsStream();

    // ── Stream / Print virtual overrides ─────────────────────────────────────
    int available() override;
    int read() override;
    int peek() override;
    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    // Singleton — created on first call. initVariant() must call this and
    // hand the pointer to GPS::setExternalSerial() before createGps() runs.
    static IpcGpsStream &instance();

  private:
    /* Latch the most recently published slot; advance read_pos as bytes are
     * consumed. When write_idx flips again, latch the new slot. */
    void refreshLatch();

    uint8_t  last_seen_idx;   ///< write_idx value at last latch (255 = none)
    uint8_t  read_pos;        ///< Bytes already returned from latched slot
    uint8_t  latched_idx;     ///< Index of the slot we are currently draining
};
