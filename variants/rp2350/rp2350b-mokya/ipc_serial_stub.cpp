// Phase 2 M1.1 — IpcSerialStream definition.
//
// See ipc_serial_stub.h for rationale. This TU owns the single global
// `Serial` instance that replaces Arduino-Pico's SerialUSB, and it is the
// only place in Core 0 that touches the shared-SRAM SPSC ring for the byte
// bridge (everything else goes through the framework Stream API).
//
// License: MIT.

#include "ipc_serial_stub.h"

#include "ipc_protocol.h"
#include "ipc_shared_layout.h"
#include "ipc_ringbuf.h"

#include <Arduino.h>

IpcSerialStream Serial;

// Maximum time to busy-wait on a full c0_to_c1 ring before giving up on a
// single chunk. 50 ms is well under any Meshtastic watchdog interval and
// matches the ADR-4 target.
static constexpr uint32_t kWriteBusyWaitMs = 50;

/* ── Public Stream API ─────────────────────────────────────────────────── */

int IpcSerialStream::available()
{
    if (rx_pos_ < rx_len_) {
        return rx_len_ - rx_pos_;
    }
    if (refill_rx_()) {
        return rx_len_ - rx_pos_;
    }
    return 0;
}

int IpcSerialStream::read()
{
    if (rx_pos_ >= rx_len_) {
        if (!refill_rx_()) {
            return -1;
        }
    }
    return rx_buf_[rx_pos_++];
}

int IpcSerialStream::peek()
{
    if (rx_pos_ >= rx_len_) {
        if (!refill_rx_()) {
            return -1;
        }
    }
    return rx_buf_[rx_pos_];
}

size_t IpcSerialStream::write(uint8_t b)
{
    return write(&b, 1);
}

size_t IpcSerialStream::write(const uint8_t *buf, size_t len)
{
    // Meshtastic's Serial framing assumes writes don't silently drop bytes.
    // We chunk into IPC_MSG_PAYLOAD_MAX-byte messages and busy-wait up to
    // kWriteBusyWaitMs per chunk when the ring is full; beyond that, we
    // drop the remainder and rely on the overflow counter + upper-layer
    // retry.
    size_t written = 0;
    while (written < len) {
        const size_t remaining = len - written;
        const uint16_t chunk = (remaining > IPC_MSG_PAYLOAD_MAX)
                                   ? static_cast<uint16_t>(IPC_MSG_PAYLOAD_MAX)
                                   : static_cast<uint16_t>(remaining);

        const uint32_t deadline = millis() + kWriteBusyWaitMs;
        bool pushed = false;
        for (;;) {
            pushed = ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                                   g_ipc_shared.c0_to_c1_slots,
                                   IPC_MSG_SERIAL_BYTES,
                                   tx_seq_,
                                   buf + written,
                                   chunk);
            if (pushed) break;
            if ((int32_t)(millis() - deadline) >= 0) break;
            // Yield briefly. No FreeRTOS delay here — this runs in the
            // __core0 Meshtastic task and the Arduino-Pico FreeRTOS port
            // remaps delay() onto vTaskDelay when available.
            yield();
        }
        if (!pushed) {
            // Dropped chunk. overflow counter already incremented inside
            // ipc_ring_push; return short write so the caller can decide.
            break;
        }
        tx_seq_++;
        written += chunk;
    }
    return written;
}

/* ── RX staging ────────────────────────────────────────────────────────── */

bool IpcSerialStream::refill_rx_()
{
    IpcMsgHeader hdr;
    if (!ipc_ring_pop(&g_ipc_shared.c1_to_c0_ctrl,
                      g_ipc_shared.c1_to_c0_slots,
                      &hdr,
                      rx_buf_,
                      sizeof(rx_buf_))) {
        return false;
    }

    if (hdr.msg_id != IPC_MSG_SERIAL_BYTES) {
        // Non-bytes messages (e.g. IPC_MSG_LOG_LINE) are not consumed by
        // Stream::read(). For M1 we discard them so the bridge doesn't
        // block; M4+ will route them into structured handlers.
        rx_len_ = 0;
        rx_pos_ = 0;
        return false;
    }

    rx_len_ = hdr.payload_len;
    rx_pos_ = 0;
    return rx_len_ > 0;
}
