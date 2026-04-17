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

#include <pico/multicore.h>
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
    // Accumulate single-byte writes (mainly from RedirectablePrint log
    // output) and flush as one ring push when the buffer is full OR on
    // newline.  Log lines from RedirectablePrint end with '\n'; flushing
    // on that boundary keeps batching efficient while preventing stalls
    // when accumulated bytes never reach the 256-byte buffer cap between
    // protobuf frame writes.
    tx_acc_[tx_acc_len_++] = b;
    if (tx_acc_len_ >= sizeof(tx_acc_) || b == '\n') {
        flush_tx_acc_();
    }
    return 1;
}

size_t IpcSerialStream::write(const uint8_t *buf, size_t len)
{
    // Multi-byte write (protobuf frames from emitTxBuffer).  Flush any
    // accumulated single-byte data first so ordering is preserved, then
    // push the caller's buffer directly — no extra copy.
    flush_tx_acc_();

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
                                   IPC_RING_SLOT_COUNT,
                                   IPC_MSG_SERIAL_BYTES,
                                   tx_seq_,
                                   buf + written,
                                   chunk);
            if (pushed) {
                multicore_doorbell_set_other_core(IPC_DOORBELL_NUM);
                break;
            }
            if ((int32_t)(millis() - deadline) >= 0) break;
            yield();
        }
        if (!pushed) {
            break;
        }
        tx_seq_++;
        written += chunk;
    }
    return written;
}

void IpcSerialStream::flush()
{
    flush_tx_acc_();
}

void IpcSerialStream::flush_tx_acc_()
{
    if (tx_acc_len_ == 0u) return;

    // Log bytes go to the dedicated log ring — best-effort, single attempt,
    // no busy-wait.  If the log ring is full the bytes are silently dropped.
    // This keeps log output from contending with protobuf data on the main
    // data ring (see docs/design-notes/ipc-ram-replan.md §2.1).
    bool pushed = ipc_ring_push(&g_ipc_shared.c0_log_to_c1_ctrl,
                                g_ipc_shared.c0_log_to_c1_slots,
                                IPC_LOG_RING_SLOT_COUNT,
                                IPC_MSG_SERIAL_BYTES,
                                tx_seq_,
                                tx_acc_,
                                tx_acc_len_);
    if (pushed) {
        tx_seq_++;
        multicore_doorbell_set_other_core(IPC_DOORBELL_NUM);
    }
    tx_acc_len_ = 0u;
}

/* ── RX staging ────────────────────────────────────────────────────────── */

bool IpcSerialStream::refill_rx_()
{
    IpcMsgHeader hdr;
    if (!ipc_ring_pop(&g_ipc_shared.c1_to_c0_ctrl,
                      g_ipc_shared.c1_to_c0_slots,
                      IPC_RING_SLOT_COUNT,
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
