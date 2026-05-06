/* ipc_dormant_handler.cpp — Phase C Sprint 3b dormant handshake (Core 0).
 *
 * Handles IPC_CMD_DORMANT_REQUEST (0x8D) and IPC_CMD_DORMANT_WAKE (0x8E)
 * from Core 1.  Dispatched from mokya_handle_ipc_command() in
 * ipc_command_handler.cpp, which is called from
 * IpcSerialStream::refill_rx_() — i.e., the Meshtastic main loop context
 * (main.cpp loop() -> Serial.available()).  Safe to delay() / call radio
 * APIs from here.
 *
 * Sprint 3b minimal scope: SX1262 warm sleep + watchdog pause + ACK.  No
 * Meshtastic main loop body suspension yet — Sprint 2 (lite) Core 0 idle
 * hook WFI already saves the gap-time, and the radio sleep adds the
 * ~10 mA continuous-RX bite.  If bench shows we still miss the < 5 mA
 * target after this, a follow-up commit can add a loop-body skip gate
 * keyed off g_c0_dormant_pending.
 *
 * Wake path: clear pending flag, startReceive() to re-arm SX1262 RX from
 * warm sleep (single SPI transaction wakes the chip; config is retained),
 * resume watchdog liveness silence-detect.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <Arduino.h>
#include "pico/multicore.h"

#include "RadioLibInterface.h"

#include "ipc_protocol.h"
#include "ipc_ringbuf.h"
#include "ipc_shared_layout.h"

/* Cap on how long to wait for an in-flight LoRa TX to finish before we
 * sleep the radio.  If still TX-busy after this, decline the dormant
 * request (no ACK pushed) and let Core 1's 500 ms wait time out.
 * 200 ms covers the longest single-frame airtime we use at SF11/125 kHz
 * (~155 ms for 30 B + headers); above that the request is suspect. */
#define DORMANT_TX_WAIT_MS  200u

/* SWD-observable counters.  Mirror the Core 1 g_dormant_* set so a
 * harness can compare both halves of the handshake. */
extern "C" {
volatile uint32_t g_c0_dormant_request_count __attribute__((used)) = 0u;
volatile uint32_t g_c0_dormant_wake_count    __attribute__((used)) = 0u;
volatile uint32_t g_c0_dormant_busy_count    __attribute__((used)) = 0u;
volatile uint32_t g_c0_dormant_ack_pushed    __attribute__((used)) = 0u;
volatile uint8_t  g_c0_dormant_pending       __attribute__((used)) = 0u;
}

extern "C" void mokya_handle_ipc_dormant_request(uint8_t /*ipc_seq*/,
                                                 const uint8_t * /*payload*/,
                                                 uint16_t /*payload_len*/)
{
    g_c0_dormant_request_count++;

    /* Wait for any in-flight LoRa TX to finish before sleeping the
     * chip.  setSleep mid-TX corrupts the air frame and kills the
     * peer-side ACK.  If we run out of patience, decline by NOT
     * sending ACK — Core 1 stays in IDLE and can retry. */
    uint32_t deadline = millis() + DORMANT_TX_WAIT_MS;
    while (RadioLibInterface::instance && RadioLibInterface::instance->isSending()) {
        if ((int32_t)(millis() - deadline) >= 0) {
            g_c0_dormant_busy_count++;
            return;
        }
        delay(1);
    }

    /* Pause the watchdog liveness silence-detect.  c0_heartbeat will
     * stop advancing once Meshtastic stops getting CPU time during the
     * SLEEP window; without pause, wd_task on Core 1 would stop
     * kicking the HW watchdog and the chip would reset.  Resume on
     * wake. */
    mokya_watchdog_pause();

    /* SX126xInterface::sleep() does setStandby + lora.sleep(true) +
     * (if wired) FEM sleep.  keepConfig=true means startReceive() on
     * wake resumes RX without a full reconfigure. */
    if (RadioLibInterface::instance) {
        RadioLibInterface::instance->sleep();
    }

    g_c0_dormant_pending = 1u;

    bool pushed = ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                                g_ipc_shared.c0_to_c1_slots,
                                IPC_RING_SLOT_COUNT,
                                IPC_MSG_DORMANT_ACK,
                                0u,
                                nullptr,
                                0u);
    if (pushed) {
        g_c0_dormant_ack_pushed++;
        multicore_doorbell_set_other_core(IPC_DOORBELL_NUM);
    }
    /* If the push failed (ring full), Core 1 will time out and stay in
     * IDLE.  We've already slept the radio though — the wake handler
     * will be called by the next pwr-button cycle on Core 1, OR (if
     * Core 1 also gave up) the next Meshtastic-driven event will spin
     * the radio back up via startReceive() in the normal path.  Not
     * pretty, but no chip-level wedge. */
}

extern "C" void mokya_handle_ipc_dormant_wake(uint8_t /*ipc_seq*/,
                                              const uint8_t * /*payload*/,
                                              uint16_t /*payload_len*/)
{
    g_c0_dormant_wake_count++;
    g_c0_dormant_pending = 0u;

    /* Re-arm SX1262 RX from warm sleep.  startReceive() issues SPI
     * commands to wake the chip and re-enter RX with the previous
     * modem config.  Idempotent if the radio was already awake. */
    if (RadioLibInterface::instance) {
        RadioLibInterface::instance->startReceive();
    }

    mokya_watchdog_resume();
}
