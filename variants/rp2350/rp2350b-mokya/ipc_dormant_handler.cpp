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

/* Phase C-1 design pivot (direction A, 2026-05-06): radio is NO LONGER
 * slept by the dormant handshake.  Rationale: putting SX1262 in warm
 * sleep saves ~6 mA but cuts off LoRa RX entirely, so peer DMs sent
 * during the SLEEP window vanish (chip can't fire DIO1 because it isn't
 * listening).  Upstream Meshtastic on nRF52 / RP2040 / ESP32 keeps the
 * radio in continuous RX and saves power on the MCU side instead via
 * `mainDelay.delay()` + idle-hook WFI; we follow that pattern here.
 *
 * What this handler still does:
 *   - Wait for any in-flight LoRa TX to finish (so a long blocking TX
 *     doesn't get cut by Core 1's WFI loop suspending the scheduler).
 *   - Pause the watchdog liveness silence-detect (Core 1's WFI loop
 *     stalls c1_heartbeat; without pause Core 1's wd_task would stop
 *     kicking the HW watchdog and the chip would reset).
 *   - Set g_c0_dormant_pending = 1 (SWD-observable marker).
 *   - Push DORMANT_ACK so Core 1's request_dormant_via_ipc unblocks and
 *     can enter its WFI loop.
 *
 * What this handler intentionally does NOT do:
 *   - Touch the SX1262.  Radio stays in continuous RX.  Peer DMs reach
 *     Core 0's RX handler normally; the resulting cascade decode pushes
 *     the FromRadio frame onto the c0->c1 ring.  Once Core 1 wakes from
 *     its WFI loop (PWR button) the queued frame is processed.  DMs are
 *     not lost across the SLEEP window.
 *
 * The TX-busy-decline path (g_c0_dormant_busy_count) is retained: if
 * Core 1 requests dormant entry while a packet is mid-TX, we wait up to
 * DORMANT_TX_WAIT_MS for the TX to finish, otherwise decline by NOT
 * sending ACK so Core 1 times out and stays in IDLE.  This is still
 * useful even without radio sleep because suspending Core 1's scheduler
 * mid-TX could starve any TX-completion callbacks that touch IPC. */

extern "C" void mokya_handle_ipc_dormant_request(uint8_t /*ipc_seq*/,
                                                 const uint8_t * /*payload*/,
                                                 uint16_t /*payload_len*/)
{
    g_c0_dormant_request_count++;

    /* Wait for any in-flight LoRa TX to finish before parking. */
    uint32_t deadline = millis() + DORMANT_TX_WAIT_MS;
    while (RadioLibInterface::instance && RadioLibInterface::instance->isSending()) {
        if ((int32_t)(millis() - deadline) >= 0) {
            g_c0_dormant_busy_count++;
            return;
        }
        delay(1);
    }

    mokya_watchdog_pause();

    /* Direction A: radio stays in continuous RX -- DO NOT sleep it.
     * (Previously called RadioLibInterface::instance->sleep() here.) */

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
}

extern "C" void mokya_handle_ipc_dormant_wake(uint8_t /*ipc_seq*/,
                                              const uint8_t * /*payload*/,
                                              uint16_t /*payload_len*/)
{
    g_c0_dormant_wake_count++;
    g_c0_dormant_pending = 0u;

    /* Direction A: radio was never slept, so no startReceive() to call.
     * (Previously called RadioLibInterface::instance->startReceive() here.) */

    mokya_watchdog_resume();
}
