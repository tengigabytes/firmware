/* poc_c0_reset.cpp — POC: trigger a Core-0-only watchdog reset.
 *
 * Goal: prove that we can reset proc0 alone via WATCHDOG + PSM_WDSEL = PROC0,
 * leaving Core 1, USB CDC, and shared SRAM intact. If this works the same
 * mechanism will eventually back IPC_CMD_RESET_CORE0 / "selective reboot
 * after settings change".
 *
 * Build-gated by MOKYA_POC_C0_RESET=1. With the flag off this TU is empty.
 *
 * Test flow:
 *   1. Core 0 boots normally.
 *   2. Spawn a FreeRTOS task from initVariant() that delays kPocDelayMs.
 *   3. Task sets PSM_WDSEL = PROC0_BITS (only proc0 in reset domain), arms
 *      watchdog at 1 ms timeout, busy-waits.
 *   4. proc0 resets. Bootrom + arduino-pico runtime + Meshtastic setup()
 *      runs again. variant.cpp's warm-boot detect path skips the cross-core
 *      handshake so it doesn't disturb the still-running Core 1.
 *   5. Observe Core 1 RTT trace + host CDC connectivity across the event.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(MOKYA_POC_C0_RESET)

#include <Arduino.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "configuration.h"

#include "hardware/regs/addressmap.h"
#include "hardware/regs/psm.h"
#include "hardware/regs/watchdog.h"

#ifndef MOKYA_POC_C0_RESET_DELAY_MS
#define MOKYA_POC_C0_RESET_DELAY_MS  30000u
#endif

namespace {

constexpr uint32_t kPocDelayMs = MOKYA_POC_C0_RESET_DELAY_MS;

void poc_trigger_proc0_reset(void)
{
    /* Restrict watchdog reset domain to proc0 only. Other subsystems
     * (PROC1, SIO, XIP, all SRAMs, ROM, BUSFABRIC, CLOCKS, RESETS, XOSC,
     * ROSC, OTP) stay out of the mask, so Core 1 + USB CDC + shared SRAM
     * are not touched.
     *
     * Direct register write — pico-sdk's watchdog_reboot() unconditionally
     * sets PSM_WDSEL = ALL_BITS, which is exactly what we don't want here.
     */
    *reinterpret_cast<volatile uint32_t *>(PSM_BASE + PSM_WDSEL_OFFSET) =
        PSM_WDSEL_PROC0_BITS;

    /* Arm watchdog: 1 ms timeout. WATCHDOG_LOAD is in 1 µs ticks on RP2350
     * (hardware divides watchdog_freq_mhz / 1 internally), so 1000 = 1 ms.
     * ENABLE bit starts the countdown immediately.
     *
     * (inferred — RP2350 datasheet §12.10 says LOAD is in microseconds via
     *  the WATCHDOG_TICK that bootrom configures from XOSC; standard pico-sdk
     *  convention. To verify if POC fails: check WATCHDOG_TICK + frequency.)
     */
    volatile uint32_t *load = reinterpret_cast<volatile uint32_t *>(
        WATCHDOG_BASE + WATCHDOG_LOAD_OFFSET);
    volatile uint32_t *ctrl = reinterpret_cast<volatile uint32_t *>(
        WATCHDOG_BASE + WATCHDOG_CTRL_OFFSET);

    *load = 1000u;
    *ctrl = WATCHDOG_CTRL_ENABLE_BITS;

    for (;;) {
        /* spin until reset asserts */
        __asm volatile("nop");
    }
}

void poc_task(void *arg)
{
    (void)arg;

    /* Wait long enough for Meshtastic setup to complete and for the user
     * to attach RTT logger / observe a normal `meshtastic --info`. */
    vTaskDelay(pdMS_TO_TICKS(kPocDelayMs));

    LOG_WARN("POC: triggering proc0-only watchdog reset (WDSEL=PROC0)");

    /* Give the log line time to drain through the IPC ring to Core 1's
     * USB CDC before we yank the rug. */
    vTaskDelay(pdMS_TO_TICKS(200));

    poc_trigger_proc0_reset();
}

} // namespace

extern "C" void mokya_poc_c0_reset_arm(void)
{
    /* xTaskCreate is callable before the scheduler is running; the task
     * just queues up and runs once vTaskStartScheduler() fires. initVariant
     * happens inside the __core0 task body though, so by the time we're
     * here the scheduler is already up and the new task starts on the
     * next tick. */
    BaseType_t rc = xTaskCreate(poc_task,
                                "poc_c0_rst",
                                512,        /* small — just delays + writes a few regs */
                                nullptr,
                                tskIDLE_PRIORITY + 1,
                                nullptr);
    (void)rc;
}

#else  /* !MOKYA_POC_C0_RESET */

extern "C" void mokya_poc_c0_reset_arm(void) { /* no-op */ }

#endif /* MOKYA_POC_C0_RESET */
