/* flash_park_listener.c — Core 0 side of P2-11 flash-park (Phase 1.6 reverse).
 *
 * Symmetric counterpart of flash_safety_wrap.c. The wrap file covers the
 * direction where Core 0 writes flash and asks Core 1 to park; this file
 * covers the direction where Core 1 writes flash (LRU persist) and asks
 * Core 0 to park.
 *
 * Protocol (Core 1 writes, Core 0 parks):
 *   1. Core 1 sets flash_lock_c0 = REQUEST and rings IPC_FLASH_DOORBELL_C0.
 *   2. This ISR (SIO_IRQ_BELL on Core 0) clears the doorbell bit and —
 *      if flash_lock_c0 == REQUEST — calls flash_park_handler_c0.
 *   3. Park handler ACKs with PARKED, disables Core 0 interrupts, spins
 *      in RAM until Core 1 clears flash_lock_c0 back to IDLE + __SEV().
 *   4. Restores interrupts and returns.
 *
 * Registration: mokya_flash_park_listener_init() claims SIO_IRQ_BELL on
 * Core 0, installed from initVariant() after ipc_shared_init(). Arduino-
 * Pico's FreeRTOS single-core port has its pico_sync_interop doorbell ISR
 * patched out (see patch_arduinopico.py Issue P2-3), so SIO_IRQ_BELL is
 * unclaimed on this core and free for us to take.
 *
 * SPDX-License-Identifier: MIT
 */
#include <stdint.h>
#include <stddef.h>

#include "pico.h"
#include "hardware/irq.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"

#include "ipc_shared_layout.h"

#ifndef SIO_IRQ_BELL
#define SIO_IRQ_BELL  26   /* RP2350 doorbell IRQ (secure) — matches Core 1. */
#endif

/* XIP_CTRL SET alias — re-enable cache after returning (defensive; the
 * parked path does not itself flip XIP but Core 1's wrap pokes XIP_CTRL
 * and we want symmetry of recovery). */
#define MOKYA_XIP_CTRL_SET  (*(volatile uint32_t *)0x400CA000u)
#define MOKYA_XIP_CACHE_EN  0x00000003u

/* ── Park handler (MUST reside in RAM — runs while XIP is off on Core 1) ── *
 *
 * Keep this RAM-only so the instruction fetch path doesn't touch flash
 * while the other core has disabled XIP. save_and_disable_interrupts()
 * and the spin loop are inlined helpers from hardware/sync.h that are
 * already RAM-resident.                                                     */
static void __no_inline_not_in_flash_func(flash_park_handler_c0)(void)
{
    __atomic_store_n(&g_ipc_shared.flash_lock_c0,
                     IPC_FLASH_LOCK_PARKED, __ATOMIC_RELEASE);
    __sev();  /* wake Core 1 polling on flash_lock_c0 */

    uint32_t saved = save_and_disable_interrupts();

    /* Spin until Core 1 finishes the flash op and releases the lock.
     * __wfe() is a low-power wait; Core 1 will __sev() after clearing. */
    while (__atomic_load_n(&g_ipc_shared.flash_lock_c0,
                           __ATOMIC_ACQUIRE) != IPC_FLASH_LOCK_IDLE) {
        __wfe();
    }

    MOKYA_XIP_CTRL_SET = MOKYA_XIP_CACHE_EN;
    restore_interrupts(saved);
}

/* ── SIO_IRQ_BELL ISR ──────────────────────────────────────────────────── *
 *
 * Fires for ANY doorbell bit set on this core. We only claim the
 * flash-park bit; if other bits ever land here (they shouldn't — Core 0's
 * normal IPC notifications use its own polled path), we clear them
 * defensively to avoid a stuck pending IRQ.                                */
static void __no_inline_not_in_flash_func(mokya_flash_park_isr)(void)
{
    if (sio_hw->doorbell_in_set & (1u << IPC_FLASH_DOORBELL_C0)) {
        sio_hw->doorbell_in_clr = 1u << IPC_FLASH_DOORBELL_C0;
        if (__atomic_load_n(&g_ipc_shared.flash_lock_c0,
                            __ATOMIC_ACQUIRE) == IPC_FLASH_LOCK_REQUEST) {
            flash_park_handler_c0();
        }
    }
    /* Clear any other pending bits we don't own. Core 0 does not use
     * doorbells outside this module, so anything else here is spurious. */
    uint32_t pending = sio_hw->doorbell_in_set;
    pending &= ~(1u << IPC_FLASH_DOORBELL_C0);
    if (pending) {
        sio_hw->doorbell_in_clr = pending;
    }
}

/* ── Public init — called from initVariant() after ipc_shared_init() ───── */

void mokya_flash_park_listener_init(void)
{
    /* Clear any spurious doorbell state left from bootrom / prior boot. */
    sio_hw->doorbell_in_clr = 0xFFFFFFFFu;

    irq_set_exclusive_handler(SIO_IRQ_BELL, mokya_flash_park_isr);
    irq_set_priority(SIO_IRQ_BELL, 0x40);  /* mid priority — below HardFault,
                                             * above SysTick-ish traffic.  */
    irq_set_enabled(SIO_IRQ_BELL, true);
}
