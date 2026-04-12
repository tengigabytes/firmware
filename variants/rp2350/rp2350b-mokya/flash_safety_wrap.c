/* flash_safety_wrap.c — MokyaLora P2-11 fix: park Core 1 during flash writes
 *
 * Problem: Pico SDK's flash_range_erase/program call flash_exit_xip() which
 * disables XIP. If Core 1 is still executing from flash (or Core 0's SysTick
 * fires), the instruction fetch causes IACCVIOL HardFault.
 *
 * Solution: Use -Wl,--wrap to intercept flash_range_erase and
 * flash_range_program. Before calling the real implementation (__real_*),
 * we park Core 1 in a RAM-only spin via doorbell + shared-SRAM handshake,
 * and disable Core 0 interrupts. After the flash op completes, we restore
 * both.
 *
 * This file is compiled as part of the variant sources and linked as a
 * strong symbol that overrides the archive (libpico.a) definition via
 * the --wrap mechanism.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>
#include <stddef.h>
#include "pico.h"
#include "hardware/structs/sio.h"
#include "hardware/sync.h"

/* ── Hardcoded shared-SRAM addresses (must match ipc_shared_layout.h) ──── *
 *
 * IpcSharedSram lives at 0x2007A000 (IPC_SHARED_ORIGIN).
 *   offset  0: boot_magic   (uint32)
 *   offset  4: c0_ready     (uint32)
 *   offset  8: c1_ready     (uint32)
 *   offset 12: flash_lock   (uint32)
 *
 * We hardcode these to avoid an #include dependency on ipc_shared_layout.h
 * from this Pico SDK-level wrapper. Any layout change must update both. */
#define MOKYA_C1_READY_ADDR    ((volatile uint32_t *)0x2007A008u)
#define MOKYA_FLASH_LOCK_ADDR  ((volatile uint32_t *)0x2007A00Cu)

#define MOKYA_FLASH_LOCK_IDLE    0u
#define MOKYA_FLASH_LOCK_REQUEST 1u
#define MOKYA_FLASH_LOCK_PARKED  2u
#define MOKYA_FLASH_DOORBELL     1u   /* must match IPC_FLASH_DOORBELL */

/* Provided by the --wrap linker mechanism */
extern void __real_flash_range_erase(uint32_t flash_offs, size_t count);
extern void __real_flash_range_program(uint32_t flash_offs,
                                       const uint8_t *data, size_t count);

/* ── Park / unpark helpers (MUST be in RAM) ────────────────────────────── */

static void __no_inline_not_in_flash_func(mokya_flash_park_core1)(
    uint32_t *saved_irq)
{
    if (*MOKYA_C1_READY_ADDR) {
        *MOKYA_FLASH_LOCK_ADDR = MOKYA_FLASH_LOCK_REQUEST;
        __compiler_memory_barrier();
        sio_hw->doorbell_out_set = 1u << MOKYA_FLASH_DOORBELL;
        /* Bounded wait (~5 ms at 150 MHz). If Core 1 ISR is not yet
         * enabled (brief window after c1_ready), fall through — local
         * interrupt disable still protects Core 0's own SysTick. */
        for (uint32_t i = 0; i < 750000u; i++) {
            if (*MOKYA_FLASH_LOCK_ADDR == MOKYA_FLASH_LOCK_PARKED) break;
        }
    }
    *saved_irq = save_and_disable_interrupts();
}

static void __no_inline_not_in_flash_func(mokya_flash_unpark_core1)(
    uint32_t saved_irq)
{
    restore_interrupts(saved_irq);
    *MOKYA_FLASH_LOCK_ADDR = MOKYA_FLASH_LOCK_IDLE;
    __compiler_memory_barrier();
    __sev();
}

/* ── Wrapped flash functions ───────────────────────────────────────────── */

void __no_inline_not_in_flash_func(__wrap_flash_range_erase)(
    uint32_t flash_offs, size_t count)
{
    uint32_t saved;
    mokya_flash_park_core1(&saved);
    __real_flash_range_erase(flash_offs, count);
    mokya_flash_unpark_core1(saved);
}

void __no_inline_not_in_flash_func(__wrap_flash_range_program)(
    uint32_t flash_offs, const uint8_t *data, size_t count)
{
    uint32_t saved;
    mokya_flash_park_core1(&saved);
    __real_flash_range_program(flash_offs, data, count);
    mokya_flash_unpark_core1(saved);
}
