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
#define MOKYA_WD_PAUSE_ADDR    ((volatile uint32_t *)0x2007A018u)

#define MOKYA_FLASH_LOCK_IDLE    0u
#define MOKYA_FLASH_LOCK_REQUEST 1u
#define MOKYA_FLASH_LOCK_PARKED  2u
#define MOKYA_FLASH_DOORBELL     1u   /* must match IPC_FLASH_DOORBELL */

/* XIP_CTRL register — hardcoded to avoid header dependency.
 * Bits 0-1 (EN_SECURE, EN_NONSECURE) enable the XIP cache.  ROM
 * flash_exit_xip() clears the register and boot2 only restores QMI —
 * never XIP_CTRL.  We re-enable cache after each flash op.
 * Use the SET alias (base + 0x2000) for atomic bit-set without RMW. */
#define MOKYA_XIP_CTRL_SET  (*(volatile uint32_t *)0x400CA000u)
#define MOKYA_XIP_CACHE_EN  0x00000003u  /* EN_SECURE | EN_NONSECURE */

/* Provided by the --wrap linker mechanism */
extern void __real_flash_range_erase(uint32_t flash_offs, size_t count);
extern void __real_flash_range_program(uint32_t flash_offs,
                                       const uint8_t *data, size_t count);

/* SIO hardware spinlock — Phase 1.X two-core flash mutex. Must match
 * IPC_FLASH_SPINLOCK_NUM in firmware/shared/ipc/ipc_shared_layout.h.
 * Hardcoded to ID 29 here too because this Pico SDK-level wrapper
 * intentionally avoids the ipc_shared_layout.h include. */
#define MOKYA_FLASH_SPINLOCK_ADDR  ((volatile uint32_t *)(0xD0000100u + 29u * 4u))

/* ── SIO hardware spinlock — Phase 1.X two-core flash mutex (RAM) ─────── *
 *
 * Acquire MUST happen before transitioning flash_lock IDLE→REQUEST.
 * See firmware/shared/ipc/ipc_shared_layout.h IPC_FLASH_SPINLOCK_* and
 * the matching helpers in firmware/core1/src/ime/flash_safety_wrap.c. */
static void __no_inline_not_in_flash_func(mokya_flash_spinlock_acquire)(void)
{
    /* Unbounded — peer wrap is bounded by actual flash op (~100 ms);
     * peer crash is recovered by HW watchdog (3 s) anyway. Read returns
     * non-zero on successful acquire, 0 if locked. */
    while (*MOKYA_FLASH_SPINLOCK_ADDR == 0u) { /* spin */ }
}

static void __no_inline_not_in_flash_func(mokya_flash_spinlock_release)(void)
{
    *MOKYA_FLASH_SPINLOCK_ADDR = 0u;
}

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
    /* Pause the watchdog liveness check across the flash op — Core 0's
     * IRQ disable stalls the heartbeat tick that wd_task expects.
     * Spinlock acquire MUST happen before park request so concurrent
     * C1 wrap can't both signal doorbells to each other. */
    __atomic_fetch_add(MOKYA_WD_PAUSE_ADDR, 1u, __ATOMIC_RELAXED);
    mokya_flash_spinlock_acquire();
    mokya_flash_park_core1(&saved);
    __real_flash_range_erase(flash_offs, count);
    MOKYA_XIP_CTRL_SET = MOKYA_XIP_CACHE_EN;  /* re-enable cache */
    mokya_flash_unpark_core1(saved);
    mokya_flash_spinlock_release();
    __atomic_fetch_sub(MOKYA_WD_PAUSE_ADDR, 1u, __ATOMIC_RELAXED);
}

void __no_inline_not_in_flash_func(__wrap_flash_range_program)(
    uint32_t flash_offs, const uint8_t *data, size_t count)
{
    uint32_t saved;
    __atomic_fetch_add(MOKYA_WD_PAUSE_ADDR, 1u, __ATOMIC_RELAXED);
    mokya_flash_spinlock_acquire();
    mokya_flash_park_core1(&saved);
    __real_flash_range_program(flash_offs, data, count);
    MOKYA_XIP_CTRL_SET = MOKYA_XIP_CACHE_EN;  /* re-enable cache */
    mokya_flash_unpark_core1(saved);
    mokya_flash_spinlock_release();
    __atomic_fetch_sub(MOKYA_WD_PAUSE_ADDR, 1u, __ATOMIC_RELAXED);
}
