/* postmortem.cpp — Core 0 panic / fault snapshot capture + boot surface.
 *
 * Mirrors firmware/core1/src/debug/postmortem.c but for Arduino-Pico's
 * crt0 path. Strong-overrides the four Cortex-M fault ISR weak symbols
 * from pico-sdk so anything that lands in HardFault / MemManage /
 * BusFault / UsageFault on Core 0 captures a record into
 * g_ipc_shared.postmortem_c0 before triggering SYSRESETREQ.
 *
 * Also provides:
 *   - mokya_pm_snapshot_graceful_reboot(): tagged record for the
 *     RebootNotifier path so a soft reboot is distinguishable from a
 *     fault on next boot.
 *   - mokya_pm_surface_on_boot(): early-setup() surface that prints
 *     any pending record via Serial. Idempotent.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>
#include <string.h>
#include <Arduino.h>

#include "ipc_shared_layout.h"
#include "mokya_postmortem.h"

/* AIRCR + SCB fault status — hardcoded addresses keep us independent of
 * Arduino-Pico's varying header set. */
#define MOKYA_AIRCR_ADDR    ((volatile uint32_t *)0xE000ED0Cu)
#define MOKYA_AIRCR_VECTKEY 0x05FA0000u
#define MOKYA_AIRCR_RESET   (MOKYA_AIRCR_VECTKEY | (1u << 2))

#define MOKYA_SCB_CFSR  ((volatile uint32_t *)0xE000ED28u)
#define MOKYA_SCB_HFSR  ((volatile uint32_t *)0xE000ED2Cu)
#define MOKYA_SCB_MMFAR ((volatile uint32_t *)0xE000ED34u)
#define MOKYA_SCB_BFAR  ((volatile uint32_t *)0xE000ED38u)

#define MOKYA_TIMER_TIMERAWL ((volatile uint32_t *)0x400B800Cu)

extern "C" {

void mokya_pm_snapshot_graceful_reboot(void)
{
    mokya_postmortem_t *pm = &g_ipc_shared.postmortem_c0;
    if (pm->magic == MOKYA_PM_MAGIC) return;  /* don't overwrite a fault */
    memset(pm, 0, sizeof(*pm));
    pm->cause        = (uint32_t)MOKYA_PM_CAUSE_GRACEFUL_REBOOT;
    pm->timestamp_us = *MOKYA_TIMER_TIMERAWL;
    pm->core         = 0;
    pm->c0_heartbeat = __atomic_load_n(&g_ipc_shared.c0_heartbeat,
                                       __ATOMIC_RELAXED);
    pm->wd_pause     = __atomic_load_n(&g_ipc_shared.wd_pause,
                                       __ATOMIC_RELAXED);
    __atomic_store_n(&pm->magic, MOKYA_PM_MAGIC, __ATOMIC_RELEASE);
}

__attribute__((noreturn, used))
void mokya_pm_fault_capture_c0(uint32_t *frame, uint32_t cause,
                                uint32_t exc_return)
{
    mokya_postmortem_t *pm = &g_ipc_shared.postmortem_c0;
    memset(pm, 0, sizeof(*pm));
    pm->cause        = cause;
    pm->timestamp_us = *MOKYA_TIMER_TIMERAWL;
    pm->core         = 0;

    pm->r0  = frame[0];
    pm->r1  = frame[1];
    pm->r2  = frame[2];
    pm->r3  = frame[3];
    pm->r12 = frame[4];
    pm->lr  = frame[5];
    pm->pc  = frame[6];
    pm->psr = frame[7];
    pm->sp  = (uint32_t)frame;
    pm->exc_return = exc_return;

    pm->cfsr  = *MOKYA_SCB_CFSR;
    pm->hfsr  = *MOKYA_SCB_HFSR;
    pm->mmfar = *MOKYA_SCB_MMFAR;
    pm->bfar  = *MOKYA_SCB_BFAR;

    pm->c0_heartbeat = __atomic_load_n(&g_ipc_shared.c0_heartbeat,
                                       __ATOMIC_RELAXED);
    pm->wd_pause     = __atomic_load_n(&g_ipc_shared.wd_pause,
                                       __ATOMIC_RELAXED);
    /* Skip task name on Core 0 — Arduino-Pico's FreeRTOS handle access
     * is fragile from fault context and not worth the complexity here.
     * Serial print on next boot still shows Core 0 = the source. */

    __atomic_store_n(&pm->magic, MOKYA_PM_MAGIC, __ATOMIC_RELEASE);

    *MOKYA_AIRCR_ADDR = MOKYA_AIRCR_RESET;
    for (;;) { __asm volatile ("dsb 0xF; isb"); }
}

#define MOKYA_PM_C0_FAULT_STUB(name, cause)                       \
    __attribute__((naked, used))                                  \
    void name(void)                                               \
    {                                                             \
        __asm volatile (                                          \
            "mov r2, lr                  \n"                      \
            "tst lr, #4                  \n"                      \
            "ite eq                      \n"                      \
            "mrseq r0, msp               \n"                      \
            "mrsne r0, psp               \n"                      \
            "movs r1, %[cs]              \n"                      \
            "b mokya_pm_fault_capture_c0 \n"                      \
            :: [cs] "i" (cause)                                   \
        );                                                        \
    }

MOKYA_PM_C0_FAULT_STUB(isr_hardfault,    MOKYA_PM_CAUSE_HARDFAULT)
MOKYA_PM_C0_FAULT_STUB(isr_memmanage,    MOKYA_PM_CAUSE_MEMMANAGE)
MOKYA_PM_C0_FAULT_STUB(isr_busfault,     MOKYA_PM_CAUSE_BUSFAULT)
MOKYA_PM_C0_FAULT_STUB(isr_usagefault,   MOKYA_PM_CAUSE_USAGEFAULT)

/* No Core-0-side surface — Core 1's mokya_pm_surface_on_boot() reads
 * BOTH g_ipc_shared.postmortem_c0 and postmortem_c1 via RTT TRACE,
 * which is reliable from Core 1's earliest boot and avoids Serial
 * timing fragility on the Arduino-Pico CDC path. Core 0's job here
 * is purely the writer side (graceful reboot tag + fault capture). */

}  // extern "C"
