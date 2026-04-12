// MokyaLora Rev A — RP2350B Core 0 variant hooks
//
// Phase 2 M1.0b: launch the Core 1 Apache-2.0 boot-spike image located at
// flash offset 0x10200000 via multicore_launch_core1_raw().
//
// Single-core FreeRTOS leaves Core 1 in the Pico SDK bootrom FIFO handler
// (see phase2-log.md Issue P2-2), so it is ready to accept the launch
// handshake the instant Core 0 reaches initVariant(). Arduino-Pico calls
// initVariant() from the __core0 FreeRTOS task, *before* Meshtastic setup(),
// so no Meshtastic subsystem has started yet when Core 1 is handed off.
//
// The spike writes a sentinel to 0x20078000 and halts in WFI; verification
// happens out-of-band via SWD (see docs/bringup/phase2-log.md M1.0b).

#include <Arduino.h>
#include <stdint.h>
#include "pico/multicore.h"
#include "ipc_ringbuf.h"
#include "ipc_shared_layout.h"
#include "ipc_protocol.h"
#include "Observer.h"
#include "sleep.h"

#define MOKYA_CORE1_VECTOR_TABLE   0x10200000u
#define MOKYA_CORE1_SENTINEL_ADDR  0x20078000u
// Debug breadcrumbs — four 32-bit slots read back over SWD after boot.
// Each slot is written at a specific phase of initVariant() so we can see
// exactly how far the Core 0 launch path progressed.
#define MOKYA_DBG_BASE             0x20078010u
// +0x00 phase marker: 0x1n for reached phase n
// +0x04 vector_table word[0] read back (initial SP)
// +0x08 vector_table word[1] read back (reset handler)
// +0x0C post-launch sentinel snapshot sampled by Core 0 after ~1 ms spin

/* ── Graceful reboot notification (M2 Part B) ───────────────────────────── */
/* When Meshtastic calls Power::reboot(), the notifyReboot observable fires
 * before watchdog_reboot(). We push IPC_MSG_REBOOT_NOTIFY so Core 1 can
 * tud_disconnect() before the chip-wide reset yanks the USB controller. */

struct RebootNotifier {
    int onReboot(void * /*arg*/)
    {
        /* Push zero-payload reboot notification to the c0→c1 ring. */
        (void)ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                            g_ipc_shared.c0_to_c1_slots,
                            IPC_MSG_REBOOT_NOTIFY,
                            0u,      /* seq */
                            nullptr, /* payload */
                            0u);     /* payload_len */
        /* Wake Core 1 immediately so it processes the notification. */
        multicore_doorbell_set_other_core(IPC_DOORBELL_NUM);
        /* Give Core 1 time to call tud_disconnect() and let the host
         * process the USB disconnect event before the watchdog fires. */
        delay(500);
        return 0;  /* continue — let Power::reboot() proceed */
    }

    CallbackObserver<RebootNotifier, void *> observer{this, &RebootNotifier::onReboot};
};

static RebootNotifier s_reboot_notifier;

extern "C" void initVariant()
{
    volatile uint32_t *const sentinel =
        reinterpret_cast<volatile uint32_t *>(MOKYA_CORE1_SENTINEL_ADDR);
    volatile uint32_t *const dbg =
        reinterpret_cast<volatile uint32_t *>(MOKYA_DBG_BASE);

    dbg[0] = 0x11u;  // phase 1: entered initVariant
    dbg[1] = 0;
    dbg[2] = 0;
    dbg[3] = 0;
    *sentinel = 0u;
    __asm volatile("dmb 0xF" ::: "memory");

    // Phase 2 M1.1: zero the shared-SRAM IPC region and publish IPC_BOOT_MAGIC
    // BEFORE launching Core 1. The bridge image spins on boot_magic until it
    // sees IPC_BOOT_MAGIC before touching any ring, so this ordering is the
    // handshake: if Core 1 ever sees bogus head/tail values it is this call
    // that failed to run.
    ipc_shared_init();
    dbg[0] = 0x11au;  // phase 1a: shared IPC zeroed + magic published

    // Core 1 image begins with a Cortex-M vector table at 0x10200000:
    //   word[0] = initial MSP
    //   word[1] = reset handler (Thumb bit set)
    const uint32_t *const vt =
        reinterpret_cast<const uint32_t *>(MOKYA_CORE1_VECTOR_TABLE);
    const uint32_t core1_sp    = vt[0];
    const uint32_t core1_entry = vt[1];

    dbg[1] = core1_sp;
    dbg[2] = core1_entry;
    dbg[0] = 0x12u;  // phase 2: vector table read back

    // Force Core 1 back into its bootrom FIFO handler before the handshake.
    // In theory Core 1 is still in the handler on first boot, but with
    // single-core FreeRTOS and unknown framework interaction this guarantees
    // a clean handshake state and mirrors what restartCore1() does.
    multicore_reset_core1();
    dbg[0] = 0x12au;  // phase 2a: reset_core1 returned

    multicore_launch_core1_raw(
        reinterpret_cast<void (*)(void)>(core1_entry),
        reinterpret_cast<uint32_t *>(core1_sp),
        MOKYA_CORE1_VECTOR_TABLE);

    dbg[0] = 0x13u;  // phase 3: multicore_launch_core1_raw returned

    // Give Core 1 a brief window to write the sentinel, then snapshot it.
    for (volatile int i = 0; i < 100000; ++i) { __asm volatile("nop"); }
    dbg[3] = *sentinel;
    dbg[0] = 0x14u;  // phase 4: sentinel snapshot captured

    // Register reboot observer — when Meshtastic calls Power::reboot(),
    // we notify Core 1 to disconnect USB before the watchdog fires.
    s_reboot_notifier.observer.observe(&notifyReboot);
    dbg[0] = 0x15u;  // phase 5: reboot observer registered
}
