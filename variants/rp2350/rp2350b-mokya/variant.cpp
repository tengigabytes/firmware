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
#include "pico/platform.h"
#include "ipc_ringbuf.h"
#include "ipc_shared_layout.h"
#include "ipc_protocol.h"
#include "Observer.h"
#include "sleep.h"

/* Pico SDK RP2350 — provides scb_hw (armv8m_scb_hw_t) and M33_SHCSR_* bit defs. */
#include "hardware/structs/scb.h"
#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/pads_qspi.h"
#include "hardware/structs/pads_qspi.h"
#include "hardware/sync.h"

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
                            IPC_RING_SLOT_COUNT,
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

/* ── P2-16 helper: retime M0 (flash) from bootrom CLKDIV=3/25 MHz to
 * CLKDIV=1/75 MHz. Must run from RAM: changing M0.timing has a tiny
 * window where an instruction fetch could land on the old timing while
 * the next one sees the new timing, and if we were executing from
 * flash we'd risk an IBUSERR. Direct-mode bracketing pauses XIP for
 * the duration so any speculative fetch is stalled, not faulted.
 *
 * RXDELAY=2 matches Arduino-Pico's rxdelay=divisor heuristic. COOLDOWN
 * is left at 1 (bootrom default); MIN_DESELECT is preserved via the
 * mask so we don't accidentally shrink the CS-high gap. Verified by
 * bringup flash_pad_ablation: this timing + SLEWFAST=1 passes the
 * CLKDIV=1 stress at 42.8 MB/s uncached / 44.4 MB/s cached. */
static void __no_inline_not_in_flash_func(flash_retime_clkdiv1)(void)
{
    const uint32_t kMask = QMI_M0_TIMING_COOLDOWN_BITS |
                           QMI_M0_TIMING_RXDELAY_BITS |
                           QMI_M0_TIMING_CLKDIV_BITS;
    const uint32_t kNew = (1u << QMI_M0_TIMING_COOLDOWN_LSB) |
                          (2u << QMI_M0_TIMING_RXDELAY_LSB) |
                          (1u << QMI_M0_TIMING_CLKDIV_LSB);
    uint32_t timing = (qmi_hw->m[0].timing & ~kMask) | kNew;

    uint32_t irq_save = save_and_disable_interrupts();
    hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
    qmi_hw->m[0].timing = timing;
    hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
    __asm volatile("dsb sy" ::: "memory");
    restore_interrupts(irq_save);
}

extern "C" void initVariant()
{
    /* ── P2-13 fix: re-enable XIP cache ────────────────────────────────────
     * The RP2350 ROM and/or PSRAM init clear the XIP_CTRL EN_SECURE /
     * EN_NONSECURE bits, disabling the 4 KB XIP cache.  Without cache,
     * every instruction fetch goes to QSPI flash at 37.5 MHz, making
     * getFromRadio() ~100× slower than expected.  Re-enable both bits
     * via the atomic SET alias so we don't disturb other XIP_CTRL fields
     * (e.g. WRITABLE_M1 set by psram_init). */
    *reinterpret_cast<volatile uint32_t *>(0x400CA000u) = 0x00000003u;

    /* ── P2-16 fix: retime flash from bootrom CLKDIV=3 (25 MHz) to
     * CLKDIV=1 (75 MHz).  Two parts:
     *
     *   1. Enable SLEWFAST on the QSPI_SCLK pad via the atomic SET alias.
     *      Bringup ablation (flash_pad_ablation) showed this is the ONE
     *      necessary pad change — DRIVE (4 mA default) and SD SCHMITT
     *      don't affect pass/fail or throughput.  Slow slew eats ~3-5 ns
     *      of the 13.3 ns period at 75 MHz, leaving no sampling margin;
     *      fast slew shrinks the edge to ~1-2 ns.
     *
     *   2. Reprogram M0.timing with CLKDIV=1, RXDELAY=2 via the RAM-
     *      resident helper above.
     *
     * Combined effect on cached flash read: 20.5 MB/s → 44 MB/s (+117%).
     * Instruction fetch latency for Core 0 Meshtastic likewise improves,
     * and Core 1 (launched later in this function) inherits the new
     * timing because it shares the same QMI M0 window for flash XIP. */
    /* P2-16 investigation: NOT shipped at CLKDIV=1. Summary:
     *
     *  - Bringup flash_deep_ablation (XOR-verified 16 MB at 4 pad
     *    configs, all at CLKDIV=1+RXDELAY=2) confirms DRIVE=8 mA +
     *    SLEWFAST=1 is the minimum pad set: 0/256 bad blocks.
     *    The earlier 2³ flash_pad_ablation that said "SLEWFAST alone"
     *    was a false positive from a one-word sentinel check.
     *
     *  - Enabling DRIVE=8mA + SLEWFAST=1 + CLKDIV=1 in production
     *    *still* HardFaults inside FreeRTOS vStartFirstTask on boot,
     *    despite bringup showing clean 16 MB reads under the same
     *    register state. Suspect probabilistic / workload-dependent
     *    failure: bringup single-pass deep_scan passes, but Meshtastic
     *    boot path with concurrent Core 0 fetches + Core 1 launch
     *    fetches occasionally drops a bit. The XOR oracle can miss
     *    transient errors since baseline and CLKDIV=1 both may have
     *    identical-but-wrong values for some cache line on any given
     *    read.
     *
     *  - W25Q128JW is the 1.8 V variant. Forum reports on other
     *    1.8 V custom boards (RP2040) had to drop to CLKDIV=4 for
     *    reliability. Pico 2 ships with 3.3 V W25Q32RV and Pimoroni
     *    Pico Plus 2 with 3.3 V W25Q128JV, both at CLKDIV=2 / 75 MHz.
     *
     * Decision: leave flash at the bootrom default (CLKDIV=3 / 25 MHz,
     * no pad changes) until Rev B, which should use the 3.3 V JV
     * variant or improved QSPI routing. The `flash_retime_clkdiv1`
     * helper is left in the source unused so the bring-up toolkit +
     * this comment can be revisited once the hardware allows. */
    (void)flash_retime_clkdiv1;  /* keep symbol for future use */

    /* ── P2-7 fix: MSP stack overflow guard ────────────────────────────────
     * Core 0 MSP starts at 0x20082000 (top of SCRATCH_Y) and grows down
     * through SCRATCH_Y + SCRATCH_X (8 KB total, ending at 0x20080000).
     * Below that lies the shared IPC region (0x2007A000..0x20080000).
     *
     * Set the Cortex-M33 MSPLIM register to the bottom of SCRATCH_X so
     * any MSP push past 0x20080000 triggers a UsageFault (STKOF) instead
     * of silently corrupting the IPC ring or breadcrumb area.
     *
     * Enable UsageFault + MemManage so overflow faults are reported at
     * their own priority level (easier to diagnose via SWD than a generic
     * HardFault escalation). */
    __asm volatile ("MSR msplim, %0" : : "r" (0x20080000u));
    scb_hw->shcsr |= M33_SHCSR_USGFAULTENA_BITS | M33_SHCSR_MEMFAULTENA_BITS;

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
