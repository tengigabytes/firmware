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

#if defined(MOKYA_IPC_GPS_STREAM)
#include "GPS.h"
#include "ipc_gps_stream.h"
#endif

/* Pico SDK RP2350 — provides scb_hw (armv8m_scb_hw_t) and M33_SHCSR_* bit defs. */
#include "hardware/structs/scb.h"
#include "hardware/structs/qmi.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/watchdog.h"
#include "hardware/sync.h"

/* POC: optional Core-0-only watchdog reset trigger (poc_c0_reset.cpp). */
extern "C" void mokya_poc_c0_reset_arm(void);
/* hardware/xip_cache.h isn't on Arduino-Pico's variant include path, so we
 * open-code the RP2350 invalidate-by-set-way sequence in
 * mokya_xip_cache_invalidate_all() below. */

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

/* RP2350 XIP cache invalidate-by-set-way, open-coded because
 * hardware/xip_cache.h isn't on Arduino-Pico's variant include path.
 * Byte-equivalent to pico-sdk xip_cache_invalidate_all():
 *   - XIP_MAINTENANCE_BASE = 0x18000000
 *   - XIP_END  - XIP_BASE  = 0x04000000  (64 MB XIP address space)
 *   - XIP_CACHE_SIZE       = 0x00004000  (16 KB)
 *   - XIP_CACHE_LINE_SIZE  = 8
 *   - op = XIP_CACHE_INVALIDATE_BY_SET_WAY = 0
 *
 * The range-start at (XIP_CACHE_ADDRESS_SPACE_SIZE - XIP_CACHE_SIZE)
 * mirrors the SDK's choice — keeps the maintenance addresses outside
 * the downstream QMI range, matching the clean_all RP2350-E11 hint.
 */
static void __no_inline_not_in_flash_func(mokya_xip_cache_invalidate_all)(void)
{
    constexpr uintptr_t kMaintBase  = 0x18000000u;
    constexpr uintptr_t kAddrSpace  = 0x04000000u; /* XIP_END - XIP_BASE */
    constexpr uintptr_t kCacheSize  = 0x00004000u; /* 16 KB */
    constexpr uintptr_t kLineSize   = 0x00000008u;
    constexpr uintptr_t kOpInvalSW  = 0u;

    uintptr_t start = kAddrSpace - kCacheSize;
    uintptr_t end   = kAddrSpace;
    for (uintptr_t off = start; off < end; off += kLineSize) {
        *reinterpret_cast<volatile uint8_t *>(kMaintBase + off + kOpInvalSW) = 0;
    }
    __asm volatile("dsb sy" ::: "memory");
    __asm volatile("isb" ::: "memory");
}

/* P2-16: retime flash M0 from bootrom CLKDIV=3 (25 MHz) to CLKDIV=2
 * (37.5 MHz). RXDELAY=2 follows Arduino-Pico's RXDELAY=CLKDIV heuristic;
 * other M0.timing fields are preserved via the mask so we inherit the
 * bootrom's MAX_SELECT / MIN_DESELECT / PAGEBREAK / SELECT_* defaults.
 * Pads are left at bootrom defaults (4 mA, SLEWFAST=0) — CLKDIV=2 has
 * enough timing margin that no pad boost is needed.
 *
 * Must run from RAM: direct_csr.EN pauses XIP around the timing write
 * so a concurrent instruction fetch can't land on a half-applied value,
 * and the function body itself must therefore live outside flash.
 * xip_cache_invalidate_all after the switch purges any lines that were
 * populated under the old timing, so the caller's first post-return
 * fetch cold-misses cleanly at the new timing.
 *
 * Why not CLKDIV=1 / 75 MHz: bringup's linear + 100 k random cached
 * scans both show 0 errors at CLKDIV=1 + 8 mA + SLEWFAST, but a 1 M
 * random scan reveals a ~10⁻⁵ transient cache-line-fill error rate that
 * production reproducibly hits within seconds of boot (HardFault inside
 * FreeRTOS scheduler start). The 1.8 V W25Q128JW on this Rev A routing
 * simply doesn't meet setup/hold at 75 MHz reliably. Full investigation
 * in docs/bringup/phase2-log.md P2-16.
 */
static void __no_inline_not_in_flash_func(flash_retime_m0)(void)
{
    const uint32_t kMask = QMI_M0_TIMING_COOLDOWN_BITS |
                           QMI_M0_TIMING_RXDELAY_BITS |
                           QMI_M0_TIMING_CLKDIV_BITS;
    const uint32_t kNew = (1u << QMI_M0_TIMING_COOLDOWN_LSB) |
                          (2u << QMI_M0_TIMING_RXDELAY_LSB) |
                          (2u << QMI_M0_TIMING_CLKDIV_LSB);
    uint32_t timing = (qmi_hw->m[0].timing & ~kMask) | kNew;

    uint32_t irq_save = save_and_disable_interrupts();
    hw_set_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
    qmi_hw->m[0].timing = timing;
    hw_clear_bits(&qmi_hw->direct_csr, QMI_DIRECT_CSR_EN_BITS);
    mokya_xip_cache_invalidate_all();
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

    /* ── P2-16: retime flash to CLKDIV=2 / 37.5 MHz (50 % over bootrom).
     * See flash_retime_m0() above for the helper rationale and why
     * CLKDIV=1 / 75 MHz is not viable on this Rev A. */
    flash_retime_m0();

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

    /* Warm-boot detect — POC for Core-0-only watchdog reset.
     *
     * If WATCHDOG_REASON is non-zero AND Core 1 had previously published
     * c1_ready, treat this as a selective reboot of proc0 only. Core 1 is
     * still running; we must NOT touch the cross-core handshake (would
     * either wedge it via the SIO FIFO multicore_launch protocol or, worse,
     * issue multicore_reset_core1 and hard-reset the very thing we wanted
     * to keep alive).
     *
     * Read both BEFORE touching anything else, especially before
     * ipc_shared_init() which would otherwise wipe the c1_ready word we
     * need to inspect.
     */
    const uint32_t wd_reason = *reinterpret_cast<volatile uint32_t *>(
        WATCHDOG_BASE + WATCHDOG_REASON_OFFSET);
    const uint32_t c1_ready_at_boot = g_ipc_shared.c1_ready;
    const bool warm_reboot = (wd_reason != 0u) && (c1_ready_at_boot != 0u);

    /* phase 1 marker: 0x11 = cold boot, 0x11b = warm reboot (selective). */
    dbg[0] = warm_reboot ? 0x11bu : 0x11u;
    dbg[1] = 0;
    dbg[2] = 0;
    dbg[3] = 0;
    if (!warm_reboot) {
        *sentinel = 0u;
    }
    __asm volatile("dmb 0xF" ::: "memory");

    // Phase 2 M1.1: zero the shared-SRAM IPC region and publish IPC_BOOT_MAGIC
    // BEFORE launching Core 1. The bridge image spins on boot_magic until it
    // sees IPC_BOOT_MAGIC before touching any ring, so this ordering is the
    // handshake: if Core 1 ever sees bogus head/tail values it is this call
    // that failed to run.
    if (!warm_reboot) {
        ipc_shared_init();
    }
    dbg[0] = 0x11au;  // phase 1a: shared IPC zeroed + magic published

    /* Phase 1.6: install SIO_IRQ_BELL listener so Core 1's LRU-persist
     * flash writes can park Core 0 via IPC_FLASH_DOORBELL_C0. Must run
     * BEFORE Core 1 is launched so the ISR vector is in place by the time
     * Core 1's ime_task starts. Defined in flash_park_listener.c. */
    extern void mokya_flash_park_listener_init(void);
    mokya_flash_park_listener_init();

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

    if (!warm_reboot) {
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
    } else {
        // Warm path: Core 1 is already running its own image. Skip the
        // FIFO handshake (it would either deadlock waiting for replies or
        // wedge Core 1 by writing into its mailbox). Just record that we
        // observed Core 1 still alive at boot.
        dbg[3] = *sentinel;  // snapshot whatever Core 1 last wrote
        dbg[0] = 0x14bu;     // phase 4 warm: skipped handshake
    }

    // Register reboot observer — when Meshtastic calls Power::reboot(),
    // we notify Core 1 to disconnect USB before the watchdog fires.
    s_reboot_notifier.observer.observe(&notifyReboot);
    dbg[0] = 0x15u;  // phase 5: reboot observer registered

    /* Phase 1.6: publish c0_ready so Core 1's flash_safety_wrap knows the
     * SIO_IRQ_BELL listener is live and park requests will be serviced.
     * Must be the last write in initVariant() — any earlier and Core 1
     * could start requesting before our ISR was fully installed. */
    __atomic_store_n(&g_ipc_shared.c0_ready, 1u, __ATOMIC_RELEASE);
    dbg[0] = 0x16u;  // phase 6: c0_ready published

#if defined(MOKYA_IPC_GPS_STREAM)
    /* M3.5: register the IpcGpsBuf-backed Stream with Meshtastic's GPS
     * class. Must run before main.cpp setup() calls GPS::createGps(). */
    GPS::setExternalSerial(&IpcGpsStream::instance());
#endif

    /* POC: arm Core-0-only watchdog reset N seconds after boot. No-op when
     * MOKYA_POC_C0_RESET is not defined. */
    mokya_poc_c0_reset_arm();
}
