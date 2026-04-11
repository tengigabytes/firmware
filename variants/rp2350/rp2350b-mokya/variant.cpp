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
}
