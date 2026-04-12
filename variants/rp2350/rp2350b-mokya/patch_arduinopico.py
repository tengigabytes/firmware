# Phase 2 M1.0 — patch Arduino-Pico framework headers and sources so the
# MokyaLora Core 0 Meshtastic build can use
#
#   1. `-DNO_USB`                    (Core 1 owns the real USB CDC)
#   2. `-DconfigNUMBER_OF_CORES=1`   (single-core FreeRTOS on Core 0 only;
#                                     Core 1 is launched separately by the
#                                     M1.0b boot spike as an Apache-2.0 image)
#
# Arduino-Pico 5.4.4 ships two pieces that break under these flags:
#
#   a) `cores/rp2040/SerialUSB.h` declares `extern SerialUSB Serial;`
#      unconditionally, even though the matching `SerialUSB.cpp` is wrapped
#      in `#if !defined(USE_TINYUSB) && !defined(NO_USB)`. We want to provide
#      our own `IpcSerialStream Serial;`, so the extern must be hidden under
#      `NO_USB`.
#
#   b) `cores/rp2040/freertos/freertos-main.cpp` unconditionally calls the
#      SMP-only FreeRTOS APIs `vTaskCoreAffinitySet`, `vTaskPreemptionDisable`,
#      and `vTaskPreemptionEnable`, which don't exist in single-core builds.
#      We guard every call site with `#if configNUMBER_OF_CORES > 1`.
#
# Strategy:
#   Rewrite the headers/sources in place, each patched block bracketed by a
#   marker comment so the patches are idempotent across rebuilds. The
#   `platform_packages` pin in platformio.ini locks the framework version, so
#   the target lines are stable for this build.
#
# License: MIT.

import os

Import("env")  # noqa: F821 — provided by PlatformIO


# --------------------------------------------------------------------- (a)
NO_USB_MARKER = "// MOKYA_NO_USB_PATCH"
SERIALUSB_TARGET = "extern SerialUSB Serial;"
SERIALUSB_REPLACEMENT = (
    "#if !defined(NO_USB)  " + NO_USB_MARKER + "\n"
    "extern SerialUSB Serial;\n"
    "#endif  " + NO_USB_MARKER
)


def _patch_serialusb_header(fw_dir):
    header = os.path.join(fw_dir, "cores", "rp2040", "SerialUSB.h")
    if not os.path.isfile(header):
        print("[mokya-patch] %s not found; skipping" % header)
        return
    with open(header, "r", encoding="utf-8") as f:
        content = f.read()
    if NO_USB_MARKER in content:
        print("[mokya-patch] SerialUSB.h already patched")
        return
    if SERIALUSB_TARGET not in content:
        print("[mokya-patch] WARNING: SerialUSB.h target line not found; "
              "framework version may have drifted. Patch NOT applied.")
        return
    content = content.replace(SERIALUSB_TARGET, SERIALUSB_REPLACEMENT)
    with open(header, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] SerialUSB.h patched: extern guarded by !NO_USB")


# --------------------------------------------------------------------- (b)
SMP_MARKER = "// MOKYA_SMP_PATCH"

# Each tuple is (target_exact_line, replacement_block). The replacement block
# wraps the SMP-only API call in `#if configNUMBER_OF_CORES > 1`.
FREERTOS_PATCHES = [
    (
        "        vTaskCoreAffinitySet(c1, 1 << 1);",
        "#if configNUMBER_OF_CORES > 1  " + SMP_MARKER + "\n"
        "        vTaskCoreAffinitySet(c1, 1 << 1);\n"
        "#endif  " + SMP_MARKER,
    ),
    (
        "        vTaskPreemptionDisable(nullptr);\n"
        "        portDISABLE_INTERRUPTS();\n"
        "        __otherCoreIdled = true;\n"
        "        while (__otherCoreIdled) {\n"
        "            /* noop */\n"
        "        }\n"
        "        portENABLE_INTERRUPTS();\n"
        "        vTaskPreemptionEnable(nullptr);",
        "#if configNUMBER_OF_CORES > 1  " + SMP_MARKER + "\n"
        "        vTaskPreemptionDisable(nullptr);\n"
        "        portDISABLE_INTERRUPTS();\n"
        "        __otherCoreIdled = true;\n"
        "        while (__otherCoreIdled) {\n"
        "            /* noop */\n"
        "        }\n"
        "        portENABLE_INTERRUPTS();\n"
        "        vTaskPreemptionEnable(nullptr);\n"
        "#endif  " + SMP_MARKER,
    ),
    (
        "extern \"C\" void __no_inline_not_in_flash_func(__freertos_idle_other_core)() {\n"
        "    vTaskPreemptionDisable(nullptr);\n"
        "    xTaskNotifyGive(__idleCoreTask[ 1 ^ sio_hw->cpuid ]);\n"
        "    while (!__otherCoreIdled) {\n"
        "        /* noop */\n"
        "    }\n"
        "    portDISABLE_INTERRUPTS();\n"
        "    vTaskSuspendAll();\n"
        "}",
        "extern \"C\" void __no_inline_not_in_flash_func(__freertos_idle_other_core)() {\n"
        "#if configNUMBER_OF_CORES > 1  " + SMP_MARKER + "\n"
        "    vTaskPreemptionDisable(nullptr);\n"
        "    xTaskNotifyGive(__idleCoreTask[ 1 ^ sio_hw->cpuid ]);\n"
        "    while (!__otherCoreIdled) {\n"
        "        /* noop */\n"
        "    }\n"
        "    portDISABLE_INTERRUPTS();\n"
        "    vTaskSuspendAll();\n"
        "#endif  " + SMP_MARKER + "\n"
        "}",
    ),
    (
        "extern \"C\" void __no_inline_not_in_flash_func(__freertos_resume_other_core)() {\n"
        "    __otherCoreIdled = false;\n"
        "    portENABLE_INTERRUPTS();\n"
        "    xTaskResumeAll();\n"
        "    vTaskPreemptionEnable(nullptr);\n"
        "}",
        "extern \"C\" void __no_inline_not_in_flash_func(__freertos_resume_other_core)() {\n"
        "#if configNUMBER_OF_CORES > 1  " + SMP_MARKER + "\n"
        "    __otherCoreIdled = false;\n"
        "    portENABLE_INTERRUPTS();\n"
        "    xTaskResumeAll();\n"
        "    vTaskPreemptionEnable(nullptr);\n"
        "#endif  " + SMP_MARKER + "\n"
        "}",
    ),
    (
        "    vTaskCoreAffinitySet(c0, 1 << 0);\n"
        "\n"
        "    // Create the idle-other-core tasks (for when flash is being written)\n"
        "    xTaskCreate(IdleThisCore, \"IdleCore0\", 128, 0, configMAX_PRIORITIES - 1, __idleCoreTask + 0);\n"
        "    vTaskCoreAffinitySet(__idleCoreTask[0], 1 << 0);\n"
        "    xTaskCreate(IdleThisCore, \"IdleCore1\", 128, 0, configMAX_PRIORITIES - 1, __idleCoreTask + 1);\n"
        "    vTaskCoreAffinitySet(__idleCoreTask[1], 1 << 1);",
        "#if configNUMBER_OF_CORES > 1  " + SMP_MARKER + "\n"
        "    vTaskCoreAffinitySet(c0, 1 << 0);\n"
        "\n"
        "    // Create the idle-other-core tasks (for when flash is being written)\n"
        "    xTaskCreate(IdleThisCore, \"IdleCore0\", 128, 0, configMAX_PRIORITIES - 1, __idleCoreTask + 0);\n"
        "    vTaskCoreAffinitySet(__idleCoreTask[0], 1 << 0);\n"
        "    xTaskCreate(IdleThisCore, \"IdleCore1\", 128, 0, configMAX_PRIORITIES - 1, __idleCoreTask + 1);\n"
        "    vTaskCoreAffinitySet(__idleCoreTask[1], 1 << 1);\n"
        "#endif  " + SMP_MARKER,
    ),
]


def _patch_freertos_main(fw_dir):
    src = os.path.join(fw_dir, "cores", "rp2040", "freertos", "freertos-main.cpp")
    if not os.path.isfile(src):
        print("[mokya-patch] %s not found; skipping" % src)
        return
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    if SMP_MARKER in content:
        print("[mokya-patch] freertos-main.cpp already patched")
        return
    missing = [i for i, (tgt, _) in enumerate(FREERTOS_PATCHES) if tgt not in content]
    if missing:
        print("[mokya-patch] WARNING: freertos-main.cpp patch targets %s not "
              "found; framework version may have drifted. Patch NOT applied."
              % missing)
        return
    for tgt, repl in FREERTOS_PATCHES:
        content = content.replace(tgt, repl, 1)
    with open(src, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] freertos-main.cpp patched: %d SMP call sites guarded"
          % len(FREERTOS_PATCHES))


PORTC_PATCHES = [
    # In single-core mode port.c declares ulCriticalNesting as `static`, so
    # Arduino-Pico's wiring_private.cpp (which uses
    # portGET_CRITICAL_NESTING_COUNT from portmacro.h) cannot link to it.
    # The port clearly intended this macro to be file-local in single-core
    # mode, but Arduino-Pico assumes global visibility. Remove the static
    # qualifier so the symbol has external linkage.
    (
        "#if ( configNUMBER_OF_CORES == 1 )\n"
        "PRIVILEGED_DATA static volatile uint32_t ulCriticalNesting = 0xaaaaaaaaUL;",
        "#if ( configNUMBER_OF_CORES == 1 )  " + SMP_MARKER + "\n"
        "PRIVILEGED_DATA volatile uint32_t ulCriticalNesting = 0xaaaaaaaaUL;",
    ),
]


def _patch_portc(fw_dir):
    src = os.path.join(fw_dir, "FreeRTOS-Kernel", "portable", "ThirdParty",
                       "GCC", "RP2350_ARM_NTZ", "non_secure", "port.c")
    if not os.path.isfile(src):
        print("[mokya-patch] %s not found; skipping" % src)
        return
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    if SMP_MARKER in content:
        print("[mokya-patch] port.c already patched")
        return
    missing = [i for i, (tgt, _) in enumerate(PORTC_PATCHES) if tgt not in content]
    if missing:
        print("[mokya-patch] WARNING: port.c patch targets %s not "
              "found; framework version may have drifted. Patch NOT applied."
              % missing)
        return
    for tgt, repl in PORTC_PATCHES:
        content = content.replace(tgt, repl, 1)
    with open(src, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] port.c patched: ulCriticalNesting promoted to extern")


# --------------------------------------------------------------------- (d2)
# Phase 2 M2.0 — disable FreeRTOS's pico_sync_interop doorbell ISR registration
# in the single-core `xPortStartScheduler` path.
#
# Problem: with `configSUPPORT_PICO_SYNC_INTEROP == 1` the RP2350_ARM_NTZ port
# claims a doorbell and registers `prvDoorbellInterruptHandler` on SIO_IRQ_BELL.
# Core 1 (our bridge) fires doorbell 0 to signal new c1→c0 data. Because
# SIO_IRQ_BELL fires for *any* doorbell, the FreeRTOS ISR re-enters endlessly
# (if the claimed doorbell differs from 0) or deadlocks on
# `spin_lock_blocking(pxCrossCoreSpinLock)` (if it matches). Either way Core 0
# is stuck and setup() never reaches consoleInit().
#
# MokyaLora does not use pico-sdk cross-core sync primitives (our IPC is via the
# SPSC ring), so disabling the registration is safe. For M2 we will install our
# own doorbell handler on IPC_DOORBELL_NUM.
DOORBELL_MARKER = "// MOKYA_DOORBELL_PATCH"

PORTC_DOORBELL_PATCHES = [
    (
        "        #if ( LIB_PICO_MULTICORE == 1 )\n"
        "            #if ( configSUPPORT_PICO_SYNC_INTEROP == 1 )\n"
        "                // claim same number of both cores for simplicity\n"
        "                cDoorbellNum = (int8_t) multicore_doorbell_claim_unused(0b11, true);\n"
        "                multicore_doorbell_clear_current_core(cDoorbellNum);\n"
        "                multicore_doorbell_clear_other_core(cDoorbellNum);\n"
        "                uint32_t irq_num = multicore_doorbell_irq_num(cDoorbellNum);\n"
        "                irq_set_priority( irq_num, portMIN_INTERRUPT_PRIORITY );\n"
        "                irq_set_exclusive_handler( irq_num, prvDoorbellInterruptHandler );\n"
        "                irq_set_enabled( irq_num, 1 );\n"
        "            #endif\n"
        "        #endif",
        "#if 0  " + DOORBELL_MARKER + "\n"
        "        #if ( LIB_PICO_MULTICORE == 1 )\n"
        "            #if ( configSUPPORT_PICO_SYNC_INTEROP == 1 )\n"
        "                // claim same number of both cores for simplicity\n"
        "                cDoorbellNum = (int8_t) multicore_doorbell_claim_unused(0b11, true);\n"
        "                multicore_doorbell_clear_current_core(cDoorbellNum);\n"
        "                multicore_doorbell_clear_other_core(cDoorbellNum);\n"
        "                uint32_t irq_num = multicore_doorbell_irq_num(cDoorbellNum);\n"
        "                irq_set_priority( irq_num, portMIN_INTERRUPT_PRIORITY );\n"
        "                irq_set_exclusive_handler( irq_num, prvDoorbellInterruptHandler );\n"
        "                irq_set_enabled( irq_num, 1 );\n"
        "            #endif\n"
        "        #endif\n"
        "#endif  " + DOORBELL_MARKER,
    ),
]


def _patch_portc_doorbell(fw_dir):
    src = os.path.join(fw_dir, "FreeRTOS-Kernel", "portable", "ThirdParty",
                       "GCC", "RP2350_ARM_NTZ", "non_secure", "port.c")
    if not os.path.isfile(src):
        print("[mokya-patch] %s not found; skipping" % src)
        return
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    if DOORBELL_MARKER in content:
        print("[mokya-patch] port.c doorbell already patched")
        return
    missing = [i for i, (tgt, _) in enumerate(PORTC_DOORBELL_PATCHES) if tgt not in content]
    if missing:
        print("[mokya-patch] WARNING: port.c doorbell patch targets %s not "
              "found; framework version may have drifted. Patch NOT applied."
              % missing)
        return
    for tgt, repl in PORTC_DOORBELL_PATCHES:
        content = content.replace(tgt, repl, 1)
    with open(src, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] port.c patched: pico_sync_interop doorbell ISR disabled")


PORTMACRO_PATCHES = [
    # In single-core mode the RP2350 port.c defines `volatile uint32_t
    # ulCriticalNesting = 0;` but portmacro.h only declares the SMP-array
    # variant as extern. Any framework TU that evaluates
    # `portGET_CRITICAL_NESTING_COUNT()` (e.g. wiring_private.cpp) then fails
    # with "ulCriticalNesting was not declared in this scope". Add the
    # missing extern declaration alongside the single-core macro.
    (
        "#if configNUMBER_OF_CORES == 1\n"
        "#define portGET_CRITICAL_NESTING_COUNT()          ulCriticalNesting",
        "#if configNUMBER_OF_CORES == 1  " + SMP_MARKER + "\n"
        "extern volatile uint32_t ulCriticalNesting;\n"
        "#define portGET_CRITICAL_NESTING_COUNT()          ulCriticalNesting",
    ),
]


def _patch_portmacro(fw_dir):
    src = os.path.join(fw_dir, "FreeRTOS-Kernel", "portable", "ThirdParty",
                       "GCC", "RP2350_ARM_NTZ", "non_secure", "portmacro.h")
    if not os.path.isfile(src):
        print("[mokya-patch] %s not found; skipping" % src)
        return
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    if SMP_MARKER in content:
        print("[mokya-patch] portmacro.h already patched")
        return
    missing = [i for i, (tgt, _) in enumerate(PORTMACRO_PATCHES) if tgt not in content]
    if missing:
        print("[mokya-patch] WARNING: portmacro.h patch targets %s not "
              "found; framework version may have drifted. Patch NOT applied."
              % missing)
        return
    for tgt, repl in PORTMACRO_PATCHES:
        content = content.replace(tgt, repl, 1)
    with open(src, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] portmacro.h patched: %d extern decls added"
          % len(PORTMACRO_PATCHES))


FREERTOS_LWIP_PATCHES = [
    (
        "    vTaskCoreAffinitySet(__lwipTask, 1 << 0);",
        "#if configNUMBER_OF_CORES > 1  " + SMP_MARKER + "\n"
        "    vTaskCoreAffinitySet(__lwipTask, 1 << 0);\n"
        "#endif  " + SMP_MARKER,
    ),
]


def _patch_freertos_lwip(fw_dir):
    src = os.path.join(fw_dir, "cores", "rp2040", "freertos", "freertos-lwip.cpp")
    if not os.path.isfile(src):
        print("[mokya-patch] %s not found; skipping" % src)
        return
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    if SMP_MARKER in content:
        print("[mokya-patch] freertos-lwip.cpp already patched")
        return
    missing = [i for i, (tgt, _) in enumerate(FREERTOS_LWIP_PATCHES) if tgt not in content]
    if missing:
        print("[mokya-patch] WARNING: freertos-lwip.cpp patch targets %s not "
              "found; framework version may have drifted. Patch NOT applied."
              % missing)
        return
    for tgt, repl in FREERTOS_LWIP_PATCHES:
        content = content.replace(tgt, repl, 1)
    with open(src, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] freertos-lwip.cpp patched: %d SMP call sites guarded"
          % len(FREERTOS_LWIP_PATCHES))


# --------------------------------------------------------------------- (c)
# Phase 2 M1.1 — reserve a fixed 24 KB SHARED_IPC region at the top of main
# SRAM so both cores can agree on the address of g_ipc_shared without any
# linker symbol exchange. The Arduino-Pico default ld gives Core 0 a .heap
# section that grows to ORIGIN(RAM)+LENGTH(RAM) = 0x20080000, so we have to
# shrink the RAM region AND add a matching NOLOAD section that lives in a
# separate MEMORY region. The section is populated by ipc_ringbuf.c's
# `__attribute__((section(".shared_ipc"))) g_ipc_shared` definition.
LD_MARKER = "/* MOKYA_SHARED_IPC_PATCH */"

LD_MEMORY_TARGET = (
    "    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = __RAM_LENGTH__\n"
    "    SCRATCH_X(rwx) : ORIGIN = 0x20080000, LENGTH = 4k"
)
LD_MEMORY_REPLACEMENT = (
    "    RAM(rwx) : ORIGIN =  0x20000000, LENGTH = __RAM_LENGTH__ - 0x14000  "
    + LD_MARKER
    + "\n"
    "    SHARED_IPC(rw) : ORIGIN = 0x2007A000, LENGTH = 0x6000  "
    + LD_MARKER
    + "\n"
    "    SCRATCH_X(rwx) : ORIGIN = 0x20080000, LENGTH = 4k"
)

# Place the .shared_ipc NOLOAD section just before .scratch_x so it is
# guaranteed to be after .bss / .heap in the link order. Both symbols are
# resolved at absolute addresses via MEMORY > SHARED_IPC, so the textual
# position only affects diagnostics, not layout.
LD_SECTION_TARGET = (
    "    /* Start and end symbols must be word-aligned */\n"
    "    .scratch_x : {"
)
LD_SECTION_REPLACEMENT = (
    "    .shared_ipc (NOLOAD) : {  " + LD_MARKER + "\n"
    "        __shared_ipc_start = .;\n"
    "        KEEP(*(.shared_ipc))\n"
    "        __shared_ipc_end = .;\n"
    "    } > SHARED_IPC\n"
    "\n"
    "    /* Start and end symbols must be word-aligned */\n"
    "    .scratch_x : {"
)


def _patch_memmap_ld(fw_dir):
    src = os.path.join(fw_dir, "lib", "rp2350", "memmap_default.ld")
    if not os.path.isfile(src):
        print("[mokya-patch] %s not found; skipping" % src)
        return
    with open(src, "r", encoding="utf-8") as f:
        content = f.read()
    if LD_MARKER in content:
        print("[mokya-patch] memmap_default.ld already patched")
        return
    if LD_MEMORY_TARGET not in content or LD_SECTION_TARGET not in content:
        print("[mokya-patch] WARNING: memmap_default.ld patch targets not "
              "found; framework version may have drifted. Patch NOT applied.")
        return
    content = content.replace(LD_MEMORY_TARGET, LD_MEMORY_REPLACEMENT, 1)
    content = content.replace(LD_SECTION_TARGET, LD_SECTION_REPLACEMENT, 1)
    with open(src, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] memmap_default.ld patched: SHARED_IPC region "
          "+ .shared_ipc NOLOAD section at 0x2007A000 (24 KB)")


# --------------------------------------------------------------------- run
fw_dir = env.PioPlatform().get_package_dir("framework-arduinopico")  # noqa: F821
if not fw_dir:
    print("[mokya-patch] framework-arduinopico not installed; skipping all patches")
else:
    _patch_serialusb_header(fw_dir)
    _patch_freertos_main(fw_dir)
    _patch_freertos_lwip(fw_dir)
    _patch_portmacro(fw_dir)
    _patch_portc(fw_dir)
    _patch_portc_doorbell(fw_dir)
    _patch_memmap_ld(fw_dir)
