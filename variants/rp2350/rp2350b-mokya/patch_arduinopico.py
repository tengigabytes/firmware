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
