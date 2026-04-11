# Phase 2 M1.0 — patch Arduino-Pico SerialUSB.h so that `NO_USB` also hides
# the `extern SerialUSB Serial;` declaration at the bottom of the header.
#
# Why this is needed:
#   Arduino-Pico 5.4.4 wraps `SerialUSB.cpp` in `#if !defined(USE_TINYUSB)
#   && !defined(NO_USB)`, so with -DNO_USB the implementation vanishes, but
#   `SerialUSB.h` still unconditionally declares `extern SerialUSB Serial;`.
#   MokyaLora Core 0 defines its own `IpcSerialStream Serial;` to pipe bytes
#   into the shared-SRAM SPSC ring for Core 1's USB bridge, and the mismatched
#   declaration breaks compilation with "conflicting declaration".
#
# Strategy:
#   Rewrite the header in place, wrapping the extern in `#if !defined(NO_USB)`.
#   A marker comment makes the patch idempotent across rebuilds and safe when
#   `framework-arduinopico` is reinstalled (platform_packages is pinned, so
#   the target line is stable for this build).
#
# License: MIT.

import os

Import("env")  # noqa: F821 — provided by PlatformIO

MARKER = "// MOKYA_NO_USB_PATCH"
TARGET_LINE = "extern SerialUSB Serial;"
REPLACEMENT = (
    "#if !defined(NO_USB)  " + MARKER + "\n"
    "extern SerialUSB Serial;\n"
    "#endif  " + MARKER
)


def _patch_serialusb_header():
    fw_dir = env.PioPlatform().get_package_dir("framework-arduinopico")  # noqa: F821
    if not fw_dir:
        print("[mokya-patch] framework-arduinopico not installed; skipping")
        return
    header = os.path.join(fw_dir, "cores", "rp2040", "SerialUSB.h")
    if not os.path.isfile(header):
        print("[mokya-patch] %s not found; skipping" % header)
        return
    with open(header, "r", encoding="utf-8") as f:
        content = f.read()
    if MARKER in content:
        print("[mokya-patch] SerialUSB.h already patched, skipping")
        return
    if TARGET_LINE not in content:
        print("[mokya-patch] WARNING: target line not found in SerialUSB.h; "
              "framework version may have drifted. Patch NOT applied.")
        return
    content = content.replace(TARGET_LINE, REPLACEMENT)
    with open(header, "w", encoding="utf-8") as f:
        f.write(content)
    print("[mokya-patch] SerialUSB.h patched: `extern SerialUSB Serial;` "
          "now guarded by !NO_USB")


_patch_serialusb_header()
