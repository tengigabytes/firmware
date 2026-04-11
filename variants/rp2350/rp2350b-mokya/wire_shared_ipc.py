# Phase 2 M1.1 — compile firmware/shared/ipc/*.c into the Core 0 Meshtastic
# build, and add its headers to the include path.
#
# Why a separate Python hook and not build_src_filter?
#   PlatformIO's `build_src_filter` paths are interpreted relative to
#   PROJECT_SRC_DIR (= firmware/core0/meshtastic/src), and `+<...>` does not
#   accept paths that reach above the project source root cleanly. shared/ipc
#   lives at firmware/shared/ipc — three directories up and in a sibling
#   subtree — so we add it as an explicit BuildSources() call that emits a
#   dedicated object-file subdirectory under $BUILD_DIR.
#
# License: MIT.

import os

Import("env")  # noqa: F821 — provided by PlatformIO


def _shared_ipc_dir(project_dir):
    # PROJECT_DIR = firmware/core0/meshtastic
    # shared/ipc  = firmware/shared/ipc      (two levels up)
    return os.path.normpath(os.path.join(project_dir, "..", "..", "shared", "ipc"))


shared_ipc = _shared_ipc_dir(env["PROJECT_DIR"])

if not os.path.isdir(shared_ipc):
    raise RuntimeError(
        "wire_shared_ipc.py: expected shared IPC directory at %s" % shared_ipc
    )

# Make <ipc_protocol.h>, <ipc_shared_layout.h>, <ipc_ringbuf.h> visible to
# every TU in the Core 0 build (Meshtastic + the rp2350b-mokya variant stub).
env.Append(CPPPATH=[shared_ipc])

# Compile shared/ipc/*.c into $BUILD_DIR/shared_ipc/... as a first-party
# source set. Using a distinct output directory keeps the object files out
# of PIO's main src tree so rebuilds and filters stay predictable.
env.BuildSources(
    os.path.join("$BUILD_DIR", "shared_ipc"),
    shared_ipc,
    src_filter=["+<*.c>"],
)
