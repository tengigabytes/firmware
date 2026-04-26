/* ipc_gps_stream.cpp — IpcGpsBuf → Arduino Stream bridge for Meshtastic GPS.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ipc_gps_stream.h"

#include <stdint.h>

#include "ipc_shared_layout.h"
#include "ipc_protocol.h"

IpcGpsStream::IpcGpsStream()
    : last_seen_idx(255u), read_pos(0u), latched_idx(0u)
{
}

IpcGpsStream &IpcGpsStream::instance()
{
    static IpcGpsStream s_instance;
    return s_instance;
}

void IpcGpsStream::refreshLatch()
{
    /* Reader convention from IpcGpsBuf: write_idx is the slot Core 1 is
     * currently writing into, so the most recently *published* slot is
     * write_idx ^ 1. Latch when the publisher has flipped since our last
     * latch — that gives us a stable snapshot of one full sentence. */
    uint8_t cur = __atomic_load_n(&g_ipc_shared.gps_buf.write_idx, __ATOMIC_ACQUIRE);
    if (cur != last_seen_idx) {
        last_seen_idx = cur;
        latched_idx   = (uint8_t)(cur ^ 1u);
        read_pos      = 0u;
    }
}

int IpcGpsStream::available()
{
    refreshLatch();
    uint8_t len = g_ipc_shared.gps_buf.len[latched_idx];
    return (read_pos < len) ? (int)(len - read_pos) : 0;
}

int IpcGpsStream::read()
{
    refreshLatch();
    uint8_t len = g_ipc_shared.gps_buf.len[latched_idx];
    if (read_pos >= len) return -1;
    return (int)g_ipc_shared.gps_buf.buf[latched_idx][read_pos++];
}

int IpcGpsStream::peek()
{
    refreshLatch();
    uint8_t len = g_ipc_shared.gps_buf.len[latched_idx];
    if (read_pos >= len) return -1;
    return (int)g_ipc_shared.gps_buf.buf[latched_idx][read_pos];
}

size_t IpcGpsStream::write(uint8_t /*b*/)
{
    /* IpcGpsBuf is one-way (Core 1 → Core 0). Meshtastic's GPS probe and
     * chip-config writes are no-ops here; the createGps() hook forces
     * tx_gpio = 0 so probe() never runs, but harmless write paths (e.g.
     * GPS::up() / down() commands sent before fix is acquired) still go
     * through here and silently drop. */
    return 1;
}

size_t IpcGpsStream::write(const uint8_t * /*buffer*/, size_t size)
{
    return size;
}
