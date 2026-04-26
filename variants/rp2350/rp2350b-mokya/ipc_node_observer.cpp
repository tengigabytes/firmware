/* ipc_node_observer.cpp — Forward NodeDB updates to Core 1 as
 * IPC_MSG_NODE_UPDATE.
 *
 * Subscribes to NodeDB::newStatus (fires after every per-node mutation
 * via NodeDB::notifyObservers(true)). On each fire, reads
 * nodeDB->updateGUIforNode — Meshtastic's "this is the node that just
 * changed" pointer — builds an IpcPayloadNodeUpdate around it, and
 * pushes it onto the c0→c1 DATA ring.
 *
 * Bootstrap: on register, walks the entire NodeDB and emits one
 * NODE_UPDATE per existing entry. Without this Core 1 only sees nodes
 * whose user-info changes after boot, missing all peers already in the
 * persistent NodeDB at startup.
 *
 * `alias` carries the node's `short_name` (4-char tag like "50ca",
 * "9c28"), trimmed to the wire payload budget. Core 1 maintains its
 * own per-node table and renders the latest snapshot in nodes_view.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include "configuration.h"
#include "Observer.h"
#include "NodeStatus.h"
#include "mesh/NodeDB.h"
#include "mesh/generated/meshtastic/deviceonly.pb.h"

#include "ipc_protocol.h"
#include "ipc_shared_layout.h"
#include "ipc_ringbuf.h"

namespace
{

static void push_node_update(const meshtastic_NodeInfoLite *n)
{
    if (!n) return;

    uint8_t buf[IPC_MSG_PAYLOAD_MAX];
    IpcPayloadNodeUpdate *out = reinterpret_cast<IpcPayloadNodeUpdate *>(buf);

    out->node_id   = n->num;
    /* RSSI isn't on NodeInfoLite directly — only SNR is stored
     * per-node in the DB. INT16_MIN = unknown for now; a future slice
     * can plumb RSSI through if needed. */
    out->rssi      = INT16_MIN;
    out->snr_x4    = (n->snr != 0.0f) ? (int8_t)(n->snr * 4.0f)
                                      : (int8_t)INT8_MIN;
    out->hops_away = n->has_hops_away ? n->hops_away : 0xFFu;

    if (n->has_position) {
        out->lat_e7 = n->position.latitude_i;
        out->lon_e7 = n->position.longitude_i;
    } else {
        out->lat_e7 = INT32_MIN;
        out->lon_e7 = INT32_MIN;
    }

    out->battery_mv = (n->has_device_metrics &&
                       n->device_metrics.has_voltage)
                          ? (uint16_t)(n->device_metrics.voltage * 1000.0f)
                          : 0u;

    /* Use short_name as the wire alias. Fall back to long_name prefix
     * when short_name is empty. */
    const char *alias_src = n->user.short_name;
    size_t alias_max = sizeof(n->user.short_name);
    if (alias_src[0] == '\0') {
        alias_src = n->user.long_name;
        alias_max = sizeof(n->user.long_name);
    }
    size_t alias_len = strnlen(alias_src, alias_max);
    if (alias_len > 16) alias_len = 16;
    out->alias_len = (uint8_t)alias_len;
    if (alias_len > 0) {
        memcpy(out->alias, alias_src, alias_len);
    }

    const uint16_t header_size = (uint16_t)offsetof(IpcPayloadNodeUpdate, alias);
    const uint16_t total = (uint16_t)(header_size + alias_len);

    (void)ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                        g_ipc_shared.c0_to_c1_slots,
                        IPC_RING_SLOT_COUNT,
                        IPC_MSG_NODE_UPDATE,
                        (uint8_t)(n->num & 0xFFu),
                        buf,
                        total);
}

class IpcNodeObserver
{
  public:
    int onNodeStatus(const meshtastic::NodeStatus *)
    {
        if (!nodeDB) return 0;
        const meshtastic_NodeInfoLite *n = nodeDB->updateGUIforNode;
        if (!n) return 0;
        push_node_update(n);
        return 0;
    }

    CallbackObserver<IpcNodeObserver, const meshtastic::NodeStatus *> nodeStatusObserver =
        CallbackObserver<IpcNodeObserver, const meshtastic::NodeStatus *>(this,
                                                                          &IpcNodeObserver::onNodeStatus);
};

IpcNodeObserver s_node_observer;

} // namespace

extern "C" void mokya_register_node_observer(void)
{
    if (!nodeDB) return;

    s_node_observer.nodeStatusObserver.observe(&nodeDB->newStatus);

    /* Bulk emit a snapshot of the persistent NodeDB. meshNodes is
     * sortMeshDB-ordered (self → favorites → most-recently-heard), so
     * the first kBulkDump entries are exactly the peers a phone-style
     * UI would want to surface. Cap the dump at the Core 1 nodes_db
     * capacity (16) — pushing more would evict our most-relevant
     * entries first. The c0_to_c1 ring is 32 slots so 16 fits with
     * headroom even if other producers are active. */
    if (nodeDB->meshNodes != nullptr) {
        constexpr size_t kBulkDump = 16;
        const auto &v = *nodeDB->meshNodes;
        const size_t n = (v.size() < kBulkDump) ? v.size() : kBulkDump;
        for (size_t i = 0; i < n; ++i) {
            push_node_update(&v[i]);
            /* delay(1) between pushes so Core 1's bridge_task gets a
             * chance to drain the slot we just wrote — keeps the ring
             * from filling on a tight burst. */
            delay(1);
        }
    }
}
