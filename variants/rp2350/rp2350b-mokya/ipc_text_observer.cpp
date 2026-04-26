/* ipc_text_observer.cpp — Forward incoming text MeshPackets to Core 1
 * via IPC_MSG_RX_TEXT (M5 Phase 1 minimal slice).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ipc_text_observer.h"

#include <stdint.h>
#include <string.h>

#include "Observer.h"
#include "configuration.h"
#include "modules/TextMessageModule.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

#include "ipc_ringbuf.h"
#include "ipc_shared_layout.h"
#include "ipc_protocol.h"

namespace
{

class IpcTextObserver
{
  public:
    int onReceiveTextMessage(const meshtastic_MeshPacket *mp)
    {
        if (!mp) return 0;

        const meshtastic_Data &d = mp->decoded;

        /* Build IpcPayloadText. Truncate text to fit one ring slot. */
        uint8_t buf[IPC_MSG_PAYLOAD_MAX];
        IpcPayloadText *out = reinterpret_cast<IpcPayloadText *>(buf);

        const uint16_t kHdrSize = (uint16_t)offsetof(IpcPayloadText, text);
        const uint16_t kMaxText = (uint16_t)(IPC_MSG_PAYLOAD_MAX - kHdrSize);
        uint16_t text_len = d.payload.size;
        if (text_len > kMaxText) text_len = kMaxText;

        out->from_node_id  = mp->from;
        out->to_node_id    = mp->to;
        out->channel_index = (uint8_t)mp->channel;
        out->want_ack      = mp->want_ack ? 1u : 0u;
        out->text_len      = text_len;
        if (text_len > 0) {
            memcpy(out->text, d.payload.bytes, text_len);
        }

        const uint16_t total = (uint16_t)(kHdrSize + text_len);

        /* Best-effort: if ring is full, the byte-bridge already saturated
         * Core 1; there's no point blocking here on a UI-only path. */
        (void)ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                            g_ipc_shared.c0_to_c1_slots,
                            IPC_RING_SLOT_COUNT,
                            IPC_MSG_RX_TEXT,
                            (uint8_t)(mp->id & 0xFFu),
                            buf,
                            total);
        return 0;
    }

    CallbackObserver<IpcTextObserver, const meshtastic_MeshPacket *> textMessageObserver =
        CallbackObserver<IpcTextObserver, const meshtastic_MeshPacket *>(this,
                                                                          &IpcTextObserver::onReceiveTextMessage);
};

IpcTextObserver s_text_observer;

} // namespace

extern "C" void mokya_register_ack_observer(void);

extern "C" void mokya_register_ipc_observers(void)
{
    static bool registered = false;
    if (registered) return;
    registered = true;

    if (textMessageModule) {
        s_text_observer.textMessageObserver.observe(textMessageModule);
    }

    /* TX-ACK observer is a MeshModule subclass that auto-registers in
     * meshModulesAvailable on construction. Allocated via new() because
     * meshModulesAvailable holds raw pointers and we need it to live
     * for the lifetime of the program. */
    mokya_register_ack_observer();
}
