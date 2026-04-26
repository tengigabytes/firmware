/* ipc_command_handler.cpp — Dispatch structured IPC commands from Core 1.
 *
 * Called by IpcSerialStream::refill_rx_() in ipc_serial_stub.cpp whenever
 * a slot pops with a non-SERIAL_BYTES msg_id. M5 Phase 2 covers only
 * IPC_CMD_SEND_TEXT (Core 1 → Core 0 outbound text); other CMD ids are
 * reserved in ipc_protocol.h and dropped silently here until they get
 * implemented.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include "configuration.h"
#include "MeshService.h"
#include "Router.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

#include "ipc_protocol.h"

/* Build a TEXT_MESSAGE_APP MeshPacket from an IpcPayloadText payload and
 * hand it off to MeshService for tx. Mirrors the canned-message send
 * path in InkHUD MenuApplet (sendText) and the Serial module's send. */
static void handle_send_text(const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len < sizeof(IpcPayloadText)) return;

    const IpcPayloadText *t =
        reinterpret_cast<const IpcPayloadText *>(payload);

    /* Length sanity: trust the smaller of the framed payload size and the
     * length stored inside the payload itself. */
    const uint16_t header_size = (uint16_t)offsetof(IpcPayloadText, text);
    if (payload_len <= header_size) return;
    uint16_t text_in_payload = payload_len - header_size;
    uint16_t text_len = (t->text_len < text_in_payload)
                            ? t->text_len
                            : text_in_payload;
    if (text_len == 0) return;
    if (text_len > meshtastic_Constants_DATA_PAYLOAD_LEN) {
        text_len = meshtastic_Constants_DATA_PAYLOAD_LEN;
    }

    if (!router || !service) {
        LOG_WARN("IPC_CMD_SEND_TEXT before router/service ready, dropping");
        return;
    }

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) {
        LOG_WARN("IPC_CMD_SEND_TEXT: allocForSending() returned null");
        return;
    }

    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to       = t->to_node_id;
    p->channel  = t->channel_index;
    p->want_ack = (t->want_ack != 0u);
    p->decoded.payload.size = text_len;
    memcpy(p->decoded.payload.bytes, t->text, text_len);

    LOG_INFO("IPC SEND_TEXT id=%u to=0x%08x ch=%u len=%u",
             p->id, (unsigned)p->to, (unsigned)p->channel,
             (unsigned)text_len);

    service->sendToMesh(p, RX_SRC_LOCAL, /*ccToPhone=*/true);
}

extern "C" void mokya_handle_ipc_command(uint8_t msg_id,
                                         const uint8_t *payload,
                                         uint16_t payload_len)
{
    switch (msg_id) {
        case IPC_CMD_SEND_TEXT:
            handle_send_text(payload, payload_len);
            return;
        case IPC_MSG_LOG_LINE:
            /* Log lines from Core 1 historically went onto the same DATA
             * ring during M1 development; production drops them — they
             * arrive on the LOG ring. Silently ignore here. */
            return;
        default:
            /* Unknown CMD id — reserved for future M5 messages
             * (NODE_UPDATE, TX_ACK, GET/SET_CONFIG, etc.). Silently drop
             * so a future Core 1 build can speak to an older Core 0. */
            return;
    }
}
