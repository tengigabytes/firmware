/* ipc_command_handler.cpp — Dispatch structured IPC commands from Core 1.
 *
 * Called by IpcSerialStream::refill_rx_() in ipc_serial_stub.cpp whenever
 * a slot pops with a non-SERIAL_BYTES msg_id. M5 Phase 2/3 covers
 * IPC_CMD_SEND_TEXT (Core 1 → Core 0 outbound text) plus the matching
 * IPC_MSG_TX_ACK reply path. Other CMD ids are reserved in
 * ipc_protocol.h and dropped silently here until they get implemented.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>

#include "configuration.h"
#include "MeshService.h"
#include "Router.h"
#include "main.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

#include "ipc_protocol.h"
#include "ipc_shared_layout.h"
#include "ipc_ringbuf.h"

#include "ipc_command_handler.h"

/* Outstanding-send tracker — small FIFO so the routing-ACK observer can
 * map an inbound `request_id` (Meshtastic packet id) back to the IPC
 * `seq` we used when Core 1 asked for the send. Four entries is
 * deliberately small: at our send rate the user can't realistically
 * have more than one in flight. */
namespace {

struct OutboundEntry {
    uint32_t packet_id;
    uint8_t  ipc_seq;
    bool     in_use;
};

constexpr size_t kOutboundTracker = 4;
OutboundEntry s_outbound[kOutboundTracker];
size_t s_outbound_next;

void tracker_remember(uint32_t packet_id, uint8_t ipc_seq)
{
    s_outbound[s_outbound_next] = { packet_id, ipc_seq, true };
    s_outbound_next = (s_outbound_next + 1u) % kOutboundTracker;
}

bool tracker_consume(uint32_t packet_id, uint8_t *out_seq)
{
    for (size_t i = 0; i < kOutboundTracker; ++i) {
        if (s_outbound[i].in_use && s_outbound[i].packet_id == packet_id) {
            if (out_seq) *out_seq = s_outbound[i].ipc_seq;
            s_outbound[i].in_use = false;
            return true;
        }
    }
    return false;
}

} // namespace

/* Push an IPC_MSG_TX_ACK onto the c0→c1 DATA ring. Best-effort — if the
 * ring is full we drop the ACK; Core 1 keeps showing whichever earlier
 * status is in messages_view's footer. */
void mokya_push_tx_ack(uint8_t ipc_seq,
                       uint8_t result,
                       uint8_t error_reason,
                       uint32_t packet_id)
{
    IpcPayloadTxAck payload = {};
    payload.seq          = ipc_seq;
    payload.result       = result;
    payload.error_reason = error_reason;
    payload.packet_id    = packet_id;

    (void)ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                        g_ipc_shared.c0_to_c1_slots,
                        IPC_RING_SLOT_COUNT,
                        IPC_MSG_TX_ACK,
                        ipc_seq,
                        &payload,
                        (uint16_t)sizeof(payload));
}

/* Build a TEXT_MESSAGE_APP MeshPacket from an IpcPayloadText payload and
 * hand it off to MeshService for tx. Mirrors the canned-message send
 * path in InkHUD MenuApplet (sendText) and the Serial module's send.
 * After sendToMesh, registers the new packet id with the tracker and
 * pushes an immediate "sending" IPC_MSG_TX_ACK so Core 1 can update its
 * footer before any LoRa-level ACK arrives. */
static void handle_send_text(uint8_t ipc_seq,
                             const uint8_t *payload,
                             uint16_t payload_len)
{
    if (payload_len < sizeof(IpcPayloadText)) return;

    const IpcPayloadText *t =
        reinterpret_cast<const IpcPayloadText *>(payload);

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
        mokya_push_tx_ack(ipc_seq,
                          /*result=failed*/ 2u,
                          (uint8_t)meshtastic_Routing_Error_NO_INTERFACE,
                          /*packet_id=*/ 0u);
        return;
    }

    meshtastic_MeshPacket *p = router->allocForSending();
    if (!p) {
        LOG_WARN("IPC_CMD_SEND_TEXT: allocForSending() returned null");
        mokya_push_tx_ack(ipc_seq, 2u,
                          (uint8_t)meshtastic_Routing_Error_TOO_LARGE, 0u);
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

    /* Remember the assignment first — sendToMesh queues async, and a
     * very fast ACK could fire the observer before we'd otherwise have
     * stored the mapping. Tracker entry is harmless if the send fails
     * (observer simply never sees a matching request_id). */
    if (p->want_ack) {
        tracker_remember(p->id, ipc_seq);
    }

    service->sendToMesh(p, RX_SRC_LOCAL, /*ccToPhone=*/true);

    mokya_push_tx_ack(ipc_seq,
                      /*result=sending*/ 0u,
                      (uint8_t)meshtastic_Routing_Error_NONE,
                      p->id);
}

bool mokya_tx_tracker_consume(uint32_t packet_id, uint8_t *out_seq)
{
    return tracker_consume(packet_id, out_seq);
}

/* Implemented in ipc_config_handler.cpp. Declared here so the dispatcher
 * can route GET / SET / COMMIT* to the appropriate handler. */
extern "C" void mokya_handle_ipc_get_config(uint8_t seq, const uint8_t *payload, uint16_t len);
extern "C" void mokya_handle_ipc_set_config(uint8_t seq, const uint8_t *payload, uint16_t len);
extern "C" void mokya_handle_ipc_commit_config(uint8_t seq, const uint8_t *payload, uint16_t len);
extern "C" void mokya_handle_ipc_commit_reboot(uint8_t seq, const uint8_t *payload, uint16_t len);

/* Thunks for ipc_config_handler.cpp — keep that TU free of MeshService.h
 * (which pulls a heavier dependency chain than we want there). */

/* Generic reloadConfig — saveWhat is a bitmask of SEGMENT_* flags from
 * NodeDB.h. Use SEGMENT_CONFIG for the LoRa/Device/Position/Power/Display
 * paths, SEGMENT_CHANNELS for channel edits. Multiple bits OK. */
extern "C" void mokya_meshservice_reload_config(uint32_t saveWhat)
{
    if (service && saveWhat != 0u) {
        service->reloadConfig(static_cast<int>(saveWhat));
    }
}

/* Legacy alias — kept so any external caller of the old name still links.
 * New code should call mokya_meshservice_reload_config(SEGMENT_CONFIG). */
extern "C" void mokya_meshservice_reload_config_segment_config(void)
{
    mokya_meshservice_reload_config(SEGMENT_CONFIG);
}

/* Owner reload — broadcasts updated owner.long_name / short_name to the mesh
 * via NodeInfoModule. Saves SEGMENT_DEVICESTATE through reloadOwner's normal
 * path. Pass shouldSave=true unless the caller is mid-edit-transaction. */
extern "C" void mokya_meshservice_reload_owner(uint8_t shouldSave)
{
    if (service) {
        service->reloadOwner(shouldSave != 0u);
    }
}

/* Schedule a graceful reboot. Setting rebootAtMsec triggers Power::reboot()
 * from the main loop, which fires notifyReboot → RebootNotifier observer
 * (variant.cpp:60) → IPC_MSG_REBOOT_NOTIFY to Core 1 → tud_disconnect()
 * → watchdog reset. delay_ms gives the COMMIT_REBOOT reply time to clear
 * the c0→c1 ring before the notify fires. */
extern "C" void mokya_request_graceful_reboot(uint32_t delay_ms)
{
    rebootAtMsec = millis() + delay_ms;
}

extern "C" void mokya_handle_ipc_command(uint8_t msg_id,
                                         uint8_t ipc_seq,
                                         const uint8_t *payload,
                                         uint16_t payload_len)
{
    switch (msg_id) {
        case IPC_CMD_SEND_TEXT:
            handle_send_text(ipc_seq, payload, payload_len);
            return;
        case IPC_CMD_GET_CONFIG:
            mokya_handle_ipc_get_config(ipc_seq, payload, payload_len);
            return;
        case IPC_CMD_SET_CONFIG:
            mokya_handle_ipc_set_config(ipc_seq, payload, payload_len);
            return;
        case IPC_CMD_COMMIT_CONFIG:
            mokya_handle_ipc_commit_config(ipc_seq, payload, payload_len);
            return;
        case IPC_CMD_COMMIT_REBOOT:
            mokya_handle_ipc_commit_reboot(ipc_seq, payload, payload_len);
            return;
        case IPC_MSG_LOG_LINE:
            /* Log lines from Core 1 historically went onto the same DATA
             * ring during M1 development; production drops them — they
             * arrive on the LOG ring. Silently ignore here. */
            return;
        default:
            /* Unknown CMD id — reserved for future M5 messages
             * (NODE_UPDATE, etc.). Silently drop so a future Core 1
             * build can speak to an older Core 0. */
            return;
    }
}
