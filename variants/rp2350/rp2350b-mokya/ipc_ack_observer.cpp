/* ipc_ack_observer.cpp — Surface mesh-level ACKs back to Core 1 as
 * IPC_MSG_TX_ACK.
 *
 * Subclass of Meshtastic's ProtobufModule<meshtastic_Routing>. Constructor
 * registers itself in the global meshModulesAvailable list, so every
 * inbound ROUTING_APP packet flows through handleReceivedProtobuf —
 * including the ACK / NAK that comes back for our own want_ack=true
 * sends. We correlate the ACK's request_id against the small tracker
 * maintained in ipc_command_handler.cpp; on a hit, push IPC_MSG_TX_ACK
 * with result = 1 (delivered) or 2 (failed, error_reason carries the
 * reason).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "configuration.h"
#include "ProtobufModule.h"
#include "MeshService.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

#include "ipc_protocol.h"
#include "ipc_command_handler.h"

namespace
{

class IpcAckObserver : public ProtobufModule<meshtastic_Routing>
{
  public:
    IpcAckObserver()
        : ProtobufModule("ipc_ack_observer",
                         meshtastic_PortNum_ROUTING_APP,
                         &meshtastic_Routing_msg)
    {
    }

  protected:
    bool handleReceivedProtobuf(const meshtastic_MeshPacket &mp,
                                meshtastic_Routing *r) override
    {
        if (r == nullptr) return false;

        /* request_id is set by the ACK/NAK routing reply; non-ACK
         * routing traffic (route discovery etc.) leaves it 0. */
        uint32_t request_id = mp.decoded.request_id;
        if (request_id == 0u) return false;

        uint8_t ipc_seq = 0u;
        if (!mokya_tx_tracker_consume(request_id, &ipc_seq)) {
            /* Not one of ours — could be an ACK for a packet we didn't
             * originate (the channel is shared) or a packet whose
             * tracker entry got evicted by FIFO turnover. Silently let
             * other modules continue handling. */
            return false;
        }

        const bool ok = (r->error_reason == meshtastic_Routing_Error_NONE);
        mokya_push_tx_ack(ipc_seq,
                          ok ? /*delivered*/ 1u : /*failed*/ 2u,
                          (uint8_t)r->error_reason,
                          request_id);

        /* Don't claim the packet — RoutingModule still needs to do its
         * own bookkeeping (stopRetransmission etc.). */
        return false;
    }
};

IpcAckObserver *s_ack_observer = nullptr;

} // namespace

extern "C" void mokya_register_ack_observer(void)
{
    if (s_ack_observer == nullptr) {
        s_ack_observer = new IpcAckObserver();
    }
}
