/* ipc_command_handler.cpp — Dispatch structured IPC commands from Core 1.
 *
 * Called by IpcSerialStream::refill_rx_() in ipc_serial_stub.cpp whenever
 * a slot pops with a non-SERIAL_BYTES msg_id. M5E.3 (2026-04-28) removed
 * the IPC_CMD_SEND_TEXT branch and the matching IPC_MSG_TX_ACK reply
 * path — Core 1's cascade PhoneAPI client now sends ToRadio bytes
 * directly through the SERIAL_BYTES ring and consumes routing-layer
 * ACKs from the FromRadio byte stream. Remaining handlers cover the
 * config IPC family (B2 soft-reload + graceful reboot) only.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "configuration.h"
#include "MeshService.h"
#include "main.h"

#include "ipc_protocol.h"

/* Implemented in ipc_config_handler.cpp. Declared here so the dispatcher
 * can route GET / SET / COMMIT* to the appropriate handler. */
extern "C" void mokya_handle_ipc_get_config(uint8_t seq, const uint8_t *payload, uint16_t len);
extern "C" void mokya_handle_ipc_set_config(uint8_t seq, const uint8_t *payload, uint16_t len);
extern "C" void mokya_handle_ipc_commit_config(uint8_t seq, const uint8_t *payload, uint16_t len);
extern "C" void mokya_handle_ipc_commit_reboot(uint8_t seq, const uint8_t *payload, uint16_t len);

/* Implemented in ipc_dormant_handler.cpp (Phase C Sprint 3b). */
extern "C" void mokya_handle_ipc_dormant_request(uint8_t seq, const uint8_t *payload, uint16_t len);
extern "C" void mokya_handle_ipc_dormant_wake(uint8_t seq, const uint8_t *payload, uint16_t len);

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
        case IPC_CMD_DORMANT_REQUEST:
            mokya_handle_ipc_dormant_request(ipc_seq, payload, payload_len);
            return;
        case IPC_CMD_DORMANT_WAKE:
            mokya_handle_ipc_dormant_wake(ipc_seq, payload, payload_len);
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
