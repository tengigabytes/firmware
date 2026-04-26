/* ipc_command_handler.h — Internal hooks for the variant's IPC command
 * dispatcher. Public users should NOT include this; the public dispatch
 * entry point is `extern "C" void mokya_handle_ipc_command(...)`
 * declared in ipc_serial_stub.cpp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

/* Pop a tracker entry by Meshtastic packet id; returns true and writes
 * the original IPC seq to *out_seq if matched. Used by the ACK observer
 * to correlate a routing-layer ACK back to the originating
 * IPC_CMD_SEND_TEXT. */
bool mokya_tx_tracker_consume(uint32_t packet_id, uint8_t *out_seq);

/* Push an IPC_MSG_TX_ACK onto the c0→c1 DATA ring. */
void mokya_push_tx_ack(uint8_t ipc_seq,
                       uint8_t result,
                       uint8_t error_reason,
                       uint32_t packet_id);
