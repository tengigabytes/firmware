/* ipc_config_handler.cpp — Core 0 handler for IPC_CMD_GET/SET/COMMIT_CONFIG.
 *
 * Bisect step 2: full handler implementation but WITHOUT MeshService.h.
 * Use the global `service` only via a forward declaration to call
 * reloadConfig.  All other state goes through `config.lora.*` and the
 * IPC ring helpers, which we already know work fine in ipc_node_observer
 * and ipc_command_handler.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "configuration.h"
#include "NodeDB.h"   /* declares the global `config` (meshtastic_LocalConfig) */

#include "ipc_protocol.h"
#include "ipc_shared_layout.h"
#include "ipc_ringbuf.h"

/* Thunk implemented in ipc_command_handler.cpp where MeshService.h is
 * already included. Avoids pulling MeshService's heavier dependency chain
 * into this TU. */
extern "C" void mokya_meshservice_reload_config_segment_config(void);

namespace {

bool s_pending_lora_reload = false;

constexpr uint8_t kResultOK            = 0u;
constexpr uint8_t kResultUnknownKey    = 1u;
constexpr uint8_t kResultInvalidValue  = 2u;
constexpr uint8_t kResultBusy          = 3u;

void push_value(uint8_t seq, uint16_t key, const void *val, uint16_t val_len)
{
    if (val_len > 32u) val_len = 32u;
    uint8_t buf[sizeof(IpcPayloadConfigValue) + 32u];
    IpcPayloadConfigValue *p =
        reinterpret_cast<IpcPayloadConfigValue *>(buf);
    p->key       = key;
    p->value_len = val_len;
    memcpy(p->value, val, val_len);
    (void)ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                        g_ipc_shared.c0_to_c1_slots,
                        IPC_RING_SLOT_COUNT,
                        IPC_MSG_CONFIG_VALUE,
                        seq,
                        buf,
                        (uint16_t)(sizeof(IpcPayloadConfigValue) + val_len));
}

void push_result(uint8_t seq, uint16_t key, uint8_t result)
{
    IpcPayloadConfigResult r{};
    r.key    = key;
    r.result = result;
    (void)ipc_ring_push(&g_ipc_shared.c0_to_c1_ctrl,
                        g_ipc_shared.c0_to_c1_slots,
                        IPC_RING_SLOT_COUNT,
                        IPC_MSG_CONFIG_RESULT,
                        seq,
                        &r,
                        (uint16_t)sizeof(r));
}

bool decode_get(const uint8_t *payload, uint16_t len, uint16_t *out_key)
{
    if (len < sizeof(IpcPayloadGetConfig)) return false;
    *out_key = reinterpret_cast<const IpcPayloadGetConfig *>(payload)->key;
    return true;
}

bool decode_set(const uint8_t *payload, uint16_t len,
                uint16_t *out_key,
                const uint8_t **out_value, uint16_t *out_value_len)
{
    if (len < sizeof(IpcPayloadConfigValue)) return false;
    const IpcPayloadConfigValue *p =
        reinterpret_cast<const IpcPayloadConfigValue *>(payload);
    const uint16_t header = (uint16_t)sizeof(IpcPayloadConfigValue);
    if ((uint32_t)header + p->value_len > (uint32_t)len) return false;
    *out_key       = p->key;
    *out_value     = p->value;
    *out_value_len = p->value_len;
    return true;
}

} // namespace

extern "C" void mokya_handle_ipc_get_config(uint8_t seq,
                                            const uint8_t *payload,
                                            uint16_t len)
{
    uint16_t key;
    if (!decode_get(payload, len, &key)) {
        push_result(seq, 0u, kResultInvalidValue);
        return;
    }

    switch (key) {
    case IPC_CFG_LORA_REGION: {
        uint8_t v = (uint8_t)config.lora.region;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_LORA_TX_POWER: {
        int8_t v = (int8_t)config.lora.tx_power;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_LORA_HOP_LIMIT: {
        uint8_t v = (uint8_t)config.lora.hop_limit;
        push_value(seq, key, &v, 1u);
        return;
    }
    default:
        push_result(seq, key, kResultUnknownKey);
        return;
    }
}

extern "C" void mokya_handle_ipc_set_config(uint8_t seq,
                                            const uint8_t *payload,
                                            uint16_t len)
{
    uint16_t key;
    const uint8_t *val = nullptr;
    uint16_t vlen = 0u;
    if (!decode_set(payload, len, &key, &val, &vlen)) {
        push_result(seq, 0u, kResultInvalidValue);
        return;
    }

    switch (key) {
    case IPC_CFG_LORA_REGION:
        if (vlen < 1u) { push_result(seq, key, kResultInvalidValue); return; }
        config.lora.region = (decltype(config.lora.region))val[0];
        s_pending_lora_reload = true;
        push_result(seq, key, kResultOK);
        return;

    case IPC_CFG_LORA_TX_POWER:
        if (vlen < 1u) { push_result(seq, key, kResultInvalidValue); return; }
        config.lora.tx_power = (int8_t)val[0];
        s_pending_lora_reload = true;
        push_result(seq, key, kResultOK);
        return;

    case IPC_CFG_LORA_HOP_LIMIT:
        if (vlen < 1u) { push_result(seq, key, kResultInvalidValue); return; }
        if (val[0] < 1u || val[0] > 7u) {
            push_result(seq, key, kResultInvalidValue);
            return;
        }
        config.lora.hop_limit = val[0];
        s_pending_lora_reload = true;
        push_result(seq, key, kResultOK);
        return;

    default:
        push_result(seq, key, kResultUnknownKey);
        return;
    }
}

extern "C" void mokya_handle_ipc_commit_config(uint8_t seq,
                                               const uint8_t * /*payload*/,
                                               uint16_t /*len*/)
{
    if (s_pending_lora_reload) {
        mokya_meshservice_reload_config_segment_config();
        s_pending_lora_reload = false;
    }
    push_result(seq, 0u, kResultOK);
}
