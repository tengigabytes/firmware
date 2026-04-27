/* ipc_config_handler.cpp — Core 0 handler for IPC_CMD_GET / SET / COMMIT_CONFIG
 *                          / COMMIT_REBOOT.
 *
 * Covers all IpcConfigKey categories defined in ipc_protocol.h:
 *   0x01xx Device  — DEVICE_NAME (alias→OWNER_LONG_NAME), DEVICE_ROLE
 *   0x02xx LoRa    — REGION, MODEM_PRESET, TX_POWER, HOP_LIMIT, CHANNEL_NUM
 *   0x03xx Position— GPS_MODE, GPS_UPDATE_INTERVAL, POSITION_BCAST_SECS
 *   0x04xx Power   — POWER_SAVING, SHUTDOWN_AFTER_SECS
 *   0x05xx Display — SCREEN_ON_SECS, UNITS_METRIC
 *   0x06xx Channel — CHANNEL_NAME, CHANNEL_PSK (primary channel only — no
 *                    index field in protocol yet, see open follow-up in
 *                    plan b2-mossy-brooks.md)
 *   0x07xx Owner   — OWNER_LONG_NAME, OWNER_SHORT_NAME
 *
 * SET path: validates value, writes to config.* / channelFile / owner,
 * tags the affected segment in s_pending_segments bitmask plus
 * s_pending_owner if any owner key was touched. Multiple SETs accumulate;
 * COMMIT triggers the actual flash save + observer notification.
 *
 * COMMIT_CONFIG: soft reload — service->reloadConfig(saveWhat) for the
 * pending segments + reloadOwner() if owner pending. No MCU reset.
 *
 * COMMIT_REBOOT: same flash + reload, then schedules graceful reboot via
 * Power::reboot() (P2-10 path: notifyReboot → IPC_MSG_REBOOT_NOTIFY →
 * Core 1 tud_disconnect → watchdog). Caller (Core 1 settings UI)
 * decides which commit path to use based on its needs_reboot table.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "configuration.h"
#include "NodeDB.h"   /* declares the global `config`, `owner`, `channelFile` */

#include "ipc_protocol.h"
#include "ipc_shared_layout.h"
#include "ipc_ringbuf.h"

/* Thunks implemented in ipc_command_handler.cpp where MeshService.h /
 * main.h are already included. Keeps this TU's dependency chain light. */
extern "C" void mokya_meshservice_reload_config(uint32_t saveWhat);
extern "C" void mokya_meshservice_reload_owner(uint8_t shouldSave);
extern "C" void mokya_request_graceful_reboot(uint32_t delay_ms);

namespace {

/* Segment bitmask of pending changes from SET ops, applied at COMMIT.
 * Owner changes are tracked separately because they go through
 * service->reloadOwner() not service->reloadConfig().  Note the bit
 * values match Meshtastic's SEGMENT_* macros so we can pass directly to
 * reloadConfig(saveWhat). */
uint32_t s_pending_segments = 0u;
bool     s_pending_owner    = false;

constexpr uint8_t kResultOK            = 0u;
constexpr uint8_t kResultUnknownKey    = 1u;
constexpr uint8_t kResultInvalidValue  = 2u;
constexpr uint8_t kResultBusy          = 3u;

/* ── Ring helpers ──────────────────────────────────────────────────── */

void push_value(uint8_t seq, uint16_t key, const void *val, uint16_t val_len)
{
    /* Largest single value today is OWNER_LONG_NAME (40 B). 64 B head-room
     * future-proofs without bloating ring slot. */
    if (val_len > 64u) val_len = 64u;
    uint8_t buf[sizeof(IpcPayloadConfigValue) + 64u];
    auto *p = reinterpret_cast<IpcPayloadConfigValue *>(buf);
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
    const auto *p = reinterpret_cast<const IpcPayloadConfigValue *>(payload);
    const uint16_t header = (uint16_t)sizeof(IpcPayloadConfigValue);
    if ((uint32_t)header + p->value_len > (uint32_t)len) return false;
    *out_key       = p->key;
    *out_value     = p->value;
    *out_value_len = p->value_len;
    return true;
}

/* ── Helpers for reading current Meshtastic config into IPC values ── */

uint16_t copy_owner_long(uint8_t *out, uint16_t out_max)
{
    size_t n = strnlen(owner.long_name, sizeof(owner.long_name));
    if (n > out_max) n = out_max;
    memcpy(out, owner.long_name, n);
    return (uint16_t)n;
}

uint16_t copy_owner_short(uint8_t *out, uint16_t out_max)
{
    size_t n = strnlen(owner.short_name, sizeof(owner.short_name));
    if (n > out_max) n = out_max;
    memcpy(out, owner.short_name, n);
    return (uint16_t)n;
}

uint16_t copy_primary_channel_name(uint8_t *out, uint16_t out_max)
{
    /* Primary channel = whichever has role=PRIMARY. Fallback to index 0. */
    int idx = 0;
    for (int i = 0; i < channelFile.channels_count; ++i) {
        if (channelFile.channels[i].role == meshtastic_Channel_Role_PRIMARY) {
            idx = i;
            break;
        }
    }
    const auto &ch = channelFile.channels[idx].settings;
    size_t n = strnlen(ch.name, sizeof(ch.name));
    if (n > out_max) n = out_max;
    memcpy(out, ch.name, n);
    return (uint16_t)n;
}

uint16_t copy_primary_channel_psk(uint8_t *out, uint16_t out_max)
{
    int idx = 0;
    for (int i = 0; i < channelFile.channels_count; ++i) {
        if (channelFile.channels[i].role == meshtastic_Channel_Role_PRIMARY) {
            idx = i;
            break;
        }
    }
    const auto &ch = channelFile.channels[idx].settings;
    uint16_t n = ch.psk.size;
    if (n > out_max) n = out_max;
    memcpy(out, ch.psk.bytes, n);
    return n;
}

/* SET helpers — apply value and tag pending segment.
 * vlen check is per-kind: scalar keys need exact byte count. */

bool set_owner_long(const uint8_t *val, uint16_t vlen)
{
    if (vlen >= sizeof(owner.long_name)) return false;
    memcpy(owner.long_name, val, vlen);
    owner.long_name[vlen] = '\0';
    s_pending_owner = true;
    return true;
}

bool set_owner_short(const uint8_t *val, uint16_t vlen)
{
    if (vlen >= sizeof(owner.short_name)) return false;
    memcpy(owner.short_name, val, vlen);
    owner.short_name[vlen] = '\0';
    s_pending_owner = true;
    return true;
}

bool set_primary_channel_name(const uint8_t *val, uint16_t vlen)
{
    int idx = 0;
    for (int i = 0; i < channelFile.channels_count; ++i) {
        if (channelFile.channels[i].role == meshtastic_Channel_Role_PRIMARY) {
            idx = i;
            break;
        }
    }
    auto &ch = channelFile.channels[idx].settings;
    if (vlen >= sizeof(ch.name)) return false;
    memcpy(ch.name, val, vlen);
    ch.name[vlen] = '\0';
    s_pending_segments |= SEGMENT_CHANNELS;
    return true;
}

bool set_primary_channel_psk(const uint8_t *val, uint16_t vlen)
{
    int idx = 0;
    for (int i = 0; i < channelFile.channels_count; ++i) {
        if (channelFile.channels[i].role == meshtastic_Channel_Role_PRIMARY) {
            idx = i;
            break;
        }
    }
    auto &ch = channelFile.channels[idx].settings;
    if (vlen > sizeof(ch.psk.bytes)) return false;
    /* Valid PSK lengths: 0 (no encryption), 1 (preset key shorthand),
     * 16 (AES-128), 32 (AES-256). Everything else is invalid. */
    if (vlen != 0 && vlen != 1 && vlen != 16 && vlen != 32) return false;
    memcpy(ch.psk.bytes, val, vlen);
    ch.psk.size = vlen;
    s_pending_segments |= SEGMENT_CHANNELS;
    return true;
}

} // namespace

/* ── GET handler ───────────────────────────────────────────────────── */

extern "C" void mokya_handle_ipc_get_config(uint8_t seq,
                                            const uint8_t *payload,
                                            uint16_t len)
{
    uint16_t key;
    if (!decode_get(payload, len, &key)) {
        push_result(seq, 0u, kResultInvalidValue);
        return;
    }

    uint8_t buf[64];
    uint16_t n = 0;

    switch (key) {
    /* Device — DEVICE_NAME aliases to owner.long_name. */
    case IPC_CFG_DEVICE_NAME:
    case IPC_CFG_OWNER_LONG_NAME:
        n = copy_owner_long(buf, sizeof(buf));
        push_value(seq, key, buf, n);
        return;
    case IPC_CFG_OWNER_SHORT_NAME:
        n = copy_owner_short(buf, sizeof(buf));
        push_value(seq, key, buf, n);
        return;
    case IPC_CFG_DEVICE_ROLE: {
        uint8_t v = (uint8_t)config.device.role;
        push_value(seq, key, &v, 1u);
        return;
    }

    /* LoRa */
    case IPC_CFG_LORA_REGION: {
        uint8_t v = (uint8_t)config.lora.region;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_LORA_MODEM_PRESET: {
        uint8_t v = (uint8_t)config.lora.modem_preset;
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
    case IPC_CFG_LORA_CHANNEL_NUM: {
        uint8_t v = (uint8_t)config.lora.channel_num;
        push_value(seq, key, &v, 1u);
        return;
    }

    /* Position / GPS */
    case IPC_CFG_GPS_MODE: {
        uint8_t v = (uint8_t)config.position.gps_mode;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_GPS_UPDATE_INTERVAL: {
        uint32_t v = config.position.gps_update_interval;
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_POSITION_BCAST_SECS: {
        uint32_t v = config.position.position_broadcast_secs;
        push_value(seq, key, &v, sizeof(v));
        return;
    }

    /* Power */
    case IPC_CFG_POWER_SAVING: {
        uint8_t v = config.power.is_power_saving ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_SHUTDOWN_AFTER_SECS: {
        uint32_t v = config.power.on_battery_shutdown_after_secs;
        push_value(seq, key, &v, sizeof(v));
        return;
    }

    /* Display */
    case IPC_CFG_SCREEN_ON_SECS: {
        uint32_t v = config.display.screen_on_secs;
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_UNITS_METRIC: {
        /* Meshtastic enum: 0=METRIC, 1=IMPERIAL. Caller wants bool
         * "metric": true = METRIC. */
        uint8_t v = (config.display.units == meshtastic_Config_DisplayConfig_DisplayUnits_METRIC) ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }

    /* Channel (primary) */
    case IPC_CFG_CHANNEL_NAME:
        n = copy_primary_channel_name(buf, sizeof(buf));
        push_value(seq, key, buf, n);
        return;
    case IPC_CFG_CHANNEL_PSK:
        n = copy_primary_channel_psk(buf, sizeof(buf));
        push_value(seq, key, buf, n);
        return;

    default:
        push_result(seq, key, kResultUnknownKey);
        return;
    }
}

/* ── SET handler ───────────────────────────────────────────────────── */

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

    /* Helper macros for the common scalar SET pattern. */
#define REQ_LEN(n) do { if (vlen < (n)) { push_result(seq, key, kResultInvalidValue); return; } } while (0)
#define REQ_BOOL_RANGE() do { if (val[0] > 1u) { push_result(seq, key, kResultInvalidValue); return; } } while (0)

    switch (key) {
    /* Device — DEVICE_NAME aliases to OWNER_LONG_NAME */
    case IPC_CFG_DEVICE_NAME:
    case IPC_CFG_OWNER_LONG_NAME:
        if (!set_owner_long(val, vlen)) { push_result(seq, key, kResultInvalidValue); return; }
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_OWNER_SHORT_NAME:
        if (!set_owner_short(val, vlen)) { push_result(seq, key, kResultInvalidValue); return; }
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DEVICE_ROLE:
        REQ_LEN(1);
        config.device.role = (decltype(config.device.role))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* LoRa */
    case IPC_CFG_LORA_REGION:
        REQ_LEN(1);
        config.lora.region = (decltype(config.lora.region))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_MODEM_PRESET:
        REQ_LEN(1);
        config.lora.modem_preset = (decltype(config.lora.modem_preset))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_TX_POWER:
        REQ_LEN(1);
        config.lora.tx_power = (int8_t)val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_HOP_LIMIT:
        REQ_LEN(1);
        if (val[0] < 1u || val[0] > 7u) { push_result(seq, key, kResultInvalidValue); return; }
        config.lora.hop_limit = val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_CHANNEL_NUM:
        REQ_LEN(1);
        config.lora.channel_num = val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* Position */
    case IPC_CFG_GPS_MODE:
        REQ_LEN(1);
        /* enum: 0=DISABLED, 1=ENABLED, 2=NOT_PRESENT */
        if (val[0] > 2u) { push_result(seq, key, kResultInvalidValue); return; }
        config.position.gps_mode = (decltype(config.position.gps_mode))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_GPS_UPDATE_INTERVAL:
        REQ_LEN(4);
        config.position.gps_update_interval = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POSITION_BCAST_SECS:
        REQ_LEN(4);
        config.position.position_broadcast_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* Power */
    case IPC_CFG_POWER_SAVING:
        REQ_LEN(1);
        REQ_BOOL_RANGE();
        config.power.is_power_saving = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_SHUTDOWN_AFTER_SECS:
        REQ_LEN(4);
        config.power.on_battery_shutdown_after_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* Display */
    case IPC_CFG_SCREEN_ON_SECS:
        REQ_LEN(4);
        config.display.screen_on_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_UNITS_METRIC:
        REQ_LEN(1);
        REQ_BOOL_RANGE();
        config.display.units = val[0]
            ? meshtastic_Config_DisplayConfig_DisplayUnits_METRIC
            : meshtastic_Config_DisplayConfig_DisplayUnits_IMPERIAL;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* Channel (primary) */
    case IPC_CFG_CHANNEL_NAME:
        if (!set_primary_channel_name(val, vlen)) { push_result(seq, key, kResultInvalidValue); return; }
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_CHANNEL_PSK:
        if (!set_primary_channel_psk(val, vlen)) { push_result(seq, key, kResultInvalidValue); return; }
        push_result(seq, key, kResultOK);
        return;

    default:
        push_result(seq, key, kResultUnknownKey);
        return;
    }

#undef REQ_LEN
#undef REQ_BOOL_RANGE
}

/* ── COMMIT helpers ────────────────────────────────────────────────── */

namespace {

void apply_pending(void)
{
    /* Owner first — reloadOwner() also broadcasts the new node info to the
     * mesh, so doing it before reloadConfig() gives the LoRa-stack reload
     * (if any) a chance to use the new identity. Order doesn't matter for
     * correctness, only for cosmetic mesh-broadcast freshness. */
    if (s_pending_owner) {
        mokya_meshservice_reload_owner(/*shouldSave=*/ 1u);
        s_pending_owner = false;
    }
    if (s_pending_segments != 0u) {
        mokya_meshservice_reload_config(s_pending_segments);
        s_pending_segments = 0u;
    }
}

} // namespace

/* ── COMMIT_CONFIG (soft reload, no reset) ─────────────────────────── */

extern "C" void mokya_handle_ipc_commit_config(uint8_t seq,
                                               const uint8_t * /*payload*/,
                                               uint16_t /*len*/)
{
    apply_pending();
    push_result(seq, 0u, kResultOK);
}

/* ── COMMIT_REBOOT (soft reload + graceful reboot) ─────────────────── */

extern "C" void mokya_handle_ipc_commit_reboot(uint8_t seq,
                                               const uint8_t * /*payload*/,
                                               uint16_t /*len*/)
{
    /* Same flash + reload as COMMIT_CONFIG so the value survives reset.
     * Then push the OK reply (so Core 1 sees ack on the c0→c1 ring before
     * USB CDC drops) and schedule rebootAtMsec → Power::reboot() →
     * notifyReboot → IPC_MSG_REBOOT_NOTIFY → Core 1 tud_disconnect() →
     * watchdog reset. The 1500 ms delay covers ring drain + CDC flush
     * even when the host is mid-burst. */
    apply_pending();
    push_result(seq, 0u, kResultOK);
    mokya_request_graceful_reboot(1500u);
}
