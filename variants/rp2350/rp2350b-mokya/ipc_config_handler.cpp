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
    memset(buf, 0, sizeof(IpcPayloadConfigValue));   /* clear header _pad */
    auto *p = reinterpret_cast<IpcPayloadConfigValue *>(buf);
    p->key           = key;
    p->value_len     = val_len;
    p->channel_index = 0;   /* B3-P3 will set for 0x06xx channel keys */
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

/* Tolerate legacy 2-byte IpcPayloadGetConfig (no channel_index): if
 * payload is exactly 2 bytes it is the legacy format and channel_index
 * is implicitly 0. New 4-byte payload carries channel_index at offset 2. */
bool decode_get(const uint8_t *payload, uint16_t len,
                uint16_t *out_key, uint8_t *out_channel_index)
{
    if (len < 2u) return false;
    *out_key = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    *out_channel_index = (len >= 3u) ? payload[2] : 0u;
    return true;
}

/* Tolerate legacy 4-byte IpcPayloadConfigValue header (no channel_index):
 * detect by comparing payload length against value_len + header_size for
 * both legacy (4 B) and current (8 B) header sizes. New code emits the
 * 8-byte form; legacy SWD test harnesses and B2-era scripts emit 4-byte. */
bool decode_set(const uint8_t *payload, uint16_t len,
                uint16_t *out_key,
                uint8_t *out_channel_index,
                const uint8_t **out_value, uint16_t *out_value_len)
{
    if (len < 4u) return false;
    uint16_t key       = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint16_t value_len = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);

    constexpr uint16_t kLegacyHeader = 4u;
    constexpr uint16_t kHeader       = (uint16_t)sizeof(IpcPayloadConfigValue);
    static_assert(kHeader == 8u, "IpcPayloadConfigValue header size changed");

    uint16_t header_used;
    uint8_t  channel_index;
    if ((uint32_t)kHeader + value_len <= (uint32_t)len) {
        header_used   = kHeader;
        channel_index = payload[4];
    } else if ((uint32_t)kLegacyHeader + value_len <= (uint32_t)len) {
        header_used   = kLegacyHeader;
        channel_index = 0u;
    } else {
        return false;
    }

    *out_key           = key;
    *out_channel_index = channel_index;
    *out_value         = payload + header_used;
    *out_value_len     = value_len;
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

/* B3-P3: addressed by channel_index (0..MAX_NUM_CHANNELS-1). The pre-B3-P3
 * helpers walked channelFile to find role==PRIMARY; that is now the
 * caller's job (Core 1 settings UI passes the index it wants to read). */
constexpr uint8_t kChannelMax = 8u;  /* matches MAX_NUM_CHANNELS / PHONEAPI_CHANNEL_COUNT */

bool channel_index_valid(uint8_t channel_index)
{
    return channel_index < kChannelMax;
}

uint16_t copy_channel_name(uint8_t channel_index, uint8_t *out, uint16_t out_max)
{
    const auto &ch = channelFile.channels[channel_index].settings;
    size_t n = strnlen(ch.name, sizeof(ch.name));
    if (n > out_max) n = out_max;
    memcpy(out, ch.name, n);
    return (uint16_t)n;
}

uint16_t copy_channel_psk(uint8_t channel_index, uint8_t *out, uint16_t out_max)
{
    const auto &ch = channelFile.channels[channel_index].settings;
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

bool set_channel_name(uint8_t channel_index, const uint8_t *val, uint16_t vlen)
{
    auto &ch = channelFile.channels[channel_index].settings;
    if (vlen >= sizeof(ch.name)) return false;
    memcpy(ch.name, val, vlen);
    ch.name[vlen] = '\0';
    s_pending_segments |= SEGMENT_CHANNELS;
    return true;
}

bool set_channel_psk(uint8_t channel_index, const uint8_t *val, uint16_t vlen)
{
    auto &ch = channelFile.channels[channel_index].settings;
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
    uint8_t  channel_index;
    if (!decode_get(payload, len, &key, &channel_index)) {
        push_result(seq, 0u, kResultInvalidValue);
        return;
    }
    /* channel_index honoured only by 0x06xx Channel keys (B3-P3). For
     * non-channel keys it is ignored. */
    if ((key & 0xFF00u) == 0x0600u && !channel_index_valid(channel_index)) {
        push_result(seq, key, kResultInvalidValue);
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

    /* Channel (B3-P3 — addressed by channel_index 0..7) */
    case IPC_CFG_CHANNEL_NAME:
        n = copy_channel_name(channel_index, buf, sizeof(buf));
        push_value(seq, key, buf, n);
        return;
    case IPC_CFG_CHANNEL_PSK:
        n = copy_channel_psk(channel_index, buf, sizeof(buf));
        push_value(seq, key, buf, n);
        return;

    /* ── Device (B3-P1 expansion) ────────────────────────────────── */
    case IPC_CFG_DEVICE_REBROADCAST_MODE: {
        uint8_t v = (uint8_t)config.device.rebroadcast_mode;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DEVICE_NODE_INFO_BCAST_SECS: {
        uint32_t v = config.device.node_info_broadcast_secs;
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_DEVICE_DOUBLE_TAP_BTN: {
        uint8_t v = config.device.double_tap_as_button_press ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DEVICE_DISABLE_TRIPLE_CLICK: {
        uint8_t v = config.device.disable_triple_click ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DEVICE_TZDEF: {
        size_t tz_len = strnlen(config.device.tzdef, sizeof(config.device.tzdef));
        if (tz_len > sizeof(buf)) tz_len = sizeof(buf);
        memcpy(buf, config.device.tzdef, tz_len);
        push_value(seq, key, buf, (uint16_t)tz_len);
        return;
    }
    case IPC_CFG_DEVICE_LED_HEARTBEAT_DISABLED: {
        uint8_t v = config.device.led_heartbeat_disabled ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }

    /* ── LoRa (B3-P1 expansion) ──────────────────────────────────── */
    case IPC_CFG_LORA_USE_PRESET: {
        uint8_t v = config.lora.use_preset ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_LORA_BANDWIDTH: {
        uint32_t v = (uint32_t)config.lora.bandwidth;   /* nanopb stores u16 */
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_LORA_SPREAD_FACTOR: {
        uint32_t v = config.lora.spread_factor;
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_LORA_CODING_RATE: {
        uint32_t v = (uint32_t)config.lora.coding_rate;  /* nanopb stores u8 */
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_LORA_TX_ENABLED: {
        uint8_t v = config.lora.tx_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_LORA_OVERRIDE_DUTY_CYCLE: {
        uint8_t v = config.lora.override_duty_cycle ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_LORA_SX126X_RX_BOOSTED_GAIN: {
        uint8_t v = config.lora.sx126x_rx_boosted_gain ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_LORA_FEM_LNA_MODE: {
        uint8_t v = (uint8_t)config.lora.fem_lna_mode;
        push_value(seq, key, &v, 1u);
        return;
    }

    /* ── Position (B3-P1 expansion) ──────────────────────────────── */
    case IPC_CFG_POSITION_BCAST_SMART_ENABLED: {
        uint8_t v = config.position.position_broadcast_smart_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_POSITION_FIXED_POSITION: {
        uint8_t v = config.position.fixed_position ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_POSITION_FLAGS: {
        uint32_t v = config.position.position_flags;
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_POSITION_BCAST_SMART_MIN_DIST: {
        uint32_t v = config.position.broadcast_smart_minimum_distance;
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_POSITION_BCAST_SMART_MIN_INT_SECS: {
        uint32_t v = config.position.broadcast_smart_minimum_interval_secs;
        push_value(seq, key, &v, sizeof(v));
        return;
    }

    /* ── Power (B3-P2 expansion) ─────────────────────────────────── */
    case IPC_CFG_POWER_SDS_SECS: {
        uint32_t v = config.power.sds_secs;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_POWER_LS_SECS: {
        uint32_t v = config.power.ls_secs;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_POWER_MIN_WAKE_SECS: {
        uint32_t v = config.power.min_wake_secs;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_POWER_BATTERY_INA_ADDRESS: {
        uint32_t v = config.power.device_battery_ina_address;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_POWER_POWERMON_ENABLES: {
        /* powermon_enables is uint64; B3-P2 surfaces only low 32 bits.
         * Anything above bit 31 in the durable config is silently
         * truncated for the UI. */
        uint32_t v = (uint32_t)(config.power.powermon_enables & 0xFFFFFFFFu);
        push_value(seq, key, &v, sizeof(v)); return;
    }

    /* ── Channel module_settings (B3-P3 — addressed by channel_index) ── */
    case IPC_CFG_CHANNEL_MODULE_POSITION_PRECISION: {
        uint32_t v = channelFile.channels[channel_index].settings.module_settings.position_precision;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_CHANNEL_MODULE_IS_MUTED: {
        uint8_t v = channelFile.channels[channel_index].settings.module_settings.is_muted ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }

    /* ── Owner extras (B3-P2) ────────────────────────────────────── */
    case IPC_CFG_OWNER_IS_LICENSED: {
        uint8_t v = owner.is_licensed ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_OWNER_PUBLIC_KEY: {
        uint16_t plen = owner.public_key.size;
        if (plen > sizeof(buf)) plen = sizeof(buf);
        memcpy(buf, owner.public_key.bytes, plen);
        push_value(seq, key, buf, plen); return;
    }

    /* ── Security (B3-P2) ────────────────────────────────────────── */
    case IPC_CFG_SECURITY_PUBLIC_KEY: {
        uint16_t plen = config.security.public_key.size;
        if (plen > sizeof(buf)) plen = sizeof(buf);
        memcpy(buf, config.security.public_key.bytes, plen);
        push_value(seq, key, buf, plen); return;
    }
    case IPC_CFG_SECURITY_IS_MANAGED: {
        uint8_t v = config.security.is_managed ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_SECURITY_SERIAL_ENABLED: {
        uint8_t v = config.security.serial_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_SECURITY_DEBUG_LOG_API_ENABLED: {
        uint8_t v = config.security.debug_log_api_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_SECURITY_ADMIN_CHANNEL_ENABLED: {
        uint8_t v = config.security.admin_channel_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }

    /* ── Display (B3-P1 expansion) ───────────────────────────────── */
    case IPC_CFG_DISPLAY_AUTO_CAROUSEL_SECS: {
        uint32_t v = config.display.auto_screen_carousel_secs;
        push_value(seq, key, &v, sizeof(v));
        return;
    }
    case IPC_CFG_DISPLAY_FLIP_SCREEN: {
        uint8_t v = config.display.flip_screen ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_OLED: {
        uint8_t v = (uint8_t)config.display.oled;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_DISPLAYMODE: {
        uint8_t v = (uint8_t)config.display.displaymode;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_HEADING_BOLD: {
        uint8_t v = config.display.heading_bold ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_WAKE_ON_TAP_OR_MOTION: {
        uint8_t v = config.display.wake_on_tap_or_motion ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_COMPASS_ORIENTATION: {
        uint8_t v = (uint8_t)config.display.compass_orientation;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_USE_12H_CLOCK: {
        uint8_t v = config.display.use_12h_clock ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_USE_LONG_NODE_NAME: {
        uint8_t v = config.display.use_long_node_name ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }
    case IPC_CFG_DISPLAY_ENABLE_MESSAGE_BUBBLES: {
        uint8_t v = config.display.enable_message_bubbles ? 1u : 0u;
        push_value(seq, key, &v, 1u);
        return;
    }

    /* ── ModuleConfig.Telemetry (B3-P3) ──────────────────────────── */
    case IPC_CFG_TELEM_DEVICE_UPDATE_INTERVAL: {
        uint32_t v = moduleConfig.telemetry.device_update_interval;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_TELEM_ENV_UPDATE_INTERVAL: {
        uint32_t v = moduleConfig.telemetry.environment_update_interval;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_TELEM_ENV_MEASUREMENT_ENABLED: {
        uint8_t v = moduleConfig.telemetry.environment_measurement_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_TELEM_ENV_SCREEN_ENABLED: {
        uint8_t v = moduleConfig.telemetry.environment_screen_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_TELEM_ENV_DISPLAY_FAHRENHEIT: {
        uint8_t v = moduleConfig.telemetry.environment_display_fahrenheit ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_TELEM_POWER_MEASUREMENT_ENABLED: {
        uint8_t v = moduleConfig.telemetry.power_measurement_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_TELEM_POWER_UPDATE_INTERVAL: {
        uint32_t v = moduleConfig.telemetry.power_update_interval;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_TELEM_POWER_SCREEN_ENABLED: {
        uint8_t v = moduleConfig.telemetry.power_screen_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_TELEM_DEVICE_TELEM_ENABLED: {
        uint8_t v = moduleConfig.telemetry.device_telemetry_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }

    /* ── ModuleConfig.NeighborInfo (B3-P3) ───────────────────────── */
    case IPC_CFG_NEIGHBOR_ENABLED: {
        uint8_t v = moduleConfig.neighbor_info.enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_NEIGHBOR_UPDATE_INTERVAL: {
        uint32_t v = moduleConfig.neighbor_info.update_interval;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_NEIGHBOR_TRANSMIT_OVER_LORA: {
        uint8_t v = moduleConfig.neighbor_info.transmit_over_lora ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }

    /* ── ModuleConfig.RangeTest (B3-P3) ──────────────────────────── */
    case IPC_CFG_RANGETEST_ENABLED: {
        uint8_t v = moduleConfig.range_test.enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_RANGETEST_SENDER: {
        uint32_t v = moduleConfig.range_test.sender;
        push_value(seq, key, &v, sizeof(v)); return;
    }

    /* ── ModuleConfig.DetectionSensor (B3-P4) ────────────────────── */
    case IPC_CFG_DETECT_ENABLED: {
        uint8_t v = moduleConfig.detection_sensor.enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_DETECT_MIN_BCAST_SECS: {
        uint32_t v = moduleConfig.detection_sensor.minimum_broadcast_secs;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_DETECT_STATE_BCAST_SECS: {
        uint32_t v = moduleConfig.detection_sensor.state_broadcast_secs;
        push_value(seq, key, &v, sizeof(v)); return;
    }
    case IPC_CFG_DETECT_NAME: {
        size_t name_len = strnlen(moduleConfig.detection_sensor.name,
                                  sizeof(moduleConfig.detection_sensor.name));
        if (name_len > sizeof(buf)) name_len = sizeof(buf);
        memcpy(buf, moduleConfig.detection_sensor.name, name_len);
        push_value(seq, key, buf, (uint16_t)name_len); return;
    }
    case IPC_CFG_DETECT_TRIGGER_TYPE: {
        uint8_t v = (uint8_t)moduleConfig.detection_sensor.detection_trigger_type;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_DETECT_USE_PULLUP: {
        uint8_t v = moduleConfig.detection_sensor.use_pullup ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }

    /* ── ModuleConfig.CannedMessage (B3-P4) ──────────────────────── */
    case IPC_CFG_CANNED_UPDOWN1_ENABLED: {
        uint8_t v = moduleConfig.canned_message.updown1_enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_CANNED_SEND_BELL: {
        uint8_t v = moduleConfig.canned_message.send_bell ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }

    /* ── ModuleConfig.AmbientLighting (B3-P4) ────────────────────── */
    case IPC_CFG_AMBIENT_LED_STATE: {
        uint8_t v = moduleConfig.ambient_lighting.led_state ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_AMBIENT_CURRENT: {
        uint8_t v = moduleConfig.ambient_lighting.current;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_AMBIENT_RED: {
        uint8_t v = moduleConfig.ambient_lighting.red;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_AMBIENT_GREEN: {
        uint8_t v = moduleConfig.ambient_lighting.green;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_AMBIENT_BLUE: {
        uint8_t v = moduleConfig.ambient_lighting.blue;
        push_value(seq, key, &v, 1u); return;
    }

    /* ── ModuleConfig.Paxcounter (B3-P4) ─────────────────────────── */
    case IPC_CFG_PAX_ENABLED: {
        uint8_t v = moduleConfig.paxcounter.enabled ? 1u : 0u;
        push_value(seq, key, &v, 1u); return;
    }
    case IPC_CFG_PAX_UPDATE_INTERVAL: {
        uint32_t v = moduleConfig.paxcounter.paxcounter_update_interval;
        push_value(seq, key, &v, sizeof(v)); return;
    }

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
    uint8_t  channel_index;
    const uint8_t *val = nullptr;
    uint16_t vlen = 0u;
    if (!decode_set(payload, len, &key, &channel_index, &val, &vlen)) {
        push_result(seq, 0u, kResultInvalidValue);
        return;
    }
    /* channel_index honoured only by 0x06xx Channel keys (B3-P3). For
     * non-channel keys it is ignored. */
    if ((key & 0xFF00u) == 0x0600u && !channel_index_valid(channel_index)) {
        push_result(seq, key, kResultInvalidValue);
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

    /* Channel (B3-P3 — addressed by channel_index 0..7) */
    case IPC_CFG_CHANNEL_NAME:
        if (!set_channel_name(channel_index, val, vlen)) { push_result(seq, key, kResultInvalidValue); return; }
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_CHANNEL_PSK:
        if (!set_channel_psk(channel_index, val, vlen)) { push_result(seq, key, kResultInvalidValue); return; }
        push_result(seq, key, kResultOK);
        return;

    /* ── Device (B3-P1 expansion) ────────────────────────────────── */
    case IPC_CFG_DEVICE_REBROADCAST_MODE:
        REQ_LEN(1);
        if (val[0] > 5u) { push_result(seq, key, kResultInvalidValue); return; }
        config.device.rebroadcast_mode =
            (decltype(config.device.rebroadcast_mode))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DEVICE_NODE_INFO_BCAST_SECS:
        REQ_LEN(4);
        config.device.node_info_broadcast_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DEVICE_DOUBLE_TAP_BTN:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.device.double_tap_as_button_press = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DEVICE_DISABLE_TRIPLE_CLICK:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.device.disable_triple_click = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DEVICE_TZDEF: {
        if (vlen >= sizeof(config.device.tzdef)) {
            push_result(seq, key, kResultInvalidValue); return;
        }
        memcpy(config.device.tzdef, val, vlen);
        config.device.tzdef[vlen] = '\0';
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    }
    case IPC_CFG_DEVICE_LED_HEARTBEAT_DISABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.device.led_heartbeat_disabled = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── LoRa (B3-P1 expansion) ──────────────────────────────────── */
    case IPC_CFG_LORA_USE_PRESET:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.lora.use_preset = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_BANDWIDTH: {
        REQ_LEN(4);
        uint32_t v = *(const uint32_t *)val;
        if (v > 0xFFFFu) { push_result(seq, key, kResultInvalidValue); return; }
        config.lora.bandwidth = (uint16_t)v;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    }
    case IPC_CFG_LORA_SPREAD_FACTOR: {
        REQ_LEN(4);
        uint32_t v = *(const uint32_t *)val;
        if (v < 7u || v > 12u) { push_result(seq, key, kResultInvalidValue); return; }
        config.lora.spread_factor = v;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    }
    case IPC_CFG_LORA_CODING_RATE: {
        REQ_LEN(4);
        uint32_t v = *(const uint32_t *)val;
        if (v < 5u || v > 8u) { push_result(seq, key, kResultInvalidValue); return; }
        config.lora.coding_rate = (uint8_t)v;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    }
    case IPC_CFG_LORA_TX_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.lora.tx_enabled = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_OVERRIDE_DUTY_CYCLE:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.lora.override_duty_cycle = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_SX126X_RX_BOOSTED_GAIN:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.lora.sx126x_rx_boosted_gain = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_LORA_FEM_LNA_MODE:
        REQ_LEN(1);
        if (val[0] > 2u) { push_result(seq, key, kResultInvalidValue); return; }
        config.lora.fem_lna_mode =
            (decltype(config.lora.fem_lna_mode))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── Position (B3-P1 expansion) ──────────────────────────────── */
    case IPC_CFG_POSITION_BCAST_SMART_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.position.position_broadcast_smart_enabled = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POSITION_FIXED_POSITION:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.position.fixed_position = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POSITION_FLAGS:
        REQ_LEN(4);
        config.position.position_flags = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POSITION_BCAST_SMART_MIN_DIST:
        REQ_LEN(4);
        config.position.broadcast_smart_minimum_distance = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POSITION_BCAST_SMART_MIN_INT_SECS:
        REQ_LEN(4);
        config.position.broadcast_smart_minimum_interval_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── Power (B3-P2 expansion) ─────────────────────────────────── */
    case IPC_CFG_POWER_SDS_SECS:
        REQ_LEN(4);
        config.power.sds_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POWER_LS_SECS:
        REQ_LEN(4);
        config.power.ls_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POWER_MIN_WAKE_SECS:
        REQ_LEN(4);
        config.power.min_wake_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POWER_BATTERY_INA_ADDRESS:
        REQ_LEN(4);
        config.power.device_battery_ina_address = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_POWER_POWERMON_ENABLES: {
        REQ_LEN(4);
        /* Preserve high 32 bits, only update low 32. */
        uint32_t v = *(const uint32_t *)val;
        config.power.powermon_enables =
            (config.power.powermon_enables & 0xFFFFFFFF00000000ULL) | (uint64_t)v;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    }

    /* ── Channel module_settings (B3-P3 — addressed by channel_index) ── */
    case IPC_CFG_CHANNEL_MODULE_POSITION_PRECISION:
        REQ_LEN(4);
        channelFile.channels[channel_index].settings.module_settings.position_precision = *(const uint32_t *)val;
        channelFile.channels[channel_index].settings.has_module_settings = true;
        s_pending_segments |= SEGMENT_CHANNELS;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_CHANNEL_MODULE_IS_MUTED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        channelFile.channels[channel_index].settings.module_settings.is_muted = (val[0] != 0u);
        channelFile.channels[channel_index].settings.has_module_settings = true;
        s_pending_segments |= SEGMENT_CHANNELS;
        push_result(seq, key, kResultOK);
        return;

    /* ── Owner extras (B3-P2) ────────────────────────────────────── */
    case IPC_CFG_OWNER_IS_LICENSED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        owner.is_licensed = (val[0] != 0u);
        s_pending_owner = true;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_OWNER_PUBLIC_KEY:
        /* Read-only — Meshtastic generates the keypair internally
         * from SecurityConfig.private_key. Mirror is via owner. */
        push_result(seq, key, kResultInvalidValue);
        return;

    /* ── Security (B3-P2) ────────────────────────────────────────── */
    case IPC_CFG_SECURITY_PUBLIC_KEY:
        /* Read-only on Core 1 — see plan exclusion list. */
        push_result(seq, key, kResultInvalidValue);
        return;
    case IPC_CFG_SECURITY_IS_MANAGED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.security.is_managed = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_SECURITY_SERIAL_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.security.serial_enabled = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_SECURITY_DEBUG_LOG_API_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.security.debug_log_api_enabled = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_SECURITY_ADMIN_CHANNEL_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.security.admin_channel_enabled = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── Display (B3-P1 expansion) ───────────────────────────────── */
    case IPC_CFG_DISPLAY_AUTO_CAROUSEL_SECS:
        REQ_LEN(4);
        config.display.auto_screen_carousel_secs = *(const uint32_t *)val;
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_FLIP_SCREEN:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.display.flip_screen = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_OLED:
        REQ_LEN(1);
        if (val[0] > 4u) { push_result(seq, key, kResultInvalidValue); return; }
        config.display.oled = (decltype(config.display.oled))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_DISPLAYMODE:
        REQ_LEN(1);
        if (val[0] > 3u) { push_result(seq, key, kResultInvalidValue); return; }
        config.display.displaymode = (decltype(config.display.displaymode))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_HEADING_BOLD:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.display.heading_bold = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_WAKE_ON_TAP_OR_MOTION:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.display.wake_on_tap_or_motion = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_COMPASS_ORIENTATION:
        REQ_LEN(1);
        if (val[0] > 7u) { push_result(seq, key, kResultInvalidValue); return; }
        config.display.compass_orientation =
            (decltype(config.display.compass_orientation))val[0];
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_USE_12H_CLOCK:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.display.use_12h_clock = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_USE_LONG_NODE_NAME:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.display.use_long_node_name = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DISPLAY_ENABLE_MESSAGE_BUBBLES:
        REQ_LEN(1); REQ_BOOL_RANGE();
        config.display.enable_message_bubbles = (val[0] != 0u);
        s_pending_segments |= SEGMENT_CONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── ModuleConfig.Telemetry (B3-P3) ──────────────────────────── */
    case IPC_CFG_TELEM_DEVICE_UPDATE_INTERVAL:
        REQ_LEN(4);
        moduleConfig.telemetry.device_update_interval = *(const uint32_t *)val;
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_ENV_UPDATE_INTERVAL:
        REQ_LEN(4);
        moduleConfig.telemetry.environment_update_interval = *(const uint32_t *)val;
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_ENV_MEASUREMENT_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.telemetry.environment_measurement_enabled = (val[0] != 0u);
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_ENV_SCREEN_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.telemetry.environment_screen_enabled = (val[0] != 0u);
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_ENV_DISPLAY_FAHRENHEIT:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.telemetry.environment_display_fahrenheit = (val[0] != 0u);
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_POWER_MEASUREMENT_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.telemetry.power_measurement_enabled = (val[0] != 0u);
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_POWER_UPDATE_INTERVAL:
        REQ_LEN(4);
        moduleConfig.telemetry.power_update_interval = *(const uint32_t *)val;
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_POWER_SCREEN_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.telemetry.power_screen_enabled = (val[0] != 0u);
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_TELEM_DEVICE_TELEM_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.telemetry.device_telemetry_enabled = (val[0] != 0u);
        moduleConfig.has_telemetry = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── ModuleConfig.NeighborInfo (B3-P3) ────────────────────────
     * AdminModule.cpp:1008 clamps update_interval below
     * min_neighbor_info_broadcast_secs (14400) at SET time. The IPC
     * range hint mirrors that floor; underflow is rejected here so
     * the user gets a clear "invalid value" instead of silent
     * substitution. */
    case IPC_CFG_NEIGHBOR_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.neighbor_info.enabled = (val[0] != 0u);
        moduleConfig.has_neighbor_info = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_NEIGHBOR_UPDATE_INTERVAL:
        REQ_LEN(4);
        moduleConfig.neighbor_info.update_interval = *(const uint32_t *)val;
        moduleConfig.has_neighbor_info = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_NEIGHBOR_TRANSMIT_OVER_LORA:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.neighbor_info.transmit_over_lora = (val[0] != 0u);
        moduleConfig.has_neighbor_info = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── ModuleConfig.RangeTest (B3-P3) ──────────────────────────── */
    case IPC_CFG_RANGETEST_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.range_test.enabled = (val[0] != 0u);
        moduleConfig.has_range_test = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_RANGETEST_SENDER:
        REQ_LEN(4);
        moduleConfig.range_test.sender = *(const uint32_t *)val;
        moduleConfig.has_range_test = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── ModuleConfig.DetectionSensor (B3-P4) ────────────────────── */
    case IPC_CFG_DETECT_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.detection_sensor.enabled = (val[0] != 0u);
        moduleConfig.has_detection_sensor = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DETECT_MIN_BCAST_SECS:
        REQ_LEN(4);
        moduleConfig.detection_sensor.minimum_broadcast_secs = *(const uint32_t *)val;
        moduleConfig.has_detection_sensor = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DETECT_STATE_BCAST_SECS:
        REQ_LEN(4);
        moduleConfig.detection_sensor.state_broadcast_secs = *(const uint32_t *)val;
        moduleConfig.has_detection_sensor = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DETECT_NAME: {
        if (vlen >= sizeof(moduleConfig.detection_sensor.name)) {
            push_result(seq, key, kResultInvalidValue); return;
        }
        memcpy(moduleConfig.detection_sensor.name, val, vlen);
        moduleConfig.detection_sensor.name[vlen] = '\0';
        moduleConfig.has_detection_sensor = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    }
    case IPC_CFG_DETECT_TRIGGER_TYPE:
        REQ_LEN(1);
        if (val[0] > 5u) { push_result(seq, key, kResultInvalidValue); return; }
        moduleConfig.detection_sensor.detection_trigger_type =
            (decltype(moduleConfig.detection_sensor.detection_trigger_type))val[0];
        moduleConfig.has_detection_sensor = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_DETECT_USE_PULLUP:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.detection_sensor.use_pullup = (val[0] != 0u);
        moduleConfig.has_detection_sensor = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── ModuleConfig.CannedMessage (B3-P4) ──────────────────────── */
    case IPC_CFG_CANNED_UPDOWN1_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.canned_message.updown1_enabled = (val[0] != 0u);
        moduleConfig.has_canned_message = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_CANNED_SEND_BELL:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.canned_message.send_bell = (val[0] != 0u);
        moduleConfig.has_canned_message = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── ModuleConfig.AmbientLighting (B3-P4) ─────────────────────
     * current/red/green/blue are uint8 in nanopb. Wire format is one
     * byte each. led_state is bool. */
    case IPC_CFG_AMBIENT_LED_STATE:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.ambient_lighting.led_state = (val[0] != 0u);
        moduleConfig.has_ambient_lighting = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_AMBIENT_CURRENT:
        REQ_LEN(1);
        moduleConfig.ambient_lighting.current = val[0];
        moduleConfig.has_ambient_lighting = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_AMBIENT_RED:
        REQ_LEN(1);
        moduleConfig.ambient_lighting.red = val[0];
        moduleConfig.has_ambient_lighting = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_AMBIENT_GREEN:
        REQ_LEN(1);
        moduleConfig.ambient_lighting.green = val[0];
        moduleConfig.has_ambient_lighting = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_AMBIENT_BLUE:
        REQ_LEN(1);
        moduleConfig.ambient_lighting.blue = val[0];
        moduleConfig.has_ambient_lighting = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;

    /* ── ModuleConfig.Paxcounter (B3-P4) ─────────────────────────── */
    case IPC_CFG_PAX_ENABLED:
        REQ_LEN(1); REQ_BOOL_RANGE();
        moduleConfig.paxcounter.enabled = (val[0] != 0u);
        moduleConfig.has_paxcounter = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
        push_result(seq, key, kResultOK);
        return;
    case IPC_CFG_PAX_UPDATE_INTERVAL:
        REQ_LEN(4);
        moduleConfig.paxcounter.paxcounter_update_interval = *(const uint32_t *)val;
        moduleConfig.has_paxcounter = true;
        s_pending_segments |= SEGMENT_MODULECONFIG;
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
