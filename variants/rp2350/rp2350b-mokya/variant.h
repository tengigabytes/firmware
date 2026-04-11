// MokyaLora Rev A — RP2350B variant for Meshtastic
// Core 0: LoRa-only modem — no display, GPS, I2C, or sensors on this core.
// LoRa: SX1262 on SPI1 (GPIO 24-27), TCXO 1.8V on DIO3, DIO2 as HW RF switch

// Bring IpcSerialStream's full class definition + `extern Serial` into every
// Meshtastic TU (configuration.h pulls variant.h in after <Arduino.h>).
// Without this, SerialConsole.cpp and friends can't resolve `Serial` now that
// Arduino-Pico's built-in SerialUSB is disabled via -DNO_USB.
#include "ipc_serial_stub.h"

// LoRa SX1262 — SPI1
#define USE_SX1262

#undef LORA_SCK
#undef LORA_MISO
#undef LORA_MOSI
#undef LORA_CS

#define LORA_SCK    26
#define LORA_MISO   24
#define LORA_MOSI   27
#define LORA_CS     25

#define LORA_DIO0   RADIOLIB_NC
#define LORA_RESET  23
#define LORA_DIO1   29      // IRQ — connected to MCU
#define LORA_DIO2   RADIOLIB_NC  // DIO2 wired directly to PE4259 RF switch, not MCU

#ifdef USE_SX1262
#define SX126X_CS           LORA_CS
#define SX126X_DIO1         LORA_DIO1
#define SX126X_BUSY         28      // BUSY pin
#define SX126X_RESET        LORA_RESET
#define SX126X_DIO2_AS_RF_SWITCH    // SetDIO2AsRfSwitchCtrl — no MCU GPIO needed
#define SX126X_DIO3_TCXO_VOLTAGE 1.8
#endif

// No I2C on this core — MESHTASTIC_EXCLUDE_I2C prevents Wire init entirely,
// avoiding GPIO 26/27 conflict between Wire1 defaults and SPI1 SCK/MOSI.

// Framework-side debug output is disabled: DEBUG_RP2040_PORT must stay
// undefined so Arduino-Pico's DEBUGV macro expands to a no-op. With -DNO_USB
// the stock SerialUSB instance is gone; Core 0 uses Meshtastic's own log
// system (RedirectablePrint) which is plumbed to IpcSerialStream instead.

// No battery ADC (BQ25622 handles via I2C on Core 1)
#undef BATTERY_PIN
