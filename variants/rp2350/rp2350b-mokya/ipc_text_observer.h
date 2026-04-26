/* ipc_text_observer.h — Push received MeshPackets onto the c0→c1 IPC ring
 * as IPC_MSG_RX_TEXT so Core 1 can show them in LVGL.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Wire any IPC observers that need post-setupModules registration. Called
 * from main.cpp after setupModules() so textMessageModule (and friends)
 * exist. Idempotent — safe to call once per boot. */
void mokya_register_ipc_observers(void);

#ifdef __cplusplus
}
#endif
