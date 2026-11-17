#pragma once

/**
 * @file tui_capi.h
 * @brief C ABI for embedding the agent monitor/chat screens
 *        (tether_terminal_ui) in non-C++ hosts — e.g. Rust via FFI.
 *
 * Each `*_start` spawns a dedicated UI thread that owns ncurses and runs
 * the PanelScreen loop; the caller feeds events with `*_push` from any
 * thread (they are queued internally).  Only ONE screen may be active at
 * a time — a second start() would fight over the terminal.
 *
 * Outbound user input arrives through the callbacks:
 *  - monitor: `on_answer` fires with the ANSWER payload when the user
 *    completes an ASK question (option index as digits, or free text).
 *  - chat:    `on_send` fires with the SEND payload for each submitted
 *    message; when the UI ends (user quit or close) it fires once more
 *    with a NULL payload so a blocked receiver can exit.
 *
 * `*_close` stops the UI thread (also ends the loop on 'q'/Esc) and
 * frees the handle.  Do not push after close.
 */

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TetherAgentMonitor TetherAgentMonitor;
typedef void (*TetherTuiAnswerFn)(const char* payload, void* user);

/**Start the agent monitor screen titled `title` (never NULL). */
TetherAgentMonitor* tether_monitor_start(const char* title,
                                         TetherTuiAnswerFn on_answer,
                                         void* user);

/**Queue one event frame (kind/payload, e.g. "STEP", "12\t3\t4"). */
void tether_monitor_push(TetherAgentMonitor* mon,
                         const char* kind, const char* payload);

/**Queue backend stderr text for the log pane. */
void tether_monitor_stderr(TetherAgentMonitor* mon, const char* text);

/**Stop the UI thread and free the handle. */
void tether_monitor_close(TetherAgentMonitor* mon);

typedef struct TetherAgentChat TetherAgentChat;
typedef void (*TetherTuiSendFn)(const char* payload, void* user);

/**Start the chat screen titled `title` (never NULL). */
TetherAgentChat* tether_chat_start(const char* title,
                                   TetherTuiSendFn on_send, void* user);

/**Queue one chat frame (kind/payload: READY/TOOL/TEXT/COST/AWAIT/TURN/ERR). */
void tether_chat_push(TetherAgentChat* chat,
                      const char* kind, const char* payload);

/**Stop the UI thread and free the handle. */
void tether_chat_close(TetherAgentChat* chat);

#ifdef __cplusplus
}
#endif
