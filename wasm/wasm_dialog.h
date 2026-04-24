// Non-blocking prompt dialog for the WASM build. Native's draw_prompt_dialog
// (src/replay_conflict.c:54) spins in its own while(!WindowShouldClose()) loop,
// which cannot work under emscripten_set_main_loop. This module keeps the same
// visual design (scrim + rounded box + numbered options + hint) but drives the
// state machine one frame at a time.
//
// Usage:
//   wasm_dialog_begin(&d, "TITLE", "  -  subtitle", {"A","B","C"}, 3);
//   ...each frame, after HUD, before EndDrawing:
//     int choice = wasm_dialog_draw_and_poll(&d, theme, font_label, font_value);
//     if (choice > 0) { ...apply choice; dialog is auto-closed... }
//
// While d.active is true, the frame callback must suppress other input.

#ifndef WASM_DIALOG_H
#define WASM_DIALOG_H

#include "raylib.h"
#include "theme.h"

#define WASM_DIALOG_MAX_OPTIONS 3
#define WASM_DIALOG_STR_MAX     64

typedef struct {
    bool active;
    char title[WASM_DIALOG_STR_MAX];
    char subtitle[WASM_DIALOG_STR_MAX];
    char options[WASM_DIALOG_MAX_OPTIONS][WASM_DIALOG_STR_MAX];
    int  option_count;
} wasm_dialog_t;

void wasm_dialog_begin(wasm_dialog_t *d,
                        const char *title, const char *subtitle,
                        const char **options, int count);

// Returns 0 if still pending, 1..N when an option is pressed (dialog is then
// marked inactive automatically). Caller must draw this inside a BeginDrawing
// block (does not wrap its own).
int  wasm_dialog_draw_and_poll(wasm_dialog_t *d, const theme_t *theme,
                                Font font_label, Font font_value);

#endif
