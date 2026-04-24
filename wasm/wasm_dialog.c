// WASM prompt dialog implementation. Measurement and draw logic mirrors
// src/replay_conflict.c:60-132 (draw_prompt_dialog) — same theme fields, same
// geometry, same font sizes. Difference: one frame per call, state persists
// across frames via wasm_dialog_t.

#include "wasm_dialog.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static void copy_str(char *dst, const char *src, size_t cap) {
    if (!src) { dst[0] = '\0'; return; }
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

void wasm_dialog_begin(wasm_dialog_t *d,
                        const char *title, const char *subtitle,
                        const char **options, int count)
{
    memset(d, 0, sizeof(*d));
    d->active = true;
    copy_str(d->title, title, WASM_DIALOG_STR_MAX);
    copy_str(d->subtitle, subtitle ? subtitle : "", WASM_DIALOG_STR_MAX);
    if (count > WASM_DIALOG_MAX_OPTIONS) count = WASM_DIALOG_MAX_OPTIONS;
    d->option_count = count;
    for (int i = 0; i < count; i++)
        copy_str(d->options[i], options[i], WASM_DIALOG_STR_MAX);
}

int wasm_dialog_draw_and_poll(wasm_dialog_t *d, const theme_t *theme,
                               Font font_label, Font font_value)
{
    if (!d->active) return 0;

    int choice = 0;
    // Poll keys 1..option_count
    for (int o = 0; o < d->option_count && o < 9; o++) {
        if (IsKeyPressed(KEY_ONE + o)) { choice = o + 1; break; }
    }

    int sw = GetScreenWidth();
    int sh = GetScreenHeight();
    float s = powf(sh / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;

    float fs_title = 15 * s, fs_option = 20 * s, fs_hint = 12 * s;
    float line_h = 36 * s, pad = 16 * s, inner_gap = 20 * s;

    float key_num_w = MeasureTextEx(font_value, "1", fs_option, 0.5f).x;
    float gap = 16 * s;
    float widest = 0;
    for (int o = 0; o < d->option_count; o++) {
        float w = key_num_w + gap +
                  MeasureTextEx(font_label, d->options[o], fs_option, 0.5f).x;
        if (w > widest) widest = w;
    }

    Vector2 tmw = MeasureTextEx(font_label, d->title, fs_title, 0.5f);
    Vector2 trw = MeasureTextEx(font_label, d->subtitle, fs_title, 0.5f);
    float title_total = tmw.x + trw.x;

    float box_w = 520 * s;
    float opts_block_h = line_h * d->option_count;
    float box_h = pad + fs_title + inner_gap + opts_block_h + inner_gap * 0.3f + fs_hint + pad * 0.5f;
    float cx = sw / 2.0f, cy = sh / 2.0f;
    float bx = cx - box_w / 2.0f, by = cy - box_h / 2.0f;

    DrawRectangle(0, 0, sw, sh, theme->prompt_scrim);

    Rectangle box = {bx, by, box_w, box_h};
    DrawRectangleRounded(box, 0.06f, 8, theme->prompt_box_bg);
    DrawRectangleRoundedLinesEx(box, 0.06f, 8, 1.5f * s, theme->prompt_border);

    float title_x = cx - title_total / 2.0f;
    DrawTextEx(font_label, d->title,
               (Vector2){title_x, by + pad}, fs_title, 0.5f, theme->prompt_title);
    DrawTextEx(font_label, d->subtitle,
               (Vector2){title_x + tmw.x, by + pad}, fs_title, 0.5f, theme->prompt_subtitle);

    float left = cx - widest / 2.0f;
    float opts_y = by + pad + fs_title + inner_gap;
    for (int o = 0; o < d->option_count; o++) {
        char num[2] = {(char)('1' + o), '\0'};
        DrawTextEx(font_value, num,
                   (Vector2){left, opts_y + o * line_h}, fs_option, 0.5f,
                   theme->prompt_key);
        DrawTextEx(font_label, d->options[o],
                   (Vector2){left + key_num_w + gap, opts_y + o * line_h},
                   fs_option, 0.5f, theme->prompt_text);
    }

    char hint[32];
    if (d->option_count == 3)      snprintf(hint, sizeof(hint), "Press 1, 2, or 3");
    else if (d->option_count == 2) snprintf(hint, sizeof(hint), "Press 1 or 2");
    else                           snprintf(hint, sizeof(hint), "Press 1");
    Vector2 hw = MeasureTextEx(font_label, hint, fs_hint, 0.5f);
    DrawTextEx(font_label, hint,
               (Vector2){cx - hw.x / 2.0f, by + box_h - pad * 0.5f - fs_hint},
               fs_hint, 0.5f, theme->prompt_hint);

    if (choice > 0) d->active = false;
    return choice;
}
