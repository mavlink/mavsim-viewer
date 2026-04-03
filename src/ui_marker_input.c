#include "ui_marker_input.h"
#include "replay_trail.h"
#include "raylib.h"
#include "theme.h"
#include <math.h>
#include <string.h>

void marker_input_update(marker_input_t *mi, user_markers_t *um,
                         data_source_t *source, vehicle_t *vehicle) {
    int ch;
    while ((ch = GetCharPressed()) != 0) {
        if (mi->len < HUD_MARKER_LABEL_MAX - 1 && ch >= 32 && ch < 127) {
            mi->buf[mi->len++] = (char)ch;
            mi->buf[mi->len] = '\0';
        }
    }
    if (IsKeyPressed(KEY_BACKSPACE) && mi->len > 0) {
        mi->buf[--mi->len] = '\0';
    }
    if (IsKeyPressed(KEY_ENTER)) {
        if (mi->target >= 0 && mi->target < um->count) {
            memcpy(um->labels[mi->target], mi->buf, HUD_MARKER_LABEL_MAX);
            data_source_seek(source, um->times[mi->target]);
            replay_sync_vehicle(source, vehicle);
        }
        um->current = mi->target;
        mi->active = false;
    }
    if (IsKeyPressed(KEY_ESCAPE)) {
        if (mi->target >= 0 && mi->target < um->count) {
            data_source_seek(source, um->times[mi->target]);
            replay_sync_vehicle(source, vehicle);
        }
        um->current = mi->target;
        mi->active = false;
    }
}

void marker_input_draw(const marker_input_t *mi, Font font_label, Font font_value,
                       const theme_t *theme, int screen_w, int screen_h) {
    int sw = screen_w, sh = screen_h;
    float s = powf(sh / 720.0f, 0.7f);

    const theme_t *th = theme;
    Color scrim_col    = th->prompt_scrim;
    Color box_bg       = th->prompt_box_bg;
    Color box_border   = th->prompt_border;
    Color prompt_col   = th->prompt_subtitle;
    Color hint_col     = th->prompt_hint;
    Color field_bg     = th->hud_bg;
    Color field_border = th->hud_border;
    Color text_col     = th->prompt_text;
    Color cursor_col   = th->hud_accent;

    DrawRectangle(0, 0, sw, sh, scrim_col);

    float box_w = 520 * s, box_h = 110 * s;
    float bx = (sw - box_w) / 2, by = (sh - box_h) / 2;
    Rectangle box = {bx, by, box_w, box_h};

    DrawRectangleRounded(box, 0.06f, 8, box_bg);
    DrawRectangleRoundedLinesEx(box, 0.06f, 8, 1.5f * s, box_border);

    float prompt_fs = 15 * s;
    DrawTextEx(font_label, "MARKER LABEL",
               (Vector2){bx + 16 * s, by + 12 * s}, prompt_fs, 0.5f, prompt_col);

    float hint_fs = 12 * s;
    float hint_w = MeasureTextEx(font_label, "Enter to confirm  |  Esc to cancel", hint_fs, 0.5f).x;
    DrawTextEx(font_label, "Enter to confirm  |  Esc to cancel",
               (Vector2){bx + box_w - hint_w - 16 * s, by + 12 * s}, hint_fs, 0.5f, hint_col);

    float field_x = bx + 16 * s, field_y = by + 40 * s;
    float field_w = box_w - 32 * s, field_h = 44 * s;
    DrawRectangleRounded((Rectangle){field_x, field_y, field_w, field_h},
                         0.08f, 6, field_bg);
    DrawRectangleRoundedLinesEx((Rectangle){field_x, field_y, field_w, field_h},
                         0.08f, 6, 1.0f * s, field_border);

    float input_fs = 20 * s;
    float text_y = field_y + (field_h - input_fs) / 2;
    DrawTextEx(font_value, mi->buf,
               (Vector2){field_x + 12 * s, text_y}, input_fs, 0.5f, text_col);

    Vector2 tw = MeasureTextEx(font_value, mi->buf, input_fs, 0.5f);
    if ((int)(GetTime() * 2.0) % 2 == 0) {
        float cx = field_x + 12 * s + tw.x + 2;
        DrawRectangle((int)cx, (int)(text_y), (int)(2 * s), (int)input_fs, cursor_col);
    }
}

void marker_input_begin(marker_input_t *mi, int marker_idx,
                        data_source_t *source) {
    mi->active = true;
    mi->target = marker_idx;
    mi->buf[0] = '\0';
    mi->len = 0;
    source->playback.paused = true;
}
