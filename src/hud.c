#include "hud.h"
#include "asset_path.h"
#include "ulog_replay.h"
#include "raylib.h"
#include "raymath.h"

#ifdef _WIN32
// windows.h defines DrawText as DrawTextA, conflicting with raylib
#undef DrawText
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

#define INSTRUMENT_RADIUS 48
#define INSTRUMENT_PADDING 12

#define NUMPAD_BTN_SIZE 22
#define NUMPAD_GAP 2

// Rez HUD accent colors
#define REZ_ACCENT (Color){ 0, 204, 218, 255 }
#define REZ_ACCENT_DIM (Color){ 0, 204, 218, 140 }
#define REZ_ORANGE (Color){ 255, 106, 0, 255 }
#define REZ_BG (Color){ 8, 8, 12, 220 }
#define REZ_BORDER (Color){ 0, 204, 218, 100 }

// 1988 HUD accent colors
#define SYNTH_ACCENT (Color){ 255, 20, 100, 255 }
#define SYNTH_ACCENT_DIM (Color){ 255, 20, 100, 140 }
#define SYNTH_HIGHLIGHT (Color){ 21, 190, 254, 255 }
#define SYNTH_BG (Color){ 5, 5, 16, 220 }
#define SYNTH_BORDER (Color){ 255, 20, 100, 100 }

// Snow HUD accent colors (dark on bright, black outlines)
#define SNOW_ACCENT (Color){ 15, 15, 20, 255 }
#define SNOW_ACCENT_DIM (Color){ 60, 65, 75, 200 }
#define SNOW_ORANGE (Color){ 200, 40, 0, 255 }
#define SNOW_BG (Color){ 248, 248, 250, 235 }
#define SNOW_BORDER (Color){ 15, 15, 20, 140 }

void hud_init(hud_t *h) {
    h->sim_time_s = 0.0f;
    h->pinned_count = 0;
    h->show_help = false;
    for (int i = 0; i < HUD_MAX_PINNED; i++)
        h->pinned[i] = -1;

    char font_path[512];
    asset_path("fonts/JetBrainsMono-Medium.ttf", font_path, sizeof(font_path));
    h->font_value = LoadFontEx(font_path, 64, NULL, 0);
    asset_path("fonts/Inter-Medium.ttf", font_path, sizeof(font_path));
    h->font_label = LoadFontEx(font_path, 48, NULL, 0);

    SetTextureFilter(h->font_value.texture, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(h->font_label.texture, TEXTURE_FILTER_BILINEAR);

    if (h->font_value.glyphCount == 0)
        printf("Warning: could not load fonts/JetBrainsMono-Medium.ttf, using default font\n");
    if (h->font_label.glyphCount == 0)
        printf("Warning: could not load fonts/Inter-Medium.ttf, using default font\n");
}

void hud_update(hud_t *h, uint64_t time_usec, bool connected, float dt) {
    if (time_usec > 0) {
        // Use sim timestamp directly when available
        h->sim_time_s = (float)(time_usec / 1000000.0);
    } else if (connected) {
        // Accumulate wall time while connected (fallback when sim doesn't send timestamps)
        h->sim_time_s += dt;
    }
}

static void draw_compass(float cx, float cy, float radius, float heading_deg, view_mode_t vm, Font font_value) {
    bool rez = (vm == VIEW_REZ);
    bool synth = (vm == VIEW_1988);
    bool snow = (vm == VIEW_SNOW);
    Color bg = snow ? SNOW_BG : rez ? REZ_BG : synth ? SYNTH_BG : (Color){30, 30, 30, 220};
    Color border = snow ? SNOW_BORDER : rez ? REZ_BORDER : synth ? SYNTH_BORDER : (Color){100, 100, 100, 255};
    Color tick_major = snow ? SNOW_ACCENT : rez ? REZ_ACCENT : synth ? SYNTH_HIGHLIGHT : WHITE;
    Color tick_minor = snow ? SNOW_ACCENT_DIM : rez ? REZ_ACCENT_DIM : synth ? (Color){ 10, 120, 160, 255 } : (Color){160, 160, 160, 255};
    Color text_color = snow ? SNOW_ACCENT : rez ? REZ_ACCENT : synth ? SYNTH_HIGHLIGHT : WHITE;
    Color north_color = snow ? SNOW_ORANGE : rez ? REZ_ORANGE : synth ? SYNTH_ACCENT : RED;
    Color pointer_color = snow ? SNOW_ORANGE : rez ? REZ_ORANGE : synth ? SYNTH_ACCENT : RED;

    DrawCircle((int)cx, (int)cy, radius, bg);
    DrawCircleLines((int)cx, (int)cy, radius, border);

    for (int deg = 0; deg < 360; deg += 10) {
        float angle = (deg - heading_deg) * DEG2RAD - PI / 2.0f;
        float inner = (deg % 30 == 0) ? radius * 0.7f : radius * 0.82f;
        float outer = radius * 0.92f;
        float x1 = cx + cosf(angle) * inner;
        float y1 = cy + sinf(angle) * inner;
        float x2 = cx + cosf(angle) * outer;
        float y2 = cy + sinf(angle) * outer;
        Color tc = (deg % 90 == 0) ? tick_major : tick_minor;
        DrawLineEx((Vector2){x1, y1}, (Vector2){x2, y2}, (deg % 90 == 0) ? 2.0f : 1.0f, tc);
    }

    const char *labels[] = {"N", "E", "S", "W"};
    int label_degs[] = {0, 90, 180, 270};
    for (int i = 0; i < 4; i++) {
        float angle = (label_degs[i] - heading_deg) * DEG2RAD - PI / 2.0f;
        float lr = radius * 0.52f;
        float lx = cx + cosf(angle) * lr;
        float ly = cy + sinf(angle) * lr;
        Vector2 tw = MeasureTextEx(font_value, labels[i], 14, 0.5f);
        Color lc = (i == 0) ? north_color : text_color;
        DrawTextEx(font_value, labels[i], (Vector2){lx - tw.x / 2, ly - 7}, 14, 0.5f, lc);
    }

    DrawTriangle((Vector2){cx, cy - radius - 4},
                 (Vector2){cx + 5, cy - radius + 6},
                 (Vector2){cx - 5, cy - radius + 6}, pointer_color);

    char hdg_buf[8];
    snprintf(hdg_buf, sizeof(hdg_buf), "%03d", ((int)heading_deg % 360 + 360) % 360);
    Vector2 tw = MeasureTextEx(font_value, hdg_buf, 14, 0.5f);
    DrawTextEx(font_value, hdg_buf, (Vector2){cx - tw.x / 2, cy + radius * 0.6f}, 14, 0.5f, text_color);
}

static void draw_attitude(float cx, float cy, float radius, float roll_deg, float pitch_deg, view_mode_t vm) {
    bool rez = (vm == VIEW_REZ);
    bool synth = (vm == VIEW_1988);
    bool snow = (vm == VIEW_SNOW);
    Color bg = snow ? SNOW_BG : rez ? REZ_BG : synth ? SYNTH_BG : (Color){30, 30, 30, 220};
    Color border = snow ? SNOW_BORDER : rez ? REZ_BORDER : synth ? SYNTH_BORDER : (Color){100, 100, 100, 255};
    Color horizon_color = snow ? WHITE : rez ? REZ_ACCENT : synth ? (Color){ 112, 40, 21, 255 } : WHITE;
    Color pitch_bar_color = snow ? (Color){ 80, 90, 110, 180 } : rez ? REZ_ACCENT_DIM : synth ? SYNTH_ACCENT_DIM : (Color){200, 200, 200, 160};
    Color wing_color = snow ? SNOW_ORANGE : rez ? REZ_ORANGE : synth ? SYNTH_HIGHLIGHT : YELLOW;
    Color roll_tri_color = snow ? SNOW_ACCENT : rez ? REZ_ACCENT : synth ? SYNTH_ACCENT : WHITE;
    Color sky_color = snow ? (Color){ 160, 185, 215, 220 } : rez ? (Color){ 0, 40, 45, 200 } : synth ? (Color){ 0, 4, 12, 200 } : (Color){ 80, 140, 200, 200 };
    Color gnd_color = snow ? (Color){ 20, 30, 55, 220 } : rez ? (Color){ 20, 10, 2, 200 } : synth ? (Color){ 251, 153, 54, 200 } : (Color){ 120, 85, 50, 200 };

    DrawCircle((int)cx, (int)cy, radius, bg);

    float pitch_clamped = pitch_deg;
    if (pitch_clamped > 40.0f) pitch_clamped = 40.0f;
    if (pitch_clamped < -40.0f) pitch_clamped = -40.0f;

    float pitch_px = pitch_clamped * (radius * 0.6f / 40.0f);

    float roll_rad = -roll_deg * DEG2RAD;
    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);

    float r = radius - 1;
    DrawCircle((int)cx, (int)cy, r, sky_color);

    float pp = pitch_px;
    if (pp > r) pp = r;
    if (pp < -r) pp = -r;
    float half_chord = sqrtf(r * r - pp * pp);

    float ix1 = cx + (-half_chord) * cos_r - pp * (-sin_r);
    float iy1 = cy + (-half_chord) * sin_r + pp * cos_r;
    float ix2 = cx + ( half_chord) * cos_r - pp * (-sin_r);
    float iy2 = cy + ( half_chord) * sin_r + pp * cos_r;

    float a1 = atan2f(iy1 - cy, ix1 - cx);
    float a2 = atan2f(iy2 - cy, ix2 - cx);

    float gnd_center = atan2f(cos_r, -sin_r);

    float da1 = a1 - gnd_center;
    float da2 = a2 - gnd_center;
    while (da1 > PI) da1 -= 2*PI;
    while (da1 < -PI) da1 += 2*PI;
    while (da2 > PI) da2 -= 2*PI;
    while (da2 < -PI) da2 += 2*PI;

    float a_start, a_end;
    if (da1 < da2) { a_start = a1; a_end = a2; }
    else           { a_start = a2; a_end = a1; }

    {
        int arc_segs = 32;
        float sweep = a_end - a_start;
        if (sweep < 0) sweep += 2*PI;

        float fan_cx = (ix1 + ix2) * 0.5f;
        float fan_cy = (iy1 + iy2) * 0.5f;

        Vector2 prev = { cx + cosf(a_start) * r, cy + sinf(a_start) * r };
        for (int i = 1; i <= arc_segs; i++) {
            float a = a_start + sweep * (float)i / arc_segs;
            Vector2 cur = { cx + cosf(a) * r, cy + sinf(a) * r };
            DrawTriangle(prev, (Vector2){fan_cx, fan_cy}, cur, gnd_color);
            prev = cur;
        }
    }

    DrawLineEx((Vector2){ix1, iy1}, (Vector2){ix2, iy2}, 2.0f, horizon_color);

    int pitch_marks[] = {-20, -10, 10, 20};
    for (int p = 0; p < 4; p++) {
        float py = pitch_marks[p] * (radius * 0.6f / 40.0f);
        float bar_w = (pitch_marks[p] == 20 || pitch_marks[p] == -20) ? radius * 0.3f : radius * 0.2f;
        float bx1 = cx + (-bar_w) * cos_r - (pitch_px - py) * sin_r;
        float by1 = cy + (-bar_w) * sin_r + (pitch_px - py) * cos_r;
        float bx2 = cx + ( bar_w) * cos_r - (pitch_px - py) * sin_r;
        float by2 = cy + ( bar_w) * sin_r + (pitch_px - py) * cos_r;
        float dist_from_center = sqrtf(
            powf((bx1+bx2)/2 - cx, 2) + powf((by1+by2)/2 - cy, 2));
        if (dist_from_center < radius * 0.85f) {
            DrawLineEx((Vector2){bx1, by1}, (Vector2){bx2, by2}, 1.0f, pitch_bar_color);
        }
    }

    float wing_w = radius * 0.25f;
    float wing_gap = radius * 0.08f;
    DrawLineEx((Vector2){cx - wing_gap - wing_w, cy},
               (Vector2){cx - wing_gap, cy}, 3.0f, wing_color);
    DrawLineEx((Vector2){cx + wing_gap, cy},
               (Vector2){cx + wing_gap + wing_w, cy}, 3.0f, wing_color);
    DrawCircle((int)cx, (int)cy, 3, wing_color);

    float tri_r = radius - 3;
    DrawTriangle(
        (Vector2){cx, cy - tri_r},
        (Vector2){cx + 4, cy - tri_r + 7},
        (Vector2){cx - 4, cy - tri_r + 7},
        roll_tri_color);

    DrawCircleLines((int)cx, (int)cy, radius, border);
}

static void draw_numpad(const hud_t *h, const vehicle_t *vehicles,
                        const data_source_t *sources, int vehicle_count,
                        int selected, float numpad_x, float numpad_y,
                        Font font_label, float btn_size, float gap, float scale) {
    float fs = 12 * scale;
    for (int i = 0; i < 9; i++) {
        int row = i / 3;
        int col = i % 3;
        float bx = numpad_x + col * (btn_size + gap);
        float by = numpad_y + row * (btn_size + gap);
        int veh_idx = i;

        bool is_connected = (veh_idx < vehicle_count) && sources[veh_idx].connected;
        bool is_primary = (veh_idx == selected);
        bool is_pinned = false;
        for (int p = 0; p < h->pinned_count; p++)
            if (h->pinned[p] == veh_idx) is_pinned = true;

        Color btn_bg, btn_text;
        if (!is_connected && veh_idx >= vehicle_count) {
            btn_bg = (Color){10, 14, 20, 100};
            btn_text = (Color){200, 208, 218, 50};
        } else if (is_primary || is_pinned) {
            btn_bg = vehicles[veh_idx].color;
            btn_text = WHITE;
        } else if (is_connected) {
            btn_bg = (Color){10, 14, 20, 200};
            btn_text = (Color){200, 208, 218, 100};
        } else {
            btn_bg = (Color){10, 14, 20, 100};
            btn_text = (Color){200, 208, 218, 50};
        }

        DrawRectangleRounded(
            (Rectangle){bx, by, btn_size, btn_size},
            0.15f, 4, btn_bg);

        if (is_connected && !is_primary && !is_pinned) {
            DrawRectangleRoundedLinesEx(
                (Rectangle){bx, by, btn_size, btn_size},
                0.15f, 4, 1.0f,
                (Color){vehicles[veh_idx].color.r, vehicles[veh_idx].color.g,
                        vehicles[veh_idx].color.b, 80});
        }

        char nb[2] = { '1' + i, '\0' };
        Vector2 nw = MeasureTextEx(font_label, nb, fs, 0.5f);
        DrawTextEx(font_label, nb,
                   (Vector2){bx + btn_size/2 - nw.x/2, by + btn_size/2 - fs/2},
                   fs, 0.5f, btn_text);
    }
}

static void draw_secondary_row(const hud_t *h, const vehicle_t *pv, int pidx,
                                int row_y, int screen_w, float nav_start,
                                float nav_step, float energy_start, float energy_step,
                                Font font_label, Font font_value,
                                Color label_color_dim, Color value_color,
                                Color warn_color, Color climb_color,
                                int secondary_h, float scale) {
    float fsl = 14 * scale;
    float fsv = 18 * scale;
    float label_val_gap = 34 * scale;  // gap between label and value on same line

    // Separator line
    DrawLineEx((Vector2){0, (float)row_y},
               (Vector2){(float)screen_w, (float)row_y},
               1.0f, (Color){255, 255, 255, 20});

    // Color bar
    DrawRectangle(0, row_y, (int)(3 * scale), secondary_h, pv->color);

    // Vehicle number
    char vnum[4];
    snprintf(vnum, sizeof(vnum), "%d", pidx + 1);
    DrawTextEx(font_value, vnum, (Vector2){8 * scale, (float)(row_y + (int)(10 * scale))}, fsv, 0.5f, pv->color);

    int text_y = row_y + (int)(10 * scale);
    float label_off_y = (float)(row_y + (int)(4 * scale));
    char b[16];

    // NAV: HDG/YAW, ROLL, PITCH — same X positions as primary
    if (h->show_yaw) {
        DrawTextEx(font_label, "YAW", (Vector2){nav_start, label_off_y}, fsl, 0.5f, label_color_dim);
        float yaw_p = pv->heading_deg;
        if (yaw_p > 180.0f) yaw_p -= 360.0f;
        snprintf(b, sizeof(b), "%+.0f", yaw_p);
    } else {
        DrawTextEx(font_label, "HDG", (Vector2){nav_start, label_off_y}, fsl, 0.5f, label_color_dim);
        snprintf(b, sizeof(b), "%03d", ((int)pv->heading_deg % 360 + 360) % 360);
    }
    DrawTextEx(font_value, b, (Vector2){nav_start + label_val_gap, (float)text_y}, fsv, 0.5f, value_color);

    DrawTextEx(font_label, "ROLL", (Vector2){nav_start + nav_step, label_off_y}, fsl, 0.5f, label_color_dim);
    snprintf(b, sizeof(b), "%+.0f", pv->roll_deg);
    DrawTextEx(font_value, b, (Vector2){nav_start + nav_step + label_val_gap, (float)text_y}, fsv, 0.5f, value_color);

    DrawTextEx(font_label, "PITCH", (Vector2){nav_start + nav_step * 2, label_off_y}, fsl, 0.5f, label_color_dim);
    snprintf(b, sizeof(b), "%+.0f", pv->pitch_deg);
    DrawTextEx(font_value, b, (Vector2){nav_start + nav_step * 2 + label_val_gap + 8 * scale, (float)text_y}, fsv, 0.5f, value_color);

    // ENERGY: ALT, GS, AS, VS — same X positions as primary
    DrawTextEx(font_label, "ALT", (Vector2){energy_start, label_off_y}, fsl, 0.5f, label_color_dim);
    snprintf(b, sizeof(b), "%.1f", pv->altitude_rel);
    DrawTextEx(font_value, b, (Vector2){energy_start + label_val_gap, (float)text_y}, fsv, 0.5f, value_color);

    DrawTextEx(font_label, "GS", (Vector2){energy_start + energy_step, label_off_y}, fsl, 0.5f, label_color_dim);
    snprintf(b, sizeof(b), "%.1f", pv->ground_speed);
    DrawTextEx(font_value, b, (Vector2){energy_start + energy_step + label_val_gap - 12 * scale, (float)text_y}, fsv, 0.5f, value_color);

    DrawTextEx(font_label, "AS", (Vector2){energy_start + energy_step * 2, label_off_y}, fsl, 0.5f, label_color_dim);
    if (pv->airspeed > 0.1f) {
        snprintf(b, sizeof(b), "%.1f", pv->airspeed);
        DrawTextEx(font_value, b, (Vector2){energy_start + energy_step * 2 + label_val_gap - 12 * scale, (float)text_y}, fsv, 0.5f, value_color);
    } else {
        DrawTextEx(font_value, "--", (Vector2){energy_start + energy_step * 2 + label_val_gap - 12 * scale, (float)text_y}, fsv, 0.5f, label_color_dim);
    }

    float vs_x = energy_start + energy_step * 3;
    DrawTextEx(font_label, "VS", (Vector2){vs_x, label_off_y}, fsl, 0.5f, label_color_dim);
    Color vs_color = (pv->vertical_speed > 0.1f) ? climb_color :
                     (pv->vertical_speed < -0.1f) ? warn_color : value_color;
    const char *arrow = (pv->vertical_speed > 0.1f) ? "^" :
                        (pv->vertical_speed < -0.1f) ? "v" : "";
    snprintf(b, sizeof(b), "%.1f%s", pv->vertical_speed, arrow);
    DrawTextEx(font_value, b, (Vector2){vs_x + label_val_gap - 12 * scale, (float)text_y}, fsv, 0.5f, vs_color);
}

void hud_draw(const hud_t *h, const vehicle_t *vehicles,
              const data_source_t *sources, int vehicle_count,
              int selected, int screen_w, int screen_h, view_mode_t view_mode,
              bool ghost_mode, bool has_tier3) {

    bool rez = (view_mode == VIEW_REZ);
    bool synth = (view_mode == VIEW_1988);
    bool snow = (view_mode == VIEW_SNOW);

    // Semantic color variables
    Color accent, accent_dim, bg, border, warn;
    Color label_color, value_color, dim_color;
    Color climb_color, connected_color;

    if (snow) {
        accent = SNOW_ACCENT;  accent_dim = SNOW_ACCENT_DIM;
        bg = SNOW_BG;  border = SNOW_BORDER;  warn = SNOW_ORANGE;
    } else if (rez) {
        accent = REZ_ACCENT;  accent_dim = REZ_ACCENT_DIM;
        bg = REZ_BG;  border = REZ_BORDER;  warn = REZ_ORANGE;
    } else if (synth) {
        accent = SYNTH_ACCENT;  accent_dim = SYNTH_ACCENT_DIM;
        bg = SYNTH_BG;  border = SYNTH_BORDER;  warn = SYNTH_ACCENT;
    } else {
        accent = (Color){0, 180, 204, 255};
        accent_dim = (Color){0, 180, 204, 140};
        bg = (Color){10, 14, 20, 220};
        border = (Color){0, 180, 204, 64};
        warn = (Color){255, 106, 0, 255};
    }

    if (snow) {
        label_color = SNOW_ACCENT_DIM;
        value_color = (Color){10, 10, 15, 255};
        dim_color = (Color){80, 85, 95, 160};
        climb_color = (Color){ 0, 120, 50, 255 };
        connected_color = (Color){ 0, 130, 60, 255 };
    } else {
        label_color = accent_dim;
        value_color = (Color){200, 208, 218, 255};
        dim_color = (Color){200, 208, 218, 100};
        climb_color = GREEN;
        connected_color = (Color){100, 200, 100, 255};
    }

    // Scale factor: 1.0 at 720p, ~1.33 at 1080p, ~1.6 at 1440p
    float s = powf(screen_h / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;

    // Scaled font sizes
    float fs_label = 16 * s;
    float fs_value = 23 * s;
    float fs_unit  = 15 * s;
    float fs_dim   = 15 * s;
    float fs_sec_label = 14 * s;
    float fs_sec_value = 18 * s;

    // Dynamic bar height
    int primary_h = (int)(120 * s);
    int secondary_h = (int)(38 * s);
    bool is_replay_source = sources[selected].playback.duration_s > 0.0f;
    int transport_h = is_replay_source ? (int)(28 * s) : 0;
    int total_bar_h = transport_h + primary_h + (h->pinned_count > 0 ? h->pinned_count * secondary_h + (int)(4 * s) : 0);
    int bar_y = screen_h - total_bar_h;

    const vehicle_t *v = &vehicles[selected];

    // Bar background (full height including transport row)
    DrawRectangle(0, bar_y, screen_w, total_bar_h, bg);
    DrawLineEx((Vector2){0, (float)bar_y}, (Vector2){(float)screen_w, (float)bar_y}, 1.0f, border);

    // ESTIMATED POSITION warning above timeline when any drone is Tier 3
    if (has_tier3 && is_replay_source) {
        const char *est_text = "ESTIMATED POSITION";
        float est_fs = 11 * s;
        Vector2 est_w = MeasureTextEx(h->font_label, est_text, est_fs, 0.5f);
        float est_y = (float)bar_y - est_w.y - 4 * s;
        float est_x = (float)(screen_w / 2) - est_w.x / 2.0f;
        DrawTextEx(h->font_label, est_text, (Vector2){est_x, est_y}, est_fs, 0.5f, warn);
    }

    // Replay transport row (above the main HUD bar)
    if (is_replay_source) {
        const playback_state_t *pb = &sources[selected].playback;
        bool connected = sources[selected].connected;
        float tr_y = (float)bar_y;
        float tr_cy = tr_y + transport_h / 2.0f;
        float tr_pad = 12 * s;
        float icon_sz = 8 * s;

        // Separator line below transport row
        DrawLineEx((Vector2){0, tr_y + transport_h},
                   (Vector2){(float)screen_w, tr_y + transport_h},
                   1.0f, (Color){accent.r, accent.g, accent.b, 30});

        float cx = tr_pad;

        // Play/Pause icon
        if (pb->paused || !connected) {
            // Play triangle
            DrawTriangle(
                (Vector2){cx, tr_cy - icon_sz * 0.5f},
                (Vector2){cx, tr_cy + icon_sz * 0.5f},
                (Vector2){cx + icon_sz * 0.7f, tr_cy},
                connected ? accent : dim_color);
        } else {
            // Pause bars
            float bw = icon_sz * 0.25f;
            float bg_val = icon_sz * 0.12f;
            DrawRectangle((int)(cx), (int)(tr_cy - icon_sz * 0.4f),
                          (int)bw, (int)(icon_sz * 0.8f), accent);
            DrawRectangle((int)(cx + bw + bg_val * 2), (int)(tr_cy - icon_sz * 0.4f),
                          (int)bw, (int)(icon_sz * 0.8f), accent);
        }
        cx += icon_sz + 8 * s;

        // Speed
        char spd_buf[16];
        snprintf(spd_buf, sizeof(spd_buf), "%.1fx", pb->speed);
        DrawTextEx(h->font_value, spd_buf, (Vector2){cx, tr_cy - fs_dim * 0.45f},
                   fs_dim, 0.5f, (pb->speed != 1.0f) ? accent : value_color);
        Vector2 spd_w = MeasureTextEx(h->font_value, spd_buf, fs_dim, 0.5f);
        cx += spd_w.x + 12 * s;

        // Time (current)
        int pos_s = (int)pb->position_s;
        char pos_buf[16];
        snprintf(pos_buf, sizeof(pos_buf), "%d:%02d", pos_s / 60, pos_s % 60);
        DrawTextEx(h->font_value, pos_buf, (Vector2){cx, tr_cy - fs_dim * 0.45f},
                   fs_dim, 0.5f, value_color);
        Vector2 pos_w = MeasureTextEx(h->font_value, pos_buf, fs_dim, 0.5f);
        cx += pos_w.x + 8 * s;

        // Progress bar (stretches to fill available space)
        int dur_s = (int)pb->duration_s;
        char dur_buf[16];
        snprintf(dur_buf, sizeof(dur_buf), "%d:%02d", dur_s / 60, dur_s % 60);
        Vector2 dur_w = MeasureTextEx(h->font_value, dur_buf, fs_dim, 0.5f);

        // Reserve space for: duration text + icons + right padding
        float icons_w = (pb->looping ? 20 * s : 0) + 20 * s; // loop + interp
        float right_reserved = dur_w.x + 8 * s + icons_w + tr_pad;
        float prog_x = cx;
        float prog_w = (float)screen_w - cx - right_reserved;
        if (prog_w < 40 * s) prog_w = 40 * s;
        float prog_h = 3 * s;
        float prog_y = tr_cy - prog_h / 2.0f;

        DrawRectangleRounded(
            (Rectangle){prog_x, prog_y, prog_w, prog_h},
            0.5f, 4, (Color){accent.r, accent.g, accent.b, 40});
        if (pb->progress > 0.0f) {
            float fill_w = prog_w * pb->progress;
            if (fill_w < prog_h) fill_w = prog_h;
            DrawRectangleRounded(
                (Rectangle){prog_x, prog_y, fill_w, prog_h},
                0.5f, 4, accent);
        }
        // Playhead dot
        float dot_x = prog_x + prog_w * pb->progress;
        DrawCircle((int)dot_x, (int)(prog_y + prog_h / 2.0f), 3 * s, accent);

        // Flight mode markers on timeline
        if (pb->mode_changes && pb->mode_change_count > 0 && pb->duration_s > 0.0f) {
            float fs_marker = 9 * s;
            float last_label_x = -100.0f;  // track last drawn label to avoid overlap
            for (int i = 0; i < pb->mode_change_count; i++) {
                float t = pb->mode_changes[i].time_s / pb->duration_s;
                if (t < 0.0f || t > 1.0f) continue;
                float mx = prog_x + prog_w * t;

                // Tick mark: white and on top once past, dim accent when ahead
                bool past = (t <= pb->progress);
                Color tick_col = past
                    ? (Color){255, 255, 255, 220}
                    : (Color){accent.r, accent.g, accent.b, 80};
                DrawCircle((int)mx, (int)(prog_y + prog_h / 2.0f), 2 * s, tick_col);

                // Label (skip if too close to previous label)
                if (mx - last_label_x > 40 * s) {
                    const char *name = ulog_nav_state_name(pb->mode_changes[i].nav_state);
                    Vector2 tw = MeasureTextEx(h->font_label, name, fs_marker, 0.5f);
                    float lx = mx - tw.x / 2.0f;
                    if (lx < prog_x) lx = prog_x;
                    if (lx + tw.x > prog_x + prog_w) lx = prog_x + prog_w - tw.x;
                    DrawTextEx(h->font_label, name,
                               (Vector2){lx, prog_y - tw.y - 2 * s},
                               fs_marker, 0.5f, tick_col);
                    last_label_x = mx;
                }
            }
        }

        cx = prog_x + prog_w + 8 * s;

        // Duration
        DrawTextEx(h->font_value, dur_buf, (Vector2){cx, tr_cy - fs_dim * 0.45f},
                   fs_dim, 0.5f, dim_color);
        cx += dur_w.x + 8 * s;

        // Loop indicator
        if (pb->looping) {
            float lr = 5 * s;
            float loop_cx = cx + lr;
            // Circular arrow
            for (int d = 0; d < 300; d += 15) {
                float a1 = (float)d * DEG2RAD;
                float a2 = (float)(d + 15) * DEG2RAD;
                DrawLineEx(
                    (Vector2){loop_cx + cosf(a1) * lr, tr_cy + sinf(a1) * lr},
                    (Vector2){loop_cx + cosf(a2) * lr, tr_cy + sinf(a2) * lr},
                    1.5f * s, accent);
            }
            float arrow_a = 300 * DEG2RAD;
            float ax = loop_cx + cosf(arrow_a) * lr;
            float ay = tr_cy + sinf(arrow_a) * lr;
            DrawTriangle(
                (Vector2){ax + 3 * s, ay - 2 * s},
                (Vector2){ax - 1 * s, ay + 3 * s},
                (Vector2){ax - 2 * s, ay - 3 * s},
                accent);
            cx += 14 * s;
        }

        // Interpolation indicator: "I" label
        {
            Color interp_color = pb->interpolation ? accent : dim_color;
            DrawTextEx(h->font_value, "I", (Vector2){cx, tr_cy - fs_dim * 0.45f},
                       fs_dim, 0.5f, interp_color);
        }
    }

    // Offset the main HUD content below the transport row
    bar_y += transport_h;

    // Primary color bar on left edge
    DrawRectangle(0, bar_y, (int)(3 * s), primary_h, v->color);

    // Instruments -- centered vertically in primary area
    float inst_radius = INSTRUMENT_RADIUS * s;
    float inst_pad = INSTRUMENT_PADDING * s;
    float inst_y = bar_y + primary_h / 2.0f - 5 * s;
    float comp_cx = inst_pad + inst_radius + 8 * s;
    draw_compass(comp_cx, inst_y, inst_radius, v->heading_deg, view_mode, h->font_value);

    float adi_cx = comp_cx + inst_radius * 2 + inst_pad + 8 * s;
    draw_attitude(adi_cx, inst_y, inst_radius, v->roll_deg, v->pitch_deg, view_mode);

    // Separator drawing helper
    Color sep_color = (Color){accent.r, accent.g, accent.b, 50};
    int sep_top = bar_y + (int)(24 * s);
    int sep_bot = bar_y + primary_h - (int)(24 * s);
    float sep_margin = 16 * s;  // margin on each side of a separator

    // Right-anchored elements: compute positions first so we know where ENERGY ends
    float np_btn = NUMPAD_BTN_SIZE * s;
    float np_gap = NUMPAD_GAP * s;
    float status_x = (float)(screen_w - 16 * s - 110 * s);
    float numpad_total_w = 3 * (np_btn + np_gap) - np_gap;
    float numpad_x = status_x - 20 * s - numpad_total_w;
    float timer_x = numpad_x - 24 * s - 60 * s;

    // Separator positions
    float sep1_x = adi_cx + inst_radius + sep_margin;       // instruments | NAV
    float sep3_x = timer_x - sep_margin;                     // ENERGY | timer

    // Distribute all 7 telemetry items with equal step across the zone
    float tel_zone_w = sep3_x - sep1_x;
    float item_step = tel_zone_w / 7.0f;
    float item_x0 = sep1_x + sep_margin;

    float nav_start = item_x0;
    float nav_step = item_step;
    float energy_start = item_x0 + 3 * item_step;
    float energy_step = item_step;

    // Separator between NAV and ENERGY groups
    float sep2_x = sep1_x + 3 * item_step;

    float nav_group_x = nav_start;  // used by secondary rows

    // Draw separators
    DrawLineEx((Vector2){sep1_x, (float)sep_top}, (Vector2){sep1_x, (float)sep_bot}, 1.0f, sep_color);
    DrawLineEx((Vector2){sep2_x, (float)sep_top}, (Vector2){sep2_x, (float)sep_bot}, 1.0f, sep_color);
    DrawLineEx((Vector2){sep3_x, (float)sep_top}, (Vector2){sep3_x, (float)sep_bot}, 1.0f, sep_color);

    int label_y = bar_y + (int)(16 * s);
    int value_y = label_y + (int)(22 * s);
    float unit_y_off = (float)(int)(6 * s);

    // NAV group: HDG, ROLL, PITCH (evenly spaced)
    // HDG / YAW (Y key toggles)
    {
        char b[8];
        float x = nav_start + nav_step * 0;
        if (h->show_yaw) {
            DrawTextEx(h->font_label, "YAW", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
            float yaw = v->heading_deg;
            if (yaw > 180.0f) yaw -= 360.0f;
            snprintf(b, sizeof(b), "%+.0f", yaw);
        } else {
            DrawTextEx(h->font_label, "HDG", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
            snprintf(b, sizeof(b), "%03d", ((int)v->heading_deg % 360 + 360) % 360);
        }
        DrawTextEx(h->font_value, b, (Vector2){x, (float)value_y}, fs_value, 0.5f, value_color);
        Vector2 vw = MeasureTextEx(h->font_value, b, fs_value, 0.5f);
        Vector2 uw = MeasureTextEx(h->font_label, "deg", fs_unit, 0.5f);
        float boundary = item_x0 + 1 * item_step;
        if (x + vw.x + 3 + uw.x < boundary - 4 * s)
            DrawTextEx(h->font_label, "deg", (Vector2){x + vw.x + 3, (float)(value_y + unit_y_off)}, fs_unit, 0.5f, dim_color);
    }

    // ROLL
    {
        char b[8];
        float x = nav_start + nav_step * 1;
        DrawTextEx(h->font_label, "ROLL", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
        snprintf(b, sizeof(b), "%+.0f", v->roll_deg);
        DrawTextEx(h->font_value, b, (Vector2){x, (float)value_y}, fs_value, 0.5f, value_color);
    }

    // PITCH
    {
        char b[8];
        float x = nav_start + nav_step * 2;
        DrawTextEx(h->font_label, "PITCH", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
        snprintf(b, sizeof(b), "%+.0f", v->pitch_deg);
        DrawTextEx(h->font_value, b, (Vector2){x, (float)value_y}, fs_value, 0.5f, value_color);
    }

    // ENERGY group: ALT, GS, AS, VS (evenly spaced)
    // ALT
    {
        char b[16];
        float x = energy_start + energy_step * 0;
        DrawTextEx(h->font_label, "ALT", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
        snprintf(b, sizeof(b), "%.1f", v->altitude_rel);
        DrawTextEx(h->font_value, b, (Vector2){x, (float)value_y}, fs_value, 0.5f, value_color);
        Vector2 vw = MeasureTextEx(h->font_value, b, fs_value, 0.5f);
        Vector2 uw = MeasureTextEx(h->font_label, "m", fs_unit, 0.5f);
        float boundary = item_x0 + 4 * item_step;
        if (x + vw.x + 3 + uw.x < boundary - 4 * s)
            DrawTextEx(h->font_label, "m", (Vector2){x + vw.x + 3, (float)(value_y + unit_y_off)}, fs_unit, 0.5f, dim_color);
    }

    // GS
    {
        char b[16];
        float x = energy_start + energy_step * 1;
        DrawTextEx(h->font_label, "GS", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
        snprintf(b, sizeof(b), "%.1f", v->ground_speed);
        DrawTextEx(h->font_value, b, (Vector2){x, (float)value_y}, fs_value, 0.5f, value_color);
        Vector2 vw = MeasureTextEx(h->font_value, b, fs_value, 0.5f);
        Vector2 uw = MeasureTextEx(h->font_label, "m/s", fs_unit, 0.5f);
        float boundary = item_x0 + 5 * item_step;
        if (x + vw.x + 3 + uw.x < boundary - 4 * s)
            DrawTextEx(h->font_label, "m/s", (Vector2){x + vw.x + 3, (float)(value_y + unit_y_off)}, fs_unit, 0.5f, dim_color);
    }

    // AS
    {
        char b[16];
        float x = energy_start + energy_step * 2;
        DrawTextEx(h->font_label, "AS", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
        if (v->airspeed > 0.1f) {
            snprintf(b, sizeof(b), "%.1f", v->airspeed);
            DrawTextEx(h->font_value, b, (Vector2){x, (float)value_y}, fs_value, 0.5f, value_color);
            Vector2 vw = MeasureTextEx(h->font_value, b, fs_value, 0.5f);
            Vector2 uw = MeasureTextEx(h->font_label, "m/s", fs_unit, 0.5f);
            float boundary = item_x0 + 6 * item_step;
            if (x + vw.x + 3 + uw.x < boundary - 4 * s)
                DrawTextEx(h->font_label, "m/s", (Vector2){x + vw.x + 3, (float)(value_y + unit_y_off)}, fs_unit, 0.5f, dim_color);
        } else {
            DrawTextEx(h->font_value, "--", (Vector2){x, (float)value_y}, fs_value, 0.5f, dim_color);
        }
    }

    // VS
    {
        char b[16];
        float x = energy_start + energy_step * 3;
        DrawTextEx(h->font_label, "VS", (Vector2){x, (float)label_y}, fs_label, 0.5f, label_color);
        Color vs_color = (v->vertical_speed > 0.1f) ? climb_color :
                         (v->vertical_speed < -0.1f) ? warn : value_color;
        const char *arrow = (v->vertical_speed > 0.1f) ? "^" :
                            (v->vertical_speed < -0.1f) ? "v" : "";
        snprintf(b, sizeof(b), "%.1f%s", v->vertical_speed, arrow);
        DrawTextEx(h->font_value, b, (Vector2){x, (float)value_y}, fs_value, 0.5f, vs_color);
        Vector2 vw = MeasureTextEx(h->font_value, b, fs_value, 0.5f);
        Vector2 uw = MeasureTextEx(h->font_label, "m/s", fs_unit, 0.5f);
        if (x + vw.x + 3 + uw.x < sep3_x - 4 * s)
            DrawTextEx(h->font_label, "m/s", (Vector2){x + vw.x + 3, (float)(value_y + unit_y_off)}, fs_unit, 0.5f, dim_color);
    }

    // Timer (sim time from HIL_STATE_QUATERNION)
    {
        char b[16];
        float fs_timer = 22 * s;
        int total_secs = (int)h->sim_time_s;
        int mins = total_secs / 60;
        int secs = total_secs % 60;
        if (mins >= 60) {
            int hrs = mins / 60;
            mins = mins % 60;
            snprintf(b, sizeof(b), "%d:%02d:%02d", hrs, mins, secs);
        } else {
            snprintf(b, sizeof(b), "%02d:%02d", mins, secs);
        }
        Vector2 tw = MeasureTextEx(h->font_value, b, fs_timer, 0.5f);
        float tcx = timer_x + 30 * s - tw.x / 2;
        DrawTextEx(h->font_value, b, (Vector2){tcx, (float)(bar_y + primary_h / 2 - (int)(16 * s))}, fs_timer, 0.5f, value_color);
        DrawTextEx(h->font_label, "SIM", (Vector2){timer_x + 30 * s - 12 * s, (float)(bar_y + primary_h / 2 + (int)(10 * s))}, fs_dim, 0.5f, dim_color);
    }

    // Numpad (only when vehicle_count > 1)
    if (vehicle_count > 1) {
        float np_y = bar_y + (primary_h / 2.0f) - (3 * (np_btn + np_gap)) / 2.0f;
        draw_numpad(h, vehicles, sources, vehicle_count, selected,
                    numpad_x, np_y, h->font_label, np_btn, np_gap, s);
    }

    // Status group (right edge) — vertically centered as a block
    {
        bool connected = sources[selected].connected;
        bool is_replay = sources[selected].playback.duration_s > 0.0f;
        float line_gap = 4 * s;
        float status_line_h = fs_dim;
        float fps_line_h = fs_dim;
        float badge_h = ghost_mode ? (fs_dim * 0.8f + 4 * s) : 0;
        float badge_gap = ghost_mode ? line_gap : 0;
        float total_h = status_line_h + line_gap + fps_line_h + badge_gap + badge_h;
        float top_y = bar_y + primary_h / 2.0f - total_h / 2.0f;
        float cx = status_x + 14 * s;

        // Connection dot
        Color dot_color = connected ? connected_color : (Color){200, 60, 60, 255};
        DrawCircle((int)(status_x + 4 * s), (int)(top_y + 6 * s), 4 * s, dot_color);

        // Status text
        char status_buf[48];
        if (is_replay) {
            if (!connected)
                snprintf(status_buf, sizeof(status_buf), "Replay ended");
            else if (sources[selected].playback.paused)
                snprintf(status_buf, sizeof(status_buf), "Replay paused");
            else
                snprintf(status_buf, sizeof(status_buf), "Replaying log...");
        } else if (connected) {
            snprintf(status_buf, sizeof(status_buf), "MAVLink SYS %u", vehicles[selected].sysid);
        } else {
            snprintf(status_buf, sizeof(status_buf), "Waiting...");
        }
        DrawTextEx(h->font_label, status_buf, (Vector2){cx, top_y}, fs_dim, 0.5f,
                   connected ? connected_color : dim_color);

        // FPS
        char fps_buf[16];
        snprintf(fps_buf, sizeof(fps_buf), "FPS: %d", GetFPS());
        float fps_y = top_y + status_line_h + line_gap;
        DrawTextEx(h->font_label, fps_buf, (Vector2){cx, fps_y}, fs_dim, 0.5f, dim_color);

        // GHOST badge (small purple pill)
        if (ghost_mode) {
            float badge_fs = fs_dim * 0.8f;
            float badge_y = fps_y + fps_line_h + badge_gap;
            const char *badge_text = "GHOST";
            Vector2 tw = MeasureTextEx(h->font_label, badge_text, badge_fs, 0.5f);
            float pad_x = 10 * s, pad_y = 1.5f * s;
            float pill_w = tw.x + pad_x * 2;
            float pill_h = tw.y + pad_y * 2;
            float pill_x = cx + (MeasureTextEx(h->font_label, "FPS: 60", fs_dim, 0.5f).x - pill_w) / 2.0f;
            Color purple_bg = (Color){140, 80, 220, 180};
            Color purple_text = (Color){255, 255, 255, 230};
            Rectangle pill_r = {pill_x, badge_y, pill_w, pill_h};
            DrawRectangleRounded(pill_r, 1.0f, 12, purple_bg);
            DrawTextEx(h->font_label, badge_text, (Vector2){pill_x + pad_x, badge_y + pad_y}, badge_fs, 0.5f, purple_text);
        }
    }

    // Secondary row: position info
    {
        int row2_y = bar_y + (int)(78 * s);
        char b[48];
        snprintf(b, sizeof(b), "Pos: %.1f, %.1f, %.1f",
                 v->position.x, v->position.y, v->position.z);
        DrawTextEx(h->font_label, b, (Vector2){nav_group_x, (float)row2_y}, fs_dim, 0.5f, dim_color);
    }

    // Draw pinned vehicle secondary rows
    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;
        int row_y = bar_y + primary_h + (int)(4 * s) + (p * secondary_h);
        draw_secondary_row(h, &vehicles[pidx], pidx, row_y, screen_w,
                           nav_start, nav_step, energy_start, energy_step,
                           h->font_label, h->font_value,
                           dim_color, value_color, warn, climb_color,
                           secondary_h, s);
    }

    // Help overlay
    if (h->show_help) {
        // Dim the background
        DrawRectangle(0, 0, screen_w, screen_h, (Color){0, 0, 0, 160});

        float help_fs_title = 22 * s;
        float help_fs = 16 * s;
        float line_h = 24 * s;
        float col_gap = 24 * s;

        // Grouped shortcut entries: NULL key = section header
        typedef struct { const char *key; const char *action; } shortcut_entry_t;

        // Left column: VIEW + VEHICLE
        shortcut_entry_t left_col[] = {
            {NULL,          "VIEW"},
            {"C",           "Camera mode (Chase / FPV)"},
            {"V",           "View mode (Grid / Rez / Snow)"},
            {"F",           "Terrain texture"},
            {"K",           "Arm colors (classic / modern)"},
            {"O",           "Orthographic side panel"},
            {"Ctrl+D",      "Debug overlay"},
            {"Alt+1-7",     "Ortho views (1=perspective)"},
            {NULL,          "VEHICLE MODEL"},
            {"M",           "Switch variant (Shift: all)"},
            {NULL,          "MULTI-VEHICLE"},
            {"TAB",         "Next vehicle"},
            {"[ / ]",       "Prev / next vehicle"},
            {"1-9",         "Select vehicle"},
            {"Sh+1-9",      "Pin / unpin to HUD"},
        };

        // Right column: HUD + CAMERA + REPLAY
        shortcut_entry_t right_col[] = {
            {NULL,          "HUD"},
            {"H",           "Toggle HUD"},
            {"T",           "Cycle trail mode"},
            {"G",           "Ground track projection"},
            {"?",           "Toggle this help"},
            {NULL,          "CAMERA"},
            {"Drag",        "Orbit (chase mode)"},
            {"Scroll",      "Zoom FOV"},
            {"Alt+Scrl",    "Zoom ortho span"},
            {NULL,          "REPLAY"},
            {"Space",       "Pause / resume"},
            {"+/-",         "Playback speed"},
            {"<-/->",       "Seek 5s (Shift: 30s)"},
            {"L",           "Toggle loop"},
            {"I",           "Interpolation"},
            {"R",           "Restart"},
        };

        int left_count = sizeof(left_col) / sizeof(left_col[0]);
        int right_count = sizeof(right_col) / sizeof(right_col[0]);
        int max_rows = left_count > right_count ? left_count : right_count;

        float help_fs_group = 14 * s;
        float group_top_pad = 6 * s;

        // Measure max key width across both columns
        float max_key_w = 0;
        for (int i = 0; i < left_count; i++) {
            if (!left_col[i].key) continue;
            Vector2 kw = MeasureTextEx(h->font_value, left_col[i].key, help_fs, 0.5f);
            if (kw.x > max_key_w) max_key_w = kw.x;
        }
        for (int i = 0; i < right_count; i++) {
            if (!right_col[i].key) continue;
            Vector2 kw = MeasureTextEx(h->font_value, right_col[i].key, help_fs, 0.5f);
            if (kw.x > max_key_w) max_key_w = kw.x;
        }

        // Count group headers for extra padding
        int left_headers = 0, right_headers = 0;
        for (int i = 0; i < left_count; i++) if (!left_col[i].key) left_headers++;
        for (int i = 0; i < right_count; i++) if (!right_col[i].key) right_headers++;
        int max_headers = left_headers > right_headers ? left_headers : right_headers;

        float col_w = max_key_w + col_gap + 220 * s;
        float mid_gap = 32 * s;
        float panel_w = col_w * 2 + mid_gap + 40 * s;
        float panel_h = 40 * s + max_rows * line_h + max_headers * group_top_pad + 20 * s;
        float panel_x = (screen_w - panel_w) / 2.0f;
        float panel_y = (screen_h - panel_h) / 2.0f;

        // Panel background
        DrawRectangleRounded(
            (Rectangle){panel_x, panel_y, panel_w, panel_h},
            0.02f, 8, (Color){10, 14, 20, 230});
        DrawRectangleRoundedLinesEx(
            (Rectangle){panel_x, panel_y, panel_w, panel_h},
            0.02f, 8, 1.0f, border);

        // Title
        const char *title = "Keyboard Shortcuts";
        Vector2 tw = MeasureTextEx(h->font_label, title, help_fs_title, 0.5f);
        DrawTextEx(h->font_label, title,
                   (Vector2){panel_x + (panel_w - tw.x) / 2, panel_y + 12 * s},
                   help_fs_title, 0.5f, accent);

        // Draw a column of grouped entries
        float ey_start = panel_y + 40 * s;
        shortcut_entry_t *cols[] = { left_col, right_col };
        int counts[] = { left_count, right_count };

        for (int c = 0; c < 2; c++) {
            float key_x = panel_x + 20 * s + c * (col_w + mid_gap);
            float action_x = key_x + max_key_w + col_gap;
            float ey = ey_start;
            for (int i = 0; i < counts[c]; i++) {
                if (!cols[c][i].key) {
                    // Section header
                    if (i > 0) ey += group_top_pad;
                    DrawTextEx(h->font_label, cols[c][i].action,
                               (Vector2){key_x, ey}, help_fs_group, 0.5f, dim_color);
                } else {
                    DrawTextEx(h->font_value, cols[c][i].key,
                               (Vector2){key_x, ey}, help_fs, 0.5f, accent);
                    DrawTextEx(h->font_label, cols[c][i].action,
                               (Vector2){action_x, ey}, help_fs, 0.5f, value_color);
                }
                ey += line_h;
            }
        }
    }
}

int hud_bar_height(const hud_t *h, int screen_h) {
    float s = powf(screen_h / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;
    int primary_h = (int)(120 * s);
    int secondary_h = (int)(38 * s);
    int transport_h = h->is_replay ? (int)(28 * s) : 0;
    return transport_h + primary_h + (h->pinned_count > 0 ? h->pinned_count * secondary_h + (int)(4 * s) : 0);
}

void hud_cleanup(hud_t *h) {
    UnloadFont(h->font_value);
    UnloadFont(h->font_label);
}
