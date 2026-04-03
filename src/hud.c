#include "hud.h"
#include "hud_help.h"
#include "hud_instruments.h"
#include "hud_transport.h"
#include "hud_telemetry.h"
#include "theme.h"
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
        h->sim_time_s = (float)(time_usec / 1000000.0);
    } else if (connected) {
        h->sim_time_s += dt;
    }
    // Tick toast timer
    if (h->toast_timer > 0.0f)
        h->toast_timer -= dt;
}

void hud_toast(hud_t *h, const char *text, float duration_s) {
    snprintf(h->toast_text, sizeof(h->toast_text), "%s", text);
    h->toast_timer = duration_s;
    h->toast_total = duration_s;
    h->toast_color = (Color){0, 0, 0, 0};  // use default
}

void hud_toast_color(hud_t *h, const char *text, float duration_s, Color color) {
    snprintf(h->toast_text, sizeof(h->toast_text), "%s", text);
    h->toast_timer = duration_s;
    h->toast_total = duration_s;
    h->toast_color = color;
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
                                const playback_state_t *pb, int selected,
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
    float vnum_x = 8 * scale;
    DrawTextEx(font_value, vnum, (Vector2){vnum_x, (float)(row_y + (int)(10 * scale))}, fsv, 0.5f, pv->color);

    // CONF / PRSN / RMSE badges (skip for the selected/reference drone)
    if (pb && pidx != selected && pb->takeoff_conf >= 0.0f) {
        float badge_fs = fsl;
        float bx = vnum_x + MeasureTextEx(font_value, vnum, fsv, 0.5f).x + 8 * scale;
        float by = (float)(row_y + (int)(10 * scale));

        char conf_buf[16];
        snprintf(conf_buf, sizeof(conf_buf), "CONF %d%%", (int)(pb->takeoff_conf * 100));
        Color conf_c = (pb->takeoff_conf >= 0.8f) ? climb_color :
                       (pb->takeoff_conf >= 0.5f) ? value_color : warn_color;
        DrawTextEx(font_label, conf_buf, (Vector2){bx, by}, badge_fs, 0.5f, conf_c);
        bx += MeasureTextEx(font_label, conf_buf, badge_fs, 0.5f).x + 6 * scale;

        if (!isnan(pb->correlation)) {
            char prsn_buf[16];
            snprintf(prsn_buf, sizeof(prsn_buf), "PRSN %.2f", pb->correlation);
            Color prsn_c = (pb->correlation >= 0.7f) ? climb_color :
                           (pb->correlation >= 0.4f) ? value_color : warn_color;
            DrawTextEx(font_label, prsn_buf, (Vector2){bx, by}, badge_fs, 0.5f, prsn_c);
            bx += MeasureTextEx(font_label, prsn_buf, badge_fs, 0.5f).x + 6 * scale;
        }

        if (!isnan(pb->rmse)) {
            char rmse_buf[16];
            snprintf(rmse_buf, sizeof(rmse_buf), "RMSE %.1fm", pb->rmse);
            Color rmse_c = (pb->rmse <= 1.0f) ? climb_color :
                           (pb->rmse <= 5.0f) ? value_color : warn_color;
            DrawTextEx(font_label, rmse_buf, (Vector2){bx, by}, badge_fs, 0.5f, rmse_c);
        }
    }

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
              int selected, int screen_w, int screen_h, const theme_t *theme,
              int trail_mode,
              const hud_marker_data_t *markers,
              const hud_marker_data_t *sys_markers,
              bool ghost_mode, bool has_tier3, bool has_awaiting_gps) {

    // Semantic color variables from theme
    Color accent = theme->hud_accent;
    Color accent_dim = theme->hud_accent_dim;
    Color bg = theme->hud_bg;
    Color border = theme->hud_border;
    Color warn = theme->hud_warn;
    Color label_color = accent_dim;
    Color value_color = theme->hud_value;
    Color dim_color = theme->hud_dim;
    Color climb_color = theme->hud_climb;
    Color connected_color = theme->hud_connected;

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

    // Toast notification (fades in/out, always above other notices)
    float toast_h_used = 0.0f;
    if (h->toast_timer > 0.0f) {
        float toast_fs = 14 * s;
        Vector2 tw = MeasureTextEx(h->font_label, h->toast_text, toast_fs, 0.5f);
        // Fade: full opacity for most of duration, fade out in last 0.5s
        float fade = 1.0f;
        if (h->toast_timer < 0.5f)
            fade = h->toast_timer / 0.5f;
        Color base_tc = (h->toast_color.a > 0) ? h->toast_color : climb_color;
        Color toast_c = (Color){base_tc.r, base_tc.g, base_tc.b,
                                (unsigned char)(fade * 255)};
        float toast_y = (float)bar_y - tw.y - 8 * s;
        float toast_x = (float)(screen_w / 2) - tw.x / 2.0f;
        DrawTextEx(h->font_label, h->toast_text, (Vector2){toast_x, toast_y},
                   toast_fs, 0.5f, toast_c);
        toast_h_used = tw.y + 8 * s;
    }

    // ESTIMATED POSITION warning above timeline when any drone is Tier 3
    if (has_tier3 && is_replay_source) {
        const char *est_text = "ESTIMATED POSITION";
        float est_fs = 11 * s;
        Vector2 est_w = MeasureTextEx(h->font_label, est_text, est_fs, 0.5f);
        float est_y = (float)bar_y - est_w.y - 4 * s - toast_h_used;
        float est_x = (float)(screen_w / 2) - est_w.x / 2.0f;
        DrawTextEx(h->font_label, est_text, (Vector2){est_x, est_y}, est_fs, 0.5f, warn);
    }

    // AWAITING GPS warning when a drone is parked at origin because
    // GPOS data exists in the log but hasn't arrived in the stream yet
    if (has_awaiting_gps && is_replay_source) {
        const char *gps_text = "AWAITING GPS";
        float gps_fs = 11 * s;
        Vector2 gps_w = MeasureTextEx(h->font_label, gps_text, gps_fs, 0.5f);
        float gps_y_off = (has_tier3 ? gps_w.y + 4 * s : 0);
        float gps_y = (float)bar_y - gps_w.y - 4 * s - gps_y_off - toast_h_used;
        float gps_x = (float)(screen_w / 2) - gps_w.x / 2.0f;
        DrawTextEx(h->font_label, gps_text, (Vector2){gps_x, gps_y}, gps_fs, 0.5f, warn);
    }

    // Replay transport row (above the main HUD bar)
    if (is_replay_source) {
        hud_draw_transport(h, &sources[selected].playback,
                           sources[selected].connected, &vehicles[selected],
                           markers, sys_markers,
                           screen_w, (float)bar_y, (float)transport_h,
                           s, trail_mode, theme);
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
    hud_draw_compass(comp_cx, inst_y, inst_radius, v->heading_deg, theme, h->font_value);

    float adi_cx = comp_cx + inst_radius * 2 + inst_pad + 8 * s;
    hud_draw_attitude(adi_cx, inst_y, inst_radius, v->roll_deg, v->pitch_deg, theme);

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

    // Telemetry layout (shared by primary telemetry and status group)
    hud_telemetry_layout_t tlay = {
        .nav_start = nav_start, .nav_step = nav_step,
        .energy_start = energy_start, .energy_step = energy_step,
        .item_x0 = item_x0, .item_step = item_step,
        .sep3_x = sep3_x, .timer_x = timer_x, .status_x = status_x,
        .bar_y = bar_y, .primary_h = primary_h,
        .label_y = label_y, .value_y = value_y, .unit_y_off = unit_y_off,
        .fs_label = fs_label, .fs_value = fs_value, .fs_unit = fs_unit, .fs_dim = fs_dim,
        .scale = s,
        .label_color = label_color, .value_color = value_color,
        .dim_color = dim_color, .warn = warn,
        .climb_color = climb_color, .connected_color = connected_color,
    };

    hud_draw_telemetry(h, v, &tlay);

    // Numpad (only when vehicle_count > 1)
    if (vehicle_count > 1) {
        float np_y = bar_y + (primary_h / 2.0f) - (3 * (np_btn + np_gap)) / 2.0f;
        draw_numpad(h, vehicles, sources, vehicle_count, selected,
                    numpad_x, np_y, h->font_label, np_btn, np_gap, s);
    }

    hud_draw_status(h, v, &sources[selected], &tlay, ghost_mode);

    // Secondary row: position info
    {
        int row2_y = bar_y + (int)(78 * s);
        char b[48];
        snprintf(b, sizeof(b), "Pos: %.1f, %.1f, %.1f",
                 v->position.x, v->position.y, v->position.z);
        DrawTextEx(h->font_label, b, (Vector2){nav_group_x, (float)row2_y}, fs_dim, 0.5f, dim_color);
        if (sources[selected].ref_rejected) {
            Vector2 pw = MeasureTextEx(h->font_label, b, fs_dim, 0.5f);
            DrawTextEx(h->font_label, "  BAD REF", (Vector2){nav_group_x + pw.x, (float)row2_y}, fs_dim, 0.5f, warn);
        }
    }

    // Draw pinned vehicle secondary rows
    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;
        int row_y = bar_y + primary_h + (int)(4 * s) + (p * secondary_h);
        draw_secondary_row(h, &vehicles[pidx], pidx, &sources[pidx].playback,
                           selected, row_y, screen_w,
                           nav_start, nav_step, energy_start, energy_step,
                           h->font_label, h->font_value,
                           dim_color, value_color, warn, climb_color,
                           secondary_h, s);
    }

    // Help overlay
    if (h->show_help) {
        hud_draw_help(h->font_value, h->font_label, screen_w, screen_h, theme);
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
