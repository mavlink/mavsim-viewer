#include "hud_transport.h"
#include "hud.h"
#include "ulog_replay.h"
#include "vehicle.h"
#include "raylib.h"
#include "theme.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void hud_draw_transport(const hud_t *h,
                        const playback_state_t *pb, bool connected,
                        const vehicle_t *selected_vehicle,
                        const hud_marker_data_t *markers_all,
                        const hud_marker_data_t *sys_markers_all,
                        int marker_vehicle_count, int selected_idx,
                        int screen_w, float bar_y, float transport_h,
                        float scale, int trail_mode, const theme_t *theme) {
    float s = scale;
    Color accent = theme->hud_accent;
    Color bg = theme->hud_bg;
    Color value_color = theme->hud_value;
    Color dim_color = theme->hud_dim;
    float fs_dim = 15 * s;

    float tr_y = bar_y;
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
    // Playhead ring — colored to match the drone's current trail color
    float dot_x = prog_x + prog_w * pb->progress;
    float dot_y = prog_y + prog_h / 2.0f;
    {
        const vehicle_t *pv = selected_vehicle;
        Color ph_col = vehicle_marker_color(pv->roll_deg, pv->pitch_deg,
                                            pv->vertical_speed,
                                            sqrtf(pv->ground_speed * pv->ground_speed +
                                                  pv->vertical_speed * pv->vertical_speed),
                                            pv->trail_speed_max, theme, trail_mode,
                                            pv->color);
        float r_outer = 6.5f * s;
        float r_inner = 5.5f * s;
        DrawRing((Vector2){dot_x, dot_y}, r_inner, r_outer, 0, 360, 24, ph_col);
    }

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

    // Frame markers on timeline (colored diamonds with labels) — all drones
    {
        float fs_mlabel = 9 * s;
        float last_mlabel_x = -100.0f;
        for (int vi = 0; vi < marker_vehicle_count; vi++) {
            const hud_marker_data_t *markers = &markers_all[vi];
            if (!markers->times || markers->count == 0 || pb->duration_s <= 0.0f) continue;
            bool is_selected_drone = (vi == selected_idx);
            for (int i = 0; i < markers->count; i++) {
                float t = markers->times[i] / pb->duration_s;
                if (t < 0.0f || t > 1.0f) continue;
                float mx = prog_x + prog_w * t;
                float my = prog_y + prog_h / 2.0f;

                bool is_cur = is_selected_drone && (i == markers->current);
                Color mc = vehicle_marker_color(markers->roll[i], markers->pitch[i],
                                                markers->vert[i], markers->speed[i],
                                                markers->speed_max, theme, trail_mode,
                                                markers->color);
                if (is_cur) {
                    if (theme->thick_trails) {
                        mc.r = (unsigned char)(mc.r * 0.55f);
                        mc.g = (unsigned char)(mc.g * 0.55f);
                        mc.b = (unsigned char)(mc.b * 0.55f);
                    } else {
                        mc.r = (unsigned char)(mc.r + (230 - mc.r) * 0.7f);
                        mc.g = (unsigned char)(mc.g + (230 - mc.g) * 0.7f);
                        mc.b = (unsigned char)(mc.b + (230 - mc.b) * 0.7f);
                    }
                }
                mc.a = is_cur ? 255 : (is_selected_drone ? 220 : 100);

                float d = is_cur ? 5.0f * s : 3.5f * s;
                if (!is_selected_drone) d *= 0.7f;
                Vector2 diamond[4] = {
                    {mx, my - d}, {mx + d, my}, {mx, my + d}, {mx - d, my},
                };
                DrawTriangle(diamond[0], diamond[3], diamond[1], mc);
                DrawTriangle(diamond[1], diamond[3], diamond[2], mc);

                // Labels only for selected drone to avoid clutter
                if (is_selected_drone) {
                    char mlbl[56];
                    if (markers->labels && markers->labels[i][0] != '\0')
                        snprintf(mlbl, sizeof(mlbl), "%d:%s", i + 1, markers->labels[i]);
                    else
                        snprintf(mlbl, sizeof(mlbl), "%d", i + 1);
                    Vector2 mlw = MeasureTextEx(h->font_label, mlbl, fs_mlabel, 0.5f);
                    float min_gap = is_cur ? 0 : (mlw.x + 6 * s);
                    if (is_cur || mx - last_mlabel_x > min_gap) {
                        float lx = mx - mlw.x / 2.0f;
                        if (lx < prog_x) lx = prog_x;
                        if (lx + mlw.x > prog_x + prog_w) lx = prog_x + prog_w - mlw.x;
                        float ly = prog_y + prog_h + 3 * s;
                        if (is_cur) {
                            float px = 4 * s, py = 2 * s;
                            DrawRectangleRounded(
                                (Rectangle){lx - px, ly - py, mlw.x + px * 2, mlw.y + py * 2},
                                0.4f, 4, bg);
                        }
                        DrawTextEx(h->font_label, mlbl,
                                   (Vector2){lx, ly}, fs_mlabel, 0.5f, mc);
                        last_mlabel_x = mx;
                    }
                }
            }
        }
    }

    // System markers on timeline (squares) — all drones
    {
        float fs_mlabel = 9 * s;
        float last_slabel_x = -100.0f;
        for (int vi = 0; vi < marker_vehicle_count; vi++) {
            const hud_marker_data_t *sysm = &sys_markers_all[vi];
            if (!sysm->times || sysm->count == 0 || pb->duration_s <= 0.0f) continue;
            bool is_selected_drone = (vi == selected_idx);
            for (int i = 0; i < sysm->count; i++) {
                float t = sysm->times[i] / pb->duration_s;
                if (t < 0.0f || t > 1.0f) continue;
                float mx = prog_x + prog_w * t;
                float my = prog_y + prog_h / 2.0f;

                bool is_cur = is_selected_drone && sysm->selected && (i == sysm->current);
                Color mc = vehicle_marker_color(sysm->roll[i], sysm->pitch[i],
                                                sysm->vert[i], sysm->speed[i],
                                                sysm->speed_max, theme, trail_mode,
                                                sysm->color);
                if (is_cur) {
                    if (theme->thick_trails) {
                        mc.r = (unsigned char)(mc.r * 0.55f);
                        mc.g = (unsigned char)(mc.g * 0.55f);
                        mc.b = (unsigned char)(mc.b * 0.55f);
                    } else {
                        mc.r = (unsigned char)(mc.r + (230 - mc.r) * 0.7f);
                        mc.g = (unsigned char)(mc.g + (230 - mc.g) * 0.7f);
                        mc.b = (unsigned char)(mc.b + (230 - mc.b) * 0.7f);
                    }
                }
                mc.a = is_cur ? 255 : (is_selected_drone ? 200 : 80);

                float d = is_cur ? 5.0f * s : 3.5f * s;
                if (!is_selected_drone) d *= 0.7f;
                DrawRectangle((int)(mx - d), (int)(my - d), (int)(d * 2), (int)(d * 2), mc);

                // Labels only for selected drone
                if (is_selected_drone) {
                    char mlbl[56];
                    if (sysm->labels && sysm->labels[i][0] != '\0')
                        snprintf(mlbl, sizeof(mlbl), "S:%s", sysm->labels[i]);
                    else
                        snprintf(mlbl, sizeof(mlbl), "S%d", i + 1);
                    Vector2 mlw = MeasureTextEx(h->font_label, mlbl, fs_mlabel, 0.5f);
                    float min_gap = is_cur ? 0 : (mlw.x + 6 * s);
                    if (is_cur || mx - last_slabel_x > min_gap) {
                        float lx = mx - mlw.x / 2.0f;
                        if (lx < prog_x) lx = prog_x;
                        if (lx + mlw.x > prog_x + prog_w) lx = prog_x + prog_w - mlw.x;
                        float ly = prog_y - mlw.y - 3 * s;
                        if (is_cur) {
                            float px = 4 * s, py = 2 * s;
                            DrawRectangleRounded(
                                (Rectangle){lx - px, ly - py, mlw.x + px * 2, mlw.y + py * 2},
                                0.4f, 4, bg);
                        }
                        DrawTextEx(h->font_label, mlbl,
                                   (Vector2){lx, ly}, fs_mlabel, 0.5f, mc);
                        last_slabel_x = mx;
                    }
                }
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
