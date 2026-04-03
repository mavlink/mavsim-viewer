#include "hud_telemetry.h"
#include "raylib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void hud_draw_telemetry(const hud_t *h, const vehicle_t *v,
                        const hud_telemetry_layout_t *lay) {
    float s = lay->scale;
    float nav_start = lay->nav_start;
    float nav_step = lay->nav_step;
    float energy_start = lay->energy_start;
    float energy_step = lay->energy_step;
    float item_x0 = lay->item_x0;
    float item_step = lay->item_step;
    float sep3_x = lay->sep3_x;
    int label_y = lay->label_y;
    int value_y = lay->value_y;
    float unit_y_off = lay->unit_y_off;
    float fs_label = lay->fs_label;
    float fs_value = lay->fs_value;
    float fs_unit = lay->fs_unit;
    float fs_dim = lay->fs_dim;
    Color label_color = lay->label_color;
    Color value_color = lay->value_color;
    Color dim_color = lay->dim_color;
    Color warn = lay->warn;
    Color climb_color = lay->climb_color;

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
        int bar_y = lay->bar_y;
        int primary_h = lay->primary_h;
        float timer_x = lay->timer_x;
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
}

void hud_draw_status(const hud_t *h, const vehicle_t *v,
                     const data_source_t *source,
                     const hud_telemetry_layout_t *lay,
                     bool ghost_mode) {
    float s = lay->scale;
    float fs_dim = lay->fs_dim;
    Color dim_color = lay->dim_color;
    Color connected_color = lay->connected_color;
    int bar_y = lay->bar_y;
    int primary_h = lay->primary_h;
    float status_x = lay->status_x;

    bool connected = source->connected;
    bool is_replay = source->playback.duration_s > 0.0f;
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
        else if (source->playback.paused)
            snprintf(status_buf, sizeof(status_buf), "Replay paused");
        else
            snprintf(status_buf, sizeof(status_buf), "Replaying log...");
    } else if (connected) {
        snprintf(status_buf, sizeof(status_buf), "MAVLink SYS %u", v->sysid);
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
