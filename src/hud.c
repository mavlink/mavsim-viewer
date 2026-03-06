#include "hud.h"
#include "raylib.h"
#include "raymath.h"

#ifdef _WIN32
// windows.h defines DrawText as DrawTextA, conflicting with raylib
#undef DrawText
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

#define BAR_HEIGHT 130
#define INSTRUMENT_RADIUS 45
#define INSTRUMENT_PADDING 12

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

void hud_init(hud_t *h) {
    h->flight_time_s = 0.0f;
    h->timing_active = false;
}

void hud_update(hud_t *h, float dt, bool connected) {
    if (connected && !h->timing_active) {
        h->timing_active = true;
    }
    if (h->timing_active) {
        h->flight_time_s += dt;
    }
}

static void draw_compass(float cx, float cy, float radius, float heading_deg, view_mode_t vm) {
    bool rez = (vm == VIEW_REZ);
    bool synth = (vm == VIEW_1988);
    Color bg = rez ? REZ_BG : synth ? SYNTH_BG : (Color){30, 30, 30, 220};
    Color border = rez ? REZ_BORDER : synth ? SYNTH_BORDER : (Color){100, 100, 100, 255};
    Color tick_major = rez ? REZ_ACCENT : synth ? SYNTH_HIGHLIGHT : WHITE;
    Color tick_minor = rez ? REZ_ACCENT_DIM : synth ? (Color){ 10, 120, 160, 255 } : (Color){160, 160, 160, 255};
    Color text_color = rez ? REZ_ACCENT : synth ? SYNTH_HIGHLIGHT : WHITE;
    Color north_color = rez ? REZ_ORANGE : synth ? SYNTH_ACCENT : RED;
    Color pointer_color = rez ? REZ_ORANGE : synth ? SYNTH_ACCENT : RED;

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
        int tw = MeasureText(labels[i], 12);
        Color lc = (i == 0) ? north_color : text_color;
        DrawText(labels[i], (int)(lx - tw / 2), (int)(ly - 6), 12, lc);
    }

    DrawTriangle((Vector2){cx, cy - radius - 4},
                 (Vector2){cx + 5, cy - radius + 6},
                 (Vector2){cx - 5, cy - radius + 6}, pointer_color);

    char hdg_buf[8];
    snprintf(hdg_buf, sizeof(hdg_buf), "%03d", ((int)heading_deg % 360 + 360) % 360);
    int tw = MeasureText(hdg_buf, 12);
    DrawText(hdg_buf, (int)(cx - tw / 2), (int)(cy + radius * 0.6f), 12, text_color);
}

static void draw_attitude(float cx, float cy, float radius, float roll_deg, float pitch_deg, view_mode_t vm) {
    bool rez = (vm == VIEW_REZ);
    bool synth = (vm == VIEW_1988);
    Color bg = rez ? REZ_BG : synth ? SYNTH_BG : (Color){30, 30, 30, 220};
    Color border = rez ? REZ_BORDER : synth ? SYNTH_BORDER : (Color){100, 100, 100, 255};
    Color horizon_color = rez ? REZ_ACCENT : synth ? (Color){ 112, 40, 21, 255 } : WHITE;
    Color pitch_bar_color = rez ? REZ_ACCENT_DIM : synth ? SYNTH_ACCENT_DIM : (Color){200, 200, 200, 160};
    Color wing_color = rez ? REZ_ORANGE : synth ? SYNTH_HIGHLIGHT : YELLOW;
    Color roll_tri_color = rez ? REZ_ACCENT : synth ? SYNTH_ACCENT : WHITE;
    Color sky_color = rez ? (Color){ 0, 40, 45, 200 } : synth ? (Color){ 0, 4, 12, 200 } : (Color){ 80, 140, 200, 200 };
    Color gnd_color = rez ? (Color){ 20, 10, 2, 200 } : synth ? (Color){ 251, 153, 54, 200 } : (Color){ 120, 85, 50, 200 };

    DrawCircle((int)cx, (int)cy, radius, bg);

    float pitch_clamped = pitch_deg;
    if (pitch_clamped > 40.0f) pitch_clamped = 40.0f;
    if (pitch_clamped < -40.0f) pitch_clamped = -40.0f;

    float pitch_px = pitch_clamped * (radius * 0.6f / 40.0f);

    float roll_rad = -roll_deg * DEG2RAD;
    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);

    float hw = radius * 1.5f;
    float hx1 = cx + (-hw) * cos_r - pitch_px * sin_r;
    float hy1 = cy + (-hw) * sin_r + pitch_px * cos_r;
    float hx2 = cx + ( hw) * cos_r - pitch_px * sin_r;
    float hy2 = cy + ( hw) * sin_r + pitch_px * cos_r;

    BeginScissorMode((int)(cx - radius), (int)(cy - radius),
                     (int)(radius * 2), (int)(radius * 2));
    {
        float big = radius * 3.0f;
        DrawCircle((int)cx, (int)cy, radius - 1, sky_color);
        Vector2 gnd_pts[4];
        float down_x = -sin_r;
        float down_y = cos_r;
        gnd_pts[0] = (Vector2){hx1, hy1};
        gnd_pts[1] = (Vector2){hx2, hy2};
        gnd_pts[2] = (Vector2){hx2 + down_x * big, hy2 + down_y * big};
        gnd_pts[3] = (Vector2){hx1 + down_x * big, hy1 + down_y * big};
        DrawTriangle(gnd_pts[0], gnd_pts[2], gnd_pts[1], gnd_color);
        DrawTriangle(gnd_pts[0], gnd_pts[3], gnd_pts[2], gnd_color);
    }
    EndScissorMode();

    DrawLineEx((Vector2){hx1, hy1}, (Vector2){hx2, hy2}, 2.0f, horizon_color);

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

void hud_draw(const hud_t *h, const vehicle_t *v, bool connected, int screen_w, int screen_h,
              int vehicle_idx, int vehicle_total, uint8_t sysid, view_mode_t view_mode) {

    bool rez = (view_mode == VIEW_REZ);
    bool synth = (view_mode == VIEW_1988);

    // Vehicle tab header (above the bottom bar, only for multi-vehicle)
    int tab_h = 0;
    if (vehicle_total > 1) {
        tab_h = 30;
        int tab_y = screen_h - BAR_HEIGHT - tab_h;
        DrawRectangle(0, tab_y, screen_w, tab_h, (Color){15, 15, 15, 200});
        DrawLineEx((Vector2){0, (float)tab_y}, (Vector2){(float)screen_w, (float)tab_y},
                   1.0f, (Color){80, 80, 80, 200});

        char tab_buf[32];
        snprintf(tab_buf, sizeof(tab_buf), "Vehicle %d/%d  [SYS %u]",
                 vehicle_idx + 1, vehicle_total, sysid);
        int tab_tw = MeasureText(tab_buf, 18);
        DrawText(tab_buf, (screen_w - tab_tw) / 2, tab_y + 6, 18, (Color){240, 240, 100, 255});
    }

    // Bottom bar
    int bar_y = screen_h - BAR_HEIGHT;
    Color bar_bg = rez ? (Color){4, 4, 8, 220} : synth ? SYNTH_BG : (Color){20, 20, 20, 180};
    Color bar_line = rez ? REZ_BORDER : synth ? SYNTH_BORDER : (Color){80, 80, 80, 200};
    DrawRectangle(0, bar_y, screen_w, BAR_HEIGHT, bar_bg);
    DrawLineEx((Vector2){0, (float)bar_y}, (Vector2){(float)screen_w, (float)bar_y},
               1.0f, bar_line);

    // Instruments — centered vertically in bar
    float inst_y = bar_y + BAR_HEIGHT / 2.0f - 5;
    float att_cx = INSTRUMENT_PADDING + INSTRUMENT_RADIUS + 8;

    draw_compass(att_cx, inst_y, INSTRUMENT_RADIUS, v->heading_deg, view_mode);

    // Attitude indicator — right of compass
    float adi_cx = att_cx + INSTRUMENT_RADIUS * 2 + INSTRUMENT_PADDING + 8;
    draw_attitude(adi_cx, inst_y, INSTRUMENT_RADIUS, v->roll_deg, v->pitch_deg, view_mode);

    // Themed text colors
    Color label_color = rez ? REZ_ACCENT_DIM : synth ? (Color){ 10, 120, 160, 255 } : (Color){150, 150, 150, 255};
    Color value_color = rez ? REZ_ACCENT : synth ? SYNTH_HIGHLIGHT : WHITE;
    Color dim_color = rez ? (Color){0, 100, 110, 255} : synth ? (Color){ 8, 80, 110, 255 } : (Color){100, 100, 100, 255};
    Color dim2_color = rez ? (Color){0, 80, 88, 255} : synth ? (Color){ 6, 60, 85, 255 } : (Color){120, 120, 120, 255};
    Color warn_color = rez ? REZ_ORANGE : synth ? SYNTH_ACCENT : RED;

    // Telemetry columns — each with its own buffer
    float tel_x = adi_cx + INSTRUMENT_RADIUS + 35;
    int label_y = bar_y + 15;
    int value_y = bar_y + 38;
    float col_w = 70.0f;
    if (screen_w < 800) col_w = 55.0f;

    // ALT
    {
        char b[16];
        DrawText("ALT", (int)tel_x, label_y, 11, label_color);
        snprintf(b, sizeof(b), "%.1f m", v->altitude_rel);
        DrawText(b, (int)tel_x, value_y, 14, value_color);
        tel_x += col_w;
    }

    // HDG
    {
        char b[8];
        DrawText("HDG", (int)tel_x, label_y, 11, label_color);
        snprintf(b, sizeof(b), "%03d", ((int)v->heading_deg % 360 + 360) % 360);
        DrawText(b, (int)tel_x, value_y, 14, value_color);
        tel_x += col_w - 10;
    }

    // ROLL
    {
        char b[8];
        DrawText("ROLL", (int)tel_x, label_y, 11, label_color);
        snprintf(b, sizeof(b), "%+.0f", v->roll_deg);
        DrawText(b, (int)tel_x, value_y, 14, value_color);
        tel_x += col_w - 10;
    }

    // PITCH
    {
        char b[8];
        DrawText("PITCH", (int)tel_x, label_y, 11, label_color);
        snprintf(b, sizeof(b), "%+.0f", v->pitch_deg);
        DrawText(b, (int)tel_x, value_y, 14, value_color);
        tel_x += col_w - 10;
    }

    // GS
    {
        char b[16];
        DrawText("GS", (int)tel_x, label_y, 11, label_color);
        snprintf(b, sizeof(b), "%.1f", v->ground_speed);
        DrawText(b, (int)tel_x, value_y, 14, value_color);
        tel_x += col_w - 10;
    }

    // VS
    {
        char b[16];
        DrawText("VS", (int)tel_x, label_y, 11, label_color);
        Color vs_color = (v->vertical_speed > 0.1f) ? GREEN :
                         (v->vertical_speed < -0.1f) ? warn_color : value_color;
        const char *arrow = (v->vertical_speed > 0.1f) ? "^" :
                            (v->vertical_speed < -0.1f) ? "v" : "";
        snprintf(b, sizeof(b), "%.1f%s", v->vertical_speed, arrow);
        DrawText(b, (int)tel_x, value_y, 14, vs_color);
        tel_x += col_w - 5;
    }

    // AS
    {
        char b[16];
        DrawText("AS", (int)tel_x, label_y, 11, label_color);
        if (v->airspeed > 0.1f) {
            snprintf(b, sizeof(b), "%.1f", v->airspeed);
            DrawText(b, (int)tel_x, value_y, 14, value_color);
        } else {
            DrawText("--", (int)tel_x, value_y, 14, dim_color);
        }
        tel_x += col_w - 10;
    }

    // TIME
    {
        char b[8];
        DrawText("TIME", (int)tel_x, label_y, 11, label_color);
        int mins = (int)(h->flight_time_s / 60.0f);
        int secs = (int)h->flight_time_s % 60;
        snprintf(b, sizeof(b), "%02d:%02d", mins, secs);
        DrawText(b, (int)tel_x, value_y, 14, value_color);
    }

    // Second row
    int row2_y = bar_y + 65;
    float row2_x = att_cx + INSTRUMENT_RADIUS + 35;
    {
        char b[48];
        snprintf(b, sizeof(b), "Pos: %.1f, %.1f, %.1f",
                 v->position.x, v->position.y, v->position.z);
        DrawText(b, (int)row2_x, row2_y, 11, dim2_color);
        row2_x += (float)MeasureText(b, 11) + 20;
    }

    {
        char b[16];
        snprintf(b, sizeof(b), "FPS: %d", GetFPS());
        int fps_x = screen_w - 70;
        DrawText(b, fps_x, row2_y, 11, dim_color);

        {
            char status_buf[32];
            if (connected) {
                snprintf(status_buf, sizeof(status_buf), "MAVLink SYS %u", sysid);
            } else {
                snprintf(status_buf, sizeof(status_buf), "Waiting for MAVLink...");
            }
            Color status_color = connected ? (Color){100, 200, 100, 255} : value_color;
            int msg_w = MeasureText(status_buf, 11);
            DrawText(status_buf, fps_x - msg_w - 15, row2_y, 11, status_color);
        }
    }
}

void hud_cleanup(hud_t *h) {
    (void)h;
}
