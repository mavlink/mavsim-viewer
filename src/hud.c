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

static void draw_compass(float cx, float cy, float radius, float heading_deg) {
    DrawCircle((int)cx, (int)cy, radius, (Color){30, 30, 30, 220});
    DrawCircleLines((int)cx, (int)cy, radius, (Color){100, 100, 100, 255});

    for (int deg = 0; deg < 360; deg += 10) {
        float angle = (deg - heading_deg) * DEG2RAD - PI / 2.0f;
        float inner = (deg % 30 == 0) ? radius * 0.7f : radius * 0.82f;
        float outer = radius * 0.92f;
        float x1 = cx + cosf(angle) * inner;
        float y1 = cy + sinf(angle) * inner;
        float x2 = cx + cosf(angle) * outer;
        float y2 = cy + sinf(angle) * outer;
        Color tc = (deg % 90 == 0) ? WHITE : (Color){160, 160, 160, 255};
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
        Color lc = (i == 0) ? RED : WHITE;
        DrawText(labels[i], (int)(lx - tw / 2), (int)(ly - 6), 12, lc);
    }

    // Red triangle pointer at top
    DrawTriangle((Vector2){cx, cy - radius - 4},
                 (Vector2){cx + 5, cy - radius + 6},
                 (Vector2){cx - 5, cy - radius + 6}, RED);

    // Heading text inside the circle at bottom
    char hdg_buf[8];
    snprintf(hdg_buf, sizeof(hdg_buf), "%03d", ((int)heading_deg % 360 + 360) % 360);
    int tw = MeasureText(hdg_buf, 12);
    DrawText(hdg_buf, (int)(cx - tw / 2), (int)(cy + radius * 0.6f), 12, WHITE);
}

void hud_draw(const hud_t *h, const vehicle_t *v, bool connected, int screen_w, int screen_h,
              int vehicle_idx, int vehicle_total, uint8_t sysid) {
    (void)connected; // handled in second row below

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
    DrawRectangle(0, bar_y, screen_w, BAR_HEIGHT, (Color){20, 20, 20, 180});
    DrawLineEx((Vector2){0, (float)bar_y}, (Vector2){(float)screen_w, (float)bar_y},
               1.0f, (Color){80, 80, 80, 200});

    // Instruments — centered vertically in bar
    float inst_y = bar_y + BAR_HEIGHT / 2.0f - 5;
    float att_cx = INSTRUMENT_PADDING + INSTRUMENT_RADIUS + 8;

    draw_compass(att_cx, inst_y, INSTRUMENT_RADIUS, v->heading_deg);

    // Telemetry columns — each with its own buffer
    float tel_x = att_cx + INSTRUMENT_RADIUS + 35;
    int label_y = bar_y + 15;
    int value_y = bar_y + 38;
    float col_w = 70.0f;
    if (screen_w < 800) col_w = 55.0f;

    // ALT
    {
        char b[16];
        DrawText("ALT", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        snprintf(b, sizeof(b), "%.1f m", v->altitude_rel);
        DrawText(b, (int)tel_x, value_y, 14, WHITE);
        tel_x += col_w;
    }

    // HDG
    {
        char b[8];
        DrawText("HDG", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        snprintf(b, sizeof(b), "%03d", ((int)v->heading_deg % 360 + 360) % 360);
        DrawText(b, (int)tel_x, value_y, 14, WHITE);
        tel_x += col_w - 10;
    }

    // ROLL
    {
        char b[8];
        DrawText("ROLL", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        snprintf(b, sizeof(b), "%+.0f", v->roll_deg);
        DrawText(b, (int)tel_x, value_y, 14, WHITE);
        tel_x += col_w - 10;
    }

    // PITCH
    {
        char b[8];
        DrawText("PITCH", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        snprintf(b, sizeof(b), "%+.0f", v->pitch_deg);
        DrawText(b, (int)tel_x, value_y, 14, WHITE);
        tel_x += col_w - 10;
    }

    // GS
    {
        char b[16];
        DrawText("GS", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        snprintf(b, sizeof(b), "%.1f", v->ground_speed);
        DrawText(b, (int)tel_x, value_y, 14, WHITE);
        tel_x += col_w - 10;
    }

    // VS
    {
        char b[16];
        DrawText("VS", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        Color vs_color = (v->vertical_speed > 0.1f) ? GREEN :
                         (v->vertical_speed < -0.1f) ? RED : WHITE;
        const char *arrow = (v->vertical_speed > 0.1f) ? "^" :
                            (v->vertical_speed < -0.1f) ? "v" : "";
        snprintf(b, sizeof(b), "%.1f%s", v->vertical_speed, arrow);
        DrawText(b, (int)tel_x, value_y, 14, vs_color);
        tel_x += col_w - 5;
    }

    // AS
    {
        char b[16];
        DrawText("AS", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        if (v->airspeed > 0.1f) {
            snprintf(b, sizeof(b), "%.1f", v->airspeed);
            DrawText(b, (int)tel_x, value_y, 14, WHITE);
        } else {
            DrawText("--", (int)tel_x, value_y, 14, (Color){100, 100, 100, 255});
        }
        tel_x += col_w - 10;
    }

    // TIME
    {
        char b[8];
        DrawText("TIME", (int)tel_x, label_y, 11, (Color){150, 150, 150, 255});
        int mins = (int)(h->flight_time_s / 60.0f);
        int secs = (int)h->flight_time_s % 60;
        snprintf(b, sizeof(b), "%02d:%02d", mins, secs);
        DrawText(b, (int)tel_x, value_y, 14, WHITE);
    }

    // Second row
    int row2_y = bar_y + 65;
    float row2_x = att_cx + INSTRUMENT_RADIUS + 35;
    {
        char b[48];
        snprintf(b, sizeof(b), "Pos: %.1f, %.1f, %.1f",
                 v->position.x, v->position.y, v->position.z);
        DrawText(b, (int)row2_x, row2_y, 11, (Color){120, 120, 120, 255});
        row2_x += (float)MeasureText(b, 11) + 20;
    }

    {
        char b[16];
        snprintf(b, sizeof(b), "FPS: %d", GetFPS());
        int fps_x = screen_w - 70;
        DrawText(b, fps_x, row2_y, 11, (Color){100, 100, 100, 255});

        if (!connected) {
            const char *msg = "Waiting for MAVLink...";
            int msg_w = MeasureText(msg, 11);
            DrawText(msg, fps_x - msg_w - 15, row2_y, 11, RED);
        }
    }
}

void hud_cleanup(hud_t *h) {
    (void)h;
}
