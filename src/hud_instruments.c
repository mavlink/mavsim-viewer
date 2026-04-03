#include "hud_instruments.h"
#include "raylib.h"
#include "theme.h"
#include <math.h>
#include <stdio.h>

void hud_draw_compass(float cx, float cy, float radius, float heading_deg, const theme_t *theme, Font font_value) {
    Color bg = theme->hud_bg;
    Color border = theme->hud_border;
    Color tick_major = theme->hud_highlight;
    Color tick_minor = theme->hud_accent_dim;
    Color text_color = theme->hud_highlight;
    Color north_color = theme->hud_warn;
    Color pointer_color = theme->hud_warn;

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

void hud_draw_attitude(float cx, float cy, float radius, float roll_deg, float pitch_deg, const theme_t *theme) {
    Color bg = theme->hud_bg;
    Color border = theme->hud_border;
    Color horizon_color = theme->adi_horizon;
    Color pitch_bar_color = theme->hud_accent_dim;
    Color wing_color = theme->adi_wing;
    Color roll_tri_color = theme->hud_accent;
    Color sky_color = theme->adi_sky;
    Color gnd_color = theme->adi_ground;

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
