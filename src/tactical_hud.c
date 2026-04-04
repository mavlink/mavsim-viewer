#include "tactical_hud.h"
#include "hud_transport.h"
#include "theme.h"
#include "ulog_replay.h"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#ifdef _WIN32
#undef DrawText
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

// All wireframe coordinates are from a 1920x1080 viewBox.
// We map proportionally to actual screen dimensions.
#define WF_W 1920.0f
#define WF_H 1080.0f

static float wx(float v, int sw) { return v * sw / WF_W; }
static float wy(float v, int sh) { return v * sh / WF_H; }
static float ws(float v, int sw, int sh) {
    float scale = (sw / WF_W < sh / WF_H) ? sw / WF_W : sh / WF_H;
    return v * scale;
}

// ── Tag shape ──────────────────────────────────────────────────────────────
static void draw_tag(float x, float y, float body_w, float h, float point_w,
                     float cr, bool point_right, float thick, Color color) {
    float mid_y = y + h / 2.0f;
    int segs = 6;

    if (point_right) {
        for (int i = 0; i <= segs; i++) {
            float a1 = PI + (PI / 2.0f) * (float)i / segs;
            float a2 = PI + (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = PI + PI / 2.0f;
            DrawLineEx(
                (Vector2){x + cr + cosf(a1) * cr, y + cr + sinf(a1) * cr},
                (Vector2){x + cr + cosf(a2) * cr, y + cr + sinf(a2) * cr},
                thick, color);
        }
        DrawLineEx((Vector2){x + cr, y}, (Vector2){x + body_w, y}, thick, color);
        DrawLineEx((Vector2){x + body_w, y}, (Vector2){x + body_w + point_w, mid_y}, thick, color);
        DrawLineEx((Vector2){x + body_w + point_w, mid_y}, (Vector2){x + body_w, y + h}, thick, color);
        DrawLineEx((Vector2){x + body_w, y + h}, (Vector2){x + cr, y + h}, thick, color);
        for (int i = 0; i <= segs; i++) {
            float a1 = PI / 2.0f + (PI / 2.0f) * (float)i / segs;
            float a2 = PI / 2.0f + (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = PI;
            DrawLineEx(
                (Vector2){x + cr + cosf(a1) * cr, y + h - cr + sinf(a1) * cr},
                (Vector2){x + cr + cosf(a2) * cr, y + h - cr + sinf(a2) * cr},
                thick, color);
        }
        DrawLineEx((Vector2){x, y + h - cr}, (Vector2){x, y + cr}, thick, color);
    } else {
        float rx = x + point_w + body_w;
        DrawLineEx((Vector2){x, mid_y}, (Vector2){x + point_w, y}, thick, color);
        DrawLineEx((Vector2){x + point_w, y}, (Vector2){rx - cr, y}, thick, color);
        for (int i = 0; i <= segs; i++) {
            float a1 = -PI / 2.0f + (PI / 2.0f) * (float)i / segs;
            float a2 = -PI / 2.0f + (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = 0;
            DrawLineEx(
                (Vector2){rx - cr + cosf(a1) * cr, y + cr + sinf(a1) * cr},
                (Vector2){rx - cr + cosf(a2) * cr, y + cr + sinf(a2) * cr},
                thick, color);
        }
        DrawLineEx((Vector2){rx, y + cr}, (Vector2){rx, y + h - cr}, thick, color);
        for (int i = 0; i <= segs; i++) {
            float a1 = (PI / 2.0f) * (float)i / segs;
            float a2 = (PI / 2.0f) * (float)(i + 1) / segs;
            if (i == segs) a2 = PI / 2.0f;
            DrawLineEx(
                (Vector2){rx - cr + cosf(a1) * cr, y + h - cr + sinf(a1) * cr},
                (Vector2){rx - cr + cosf(a2) * cr, y + h - cr + sinf(a2) * cr},
                thick, color);
        }
        DrawLineEx((Vector2){rx - cr, y + h}, (Vector2){x + point_w, y + h}, thick, color);
        DrawLineEx((Vector2){x + point_w, y + h}, (Vector2){x, mid_y}, thick, color);
    }
}

// ── Timer + Status (top-left) ─────────────────────────────────────────────
static void tac_draw_timer_status(const hud_t *h, const data_source_t *src,
                                   int sw, int sh, const theme_t *theme) {
    float mx = wx(27, sw);
    float my = wy(27, sh);
    float fs_timer = wy(21, sh);
    float fs_label = wy(12, sh);
    float fs_status = wy(13, sh);

    char b[16];
    int total_secs = (int)h->sim_time_s;
    int mins = total_secs / 60;
    int secs = total_secs % 60;
    if (mins >= 60)
        snprintf(b, sizeof(b), "%d:%02d:%02d", mins / 60, mins % 60, secs);
    else
        snprintf(b, sizeof(b), "%02d:%02d", mins, secs);
    DrawTextEx(h->font_value, b, (Vector2){mx, my}, fs_timer, 0.5f, theme->hud_value);
    DrawTextEx(h->font_label, "SIM",
               (Vector2){mx, my + fs_timer + wy(2, sh)}, fs_label, 0.5f, theme->hud_dim);

    bool connected = src->connected;
    float dot_y = my + fs_timer + fs_label + wy(12, sh);
    Color dot_c = connected ? theme->hud_connected : (Color){200, 60, 60, 255};
    DrawCircle((int)(mx + wy(4, sh)), (int)(dot_y + wy(4, sh)), wy(4, sh), dot_c);

    char status_buf[48];
    if (src->playback.duration_s > 0.0f) {
        if (!connected) snprintf(status_buf, sizeof(status_buf), "Replay ended");
        else if (src->playback.paused) snprintf(status_buf, sizeof(status_buf), "Replay paused");
        else snprintf(status_buf, sizeof(status_buf), "Replaying log...");
    } else if (connected) {
        snprintf(status_buf, sizeof(status_buf), "MAVLink connected");
    } else {
        snprintf(status_buf, sizeof(status_buf), "Waiting...");
    }
    DrawTextEx(h->font_label, status_buf,
               (Vector2){mx + wy(14, sh), dot_y}, fs_status, 0.5f,
               connected ? theme->hud_connected : theme->hud_dim);
}

// ── Ticker (top-center) ───────────────────────────────────────────────────
static void tac_draw_ticker(const hud_t *h, const playback_state_t *pb,
                             const hud_marker_data_t *user_md,
                             const hud_marker_data_t *sys_md,
                             int sw, int sh, const theme_t *theme) {
    float fs = wy(16, sh);
    float line_y = wy(27, sh);

    // Check if a marker is selected
    const char *marker_label = NULL;
    char marker_buf[64];
    if (sys_md && sys_md->selected && sys_md->current >= 0 && sys_md->current < sys_md->count) {
        if (sys_md->labels[sys_md->current][0] != '\0')
            snprintf(marker_buf, sizeof(marker_buf), "S: %s", sys_md->labels[sys_md->current]);
        else
            snprintf(marker_buf, sizeof(marker_buf), "S%d", sys_md->current + 1);
        marker_label = marker_buf;
    } else if (user_md && user_md->current >= 0 && user_md->current < user_md->count) {
        if (user_md->labels[user_md->current][0] != '\0')
            snprintf(marker_buf, sizeof(marker_buf), "%d: %s", user_md->current + 1, user_md->labels[user_md->current]);
        else
            snprintf(marker_buf, sizeof(marker_buf), "MARKER %d", user_md->current + 1);
        marker_label = marker_buf;
    }

    // Get flight mode name
    const char *mode_name = NULL;
    if (pb && pb->mode_changes && pb->mode_change_count > 0) {
        for (int i = pb->mode_change_count - 1; i >= 0; i--) {
            if (pb->mode_changes[i].time_s <= pb->position_s) {
                mode_name = ulog_nav_state_name(pb->mode_changes[i].nav_state);
                break;
            }
        }
    }

    // Cycle between mode and marker every 3 seconds
    if (marker_label && mode_name) {
        bool show_marker = ((int)(GetTime() / 3.0) % 2) == 1;
        if (show_marker) {
            Vector2 mw = MeasureTextEx(h->font_label, marker_label, fs, 0.5f);
            DrawTextEx(h->font_label, marker_label,
                       (Vector2){sw / 2.0f - mw.x / 2, line_y},
                       fs, 0.5f, theme->hud_accent);
        } else {
            char buf[64];
            snprintf(buf, sizeof(buf), "MODE: %s", mode_name);
            Vector2 tw = MeasureTextEx(h->font_label, buf, fs, 0.5f);
            DrawTextEx(h->font_label, buf,
                       (Vector2){sw / 2.0f - tw.x / 2, line_y},
                       fs, 0.5f, theme->hud_warn);
        }
    } else if (marker_label) {
        Vector2 mw = MeasureTextEx(h->font_label, marker_label, fs, 0.5f);
        DrawTextEx(h->font_label, marker_label,
                   (Vector2){sw / 2.0f - mw.x / 2, line_y},
                   fs, 0.5f, theme->hud_accent);
    } else if (mode_name) {
        char buf[64];
        snprintf(buf, sizeof(buf), "MODE: %s", mode_name);
        Vector2 tw = MeasureTextEx(h->font_label, buf, fs, 0.5f);
        DrawTextEx(h->font_label, buf,
                   (Vector2){sw / 2.0f - tw.x / 2, line_y},
                   fs, 0.5f, theme->hud_warn);
    }
}

// ── Heading (top-right) ───────────────────────────────────────────────────
static void tac_draw_heading(const hud_t *h, const vehicle_t *v,
                              int sw, int sh, const theme_t *theme) {
    float mx = wx(27, sw);
    float fs_label = wy(13, sh);
    float fs_value = wy(27, sh);

    const char *lbl = "HDG";
    Vector2 lw = MeasureTextEx(h->font_label, lbl, fs_label, 0.5f);
    DrawTextEx(h->font_label, lbl,
               (Vector2){sw - mx - lw.x, wy(27, sh)},
               fs_label, 0.5f, theme->hud_accent_dim);

    char b[8];
    snprintf(b, sizeof(b), "%03d", ((int)v->heading_deg % 360 + 360) % 360);
    Vector2 vw = MeasureTextEx(h->font_value, b, fs_value, 0.5f);
    DrawTextEx(h->font_value, b,
               (Vector2){sw - mx - vw.x, wy(27, sh) + fs_label + wy(2, sh)},
               fs_value, 0.5f, theme->hud_value);
}

// ── Speed Stack (GS tag, left of center) ──────────────────────────────────
static void tac_draw_speed_stack(const hud_t *h, const vehicle_t *vehicles,
                                  int vehicle_count, int selected,
                                  int sw, int sh, const theme_t *theme) {
    float anchor = sw * 0.25f;
    float cy = sh * 0.5f;

    float body_w = wx(102, sw);
    float tag_h = wy(36, sh);
    float point_w = wx(22, sw);
    float tag_total = body_w + point_w;

    float fs_label = wy(13, sh);
    float fs_value = wy(28, sh);
    float fs_unit = wy(11, sh);
    float fs_pinned = wy(19, sh);
    float line_h = wy(22, sh);

    Color border = theme->hud_border;
    Color value_c = theme->hud_value;
    Color label_c = theme->hud_accent_dim;
    Color dim_c = theme->hud_dim;

    const vehicle_t *v = &vehicles[selected];

    float tag_x = anchor - tag_total;
    float tag_y = cy - tag_h / 2.0f;
    float cr = ws(6.66f, sw, sh);
    draw_tag(tag_x, tag_y, body_w, tag_h, point_w, cr, true, 1.3f, border);

    char val[16];
    snprintf(val, sizeof(val), "%.1f", v->ground_speed);
    Vector2 vw = MeasureTextEx(h->font_value, val, fs_value, 0.5f);
    DrawTextEx(h->font_value, val,
               (Vector2){tag_x + body_w / 2 - vw.x / 2, tag_y + tag_h / 2 - vw.y / 2},
               fs_value, 0.5f, value_c);

    Vector2 lw = MeasureTextEx(h->font_label, "GS", fs_label, 0.5f);
    DrawTextEx(h->font_label, "GS",
               (Vector2){tag_x + body_w - lw.x, tag_y - fs_label - wy(4, sh)},
               fs_label, 0.5f, label_c);

    Vector2 uw = MeasureTextEx(h->font_label, "m/s", fs_unit, 0.5f);
    DrawTextEx(h->font_label, "m/s",
               (Vector2){tag_x + body_w - uw.x, tag_y + tag_h + wy(4, sh)},
               fs_unit, 0.5f, dim_c);

    float pinned_y = tag_y - fs_label - wy(4, sh) - line_h;
    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;
        char pb[16];
        snprintf(pb, sizeof(pb), "%.1f", vehicles[pidx].ground_speed);
        Vector2 pw = MeasureTextEx(h->font_value, pb, fs_pinned, 0.5f);
        DrawTextEx(h->font_value, pb,
                   (Vector2){tag_x + body_w - pw.x, pinned_y - p * line_h},
                   fs_pinned, 0.5f, vehicles[pidx].color);
    }
}

// ── Altitude Stack (ALT tag, right of center) ─────────────────────────────
static void tac_draw_alt_stack(const hud_t *h, const vehicle_t *vehicles,
                                int vehicle_count, int selected,
                                int sw, int sh, const theme_t *theme) {
    float anchor = sw * 0.75f;
    float cy = sh * 0.5f;

    float body_w = wx(102, sw);
    float tag_h = wy(36, sh);
    float point_w = wx(22, sw);

    float fs_label = wy(13, sh);
    float fs_value = wy(28, sh);
    float fs_vs = wy(13, sh);
    float fs_pinned = wy(19, sh);
    float fs_pinned_vs = wy(12, sh);
    float line_h = wy(22, sh);

    Color border = theme->hud_border;
    Color value_c = theme->hud_value;
    Color label_c = theme->hud_accent_dim;
    Color dim_c = theme->hud_dim;
    Color climb_c = theme->hud_climb;
    Color warn_c = theme->hud_warn;

    const vehicle_t *v = &vehicles[selected];

    float tag_x = anchor;
    float tag_y = cy - tag_h / 2.0f;
    float cr = ws(6.66f, sw, sh);
    draw_tag(tag_x, tag_y, body_w, tag_h, point_w, cr, false, 1.3f, border);

    char val[16];
    snprintf(val, sizeof(val), "%.1f", v->altitude_rel);
    Vector2 vw = MeasureTextEx(h->font_value, val, fs_value, 0.5f);
    float body_x = tag_x + point_w;
    DrawTextEx(h->font_value, val,
               (Vector2){body_x + body_w / 2 - vw.x / 2, tag_y + tag_h / 2 - vw.y / 2},
               fs_value, 0.5f, value_c);

    DrawTextEx(h->font_label, "ALT",
               (Vector2){body_x, tag_y - fs_label - wy(4, sh)},
               fs_label, 0.5f, label_c);

    Color vs_c = (v->vertical_speed > 0.1f) ? climb_c :
                 (v->vertical_speed < -0.1f) ? warn_c : value_c;
    const char *arrow = (v->vertical_speed > 0.1f) ? "^" :
                        (v->vertical_speed < -0.1f) ? "v" : "";
    char vs_buf[16];
    snprintf(vs_buf, sizeof(vs_buf), "%s %.1f m/s", arrow, fabsf(v->vertical_speed));
    DrawTextEx(h->font_value, vs_buf,
               (Vector2){body_x, tag_y + tag_h + wy(4, sh)},
               fs_vs, 0.5f, vs_c);

    float pinned_y = tag_y - fs_label - wy(4, sh) - line_h;
    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;
        char pb[16];
        snprintf(pb, sizeof(pb), "%.1f", vehicles[pidx].altitude_rel);
        DrawTextEx(h->font_value, pb,
                   (Vector2){body_x, pinned_y - p * line_h},
                   fs_pinned, 0.5f, vehicles[pidx].color);

        Vector2 pw = MeasureTextEx(h->font_value, pb, fs_pinned, 0.5f);
        const char *va = (vehicles[pidx].vertical_speed > 0.1f) ? "^" :
                         (vehicles[pidx].vertical_speed < -0.1f) ? "v" : "-";
        DrawTextEx(h->font_value, va,
                   (Vector2){body_x + pw.x + wx(4, sw), pinned_y - p * line_h},
                   fs_pinned_vs, 0.5f, vehicles[pidx].color);
    }
}

// ── Radar Panel (bottom-left) ─────────────────────────────────────────────
static void tac_draw_radar(const hud_t *h, const vehicle_t *vehicles,
                            int vehicle_count, int selected,
                            int sw, int sh, const theme_t *theme,
                            const ortho_panel_t *ortho,
                            int trail_mode, int corr_mode) {
    float ps = wx(293, sw);
    float px = wx(95, sw);
    float py = (float)sh - wy(130, sh) - ps;

    float ccx = px + ps / 2.0f;
    float compass_offset = ps / 4.0f;
    float ccy = py + ps / 2.0f + compass_offset;
    float cr = ps * 0.46f;

    Color accent = theme->hud_accent;
    Color dim_c = theme->hud_dim;
    Color warn = theme->hud_warn;

    DrawRectangle((int)px, (int)py, (int)ps, (int)ps, theme->hud_bg);

    if (ortho) {
        BeginScissorMode((int)px, (int)py, (int)ps, (int)ps);
        Vector3 center = vehicles[selected].position;
        float span = ortho->ortho_span;
        float grid_scale = ps / span;

        float spacing = 10.0f;
        if (span > 200.0f) spacing = 50.0f;
        else if (span > 80.0f) spacing = 20.0f;
        else if (span < 20.0f) spacing = 2.0f;

        Color grid_minor = theme->ortho_grid_minor;
        grid_minor.a = (unsigned char)(grid_minor.a * 0.5f);
        Color grid_major = theme->ortho_grid_major;
        grid_major.a = (unsigned char)(grid_major.a * 0.5f);
        float ext = span * 0.8f;
        float sx_start = floorf((center.x - ext) / spacing) * spacing;
        float sz_start = floorf((center.z - ext) / spacing) * spacing;

        for (float x = sx_start; x <= center.x + ext; x += spacing) {
            bool major = fabsf(fmodf(x, spacing * 5)) < 0.1f;
            Color gc = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            float sx1 = ccx + (x - center.x) * grid_scale;
            float sy1 = ccy + (-ext) * grid_scale;
            float sy2 = ccy + (ext) * grid_scale;
            DrawLineEx((Vector2){sx1, sy1}, (Vector2){sx1, sy2}, lw, gc);
        }
        for (float z = sz_start; z <= center.z + ext; z += spacing) {
            bool major = fabsf(fmodf(z, spacing * 5)) < 0.1f;
            Color gc = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            float sy1 = ccy + (z - center.z) * grid_scale;
            float sx1 = ccx + (-ext) * grid_scale;
            float sx2 = ccx + (ext) * grid_scale;
            DrawLineEx((Vector2){sx1, sy1}, (Vector2){sx2, sy1}, lw, gc);
        }

        // Trails
        for (int vi = 0; vi < vehicle_count; vi++) {
            const vehicle_t *v = &vehicles[vi];
            if (trail_mode <= 0 || v->trail_count < 2) continue;
            if (!v->active && vehicle_count > 1) continue;
            int start = (v->trail_count < v->trail_capacity) ? 0 : v->trail_head;

            Color col_fwd  = theme->trail_forward;
            Color col_back = theme->trail_backward;
            Color col_up   = theme->trail_climb;
            Color col_down = theme->trail_descend;
            Color col_rp   = theme->trail_roll_pos;
            Color col_rn   = theme->trail_roll_neg;

            for (int ti = 1; ti < v->trail_count; ti++) {
                int i0 = (start + ti - 1) % v->trail_capacity;
                int i1 = (start + ti) % v->trail_capacity;
                float t = (float)ti / (float)v->trail_count;

                float cr_f, cg_f, cb_f;
                unsigned char ca;

                if (trail_mode == 1) {
                    cr_f = (float)col_fwd.r; cg_f = (float)col_fwd.g; cb_f = (float)col_fwd.b;
                    float pitch = v->trail_pitch[i1];
                    float vert  = v->trail_vert[i1];
                    float roll  = v->trail_roll[i1];

                    float bt = pitch / 15.0f;
                    if (bt < 0) bt = 0; if (bt > 1) bt = 1;
                    cr_f += (col_back.r - cr_f) * bt;
                    cg_f += (col_back.g - cg_f) * bt;
                    cb_f += (col_back.b - cb_f) * bt;

                    float vt = vert / 5.0f;
                    if (vt > 1) vt = 1; if (vt < -1) vt = -1;
                    if (vt > 0) { cr_f += (col_up.r - cr_f)*vt; cg_f += (col_up.g - cg_f)*vt; cb_f += (col_up.b - cb_f)*vt; }
                    else if (vt < 0) { float d=-vt; cr_f += (col_down.r - cr_f)*d; cg_f += (col_down.g - cg_f)*d; cb_f += (col_down.b - cb_f)*d; }

                    float rt = roll / 15.0f;
                    if (rt > 1) rt = 1; if (rt < -1) rt = -1;
                    if (rt > 0) { cr_f += (col_rp.r - cr_f)*rt*0.7f; cg_f += (col_rp.g - cg_f)*rt*0.7f; cb_f += (col_rp.b - cb_f)*rt*0.7f; }
                    else if (rt < 0) { float d=-rt; cr_f += (col_rn.r - cr_f)*d*0.7f; cg_f += (col_rn.g - cg_f)*d*0.7f; cb_f += (col_rn.b - cb_f)*d*0.7f; }

                    ca = (unsigned char)(t * col_fwd.a);
                } else if (trail_mode == 2) {
                    float max_spd = v->trail_speed_max > 1.0f ? v->trail_speed_max : 1.0f;
                    float spd = v->trail_speed[i1];
                    float heat = spd / max_spd;
                    if (heat > 1.0f) heat = 1.0f;
                    if (heat < 0.0f) heat = 0.0f;
                    Color hc = theme_heat_color(theme, heat, (unsigned char)(t * 200));
                    cr_f = hc.r; cg_f = hc.g; cb_f = hc.b;
                    ca = hc.a;
                } else {
                    cr_f = v->color.r; cg_f = v->color.g; cb_f = v->color.b;
                    ca = (unsigned char)(t * 200);
                }

                float sx0 = ccx + (v->trail[i0].x - center.x) * grid_scale;
                float sz0 = ccy + (v->trail[i0].z - center.z) * grid_scale;
                float sx1 = ccx + (v->trail[i1].x - center.x) * grid_scale;
                float sz1 = ccy + (v->trail[i1].z - center.z) * grid_scale;
                float line_w = (trail_mode == 2) ? 2.0f : 1.0f;
                DrawLineEx((Vector2){sx0, sz0}, (Vector2){sx1, sz1}, line_w,
                           (Color){(unsigned char)cr_f, (unsigned char)cg_f, (unsigned char)cb_f, ca});
            }
        }

        // Compass rings
        DrawCircleLines((int)ccx, (int)ccy, cr, (Color){accent.r, accent.g, accent.b, 100});
        DrawCircleLines((int)ccx, (int)ccy, cr * 0.66f, (Color){accent.r, accent.g, accent.b, 60});
        DrawCircleLines((int)ccx, (int)ccy, cr * 0.33f, (Color){accent.r, accent.g, accent.b, 40});

        // Heading-up bearing lines
        float heading_rad = vehicles[selected].heading_deg * DEG2RAD;
        for (int deg = 0; deg < 360; deg += 30) {
            float angle = (deg * DEG2RAD) - heading_rad - PI / 2.0f;
            float inner = cr * 0.88f;
            float outer = cr;
            Color tc = (deg % 90 == 0) ? (Color){accent.r, accent.g, accent.b, 120}
                                       : (Color){accent.r, accent.g, accent.b, 60};
            DrawLineEx(
                (Vector2){ccx + cosf(angle) * inner, ccy + sinf(angle) * inner},
                (Vector2){ccx + cosf(angle) * outer, ccy + sinf(angle) * outer},
                1.0f, tc);
        }

        // Cardinal labels
        const char *labels[] = {"N", "E", "S", "W"};
        int label_degs[] = {0, 90, 180, 270};
        float fs_card = wy(13, sh);
        for (int i = 0; i < 4; i++) {
            float angle = (label_degs[i] * DEG2RAD) - heading_rad - PI / 2.0f;
            float lr = cr + wy(10, sh);
            float lx = ccx + cosf(angle) * lr;
            float ly = ccy + sinf(angle) * lr;
            Vector2 tw = MeasureTextEx(h->font_value, labels[i], fs_card, 0.5f);
            Color lc = (i == 0) ? warn : dim_c;
            DrawTextEx(h->font_value, labels[i],
                       (Vector2){lx - tw.x / 2, ly - tw.y / 2}, fs_card, 0.5f, lc);
        }

        // Drone position dots
        {
            Vector3 self_pos = center;
            float dot_scale = grid_scale;
            for (int vi = 0; vi < vehicle_count; vi++) {
                if (!vehicles[vi].active && vehicle_count > 1) continue;
                float dx = vehicles[vi].position.x - self_pos.x;
                float dz = vehicles[vi].position.z - self_pos.z;
                float bx = ccx + dx * dot_scale;
                float by = ccy + dz * dot_scale;
                float dot_r = (vi == selected) ? ws(5, sw, sh) : ws(4, sw, sh);
                DrawCircle((int)bx, (int)by, dot_r, vehicles[vi].color);
            }

            // Correlation overlays
            if (corr_mode > 0 && h->pinned_count > 0) {
                for (int p = 0; p < h->pinned_count; p++) {
                    int pidx = h->pinned[p];
                    if (pidx < 0 || pidx >= vehicle_count || !vehicles[pidx].active || pidx == selected) continue;
                    const vehicle_t *va = &vehicles[selected];
                    const vehicle_t *vb = &vehicles[pidx];

                    if (corr_mode == 2) {
                        int n = va->trail_count < vb->trail_count ? va->trail_count : vb->trail_count;
                        if (n >= 2) {
                            int sa = (va->trail_count < va->trail_capacity) ? 0 : va->trail_head;
                            int sb = (vb->trail_count < vb->trail_capacity) ? 0 : vb->trail_head;
                            for (int i = 1; i < n; i++) {
                                int ia0 = (sa + (int)((float)(i-1)/n * va->trail_count)) % va->trail_capacity;
                                int ia1 = (sa + (int)((float)i/n * va->trail_count)) % va->trail_capacity;
                                int ib0 = (sb + (int)((float)(i-1)/n * vb->trail_count)) % vb->trail_capacity;
                                int ib1 = (sb + (int)((float)i/n * vb->trail_count)) % vb->trail_capacity;
                                float t = (float)i / (float)n;
                                unsigned char al = (unsigned char)(t * 120);
                                Color mc = {(unsigned char)((va->color.r + vb->color.r)/2),
                                            (unsigned char)((va->color.g + vb->color.g)/2),
                                            (unsigned char)((va->color.b + vb->color.b)/2), al};
                                Vector2 a0 = {ccx + (va->trail[ia0].x - self_pos.x) * dot_scale,
                                              ccy + (va->trail[ia0].z - self_pos.z) * dot_scale};
                                Vector2 a1 = {ccx + (va->trail[ia1].x - self_pos.x) * dot_scale,
                                              ccy + (va->trail[ia1].z - self_pos.z) * dot_scale};
                                Vector2 b0 = {ccx + (vb->trail[ib0].x - self_pos.x) * dot_scale,
                                              ccy + (vb->trail[ib0].z - self_pos.z) * dot_scale};
                                Vector2 b1 = {ccx + (vb->trail[ib1].x - self_pos.x) * dot_scale,
                                              ccy + (vb->trail[ib1].z - self_pos.z) * dot_scale};
                                DrawTriangle(a0, b0, a1, mc);
                                DrawTriangle(a0, a1, b0, mc);
                                DrawTriangle(b0, b1, a1, mc);
                                DrawTriangle(b0, a1, b1, mc);
                            }
                        }
                    }

                    float dx_p = vb->position.x - self_pos.x;
                    float dz_p = vb->position.z - self_pos.z;
                    float bx_c = ccx + dx_p * dot_scale;
                    float by_c = ccy + dz_p * dot_scale;
                    Color mid = {(unsigned char)((va->color.r + vb->color.r)/2),
                                 (unsigned char)((va->color.g + vb->color.g)/2),
                                 (unsigned char)((va->color.b + vb->color.b)/2), 200};
                    DrawLineEx((Vector2){ccx, ccy}, (Vector2){bx_c, by_c}, 2.0f, mid);
                }
            }
        }

        EndScissorMode();
    }

    DrawRectangleLinesEx((Rectangle){px, py, ps, ps}, 1.0f, accent);

    float fs_label = wy(12, sh);
    DrawTextEx(h->font_value, "RADAR",
               (Vector2){px + wx(8, sw), py + wy(5, sh)}, fs_label, 0.5f, accent);
}

// ── Gimbal Rings ──────────────────────────────────────────────────────────
static Color blend_color(Color base, Color drone, float t) {
    return (Color){
        (unsigned char)(base.r + (drone.r - base.r) * t),
        (unsigned char)(base.g + (drone.g - base.g) * t),
        (unsigned char)(base.b + (drone.b - base.b) * t),
        base.a
    };
}

static Vector2 project_ring_point(float x3, float y3, float z3) {
    float cam_pitch = -30.0f * DEG2RAD;
    float cos_p = cosf(cam_pitch);
    float sin_p = sinf(cam_pitch);
    float py = y3 * cos_p - z3 * sin_p;
    return (Vector2){ x3, -py };
}

static void draw_projected_ring(float cx, float cy, float radius,
                                 Quaternion q, Vector3 axis_a, Vector3 axis_b,
                                 float scale, float thick, Color color) {
    int segs = 48;
    for (int i = 0; i < segs; i++) {
        float a1 = (float)i / segs * 2.0f * PI;
        float a2 = (float)(i + 1) / segs * 2.0f * PI;
        Vector3 local1 = {
            axis_a.x * cosf(a1) + axis_b.x * sinf(a1),
            axis_a.y * cosf(a1) + axis_b.y * sinf(a1),
            axis_a.z * cosf(a1) + axis_b.z * sinf(a1)
        };
        Vector3 local2 = {
            axis_a.x * cosf(a2) + axis_b.x * sinf(a2),
            axis_a.y * cosf(a2) + axis_b.y * sinf(a2),
            axis_a.z * cosf(a2) + axis_b.z * sinf(a2)
        };
        Vector3 w1 = Vector3RotateByQuaternion(local1, q);
        Vector3 w2 = Vector3RotateByQuaternion(local2, q);
        Vector2 p1 = project_ring_point(w1.x * radius, w1.y * radius, w1.z * radius);
        Vector2 p2 = project_ring_point(w2.x * radius, w2.y * radius, w2.z * radius);
        DrawLineEx(
            (Vector2){cx + p1.x * scale, cy + p1.y * scale},
            (Vector2){cx + p2.x * scale, cy + p2.y * scale},
            thick, color);
    }
}

static void tac_draw_gimbal_rings(const hud_t *h, const vehicle_t *vehicles,
                                   int vehicle_count, int sw, int sh,
                                   const theme_t *theme) {
    if (h->pinned_count == 0) return;

    float ring_y = wy(968, sh);

    float transport_w = sw * 0.5f;
    float cell_w = transport_w / 15.0f;
    float r = cell_w * 0.38f;
    float spacing = cell_w;
    float total_w = (h->pinned_count - 1) * spacing;
    float start_x = sw / 2.0f - total_w / 2.0f;

    float fs_id = r * 0.6f;
    float tint = 0.5f;

    Color base_roll  = (Color){255, 100, 100, 180};
    Color base_pitch = (Color){ 52, 211, 153, 180};
    Color base_yaw   = (Color){  0, 180, 204, 160};

    for (int p = 0; p < h->pinned_count; p++) {
        int pidx = h->pinned[p];
        if (pidx < 0 || pidx >= vehicle_count) continue;

        Color dc = vehicles[pidx].color;
        float cx = start_x + p * spacing;
        float cy = ring_y;

        Quaternion rot = vehicles[pidx].rotation;

        Vector3 fwd = Vector3RotateByQuaternion((Vector3){0, 0, -1}, rot);
        float yaw = atan2f(-fwd.x, -fwd.z);
        Quaternion q_yaw = QuaternionFromAxisAngle((Vector3){0, 1, 0}, yaw);

        Quaternion q_yaw_inv = QuaternionInvert(q_yaw);
        Quaternion q_remainder = QuaternionMultiply(q_yaw_inv, rot);
        Vector3 fwd_rem = Vector3RotateByQuaternion((Vector3){0, 0, -1}, q_remainder);
        float pitch = atan2f(fwd_rem.y, -fwd_rem.z);
        Quaternion q_pitch = QuaternionFromAxisAngle((Vector3){1, 0, 0}, pitch);

        Quaternion q_yaw_pitch = QuaternionMultiply(q_yaw, q_pitch);

        Color yc = blend_color(base_yaw, dc, tint);
        draw_projected_ring(cx, cy, 1.15f, q_yaw,
                            (Vector3){1, 0, 0}, (Vector3){0, 0, -1},
                            r, 1.0f, yc);

        {
            Vector3 north_local = {cosf(0) * 1.15f, 0, -sinf(0) * 1.15f};
            Vector3 nw = Vector3RotateByQuaternion(north_local, q_yaw);
            Vector2 np = project_ring_point(nw.x * r, nw.y * r, nw.z * r);
            DrawCircle((int)(cx + np.x), (int)(cy + np.y),
                       ws(2.5f, sw, sh), (Color){255, 68, 68, 220});
        }

        Color pc = blend_color(base_pitch, dc, tint);
        draw_projected_ring(cx, cy, 1.0f, q_yaw_pitch,
                            (Vector3){0, 0, -1}, (Vector3){0, 1, 0},
                            r, 1.2f, pc);

        Color rc = blend_color(base_roll, dc, tint);
        draw_projected_ring(cx, cy, 0.85f, rot,
                            (Vector3){1, 0, 0}, (Vector3){0, 1, 0},
                            r, 1.2f, rc);

        DrawCircle((int)cx, (int)cy, ws(2, sw, sh), (Color){dc.r, dc.g, dc.b, 130});

        char id_buf[8];
        snprintf(id_buf, sizeof(id_buf), "D%d", pidx + 1);
        Vector2 tw = MeasureTextEx(h->font_value, id_buf, fs_id, 0.5f);
        DrawTextEx(h->font_value, id_buf,
                   (Vector2){cx - tw.x / 2, cy + r * 1.2f + wy(4, sh)},
                   fs_id, 0.5f, dc);
    }
}

// ── Toast ─────────────────────────────────────────────────────────────────
static void tac_draw_toast(const hud_t *h, int sw, int sh, const theme_t *theme) {
    if (h->toast_timer <= 0.0f) return;
    float fs = wy(14, sh);
    Vector2 tw = MeasureTextEx(h->font_label, h->toast_text, fs, 0.5f);
    float fade = (h->toast_timer < 0.5f) ? h->toast_timer / 0.5f : 1.0f;
    Color base = (h->toast_color.a > 0) ? h->toast_color : theme->hud_climb;
    Color tc = (Color){base.r, base.g, base.b, (unsigned char)(fade * 255)};
    DrawTextEx(h->font_label, h->toast_text,
               (Vector2){sw / 2.0f - tw.x / 2, wy(50, sh)}, fs, 0.5f, tc);
}

// ── Warnings ──────────────────────────────────────────────────────────────
static void tac_draw_warnings(const hud_t *h, bool has_tier3, bool has_awaiting_gps,
                               int sw, int sh, const theme_t *theme) {
    float fs = wy(13, sh);
    float base_y = wy(70, sh);
    if (has_tier3) {
        const char *t = "ESTIMATED POSITION";
        Vector2 tw = MeasureTextEx(h->font_label, t, fs, 0.5f);
        DrawTextEx(h->font_label, t,
                   (Vector2){sw / 2.0f - tw.x / 2, base_y}, fs, 0.5f, theme->hud_warn);
        base_y += tw.y + wy(4, sh);
    }
    if (has_awaiting_gps) {
        const char *t = "AWAITING GPS";
        Vector2 tw = MeasureTextEx(h->font_label, t, fs, 0.5f);
        DrawTextEx(h->font_label, t,
                   (Vector2){sw / 2.0f - tw.x / 2, base_y}, fs, 0.5f, theme->hud_warn);
    }
}

// ── Reticle ───────────────────────────────────────────────────────────────
static void tac_draw_reticle(int sw, int sh, const theme_t *theme) {
    float cx = sw / 2.0f;
    float cy = sh / 2.0f;
    Color r = (Color){theme->hud_value.r, theme->hud_value.g, theme->hud_value.b, 90};
    float l = ws(12, sw, sh);
    float g = ws(5, sw, sh);
    DrawLineEx((Vector2){cx - l, cy}, (Vector2){cx - g, cy}, 1.3f, r);
    DrawLineEx((Vector2){cx + g, cy}, (Vector2){cx + l, cy}, 1.3f, r);
    DrawCircle((int)cx, (int)cy, ws(2.5f, sw, sh), r);
}

// ── Main Draw ─────────────────────────────────────────────────────────────
void tactical_hud_draw(const hud_t *h, const vehicle_t *vehicles,
                       const data_source_t *sources, int vehicle_count,
                       int selected, int screen_w, int screen_h,
                       const theme_t *theme, bool ghost_mode,
                       bool has_tier3, bool has_awaiting_gps,
                       const ortho_panel_t *ortho,
                       int trail_mode, int corr_mode,
                       const hud_marker_data_t *markers_all,
                       const hud_marker_data_t *sys_markers_all,
                       int marker_vehicle_count) {

    const data_source_t *src = &sources[selected];
    (void)ghost_mode;

    tac_draw_reticle(screen_w, screen_h, theme);
    tac_draw_timer_status(h, src, screen_w, screen_h, theme);
    tac_draw_heading(h, &vehicles[selected], screen_w, screen_h, theme);
    {
        const hud_marker_data_t *sel_user = markers_all ? &markers_all[selected] : NULL;
        const hud_marker_data_t *sel_sys = sys_markers_all ? &sys_markers_all[selected] : NULL;
        tac_draw_ticker(h, &src->playback, sel_user, sel_sys, screen_w, screen_h, theme);
    }
    tac_draw_toast(h, screen_w, screen_h, theme);
    tac_draw_warnings(h, has_tier3, has_awaiting_gps, screen_w, screen_h, theme);
    tac_draw_speed_stack(h, vehicles, vehicle_count, selected,
                         screen_w, screen_h, theme);
    tac_draw_alt_stack(h, vehicles, vehicle_count, selected,
                       screen_w, screen_h, theme);
    tac_draw_radar(h, vehicles, vehicle_count, selected,
                   screen_w, screen_h, theme, ortho, trail_mode, corr_mode);
    tac_draw_gimbal_rings(h, vehicles, vehicle_count, screen_w, screen_h, theme);

    // Ortho insets (O toggle): side view top-left, front view top-right
    // Replaces the normal sidebar ortho panels in tactical mode
    if (ortho && ortho->visible) {
        float inset_size = screen_w * 0.153f;
        float margin = screen_w * 0.05f;
        float top_y = margin + screen_h * 0.04f;
        // Front view — top-left corner
        ortho_panel_draw_single(ortho, 1, margin, top_y, inset_size, 0.5f,
                                theme, h->font_label,
                                vehicles, vehicle_count, selected, trail_mode,
                                corr_mode, h->pinned, h->pinned_count);
        // Side (right) view — top-right corner
        ortho_panel_draw_single(ortho, 2, screen_w - margin - inset_size, top_y, inset_size, 0.5f,
                                theme, h->font_label,
                                vehicles, vehicle_count, selected, trail_mode,
                                corr_mode, h->pinned, h->pinned_count);
    }

    // Reuse shared transport bar — 50% width centered (25% to 75%)
    if (src->playback.duration_s > 0.0f) {
        float s = powf(screen_h / 720.0f, 0.7f);
        if (s < 1.0f) s = 1.0f;
        float transport_h = 28 * s;
        float bar_y = (float)screen_h - transport_h;
        float tbar_w = screen_w * 0.5f;
        float tbar_x = screen_w * 0.25f;
        BeginScissorMode((int)tbar_x, (int)bar_y, (int)tbar_w, (int)transport_h + 20);
        rlPushMatrix();
        rlTranslatef(tbar_x, 0, 0);
        hud_draw_transport(h, &src->playback, src->connected,
                           &vehicles[selected],
                           markers_all, sys_markers_all,
                           marker_vehicle_count, selected,
                           (int)tbar_w, bar_y, transport_h,
                           s, trail_mode, theme);
        rlPopMatrix();
        EndScissorMode();
    }
}
