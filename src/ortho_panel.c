#include "ortho_panel.h"
#include "rlgl.h"
#include "raymath.h"
#include <stdio.h>
#include <math.h>

#define TRAIL_INTERVAL_ORTHO 0.016f  // matches vehicle.c TRAIL_INTERVAL

static const char *view_labels[ORTHO_VIEW_COUNT] = { "TOP", "FRONT", "RIGHT" };

// Forward declarations
static void draw_axis_gizmo_at(float ox, float oy, float sc, int ortho_mode, Font font);

// ── Sky / ground colors per view mode ────────────────────────────────────────

static void get_sky_ground(view_mode_t vm, Color *sky, Color *gnd) {
    switch (vm) {
        case VIEW_REZ:
            *sky = (Color){ 12,  12,  18, 255 };
            *gnd = (Color){  2,   2,   4, 240 };
            break;
        case VIEW_1988:
            *sky = (Color){  8,   8,  20, 255 };
            *gnd = (Color){  5,   5,  16, 240 };
            break;
        case VIEW_SNOW:
            *sky = (Color){ 240, 242, 245, 255 };
            *gnd = (Color){ 210, 212, 216, 240 };
            break;
        default: // VIEW_GRID
            *sky = (Color){ 56,  56,  60, 255 };
            *gnd = (Color){ 32,  32,  34, 240 };
            break;
    }
}

static void get_grid_colors(view_mode_t vm, Color *minor, Color *major) {
    switch (vm) {
        case VIEW_REZ:
            *minor = (Color){  0, 204, 218,  80 };
            *major = (Color){  0, 204, 218, 190 };
            break;
        case VIEW_1988:
            *minor = (Color){ 255, 20, 100,  80 };
            *major = (Color){ 255, 20, 100, 200 };
            break;
        case VIEW_SNOW:
            *minor = (Color){ 140, 145, 155, 180 };
            *major = (Color){  50,  55,  65, 230 };
            break;
        default:
            *minor = (Color){ 115, 115, 120, 200 };
            *major = (Color){ 175, 175, 180, 240 };
            break;
    }
}

// ── Projection helpers ───────────────────────────────────────────────────────

// Sidebar projection: view 0=TOP, 1=FRONT, 2=RIGHT
static Vector2 world_to_panel(Vector3 w, Vector3 center, float span,
                               float px, float py, float ps, int view) {
    float scale = ps / span;
    float dx, dy;
    switch (view) {
        case 0: // TOP: +X right, +Z down
            dx = (w.x - center.x); dy = (w.z - center.z); break;
        case 1: // FRONT: camera at z-100 looking +Z, vx=(-1,0,0), -X is right
            dx = -(w.x - center.x); dy = -(w.y - center.y); break;
        default: // RIGHT: -Z is right
            dx = -(w.z - center.z); dy = -(w.y - center.y); break;
    }
    return (Vector2){ px + ps * 0.5f + dx * scale, py + ps * 0.5f + dy * scale };
}

// Fullscreen ortho projection: all 6 views
static Vector2 world_to_screen_ortho(Vector3 w, Vector3 center, float span,
                                      int screen_w, int screen_h, int ortho_mode) {
    int min_dim = screen_w < screen_h ? screen_w : screen_h;
    float scale = (float)min_dim / span;
    float dx, dy;
    switch (ortho_mode) {
        case 1: dx = (w.x - center.x); dy = (w.z - center.z); break;          // TOP
        case 2: dx = (w.x - center.x); dy = -(w.z - center.z); break;         // BOTTOM
        case 3: dx = -(w.x - center.x); dy = -(w.y - center.y); break;        // FRONT
        case 4: dx = (w.x - center.x); dy = -(w.y - center.y); break;         // BACK
        case 5: dx = (w.z - center.z); dy = -(w.y - center.y); break;         // LEFT
        case 6: dx = -(w.z - center.z); dy = -(w.y - center.y); break;        // RIGHT
        default: dx = 0; dy = 0; break;
    }
    return (Vector2){ screen_w * 0.5f + dx * scale, screen_h * 0.5f + dy * scale };
}

// ── Heat-to-color palette (copy from vehicle.c) ─────────────────────────────

static Color heat_to_color(float heat, unsigned char alpha, view_mode_t mode) {
    float cr, cg, cb;

    if (mode == VIEW_1988) {
        if (heat < 0.16f) {
            float s = heat / 0.16f;
            cr = 100 + 40 * s; cg = 10 * s; cb = 120 + 50 * s;
        } else if (heat < 0.33f) {
            float s = (heat - 0.16f) / 0.17f;
            cr = 140 + 115 * s; cg = 10; cb = 170 - 70 * s;
        } else if (heat < 0.5f) {
            float s = (heat - 0.33f) / 0.17f;
            cr = 255; cg = 10 + 30 * s; cb = 100 - 100 * s;
        } else if (heat < 0.66f) {
            float s = (heat - 0.5f) / 0.16f;
            cr = 255; cg = 40 + 130 * s; cb = 0;
        } else if (heat < 0.83f) {
            float s = (heat - 0.66f) / 0.17f;
            cr = 255; cg = 170 + 85 * s; cb = 50 * s;
        } else {
            float s = (heat - 0.83f) / 0.17f;
            cr = 255; cg = 255; cb = 50 + 205 * s;
        }
    } else if (mode == VIEW_REZ) {
        if (heat < 0.16f) {
            float s = heat / 0.16f;
            cr = 50 + 20 * s; cg = 20 + 30 * s; cb = 140 + 40 * s;
        } else if (heat < 0.33f) {
            float s = (heat - 0.16f) / 0.17f;
            cr = 70 + 100 * s; cg = 50 + 10 * s; cb = 180 - 60 * s;
        } else if (heat < 0.5f) {
            float s = (heat - 0.33f) / 0.17f;
            cr = 170 + 85 * s; cg = 60 - 20 * s; cb = 120 - 120 * s;
        } else if (heat < 0.66f) {
            float s = (heat - 0.5f) / 0.16f;
            cr = 255; cg = 40 + 120 * s; cb = 20 * s;
        } else if (heat < 0.83f) {
            float s = (heat - 0.66f) / 0.17f;
            cr = 255; cg = 160 + 90 * s; cb = 20 + 40 * s;
        } else {
            float s = (heat - 0.83f) / 0.17f;
            cr = 255; cg = 250 + 5 * s; cb = 60 + 195 * s;
        }
    } else if (mode == VIEW_SNOW) {
        if (heat < 0.16f) {
            float s = heat / 0.16f;
            cr = 10 + 10 * s; cg = 20 + 20 * s; cb = 100 + 40 * s;
        } else if (heat < 0.33f) {
            float s = (heat - 0.16f) / 0.17f;
            cr = 20 - 10 * s; cg = 40 + 80 * s; cb = 140 + 40 * s;
        } else if (heat < 0.5f) {
            float s = (heat - 0.33f) / 0.17f;
            cr = 10 - 10 * s; cg = 120 + 40 * s; cb = 180 - 100 * s;
        } else if (heat < 0.66f) {
            float s = (heat - 0.5f) / 0.16f;
            cr = 0 + 60 * s; cg = 160 + 40 * s; cb = 80 - 80 * s;
        } else if (heat < 0.83f) {
            float s = (heat - 0.66f) / 0.17f;
            cr = 60 + 180 * s; cg = 200 + 20 * s; cb = 0;
        } else {
            float s = (heat - 0.83f) / 0.17f;
            cr = 240; cg = 220 - 180 * s; cb = 0;
        }
    } else {
        // Grid (default): purple -> magenta -> red -> orange -> yellow -> white
        if (heat < 0.16f) {
            float s = heat / 0.16f;
            cr = 80 + 40 * s; cg = 20 * s; cb = 140 + 40 * s;
        } else if (heat < 0.33f) {
            float s = (heat - 0.16f) / 0.17f;
            cr = 120 + 80 * s; cg = 20 - 20 * s; cb = 180 - 60 * s;
        } else if (heat < 0.5f) {
            float s = (heat - 0.33f) / 0.17f;
            cr = 200 + 55 * s; cg = 20 * s; cb = 120 - 120 * s;
        } else if (heat < 0.66f) {
            float s = (heat - 0.5f) / 0.16f;
            cr = 255; cg = 20 + 140 * s; cb = 0;
        } else if (heat < 0.83f) {
            float s = (heat - 0.66f) / 0.17f;
            cr = 255; cg = 160 + 95 * s; cb = 40 * s;
        } else {
            float s = (heat - 0.83f) / 0.17f;
            cr = 255; cg = 255; cb = 40 + 215 * s;
        }
    }

    return (Color){
        (unsigned char)(cr > 255 ? 255 : (cr < 0 ? 0 : cr)),
        (unsigned char)(cg > 255 ? 255 : (cg < 0 ? 0 : cg)),
        (unsigned char)(cb > 255 ? 255 : (cb < 0 ? 0 : cb)),
        alpha
    };
}

// ── 2D trail drawing (sidebar) ───────────────────────────────────────────────

static void draw_trail_2d(const vehicle_t *v, view_mode_t view_mode,
                           int trail_mode, Vector3 center, float span,
                           float px, float py, float ps, int view) {
    if (trail_mode <= 0 || v->trail_count < 2) return;

    int start = (v->trail_count < v->trail_capacity) ? 0 : v->trail_head;

    if (trail_mode == 1) {
        // ── Normal directional trail ──
        Color trail_color;
        Color col_back, col_up, col_down, col_roll_pos, col_roll_neg;
        if (view_mode == VIEW_SNOW) {
            trail_color  = (Color){ 200, 140,  20, 200 };
            col_back     = (Color){ 140,  20, 200, 255 };
            col_up       = (Color){   0, 150,  60, 255 };
            col_down     = (Color){ 200,  50,   0, 255 };
            col_roll_pos = (Color){  20, 160,  40, 255 };
            col_roll_neg = (Color){ 200,  20,  60, 255 };
        } else if (view_mode == VIEW_1988) {
            trail_color  = (Color){ 255, 220,  60, 160 };
            col_back     = (Color){ 180,  40, 255, 255 };
            col_up       = (Color){   0, 240, 255, 255 };
            col_down     = (Color){ 255, 140,   0, 255 };
            col_roll_pos = (Color){  40, 255,  80, 255 };
            col_roll_neg = (Color){ 255,  40,  80, 255 };
        } else if (view_mode == VIEW_REZ) {
            trail_color  = (Color){ 220, 180,  30, 160 };
            col_back     = (Color){ 160,  40, 240, 255 };
            col_up       = (Color){   0, 200, 255, 255 };
            col_down     = (Color){ 255, 160,   0, 255 };
            col_roll_pos = (Color){  40, 255, 100, 255 };
            col_roll_neg = (Color){ 255,  40,  80, 255 };
        } else {
            trail_color  = (Color){ 255, 200,  50, 180 };
            col_back     = (Color){ 160,  60, 255, 255 };
            col_up       = (Color){   0, 220, 255, 255 };
            col_down     = (Color){ 255, 140,   0, 255 };
            col_roll_pos = (Color){  40, 255,  80, 255 };
            col_roll_neg = (Color){ 255,  40,  80, 255 };
        }

        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;
            float t = (float)i / (float)v->trail_count;

            float pitch = v->trail_pitch[idx1];
            float vert  = v->trail_vert[idx1];
            float roll  = v->trail_roll[idx1];

            float cr = (float)trail_color.r;
            float cg = (float)trail_color.g;
            float cb = (float)trail_color.b;

            float back_t = pitch / 15.0f;
            if (back_t < 0.0f) back_t = 0.0f;
            if (back_t > 1.0f) back_t = 1.0f;
            cr += (col_back.r - cr) * back_t;
            cg += (col_back.g - cg) * back_t;
            cb += (col_back.b - cb) * back_t;

            float vert_t = vert / 5.0f;
            if (vert_t > 1.0f) vert_t = 1.0f;
            if (vert_t < -1.0f) vert_t = -1.0f;
            if (vert_t > 0.0f) {
                cr += (col_up.r - cr) * vert_t;
                cg += (col_up.g - cg) * vert_t;
                cb += (col_up.b - cb) * vert_t;
            } else if (vert_t < 0.0f) {
                float dt2 = -vert_t;
                cr += (col_down.r - cr) * dt2;
                cg += (col_down.g - cg) * dt2;
                cb += (col_down.b - cb) * dt2;
            }

            float roll_t = roll / 15.0f;
            if (roll_t > 1.0f) roll_t = 1.0f;
            if (roll_t < -1.0f) roll_t = -1.0f;
            if (roll_t > 0.0f) {
                cr += (col_roll_pos.r - cr) * roll_t * 0.7f;
                cg += (col_roll_pos.g - cg) * roll_t * 0.7f;
                cb += (col_roll_pos.b - cb) * roll_t * 0.7f;
            } else if (roll_t < 0.0f) {
                float rt = -roll_t;
                cr += (col_roll_neg.r - cr) * rt * 0.7f;
                cg += (col_roll_neg.g - cg) * rt * 0.7f;
                cb += (col_roll_neg.b - cb) * rt * 0.7f;
            }

            unsigned char ccr = (unsigned char)(cr > 255 ? 255 : cr);
            unsigned char ccg = (unsigned char)(cg > 255 ? 255 : cg);
            unsigned char ccb = (unsigned char)(cb > 255 ? 255 : cb);
            unsigned char ca  = (unsigned char)(t * trail_color.a * v->ghost_alpha);

            Vector2 s0 = world_to_panel(v->trail[idx0], center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(v->trail[idx1], center, span, px, py, ps, view);
            DrawLineEx(s0, s1, 1.0f, (Color){ ccr, ccg, ccb, ca });
        }
    } else if (trail_mode == 3) {
        // ── Drone-color trail ──
        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;
            float t = (float)i / (float)v->trail_count;
            unsigned char ca = (unsigned char)(t * 200 * v->ghost_alpha);

            Vector2 s0 = world_to_panel(v->trail[idx0], center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(v->trail[idx1], center, span, px, py, ps, view);
            DrawLineEx(s0, s1, 1.0f, (Color){ v->color.r, v->color.g, v->color.b, ca });
        }
    } else {
        // ── Speed ribbon trail (mode 2) ──
        float max_speed = v->trail_speed_max > 1.0f ? v->trail_speed_max : 1.0f;

        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;

            float spd0 = v->trail_speed[idx0];
            float spd1 = v->trail_speed[idx1];

            float spd_avg = (spd0 + spd1) * 0.5f;
            float accel = (spd1 - spd0) / TRAIL_INTERVAL_ORTHO;
            float accel_shift = accel / 40.0f;
            if (accel_shift > 0.15f) accel_shift = 0.15f;
            if (accel_shift < -0.15f) accel_shift = -0.15f;
            float heat = spd_avg / max_speed + accel_shift;
            if (heat > 1.0f) heat = 1.0f;
            if (heat < 0.0f) heat = 0.0f;

            float t = (float)i / (float)v->trail_count;
            Color c = heat_to_color(heat, (unsigned char)(t * 200), view_mode);
            c.a = (unsigned char)(c.a * v->ghost_alpha);

            Vector2 s0 = world_to_panel(v->trail[idx0], center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(v->trail[idx1], center, span, px, py, ps, view);
            DrawLineEx(s0, s1, 2.0f, c);
        }
    }
}

// ── 2D trail drawing (fullscreen ortho) ──────────────────────────────────────

static void draw_trail_2d_fullscreen(const vehicle_t *v, view_mode_t view_mode,
                                      int trail_mode, Vector3 center, float span,
                                      int screen_w, int screen_h, int ortho_mode) {
    if (trail_mode <= 0 || v->trail_count < 2) return;

    int start = (v->trail_count < v->trail_capacity) ? 0 : v->trail_head;

    if (trail_mode == 1) {
        // ── Normal directional trail ──
        Color trail_color;
        Color col_back, col_up, col_down, col_roll_pos, col_roll_neg;
        if (view_mode == VIEW_SNOW) {
            trail_color  = (Color){ 200, 140,  20, 200 };
            col_back     = (Color){ 140,  20, 200, 255 };
            col_up       = (Color){   0, 150,  60, 255 };
            col_down     = (Color){ 200,  50,   0, 255 };
            col_roll_pos = (Color){  20, 160,  40, 255 };
            col_roll_neg = (Color){ 200,  20,  60, 255 };
        } else if (view_mode == VIEW_1988) {
            trail_color  = (Color){ 255, 220,  60, 160 };
            col_back     = (Color){ 180,  40, 255, 255 };
            col_up       = (Color){   0, 240, 255, 255 };
            col_down     = (Color){ 255, 140,   0, 255 };
            col_roll_pos = (Color){  40, 255,  80, 255 };
            col_roll_neg = (Color){ 255,  40,  80, 255 };
        } else if (view_mode == VIEW_REZ) {
            trail_color  = (Color){ 220, 180,  30, 160 };
            col_back     = (Color){ 160,  40, 240, 255 };
            col_up       = (Color){   0, 200, 255, 255 };
            col_down     = (Color){ 255, 160,   0, 255 };
            col_roll_pos = (Color){  40, 255, 100, 255 };
            col_roll_neg = (Color){ 255,  40,  80, 255 };
        } else {
            trail_color  = (Color){ 255, 200,  50, 180 };
            col_back     = (Color){ 160,  60, 255, 255 };
            col_up       = (Color){   0, 220, 255, 255 };
            col_down     = (Color){ 255, 140,   0, 255 };
            col_roll_pos = (Color){  40, 255,  80, 255 };
            col_roll_neg = (Color){ 255,  40,  80, 255 };
        }

        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;
            float t = (float)i / (float)v->trail_count;

            float pitch = v->trail_pitch[idx1];
            float vert  = v->trail_vert[idx1];
            float roll  = v->trail_roll[idx1];

            float cr = (float)trail_color.r;
            float cg = (float)trail_color.g;
            float cb = (float)trail_color.b;

            float back_t = pitch / 15.0f;
            if (back_t < 0.0f) back_t = 0.0f;
            if (back_t > 1.0f) back_t = 1.0f;
            cr += (col_back.r - cr) * back_t;
            cg += (col_back.g - cg) * back_t;
            cb += (col_back.b - cb) * back_t;

            float vert_t = vert / 5.0f;
            if (vert_t > 1.0f) vert_t = 1.0f;
            if (vert_t < -1.0f) vert_t = -1.0f;
            if (vert_t > 0.0f) {
                cr += (col_up.r - cr) * vert_t;
                cg += (col_up.g - cg) * vert_t;
                cb += (col_up.b - cb) * vert_t;
            } else if (vert_t < 0.0f) {
                float dt2 = -vert_t;
                cr += (col_down.r - cr) * dt2;
                cg += (col_down.g - cg) * dt2;
                cb += (col_down.b - cb) * dt2;
            }

            float roll_t = roll / 15.0f;
            if (roll_t > 1.0f) roll_t = 1.0f;
            if (roll_t < -1.0f) roll_t = -1.0f;
            if (roll_t > 0.0f) {
                cr += (col_roll_pos.r - cr) * roll_t * 0.7f;
                cg += (col_roll_pos.g - cg) * roll_t * 0.7f;
                cb += (col_roll_pos.b - cb) * roll_t * 0.7f;
            } else if (roll_t < 0.0f) {
                float rt = -roll_t;
                cr += (col_roll_neg.r - cr) * rt * 0.7f;
                cg += (col_roll_neg.g - cg) * rt * 0.7f;
                cb += (col_roll_neg.b - cb) * rt * 0.7f;
            }

            unsigned char ccr = (unsigned char)(cr > 255 ? 255 : cr);
            unsigned char ccg = (unsigned char)(cg > 255 ? 255 : cg);
            unsigned char ccb = (unsigned char)(cb > 255 ? 255 : cb);
            unsigned char ca  = (unsigned char)(t * trail_color.a * v->ghost_alpha);

            Vector2 s0 = world_to_screen_ortho(v->trail[idx0], center, span, screen_w, screen_h, ortho_mode);
            Vector2 s1 = world_to_screen_ortho(v->trail[idx1], center, span, screen_w, screen_h, ortho_mode);
            DrawLineEx(s0, s1, 1.0f, (Color){ ccr, ccg, ccb, ca });
        }
    } else if (trail_mode == 3) {
        // ── Drone-color trail ──
        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;
            float t = (float)i / (float)v->trail_count;
            unsigned char ca = (unsigned char)(t * 200 * v->ghost_alpha);

            Vector2 s0 = world_to_screen_ortho(v->trail[idx0], center, span, screen_w, screen_h, ortho_mode);
            Vector2 s1 = world_to_screen_ortho(v->trail[idx1], center, span, screen_w, screen_h, ortho_mode);
            DrawLineEx(s0, s1, 1.0f, (Color){ v->color.r, v->color.g, v->color.b, ca });
        }
    } else {
        // ── Speed ribbon trail (mode 2) ──
        float max_speed = v->trail_speed_max > 1.0f ? v->trail_speed_max : 1.0f;

        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;

            float spd0 = v->trail_speed[idx0];
            float spd1 = v->trail_speed[idx1];

            float spd_avg = (spd0 + spd1) * 0.5f;
            float accel = (spd1 - spd0) / TRAIL_INTERVAL_ORTHO;
            float accel_shift = accel / 40.0f;
            if (accel_shift > 0.15f) accel_shift = 0.15f;
            if (accel_shift < -0.15f) accel_shift = -0.15f;
            float heat = spd_avg / max_speed + accel_shift;
            if (heat > 1.0f) heat = 1.0f;
            if (heat < 0.0f) heat = 0.0f;

            float t = (float)i / (float)v->trail_count;
            Color c = heat_to_color(heat, (unsigned char)(t * 200), view_mode);
            c.a = (unsigned char)(c.a * v->ghost_alpha);

            Vector2 s0 = world_to_screen_ortho(v->trail[idx0], center, span, screen_w, screen_h, ortho_mode);
            Vector2 s1 = world_to_screen_ortho(v->trail[idx1], center, span, screen_w, screen_h, ortho_mode);
            DrawLineEx(s0, s1, 2.0f, c);
        }
    }
}

// ── Init ─────────────────────────────────────────────────────────────────────

void ortho_panel_init(ortho_panel_t *op) {
    op->ortho_span = 60.0f;
    op->visible = false;

    for (int i = 0; i < ORTHO_VIEW_COUNT; i++) {
        op->targets[i] = LoadRenderTexture(ORTHO_TEX_SIZE, ORTHO_TEX_SIZE);
        op->cameras[i] = (Camera3D){
            .position = (Vector3){0, 0, 0},
            .target   = (Vector3){0, 0, 0},
            .up       = (Vector3){0, 1, 0},
            .fovy     = 1.0f,
            .projection = CAMERA_ORTHOGRAPHIC,
        };
    }
}

// ── Update ───────────────────────────────────────────────────────────────────

void ortho_panel_update(ortho_panel_t *op, Vector3 pos) {
    float span = op->ortho_span;

    // Top-down: camera above, looking down
    op->cameras[0].position = (Vector3){ pos.x, pos.y + 100.0f, pos.z };
    op->cameras[0].target   = pos;
    op->cameras[0].up       = (Vector3){ 0, 0, -1 };
    op->cameras[0].fovy     = span;

    // Front: camera in front (looking -Z), shows XY plane
    op->cameras[1].position = (Vector3){ pos.x, pos.y, pos.z - 100.0f };
    op->cameras[1].target   = pos;
    op->cameras[1].up       = (Vector3){ 0, 1, 0 };
    op->cameras[1].fovy     = span;

    // Side: camera to the right (looking -X), shows ZY plane
    op->cameras[2].position = (Vector3){ pos.x + 100.0f, pos.y, pos.z };
    op->cameras[2].target   = pos;
    op->cameras[2].up       = (Vector3){ 0, 1, 0 };
    op->cameras[2].fovy     = span;

    // Alt+scroll to zoom ortho views
    if (IsKeyDown(KEY_LEFT_ALT)) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) {
            op->ortho_span -= wheel * 10.0f;
            if (op->ortho_span < 10.0f) op->ortho_span = 10.0f;
            if (op->ortho_span > 500.0f) op->ortho_span = 500.0f;
        }
    }
}

// ── Render (SLIM 3D: models + correlation curtain ONLY) ──────────────────────

void ortho_panel_render(ortho_panel_t *op, const vehicle_t *vehicles,
                        int vehicle_count, int selected, view_mode_t view_mode,
                        int corr_mode, const int *pinned, int pinned_count)
{
    Color bg_col;
    {
        Color gnd_dummy;
        get_sky_ground(view_mode, &bg_col, &gnd_dummy);
    }

    for (int v = 0; v < ORTHO_VIEW_COUNT; v++) {
        BeginTextureMode(op->targets[v]);
            ClearBackground(bg_col);
            BeginMode3D(op->cameras[v]);
                // Draw vehicle models only (trail_mode=0, no trails in 3D)
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        vehicle_draw((vehicle_t *)&vehicles[i], view_mode, i == selected,
                                     0, false, op->cameras[v].position, false);
                    }
                }

                // Correlation curtain stays in 3D (exception — ruled surface needs depth)
                if (corr_mode == 2 && pinned_count > 0) {
                    for (int p = 0; p < pinned_count; p++) {
                        int pidx = pinned[p];
                        if (pidx >= 0 && pidx < vehicle_count && vehicles[pidx].active
                            && pidx != selected) {
                            vehicle_draw_correlation_curtain(
                                &vehicles[selected], &vehicles[pidx],
                                view_mode, op->cameras[v].position);
                        }
                    }
                }
            EndMode3D();
        EndTextureMode();
    }
}

// ── Scale bar helper ─────────────────────────────────────────────────────────

static void draw_scale_bar(float x, float y, float ps, float span, Color col, Font font, float font_size) {
    float target_px = ps * 0.35f;
    float meters_per_px = span / ps;
    float target_m = target_px * meters_per_px;

    float nice[] = { 1, 2, 5, 10, 20, 50, 100, 200, 500 };
    float scale_m = nice[0];
    for (int i = 0; i < 9; i++) {
        if (nice[i] <= target_m) scale_m = nice[i];
        else break;
    }
    float scale_px = scale_m / meters_per_px;

    float sx = x + 8;
    float sy = y + ps - 12;

    DrawRectangle((int)sx, (int)sy, (int)scale_px, 2, col);
    DrawRectangle((int)sx, (int)(sy - 3), 1, 8, col);
    DrawRectangle((int)(sx + scale_px), (int)(sy - 3), 1, 8, col);

    char buf[32];
    if (scale_m >= 1.0f)
        snprintf(buf, sizeof(buf), "%.0fm", scale_m);
    else
        snprintf(buf, sizeof(buf), "%.1fm", scale_m);
    Vector2 ts = MeasureTextEx(font, buf, font_size, 0);
    DrawTextEx(font, buf, (Vector2){ sx + scale_px / 2 - ts.x / 2, sy - font_size - 4 }, font_size, 0, col);
}

// ── Crosshair helper ─────────────────────────────────────────────────────────

static void draw_crosshair(float cx, float cy, float size, Color col) {
    DrawLine((int)(cx - size), (int)cy, (int)(cx + size), (int)cy, col);
    DrawLine((int)cx, (int)(cy - size), (int)cx, (int)(cy + size), col);
}

// ── 2D grid drawing (sidebar) ────────────────────────────────────────────────

static void draw_grid_2d(Vector3 center, float span, float px, float py,
                          float ps, int view, Color grid_minor, Color grid_major) {
    float ext = span * 2.0f;
    float spacing = 10.0f;
    if (span > 200.0f) spacing = 50.0f;
    else if (span > 80.0f) spacing = 20.0f;
    else if (span < 20.0f) spacing = 2.0f;

    // Determine which world axes map to the view's horizontal/vertical
    // For each grid line in world space, project both endpoints to panel coords
    if (view == 0) {
        // TOP: XZ grid
        float sh = floorf((center.x - ext) / spacing) * spacing;
        float sv = floorf((center.z - ext) / spacing) * spacing;
        for (float x = sh; x <= center.x + ext; x += spacing) {
            bool major = fabsf(fmodf(x, spacing * 5)) < 0.1f;
            Color c = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            Vector3 w0 = { x, 0, center.z - ext };
            Vector3 w1 = { x, 0, center.z + ext };
            Vector2 s0 = world_to_panel(w0, center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(w1, center, span, px, py, ps, view);
            DrawLineEx(s0, s1, lw, c);
        }
        for (float z = sv; z <= center.z + ext; z += spacing) {
            bool major = fabsf(fmodf(z, spacing * 5)) < 0.1f;
            Color c = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            Vector3 w0 = { center.x - ext, 0, z };
            Vector3 w1 = { center.x + ext, 0, z };
            Vector2 s0 = world_to_panel(w0, center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(w1, center, span, px, py, ps, view);
            DrawLineEx(s0, s1, lw, c);
        }
    } else if (view == 1) {
        // FRONT: XY grid
        float sh = floorf((center.x - ext) / spacing) * spacing;
        float sv = floorf((center.y - ext) / spacing) * spacing;
        for (float x = sh; x <= center.x + ext; x += spacing) {
            bool major = fabsf(fmodf(x, spacing * 5)) < 0.1f;
            Color c = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            Vector3 w0 = { x, center.y - ext, center.z };
            Vector3 w1 = { x, center.y + ext, center.z };
            Vector2 s0 = world_to_panel(w0, center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(w1, center, span, px, py, ps, view);
            DrawLineEx(s0, s1, lw, c);
        }
        for (float y = sv; y <= center.y + ext; y += spacing) {
            bool major = fabsf(fmodf(y, spacing * 5)) < 0.1f;
            Color c = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            Vector3 w0 = { center.x - ext, y, center.z };
            Vector3 w1 = { center.x + ext, y, center.z };
            Vector2 s0 = world_to_panel(w0, center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(w1, center, span, px, py, ps, view);
            DrawLineEx(s0, s1, lw, c);
        }
    } else {
        // RIGHT: ZY grid
        float sh = floorf((center.z - ext) / spacing) * spacing;
        float sv = floorf((center.y - ext) / spacing) * spacing;
        for (float z = sh; z <= center.z + ext; z += spacing) {
            bool major = fabsf(fmodf(z, spacing * 5)) < 0.1f;
            Color c = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            Vector3 w0 = { center.x, center.y - ext, z };
            Vector3 w1 = { center.x, center.y + ext, z };
            Vector2 s0 = world_to_panel(w0, center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(w1, center, span, px, py, ps, view);
            DrawLineEx(s0, s1, lw, c);
        }
        for (float y = sv; y <= center.y + ext; y += spacing) {
            bool major = fabsf(fmodf(y, spacing * 5)) < 0.1f;
            Color c = major ? grid_major : grid_minor;
            float lw = major ? 1.5f : 1.0f;
            Vector3 w0 = { center.x, y, center.z - ext };
            Vector3 w1 = { center.x, y, center.z + ext };
            Vector2 s0 = world_to_panel(w0, center, span, px, py, ps, view);
            Vector2 s1 = world_to_panel(w1, center, span, px, py, ps, view);
            DrawLineEx(s0, s1, lw, c);
        }
    }
}

// ── Draw (2D overlays after texture composite) ───────────────────────────────

void ortho_panel_draw(const ortho_panel_t *op, int screen_h, int hud_bar_h,
                      view_mode_t view_mode, Font font,
                      const vehicle_t *vehicles, int vehicle_count,
                      int selected, int trail_mode,
                      int corr_mode, const int *pinned, int pinned_count,
                      bool show_axes)
{
    if (!op->visible) return;

    int margin = 8;
    int gap = 4;

    // Available height = screen minus HUD bar minus margins
    int avail_h = screen_h - hud_bar_h - margin * 2;
    int ps = (avail_h - gap * (ORTHO_VIEW_COUNT - 1)) / ORTHO_VIEW_COUNT;
    if (ps < 60) ps = 60;

    int total_h = ps * ORTHO_VIEW_COUNT + gap * (ORTHO_VIEW_COUNT - 1);
    int start_y = screen_h - hud_bar_h - total_h - margin;

    Color border_col, label_col, scale_col, cross_col;
    if (view_mode == VIEW_1988) {
        border_col = (Color){ 255, 20, 100, 120 };
        label_col  = (Color){ 255, 20, 100, 200 };
        scale_col  = (Color){ 255, 20, 100, 160 };
        cross_col  = (Color){ 255, 20, 100, 60 };
    } else if (view_mode == VIEW_REZ) {
        border_col = (Color){ 0, 204, 218, 120 };
        label_col  = (Color){ 0, 204, 218, 200 };
        scale_col  = (Color){ 0, 204, 218, 160 };
        cross_col  = (Color){ 0, 204, 218, 60 };
    } else {
        border_col = (Color){ 180, 180, 200, 120 };
        label_col  = (Color){ 200, 200, 220, 200 };
        scale_col  = (Color){ 180, 180, 200, 160 };
        cross_col  = (Color){ 180, 180, 200, 60 };
    }

    // Match HUD scaling: powf(screen_h / 720.0f, 0.7f)
    float s = powf(screen_h / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;
    float font_size = 12 * s;

    Color sky_col, gnd_col;
    get_sky_ground(view_mode, &sky_col, &gnd_col);

    Color grid_minor, grid_major;
    get_grid_colors(view_mode, &grid_minor, &grid_major);

    for (int i = 0; i < ORTHO_VIEW_COUNT; i++) {
        float x = (float)margin;
        float y = (float)(start_y + i * (ps + gap));

        // Background
        DrawRectangle((int)x, (int)y, ps, ps, (Color){ 5, 5, 12, 200 });

        // Render texture scaled to panel size (flip Y)
        Rectangle src = { 0, (float)ORTHO_TEX_SIZE, (float)ORTHO_TEX_SIZE, (float)(-ORTHO_TEX_SIZE) };
        Rectangle dst = { x, y, (float)ps, (float)ps };
        DrawTexturePro(op->targets[i].texture, src, dst, (Vector2){0, 0}, 0.0f, WHITE);

        // Clip 2D overlays to panel
        BeginScissorMode((int)x, (int)y, ps, ps);

        Vector3 center = op->cameras[i].target;

        // 2D grid
        draw_grid_2d(center, op->ortho_span, x, y, (float)ps, i, grid_minor, grid_major);

        // Ground fill for front/right views (below Y=0)
        if (i > 0) {
            Vector3 gnd_pt = { center.x, 0.0f, center.z };
            Vector2 gp = world_to_panel(gnd_pt, center, op->ortho_span, x, y, (float)ps, i);
            int gy = (int)gp.y;
            if (gy < (int)y) gy = (int)y;
            if (gy + 1 < (int)(y + ps)) {
                DrawRectangle((int)x, gy + 1, ps, (int)(y + ps) - gy - 1, gnd_col);
            }

            // Ground line at Y=0
            Color gnd_line = grid_major;
            gnd_line.a = 220;
            Vector2 gl0 = world_to_panel((Vector3){ center.x - op->ortho_span * 2.0f, 0, center.z },
                                          center, op->ortho_span, x, y, (float)ps, i);
            Vector2 gl1 = world_to_panel((Vector3){ center.x + op->ortho_span * 2.0f, 0, center.z },
                                          center, op->ortho_span, x, y, (float)ps, i);
            DrawLineEx(gl0, gl1, 1.5f, gnd_line);
        }

        // 2D trails
        for (int vi = 0; vi < vehicle_count; vi++) {
            if (vehicles[vi].active || vehicle_count == 1) {
                draw_trail_2d(&vehicles[vi], view_mode, trail_mode,
                              center, op->ortho_span, x, y, (float)ps, i);
            }
        }

        // 2D correlation line (corr_mode == 1 only; curtain stays in 3D via render)
        if (corr_mode == 1 && pinned_count > 0) {
            for (int p = 0; p < pinned_count; p++) {
                int pidx = pinned[p];
                if (pidx >= 0 && pidx < vehicle_count && vehicles[pidx].active
                    && pidx != selected) {
                    float off_a = vehicles[selected].model_scale * 0.15f;
                    float off_b = vehicles[pidx].model_scale * 0.15f;
                    Vector3 pa = { vehicles[selected].position.x,
                                   vehicles[selected].position.y + off_a,
                                   vehicles[selected].position.z };
                    Vector3 pb = { vehicles[pidx].position.x,
                                   vehicles[pidx].position.y + off_b,
                                   vehicles[pidx].position.z };
                    Vector2 sa = world_to_panel(pa, center, op->ortho_span, x, y, (float)ps, i);
                    Vector2 sb = world_to_panel(pb, center, op->ortho_span, x, y, (float)ps, i);

                    // Gradient line: interpolate vehicle colors
                    Color ca = vehicles[selected].color;
                    Color cb = vehicles[pidx].color;
                    ca.a = 200; cb.a = 200;
                    // Draw as single line with blended midpoint color
                    Color mid = {
                        (unsigned char)((ca.r + cb.r) / 2),
                        (unsigned char)((ca.g + cb.g) / 2),
                        (unsigned char)((ca.b + cb.b) / 2),
                        200
                    };
                    DrawLineEx(sa, sb, 2.0f, mid);
                }
            }
        }

        // Correlation curtain (corr_mode == 2) drawn in 3D render pass — not here

        EndScissorMode();

        // Border
        DrawRectangleLines((int)x, (int)y, ps, ps, border_col);

        // Center crosshair
        draw_crosshair(x + ps / 2.0f, y + ps / 2.0f, 6.0f, cross_col);

        // View label (top-center)
        Vector2 ts = MeasureTextEx(font, view_labels[i], font_size, 0);
        DrawTextEx(font, view_labels[i],
            (Vector2){ x + ps / 2.0f - ts.x / 2.0f, y + 3 }, font_size, 0, label_col);

        // Scale bar
        draw_scale_bar(x, y, (float)ps, op->ortho_span, scale_col, font, font_size * 0.85f);

        // Axis gizmo (Z key toggle) — map sidebar view index to ortho_mode
        if (show_axes) {
            static const int view_to_ortho[] = { 1, 3, 6 };  // TOP, FRONT, RIGHT
            float gsc = (float)ps / 200.0f;
            if (gsc < 0.5f) gsc = 0.5f;
            if (gsc > 1.5f) gsc = 1.5f;
            float glen = 20.0f * gsc;
            draw_axis_gizmo_at(x + ps - 8 * gsc - glen,
                                y + ps - 8 * gsc - glen,
                                gsc, view_to_ortho[i], font);
        }
    }
}

// ── Fullscreen 2D ortho overlay ──────────────────────────────────────────────

// Compute the 2D world-space distance visible in a given ortho view.
// Only uses the two axes that the view projects (ignores depth axis).
static float ortho_2d_distance(Vector3 a, Vector3 b, int ortho_mode) {
    float dx, dy;
    switch (ortho_mode) {
        case 1: // TOP: X, Z
        case 2: // BOTTOM: X, Z
            dx = a.x - b.x; dy = a.z - b.z; break;
        case 3: // FRONT: X, Y
        case 4: // BACK: X, Y
            dx = a.x - b.x; dy = a.y - b.y; break;
        case 5: // LEFT: Z, Y
        case 6: // RIGHT: Z, Y
            dx = a.z - b.z; dy = a.y - b.y; break;
        default: dx = 0; dy = 0; break;
    }
    return sqrtf(dx * dx + dy * dy);
}

void ortho_draw_fullscreen_2d(const scene_t *s, const vehicle_t *vehicles,
                              int vehicle_count, int selected, int trail_mode,
                              int corr_mode, const int *pinned, int pinned_count,
                              int screen_w, int screen_h, Font font,
                              bool show_labels) {
    if (s->ortho_mode == ORTHO_NONE) return;

    int ortho_mode = (int)s->ortho_mode;
    Vector3 center = s->camera.target;
    float span = s->ortho_span;

    // 2D trails
    for (int i = 0; i < vehicle_count; i++) {
        if (vehicles[i].active || vehicle_count == 1) {
            draw_trail_2d_fullscreen(&vehicles[i], s->view_mode, trail_mode,
                                      center, span, screen_w, screen_h, ortho_mode);
        }
    }

    // 2D correlation line (corr_mode == 1 only; curtain stays in 3D)
    if (corr_mode == 1 && pinned_count > 0) {
        for (int p = 0; p < pinned_count; p++) {
            int pidx = pinned[p];
            if (pidx >= 0 && pidx < vehicle_count && vehicles[pidx].active
                && pidx != selected) {
                float off_a = vehicles[selected].model_scale * 0.15f;
                float off_b = vehicles[pidx].model_scale * 0.15f;
                Vector3 pa = { vehicles[selected].position.x,
                               vehicles[selected].position.y + off_a,
                               vehicles[selected].position.z };
                Vector3 pb = { vehicles[pidx].position.x,
                               vehicles[pidx].position.y + off_b,
                               vehicles[pidx].position.z };
                Vector2 sa = world_to_screen_ortho(pa, center, span, screen_w, screen_h, ortho_mode);
                Vector2 sb = world_to_screen_ortho(pb, center, span, screen_w, screen_h, ortho_mode);

                Color ca = vehicles[selected].color;
                Color cb = vehicles[pidx].color;
                Color mid = {
                    (unsigned char)((ca.r + cb.r) / 2),
                    (unsigned char)((ca.g + cb.g) / 2),
                    (unsigned char)((ca.b + cb.b) / 2),
                    200
                };
                DrawLineEx(sa, sb, 2.0f, mid);
            }
        }
    }

    // Distance labels for all pinned drones (any corr_mode > 0, Ctrl+L toggle)
    if (show_labels && corr_mode > 0 && pinned_count > 0 && selected >= 0 && selected < vehicle_count) {
        float sc = powf(screen_h / 720.0f, 0.7f);
        if (sc < 1.0f) sc = 1.0f;
        float fs = 14 * sc;

        for (int p = 0; p < pinned_count; p++) {
            int pidx = pinned[p];
            if (pidx >= 0 && pidx < vehicle_count && vehicles[pidx].active
                && pidx != selected) {
                Vector3 wa = vehicles[selected].position;
                Vector3 wb = vehicles[pidx].position;

                // Project both positions to screen
                Vector2 sa = world_to_screen_ortho(wa, center, span, screen_w, screen_h, ortho_mode);
                Vector2 sb = world_to_screen_ortho(wb, center, span, screen_w, screen_h, ortho_mode);

                // 2D distance in this view's plane (meters)
                float dist = ortho_2d_distance(wa, wb, ortho_mode);

                // Format label
                char buf[32];
                if (dist >= 100.0f)
                    snprintf(buf, sizeof(buf), "%.0fm", dist);
                else if (dist >= 10.0f)
                    snprintf(buf, sizeof(buf), "%.1fm", dist);
                else
                    snprintf(buf, sizeof(buf), "%.2fm", dist);

                // Midpoint between the two drones
                Vector2 mid = { (sa.x + sb.x) * 0.5f, (sa.y + sb.y) * 0.5f };

                // Pinned drone's color (not the lead drone)
                Color label_col = vehicles[pidx].color;
                label_col.a = 230;

                // Center the text on the midpoint, offset slightly above
                Vector2 tw = MeasureTextEx(font, buf, fs, 0.5f);
                DrawTextEx(font, buf,
                    (Vector2){ mid.x - tw.x * 0.5f, mid.y - tw.y - 4 },
                    fs, 0.5f, label_col);
            }
        }
    }
}

// ── Fullscreen ortho label / scale bar / crosshair ───────────────────────────

void ortho_panel_draw_fullscreen_label(int screen_w, int screen_h, int ortho_mode,
                                       float ortho_span, int view_mode, Font font,
                                       bool show_axes)
{
    if (ortho_mode == 0) return;  // ORTHO_NONE

    static const char *names[] = { "", "TOP", "BOTTOM", "FRONT", "BACK", "LEFT", "RIGHT" };
    const char *name = names[ortho_mode];

    float s = powf(screen_h / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;
    float fs = 16 * s;

    Color col;
    if (view_mode == 2)       // VIEW_1988
        col = (Color){ 255, 20, 100, 200 };
    else if (view_mode == 1)  // VIEW_REZ
        col = (Color){ 0, 204, 218, 200 };
    else
        col = (Color){ 200, 200, 220, 200 };

    // View name top-right
    Vector2 ts = MeasureTextEx(font, name, fs, 0);
    DrawTextEx(font, name, (Vector2){ screen_w - ts.x - 12, 12 }, fs, 0, col);

    // Scale bar in top-left area
    float bar_span = ortho_span;
    float meters_per_px = bar_span / (float)(screen_h < screen_w ? screen_h : screen_w);
    float target_px = screen_w * 0.15f;
    float target_m = target_px * meters_per_px;
    float nice[] = { 1, 2, 5, 10, 20, 50, 100, 200, 500 };
    float scale_m = nice[0];
    for (int i = 0; i < 9; i++) {
        if (nice[i] <= target_m) scale_m = nice[i];
        else break;
    }
    float scale_px = scale_m / meters_per_px;

    float sx = 12;
    float sy = 12 + fs + 8;
    Color scale_col = (Color){ col.r, col.g, col.b, (unsigned char)(col.a * 0.7f) };
    DrawRectangle((int)sx, (int)sy, (int)scale_px, 2, scale_col);
    DrawRectangle((int)sx, (int)(sy - 3), 1, 8, scale_col);
    DrawRectangle((int)(sx + scale_px), (int)(sy - 3), 1, 8, scale_col);

    char buf[32];
    snprintf(buf, sizeof(buf), "%.0fm", scale_m);
    DrawTextEx(font, buf, (Vector2){ sx + scale_px + 6, sy - fs * 0.4f }, fs * 0.75f, 0, scale_col);

    // Crosshair at center
    Color cross = (Color){ col.r, col.g, col.b, 40 };
    float cx = screen_w / 2.0f, cy = screen_h / 2.0f;
    DrawLine((int)(cx - 10), (int)cy, (int)(cx + 10), (int)cy, cross);
    DrawLine((int)cx, (int)(cy - 10), (int)cx, (int)(cy + 10), cross);

    // Axis gizmo (Z key toggle)
    if (show_axes) {
        float gsc = powf(screen_h / 720.0f, 0.7f);
        if (gsc < 1.0f) gsc = 1.0f;
        float glen = 30.0f * gsc;
        draw_axis_gizmo_at(screen_w - 20 * gsc - glen,
                            screen_h - 20 * gsc - glen,
                            gsc, ortho_mode, font);
    }
}

// ── Axis gizmo for ortho views ──────────────────────────────────────────────

// Draw XYZ orientation gizmo. ox,oy = origin point, sc = scale factor.
static void draw_axis_gizmo_at(float ox, float oy, float sc, int ortho_mode, Font font) {
    if (ortho_mode == 0) return;

    float len = 30.0f * sc;      // axis arrow length in pixels
    float fs = 11.0f * sc;       // label font size
    float thick = 2.0f * sc;

    // For each world axis, compute its 2D screen direction in this ortho view.
    // Use the same projection logic as world_to_screen_ortho but just for unit vectors.
    typedef struct { float dx, dy; const char *label; Color col; } axis_info;
    axis_info axes[3];

    // X axis = red, Y axis = green, Z axis = blue
    Color col_x = { 220, 60, 60, 220 };
    Color col_y = { 60, 200, 60, 220 };
    Color col_z = { 60, 100, 220, 220 };

    switch (ortho_mode) {
        case 1: // TOP: +X right, +Z down
            axes[0] = (axis_info){ 1, 0, "X", col_x };
            axes[1] = (axis_info){ 0, 0, NULL, col_y };   // Y is depth (invisible)
            axes[2] = (axis_info){ 0, 1, "Z", col_z };
            break;
        case 2: // BOTTOM: +X right, -Z down
            axes[0] = (axis_info){ 1, 0, "X", col_x };
            axes[1] = (axis_info){ 0, 0, NULL, col_y };
            axes[2] = (axis_info){ 0, -1, "Z", col_z };
            break;
        case 3: // FRONT: -X right, -Y down (Y up)
            axes[0] = (axis_info){ -1, 0, "X", col_x };
            axes[1] = (axis_info){ 0, -1, "Y", col_y };
            axes[2] = (axis_info){ 0, 0, NULL, col_z };
            break;
        case 4: // BACK: +X right, -Y down (Y up)
            axes[0] = (axis_info){ 1, 0, "X", col_x };
            axes[1] = (axis_info){ 0, -1, "Y", col_y };
            axes[2] = (axis_info){ 0, 0, NULL, col_z };
            break;
        case 5: // LEFT: +Z right, -Y down (Y up)
            axes[0] = (axis_info){ 0, 0, NULL, col_x };
            axes[1] = (axis_info){ 0, -1, "Y", col_y };
            axes[2] = (axis_info){ 1, 0, "Z", col_z };
            break;
        case 6: // RIGHT: -Z right, -Y down (Y up)
            axes[0] = (axis_info){ 0, 0, NULL, col_x };
            axes[1] = (axis_info){ 0, -1, "Y", col_y };
            axes[2] = (axis_info){ -1, 0, "Z", col_z };
            break;
        default: return;
    }

    // Draw each visible axis
    for (int i = 0; i < 3; i++) {
        if (!axes[i].label) continue;  // depth axis, skip

        float ex = ox + axes[i].dx * len;
        float ey = oy + axes[i].dy * len;

        // Arrow line
        DrawLineEx((Vector2){ox, oy}, (Vector2){ex, ey}, thick, axes[i].col);

        // Arrowhead (small triangle)
        float dx = axes[i].dx, dy = axes[i].dy;
        float ah = 6.0f * sc;  // arrowhead size
        Vector2 tip = { ex + dx * ah, ey + dy * ah };
        Vector2 left = { ex - dy * ah * 0.4f, ey + dx * ah * 0.4f };
        Vector2 right = { ex + dy * ah * 0.4f, ey - dx * ah * 0.4f };
        DrawTriangle(tip, left, right, axes[i].col);

        // Label at tip
        Vector2 tw = MeasureTextEx(font, axes[i].label, fs, 0);
        float lx = tip.x + dx * 4 - tw.x * 0.5f;
        float ly = tip.y + dy * 4 - tw.y * 0.5f;
        // Nudge label away from the arrow direction
        if (fabsf(dx) > 0.5f) lx = tip.x + dx * 6 - (dx > 0 ? 0 : tw.x);
        if (fabsf(dy) > 0.5f) ly = tip.y + dy * 6 - (dy > 0 ? 0 : tw.y);
        DrawTextEx(font, axes[i].label, (Vector2){lx, ly}, fs, 0, axes[i].col);
    }

    // Origin dot
    DrawCircle((int)ox, (int)oy, 3.0f * sc, (Color){200, 200, 200, 180});
}

// ── 3D axis gizmo at world position ──────────────────────────────────────────

void draw_axis_gizmo_3d(Vector3 pos, float scale, Quaternion rot) {
    float len = scale;

    // Rotate unit axis vectors by the drone's quaternion
    Vector3 ax = Vector3RotateByQuaternion((Vector3){len, 0, 0}, rot);
    Vector3 ay = Vector3RotateByQuaternion((Vector3){0, len, 0}, rot);
    Vector3 az = Vector3RotateByQuaternion((Vector3){0, 0, len}, rot);

    Vector3 tip_x = { pos.x + ax.x, pos.y + ax.y, pos.z + ax.z };
    Vector3 tip_y = { pos.x + ay.x, pos.y + ay.y, pos.z + ay.z };
    Vector3 tip_z = { pos.x + az.x, pos.y + az.y, pos.z + az.z };

    // X = red, Y = green, Z = blue
    Color col_x = { 220,  60,  60, 240 };
    Color col_y = {  60, 200,  60, 240 };
    Color col_z = {  60, 100, 220, 240 };

    DrawLine3D(pos, tip_x, col_x);
    DrawLine3D(pos, tip_y, col_y);
    DrawLine3D(pos, tip_z, col_z);

    float r = len * 0.06f;
    DrawSphere(tip_x, r, col_x);
    DrawSphere(tip_y, r, col_y);
    DrawSphere(tip_z, r, col_z);
}

// ── Cleanup ──────────────────────────────────────────────────────────────────

void ortho_panel_cleanup(ortho_panel_t *op) {
    for (int i = 0; i < ORTHO_VIEW_COUNT; i++) {
        UnloadRenderTexture(op->targets[i]);
    }
}
