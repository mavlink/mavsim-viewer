#include "ortho_panel.h"
#include "rlgl.h"
#include <stdio.h>
#include <math.h>

static const char *view_labels[ORTHO_VIEW_COUNT] = { "TOP", "FRONT", "RIGHT" };

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

void ortho_panel_render(ortho_panel_t *op, const scene_t *s,
                        const vehicle_t *vehicles, int vehicle_count,
                        int selected, view_mode_t view_mode,
                        int trail_mode)
{
    for (int v = 0; v < ORTHO_VIEW_COUNT; v++) {
        BeginTextureMode(op->targets[v]);
            ClearBackground((Color){ 10, 10, 18, 255 });
            BeginMode3D(op->cameras[v]);
                scene_draw(s);

                float ext = op->ortho_span * 2.0f;
                // Pick grid spacing based on span
                float spacing = 10.0f;
                if (op->ortho_span > 200.0f) spacing = 50.0f;
                else if (op->ortho_span > 80.0f) spacing = 20.0f;
                else if (op->ortho_span < 20.0f) spacing = 2.0f;

                Color grid_minor = { 40, 40, 55, 60 };
                Color grid_major = { 60, 60, 80, 100 };

                Vector3 center = op->cameras[v].target;

                if (v == 0) {
                    // Top-down: XZ grid
                    float snap_x = floorf(center.x / spacing) * spacing;
                    float snap_z = floorf(center.z / spacing) * spacing;
                    for (float g = -ext; g <= ext; g += spacing) {
                        bool major = fabsf(fmodf(snap_x + g, spacing * 5)) < 0.1f;
                        Color c = major ? grid_major : grid_minor;
                        DrawLine3D((Vector3){snap_x + g, 0.01f, center.z - ext},
                                   (Vector3){snap_x + g, 0.01f, center.z + ext}, c);
                    }
                    for (float g = -ext; g <= ext; g += spacing) {
                        bool major = fabsf(fmodf(snap_z + g, spacing * 5)) < 0.1f;
                        Color c = major ? grid_major : grid_minor;
                        DrawLine3D((Vector3){center.x - ext, 0.01f, snap_z + g},
                                   (Vector3){center.x + ext, 0.01f, snap_z + g}, c);
                    }
                } else if (v == 1) {
                    // Front: XY grid (constant Z)
                    float snap_x = floorf(center.x / spacing) * spacing;
                    float snap_y = floorf(center.y / spacing) * spacing;
                    float z = center.z;
                    for (float g = -ext; g <= ext; g += spacing) {
                        bool major = fabsf(fmodf(snap_x + g, spacing * 5)) < 0.1f;
                        Color c = major ? grid_major : grid_minor;
                        DrawLine3D((Vector3){snap_x + g, center.y - ext, z},
                                   (Vector3){snap_x + g, center.y + ext, z}, c);
                    }
                    for (float g = -ext; g <= ext; g += spacing) {
                        bool major = fabsf(fmodf(snap_y + g, spacing * 5)) < 0.1f;
                        Color c = major ? grid_major : grid_minor;
                        DrawLine3D((Vector3){center.x - ext, snap_y + g, z},
                                   (Vector3){center.x + ext, snap_y + g, z}, c);
                    }
                } else {
                    // Side (Right): ZY grid (constant X)
                    float snap_z = floorf(center.z / spacing) * spacing;
                    float snap_y = floorf(center.y / spacing) * spacing;
                    float x = center.x;
                    for (float g = -ext; g <= ext; g += spacing) {
                        bool major = fabsf(fmodf(snap_z + g, spacing * 5)) < 0.1f;
                        Color c = major ? grid_major : grid_minor;
                        DrawLine3D((Vector3){x, center.y - ext, snap_z + g},
                                   (Vector3){x, center.y + ext, snap_z + g}, c);
                    }
                    for (float g = -ext; g <= ext; g += spacing) {
                        bool major = fabsf(fmodf(snap_y + g, spacing * 5)) < 0.1f;
                        Color c = major ? grid_major : grid_minor;
                        DrawLine3D((Vector3){x, snap_y + g, center.z - ext},
                                   (Vector3){x, snap_y + g, center.z + ext}, c);
                    }
                }

                // Ground line at Y=0 (side/front views)
                if (v > 0) {
                    Color gnd_line = { 120, 120, 150, 200 };
                    DrawLine3D((Vector3){-ext, 0, -ext}, (Vector3){ ext, 0, -ext}, gnd_line);
                    DrawLine3D((Vector3){-ext, 0,  ext}, (Vector3){ ext, 0,  ext}, gnd_line);
                    DrawLine3D((Vector3){-ext, 0, -ext}, (Vector3){-ext, 0,  ext}, gnd_line);
                    DrawLine3D((Vector3){ ext, 0, -ext}, (Vector3){ ext, 0,  ext}, gnd_line);
                }

                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        vehicle_draw((vehicle_t *)&vehicles[i], view_mode, i == selected,
                                     trail_mode, false, op->cameras[v].position);
                    }
                }
            EndMode3D();

            // 2D ground fill below Y=0 for side/front views
            if (v > 0) {
                Vector3 gnd_pt = op->cameras[v].target;
                gnd_pt.y = 0.0f;
                Vector2 sp = GetWorldToScreen(gnd_pt, op->cameras[v]);
                int gy = (int)sp.y;
                if (gy < 0) gy = 0;
                if (gy + 1 < ORTHO_TEX_SIZE) {
                    DrawRectangle(0, gy + 1, ORTHO_TEX_SIZE, ORTHO_TEX_SIZE - gy - 1, (Color){ 2, 2, 6, 180 });
                }
            }
        EndTextureMode();
    }
}

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

static void draw_crosshair(float cx, float cy, float size, Color col) {
    DrawLine((int)(cx - size), (int)cy, (int)(cx + size), (int)cy, col);
    DrawLine((int)cx, (int)(cy - size), (int)cx, (int)(cy + size), col);
}

void ortho_panel_draw(const ortho_panel_t *op, int screen_h, int hud_bar_h,
                      view_mode_t view_mode, Font font)
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

    for (int i = 0; i < ORTHO_VIEW_COUNT; i++) {
        float x = (float)margin;
        float y = (float)(start_y + i * (ps + gap));

        // Background
        DrawRectangle((int)x, (int)y, ps, ps, (Color){ 5, 5, 12, 200 });

        // Render texture scaled to panel size (flip Y)
        Rectangle src = { 0, (float)ORTHO_TEX_SIZE, (float)ORTHO_TEX_SIZE, (float)(-ORTHO_TEX_SIZE) };
        Rectangle dst = { x, y, (float)ps, (float)ps };
        DrawTexturePro(op->targets[i].texture, src, dst, (Vector2){0, 0}, 0.0f, WHITE);

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
    }
}

void ortho_panel_draw_fullscreen_label(int screen_w, int screen_h, int ortho_mode,
                                       float ortho_span, int view_mode, Font font)
{
    if (ortho_mode == 0) return;  // ORTHO_NONE

    static const char *names[] = { "", "TOP", "BOTTOM", "FRONT", "BACK", "LEFT", "RIGHT" };
    const char *name = names[ortho_mode];

    float s = powf(screen_h / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;
    float fs = 16 * s;

    Color col;
    if (view_mode == 2)       // VIEW_1988 (can't use enum directly due to header order)
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
}

void ortho_panel_cleanup(ortho_panel_t *op) {
    for (int i = 0; i < ORTHO_VIEW_COUNT; i++) {
        UnloadRenderTexture(op->targets[i]);
    }
}
