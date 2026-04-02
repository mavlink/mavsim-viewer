#include "debug_panel.h"
#include "rlgl.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

void debug_panel_init(debug_panel_t *d) {
    memset(d, 0, sizeof(*d));
    d->visible = false;
}

void debug_panel_update(debug_panel_t *d, float dt) {
    if (!d->visible) return;

    d->update_timer += dt;
    float ft = GetFrameTime() * 1000.0f;

    d->fps_history[d->history_idx] = (float)GetFPS();
    d->frametime_history[d->history_idx] = ft;
    d->history_idx = (d->history_idx + 1) % DEBUG_FPS_HISTORY;

    if (ft > d->peak_frametime) d->peak_frametime = ft;

    // Reset peak every 5 seconds
    if (d->update_timer > 5.0f) {
        d->update_timer = 0.0f;
        d->peak_frametime = ft;
    }
}

static void draw_graph(float x, float y, float w, float h,
                       const float *data, int count, int head,
                       float min_val, float max_val,
                       Color line_col, Color bg_col, Color grid_col)
{
    // Background
    DrawRectangle((int)x, (int)y, (int)w, (int)h, bg_col);

    // Grid lines (3 horizontal)
    for (int i = 1; i <= 3; i++) {
        float gy = y + h * (float)i / 4.0f;
        DrawLine((int)x, (int)gy, (int)(x + w), (int)gy, grid_col);
    }

    // Data line
    float range = max_val - min_val;
    if (range < 0.001f) range = 1.0f;

    for (int i = 1; i < count; i++) {
        int idx0 = (head - count + i - 1 + count * 2) % count;
        int idx1 = (head - count + i + count * 2) % count;
        float v0 = (data[idx0] - min_val) / range;
        float v1 = (data[idx1] - min_val) / range;
        if (v0 < 0) v0 = 0; if (v0 > 1) v0 = 1;
        if (v1 < 0) v1 = 0; if (v1 > 1) v1 = 1;

        float x0 = x + (float)(i - 1) / (float)(count - 1) * w;
        float x1 = x + (float)i / (float)(count - 1) * w;
        float y0 = y + h - v0 * h;
        float y1 = y + h - v1 * h;

        DrawLine((int)x0, (int)y0, (int)x1, (int)y1, line_col);
    }

    // Border
    DrawRectangleLines((int)x, (int)y, (int)w, (int)h, (Color){ 60, 60, 80, 100 });
}

void debug_panel_draw(const debug_panel_t *d, int screen_w, int screen_h,
                      view_mode_t view_mode, Font font,
                      int vehicle_count, int active_count,
                      int total_trail_points, Vector3 vehicle_pos,
                      bool ref_rejected)
{
    if (!d->visible) return;

    float s = powf(screen_h / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;
    float fs = 11 * s;
    float fs_title = 13 * s;
    float line_h = fs + 3 * s;

    float panel_w = 200 * s;
    float margin = 8;
    float px = screen_w - panel_w - margin;
    float py = margin;
    float graph_h = 40 * s;
    float graph_w = panel_w - 16 * s;
    float section_gap = 8 * s;

    // Colors per view mode
    Color accent, text_col, dim_col, bg, graph_bg, graph_grid;
    if (view_mode == VIEW_1988) {
        accent     = (Color){ 255, 20, 100, 220 };
        text_col   = (Color){ 255, 200, 210, 230 };
        dim_col    = (Color){ 255, 20, 100, 100 };
    } else if (view_mode == VIEW_REZ) {
        accent     = (Color){ 0, 204, 218, 220 };
        text_col   = (Color){ 180, 230, 235, 230 };
        dim_col    = (Color){ 0, 204, 218, 100 };
    } else {
        accent     = (Color){ 120, 180, 255, 220 };
        text_col   = (Color){ 200, 210, 225, 230 };
        dim_col    = (Color){ 120, 180, 255, 100 };
    }
    bg         = (Color){ 8, 8, 16, 210 };
    graph_bg   = (Color){ 4, 4, 10, 200 };
    graph_grid = (Color){ 40, 40, 60, 80 };

    // Compute total panel height
    float total_h = 0;
    total_h += line_h * 2 + graph_h + line_h;          // FPS
    total_h += section_gap;
    total_h += line_h * 2 + graph_h + line_h;          // Frametime
    total_h += section_gap;
    total_h += line_h * 3;                               // Render
    total_h += section_gap;
    total_h += line_h * 3;                               // Memory
    total_h += section_gap;
    total_h += line_h * 4;                               // Position
    total_h += margin * 2;

    // Panel background
    DrawRectangle((int)px, (int)py, (int)panel_w, (int)total_h, bg);
    DrawRectangleLines((int)px, (int)py, (int)panel_w, (int)total_h, dim_col);

    float cx = px + 8 * s;  // content x
    float cy = py + margin;  // content y
    char buf[128];

    // --- FPS ---
    DrawTextEx(font, "FPS", (Vector2){cx, cy}, fs_title, 0, accent);
    cy += line_h;

    int fps = GetFPS();
    snprintf(buf, sizeof(buf), "%d", fps);
    Color fps_col = fps >= 55 ? (Color){80, 255, 120, 255} :
                    fps >= 30 ? (Color){255, 220, 40, 255} :
                                (Color){255, 60, 40, 255};
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs * 1.4f, 0, fps_col);
    cy += line_h;

    // FPS graph
    float fps_min = 0, fps_max = 80;
    draw_graph(cx, cy, graph_w, graph_h,
               d->fps_history, DEBUG_FPS_HISTORY, d->history_idx,
               fps_min, fps_max, fps_col, graph_bg, graph_grid);

    // Min/max labels on graph
    snprintf(buf, sizeof(buf), "%.0f", fps_max);
    DrawTextEx(font, buf, (Vector2){cx + graph_w + 2, cy}, fs * 0.7f, 0, dim_col);
    snprintf(buf, sizeof(buf), "%.0f", fps_min);
    DrawTextEx(font, buf, (Vector2){cx + graph_w + 2, cy + graph_h - fs * 0.7f}, fs * 0.7f, 0, dim_col);
    cy += graph_h;

    // FPS range
    float fmin = 999, fmax = 0;
    for (int i = 0; i < DEBUG_FPS_HISTORY; i++) {
        if (d->fps_history[i] > 0) {
            if (d->fps_history[i] < fmin) fmin = d->fps_history[i];
            if (d->fps_history[i] > fmax) fmax = d->fps_history[i];
        }
    }
    if (fmin > 900) fmin = 0;
    snprintf(buf, sizeof(buf), "min %.0f  max %.0f", fmin, fmax);
    DrawTextEx(font, buf, (Vector2){cx, cy + 2}, fs * 0.8f, 0, dim_col);
    cy += line_h;

    cy += section_gap;

    // --- FRAME TIME ---
    DrawTextEx(font, "FRAME TIME", (Vector2){cx, cy}, fs_title, 0, accent);
    cy += line_h;

    float ft = GetFrameTime() * 1000.0f;
    snprintf(buf, sizeof(buf), "%.1f ms", ft);
    Color ft_col = ft <= 17.0f ? (Color){80, 255, 120, 255} :
                   ft <= 33.0f ? (Color){255, 220, 40, 255} :
                                 (Color){255, 60, 40, 255};
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs * 1.2f, 0, ft_col);
    cy += line_h;

    // Frametime graph
    draw_graph(cx, cy, graph_w, graph_h,
               d->frametime_history, DEBUG_FPS_HISTORY, d->history_idx,
               0, 33.3f, ft_col, graph_bg, graph_grid);

    // Target line at 16.67ms
    float target_y = cy + graph_h - (16.67f / 33.3f) * graph_h;
    DrawLine((int)cx, (int)target_y, (int)(cx + graph_w), (int)target_y, (Color){80, 255, 120, 40});

    snprintf(buf, sizeof(buf), "33ms");
    DrawTextEx(font, buf, (Vector2){cx + graph_w + 2, cy}, fs * 0.7f, 0, dim_col);
    snprintf(buf, sizeof(buf), "0");
    DrawTextEx(font, buf, (Vector2){cx + graph_w + 2, cy + graph_h - fs * 0.7f}, fs * 0.7f, 0, dim_col);
    cy += graph_h;

    snprintf(buf, sizeof(buf), "peak %.1f ms", d->peak_frametime);
    DrawTextEx(font, buf, (Vector2){cx, cy + 2}, fs * 0.8f, 0, dim_col);
    cy += line_h;

    cy += section_gap;

    // --- RENDER ---
    DrawTextEx(font, "RENDER", (Vector2){cx, cy}, fs_title, 0, accent);
    cy += line_h;

    snprintf(buf, sizeof(buf), "Vehicles    %d / %d", active_count, vehicle_count);
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs, 0, text_col);
    cy += line_h;

    snprintf(buf, sizeof(buf), "Trail pts   %d", total_trail_points);
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs, 0, text_col);
    cy += line_h;

    cy += section_gap;

    // --- MEMORY ---
    DrawTextEx(font, "MEMORY", (Vector2){cx, cy}, fs_title, 0, accent);
    cy += line_h;

    // Trail buffer estimate
    int trail_mem_kb = vehicle_count * 2000 * (12 + 4 + 4 + 4) / 1024;  // Vec3 + 3 floats
    snprintf(buf, sizeof(buf), "Trails      %d KB", trail_mem_kb);
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs, 0, text_col);
    cy += line_h;

    int total_kb = trail_mem_kb + 1024 + 5000;  // +1MB fonts +5MB raylib overhead
    snprintf(buf, sizeof(buf), "Est total   ~%d MB", total_kb / 1024);
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs, 0, accent);
    cy += line_h;

    cy += section_gap;

    // --- POSITION ---
    DrawTextEx(font, "POSITION", (Vector2){cx, cy}, fs_title, 0, accent);
    if (ref_rejected) {
        Vector2 pw = MeasureTextEx(font, "POSITION", fs_title, 0);
        Color warn_col = (Color){255, 160, 40, 220};
        DrawTextEx(font, " BAD REF", (Vector2){cx + pw.x, cy}, fs_title * 0.8f, 0, warn_col);
    }
    cy += line_h;

    snprintf(buf, sizeof(buf), "X  %+.1f", vehicle_pos.x);
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs, 0, text_col);
    cy += line_h;

    snprintf(buf, sizeof(buf), "Y  %+.1f", vehicle_pos.y);
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs, 0, text_col);
    cy += line_h;

    snprintf(buf, sizeof(buf), "Z  %+.1f", vehicle_pos.z);
    DrawTextEx(font, buf, (Vector2){cx, cy}, fs, 0, text_col);
}
