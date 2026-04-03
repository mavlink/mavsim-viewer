#include "hud_help.h"
#include "raylib.h"
#include "theme.h"
#include <math.h>
#include <stddef.h>

typedef struct { const char *key; const char *action; } shortcut_entry_t;

void hud_draw_help(Font font_value, Font font_label,
                   int screen_w, int screen_h, const theme_t *theme) {
    Color accent = theme->hud_accent;
    Color border = theme->hud_border;
    Color value_color = theme->hud_value;
    Color dim_color = theme->hud_dim;

    float s = powf(screen_h / 720.0f, 0.7f);
    if (s < 1.0f) s = 1.0f;

    // Dim the background
    DrawRectangle(0, 0, screen_w, screen_h, (Color){0, 0, 0, 160});

    // Grouped shortcut entries: NULL key = section header
    // 3 balanced columns
    shortcut_entry_t col1[] = {
        {NULL,          "VIEW"},
        {"C",           "Camera mode (Chase/FPV/Free)"},
        {"V",           "View mode (Grid/Rez/Snow)"},
        {"F",           "Terrain texture"},
        {"K",           "Arm colors (classic/modern)"},
        {"O",           "Orthographic side panel"},
        {"Ctrl+D",      "Debug overlay"},
        {"Alt+1-7",     "Ortho views (1=perspective)"},
        {NULL,          "VEHICLE"},
        {"M",           "Switch variant (Sh: all)"},
        {NULL,          "MULTI-VEHICLE"},
        {"TAB",         "Next vehicle"},
        {"[ / ]",       "Prev / next vehicle"},
        {"1-9",         "Select vehicle"},
        {"Sh+1-9",      "Pin / unpin to HUD"},
    };
    shortcut_entry_t col2[] = {
        {NULL,          "HUD"},
        {"H",           "Toggle HUD"},
        {"T",           "Cycle trail mode"},
        {"Sh+T",        "Correlation (curtain/line)"},
        {"G",           "Ground track projection"},
        {"?",           "Toggle this help"},
        {NULL,          "CAMERA"},
        {"Drag",        "Orbit (chase) / look (free)"},
        {"Scroll",      "Zoom FOV / distance"},
        {"WASDQE",      "Fly (free cam)"},
        {"Alt+Scrl",    "Zoom ortho span"},
        {NULL,          "EDGE INDICATORS"},
        {"Ctrl+L",      "Toggle edge indicators"},
    };
    shortcut_entry_t col3[] = {
        {NULL,          "REPLAY"},
        {"Space",       "Pause / resume"},
        {"+/-",         "Playback speed"},
        {"<-/->",       "Seek 5s"},
        {"Sh+<-/->",    "Frame step"},
        {"Ctrl+Sh+<->", "Seek 1s"},
        {"L",           "Toggle labels"},
        {"Sh+L",        "Toggle loop"},
        {"I",           "Interpolation"},
        {"R",           "Restart"},
        {NULL,          "MARKERS"},
        {"B",           "Drop marker"},
        {"B then L",    "Drop + label marker"},
        {"Sh+B",        "Delete current marker"},
        {"[ / ]",       "Jump to prev/next marker"},
        {"Sh+[ / ]",    "Track from marker"},
        {"A",           "Takeoff alignment"},
    };

    int counts[3] = {
        sizeof(col1) / sizeof(col1[0]),
        sizeof(col2) / sizeof(col2[0]),
        sizeof(col3) / sizeof(col3[0]),
    };
    shortcut_entry_t *cols[3] = { col1, col2, col3 };

    // Find tallest column (entries + headers)
    int max_rows = 0;
    int max_headers = 0;
    for (int c = 0; c < 3; c++) {
        if (counts[c] > max_rows) max_rows = counts[c];
        int hdr = 0;
        for (int i = 0; i < counts[c]; i++) if (!cols[c][i].key) hdr++;
        if (hdr > max_headers) max_headers = hdr;
    }

    // Scale font to fit screen height
    float base_fs = 14 * s;
    float base_line_h = 20 * s;
    float base_group_pad = 5 * s;
    float title_h = 36 * s;
    float margin = 20 * s;
    float needed_h = title_h + max_rows * base_line_h + max_headers * base_group_pad + margin * 2;
    float scale_down = 1.0f;
    if (needed_h > screen_h * 0.95f) {
        scale_down = (screen_h * 0.95f) / needed_h;
        if (scale_down < 0.6f) scale_down = 0.6f;
    }

    float help_fs_title = 20 * s * scale_down;
    float help_fs = base_fs * scale_down;
    float help_fs_group = 12 * s * scale_down;
    float line_h = base_line_h * scale_down;
    float group_top_pad = base_group_pad * scale_down;
    float col_gap = 16 * s * scale_down;

    // Measure max key width across all columns
    float max_key_w = 0;
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < counts[c]; i++) {
            if (!cols[c][i].key) continue;
            Vector2 kw = MeasureTextEx(font_value, cols[c][i].key, help_fs, 0.5f);
            if (kw.x > max_key_w) max_key_w = kw.x;
        }
    }

    // Measure max action text width across all columns
    float max_action_w = 0;
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < counts[c]; i++) {
            if (!cols[c][i].key) continue;
            Vector2 aw = MeasureTextEx(font_label, cols[c][i].action, help_fs, 0.5f);
            if (aw.x > max_action_w) max_action_w = aw.x;
        }
    }

    float col_w = max_key_w + col_gap + max_action_w;
    float mid_gap = 24 * s * scale_down;
    float panel_w = col_w * 3 + mid_gap * 2 + margin * 2;
    // Clamp panel width to screen
    if (panel_w > screen_w * 0.98f) panel_w = screen_w * 0.98f;
    float panel_h = title_h * scale_down + max_rows * line_h + max_headers * group_top_pad + margin * 2;
    float panel_x = (screen_w - panel_w) / 2.0f;
    float panel_y = (screen_h - panel_h) / 2.0f;
    if (panel_y < 4) panel_y = 4;

    // Panel background
    DrawRectangleRounded(
        (Rectangle){panel_x, panel_y, panel_w, panel_h},
        0.02f, 8, (Color){10, 14, 20, 230});
    DrawRectangleRoundedLinesEx(
        (Rectangle){panel_x, panel_y, panel_w, panel_h},
        0.02f, 8, 1.0f, border);

    // Title
    const char *title = "Keyboard Shortcuts";
    Vector2 tw = MeasureTextEx(font_label, title, help_fs_title, 0.5f);
    DrawTextEx(font_label, title,
               (Vector2){panel_x + (panel_w - tw.x) / 2, panel_y + 10 * s * scale_down},
               help_fs_title, 0.5f, accent);

    // Draw 3 columns
    float ey_start = panel_y + title_h * scale_down;
    // Recompute col_w to distribute evenly within actual panel
    float usable_w = panel_w - margin * 2 - mid_gap * 2;
    float actual_col_w = usable_w / 3.0f;

    for (int c = 0; c < 3; c++) {
        float key_x = panel_x + margin + c * (actual_col_w + mid_gap);
        float action_x = key_x + max_key_w + col_gap;
        float ey = ey_start;
        for (int i = 0; i < counts[c]; i++) {
            if (!cols[c][i].key) {
                // Section header
                if (i > 0) ey += group_top_pad;
                DrawTextEx(font_label, cols[c][i].action,
                           (Vector2){key_x, ey}, help_fs_group, 0.5f, dim_color);
            } else {
                DrawTextEx(font_value, cols[c][i].key,
                           (Vector2){key_x, ey}, help_fs, 0.5f, accent);
                DrawTextEx(font_label, cols[c][i].action,
                           (Vector2){action_x, ey}, help_fs, 0.5f, value_color);
            }
            ey += line_h;
        }
    }
}
