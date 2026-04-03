#include "replay_conflict.h"
#include "raylib.h"
#include "scene.h"
#include "theme.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

conflict_result_t replay_detect_conflict(const data_source_t *sources,
                                         int num_files) {
    conflict_result_t result = {false, false};

    // Tier assignment
    int tier3_count = 0;
    for (int i = 0; i < num_files; i++) {
        if (sources[i].playback.home_from_topic) {
            // Tier 1
        } else if (sources[i].home.valid) {
            // Tier 2
        } else {
            tier3_count++;  // Tier 3
        }
    }

    // Detect conflicts
    bool conflict = false;
    if (tier3_count > 0) conflict = true;

    // Pairwise distance check (only if all have valid homes)
    if (!conflict) {
        for (int i = 0; i < num_files && !conflict; i++) {
            if (!sources[i].home.valid) continue;
            for (int j = i + 1; j < num_files && !conflict; j++) {
                if (!sources[j].home.valid) continue;
                double dlat_m = ((double)sources[i].home.lat - (double)sources[j].home.lat) / 1e7 * 111319.5;
                double lat_rad = (sources[i].home.lat / 1e7) * (M_PI / 180.0);
                double dlon_m = ((double)sources[i].home.lon - (double)sources[j].home.lon) / 1e7 * 111319.5 * cos(lat_rad);
                double dalt_m = ((double)sources[i].home.alt - (double)sources[j].home.alt) / 1000.0;
                double dist = sqrt(dlat_m * dlat_m + dlon_m * dlon_m + dalt_m * dalt_m);
                if (dist < 0.1) conflict = true;              // identical position
                if (dist > 500.0) { conflict = true; result.conflict_far = true; }  // too far
            }
        }
    }

    result.conflict_detected = conflict;
    return result;
}

int draw_prompt_dialog(const char *title, const char *subtitle,
                       const char **options, int option_count,
                       const theme_t *theme, Font font_label, Font font_value,
                       const scene_t *scene)
{
    int choice = 0;
    while (choice == 0 && !WindowShouldClose()) {
        int sw = GetScreenWidth();
        int sh = GetScreenHeight();
        float s = powf(sh / 720.0f, 0.7f);
        if (s < 1.0f) s = 1.0f;

        int key = GetKeyPressed();
        for (int o = 0; o < option_count && o < 9; o++) {
            if (key == KEY_ONE + o) { choice = o + 1; break; }
        }

        float fs_title = 15 * s, fs_option = 20 * s, fs_hint = 12 * s;
        float line_h = 36 * s, pad = 16 * s, inner_gap = 20 * s;

        // Measure option widths
        float key_num_w = MeasureTextEx(font_value, "1", fs_option, 0.5f).x;
        float gap = 16 * s;
        float widest = 0;
        for (int o = 0; o < option_count; o++) {
            float w = key_num_w + gap + MeasureTextEx(font_label, options[o], fs_option, 0.5f).x;
            if (w > widest) widest = w;
        }

        // Measure title
        Vector2 tmw = MeasureTextEx(font_label, title, fs_title, 0.5f);
        Vector2 trw = MeasureTextEx(font_label, subtitle, fs_title, 0.5f);
        float title_total = tmw.x + trw.x;

        // Box dimensions
        float box_w = 520 * s;
        float opts_block_h = line_h * option_count;
        float box_h = pad + fs_title + inner_gap + opts_block_h + inner_gap * 0.3f + fs_hint + pad * 0.5f;
        float cx = sw / 2.0f, cy = sh / 2.0f;
        float bx = cx - box_w / 2.0f, by = cy - box_h / 2.0f;

        BeginDrawing();
            scene_draw_sky(scene);
            BeginMode3D(scene->camera);
                scene_draw(scene);
            EndMode3D();
            DrawRectangle(0, 0, sw, sh, theme->prompt_scrim);

            Rectangle box = {bx, by, box_w, box_h};
            DrawRectangleRounded(box, 0.06f, 8, theme->prompt_box_bg);
            DrawRectangleRoundedLinesEx(box, 0.06f, 8, 1.5f * s, theme->prompt_border);

            // Title + subtitle
            float title_x = cx - title_total / 2.0f;
            DrawTextEx(font_label, title,
                       (Vector2){title_x, by + pad}, fs_title, 0.5f, theme->prompt_title);
            DrawTextEx(font_label, subtitle,
                       (Vector2){title_x + tmw.x, by + pad}, fs_title, 0.5f, theme->prompt_subtitle);

            // Options
            float left = cx - widest / 2.0f;
            float opts_y = by + pad + fs_title + inner_gap;
            for (int o = 0; o < option_count; o++) {
                char num[2] = {(char)('1' + o), '\0'};
                DrawTextEx(font_value, num,
                           (Vector2){left, opts_y + o * line_h}, fs_option, 0.5f, theme->prompt_key);
                DrawTextEx(font_label, options[o],
                           (Vector2){left + key_num_w + gap, opts_y + o * line_h}, fs_option, 0.5f, theme->prompt_text);
            }

            // Hint
            char hint[32];
            snprintf(hint, sizeof(hint), "Press 1%s%d", option_count > 1 ? ", 2, or " : " or ", option_count);
            if (option_count == 3) snprintf(hint, sizeof(hint), "Press 1, 2, or 3");
            else if (option_count == 2) snprintf(hint, sizeof(hint), "Press 1 or 2");
            Vector2 hw = MeasureTextEx(font_label, hint, fs_hint, 0.5f);
            DrawTextEx(font_label, hint,
                       (Vector2){cx - hw.x / 2.0f, by + box_h - pad * 0.5f - fs_hint}, fs_hint, 0.5f, theme->prompt_hint);
        EndDrawing();
    }

    if (WindowShouldClose() && choice == 0) return 0;
    return choice;
}
