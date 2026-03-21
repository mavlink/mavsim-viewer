#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "raylib.h"
#include "raymath.h"
#include "data_source.h"
#include "ulog_replay.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"
#include "debug_panel.h"
#include "ortho_panel.h"
#include "asset_path.h"

#define MAX_VEHICLES 16
#define EARTH_RADIUS 6371000.0

// Incremental Pearson correlation (position only: x, y, z)
#define CORR_CHANNELS 3
#define CORR_MIN_SAMPLES 30

typedef struct {
    double sum_x, sum_y, sum_xy, sum_x2, sum_y2;
} corr_channel_t;

typedef struct {
    corr_channel_t ch[CORR_CHANNELS];
    double sum_sq_dist;  // running sum of squared Euclidean distances
    int n;
} corr_state_t;

static float corr_compute(const corr_state_t *cs) {
    if (cs->n < CORR_MIN_SAMPLES) return NAN;
    float r_sum = 0.0f;
    int valid = 0;
    double n = cs->n;
    for (int c = 0; c < CORR_CHANNELS; c++) {
        const corr_channel_t *ch = &cs->ch[c];
        double num = n * ch->sum_xy - ch->sum_x * ch->sum_y;
        double d1  = n * ch->sum_x2 - ch->sum_x * ch->sum_x;
        double d2  = n * ch->sum_y2 - ch->sum_y * ch->sum_y;
        double den = sqrt(d1 * d2);
        if (den > 1e-9) {
            r_sum += (float)(num / den);
            valid++;
        }
    }
    return (valid > 0) ? r_sum / valid : 0.0f;
}

// Per-view-mode vehicle palettes. Slots 1-6 match trail directional colors.
// Alternating warm/cool so adjacent drones are visually distinct.
static const Color vehicle_palette_grid[MAX_VEHICLES] = {
    {230, 230, 230, 255}, // 0: white (primary)
    { 40, 120, 255, 255}, // 1: blue (cool)
    {255,  40,  80, 255}, // 2: red (warm)
    {255, 200,  50, 255}, // 3: yellow (warm)
    { 40, 255,  80, 255}, // 4: green (cool)
    {255, 140,   0, 255}, // 5: orange (warm)
    {160,  60, 255, 255}, // 6: purple (cool)
    {255, 100, 180, 255}, // 7: pink (warm)
    {  0, 180, 140, 255}, // 8: teal (cool)
    {200, 180,  80, 255}, // 9: gold (warm)
    {100, 100, 255, 255}, // 10: indigo (cool)
    {255, 180, 100, 255}, // 11: peach (warm)
    {100, 220, 200, 255}, // 12: mint (cool)
    {220,  80, 180, 255}, // 13: magenta (warm)
    {120, 200, 255, 255}, // 14: sky blue (cool)
    {180, 255,  60, 255}, // 15: lime (cool)
};
static const Color vehicle_palette_rez[MAX_VEHICLES] = {
    {200, 208, 218, 255}, { 30, 100, 240, 255}, {255,  40,  80, 255},
    {220, 180,  30, 255}, { 40, 255, 100, 255}, {255, 160,   0, 255},
    {160,  40, 240, 255}, {255,  80, 160, 255}, {  0, 160, 130, 255},
    {180, 160,  60, 255}, { 80,  90, 240, 255}, {240, 170,  90, 255},
    { 80, 200, 180, 255}, {200,  60, 160, 255}, {100, 180, 240, 255},
    {160, 240,  50, 255},
};
static const Color vehicle_palette_snow[MAX_VEHICLES] = {
    { 40,  40,  50, 255}, { 20,  80, 160, 255}, {200,  20,  60, 255},
    {200, 140,  20, 255}, { 20, 160,  40, 255}, {160,  20,  80, 255},
    {140,  20, 200, 255}, {180,  40, 100, 255}, {  0, 120, 100, 255},
    {180, 120,  40, 255}, { 60,  60, 180, 255}, {140, 120,  40, 255},
    { 40, 140, 120, 255}, {160,  40, 120, 255}, { 60, 130, 180, 255},
    {120, 160,  20, 255},
};
static const Color vehicle_palette_1988[MAX_VEHICLES] = {
    {255, 255, 255, 255}, { 60, 100, 255, 255}, {255,  40,  80, 255},
    {255, 220,  60, 255}, { 40, 255,  80, 255}, {255, 140,   0, 255},
    {180,  40, 255, 255}, {255,  20, 100, 255}, {  0, 200, 160, 255},
    {255, 180,  60, 255}, {120,  60, 255, 255}, {255, 255, 100, 255},
    { 60, 255, 200, 255}, {255,  60, 200, 255}, { 60, 200, 255, 255},
    {200, 255,  40, 255},
};

static const Color *get_vehicle_palette(view_mode_t vm) {
    switch (vm) {
        case VIEW_SNOW: return vehicle_palette_snow;
        case VIEW_REZ:  return vehicle_palette_rez;
        case VIEW_1988: return vehicle_palette_1988;
        default:        return vehicle_palette_grid;
    }
}

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410)\n");
    printf("  -n <count>     Number of vehicles (default: 1, max: %d)\n", MAX_VEHICLES);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  --replay <file1.ulg> [file2.ulg ...]  Replay ULog file(s)\n");
    printf("  --ghost <file1.ulg> [file2.ulg ...]   Ghost mode replay\n");
    printf("  -w <width>     Window width (default: 1280)\n");
    printf("  -h <height>    Window height (default: 720)\n");
}

int main(int argc, char *argv[]) {
    uint16_t base_port = 19410;
    int vehicle_count = 1;
    int model_idx = MODEL_QUADROTOR;
    int win_w = 1280;
    int win_h = 720;
    bool debug = false;
    // PX4 SIH default spawn position
    double origin_lat = 47.397742;
    double origin_lon = 8.545594;
    double origin_alt = 489.4;
    bool origin_specified = false;
    char *replay_paths[MAX_VEHICLES] = {0};
    int num_replay_files = 0;
    bool ghost_mode = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-udp") == 0 && i + 1 < argc) {
            base_port = (uint16_t)atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            vehicle_count = atoi(argv[++i]);
            if (vehicle_count < 1) vehicle_count = 1;
            if (vehicle_count > MAX_VEHICLES) vehicle_count = MAX_VEHICLES;
        } else if (strcmp(argv[i], "-origin") == 0 && i + 3 < argc) {
            origin_lat = atof(argv[++i]);
            origin_lon = atof(argv[++i]);
            origin_alt = atof(argv[++i]);
            origin_specified = true;
        } else if (strcmp(argv[i], "-mc") == 0) {
            model_idx = MODEL_QUADROTOR;
        } else if (strcmp(argv[i], "-fw") == 0) {
            model_idx = MODEL_FIXEDWING;
        } else if (strcmp(argv[i], "-ts") == 0) {
            model_idx = MODEL_TAILSITTER;
        } else if (strcmp(argv[i], "-d") == 0) {
            debug = true;
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            win_w = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
            win_h = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--replay") == 0) {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= MAX_VEHICLES) {
                    fprintf(stderr, "Too many replay files (max %d)\n", MAX_VEHICLES);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
        } else if (strcmp(argv[i], "--ghost") == 0) {
            ghost_mode = true;
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                if (num_replay_files >= MAX_VEHICLES) {
                    fprintf(stderr, "Too many replay files (max %d)\n", MAX_VEHICLES);
                    return 1;
                }
                replay_paths[num_replay_files++] = argv[++i];
            }
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    asset_path_init();

    // Init Raylib
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(win_w, win_h, "MAVSim Viewer");
    SetTargetFPS(60);

    // Init data sources
    data_source_t sources[MAX_VEHICLES];
    memset(sources, 0, sizeof(sources));
    bool is_replay = (num_replay_files > 0);

    if (is_replay) {
        for (int i = 0; i < num_replay_files; i++) {
            if (data_source_ulog_create(&sources[i], replay_paths[i]) != 0) {
                fprintf(stderr, "Failed to open ULog: %s\n", replay_paths[i]);
                CloseWindow();
                return 1;
            }
        }
        vehicle_count = num_replay_files;
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            if (data_source_mavlink_create(&sources[i], base_port + i, (uint8_t)i, debug) != 0) {
                fprintf(stderr, "Failed to init MAVLink receiver on port %u\n", base_port + i);
                CloseWindow();
                return 1;
            }
        }
    }

    // Init vehicles
    // Init scene first (provides lighting shader for vehicles)
    scene_t scene;
    scene_init(&scene);

    vehicle_t vehicles[MAX_VEHICLES];
    corr_state_t corr[MAX_VEHICLES];
    memset(corr, 0, sizeof(corr));
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
        vehicles[i].color = get_vehicle_palette(scene.view_mode)[i];
    }

    // For multi-vehicle MAVLink or explicit origin: pre-set the NED origin
    if (!is_replay && (vehicle_count > 1 || origin_specified)) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
        printf("NED origin: lat=%.6f lon=%.6f alt=%.1f\n", origin_lat, origin_lon, origin_alt);
    } else if (origin_specified) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
    }

    // ── Takeoff alignment state (toggled by A key) ──
    bool takeoff_aligned = false;
    if (is_replay && num_replay_files > 1) {
        // Set multi-file CONF for each source (always available)
        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            sources[i].playback.takeoff_conf = r->takeoff_conf;
            sources[i].playback.time_offset_s = 0.0f;
        }
    }

    // ── Conflict detection + resolution (multi-file replay, not --ghost) ──
    bool conflict_detected = false;
    bool conflict_far = false;
    bool ghost_mode_grid = false;

    if (is_replay && num_replay_files > 1 && !ghost_mode) {
        // Tier assignment
        int tier3_count = 0;
        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            if (r->home_from_topic) {
                // Tier 1
            } else if (r->home.valid) {
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
            for (int i = 0; i < num_replay_files && !conflict; i++) {
                if (!sources[i].home.valid) continue;
                for (int j = i + 1; j < num_replay_files && !conflict; j++) {
                    if (!sources[j].home.valid) continue;
                    double dlat_m = ((double)sources[i].home.lat - (double)sources[j].home.lat) / 1e7 * 111319.5;
                    double lat_rad = (sources[i].home.lat / 1e7) * (M_PI / 180.0);
                    double dlon_m = ((double)sources[i].home.lon - (double)sources[j].home.lon) / 1e7 * 111319.5 * cos(lat_rad);
                    double dalt_m = ((double)sources[i].home.alt - (double)sources[j].home.alt) / 1000.0;
                    double dist = sqrt(dlat_m * dlat_m + dlon_m * dlon_m + dalt_m * dalt_m);
                    if (dist < 0.1) conflict = true;              // identical position
                    if (dist > 500.0) { conflict = true; conflict_far = true; }  // too far
                }
            }
        }

        // Show deconfliction prompt
        conflict_detected = conflict;
        if (conflict) {
            // Init HUD early for fonts
            hud_t prompt_hud;
            hud_init(&prompt_hud);

            int choice = 0;
            while (choice == 0 && !WindowShouldClose()) {
                int sw = GetScreenWidth();
                int sh = GetScreenHeight();
                float s = powf(sh / 720.0f, 0.7f);
                if (s < 1.0f) s = 1.0f;

                int key = GetKeyPressed();
                if (key == KEY_ONE) choice = 1;
                else if (key == KEY_TWO) choice = 2;
                else if (key == KEY_THREE) choice = 3;

                // View-mode colors
                Color scrim_col, box_bg, box_border, subtitle_col, hint_col, key_col, text_col, title_col;
                if (scene.view_mode == VIEW_SNOW) {
                    scrim_col=(Color){255,255,255,140}; box_bg=(Color){248,248,250,240};
                    box_border=(Color){15,15,20,120}; subtitle_col=(Color){60,65,75,255};
                    hint_col=(Color){120,125,135,200}; key_col=(Color){15,15,20,220};
                    text_col=(Color){10,10,15,255}; title_col=(Color){200,140,0,255};
                } else if (scene.view_mode == VIEW_1988) {
                    scrim_col=(Color){5,0,15,160}; box_bg=(Color){5,5,16,240};
                    box_border=(Color){255,20,100,140}; subtitle_col=(Color){255,20,100,200};
                    hint_col=(Color){180,60,120,160}; key_col=(Color){255,220,60,255};
                    text_col=(Color){255,220,60,255}; title_col=(Color){255,20,100,255};
                } else if (scene.view_mode == VIEW_REZ) {
                    scrim_col=(Color){0,0,0,150}; box_bg=(Color){8,8,12,235};
                    box_border=(Color){0,204,218,100}; subtitle_col=(Color){0,140,150,160};
                    hint_col=(Color){0,140,150,160}; key_col=(Color){0,204,218,220};
                    text_col=(Color){200,208,218,255}; title_col=YELLOW;
                } else {
                    scrim_col=(Color){0,0,0,140}; box_bg=(Color){10,14,20,235};
                    box_border=(Color){0,180,204,100}; subtitle_col=(Color){140,150,170,255};
                    hint_col=(Color){90,95,110,200}; key_col=(Color){0,255,255,220};
                    text_col=WHITE; title_col=YELLOW;
                }

                float fs_title=15*s, fs_option=20*s, fs_hint=12*s;
                float line_h=36*s, pad=16*s, inner_gap=20*s;

                const char *title_main = "POSITION CONFLICT";
                const char *grid_label = conflict_far ? "Narrow grid offset" : "Grid offset";
                char title_rest[64];
                if (conflict_far)
                    snprintf(title_rest, sizeof(title_rest), "  -  %d drones too far apart", num_replay_files);
                else
                    snprintf(title_rest, sizeof(title_rest), "  -  %d drones overlap", num_replay_files);

                const char *labels[] = {"Cancel & reupload", "Ghost mode", grid_label};
                float key_num_w = MeasureTextEx(prompt_hud.font_value, "1", fs_option, 0.5f).x;
                float gap = 16*s;
                float widest = 0;
                for (int o = 0; o < 3; o++) {
                    float w = key_num_w + gap + MeasureTextEx(prompt_hud.font_label, labels[o], fs_option, 0.5f).x;
                    if (w > widest) widest = w;
                }

                Vector2 tmw = MeasureTextEx(prompt_hud.font_label, title_main, fs_title, 0.5f);
                Vector2 trw = MeasureTextEx(prompt_hud.font_label, title_rest, fs_title, 0.5f);
                float title_total = tmw.x + trw.x;

                float box_w = 520*s;
                float opts_block_h = line_h * 3;
                float box_h = pad + fs_title + inner_gap + opts_block_h + inner_gap*0.3f + fs_hint + pad*0.5f;
                float cx = sw/2.0f, cy = sh/2.0f;
                float bx = cx - box_w/2.0f, by = cy - box_h/2.0f;

                BeginDrawing();
                    scene_draw_sky(&scene);
                    BeginMode3D(scene.camera);
                        scene_draw(&scene);
                    EndMode3D();
                    DrawRectangle(0, 0, sw, sh, scrim_col);

                    Rectangle box = {bx, by, box_w, box_h};
                    DrawRectangleRounded(box, 0.06f, 8, box_bg);
                    DrawRectangleRoundedLinesEx(box, 0.06f, 8, 1.5f*s, box_border);

                    float title_x = cx - title_total/2.0f;
                    DrawTextEx(prompt_hud.font_label, title_main,
                               (Vector2){title_x, by + pad}, fs_title, 0.5f, title_col);
                    DrawTextEx(prompt_hud.font_label, title_rest,
                               (Vector2){title_x + tmw.x, by + pad}, fs_title, 0.5f, subtitle_col);

                    float left = cx - widest/2.0f;
                    float opts_y = by + pad + fs_title + inner_gap;
                    for (int o = 0; o < 3; o++) {
                        char num[2] = {(char)('1'+o), '\0'};
                        DrawTextEx(prompt_hud.font_value, num,
                                   (Vector2){left, opts_y + o*line_h}, fs_option, 0.5f, key_col);
                        DrawTextEx(prompt_hud.font_label, labels[o],
                                   (Vector2){left + key_num_w + gap, opts_y + o*line_h}, fs_option, 0.5f, text_col);
                    }

                    const char *hint = "Press 1, 2, or 3";
                    Vector2 hw = MeasureTextEx(prompt_hud.font_label, hint, fs_hint, 0.5f);
                    DrawTextEx(prompt_hud.font_label, hint,
                               (Vector2){cx - hw.x/2.0f, by + box_h - pad*0.5f - fs_hint}, fs_hint, 0.5f, hint_col);
                EndDrawing();
            }

            hud_cleanup(&prompt_hud);

            if (choice == 1 || WindowShouldClose()) {
                for (int i = 0; i < num_replay_files; i++)
                    data_source_close(&sources[i]);
                scene_cleanup(&scene);
                CloseWindow();
                return 0;
            } else if (choice == 2) {
                ghost_mode = true;
            } else if (choice == 3) {
                ghost_mode_grid = true;
            }
        }
    }

    // Compute shared NED origin for multi-drone replay.
    // ref_lat_rad/ref_lon_rad/min_alt persist for runtime mode switching (P key).
    double ref_lat_rad = 0.0, ref_lon_rad = 0.0, min_alt = 0.0;
    int ref_idx = -1;
    if (is_replay && num_replay_files > 1) {
        for (int i = 0; i < num_replay_files; i++) {
            if (sources[i].home.valid) {
                ref_idx = i;
                ref_lat_rad = sources[i].home.lat / 1e7 * (M_PI / 180.0);
                ref_lon_rad = sources[i].home.lon / 1e7 * (M_PI / 180.0);
                break;
            }
        }

        min_alt = 1e9;
        for (int i = 0; i < num_replay_files; i++) {
            if (sources[i].home.valid) {
                double a = sources[i].home.alt * 1e-3;
                if (a < min_alt) min_alt = a;
            }
        }
        if (min_alt > 1e8) min_alt = 0.0;

        if (ghost_mode || ghost_mode_grid) {
            // Ghost/grid: each drone uses its own home as origin (collapse to center).
            // Grid mode additionally offsets each drone along X.
            // Don't set origin_set — vehicle_update will set each drone's own origin.
            if (ghost_mode_grid) {
                for (int i = 1; i < num_replay_files; i++)
                    vehicles[i].grid_offset.x = i * 5.0f;
            }
        } else {
            // Normal replay: shared origin so drones render at real relative positions
            for (int i = 0; i < num_replay_files; i++) {
                if (sources[i].home.valid) {
                    vehicles[i].lat0 = ref_lat_rad;
                    vehicles[i].lon0 = ref_lon_rad;
                    vehicles[i].alt0 = min_alt;
                    vehicles[i].origin_set = true;
                }
            }
        }

        if (ref_idx >= 0)
            printf("Multi-drone origin: lat=%.6f lon=%.6f alt=%.1f (min datum)\n",
                   ref_lat_rad * (180.0 / M_PI), ref_lon_rad * (180.0 / M_PI), min_alt);
    }

    // Compute position tier per vehicle (for debug panel)
    int vehicle_tier[MAX_VEHICLES];
    memset(vehicle_tier, 0, sizeof(vehicle_tier));
    if (is_replay) {
        for (int i = 0; i < num_replay_files; i++) {
            ulog_replay_ctx_t *r = (ulog_replay_ctx_t *)sources[i].impl;
            if (r->home_from_topic) vehicle_tier[i] = 1;
            else if (r->home.valid) vehicle_tier[i] = 2;
            else vehicle_tier[i] = 3;
        }
    }

    // Check for Tier 3 drones (no valid home = estimated position)
    bool has_tier3 = false;
    for (int i = 0; i < num_replay_files; i++) {
        if (!sources[i].home.valid) { has_tier3 = true; break; }
    }

    // Apply ghost mode: translucent non-primary drones
    if (ghost_mode && num_replay_files > 1) {
        vehicle_set_ghost_alpha(&vehicles[0], 1.0f);
        for (int i = 1; i < num_replay_files; i++)
            vehicle_set_ghost_alpha(&vehicles[i], 0.35f);
    }

    hud_t hud;
    hud_init(&hud);
    hud.is_replay = is_replay;

    debug_panel_t dbg_panel;
    debug_panel_init(&dbg_panel);

    ortho_panel_t ortho;
    ortho_panel_init(&ortho);

    int selected = 0;
    int prev_selected = 0;
    bool was_connected[MAX_VEHICLES];
    memset(was_connected, 0, sizeof(was_connected));
    Vector3 last_pos[MAX_VEHICLES];
    memset(last_pos, 0, sizeof(last_pos));
    bool show_hud = true;
    int trail_mode = 1;              // 0=off, 1=directional trail, 2=speed ribbon
    bool show_ground_track = false;  // ground projection off by default
    bool classic_colors = false;     // L key: toggle classic (red/blue) vs modern (yellow/purple)
    int corr_mode = 0;               // Shift+T: 0=off, 1=ribbon, 2=line
    bool show_corr_labels = true;    // Ctrl+L: distance labels in ortho correlation
    bool show_axes = false;          // Z: axis orientation gizmo
    bool insufficient_data[MAX_VEHICLES];  // drones with no position data
    memset(insufficient_data, 0, sizeof(insufficient_data));
    int insufficient_check_frames = 0;
    bool insufficient_toasted = false;

    // Main loop
    while (!WindowShouldClose()) {
        // Poll all data sources and update vehicles
        for (int i = 0; i < vehicle_count; i++) {
            data_source_poll(&sources[i], GetFrameTime());

            // Reset trail and origin on reconnect
            if (sources[i].connected && !was_connected[i]) {
                vehicle_reset_trail(&vehicles[i]);
                if (sources[i].mav_type != 0)
                    vehicle_set_type(&vehicles[i], sources[i].mav_type);
                if (!origin_specified && vehicle_count == 1) {
                    vehicles[i].origin_set = false;
                    vehicles[i].origin_wait_count = 0;
                }
            }
            was_connected[i] = sources[i].connected;

            vehicle_update(&vehicles[i], &sources[i].state, &sources[i].home);
            vehicles[i].sysid = sources[i].sysid;

            // Placeholder: show model at origin while waiting for position data.
            if (sources[i].connected && !vehicles[i].active) {
                vehicles[i].active = true;
                vehicles[i].position = vehicles[i].grid_offset;
            }

            // Detect position jump (new SITL connecting before disconnect timeout)
            if (vehicles[i].active && vehicles[i].trail_count > 0) {
                Vector3 delta = Vector3Subtract(vehicles[i].position, last_pos[i]);
                if (Vector3Length(delta) > 50.0f) {
                    vehicle_reset_trail(&vehicles[i]);
                }
            }
            last_pos[i] = vehicles[i].position;
        }

        // Detect drones with insufficient position data (~2s after start)
        if (is_replay && !insufficient_toasted) {
            insufficient_check_frames++;
            if (insufficient_check_frames > 120) {
                for (int i = 0; i < vehicle_count; i++) {
                    if (sources[i].connected && !sources[i].state.valid) {
                        insufficient_data[i] = true;
                        char msg[64];
                        snprintf(msg, sizeof(msg), "DRONE %d: INSUFFICIENT DATA", i + 1);
                        hud_toast_color(&hud, msg, 4.0f, vehicles[i].color);
                    }
                }
                insufficient_toasted = true;
            }
        }

        // Grid offsets for ghost/narrow-grid are now computed at init time
        // from home positions (see shared origin block above).

        // Positional correlation vs selected drone (only while playing)
        if (is_replay && num_replay_files > 1) {
            // Reset accumulators when selection changes
            if (selected != prev_selected) {
                memset(corr, 0, sizeof(corr));
                for (int i = 0; i < num_replay_files; i++) {
                    sources[i].playback.correlation = NAN;
                    sources[i].playback.rmse = NAN;
                }
                prev_selected = selected;
            }

            // Only accumulate while playback is active (not paused, not ended)
            bool playing = sources[selected].connected && !sources[selected].playback.paused;
            if (playing && vehicles[selected].origin_set) {
                const vehicle_t *ref = &vehicles[selected];
                double rx[CORR_CHANNELS] = {
                    ref->position.z, ref->position.x, ref->position.y
                };
                for (int i = 0; i < num_replay_files; i++) {
                    if (i == selected || !vehicles[i].origin_set) continue;
                    const vehicle_t *v = &vehicles[i];
                    double vx[CORR_CHANNELS] = {
                        v->position.z, v->position.x, v->position.y
                    };
                    for (int c = 0; c < CORR_CHANNELS; c++) {
                        corr[i].ch[c].sum_x  += rx[c];
                        corr[i].ch[c].sum_y  += vx[c];
                        corr[i].ch[c].sum_xy += rx[c] * vx[c];
                        corr[i].ch[c].sum_x2 += rx[c] * rx[c];
                        corr[i].ch[c].sum_y2 += vx[c] * vx[c];
                    }
                    // Euclidean distance squared for RMSE (subtract grid offsets)
                    double dx = (v->position.x - v->grid_offset.x) - (ref->position.x - ref->grid_offset.x);
                    double dy = (v->position.y - v->grid_offset.y) - (ref->position.y - ref->grid_offset.y);
                    double dz = (v->position.z - v->grid_offset.z) - (ref->position.z - ref->grid_offset.z);
                    corr[i].sum_sq_dist += dx*dx + dy*dy + dz*dz;
                    corr[i].n++;
                    sources[i].playback.correlation = corr_compute(&corr[i]);
                    sources[i].playback.rmse = (corr[i].n >= CORR_MIN_SAMPLES)
                        ? (float)sqrt(corr[i].sum_sq_dist / corr[i].n) : NAN;
                }
                sources[selected].playback.correlation = 1.0f;
                sources[selected].playback.rmse = 0.0f;
            }
        }

        // Check if any source is connected (for HUD)
        bool any_connected = false;
        for (int i = 0; i < vehicle_count; i++) {
            if (sources[i].connected) { any_connected = true; break; }
        }

        // Update HUD sim time from selected vehicle
        hud_update(&hud, sources[selected].state.time_usec,
                   sources[selected].connected, GetFrameTime());

        // Handle input
        scene_handle_input(&scene);

        // P key: switch between Swarm / Ghost / Grid modes during multi-file replay
        // P key: mode switcher for multi-file replay
        // - No conflict: Formation / Ghost / Grid
        // - Conflict (close/no-data): Cancel / Ghost / Grid offset
        // - Conflict (too far): Cancel / Ghost / Narrow grid
        if (IsKeyPressed(KEY_P) && is_replay && num_replay_files > 1) {
            int ch = 0;
            while (ch == 0 && !WindowShouldClose()) {
                int psw = GetScreenWidth(), psh = GetScreenHeight();
                float ps = powf(psh / 720.0f, 0.7f);
                if (ps < 1.0f) ps = 1.0f;
                int pk = GetKeyPressed();
                if (pk == KEY_ONE) ch = 1;
                else if (pk == KEY_TWO) ch = 2;
                else if (pk == KEY_THREE) ch = 3;

                Color pscrim, pbbg, pbbd, psub, phint_c, pkc, ptc, pttl;
                if (scene.view_mode == VIEW_SNOW) {
                    pscrim=(Color){255,255,255,140}; pbbg=(Color){248,248,250,240};
                    pbbd=(Color){15,15,20,120}; psub=(Color){60,65,75,255};
                    phint_c=(Color){120,125,135,200}; pkc=(Color){15,15,20,220};
                    ptc=(Color){10,10,15,255}; pttl=(Color){200,140,0,255};
                } else if (scene.view_mode == VIEW_1988) {
                    pscrim=(Color){5,0,15,160}; pbbg=(Color){5,5,16,240};
                    pbbd=(Color){255,20,100,140}; psub=(Color){255,20,100,200};
                    phint_c=(Color){180,60,120,160}; pkc=(Color){255,220,60,255};
                    ptc=(Color){255,220,60,255}; pttl=(Color){255,20,100,255};
                } else if (scene.view_mode == VIEW_REZ) {
                    pscrim=(Color){0,0,0,150}; pbbg=(Color){8,8,12,235};
                    pbbd=(Color){0,204,218,100}; psub=(Color){0,140,150,160};
                    phint_c=(Color){0,140,150,160}; pkc=(Color){0,204,218,220};
                    ptc=(Color){200,208,218,255}; pttl=YELLOW;
                } else {
                    pscrim=(Color){0,0,0,140}; pbbg=(Color){10,14,20,235};
                    pbbd=(Color){0,180,204,100}; psub=(Color){140,150,170,255};
                    phint_c=(Color){90,95,110,200}; pkc=(Color){0,255,255,220};
                    ptc=WHITE; pttl=YELLOW;
                }

                // Menu labels and title depend on conflict state
                const char *title;
                const char *pl[3];
                const char *grid_label = conflict_far ? "Narrow grid" : "Grid offset";
                if (conflict_detected) {
                    title = "POSITION CONFLICT";
                    pl[0] = "Cancel"; pl[1] = "Ghost mode"; pl[2] = grid_label;
                } else {
                    title = "REPLAY MODE";
                    pl[0] = "Formation"; pl[1] = "Ghost"; pl[2] = "Grid offset";
                }

                float pfs_t=15*ps, pfs_o=20*ps, pfs_h=12*ps, plh=36*ps, pp=16*ps, pig=20*ps;
                float pkw=MeasureTextEx(hud.font_value,"1",pfs_o,0.5f).x, pg=16*ps, pw=0;
                for(int o=0;o<3;o++){float w=pkw+pg+MeasureTextEx(hud.font_label,pl[o],pfs_o,0.5f).x;if(w>pw)pw=w;}
                float pbw=520*ps, pbh=pp+pfs_t+pig+plh*3+pig*0.3f+pfs_h+pp*0.5f;
                float pcx=psw/2.0f, pcy=psh/2.0f, pbx=pcx-pbw/2, pby=pcy-pbh/2;

                BeginDrawing();
                scene_draw_sky(&scene);
                BeginMode3D(scene.camera); scene_draw(&scene); EndMode3D();
                DrawRectangle(0,0,psw,psh,pscrim);
                Rectangle pb={pbx,pby,pbw,pbh};
                DrawRectangleRounded(pb,0.06f,8,pbbg);
                DrawRectangleRoundedLinesEx(pb,0.06f,8,1.5f*ps,pbbd);
                char pt2[64];
                if (conflict_detected) {
                    if (conflict_far)
                        snprintf(pt2,sizeof(pt2),"  -  %d drones too far apart",num_replay_files);
                    else
                        snprintf(pt2,sizeof(pt2),"  -  %d drones overlap",num_replay_files);
                } else {
                    snprintf(pt2,sizeof(pt2),"  -  %d drones",num_replay_files);
                }
                Vector2 ptm=MeasureTextEx(hud.font_label,title,pfs_t,0.5f);
                Vector2 ptr=MeasureTextEx(hud.font_label,pt2,pfs_t,0.5f);
                float ptx=pcx-(ptm.x+ptr.x)/2;
                DrawTextEx(hud.font_label,title,(Vector2){ptx,pby+pp},pfs_t,0.5f,pttl);
                DrawTextEx(hud.font_label,pt2,(Vector2){ptx+ptm.x,pby+pp},pfs_t,0.5f,psub);
                float pleft=pcx-pw/2, poy=pby+pp+pfs_t+pig;
                for(int o=0;o<3;o++){
                    char n[2]={(char)('1'+o),'\0'};
                    DrawTextEx(hud.font_value,n,(Vector2){pleft,poy+o*plh},pfs_o,0.5f,pkc);
                    DrawTextEx(hud.font_label,pl[o],(Vector2){pleft+pkw+pg,poy+o*plh},pfs_o,0.5f,ptc);
                }
                const char*pht="Press 1, 2, or 3";
                Vector2 phw=MeasureTextEx(hud.font_label,pht,pfs_h,0.5f);
                DrawTextEx(hud.font_label,pht,(Vector2){pcx-phw.x/2,pby+pbh-pp*0.5f-pfs_h},pfs_h,0.5f,phint_c);
                EndDrawing();
            }
            if (ch == 1) {
                if (conflict_detected) {
                    // Cancel — reset each drone to own origin
                    ghost_mode = false; ghost_mode_grid = false;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicles[i].grid_offset = (Vector3){0,0,0};
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                            vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                            vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                    }
                } else {
                    // Formation mode — shared origin, real relative positions
                    ghost_mode = false; ghost_mode_grid = false;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicles[i].grid_offset = (Vector3){0,0,0};
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = ref_lat_rad;
                            vehicles[i].lon0 = ref_lon_rad;
                            vehicles[i].alt0 = min_alt;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                    }
                }
            } else if (ch == 2) {
                // Ghost mode — each drone uses own home, collapse to center
                ghost_mode = true; ghost_mode_grid = false;
                for (int i = 0; i < num_replay_files; i++) {
                    vehicles[i].grid_offset = (Vector3){0,0,0};
                    if (sources[i].home.valid) {
                        vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                        vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                        vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                        vehicles[i].origin_set = true;
                    }
                    vehicles[i].origin_wait_count = 0;
                    vehicle_reset_trail(&vehicles[i]);
                }
                vehicle_set_ghost_alpha(&vehicles[0], 1.0f);
                for (int i = 1; i < num_replay_files; i++)
                    vehicle_set_ghost_alpha(&vehicles[i], 0.35f);
            } else if (ch == 3) {
                if (conflict_far) {
                    // Narrow grid — each drone keeps own origin, narrow spacing
                    ghost_mode = false; ghost_mode_grid = true;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                            vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                            vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                        vehicles[i].grid_offset = (i > 0) ? (Vector3){ i * 5.0f, 0.0f, 0.0f } : (Vector3){0,0,0};
                    }
                } else {
                    // Grid offset — each drone uses own home + X offset
                    ghost_mode = false; ghost_mode_grid = true;
                    for (int i = 0; i < num_replay_files; i++) {
                        vehicle_set_ghost_alpha(&vehicles[i], 1.0f);
                        if (sources[i].home.valid) {
                            vehicles[i].lat0 = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                            vehicles[i].lon0 = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                            vehicles[i].alt0 = sources[i].home.alt * 1e-3;
                            vehicles[i].origin_set = true;
                        }
                        vehicles[i].origin_wait_count = 0;
                        vehicle_reset_trail(&vehicles[i]);
                        vehicles[i].grid_offset = (i > 0) ? (Vector3){ i * 5.0f, 0.0f, 0.0f } : (Vector3){0,0,0};
                    }
                }
            }
            // Reset stats on mode switch
            memset(corr, 0, sizeof(corr));
            for (int i = 0; i < num_replay_files; i++) {
                sources[i].playback.correlation = NAN;
                sources[i].playback.rmse = NAN;
            }
        }

        // Update vehicle colors when view mode changes
        {
            const Color *pal = get_vehicle_palette(scene.view_mode);
            for (int i = 0; i < vehicle_count; i++)
                vehicles[i].color = pal[i];
        }

        // Help overlay toggle (? key = Shift+/)
        if (IsKeyPressed(KEY_SLASH) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))) {
            hud.show_help = !hud.show_help;
        }

        // Toggle HUD visibility
        if (IsKeyPressed(KEY_H)) {
            show_hud = !show_hud;
        }

        // Shift+T: cycle correlation overlay (off → ribbon → line → off)
        if (IsKeyPressed(KEY_T) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
            && is_replay && num_replay_files > 1) {
            corr_mode = (corr_mode + 1) % 3;
            const char *names[] = { "Correlation Off", "Correlation Line", "Correlation Curtain" };
            hud_toast(&hud, names[corr_mode], 2.0f);
        }
        // Cycle trail mode: off → trail → speed ribbon
        else if (IsKeyPressed(KEY_T)) {
            int max_modes = (num_replay_files > 1) ? 4 : 3;
            trail_mode = (trail_mode + 1) % max_modes;
            const char *trail_names[] = { "Trails Off", "Direction Trails", "Speed Ribbons", "ID Trails" };
            hud_toast(&hud, trail_names[trail_mode], 2.0f);
        }

        // Toggle classic/modern arm colors
        if (IsKeyPressed(KEY_K)) {
            classic_colors = !classic_colors;
        }

        // Toggle ground track projection
        if (IsKeyPressed(KEY_G)) {
            show_ground_track = !show_ground_track;
        }

        // Toggle debug panel (Ctrl+D)
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_D)) {
            dbg_panel.visible = !dbg_panel.visible;
        }

        // Toggle correlation distance labels (Ctrl+L)
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_L)) {
            show_corr_labels = !show_corr_labels;
        }

        // Toggle axis gizmo (Z)
        if (IsKeyPressed(KEY_Z) && !IsKeyDown(KEY_LEFT_SHIFT) && !IsKeyDown(KEY_RIGHT_SHIFT)
            && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)) {
            show_axes = !show_axes;
        }

        // Toggle ortho panel
        if (IsKeyPressed(KEY_O)) {
            ortho.visible = !ortho.visible;
        }

        // Cycle model for selected vehicle
        // Cycle model: M = within group, Shift+M = all models
        if (IsKeyPressed(KEY_M)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                int next = (vehicles[selected].model_idx + 1) % vehicle_model_count;
                vehicle_load_model(&vehicles[selected], next);
            } else {
                vehicle_cycle_model(&vehicles[selected]);
            }
        }

        // Vehicle selection input
        if (vehicle_count > 1) {
            if (IsKeyPressed(KEY_TAB)) {
                // Cycle to next connected vehicle, clear pins
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_LEFT_BRACKET)) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int prev = (selected - j + vehicle_count) % vehicle_count;
                    if (sources[prev].connected) { selected = prev; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_RIGHT_BRACKET)) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            // Number keys 1-9: plain = select + clear pins, SHIFT = toggle pin
            for (int k = KEY_ONE; k <= KEY_NINE; k++) {
                if (IsKeyPressed(k)) {
                    int idx = k - KEY_ONE;
                    if (idx >= vehicle_count) continue;

                    if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                        // SHIFT+number: toggle pin
                        if (idx == selected) continue;  // can't pin the primary

                        // Check if already pinned
                        int found = -1;
                        for (int p = 0; p < hud.pinned_count; p++) {
                            if (hud.pinned[p] == idx) { found = p; break; }
                        }

                        if (found >= 0) {
                            // Unpin: shift remaining
                            for (int p = found; p < hud.pinned_count - 1; p++)
                                hud.pinned[p] = hud.pinned[p + 1];
                            hud.pinned_count--;
                            hud.pinned[hud.pinned_count] = -1;
                        } else if (hud.pinned_count < HUD_MAX_PINNED && hud.pinned_count < vehicle_count - 1) {
                            hud.pinned[hud.pinned_count++] = idx;
                        }
                    } else if (!IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)
                               && !IsKeyDown(KEY_LEFT_ALT) && !IsKeyDown(KEY_RIGHT_ALT)) {
                        // Plain number: switch primary, clear pins
                        selected = idx;
                        hud.pinned_count = 0;
                        memset(hud.pinned, -1, sizeof(hud.pinned));
                    }
                    // Note: Ctrl+number is reserved for 1988 easter egg
                }
            }
        }

        // Re-toast insufficient data warning when switching to a bad drone
        {
            static int last_selected_for_insuf = -1;
            if (selected != last_selected_for_insuf && insufficient_data[selected]) {
                char msg[64];
                snprintf(msg, sizeof(msg), "DRONE %d: INSUFFICIENT DATA", selected + 1);
                hud_toast_color(&hud, msg, 3.0f, vehicles[selected].color);
            }
            last_selected_for_insuf = selected;
        }

        // Replay playback controls (apply to all sources)
        if (is_replay) {
            int nrf = num_replay_files > 0 ? num_replay_files : 1;
            if (IsKeyPressed(KEY_SPACE)) {
                if (!sources[selected].connected) {
                    for (int i = 0; i < nrf; i++) {
                        ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[i].impl;
                        ulog_replay_seek(rctx, 0.0f);
                        sources[i].connected = true;
                        sources[i].playback.paused = false;
                        vehicle_reset_trail(&vehicles[i]);
                    }
                    memset(corr, 0, sizeof(corr));
                } else {
                    bool p = !sources[selected].playback.paused;
                    for (int i = 0; i < nrf; i++)
                        sources[i].playback.paused = p;
                }
            }
            if (IsKeyPressed(KEY_L)) {
                bool l = !sources[selected].playback.looping;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.looping = l;
            }
            if (IsKeyPressed(KEY_Y)) {
                hud.show_yaw = !hud.show_yaw;
            }
            if (IsKeyPressed(KEY_I)) {
                bool interp = !sources[selected].playback.interpolation;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.interpolation = interp;
                printf("Interpolation: %s\n", interp ? "ON" : "OFF");
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                float spd = sources[selected].playback.speed;
                if (spd < 0.5f) spd = 0.5f;
                else if (spd < 1.0f) spd = 1.0f;
                else if (spd < 2.0f) spd = 2.0f;
                else if (spd < 4.0f) spd = 4.0f;
                else if (spd < 8.0f) spd = 8.0f;
                else spd = 16.0f;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.speed = spd;
            }
            if (IsKeyPressed(KEY_MINUS)) {
                float spd = sources[selected].playback.speed;
                if (spd > 8.0f) spd = 8.0f;
                else if (spd > 4.0f) spd = 4.0f;
                else if (spd > 2.0f) spd = 2.0f;
                else if (spd > 1.0f) spd = 1.0f;
                else if (spd > 0.5f) spd = 0.5f;
                else spd = 0.25f;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.speed = spd;
            }
            if (IsKeyPressed(KEY_R)) {
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(ctx, 0.0f);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
            }
            // A key: toggle takeoff time alignment
            if (IsKeyPressed(KEY_A) && num_replay_files > 1) {
                takeoff_aligned = !takeoff_aligned;
                const float takeoff_buffer = 5.0f;
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    if (takeoff_aligned) {
                        float skip = ctx->takeoff_detected ? ctx->takeoff_time_s - takeoff_buffer : 0.0f;
                        if (skip < 0.0f) skip = 0.0f;
                        ctx->time_offset_s = (double)skip;
                    } else {
                        ctx->time_offset_s = 0.0;
                    }
                    sources[i].playback.time_offset_s = (float)ctx->time_offset_s;
                    ulog_replay_seek(ctx, 0.0f);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
                for (int i = 0; i < nrf; i++) {
                    sources[i].playback.correlation = NAN;
                    sources[i].playback.rmse = NAN;
                }
                hud_toast(&hud, takeoff_aligned ? "Auto Align On" : "Auto Align Off", 2.0f);
            }
            if (IsKeyPressed(KEY_RIGHT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float seek_target = sources[selected].playback.position_s + step;
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(ctx, seek_target);
                    sources[i].playback.position_s = (float)ctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
            }
            if (IsKeyPressed(KEY_LEFT)) {
                float step = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 30.0f : 5.0f;
                float target = sources[selected].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(ctx, target);
                    sources[i].playback.position_s = (float)ctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
            }
        }

        // Update debug panel
        debug_panel_update(&dbg_panel, GetFrameTime());

        // Update camera to follow selected vehicle
        scene_update_camera(&scene, vehicles[selected].position, vehicles[selected].rotation);

        // Render ortho views to textures (before main BeginDrawing)
        if (ortho.visible) {
            ortho_panel_update(&ortho, vehicles[selected].position);
            ortho_panel_render(&ortho, vehicles, vehicle_count,
                               selected, scene.view_mode,
                               corr_mode, hud.pinned, hud.pinned_count);
        }

        // Render
        BeginDrawing();

            // Sky background
            scene_draw_sky(&scene);

            // Fullscreen ortho: suppress 3D trails, conditionally suppress 3D correlation line
            bool fs_ortho = (scene.ortho_mode != ORTHO_NONE);
            int tm_3d = fs_ortho ? 0 : trail_mode;

            BeginMode3D(scene.camera);
                scene_draw(&scene);
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        vehicle_draw(&vehicles[i], scene.view_mode, i == selected,
                                     tm_3d, show_ground_track, scene.camera.position,
                                     classic_colors);
                    }
                }

                // Home position markers (formation mode only)
                if (num_replay_files > 1 && !ghost_mode && !ghost_mode_grid) {
                    for (int i = 0; i < vehicle_count; i++) {
                        if (!sources[i].home.valid) continue;
                        double lat = sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                        double lon = sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                        double alt = sources[i].home.alt * 1e-3;
                        float hx = (float)(EARTH_RADIUS * (lon - ref_lon_rad) * cos(ref_lat_rad)) + vehicles[i].grid_offset.x;
                        float hy = (float)(alt - min_alt) + 0.02f;
                        float hz = (float)(-(EARTH_RADIUS * (lat - ref_lat_rad))) + vehicles[i].grid_offset.z;
                        float half = 0.333f;
                        Color fill = vehicles[i].color;
                        fill.a = 70;
                        Color border = vehicles[i].color;
                        border.a = 180;
                        DrawPlane((Vector3){hx, hy, hz}, (Vector2){half * 2, half * 2}, fill);
                        DrawLine3D((Vector3){hx - half, hy, hz - half}, (Vector3){hx + half, hy, hz - half}, border);
                        DrawLine3D((Vector3){hx + half, hy, hz - half}, (Vector3){hx + half, hy, hz + half}, border);
                        DrawLine3D((Vector3){hx + half, hy, hz + half}, (Vector3){hx - half, hy, hz + half}, border);
                        DrawLine3D((Vector3){hx - half, hy, hz + half}, (Vector3){hx - half, hy, hz - half}, border);
                    }
                }

                // Axis gizmo at selected drone (Z key toggle)
                if (show_axes && vehicles[selected].active) {
                    Vector3 com = vehicles[selected].position;
                    com.y += vehicles[selected].model_scale * 0.15f;
                    draw_axis_gizmo_3d(com, vehicles[selected].model_scale * 0.5f,
                                        vehicles[selected].rotation);
                }

                // Correlation overlay: line (mode 1) or curtain (mode 2)
                // In fullscreen ortho, skip corr_mode==1 (line) — drawn in 2D instead.
                // Curtain (corr_mode==2) always stays in 3D.
                if (corr_mode > 0 && hud.pinned_count > 0) {
                    for (int p = 0; p < hud.pinned_count; p++) {
                        int pidx = hud.pinned[p];
                        if (pidx >= 0 && pidx < vehicle_count && vehicles[pidx].active
                            && pidx != selected) {
                            if (corr_mode == 1 && !fs_ortho) {
                                vehicle_draw_correlation_line(
                                    &vehicles[selected], &vehicles[pidx]);
                            } else if (corr_mode == 2) {
                                vehicle_draw_correlation_curtain(
                                    &vehicles[selected], &vehicles[pidx],
                                    scene.view_mode, scene.camera.position);
                            }
                        }
                    }
                }
            EndMode3D();

            // Ortho ground fill (2D overlay)
            scene_draw_ortho_ground(&scene, GetScreenWidth(), GetScreenHeight());

            // Fullscreen ortho 2D overlays (trails + correlation line)
            ortho_draw_fullscreen_2d(&scene, vehicles, vehicle_count,
                                      selected, trail_mode,
                                      corr_mode, hud.pinned, hud.pinned_count,
                                      GetScreenWidth(), GetScreenHeight(),
                                      hud.font_label, show_corr_labels);

            // HUD
            if (show_hud) {
                bool has_awaiting_gps = vehicles[selected].active &&
                    !vehicles[selected].origin_set && sources[selected].home.valid;
                hud_draw(&hud, vehicles, sources, vehicle_count,
                         selected, GetScreenWidth(), GetScreenHeight(),
                         scene.view_mode, ghost_mode, has_tier3, has_awaiting_gps);
            }

            // Debug panel
            {
                int active_count = 0;
                int total_trail = 0;
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active) active_count++;
                    total_trail += vehicles[i].trail_count;
                }
                debug_panel_draw(&dbg_panel, GetScreenWidth(), GetScreenHeight(),
                                 scene.view_mode, hud.font_label,
                                 vehicle_count, active_count, total_trail,
                                 vehicles[selected].position,
                                 sources[selected].ref_rejected,
                                 vehicle_tier[selected]);
            }

            // Ortho panel overlay
            int bar_h = show_hud ? hud_bar_height(&hud, GetScreenHeight()) : 0;
            ortho_panel_draw(&ortho, GetScreenHeight(), bar_h, scene.view_mode, hud.font_label,
                             vehicles, vehicle_count, selected, trail_mode,
                             corr_mode, hud.pinned, hud.pinned_count,
                             show_axes);

            // Fullscreen ortho view label
            ortho_panel_draw_fullscreen_label(GetScreenWidth(), GetScreenHeight(),
                scene.ortho_mode, scene.ortho_span, scene.view_mode, hud.font_label,
                show_axes);

        EndDrawing();
    }

    // Cleanup
    ortho_panel_cleanup(&ortho);
    hud_cleanup(&hud);
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_cleanup(&vehicles[i]);
        data_source_close(&sources[i]);
    }
    scene_cleanup(&scene);
    CloseWindow();

    return 0;
}
