#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#include <direct.h>
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
#include "theme.h"
#include "asset_path.h"

#define MAX_VEHICLES 16
#define EARTH_RADIUS 6371000.0

// Incremental Pearson correlation (position only: x, y, z)
#define CORR_CHANNELS 3
#define CORR_MIN_SAMPLES 30
#define CHORD_TIMEOUT_S 0.3

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

static void apply_vehicle_selection(hud_t *hud, int idx, bool pin,
                                    int *selected, int vehicle_count) {
    if (pin) {
        if (idx != *selected) {
            int found = -1;
            for (int p = 0; p < hud->pinned_count; p++)
                if (hud->pinned[p] == idx) { found = p; break; }
            if (found >= 0) {
                for (int p = found; p < hud->pinned_count - 1; p++)
                    hud->pinned[p] = hud->pinned[p + 1];
                hud->pinned_count--;
                hud->pinned[hud->pinned_count] = -1;
            } else if (hud->pinned_count < HUD_MAX_PINNED && hud->pinned_count < vehicle_count - 1) {
                hud->pinned[hud->pinned_count++] = idx;
            }
        }
    } else {
        *selected = idx;
        hud->pinned_count = 0;
        memset(hud->pinned, -1, sizeof(hud->pinned));
    }
}

static void replay_sync_after_seek(ulog_replay_ctx_t *ctx, data_source_t *src, vehicle_t *veh) {
    src->state = ctx->state;
    src->home = ctx->home;
    src->playback.position_s = (float)ctx->wall_accum;
    uint64_t range = ctx->parser.end_timestamp - ctx->parser.start_timestamp;
    if (range > 0) src->playback.progress = src->playback.position_s / ((float)((double)range / 1e6));
    veh->current_time = src->playback.position_s;
    vehicle_update(veh, &src->state, &src->home);
    vehicle_truncate_trail(veh, src->playback.position_s);
}


static void draw_edge_indicators(const vehicle_t *vehicles, int vehicle_count,
                                  int selected, Camera3D camera, Font font,
                                  float scale)
{
    (void)scale;
    int ei_sw = GetScreenWidth();
    int ei_sh = GetScreenHeight();
    float ei_margin = 40.0f;
    float ei_scale = powf(ei_sh / 720.0f, 0.7f);
    if (ei_scale < 1.0f) ei_scale = 1.0f;
    Vector3 cam_fwd = Vector3Normalize(Vector3Subtract(
        camera.target, camera.position));

    for (int i = 0; i < vehicle_count; i++) {
        if (i == selected || !vehicles[i].active) continue;

        Vector3 to_drone = Vector3Subtract(vehicles[i].position,
                                            camera.position);
        float dot = to_drone.x * cam_fwd.x + to_drone.y * cam_fwd.y
                    + to_drone.z * cam_fwd.z;

        Vector2 sp = GetWorldToScreen(vehicles[i].position, camera);

        if (sp.x >= ei_margin && sp.x <= ei_sw - ei_margin &&
            sp.y >= ei_margin && sp.y <= ei_sh - ei_margin) continue;

        float ei_cx = ei_sw / 2.0f;
        float ei_cy = ei_sh / 2.0f;
        float ei_dx = sp.x - ei_cx;
        float ei_dy = sp.y - ei_cy;

        if (dot < 0.5f) {
            Vector3 cam_right = Vector3Normalize(
                Vector3CrossProduct(cam_fwd, (Vector3){0, 1, 0}));
            Vector3 cam_up_approx = Vector3CrossProduct(cam_right, cam_fwd);
            ei_dx = Vector3DotProduct(to_drone, cam_right);
            ei_dy = -Vector3DotProduct(to_drone, cam_up_approx);
            float len = sqrtf(ei_dx * ei_dx + ei_dy * ei_dy);
            if (len > 0.01f) { ei_dx /= len; ei_dy /= len; }
            ei_dx *= ei_sw;
            ei_dy *= ei_sh;
        }

        float sx = (ei_dx != 0)
            ? ((ei_dx > 0 ? ei_sw - ei_margin : ei_margin) - ei_cx) / ei_dx
            : 1e9f;
        float sy = (ei_dy != 0)
            ? ((ei_dy > 0 ? ei_sh - ei_margin : ei_margin) - ei_cy) / ei_dy
            : 1e9f;
        float se = fminf(fabsf(sx), fabsf(sy));
        float ex = ei_cx + ei_dx * se;
        float ey = ei_cy + ei_dy * se;
        if (ex < ei_margin) ex = ei_margin;
        if (ex > ei_sw - ei_margin) ex = ei_sw - ei_margin;
        if (ey < ei_margin) ey = ei_margin;
        if (ey > ei_sh - ei_margin) ey = ei_sh - ei_margin;

        Color col = vehicles[i].color;
        col.a = 220;
        float angle = atan2f(ei_dy, ei_dx);
        float sz = 14.0f * ei_scale;

        // Chevron
        float chev_len = sz * 1.2f;
        float chev_spread = 0.5f;
        Vector2 tip = { ex + cosf(angle) * chev_len,
                        ey + sinf(angle) * chev_len };
        Vector2 cl = { ex + cosf(angle + chev_spread) * sz * 0.6f,
                       ey + sinf(angle + chev_spread) * sz * 0.6f };
        Vector2 cr = { ex + cosf(angle - chev_spread) * sz * 0.6f,
                       ey + sinf(angle - chev_spread) * sz * 0.6f };
        DrawLineEx(tip, cl, 2.5f * ei_scale, col);
        DrawLineEx(tip, cr, 2.5f * ei_scale, col);

        // Drone number
        char num[4];
        snprintf(num, sizeof(num), "%d", i + 1);
        float lfs = 18.0f * ei_scale;
        Vector2 tw = MeasureTextEx(font, num, lfs, 0.5f);
        float lx = ex - cosf(angle) * (sz * 0.3f) - tw.x / 2;
        float ly = ey - sinf(angle) * (sz * 0.3f) - tw.y / 2;
        DrawTextEx(font, num, (Vector2){ lx, ly }, lfs, 0.5f, col);
    }
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
    if (is_replay) {
        // Persistent trail for replay: 36000 points (~10+ min at adaptive rate)
        vehicle_init_ex(&vehicles[0], model_idx, scene.lighting_shader, 36000);
        vehicles[0].color = scene.theme->drone_palette[0];
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
            vehicles[i].color = scene.theme->drone_palette[i];
        }
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

                // Theme colors
                Color scrim_col = scene.theme->prompt_scrim;
                Color box_bg = scene.theme->prompt_box_bg;
                Color box_border = scene.theme->prompt_border;
                Color subtitle_col = scene.theme->prompt_subtitle;
                Color hint_col = scene.theme->prompt_hint;
                Color key_col = scene.theme->prompt_key;
                Color text_col = scene.theme->prompt_text;
                Color title_col = scene.theme->prompt_title;

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
                // Cancel: close sources and exit cleanly.
                // vehicle_cleanup not needed — only vehicles[0] was inited
                // and it will be freed when the process exits.
                for (int i = 0; i < num_replay_files; i++)
                    data_source_close(&sources[i]);
                vehicle_cleanup(&vehicles[0]);
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

    // Key chord state for two-digit drone selection (10-16)
    int chord_first = -1;       // first digit pressed (-1 = no chord in progress)
    double chord_time = 0.0;    // when first digit was pressed
    bool chord_shift = false;   // whether shift was held on first digit
    int trail_mode = 1;              // 0=off, 1=directional trail, 2=speed ribbon
    bool show_ground_track = false;  // ground projection off by default
    bool classic_colors = false;     // K key: toggle classic (red/blue) vs modern (yellow/purple)
    bool show_edge_indicators = true; // Ctrl+L: screen edge drone indicators
    int corr_mode = 0;               // Shift+T: 0=off, 1=ribbon, 2=line
    bool show_corr_labels = true;    // Ctrl+L: distance labels in ortho correlation
    bool show_axes = false;          // Z: axis orientation gizmo
    bool insufficient_data[MAX_VEHICLES];  // drones with no position data
    memset(insufficient_data, 0, sizeof(insufficient_data));
    int insufficient_check_frames = 0;
    bool insufficient_toasted = false;

    // Frame markers for replay
    #define MAX_MARKERS 256
    #define MARKER_LABEL_MAX 48
    float marker_times[MAX_MARKERS];
    Vector3 marker_positions[MAX_MARKERS];
    char marker_labels[MAX_MARKERS][MARKER_LABEL_MAX];
    float marker_roll[MAX_MARKERS];
    float marker_pitch[MAX_MARKERS];
    float marker_vert[MAX_MARKERS];
    float marker_speed[MAX_MARKERS];
    memset(marker_labels, 0, sizeof(marker_labels));
    int marker_count = 0;
    int current_marker = -1;
    double last_marker_drop_time = 0.0;
    int last_marker_drop_idx = -1;
    float marker_speed_max = 0.0f;

    // System markers (from ULog mode changes) — cubes, not user-editable
    #define MAX_SYS_MARKERS 256
    int sys_marker_count = 0;
    float sys_marker_times[MAX_SYS_MARKERS];
    Vector3 sys_marker_positions[MAX_SYS_MARKERS];
    char sys_marker_labels[MAX_SYS_MARKERS][MARKER_LABEL_MAX];
    float sys_marker_roll[MAX_SYS_MARKERS];
    float sys_marker_pitch[MAX_SYS_MARKERS];
    float sys_marker_vert[MAX_SYS_MARKERS];
    float sys_marker_speed[MAX_SYS_MARKERS];
    memset(sys_marker_labels, 0, sizeof(sys_marker_labels));
    int current_sys_marker = -1;
    bool sys_marker_selected = false;

    // Populate system markers from ULog mode changes
    if (is_replay) {
        ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
        int count = ctx->mode_change_count;
        if (count > MAX_SYS_MARKERS) count = MAX_SYS_MARKERS;
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                sys_marker_times[i] = ctx->mode_changes[i].time_s;
                const char *name = ulog_nav_state_name(ctx->mode_changes[i].nav_state);
                snprintf(sys_marker_labels[i], MARKER_LABEL_MAX, "%s", name);
                sys_marker_positions[i] = (Vector3){0};
            }
            sys_marker_count = count;
        }
    }

    bool sys_markers_resolved = false;

    // Pre-computed flight trail (built once after origin is established)
    #define PRECOMP_TRAIL_MAX 36000
    Vector3 *precomp_trail = NULL;
    float *precomp_roll = NULL, *precomp_pitch = NULL;
    float *precomp_vert = NULL, *precomp_speed = NULL, *precomp_time = NULL;
    int precomp_count = 0;
    float precomp_speed_max = 0.0f;
    bool precomp_ready = false;

    // Marker label input state
    bool show_marker_labels = true;
    bool marker_input_active = false;
    char marker_input_buf[MARKER_LABEL_MAX];
    int marker_input_len = 0;
    int marker_input_target = -1;

    // Main loop
    while (!WindowShouldClose()) {
        // Guard: if vehicle_count is somehow 0, exit the loop to avoid
        // out-of-bounds access on sources[selected] / vehicles[selected].
        if (vehicle_count <= 0) break;

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

            vehicles[i].current_time = sources[i].playback.position_s;
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

        // Lazy-resolve system marker positions once origin is established
        if (is_replay && !sys_markers_resolved && sys_marker_count > 0 && vehicles[0].origin_set) {
            ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
            float saved_pos = sources[0].playback.position_s;
            int valid = 0;
            for (int i = 0; i < sys_marker_count; i++) {
                ulog_replay_seek(ctx, sys_marker_times[i]);
                replay_sync_after_seek(ctx, &sources[0], &vehicles[0]);
                if (sources[0].state.lat == 0 && sources[0].state.lon == 0) continue;
                Vector3 mp = vehicles[0].position;
                if (mp.x == 0.0f && mp.y == 0.0f && mp.z == 0.0f) continue;
                float mdist = sqrtf(mp.x * mp.x + mp.y * mp.y + mp.z * mp.z);
                if (mdist > 5000.0f) continue;
                sys_marker_times[valid] = sys_marker_times[i];
                memcpy(sys_marker_labels[valid], sys_marker_labels[i], MARKER_LABEL_MAX);
                sys_marker_positions[valid] = vehicles[0].position;
                sys_marker_roll[valid] = vehicles[0].roll_deg;
                sys_marker_pitch[valid] = vehicles[0].pitch_deg;
                sys_marker_vert[valid] = vehicles[0].vertical_speed;
                sys_marker_speed[valid] = sqrtf(vehicles[0].ground_speed * vehicles[0].ground_speed +
                                                 vehicles[0].vertical_speed * vehicles[0].vertical_speed);
                valid++;
            }
            sys_marker_count = valid;

            // Pre-compute entire flight trail from ULog position data
            if (!precomp_ready) {
                precomp_trail = calloc(PRECOMP_TRAIL_MAX, sizeof(Vector3));
                precomp_roll  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_pitch = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_vert  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_speed = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
                precomp_time  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));

                if (precomp_trail && precomp_roll && precomp_pitch &&
                    precomp_vert && precomp_speed && precomp_time) {
                    double lat0 = vehicles[0].lat0;
                    double lon0 = vehicles[0].lon0;
                    double alt0 = vehicles[0].alt0;
                    float cos_lat0 = (float)cos(lat0);
                    Vector3 prev_pos = {0};
                    bool prev_valid = false;
                    int pc = 0;

                    ulog_replay_seek(ctx, 0.0f);
                    float step = 0.2f;
                    while (pc < PRECOMP_TRAIL_MAX) {
                        float prev_wall = (float)ctx->wall_accum;
                        bool ok = ulog_replay_advance(ctx, step, 1.0f, false, false);
                        if (!ok || (float)ctx->wall_accum <= prev_wall) break;

                        hil_state_t *st = &ctx->state;
                        if (!st->valid) continue;
                        if (st->lat == 0 && st->lon == 0) continue;

                        double lat = st->lat * 1e-7 * (M_PI / 180.0);
                        double lon = st->lon * 1e-7 * (M_PI / 180.0);
                        double alt = st->alt * 1e-3;
                        double ned_x = 6371000.0 * (lat - lat0);
                        double ned_y = 6371000.0 * (lon - lon0) * cos_lat0;
                        double ned_z = alt - alt0;
                        Vector3 pos = {
                            (float)ned_y,
                            (float)ned_z < 0.0f ? 0.0f : (float)ned_z,
                            (float)(-ned_x)
                        };

                        if (prev_valid) {
                            float dx = pos.x - prev_pos.x;
                            float dy = pos.y - prev_pos.y;
                            float dz = pos.z - prev_pos.z;
                            if (dx*dx + dy*dy + dz*dz < 0.25f) continue;
                        }

                        float qw = st->quaternion[0], qx = st->quaternion[1];
                        float qy = st->quaternion[2], qz = st->quaternion[3];
                        if (qw < 0) { qw = -qw; qx = -qx; qy = -qy; qz = -qz; }
                        float roll_v = atan2f(2.0f*(qw*qx + qy*qz),
                                            1.0f - 2.0f*(qx*qx + qy*qy)) * RAD2DEG;
                        float sin_p = 2.0f*(qw*qy - qz*qx);
                        if (sin_p > 1.0f) sin_p = 1.0f;
                        if (sin_p < -1.0f) sin_p = -1.0f;
                        float pitch_v = asinf(sin_p) * RAD2DEG;
                        float vert_s = -st->vz * 0.01f;
                        float gs = sqrtf((float)st->vx*st->vx + (float)st->vy*st->vy) * 0.01f;
                        float spd = sqrtf(gs*gs + vert_s*vert_s);

                        precomp_trail[pc] = pos;
                        precomp_roll[pc] = roll_v;
                        precomp_pitch[pc] = pitch_v;
                        precomp_vert[pc] = vert_s;
                        precomp_speed[pc] = spd;
                        precomp_time[pc] = (float)ctx->wall_accum;
                        if (spd > precomp_speed_max) precomp_speed_max = spd;
                        pc++;
                        prev_pos = pos;
                        prev_valid = true;
                    }
                    precomp_count = pc;
                    precomp_ready = true;
                }
            }

            // Restore playback to where it was
            ulog_replay_seek(ctx, saved_pos);
            vehicle_reset_trail(&vehicles[0]);
            replay_sync_after_seek(ctx, &sources[0], &vehicles[0]);
            sys_markers_resolved = true;
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

        // Handle file drops (.mvt theme files)
        if (IsFileDropped()) {
            FilePathList dropped = LoadDroppedFiles();
            for (unsigned int i = 0; i < dropped.count; i++) {
                if (theme_registry_add(&scene.theme_reg, dropped.paths[i])) {
                    int last = scene.theme_reg.user_count - 1;
                    char msg[80];
                    snprintf(msg, sizeof(msg), "Theme: %s", scene.theme_reg.name_bufs[last]);
                    hud_toast(&hud, msg, 3.0f);

                    // Copy to themes/ so it persists
                    const char *src = dropped.paths[i];
                    const char *fname = src;
                    for (const char *p = src; *p; p++) {
                        if (*p == '/' || *p == '\\') fname = p + 1;
                    }
                    char dest[512];
                    snprintf(dest, sizeof(dest), "./themes/%s", fname);
                    // Ensure themes/ exists, then copy if not already there
                    #ifdef _WIN32
                    _mkdir("./themes");
                    #else
                    mkdir("./themes", 0755);
                    #endif
                    if (strcmp(src, dest) != 0) {
                        FILE *fin = fopen(src, "rb");
                        FILE *fout = fin ? fopen(dest, "wb") : NULL;
                        if (fin && fout) {
                            char buf[4096];
                            size_t n;
                            while ((n = fread(buf, 1, sizeof(buf), fin)) > 0)
                                fwrite(buf, 1, n, fout);
                            printf("Theme saved to %s\n", dest);
                        }
                        if (fin) fclose(fin);
                        if (fout) fclose(fout);
                    }
                }
            }
            UnloadDroppedFiles(dropped);
        }

        // Handle input (blocked during marker label entry)
        if (!marker_input_active) {
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

                Color pscrim = scene.theme->prompt_scrim;
                Color pbbg = scene.theme->prompt_box_bg;
                Color pbbd = scene.theme->prompt_border;
                Color psub = scene.theme->prompt_subtitle;
                Color phint_c = scene.theme->prompt_hint;
                Color pkc = scene.theme->prompt_key;
                Color ptc = scene.theme->prompt_text;
                Color pttl = scene.theme->prompt_title;

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
            for (int i = 0; i < vehicle_count; i++)
                vehicles[i].color = scene.theme->drone_palette[i];
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

        // Toggle screen edge indicators and correlation labels (Ctrl+L)
        if ((IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_L)) {
            show_edge_indicators = !show_edge_indicators;
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
                chord_first = -1;
            }
            if (IsKeyPressed(KEY_LEFT_BRACKET) && !is_replay) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int prev = (selected - j + vehicle_count) % vehicle_count;
                    if (sources[prev].connected) { selected = prev; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
                chord_first = -1;
            }
            if (IsKeyPressed(KEY_RIGHT_BRACKET) && !is_replay) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int next = (selected + j) % vehicle_count;
                    if (sources[next].connected) { selected = next; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
                chord_first = -1;
            }
            // Number keys 0-9: single digit or two-digit chord for drones 10-16
            // Chord: press first digit, then second within 300ms (e.g. 1+0 = drone 10)
            {
                int digit = -1;
                for (int k = KEY_ZERO; k <= KEY_NINE; k++) {
                    if (IsKeyPressed(k)) { digit = k - KEY_ZERO; break; }
                }

                bool shift_held = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
                bool ctrl_held = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);

                // Check chord timeout
                if (chord_first >= 0 && GetTime() - chord_time > CHORD_TIMEOUT_S) {
                    // Timeout: apply first digit as single-digit selection
                    int idx = chord_first - 1;  // digit 1 = drone index 0
                    if (idx >= 0 && idx < vehicle_count)
                        apply_vehicle_selection(&hud, idx, chord_shift, &selected, vehicle_count);
                    chord_first = -1;
                }

                if (digit >= 0 && !ctrl_held) {
                    if (chord_first >= 0) {
                        // Second digit of chord: combine into two-digit number
                        int two_digit = chord_first * 10 + digit;
                        int idx = two_digit - 1;  // drone 10 = index 9
                        chord_first = -1;

                        if (idx >= 0 && idx < vehicle_count)
                            apply_vehicle_selection(&hud, idx, chord_shift, &selected, vehicle_count);
                    } else if (digit >= 1 && digit <= 9) {
                        // First digit: start chord or apply immediately if vehicle_count <= 9
                        if (vehicle_count > 9) {
                            // Could be start of two-digit chord
                            chord_first = digit;
                            chord_time = GetTime();
                            chord_shift = shift_held;
                        } else {
                            // No need for chords, apply single digit immediately
                            int idx = digit - 1;
                            if (idx < vehicle_count)
                                apply_vehicle_selection(&hud, idx, shift_held, &selected, vehicle_count);
                        }
                    }
                    // Note: digit 0 alone is ignored (no drone 0); only valid as chord second digit
                }
            }
        }
        } // end !marker_input_active guard

        // Marker label text input — consumes all keyboard while active
        if (marker_input_active) {
            int ch;
            while ((ch = GetCharPressed()) != 0) {
                if (marker_input_len < MARKER_LABEL_MAX - 1 && ch >= 32 && ch < 127) {
                    marker_input_buf[marker_input_len++] = (char)ch;
                    marker_input_buf[marker_input_len] = '\0';
                }
            }
            if (IsKeyPressed(KEY_BACKSPACE) && marker_input_len > 0) {
                marker_input_buf[--marker_input_len] = '\0';
            }
            if (IsKeyPressed(KEY_ENTER)) {
                if (marker_input_target >= 0 && marker_input_target < marker_count) {
                    memcpy(marker_labels[marker_input_target], marker_input_buf, MARKER_LABEL_MAX);
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, marker_times[marker_input_target]);
                    replay_sync_after_seek(rctx, &sources[0], &vehicles[0]);
                }
                current_marker = marker_input_target;
                marker_input_active = false;
            }
            if (IsKeyPressed(KEY_ESCAPE)) {
                if (marker_input_target >= 0 && marker_input_target < marker_count) {
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, marker_times[marker_input_target]);
                    replay_sync_after_seek(rctx, &sources[0], &vehicles[0]);
                }
                current_marker = marker_input_target;
                marker_input_active = false;
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
        if (is_replay && !marker_input_active) {
            bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
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
            if (IsKeyPressed(KEY_L) && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL) && shift) {
                bool l = !sources[selected].playback.looping;
                for (int i = 0; i < nrf; i++)
                    sources[i].playback.looping = l;
            }
            if (IsKeyPressed(KEY_Y)) {
                hud.show_yaw = !hud.show_yaw;
            }
            if (IsKeyPressed(KEY_L) && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL) && !shift && (last_marker_drop_idx < 0 || GetTime() - last_marker_drop_time >= 0.5)) {
                show_marker_labels = !show_marker_labels;
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
                marker_count = 0;
                current_marker = -1;
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

            // Timeline scrubbing: 3 levels of granularity
            // Shift+Arrow = single frame step (~20ms), Ctrl+Shift = 1s, plain = 5s
            if (IsKeyPressed(KEY_RIGHT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                float seek_target = sources[0].playback.position_s + step;
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(rctx, seek_target);
                    sources[i].playback.position_s = (float)rctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
                replay_sync_after_seek(ctx, &sources[0], &vehicles[0]);
            }
            if (IsKeyPressed(KEY_LEFT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                float target = sources[0].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                for (int i = 0; i < nrf; i++) {
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[i].impl;
                    ulog_replay_seek(rctx, target);
                    sources[i].playback.position_s = (float)rctx->wall_accum;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
                replay_sync_after_seek(ctx, &sources[0], &vehicles[0]);
            }

            // Frame markers: B = drop marker, B->L = drop + label, Shift+B = delete current
            if (IsKeyPressed(KEY_B) && vehicles[0].active && !marker_input_active) {
                if (shift) {
                    // Shift+B: delete currently highlighted marker
                    if (current_marker >= 0 && current_marker < marker_count) {
                        for (int m = current_marker; m < marker_count - 1; m++) {
                            marker_times[m] = marker_times[m + 1];
                            marker_positions[m] = marker_positions[m + 1];
                            memcpy(marker_labels[m], marker_labels[m + 1], MARKER_LABEL_MAX);
                            marker_roll[m] = marker_roll[m + 1];
                            marker_pitch[m] = marker_pitch[m + 1];
                            marker_vert[m] = marker_vert[m + 1];
                            marker_speed[m] = marker_speed[m + 1];
                        }
                        marker_count--;
                        if (current_marker >= marker_count) current_marker = marker_count - 1;
                        last_marker_drop_idx = -1;
                    }
                } else if (marker_count < MAX_MARKERS) {
                    // B: drop marker at current position
                    float t = sources[0].playback.position_s;
                    Vector3 pos = vehicles[0].position;
                    int insert = marker_count;
                    for (int m = 0; m < marker_count; m++) {
                        if (marker_times[m] > t) { insert = m; break; }
                    }
                    for (int m = marker_count; m > insert; m--) {
                        marker_times[m] = marker_times[m - 1];
                        marker_positions[m] = marker_positions[m - 1];
                        memcpy(marker_labels[m], marker_labels[m - 1], MARKER_LABEL_MAX);
                        marker_roll[m] = marker_roll[m - 1];
                        marker_pitch[m] = marker_pitch[m - 1];
                        marker_vert[m] = marker_vert[m - 1];
                        marker_speed[m] = marker_speed[m - 1];
                    }
                    marker_times[insert] = t;
                    marker_positions[insert] = pos;
                    marker_labels[insert][0] = '\0';
                    marker_roll[insert] = vehicles[0].roll_deg;
                    marker_pitch[insert] = vehicles[0].pitch_deg;
                    marker_vert[insert] = vehicles[0].vertical_speed;
                    marker_speed[insert] = sqrtf(vehicles[0].ground_speed * vehicles[0].ground_speed +
                                                  vehicles[0].vertical_speed * vehicles[0].vertical_speed);
                    marker_count++;
                    current_marker = insert;
                    sys_marker_selected = false;
                    current_sys_marker = -1;
                    last_marker_drop_time = GetTime();
                    last_marker_drop_idx = insert;
                }
            }

            // B->L chord: if L pressed within 0.5s of dropping a marker, open label input
            if (IsKeyPressed(KEY_L) && !marker_input_active && last_marker_drop_idx >= 0) {
                double elapsed = GetTime() - last_marker_drop_time;
                if (elapsed < 0.5) {
                    marker_input_active = true;
                    marker_input_target = last_marker_drop_idx;
                    marker_input_buf[0] = '\0';
                    marker_input_len = 0;
                    sources[0].playback.paused = true;
                    last_marker_drop_idx = -1;
                }
            }

            // Unified [/] cycling through both user markers and system markers (sorted by time)
            if ((IsKeyPressed(KEY_LEFT_BRACKET) || IsKeyPressed(KEY_RIGHT_BRACKET))
                && (marker_count > 0 || sys_marker_count > 0)) {
                int total = marker_count + sys_marker_count;
                typedef struct { float time; int idx; bool is_sys; } merged_marker_t;
                merged_marker_t merged[MAX_MARKERS + MAX_SYS_MARKERS];
                int mi = 0;
                for (int m = 0; m < marker_count; m++) {
                    merged[mi++] = (merged_marker_t){marker_times[m], m, false};
                }
                for (int m = 0; m < sys_marker_count; m++) {
                    merged[mi++] = (merged_marker_t){sys_marker_times[m], m, true};
                }
                for (int a = 1; a < total; a++) {
                    merged_marker_t key = merged[a];
                    int b = a - 1;
                    while (b >= 0 && merged[b].time > key.time) {
                        merged[b + 1] = merged[b];
                        b--;
                    }
                    merged[b + 1] = key;
                }

                int cur_merged = -1;
                if (sys_marker_selected && current_sys_marker >= 0) {
                    for (int m = 0; m < total; m++) {
                        if (merged[m].is_sys && merged[m].idx == current_sys_marker) {
                            cur_merged = m; break;
                        }
                    }
                } else if (!sys_marker_selected && current_marker >= 0) {
                    for (int m = 0; m < total; m++) {
                        if (!merged[m].is_sys && merged[m].idx == current_marker) {
                            cur_merged = m; break;
                        }
                    }
                }

                int target_merged = -1;
                if (IsKeyPressed(KEY_LEFT_BRACKET)) {
                    if (cur_merged > 0) {
                        target_merged = cur_merged - 1;
                    } else if (cur_merged == 0) {
                        target_merged = total - 1;
                    } else {
                        float t = sources[0].playback.position_s;
                        for (int m = total - 1; m >= 0; m--) {
                            if (merged[m].time <= t) { target_merged = m; break; }
                        }
                        if (target_merged < 0) target_merged = total - 1;
                    }
                } else {
                    if (cur_merged >= 0 && cur_merged < total - 1) {
                        target_merged = cur_merged + 1;
                    } else if (cur_merged < 0) {
                        float t = sources[0].playback.position_s;
                        for (int m = 0; m < total; m++) {
                            if (merged[m].time >= t) { target_merged = m; break; }
                        }
                        if (target_merged < 0) target_merged = 0;
                    } else {
                        target_merged = 0;
                    }
                }

                if (target_merged >= 0 && target_merged < total) {
                    merged_marker_t tgt = merged[target_merged];
                    float seek_time = tgt.time;
                    Vector3 seek_pos = tgt.is_sys ? sys_marker_positions[tgt.idx] : marker_positions[tgt.idx];

                    sys_marker_selected = tgt.is_sys;
                    if (tgt.is_sys) {
                        current_sys_marker = tgt.idx;
                        current_marker = -1;
                    } else {
                        current_marker = tgt.idx;
                        current_sys_marker = -1;
                    }

                    if (shift) {
                        scene.cam_mode = CAM_MODE_FREE;
                        scene.free_track = true;
                        scene.camera.position = seek_pos;
                        scene.camera.target = vehicles[0].position;
                    } else {
                    ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;

                    // Seek and sync state
                    ulog_replay_seek(ctx, seek_time);
                    sources[0].state = ctx->state;
                    sources[0].home = ctx->home;
                    sources[0].playback.position_s = (float)ctx->wall_accum;
                    uint64_t _range = ctx->parser.end_timestamp - ctx->parser.start_timestamp;
                    if (_range > 0) sources[0].playback.progress = sources[0].playback.position_s / ((float)((double)_range / 1e6));
                    vehicles[0].current_time = sources[0].playback.position_s;

                    // Restore trail from pre-computed data up to seek_time
                    if (precomp_ready && precomp_count > 0) {
                        int lo = 0, hi = precomp_count - 1, cut = 0;
                        while (lo <= hi) {
                            int mid = (lo + hi) / 2;
                            if (precomp_time[mid] <= seek_time) {
                                cut = mid + 1;
                                lo = mid + 1;
                            } else {
                                hi = mid - 1;
                            }
                        }
                        vehicle_t *v = &vehicles[0];
                        int n = cut;
                        if (n > v->trail_capacity) n = v->trail_capacity;
                        int src_start = cut - n;
                        v->trail_count = n;
                        v->trail_head = n % v->trail_capacity;
                        v->trail_speed_max = 0.0f;
                        for (int ti = 0; ti < n; ti++) {
                            int si = src_start + ti;
                            v->trail[ti] = precomp_trail[si];
                            v->trail_roll[ti] = precomp_roll[si];
                            v->trail_pitch[ti] = precomp_pitch[si];
                            v->trail_vert[ti] = precomp_vert[si];
                            v->trail_speed[ti] = precomp_speed[si];
                            v->trail_time[ti] = precomp_time[si];
                            if (precomp_speed[si] > v->trail_speed_max)
                                v->trail_speed_max = precomp_speed[si];
                        }
                    }
                    // Update vehicle state and suppress jump detector on next frame
                    vehicle_update(&vehicles[0], &sources[0].state, &sources[0].home);
                    last_pos[0] = vehicles[0].position;
                    } // end else (non-shift seek)
                }
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
                               selected, scene.theme,
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
                        vehicle_draw(&vehicles[i], scene.theme, i == selected,
                                     tm_3d, show_ground_track, scene.camera.position,
                                     classic_colors);
                    }
                }
                // Draw frame marker spheres during replay
                if (is_replay && marker_count > 0) {
                    vehicle_draw_markers(marker_positions, marker_labels, marker_count,
                                         sys_marker_selected ? -1 : current_marker,
                                         scene.camera.position, scene.camera,
                                         marker_roll, marker_pitch, marker_vert, marker_speed,
                                         vehicles[0].trail_speed_max, scene.theme, trail_mode);
                }
                // Draw system marker cubes during replay
                if (is_replay && sys_marker_count > 0) {
                    vehicle_draw_sys_markers(sys_marker_positions, sys_marker_labels, sys_marker_count,
                                             sys_marker_selected ? current_sys_marker : -1,
                                             scene.camera.position,
                                             sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed,
                                             vehicles[0].trail_speed_max, scene.theme, trail_mode);
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
                                    scene.theme, scene.camera.position);
                            }
                        }
                    }
                }
            EndMode3D();

            // Ortho ground fill (2D overlay)
            scene_draw_ortho_ground(&scene, GetScreenWidth(), GetScreenHeight());

            // Screen edge indicators for off-screen drones
            if (vehicle_count > 1 && show_edge_indicators) {
                float ei_scale = powf(GetScreenHeight() / 720.0f, 0.7f);
                if (ei_scale < 1.0f) ei_scale = 1.0f;
                draw_edge_indicators(vehicles, vehicle_count, selected,
                                     scene.camera, hud.font_value, ei_scale);
            }

            // Marker labels (2D billboarded text, after EndMode3D)
            if (is_replay && marker_count > 0 && show_marker_labels) {
                vehicle_draw_marker_labels(marker_positions, marker_labels, marker_count,
                                           sys_marker_selected ? -1 : current_marker,
                                           scene.camera.position, scene.camera,
                                           hud.font_label, hud.font_value,
                                           marker_roll, marker_pitch, marker_vert, marker_speed,
                                           vehicles[0].trail_speed_max, scene.theme, trail_mode);
            }
            // System marker labels
            if (is_replay && sys_marker_count > 0 && show_marker_labels) {
                vehicle_draw_sys_marker_labels(sys_marker_positions, sys_marker_labels, sys_marker_count,
                                               sys_marker_selected ? current_sys_marker : -1,
                                               scene.camera.position, scene.camera,
                                               hud.font_label, hud.font_value,
                                               sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed,
                                               vehicles[0].trail_speed_max, scene.theme, trail_mode);
            }

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
                         scene.theme, trail_mode,
                         marker_times, marker_labels,
                         marker_count, current_marker,
                         marker_roll, marker_pitch,
                         marker_vert, marker_speed,
                         marker_speed_max,
                         sys_marker_times, sys_marker_labels,
                         sys_marker_count, current_sys_marker, sys_marker_selected,
                         sys_marker_roll, sys_marker_pitch,
                         sys_marker_vert, sys_marker_speed,
                         ghost_mode, has_tier3, has_awaiting_gps);
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
                                 scene.theme, hud.font_label,
                                 vehicle_count, active_count, total_trail,
                                 vehicles[selected].position,
                                 sources[selected].ref_rejected,
                                 vehicle_tier[selected]);
            }

            // Ortho panel overlay
            int bar_h = show_hud ? hud_bar_height(&hud, GetScreenHeight()) : 0;
            ortho_panel_draw(&ortho, GetScreenHeight(), bar_h, scene.theme, hud.font_label,
                             vehicles, vehicle_count, selected, trail_mode,
                             corr_mode, hud.pinned, hud.pinned_count,
                             show_axes);

            // Fullscreen ortho view label
            ortho_panel_draw_fullscreen_label(GetScreenWidth(), GetScreenHeight(),
                scene.ortho_mode, scene.ortho_span, scene.theme, hud.font_label,
                show_axes);

            // Marker label input overlay (view-mode-aware)
            if (marker_input_active) {
                int sw = GetScreenWidth(), sh = GetScreenHeight();
                float s = powf(sh / 720.0f, 0.7f);

                const theme_t *th = scene.theme;
                Color scrim_col    = th->prompt_scrim;
                Color box_bg       = th->prompt_box_bg;
                Color box_border   = th->prompt_border;
                Color prompt_col   = th->prompt_subtitle;
                Color hint_col     = th->prompt_hint;
                Color field_bg     = th->hud_bg;
                Color field_border = th->hud_border;
                Color text_col     = th->prompt_text;
                Color cursor_col   = th->hud_accent;

                DrawRectangle(0, 0, sw, sh, scrim_col);

                float box_w = 520 * s, box_h = 110 * s;
                float bx = (sw - box_w) / 2, by = (sh - box_h) / 2;
                Rectangle box = {bx, by, box_w, box_h};

                DrawRectangleRounded(box, 0.06f, 8, box_bg);
                DrawRectangleRoundedLinesEx(box, 0.06f, 8, 1.5f * s, box_border);

                float prompt_fs = 15 * s;
                DrawTextEx(hud.font_label, "MARKER LABEL",
                           (Vector2){bx + 16 * s, by + 12 * s}, prompt_fs, 0.5f, prompt_col);

                float hint_fs = 12 * s;
                float hint_w = MeasureTextEx(hud.font_label, "Enter to confirm  |  Esc to cancel", hint_fs, 0.5f).x;
                DrawTextEx(hud.font_label, "Enter to confirm  |  Esc to cancel",
                           (Vector2){bx + box_w - hint_w - 16 * s, by + 12 * s}, hint_fs, 0.5f, hint_col);

                float field_x = bx + 16 * s, field_y = by + 40 * s;
                float field_w = box_w - 32 * s, field_h = 44 * s;
                DrawRectangleRounded((Rectangle){field_x, field_y, field_w, field_h},
                                     0.08f, 6, field_bg);
                DrawRectangleRoundedLinesEx((Rectangle){field_x, field_y, field_w, field_h},
                                     0.08f, 6, 1.0f * s, field_border);

                float input_fs = 20 * s;
                float text_y = field_y + (field_h - input_fs) / 2;
                DrawTextEx(hud.font_value, marker_input_buf,
                           (Vector2){field_x + 12 * s, text_y}, input_fs, 0.5f, text_col);

                Vector2 tw = MeasureTextEx(hud.font_value, marker_input_buf, input_fs, 0.5f);
                if ((int)(GetTime() * 2.0) % 2 == 0) {
                    float cx = field_x + 12 * s + tw.x + 2;
                    DrawRectangle((int)cx, (int)(text_y), (int)(2 * s), (int)input_fs, cursor_col);
                }
            }

        EndDrawing();
    }

    // Cleanup
    ortho_panel_cleanup(&ortho);
    hud_cleanup(&hud);
    for (int i = 0; i < vehicle_count; i++) {
        vehicle_cleanup(&vehicles[i]);
        if (sources[i].ops) data_source_close(&sources[i]);
    }
    scene_cleanup(&scene);
    free(precomp_trail); free(precomp_roll); free(precomp_pitch);
    free(precomp_vert); free(precomp_speed); free(precomp_time);
    CloseWindow();

    return 0;
}
