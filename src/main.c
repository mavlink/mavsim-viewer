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
#include "vehicle.h"
#include "scene.h"
#include "hud.h"
#include "ui_logic.h"
#include "debug_panel.h"
#include "ortho_panel.h"
#include "theme.h"
#include "asset_path.h"
#include "replay_conflict.h"
#include "replay_trail.h"
#include "replay_markers.h"
#include "ui_marker_input.h"
#include "tactical_hud.h"

#define MAX_VEHICLES 16
#define EARTH_RADIUS 6371000.0

#include "correlation.h"

#define CHORD_TIMEOUT_S 0.3

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

/* Thin wrapper: delegates to the testable inline in ui_logic.h */
static void apply_vehicle_selection_hud(hud_t *hud, int idx, bool pin,
                                        int *selected, int vehicle_count) {
    apply_vehicle_selection(hud->pinned, &hud->pinned_count,
                            idx, pin, selected, vehicle_count);
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
    InitWindow(win_w, win_h, "Hawkeye");
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
        for (int i = 0; i < num_replay_files; i++) {
            vehicle_init_ex(&vehicles[i], model_idx, scene.lighting_shader, 36000);
            vehicles[i].color = scene.theme->drone_palette[i % 16];
        }
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
            // takeoff_conf already populated by data_source_ulog_create
            sources[i].playback.time_offset_s = 0.0f;
        }
    }

    // ── Conflict detection + resolution (multi-file replay, not --ghost) ──
    bool conflict_detected = false;
    bool conflict_far = false;
    bool ghost_mode_grid = false;

    if (is_replay && num_replay_files > 1 && !ghost_mode) {
        conflict_result_t cr = replay_detect_conflict(sources, num_replay_files);
        conflict_detected = cr.conflict_detected;
        conflict_far = cr.conflict_far;

        if (conflict_detected) {
            // Init HUD early for fonts
            hud_t prompt_hud;
            hud_init(&prompt_hud);

            const char *grid_label = conflict_far ? "Narrow grid offset" : "Grid offset";
            char subtitle[64];
            if (conflict_far)
                snprintf(subtitle, sizeof(subtitle), "  -  %d drones too far apart", num_replay_files);
            else
                snprintf(subtitle, sizeof(subtitle), "  -  %d drones overlap", num_replay_files);
            const char *labels[] = {"Cancel & reupload", "Ghost mode", grid_label};

            int choice = draw_prompt_dialog("POSITION CONFLICT", subtitle,
                                            labels, 3, scene.theme,
                                            prompt_hud.font_label, prompt_hud.font_value,
                                            &scene);

            hud_cleanup(&prompt_hud);

            if (choice <= 1) {
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
            if (sources[i].playback.home_from_topic) vehicle_tier[i] = 1;
            else if (sources[i].home.valid) vehicle_tier[i] = 2;
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
    float saved_chase_distance = 0.0f;
    float tactical_chase_target = 1.6f;

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

    // Per-drone markers, system markers, and pre-computed trails
    user_markers_t markers[MAX_VEHICLES] = {0};
    sys_markers_t sys_markers[MAX_VEHICLES] = {0};
    precomp_trail_t precomp[MAX_VEHICLES] = {0};
    for (int i = 0; i < vehicle_count; i++) {
        markers[i].current = -1;
        markers[i].last_drop_idx = -1;
        sys_markers[i].current = -1;
        precomp_trail_init(&precomp[i]);
        if (is_replay)
            replay_init_sys_markers(&sys_markers[i], &sources[i]);
    }

    // Marker label input state
    bool show_marker_labels = true;
    marker_input_t marker_input = {0};
    marker_input.target = -1;

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
        for (int i = 0; i < vehicle_count; i++) {
            if (is_replay && !sys_markers[i].resolved && sys_markers[i].count > 0
                && vehicles[i].origin_set) {
                replay_resolve_and_build_trail(&sys_markers[i], &precomp[i],
                                               &sources[i], &vehicles[i]);
            }
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
        if (!marker_input.active) {
        scene_handle_input(&scene);

        // P key: switch between Swarm / Ghost / Grid modes during multi-file replay
        // P key: mode switcher for multi-file replay
        // - No conflict: Formation / Ghost / Grid
        // - Conflict (close/no-data): Cancel / Ghost / Grid offset
        // - Conflict (too far): Cancel / Ghost / Narrow grid
        if (IsKeyPressed(KEY_P) && is_replay && num_replay_files > 1) {
            const char *p_title;
            const char *pl[3];
            const char *p_grid_label = conflict_far ? "Narrow grid" : "Grid offset";
            char p_subtitle[64];
            if (conflict_detected) {
                p_title = "POSITION CONFLICT";
                pl[0] = "Cancel"; pl[1] = "Ghost mode"; pl[2] = p_grid_label;
                if (conflict_far)
                    snprintf(p_subtitle, sizeof(p_subtitle), "  -  %d drones too far apart", num_replay_files);
                else
                    snprintf(p_subtitle, sizeof(p_subtitle), "  -  %d drones overlap", num_replay_files);
            } else {
                p_title = "REPLAY MODE";
                pl[0] = "Formation"; pl[1] = "Ghost"; pl[2] = "Grid offset";
                snprintf(p_subtitle, sizeof(p_subtitle), "  -  %d drones", num_replay_files);
            }

            int ch = draw_prompt_dialog(p_title, p_subtitle,
                                        pl, 3, scene.theme,
                                        hud.font_label, hud.font_value,
                                        &scene);
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

        // Cycle HUD mode: Console → Tactical → Off
        if (IsKeyPressed(KEY_H)) {
            hud_mode_t prev_mode = hud.mode;
            hud.mode = (hud.mode + 1) % HUD_MODE_COUNT;
            const char *mode_names[] = { "Console HUD", "Tactical HUD", "HUD Off" };
            hud_toast(&hud, mode_names[hud.mode], 2.0f);
            show_hud = (hud.mode != HUD_OFF);

            if (hud.mode == HUD_TACTICAL) {
                saved_chase_distance = scene.chase_distance;
                scene.chase_distance = tactical_chase_target;
            } else if (prev_mode == HUD_TACTICAL) {
                scene.chase_distance = saved_chase_distance;
            }
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
                bool alt_held = IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT);

                // Check chord timeout
                if (chord_first >= 0 && GetTime() - chord_time > CHORD_TIMEOUT_S) {
                    // Timeout: apply first digit as single-digit selection
                    int idx = chord_first - 1;  // digit 1 = drone index 0
                    if (idx >= 0 && idx < vehicle_count)
                        apply_vehicle_selection_hud(&hud, idx, chord_shift, &selected, vehicle_count);
                    chord_first = -1;
                }

                if (digit >= 0 && !ctrl_held && !alt_held) {
                    if (chord_first >= 0) {
                        // Second digit of chord: combine into two-digit number
                        int two_digit = chord_first * 10 + digit;
                        int idx = two_digit - 1;  // drone 10 = index 9
                        chord_first = -1;

                        if (idx >= 0 && idx < vehicle_count)
                            apply_vehicle_selection_hud(&hud, idx, chord_shift, &selected, vehicle_count);
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
                                apply_vehicle_selection_hud(&hud, idx, shift_held, &selected, vehicle_count);
                        }
                    }
                    // Note: digit 0 alone is ignored (no drone 0); only valid as chord second digit
                }
            }
        }
        } // end !marker_input.active guard

        // Marker label text input — consumes all keyboard while active
        if (marker_input.active) {
            int di = marker_input.drone_idx;
            marker_input_update(&marker_input, &markers[di], &sources[di], &vehicles[di]);
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
        if (is_replay && !marker_input.active) {
            bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
            int nrf = num_replay_files > 0 ? num_replay_files : 1;
            if (IsKeyPressed(KEY_SPACE)) {
                if (!sources[selected].connected) {
                    for (int i = 0; i < nrf; i++) {
                        data_source_seek(&sources[i], 0.0f);
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
            if (IsKeyPressed(KEY_L) && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL) && !shift && (markers[selected].last_drop_idx < 0 || GetTime() - markers[selected].last_drop_time >= 0.5)) {
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
                    data_source_seek(&sources[i], 0.0f);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
                for (int i = 0; i < vehicle_count; i++) {
                    markers[i].count = 0;
                    markers[i].current = -1;
                }
            }
            // A key: toggle takeoff time alignment
            if (IsKeyPressed(KEY_A) && num_replay_files > 1) {
                takeoff_aligned = !takeoff_aligned;
                const float takeoff_buffer = 5.0f;
                for (int i = 0; i < nrf; i++) {
                    if (takeoff_aligned) {
                        float skip = sources[i].playback.takeoff_detected
                            ? sources[i].playback.takeoff_time_s - takeoff_buffer : 0.0f;
                        if (skip < 0.0f) skip = 0.0f;
                        data_source_set_time_offset(&sources[i], (double)skip);
                    } else {
                        data_source_set_time_offset(&sources[i], 0.0);
                    }
                    data_source_seek(&sources[i], 0.0f);
                    sources[i].connected = true;
                    sources[i].playback.paused = false;
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
                for (int i = 0; i < nrf; i++) {
                    sources[i].playback.correlation = NAN;
                    sources[i].playback.rmse = NAN;
                }
                // Re-resolve system marker positions with new time offsets
                for (int i = 0; i < nrf; i++)
                    sys_markers[i].resolved = false;
                hud_toast(&hud, takeoff_aligned ? "Auto Align On" : "Auto Align Off", 2.0f);
            }

            // Timeline scrubbing: 3 levels of granularity
            // Shift+Arrow = single frame step (~20ms), Ctrl+Shift = 1s, plain = 5s
            if (IsKeyPressed(KEY_RIGHT)) {
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                float seek_target = sources[0].playback.position_s + step;
                for (int i = 0; i < nrf; i++) {
                    data_source_seek(&sources[i], seek_target);
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
                replay_sync_vehicle(&sources[0], &vehicles[0]);
            }
            if (IsKeyPressed(KEY_LEFT)) {
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                float target = sources[0].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                for (int i = 0; i < nrf; i++) {
                    data_source_seek(&sources[i], target);
                    vehicle_reset_trail(&vehicles[i]);
                }
                memset(corr, 0, sizeof(corr));
                replay_sync_vehicle(&sources[0], &vehicles[0]);
            }

            // Frame markers: B = drop marker, B->L = drop + label, Shift+B = delete current
            if (IsKeyPressed(KEY_B) && vehicles[selected].active && !marker_input.active) {
                if (shift) {
                    marker_delete(&markers[selected]);
                } else if (markers[selected].count < REPLAY_MAX_MARKERS) {
                    marker_drop(&markers[selected], sources[selected].playback.position_s,
                                vehicles[selected].position, &vehicles[selected],
                                &sys_markers[selected]);
                }
            }

            // B->L chord: if L pressed within 0.5s of dropping a marker, open label input
            if (IsKeyPressed(KEY_L) && !marker_input.active
                && markers[selected].last_drop_idx >= 0) {
                double elapsed = GetTime() - markers[selected].last_drop_time;
                if (elapsed < 0.5) {
                    marker_input_begin(&marker_input, markers[selected].last_drop_idx,
                                       &sources[selected]);
                    marker_input.drone_idx = selected;
                    markers[selected].last_drop_idx = -1;
                    // Pause all drones during label input
                    for (int i = 0; i < vehicle_count; i++)
                        sources[i].playback.paused = true;
                }
            }

            // [/] cycling: per-drone or global (Ctrl)
            if (IsKeyPressed(KEY_LEFT_BRACKET) || IsKeyPressed(KEY_RIGHT_BRACKET)) {
                int dir = IsKeyPressed(KEY_LEFT_BRACKET) ? -1 : 1;
                bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
                if (ctrl) {
                    marker_cycle_result_t r = marker_cycle_global(
                        markers, sys_markers, vehicle_count, selected, dir,
                        sources, vehicles, precomp, &scene, last_pos);
                    if (r.jumped && r.drone_idx != selected)
                        selected = r.drone_idx;
                } else if (markers[selected].count > 0 || sys_markers[selected].count > 0) {
                    marker_cycle(&markers[selected], &sys_markers[selected], dir, shift,
                                 &sources[selected], &vehicles[selected],
                                 &precomp[selected], &scene, &last_pos[selected]);
                }
            }
        }

        // Update debug panel
        debug_panel_update(&dbg_panel, GetFrameTime());

        // Update camera to follow selected vehicle
        {
            Vector3 cam_target = vehicles[selected].position;
            if (hud.mode == HUD_TACTICAL)
                cam_target.y += scene.chase_distance * 0.10f;
            scene_update_camera(&scene, cam_target, vehicles[selected].rotation);
        }

        // Render ortho views to textures (before main BeginDrawing)
        // Always render when tactical HUD is active (radar uses top-down data)
        if (ortho.visible || hud.mode == HUD_TACTICAL) {
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
                // Draw frame marker spheres and system marker cubes for all drones
                for (int i = 0; i < vehicle_count; i++) {
                    if (is_replay && markers[i].count > 0) {
                        int cur = (i == selected && !sys_markers[i].selected)
                                  ? markers[i].current : -1;
                        vehicle_draw_markers(markers[i].positions, markers[i].labels,
                                             markers[i].count, cur,
                                             scene.camera.position, scene.camera,
                                             markers[i].roll, markers[i].pitch,
                                             markers[i].vert, markers[i].speed,
                                             vehicles[i].trail_speed_max, scene.theme,
                                             trail_mode, MARKER_USER,
                                             vehicles[i].color, vehicle_count > 1);
                    }
                    if (is_replay && sys_markers[i].count > 0) {
                        int cur = (i == selected && sys_markers[i].selected)
                                  ? sys_markers[i].current : -1;
                        vehicle_draw_markers(sys_markers[i].positions, sys_markers[i].labels,
                                             sys_markers[i].count, cur,
                                             scene.camera.position, scene.camera,
                                             sys_markers[i].roll, sys_markers[i].pitch,
                                             sys_markers[i].vert, sys_markers[i].speed,
                                             vehicles[i].trail_speed_max, scene.theme,
                                             trail_mode, MARKER_SYSTEM,
                                             vehicles[i].color, vehicle_count > 1);
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
            if (is_replay && show_marker_labels) {
                for (int i = 0; i < vehicle_count; i++) {
                    if (markers[i].count > 0) {
                        int cur = (i == selected && !sys_markers[i].selected)
                                  ? markers[i].current : -1;
                        vehicle_draw_marker_labels(markers[i].positions, markers[i].labels,
                                                   markers[i].count, cur,
                                                   scene.camera.position, scene.camera,
                                                   hud.font_label, hud.font_value,
                                                   markers[i].roll, markers[i].pitch,
                                                   markers[i].vert, markers[i].speed,
                                                   vehicles[i].trail_speed_max, scene.theme,
                                                   trail_mode, MARKER_USER,
                                                   vehicles[i].color, vehicle_count > 1);
                    }
                    if (sys_markers[i].count > 0) {
                        int cur = (i == selected && sys_markers[i].selected)
                                  ? sys_markers[i].current : -1;
                        vehicle_draw_marker_labels(sys_markers[i].positions, sys_markers[i].labels,
                                                   sys_markers[i].count, cur,
                                                   scene.camera.position, scene.camera,
                                                   hud.font_label, hud.font_value,
                                                   sys_markers[i].roll, sys_markers[i].pitch,
                                                   sys_markers[i].vert, sys_markers[i].speed,
                                                   vehicles[i].trail_speed_max, scene.theme,
                                                   trail_mode, MARKER_SYSTEM,
                                                   vehicles[i].color, vehicle_count > 1);
                    }
                }
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
                hud_marker_data_t all_user_md[MAX_VEHICLES] = {0};
                hud_marker_data_t all_sys_md[MAX_VEHICLES] = {0};
                for (int i = 0; i < vehicle_count; i++) {
                    all_user_md[i] = (hud_marker_data_t){
                        .times = markers[i].times,
                        .labels = markers[i].labels,
                        .roll = markers[i].roll,
                        .pitch = markers[i].pitch,
                        .vert = markers[i].vert,
                        .speed = markers[i].speed,
                        .speed_max = markers[i].speed_max,
                        .count = markers[i].count,
                        .current = markers[i].current,
                        .selected = true,
                        .color = vehicles[i].color,
                    };
                    all_sys_md[i] = (hud_marker_data_t){
                        .times = sys_markers[i].times,
                        .labels = sys_markers[i].labels,
                        .roll = sys_markers[i].roll,
                        .pitch = sys_markers[i].pitch,
                        .vert = sys_markers[i].vert,
                        .speed = sys_markers[i].speed,
                        .speed_max = markers[i].speed_max,
                        .count = sys_markers[i].count,
                        .current = sys_markers[i].current,
                        .selected = sys_markers[i].selected,
                        .color = vehicles[i].color,
                    };
                }
                if (hud.mode == HUD_CONSOLE) {
                    hud_draw(&hud, vehicles, sources, vehicle_count,
                             selected, GetScreenWidth(), GetScreenHeight(),
                             scene.theme, trail_mode,
                             all_user_md, all_sys_md, vehicle_count,
                             ghost_mode, has_tier3, has_awaiting_gps);
                } else if (hud.mode == HUD_TACTICAL) {
                    tactical_hud_draw(&hud, vehicles, sources, vehicle_count,
                                      selected, GetScreenWidth(), GetScreenHeight(),
                                      scene.theme, ghost_mode,
                                      has_tier3, has_awaiting_gps,
                                      &ortho, trail_mode, corr_mode,
                                      all_user_md, all_sys_md, vehicle_count);
                }
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

            // Ortho panel overlay (sidebar in Console mode; tactical draws its own insets)
            if (hud.mode != HUD_TACTICAL) {
                int bar_h = show_hud ? hud_bar_height(&hud, GetScreenHeight()) : 0;
                ortho_panel_draw(&ortho, GetScreenHeight(), bar_h, scene.theme, hud.font_label,
                                 vehicles, vehicle_count, selected, trail_mode,
                                 corr_mode, hud.pinned, hud.pinned_count,
                                 show_axes);
            }

            // Fullscreen ortho view label
            ortho_panel_draw_fullscreen_label(GetScreenWidth(), GetScreenHeight(),
                scene.ortho_mode, scene.ortho_span, scene.theme, hud.font_label,
                show_axes);

            // Marker label input overlay (view-mode-aware)
            if (marker_input.active) {
                marker_input_draw(&marker_input, hud.font_label, hud.font_value,
                                  scene.theme, GetScreenWidth(), GetScreenHeight());
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
    for (int i = 0; i < vehicle_count; i++)
        precomp_trail_cleanup(&precomp[i]);
    CloseWindow();

    return 0;
}
