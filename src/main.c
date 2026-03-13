#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>

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

static const Color vehicle_colors[MAX_VEHICLES] = {
    {230, 230, 230, 255}, // 0: white (default single)
    {230,  41,  55, 255}, // 1: red
    {  0, 228,  48, 255}, // 2: green
    {  0, 121, 241, 255}, // 3: blue
    {253, 249,   0, 255}, // 4: yellow
    {255,   0, 255, 255}, // 5: magenta
    {  0, 255, 255, 255}, // 6: cyan
    {255, 161,   0, 255}, // 7: orange
    {200, 122, 255, 255}, // 8: purple
    {127, 106,  79, 255}, // 9: brown
    {255, 109, 194, 255}, // 10: pink
    {  0, 182, 172, 255}, // 11: teal
    {135, 206, 235, 255}, // 12: sky blue
    {255, 203, 164, 255}, // 13: peach
    {170, 255, 128, 255}, // 14: lime
    {200, 200, 200, 255}, // 15: silver
};

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("  -udp <port>    UDP base port (default: 19410)\n");
    printf("  -n <count>     Number of vehicles (default: 1, max: %d)\n", MAX_VEHICLES);
    printf("  -mc            Multicopter model (default)\n");
    printf("  -fw            Fixed-wing model\n");
    printf("  -ts            Tailsitter model\n");
    printf("  -origin <lat> <lon> <alt>  NED origin in degrees/meters (default: PX4 SIH)\n");
    printf("  --replay <file.ulg>  Replay ULog file\n");
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
    const char *replay_file = NULL;

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
        } else if (strcmp(argv[i], "--replay") == 0 && i + 1 < argc) {
            replay_file = argv[++i];
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
    bool is_replay = (replay_file != NULL);

    if (is_replay) {
        vehicle_count = 1;  // single vehicle replay (multi-file is future scope)
        if (data_source_ulog_create(&sources[0], replay_file) != 0) {
            fprintf(stderr, "Failed to open ULog: %s\n", replay_file);
            CloseWindow();
            return 1;
        }
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
    if (is_replay) {
        // Persistent trail for replay: 36000 points (~10+ min at adaptive rate)
        vehicle_init_ex(&vehicles[0], model_idx, scene.lighting_shader, 36000);
        vehicles[0].color = vehicle_colors[0];
    } else {
        for (int i = 0; i < vehicle_count; i++) {
            vehicle_init(&vehicles[i], model_idx, scene.lighting_shader);
            vehicles[i].color = vehicle_colors[i];
        }
    }

    // For multi-vehicle or explicit origin: pre-set the NED origin on all vehicles
    if (vehicle_count > 1 || origin_specified) {
        double lat0_rad = origin_lat * (M_PI / 180.0);
        double lon0_rad = origin_lon * (M_PI / 180.0);
        for (int i = 0; i < vehicle_count; i++) {
            vehicles[i].lat0 = lat0_rad;
            vehicles[i].lon0 = lon0_rad;
            vehicles[i].alt0 = origin_alt;
            vehicles[i].origin_set = true;
        }
        printf("NED origin: lat=%.6f lon=%.6f alt=%.1f\n", origin_lat, origin_lon, origin_alt);
    }

    hud_t hud;
    hud_init(&hud);
    hud.is_replay = is_replay;

    debug_panel_t dbg_panel;
    debug_panel_init(&dbg_panel);

    ortho_panel_t ortho;
    ortho_panel_init(&ortho);

    int selected = 0;
    bool was_connected[MAX_VEHICLES];
    memset(was_connected, 0, sizeof(was_connected));
    Vector3 last_pos[MAX_VEHICLES];
    memset(last_pos, 0, sizeof(last_pos));
    bool show_hud = true;
    int trail_mode = 1;              // 0=off, 1=directional trail, 2=speed ribbon
    bool show_ground_track = false;  // ground projection off by default
    bool classic_colors = false;     // K key: toggle classic (red/blue) vs modern (yellow/purple)

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

    // Helper: sync vehicle state after a replay seek.
    #define REPLAY_SYNC_AFTER_SEEK(ctx, src, veh) do { \
        (src)->state = (ctx)->state; \
        (src)->home = (ctx)->home; \
        (src)->playback.position_s = (float)(ctx)->wall_accum; \
        uint64_t _range = (ctx)->parser.end_timestamp - (ctx)->parser.start_timestamp; \
        if (_range > 0) (src)->playback.progress = (src)->playback.position_s / ((float)((double)_range / 1e6)); \
        (veh)->current_time = (src)->playback.position_s; \
        vehicle_update((veh), &(src)->state, &(src)->home); \
        vehicle_truncate_trail((veh), (src)->playback.position_s); \
    } while(0)

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
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
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
            REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
            sys_markers_resolved = true;
        }

        // Check if any source is connected (for HUD)
        bool any_connected = false;
        for (int i = 0; i < vehicle_count; i++) {
            if (sources[i].connected) { any_connected = true; break; }
        }

        // Update HUD sim time from selected vehicle
        hud_update(&hud, sources[selected].state.time_usec,
                   sources[selected].connected, GetFrameTime());

        // Handle input (blocked during marker label entry)
        if (!marker_input_active) {
        scene_handle_input(&scene);

        // Help overlay toggle (? key = Shift+/)
        if (IsKeyPressed(KEY_SLASH) && (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))) {
            hud.show_help = !hud.show_help;
        }

        // Toggle HUD visibility
        if (IsKeyPressed(KEY_H)) {
            show_hud = !show_hud;
        }

        // Cycle trail mode: off → trail → speed ribbon
        if (IsKeyPressed(KEY_T)) {
            trail_mode = (trail_mode + 1) % 3;
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
            if (IsKeyPressed(KEY_LEFT_BRACKET) && !is_replay) {
                for (int j = 1; j <= vehicle_count; j++) {
                    int prev = (selected - j + vehicle_count) % vehicle_count;
                    if (sources[prev].connected) { selected = prev; break; }
                }
                hud.pinned_count = 0;
                memset(hud.pinned, -1, sizeof(hud.pinned));
            }
            if (IsKeyPressed(KEY_RIGHT_BRACKET) && !is_replay) {
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
                    } else if (!IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)) {
                        // Plain number: switch primary, clear pins
                        selected = idx;
                        hud.pinned_count = 0;
                        memset(hud.pinned, -1, sizeof(hud.pinned));
                    }
                    // Note: Ctrl+number is reserved for hidden mode
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
                    REPLAY_SYNC_AFTER_SEEK(rctx, &sources[0], &vehicles[0]);
                }
                current_marker = marker_input_target;
                marker_input_active = false;
            }
            if (IsKeyPressed(KEY_ESCAPE)) {
                if (marker_input_target >= 0 && marker_input_target < marker_count) {
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, marker_times[marker_input_target]);
                    REPLAY_SYNC_AFTER_SEEK(rctx, &sources[0], &vehicles[0]);
                }
                current_marker = marker_input_target;
                marker_input_active = false;
            }
        }

        // Replay playback controls
        if (is_replay && !marker_input_active) {
            bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
            bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
            if (IsKeyPressed(KEY_SPACE)) {
                if (!sources[0].connected) {
                    // Replay ended — restart from beginning
                    ulog_replay_ctx_t *rctx = (ulog_replay_ctx_t *)sources[0].impl;
                    ulog_replay_seek(rctx, 0.0f);
                    sources[0].connected = true;
                    sources[0].playback.paused = false;
                    vehicle_reset_trail(&vehicles[0]);
                    vehicles[0].origin_set = false;
                    vehicles[0].origin_wait_count = 0;
                    REPLAY_SYNC_AFTER_SEEK(rctx, &sources[0], &vehicles[0]);
                } else {
                    sources[0].playback.paused = !sources[0].playback.paused;
                }
            }
            if (IsKeyPressed(KEY_L) && shift) {
                sources[0].playback.looping = !sources[0].playback.looping;
            }
            if (IsKeyPressed(KEY_L) && !shift && (last_marker_drop_idx < 0 || GetTime() - last_marker_drop_time >= 0.5)) {
                show_marker_labels = !show_marker_labels;
            }
            if (IsKeyPressed(KEY_I)) {
                sources[0].playback.interpolation = !sources[0].playback.interpolation;
                printf("Interpolation: %s\n", sources[0].playback.interpolation ? "ON" : "OFF");
            }
            if (IsKeyPressed(KEY_EQUAL)) {
                float *spd = &sources[0].playback.speed;
                if (*spd < 0.5f) *spd = 0.5f;
                else if (*spd < 1.0f) *spd = 1.0f;
                else if (*spd < 2.0f) *spd = 2.0f;
                else if (*spd < 4.0f) *spd = 4.0f;
                else if (*spd < 8.0f) *spd = 8.0f;
                else *spd = 16.0f;
            }
            if (IsKeyPressed(KEY_MINUS)) {
                float *spd = &sources[0].playback.speed;
                if (*spd > 8.0f) *spd = 8.0f;
                else if (*spd > 4.0f) *spd = 4.0f;
                else if (*spd > 2.0f) *spd = 2.0f;
                else if (*spd > 1.0f) *spd = 1.0f;
                else if (*spd > 0.5f) *spd = 0.5f;
                else *spd = 0.25f;
            }
            if (IsKeyPressed(KEY_R)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                ulog_replay_seek(ctx, 0.0f);
                sources[0].connected = true;
                sources[0].playback.paused = false;
                vehicle_reset_trail(&vehicles[0]);
                vehicles[0].origin_set = false;
                vehicles[0].origin_wait_count = 0;
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
                marker_count = 0;
                current_marker = -1;
            }

            // Timeline scrubbing: 3 levels of granularity
            // Shift+Arrow = single frame step (~20ms), Ctrl+Shift = 1s, plain = 5s
            if (IsKeyPressed(KEY_RIGHT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                ulog_replay_seek(ctx, sources[0].playback.position_s + step);
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
            }
            if (IsKeyPressed(KEY_LEFT)) {
                ulog_replay_ctx_t *ctx = (ulog_replay_ctx_t *)sources[0].impl;
                float step;
                if (shift && ctrl) step = 1.0f;
                else if (shift) { step = 0.02f; sources[0].playback.paused = true; }
                else step = 5.0f;
                float target = sources[0].playback.position_s - step;
                if (target < 0.0f) target = 0.0f;
                ulog_replay_seek(ctx, target);
                REPLAY_SYNC_AFTER_SEEK(ctx, &sources[0], &vehicles[0]);
            }

            // Frame markers: B = drop marker, B→L = drop + label, Shift+B = delete current
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

            // B→L chord: if L pressed within 0.5s of dropping a marker, open label input
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
            ortho_panel_render(&ortho, &scene, vehicles, vehicle_count,
                               selected, scene.view_mode, trail_mode);
        }

        // Render
        BeginDrawing();

            // Sky background
            scene_draw_sky(&scene);

            BeginMode3D(scene.camera);
                scene_draw(&scene);
                for (int i = 0; i < vehicle_count; i++) {
                    if (vehicles[i].active || vehicle_count == 1) {
                        vehicle_draw(&vehicles[i], scene.view_mode, i == selected,
                                     trail_mode, show_ground_track, scene.camera.position,
                                     classic_colors);
                    }
                }
                // Draw frame marker spheres during replay
                if (is_replay && marker_count > 0) {
                    vehicle_draw_markers(marker_positions, marker_labels, marker_count,
                                         sys_marker_selected ? -1 : current_marker,
                                         scene.camera.position, scene.camera,
                                         marker_roll, marker_pitch, marker_vert, marker_speed,
                                         vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
                }
                // Draw system marker cubes during replay
                if (is_replay && sys_marker_count > 0) {
                    vehicle_draw_sys_markers(sys_marker_positions, sys_marker_labels, sys_marker_count,
                                             sys_marker_selected ? current_sys_marker : -1,
                                             scene.camera.position,
                                             sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed,
                                             vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
                }
            EndMode3D();

            // Ortho ground fill (2D overlay)
            scene_draw_ortho_ground(&scene, GetScreenWidth(), GetScreenHeight());

            // Marker labels (2D billboarded text, after EndMode3D)
            if (is_replay && marker_count > 0 && show_marker_labels) {
                vehicle_draw_marker_labels(marker_positions, marker_labels, marker_count,
                                           sys_marker_selected ? -1 : current_marker,
                                           scene.camera.position, scene.camera,
                                           hud.font_label, hud.font_value,
                                           marker_roll, marker_pitch, marker_vert, marker_speed,
                                           vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
            }
            // System marker labels
            if (is_replay && sys_marker_count > 0 && show_marker_labels) {
                vehicle_draw_sys_marker_labels(sys_marker_positions, sys_marker_labels, sys_marker_count,
                                               sys_marker_selected ? current_sys_marker : -1,
                                               scene.camera.position, scene.camera,
                                               hud.font_label, hud.font_value,
                                               sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed,
                                               vehicles[0].trail_speed_max, scene.view_mode, trail_mode);
            }

            // HUD
            if (show_hud) {
                hud_draw(&hud, vehicles, sources, vehicle_count,
                         selected, GetScreenWidth(), GetScreenHeight(),
                         scene.view_mode, trail_mode,
                         marker_times, marker_labels, marker_count,
                         sys_marker_selected ? -1 : current_marker,
                         marker_roll, marker_pitch, marker_vert, marker_speed,
                         vehicles[0].trail_speed_max,
                         sys_marker_times, sys_marker_labels,
                         sys_marker_count, current_sys_marker, sys_marker_selected,
                         sys_marker_roll, sys_marker_pitch, sys_marker_vert, sys_marker_speed);
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
                                 vehicle_count, active_count, total_trail);
            }

            // Ortho panel overlay
            int bar_h = show_hud ? hud_bar_height(&hud, GetScreenHeight()) : 0;
            ortho_panel_draw(&ortho, GetScreenHeight(), bar_h, scene.view_mode, hud.font_label);

            // Fullscreen ortho view label
            ortho_panel_draw_fullscreen_label(GetScreenWidth(), GetScreenHeight(),
                scene.ortho_mode, scene.ortho_span, scene.view_mode, hud.font_label);

            // Marker label input overlay (view-mode-aware)
            if (marker_input_active) {
                int sw = GetScreenWidth(), sh = GetScreenHeight();
                float s = powf(sh / 720.0f, 0.7f);

                Color scrim_col, box_bg, box_border, prompt_col, hint_col;
                Color field_bg, field_border, text_col, cursor_col;
                if (scene.view_mode == VIEW_SNOW) {
                    scrim_col    = (Color){255, 255, 255, 140};
                    box_bg       = (Color){248, 248, 250, 240};
                    box_border   = (Color){15, 15, 20, 120};
                    prompt_col   = (Color){60, 65, 75, 255};
                    hint_col     = (Color){120, 125, 135, 200};
                    field_bg     = (Color){235, 236, 240, 255};
                    field_border = (Color){15, 15, 20, 80};
                    text_col     = (Color){10, 10, 15, 255};
                    cursor_col   = (Color){15, 15, 20, 220};
                } else if (scene.view_mode == VIEW_1988) {
                    scrim_col    = (Color){5, 0, 15, 160};
                    box_bg       = (Color){5, 5, 16, 240};
                    box_border   = (Color){255, 20, 100, 140};
                    prompt_col   = (Color){255, 20, 100, 200};
                    hint_col     = (Color){180, 60, 120, 160};
                    field_bg     = (Color){12, 8, 24, 255};
                    field_border = (Color){255, 20, 100, 100};
                    text_col     = (Color){255, 220, 60, 255};
                    cursor_col   = (Color){255, 20, 100, 220};
                } else if (scene.view_mode == VIEW_REZ) {
                    scrim_col    = (Color){0, 0, 0, 150};
                    box_bg       = (Color){8, 8, 12, 235};
                    box_border   = (Color){0, 204, 218, 100};
                    prompt_col   = (Color){0, 204, 218, 200};
                    hint_col     = (Color){0, 140, 150, 160};
                    field_bg     = (Color){4, 4, 8, 255};
                    field_border = (Color){0, 204, 218, 80};
                    text_col     = (Color){200, 208, 218, 255};
                    cursor_col   = (Color){0, 204, 218, 220};
                } else {
                    scrim_col    = (Color){0, 0, 0, 140};
                    box_bg       = (Color){10, 14, 20, 235};
                    box_border   = (Color){0, 180, 204, 100};
                    prompt_col   = (Color){140, 150, 170, 255};
                    hint_col     = (Color){90, 95, 110, 200};
                    field_bg     = (Color){6, 8, 12, 255};
                    field_border = (Color){50, 55, 70, 180};
                    text_col     = WHITE;
                    cursor_col   = (Color){0, 255, 255, 220};
                }

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
