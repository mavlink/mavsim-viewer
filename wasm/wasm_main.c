// Hawkeye WASM entry point. Line-ordered port of src/main.c with the following
// platform adaptations — every other call in main.c has a matching line here
// in the same order.
//
//   native                                 WASM
//   -----------------------------------    --------------------------------------
//   argv CLI parsing                       fixed defaults (replay-only)
//   while(!WindowShouldClose()) loop       emscripten_set_main_loop_arg(frame)
//   data_source_ulog_create(path)          wasm_replay_ctx_t + data_source shim
//   IsFileDropped() for .mvt themes        hawkeye_add_theme_bytes() JS API
//   draw_prompt_dialog (blocking loop)     in-canvas dialog state machine (Phase 2)
//   shader GLSL #version 330               SetLoadFileTextCallback patch to GLES
//
// See WASM_PORT_FAILURES.md for the history behind these adaptations.

#include <math.h>
#include <malloc.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <emscripten.h>
#include <emscripten/heap.h>
#include <emscripten/html5.h>

#include "raylib.h"
#include "raymath.h"

#include "wasm_api.h"
#include "wasm_dialog.h"
#include "wasm/wasm_replay.h"

#include "data_source.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"
#include "tactical_hud.h"
#include "ui_logic.h"
#include "debug_panel.h"
#include "ortho_panel.h"
#include "theme.h"
#include "asset_path.h"
#include "replay_trail.h"
#include "replay_markers.h"
#include "replay_conflict.h"
#include "ui_marker_input.h"
#include "correlation.h"

static void log_heap(const char *label) {
    struct mallinfo mi = mallinfo();
    printf("[heap] %-24s live=%5zu KB  arena=%5zu KB  free=%5zu KB\n",
           label,
           (size_t)mi.uordblks / 1024,
           (size_t)mi.arena / 1024,
           (size_t)mi.fordblks / 1024);
}

#define MAX_VEHICLES 16
#define EARTH_RADIUS 6371000.0
#define CHORD_TIMEOUT_S 0.3

// ---------------------------------------------------------------------------
// GLSL 330 → GLES 300 es shader patcher. SetLoadFileTextCallback routes every
// LoadFileText() call through here so Raylib's bundled #version 330 shaders
// compile under WebGL2.
// ---------------------------------------------------------------------------

static char *read_whole_file(const char *path, int *out_size) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return NULL;
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (sz < 0) { fclose(fp); return NULL; }
    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(fp); return NULL; }
    size_t n = fread(buf, 1, (size_t)sz, fp);
    fclose(fp);
    buf[n] = '\0';
    if (out_size) *out_size = (int)n;
    return buf;
}

static char *patch_shader_text(char *src) {
    const char *needle = "#version 330";
    if (strncmp(src, needle, strlen(needle)) != 0) return src;
    const char *eol = strchr(src, '\n');
    if (!eol) return src;
    const char *header = "#version 300 es\nprecision highp float;\nprecision highp int;\n";
    size_t header_len = strlen(header);
    size_t tail_len = strlen(eol + 1);
    char *out = (char *)malloc(header_len + tail_len + 1);
    if (!out) return src;
    memcpy(out, header, header_len);
    memcpy(out + header_len, eol + 1, tail_len);
    out[header_len + tail_len] = '\0';
    free(src);
    return out;
}

static char *hawkeye_load_file_text(const char *filename) {
    int size = 0;
    char *src = read_whole_file(filename, &size);
    if (!src) return NULL;
    return patch_shader_text(src);
}

// ---------------------------------------------------------------------------
// Edge indicator chevrons. Ported verbatim from main.c:62-145 with the single
// call-site substitution of g.vehicles / g.selected etc.
// ---------------------------------------------------------------------------

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

        char num[4];
        snprintf(num, sizeof(num), "%d", i + 1);
        float lfs = 18.0f * ei_scale;
        Vector2 tw = MeasureTextEx(font, num, lfs, 0.5f);
        float lx = ex - cosf(angle) * (sz * 0.3f) - tw.x / 2;
        float ly = ey - sinf(angle) * (sz * 0.3f) - tw.y / 2;
        DrawTextEx(font, num, (Vector2){ lx, ly }, lfs, 0.5f, col);
    }
}

// Thin wrapper mirroring main.c:54-58
static void apply_vehicle_selection_hud(hud_t *hud, int idx, bool pin,
                                        int *selected, int vehicle_count) {
    apply_vehicle_selection(hud->pinned, &hud->pinned_count,
                            idx, pin, selected, vehicle_count);
}

// ---------------------------------------------------------------------------
// Global state. Mirrors the local variables declared in main()'s body, scoped
// to module lifetime so the frame callback can access them.
// ---------------------------------------------------------------------------

static const data_source_ops_t wasm_ds_ops;  // forward decl

static struct hawkeye_state {
    bool initialized;
    bool log_loaded;
    int  width, height;
    char canvas_id[64];

    // Rendering spine
    scene_t       scene;
    vehicle_t     vehicles[MAX_VEHICLES];
    data_source_t sources[MAX_VEHICLES];
    int           vehicle_count;

    // HUD
    hud_t         hud;
    debug_panel_t dbg_panel;
    ortho_panel_t ortho;
    bool          show_hud;
    float         saved_chase_distance;
    float         tactical_chase_target;

    // Selection + chord
    int           selected;
    int           prev_selected;
    int           chord_first;
    double        chord_time;
    bool          chord_shift;

    // Per-vehicle tracking
    bool          was_connected[MAX_VEHICLES];
    Vector3       last_pos[MAX_VEHICLES];
    float         prev_playback_pos[MAX_VEHICLES];
    bool          insufficient_data[MAX_VEHICLES];
    int           vehicle_tier[MAX_VEHICLES];
    int           insufficient_check_frames;
    bool          insufficient_toasted;
    int           last_selected_for_insuf;

    // Display toggles
    int           trail_mode;
    bool          show_ground_track;
    bool          classic_colors;
    bool          show_edge_indicators;
    int           corr_mode;
    bool          show_corr_labels;
    bool          show_axes;
    bool          show_marker_labels;

    // Multi-drone state
    bool          ghost_mode;
    bool          ghost_mode_grid;
    bool          conflict_detected;
    bool          conflict_far;
    bool          has_tier3;
    double        ref_lat_rad, ref_lon_rad, min_alt;
    bool          takeoff_aligned;
    corr_state_t  corr[MAX_VEHICLES];

    // Markers
    user_markers_t  markers[MAX_VEHICLES];
    sys_markers_t   sys_markers[MAX_VEHICLES];
    precomp_trail_t precomp[MAX_VEHICLES];
    marker_input_t  marker_input;

    // Replay contexts (one per loaded drone)
    wasm_replay_ctx_t replays[MAX_VEHICLES];

    // Non-blocking prompt dialog (replaces native's draw_prompt_dialog)
    wasm_dialog_t dialog;
    int           dialog_context;  // 0 = initial conflict, 1 = P-key mode switcher
} g;

// ---------------------------------------------------------------------------
// data_source shim. The WASM build substitutes this vtable for native's
// data_source_ulog implementation. poll() drives wasm_replay_advance, which
// pulls events from the pre-extracted flat arrays. The static fields
// (mode_changes, takeoff_*, statustext ring pointer, vehicle_type) are
// populated once at finalize time by ds_init_static and never rewritten,
// so user-toggleable fields (speed, looping, paused, interpolation,
// time_offset_s) survive frame-to-frame.
// ---------------------------------------------------------------------------

static void ds_init_static(data_source_t *ds, wasm_replay_ctx_t *r) {
    ds->ops     = &wasm_ds_ops;
    ds->impl    = r;
    ds->state   = r->state;
    ds->home    = r->home;
    // Match native data_source_ulog_create:97 — connected is true as soon as
    // the replay is opened, independent of state.valid. The main loop uses
    // connected to unlock the grid_offset placeholder draw (main.c:531-534),
    // so tying connected to state.valid would hide the drone until takeoff.
    ds->connected   = true;
    ds->mav_type    = r->vehicle_type;
    ds->ref_rejected = r->ref_rejected;

    ds->playback.speed          = 1.0f;
    ds->playback.looping        = true;
    ds->playback.interpolation  = true;
    ds->playback.paused         = false;
    ds->playback.progress       = 0.0f;
    ds->playback.position_s     = 0.0f;
    ds->playback.time_offset_s  = (float)r->time_offset_s;
    ds->playback.current_nav_state = r->current_nav_state;
    ds->playback.mode_changes   = (const playback_mode_change_t *)r->mode_changes;
    ds->playback.mode_change_count = r->mode_change_count;
    ds->playback.takeoff_conf       = r->takeoff_conf;
    ds->playback.takeoff_time_s     = r->takeoff_time_s;
    ds->playback.takeoff_detected   = r->takeoff_detected;
    ds->playback.home_from_topic    = r->home_from_topic;
    ds->playback.statustext     = &r->statustext;
    ds->playback.correlation    = NAN;
    ds->playback.rmse           = NAN;

    uint64_t range = wasm_replay_duration_us(r);
    ds->playback.duration_s = range > 0 ? (float)((double)range / 1e6) : 0.0f;
}

static void wasm_ds_poll(data_source_t *ds, float dt) {
    wasm_replay_ctx_t *r = (wasm_replay_ctx_t *)ds->impl;
    if (!r) return;

    if (!ds->playback.paused) {
        bool still_playing = wasm_replay_advance(r, dt, ds->playback.speed,
                                                  ds->playback.looping,
                                                  ds->playback.interpolation);
        // Native semantics (data_source_ulog.c:15): connected stays true as
        // long as more events remain or the replay is looping. State validity
        // is tracked separately on state.valid.
        ds->connected = still_playing || ds->playback.looping;
    }

    ds->state = r->state;
    ds->home  = r->home;
    ds->playback.current_nav_state = r->current_nav_state;

    uint64_t range = wasm_replay_duration_us(r);
    if (range > 0) {
        ds->playback.duration_s = (float)((double)range / 1e6);
        ds->playback.position_s = (float)(r->wall_accum - r->time_offset_s);
        if (ds->playback.position_s < 0.0f) ds->playback.position_s = 0.0f;
        if (ds->playback.position_s > ds->playback.duration_s)
            ds->playback.position_s = ds->playback.duration_s;
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
    }
}

static void wasm_ds_seek(data_source_t *ds, float target_s) {
    wasm_replay_ctx_t *r = (wasm_replay_ctx_t *)ds->impl;
    if (!r) return;
    wasm_replay_seek(r, target_s);
    ds->state = r->state;
    ds->home  = r->home;
    ds->playback.current_nav_state = r->current_nav_state;
    uint64_t range = wasm_replay_duration_us(r);
    if (range > 0) {
        ds->playback.position_s = (float)(r->wall_accum - r->time_offset_s);
        if (ds->playback.position_s < 0.0f) ds->playback.position_s = 0.0f;
        ds->playback.progress = ds->playback.position_s / ds->playback.duration_s;
    }
}

static void wasm_ds_set_time_offset(data_source_t *ds, double offset_s) {
    wasm_replay_ctx_t *r = (wasm_replay_ctx_t *)ds->impl;
    if (r) r->time_offset_s = offset_s;
    ds->playback.time_offset_s = (float)offset_s;
}

static void wasm_ds_close(data_source_t *ds) { (void)ds; }

static const data_source_ops_t wasm_ds_ops = {
    .poll            = wasm_ds_poll,
    .seek            = wasm_ds_seek,
    .set_time_offset = wasm_ds_set_time_offset,
    .close           = wasm_ds_close,
};

// ---------------------------------------------------------------------------
// Replay mode helpers. apply_replay_mode consolidates the per-mode vehicle
// reconfiguration that main.c spreads across the initial conflict block
// (339-409) and the P-key handler (682-792).
//
// rebase_to_home matters because native behaves differently between the two
// paths:
//   • Initial (main.c:362-380): leaves origin_set=false and lets vehicle_update
//     set origin from the first received state. This works because the drone
//     hasn't been playing yet — first state == takeoff position.
//   • P key   (main.c:705-783): explicitly forces origin = home so the drone
//     jumps back to home in the new coordinate system rather than continuing
//     from its last position.
// Using rebase_to_home=true on the initial path shifts drones away from
// first-state-position by whatever delta exists between that point and
// home.lat/lon/alt — which in multi-country ULogs can be large.
//
// Modes:
//   1 = formation (shared origin, full alpha)
//   2 = ghost    (own origin, non-primary alpha 0.35)
//   3 = grid     (own origin + X offset per drone)
//   4 = cancel   (P-key cancel: own origin, full alpha, no grid offset)
// ---------------------------------------------------------------------------

static void apply_replay_mode(int mode, bool rebase_to_home) {
    int count = g.vehicle_count;
    if (count < 1) return;

    for (int i = 0; i < count; i++) {
        g.vehicles[i].grid_offset = (Vector3){0, 0, 0};
        vehicle_set_ghost_alpha(&g.vehicles[i], 1.0f);
        g.vehicles[i].origin_wait_count = 0;
        vehicle_reset_trail(&g.vehicles[i]);
        if (rebase_to_home) g.vehicles[i].origin_set = false;
    }

    g.ghost_mode      = (mode == 2);
    g.ghost_mode_grid = (mode == 3);

    if (mode == 1 && count > 1) {
        // Formation: shared NED origin. Native always sets this explicitly
        // (main.c:372-378) — no first-state fallback — so we do the same.
        for (int i = 0; i < count; i++) {
            if (g.sources[i].home.valid) {
                g.vehicles[i].lat0 = g.ref_lat_rad;
                g.vehicles[i].lon0 = g.ref_lon_rad;
                g.vehicles[i].alt0 = g.min_alt;
                g.vehicles[i].origin_set = true;
            }
        }
    } else if (mode == 2 || mode == 3 || mode == 4) {
        if (rebase_to_home) {
            for (int i = 0; i < count; i++) {
                if (g.sources[i].home.valid) {
                    g.vehicles[i].lat0 = g.sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                    g.vehicles[i].lon0 = g.sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                    g.vehicles[i].alt0 = g.sources[i].home.alt * 1e-3;
                    g.vehicles[i].origin_set = true;
                }
            }
        }
        // else: vehicle_update will set origin from first state (native initial
        // path behavior, main.c:362-380 — note the "Don't set origin_set"
        // comment there).

        if (mode == 2 && count > 1) {
            vehicle_set_ghost_alpha(&g.vehicles[0], 1.0f);
            for (int i = 1; i < count; i++)
                vehicle_set_ghost_alpha(&g.vehicles[i], 0.35f);
        }
        if (mode == 3) {
            for (int i = 1; i < count; i++)
                g.vehicles[i].grid_offset = (Vector3){ i * 5.0f, 0.0f, 0.0f };
        }
    }

    memset(g.corr, 0, sizeof(g.corr));
    for (int i = 0; i < count; i++) {
        g.sources[i].playback.correlation = NAN;
        g.sources[i].playback.rmse = NAN;
    }
    for (int i = 0; i < count; i++)
        g.sys_markers[i].resolved = false;
}

// Reset to the pre-load single-placeholder state (used by initial conflict
// dialog's "Cancel & reupload" choice).
static void hawkeye_unload_internal(void) {
    if (g.log_loaded) {
        for (int i = 0; i < g.vehicle_count; i++) {
            wasm_replay_close(&g.replays[i]);
            vehicle_cleanup(&g.vehicles[i]);
            precomp_trail_cleanup(&g.precomp[i]);
        }
    }
    g.log_loaded = false;
    g.vehicle_count = 1;
    g.selected = 0;
    g.prev_selected = 0;

    vehicle_init_ex(&g.vehicles[0], MODEL_QUADROTOR, g.scene.lighting_shader, 36000);
    g.vehicles[0].color = g.scene.theme->drone_palette[0];
    precomp_trail_init(&g.precomp[0]);

    memset(&g.sources[0], 0, sizeof(g.sources[0]));
    g.sources[0].ops = &wasm_ds_ops;
    g.sources[0].playback.speed = 1.0f;
    g.sources[0].playback.looping = true;
    g.sources[0].playback.interpolation = true;

    memset(g.markers, 0, sizeof(g.markers));
    memset(g.sys_markers, 0, sizeof(g.sys_markers));
    g.markers[0].current = -1;
    g.markers[0].last_drop_idx = -1;
    g.sys_markers[0].current = -1;

    g.conflict_detected = false;
    g.conflict_far = false;
    g.has_tier3 = false;
    g.ghost_mode = false;
    g.ghost_mode_grid = false;
    g.takeoff_aligned = false;
}

// ---------------------------------------------------------------------------
// Frame callback. Line-ordered port of main.c:468-1428.
// Cross-references tagged "(main.c:NNN)" point at the native line being ported.
// ---------------------------------------------------------------------------

static void frame(void *arg) {
    (void)arg;
    if (!g.initialized) return;

    float frame_dt = GetFrameTime();

    if (g.vehicle_count <= 0) return;  // main.c:471

    // ── Poll all data sources + update vehicles (main.c:473-544) ────────────
    // Freeze the entire update pipeline while the modal dialog is open.
    // Native achieves this naturally because draw_prompt_dialog blocks in its
    // own render loop; we do it explicitly here. Without this, playback keeps
    // advancing behind the dialog and the drones fly off to wherever they
    // happened to be when the user finally clicked a mode, making the chosen
    // grid/ghost offset look visually wrong.
    if (g.log_loaded && !g.dialog.active) {
        for (int i = 0; i < g.vehicle_count; i++) {
            data_source_poll(&g.sources[i], frame_dt);                  // main.c:475

            if (g.sources[i].playback.statustext)                        // main.c:478
                hud_feed_statustext(&g.hud, g.sources[i].playback.statustext, i);

            if (!g.sources[i].playback.paused) {                         // main.c:482
                float cur_pos = g.sources[i].playback.position_s;
                float prev_pos = g.prev_playback_pos[i];
                for (int m = 0; m < g.markers[i].count; m++) {           // main.c:486
                    if (g.markers[i].times[m] > prev_pos && g.markers[i].times[m] <= cur_pos) {
                        annunc_trigger_tab_fade(&g.hud.annunciators, i);
                        annunc_trigger_radar_wave(&g.hud.annunciators, i);
                        if (i != g.selected) {
                            for (int p = 0; p < g.hud.pinned_count; p++)
                                if (g.hud.pinned[p] == i)
                                    annunc_trigger_ring_bounce(&g.hud.annunciators, i);
                        }
                        break;
                    }
                }
                for (int m = 0; m < g.sys_markers[i].count; m++) {       // main.c:499
                    if (g.sys_markers[i].times[m] > prev_pos && g.sys_markers[i].times[m] <= cur_pos) {
                        annunc_trigger_tab_fade(&g.hud.annunciators, i);
                        annunc_trigger_radar_wave(&g.hud.annunciators, i);
                        if (i != g.selected) {
                            for (int p = 0; p < g.hud.pinned_count; p++)
                                if (g.hud.pinned[p] == i)
                                    annunc_trigger_ring_bounce(&g.hud.annunciators, i);
                        }
                        break;
                    }
                }
                g.prev_playback_pos[i] = cur_pos;
            }

            if (g.sources[i].connected && !g.was_connected[i]) {         // main.c:515
                vehicle_reset_trail(&g.vehicles[i]);
                if (g.sources[i].mav_type != 0)
                    vehicle_set_type(&g.vehicles[i], g.sources[i].mav_type);
            }
            g.was_connected[i] = g.sources[i].connected;

            g.vehicles[i].current_time = g.sources[i].playback.position_s;   // main.c:526
            vehicle_update(&g.vehicles[i], &g.sources[i].state, &g.sources[i].home);
            g.vehicles[i].sysid = g.sources[i].sysid;

            if (g.sources[i].connected && !g.vehicles[i].active) {       // main.c:531
                g.vehicles[i].active = true;
                g.vehicles[i].position = g.vehicles[i].grid_offset;
            }

            if (g.vehicles[i].active && g.vehicles[i].trail_count > 0) { // main.c:537
                Vector3 delta = Vector3Subtract(g.vehicles[i].position, g.last_pos[i]);
                if (Vector3Length(delta) > 50.0f)
                    vehicle_reset_trail(&g.vehicles[i]);
            }
            g.last_pos[i] = g.vehicles[i].position;
        }

        // Lazy-resolve system markers (main.c:547-553). Precomp trail build walks
        // the replay via poll() — save/restore the cursor state so the main
        // playback cursor doesn't jump when precomp consumes the whole timeline.
        for (int i = 0; i < g.vehicle_count; i++) {
            if (!g.sys_markers[i].resolved && g.sys_markers[i].count > 0
                && g.vehicles[i].origin_set) {
                wasm_replay_ctx_t saved = g.replays[i];
                replay_resolve_and_build_trail(&g.sys_markers[i], &g.precomp[i],
                                               &g.sources[i], &g.vehicles[i]);
                g.replays[i] = saved;
            }
        }

        // Insufficient data detection (main.c:556-569)
        if (!g.insufficient_toasted) {
            g.insufficient_check_frames++;
            if (g.insufficient_check_frames > 120) {
                for (int i = 0; i < g.vehicle_count; i++) {
                    if (g.sources[i].connected && !g.sources[i].state.valid) {
                        g.insufficient_data[i] = true;
                        char msg[64];
                        snprintf(msg, sizeof(msg), "DRONE %d: INSUFFICIENT DATA", i + 1);
                        hud_toast_color(&g.hud, msg, 4.0f, g.vehicles[i].color);
                    }
                }
                g.insufficient_toasted = true;
            }
        }

        // Correlation accumulators (main.c:575-619)
        if (g.vehicle_count > 1) {
            if (g.selected != g.prev_selected) {
                memset(g.corr, 0, sizeof(g.corr));
                for (int i = 0; i < g.vehicle_count; i++) {
                    g.sources[i].playback.correlation = NAN;
                    g.sources[i].playback.rmse = NAN;
                }
                g.prev_selected = g.selected;
            }

            bool playing = g.sources[g.selected].connected && !g.sources[g.selected].playback.paused;
            if (playing && g.vehicles[g.selected].origin_set) {
                const vehicle_t *ref = &g.vehicles[g.selected];
                double rx[CORR_CHANNELS] = { ref->position.z, ref->position.x, ref->position.y };
                for (int i = 0; i < g.vehicle_count; i++) {
                    if (i == g.selected || !g.vehicles[i].origin_set) continue;
                    const vehicle_t *v = &g.vehicles[i];
                    double vx[CORR_CHANNELS] = { v->position.z, v->position.x, v->position.y };
                    for (int c = 0; c < CORR_CHANNELS; c++) {
                        g.corr[i].ch[c].sum_x  += rx[c];
                        g.corr[i].ch[c].sum_y  += vx[c];
                        g.corr[i].ch[c].sum_xy += rx[c] * vx[c];
                        g.corr[i].ch[c].sum_x2 += rx[c] * rx[c];
                        g.corr[i].ch[c].sum_y2 += vx[c] * vx[c];
                    }
                    double dx = (v->position.x - v->grid_offset.x) - (ref->position.x - ref->grid_offset.x);
                    double dy = (v->position.y - v->grid_offset.y) - (ref->position.y - ref->grid_offset.y);
                    double dz = (v->position.z - v->grid_offset.z) - (ref->position.z - ref->grid_offset.z);
                    g.corr[i].sum_sq_dist += dx*dx + dy*dy + dz*dz;
                    g.corr[i].n++;
                    g.sources[i].playback.correlation = corr_compute(&g.corr[i]);
                    g.sources[i].playback.rmse = (g.corr[i].n >= CORR_MIN_SAMPLES)
                        ? (float)sqrt(g.corr[i].sum_sq_dist / g.corr[i].n) : NAN;
                }
                g.sources[g.selected].playback.correlation = 1.0f;
                g.sources[g.selected].playback.rmse = 0.0f;
            }
        }
    }

    bool any_connected = false;                                          // main.c:622
    for (int i = 0; i < g.vehicle_count; i++) {
        if (g.sources[i].connected) { any_connected = true; break; }
    }
    (void)any_connected;

    hud_update(&g.hud, g.sources[g.selected].state.time_usec,             // main.c:628
               g.sources[g.selected].connected, frame_dt);

    // File-drop theme handling (main.c:632-671) — on WASM, JS calls
    // hawkeye_add_theme_bytes() directly. See that function below.

    // ── Input handling (main.c:674-1171) ────────────────────────────────────
    // Dialog, when active, captures all keyboard input. Mirrors native's
    // blocking draw_prompt_dialog semantics without blocking the frame loop.
    if (!g.marker_input.active && !g.dialog.active) {
        scene_handle_input(&g.scene);

        // P key mode switcher (main.c:682-792)
        if (IsKeyPressed(KEY_P) && g.log_loaded && g.vehicle_count > 1) {
            const char *p_grid_label = g.conflict_far ? "Narrow grid" : "Grid offset";
            const char *pl[3];
            char p_title[32];
            char p_subtitle[64];
            if (g.conflict_detected) {
                snprintf(p_title, sizeof(p_title), "POSITION CONFLICT");
                pl[0] = "Cancel";
                pl[1] = "Ghost mode";
                pl[2] = p_grid_label;
                if (g.conflict_far)
                    snprintf(p_subtitle, sizeof(p_subtitle),
                             "  -  %d drones too far apart", g.vehicle_count);
                else
                    snprintf(p_subtitle, sizeof(p_subtitle),
                             "  -  %d drones overlap", g.vehicle_count);
            } else {
                snprintf(p_title, sizeof(p_title), "REPLAY MODE");
                pl[0] = "Formation";
                pl[1] = "Ghost";
                pl[2] = "Grid offset";
                snprintf(p_subtitle, sizeof(p_subtitle),
                         "  -  %d drones", g.vehicle_count);
            }
            wasm_dialog_begin(&g.dialog, p_title, p_subtitle, pl, 3);
            g.dialog_context = 1;
        }

        // Per-drone color refresh on theme change (main.c:796-798)
        for (int i = 0; i < g.vehicle_count; i++)
            g.vehicles[i].color = g.scene.theme->drone_palette[i];

        // ? help overlay (main.c:801)
        if (IsKeyPressed(KEY_SLASH) &&
            (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)))
            g.hud.show_help = !g.hud.show_help;

        // H HUD mode cycle (main.c:806)
        if (IsKeyPressed(KEY_H)) {
            hud_mode_t prev_mode = g.hud.mode;
            g.hud.mode = (g.hud.mode + 1) % HUD_MODE_COUNT;
            const char *mode_names[] = { "Console HUD", "Tactical HUD", "HUD Off" };
            hud_toast(&g.hud, mode_names[g.hud.mode], 2.0f);
            g.show_hud = (g.hud.mode != HUD_OFF);
            if (g.hud.mode == HUD_TACTICAL) {
                g.saved_chase_distance = g.scene.chase_distance;
                g.scene.chase_distance = g.tactical_chase_target;
            } else if (prev_mode == HUD_TACTICAL) {
                g.scene.chase_distance = g.saved_chase_distance;
            }
        }

        // Shift+T correlation cycle (main.c:822)
        if (IsKeyPressed(KEY_T) &&
            (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT))
            && g.log_loaded && g.vehicle_count > 1) {
            g.corr_mode = (g.corr_mode + 1) % 3;
            const char *names[] = { "Correlation Off", "Correlation Line", "Correlation Curtain" };
            hud_toast(&g.hud, names[g.corr_mode], 2.0f);
        }
        // T trail cycle (main.c:829)
        else if (IsKeyPressed(KEY_T)) {
            int max_modes = (g.vehicle_count > 1) ? 4 : 3;
            g.trail_mode = (g.trail_mode + 1) % max_modes;
            const char *trail_names[] = { "Trails Off", "Direction Trails", "Speed Ribbons", "ID Trails" };
            hud_toast(&g.hud, trail_names[g.trail_mode], 2.0f);
        }

        // K classic colors (main.c:837)
        if (IsKeyPressed(KEY_K))
            g.classic_colors = !g.classic_colors;

        // G ground track (main.c:842)
        if (IsKeyPressed(KEY_G))
            g.show_ground_track = !g.show_ground_track;

        // Ctrl+D debug panel (main.c:847)
        if ((IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_D))
            g.dbg_panel.visible = !g.dbg_panel.visible;

        // Ctrl+L edge indicators + correlation labels (main.c:852)
        if ((IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_L)) {
            g.show_edge_indicators = !g.show_edge_indicators;
            g.show_corr_labels = !g.show_corr_labels;
        }

        // Z axis gizmo (main.c:858)
        if (IsKeyPressed(KEY_Z) && !IsKeyDown(KEY_LEFT_SHIFT) && !IsKeyDown(KEY_RIGHT_SHIFT)
            && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL))
            g.show_axes = !g.show_axes;

        // O ortho panel (main.c:864)
        if (IsKeyPressed(KEY_O))
            g.ortho.visible = !g.ortho.visible;

        // M / Shift+M model cycle (main.c:870)
        if (IsKeyPressed(KEY_M)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                int next = (g.vehicles[g.selected].model_idx + 1) % vehicle_model_count;
                vehicle_load_model(&g.vehicles[g.selected], next);
            } else {
                vehicle_cycle_model(&g.vehicles[g.selected]);
            }
        }

        // Vehicle selection (main.c:880-956)
        if (g.vehicle_count > 1) {
            if (IsKeyPressed(KEY_TAB)) {
                for (int j = 1; j <= g.vehicle_count; j++) {
                    int next = (g.selected + j) % g.vehicle_count;
                    if (g.sources[next].connected) { g.selected = next; break; }
                }
                g.hud.pinned_count = 0;
                memset(g.hud.pinned, -1, sizeof(g.hud.pinned));
                g.chord_first = -1;
            }
            // Number-key drone selection with two-digit chord for 10-16
            {
                int digit = -1;
                for (int k = KEY_ZERO; k <= KEY_NINE; k++) {
                    if (IsKeyPressed(k)) { digit = k - KEY_ZERO; break; }
                }

                bool shift_held = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
                bool ctrl_held = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
                bool alt_held  = IsKeyDown(KEY_LEFT_ALT)     || IsKeyDown(KEY_RIGHT_ALT);

                if (g.chord_first >= 0 && GetTime() - g.chord_time > CHORD_TIMEOUT_S) {
                    int idx = g.chord_first - 1;
                    if (idx >= 0 && idx < g.vehicle_count)
                        apply_vehicle_selection_hud(&g.hud, idx, g.chord_shift, &g.selected, g.vehicle_count);
                    g.chord_first = -1;
                }

                if (digit >= 0 && !ctrl_held && !alt_held) {
                    if (g.chord_first >= 0) {
                        int two_digit = g.chord_first * 10 + digit;
                        int idx = two_digit - 1;
                        g.chord_first = -1;
                        if (idx >= 0 && idx < g.vehicle_count)
                            apply_vehicle_selection_hud(&g.hud, idx, g.chord_shift, &g.selected, g.vehicle_count);
                    } else if (digit >= 1 && digit <= 9) {
                        if (g.vehicle_count > 9) {
                            g.chord_first = digit;
                            g.chord_time = GetTime();
                            g.chord_shift = shift_held;
                        } else {
                            int idx = digit - 1;
                            if (idx < g.vehicle_count)
                                apply_vehicle_selection_hud(&g.hud, idx, shift_held, &g.selected, g.vehicle_count);
                        }
                    }
                }
            }
        }
    } // end !marker_input.active block

    // Marker label text input (main.c:960-963)
    if (g.marker_input.active) {
        int di = g.marker_input.drone_idx;
        marker_input_update(&g.marker_input, &g.markers[di], &g.sources[di], &g.vehicles[di]);
    }

    // Re-toast insufficient data on selection change (main.c:966-974)
    if (g.log_loaded && g.selected != g.last_selected_for_insuf && g.insufficient_data[g.selected]) {
        char msg[64];
        snprintf(msg, sizeof(msg), "DRONE %d: INSUFFICIENT DATA", g.selected + 1);
        hud_toast_color(&g.hud, msg, 3.0f, g.vehicles[g.selected].color);
    }
    g.last_selected_for_insuf = g.selected;

    // ── Replay playback controls (main.c:977-1171) ──────────────────────────
    if (g.log_loaded && !g.marker_input.active && !g.dialog.active) {
        bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        bool ctrl  = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
        int  nrf   = g.vehicle_count;

        // Space play/pause + restart-from-end (main.c:981)
        if (IsKeyPressed(KEY_SPACE)) {
            if (!g.sources[g.selected].connected) {
                for (int i = 0; i < nrf; i++) {
                    data_source_seek(&g.sources[i], 0.0f);
                    g.sources[i].connected = true;
                    g.sources[i].playback.paused = false;
                    vehicle_reset_trail(&g.vehicles[i]);
                }
                memset(g.corr, 0, sizeof(g.corr));
            } else {
                bool p = !g.sources[g.selected].playback.paused;
                for (int i = 0; i < nrf; i++)
                    g.sources[i].playback.paused = p;
            }
        }

        // Shift+L looping (main.c:996)
        if (IsKeyPressed(KEY_L) && !ctrl && shift) {
            bool l = !g.sources[g.selected].playback.looping;
            for (int i = 0; i < nrf; i++)
                g.sources[i].playback.looping = l;
        }

        // Y yaw/heading (main.c:1001)
        if (IsKeyPressed(KEY_Y))
            g.hud.show_yaw = !g.hud.show_yaw;

        // N notifications (main.c:1004)
        if (IsKeyPressed(KEY_N)) {
            g.hud.show_notifications = !g.hud.show_notifications;
            hud_toast(&g.hud, g.hud.show_notifications ? "Notifications On" : "Notifications Off", 2.0f);
        }

        // L marker labels toggle (main.c:1008) — guarded against B→L chord
        if (IsKeyPressed(KEY_L) && !ctrl && !shift
            && (g.markers[g.selected].last_drop_idx < 0
                || GetTime() - g.markers[g.selected].last_drop_time >= 0.5)) {
            g.show_marker_labels = !g.show_marker_labels;
        }

        // I interpolation (main.c:1011)
        if (IsKeyPressed(KEY_I)) {
            bool interp = !g.sources[g.selected].playback.interpolation;
            for (int i = 0; i < nrf; i++)
                g.sources[i].playback.interpolation = interp;
            hud_toast(&g.hud, interp ? "Interpolation On" : "Interpolation Off", 2.0f);
        }

        // +/= speed up (main.c:1017)
        if (IsKeyPressed(KEY_EQUAL)) {
            float spd = g.sources[g.selected].playback.speed;
            if (spd < 0.5f) spd = 0.5f;
            else if (spd < 1.0f) spd = 1.0f;
            else if (spd < 2.0f) spd = 2.0f;
            else if (spd < 4.0f) spd = 4.0f;
            else if (spd < 8.0f) spd = 8.0f;
            else spd = 16.0f;
            for (int i = 0; i < nrf; i++)
                g.sources[i].playback.speed = spd;
        }

        // - speed down (main.c:1028)
        if (IsKeyPressed(KEY_MINUS)) {
            float spd = g.sources[g.selected].playback.speed;
            if (spd > 8.0f) spd = 8.0f;
            else if (spd > 4.0f) spd = 4.0f;
            else if (spd > 2.0f) spd = 2.0f;
            else if (spd > 1.0f) spd = 1.0f;
            else if (spd > 0.5f) spd = 0.5f;
            else spd = 0.25f;
            for (int i = 0; i < nrf; i++)
                g.sources[i].playback.speed = spd;
        }

        // R reset all drones (main.c:1039)
        if (IsKeyPressed(KEY_R)) {
            for (int i = 0; i < nrf; i++) {
                data_source_seek(&g.sources[i], 0.0f);
                g.sources[i].connected = true;
                g.sources[i].playback.paused = false;
                vehicle_reset_trail(&g.vehicles[i]);
            }
            memset(g.corr, 0, sizeof(g.corr));
            for (int i = 0; i < g.vehicle_count; i++) {
                g.markers[i].count = 0;
                g.markers[i].current = -1;
            }
        }

        // A takeoff alignment (main.c:1053)
        if (IsKeyPressed(KEY_A) && g.vehicle_count > 1) {
            g.takeoff_aligned = !g.takeoff_aligned;
            const float takeoff_buffer = 5.0f;
            for (int i = 0; i < nrf; i++) {
                if (g.takeoff_aligned) {
                    float skip = g.sources[i].playback.takeoff_detected
                        ? g.sources[i].playback.takeoff_time_s - takeoff_buffer : 0.0f;
                    if (skip < 0.0f) skip = 0.0f;
                    data_source_set_time_offset(&g.sources[i], (double)skip);
                } else {
                    data_source_set_time_offset(&g.sources[i], 0.0);
                }
                data_source_seek(&g.sources[i], 0.0f);
                g.sources[i].connected = true;
                g.sources[i].playback.paused = false;
                vehicle_reset_trail(&g.vehicles[i]);
            }
            memset(g.corr, 0, sizeof(g.corr));
            for (int i = 0; i < nrf; i++) {
                g.sources[i].playback.correlation = NAN;
                g.sources[i].playback.rmse = NAN;
            }
            for (int i = 0; i < nrf; i++)
                g.sys_markers[i].resolved = false;
            hud_toast(&g.hud, g.takeoff_aligned ? "Auto Align On" : "Auto Align Off", 2.0f);
        }

        // Arrow keys seek all drones with 3 granularity levels (main.c:1083)
        if (IsKeyPressed(KEY_RIGHT)) {
            float step;
            if (shift && ctrl) step = 1.0f;
            else if (shift) { step = 0.02f; g.sources[g.selected].playback.paused = true; }
            else step = 5.0f;
            float seek_target = g.sources[g.selected].playback.position_s + step;
            for (int i = 0; i < nrf; i++) {
                data_source_seek(&g.sources[i], seek_target);
                vehicle_reset_trail(&g.vehicles[i]);
            }
            memset(g.corr, 0, sizeof(g.corr));
        }
        if (IsKeyPressed(KEY_LEFT)) {
            float step;
            if (shift && ctrl) step = 1.0f;
            else if (shift) { step = 0.02f; g.sources[g.selected].playback.paused = true; }
            else step = 5.0f;
            float target = g.sources[g.selected].playback.position_s - step;
            if (target < 0.0f) target = 0.0f;
            for (int i = 0; i < nrf; i++) {
                data_source_seek(&g.sources[i], target);
                vehicle_reset_trail(&g.vehicles[i]);
            }
            memset(g.corr, 0, sizeof(g.corr));
        }

        // B drop / Shift+B delete (main.c:1110)
        if (IsKeyPressed(KEY_B) && g.vehicles[g.selected].active && !g.marker_input.active) {
            if (shift) {
                marker_delete(&g.markers[g.selected]);
            } else if (g.markers[g.selected].count < REPLAY_MAX_MARKERS) {
                marker_drop(&g.markers[g.selected], g.sources[g.selected].playback.position_s,
                            g.vehicles[g.selected].position, &g.vehicles[g.selected],
                            &g.sys_markers[g.selected]);
            }
        }

        // B→L chord label input (main.c:1121)
        if (IsKeyPressed(KEY_L) && !g.marker_input.active
            && g.markers[g.selected].last_drop_idx >= 0) {
            double elapsed = GetTime() - g.markers[g.selected].last_drop_time;
            if (elapsed < 0.5) {
                marker_input_begin(&g.marker_input, g.markers[g.selected].last_drop_idx,
                                   &g.sources[g.selected]);
                g.marker_input.drone_idx = g.selected;
                g.markers[g.selected].last_drop_idx = -1;
                for (int i = 0; i < g.vehicle_count; i++)
                    g.sources[i].playback.paused = true;
            }
        }

        // [/] marker cycling with Ctrl+[/] global (main.c:1136)
        if (IsKeyPressed(KEY_LEFT_BRACKET) || IsKeyPressed(KEY_RIGHT_BRACKET)) {
            int dir = IsKeyPressed(KEY_LEFT_BRACKET) ? -1 : 1;
            if (ctrl) {
                marker_cycle_result_t r = marker_cycle_global(
                    g.markers, g.sys_markers, g.vehicle_count, g.selected, dir,
                    g.sources, g.vehicles, g.precomp, &g.scene, g.last_pos);
                if (r.jumped && r.drone_idx != g.selected)
                    g.selected = r.drone_idx;
            } else if (g.markers[g.selected].count > 0 || g.sys_markers[g.selected].count > 0) {
                marker_cycle(&g.markers[g.selected], &g.sys_markers[g.selected], dir, shift,
                             &g.sources[g.selected], &g.vehicles[g.selected],
                             &g.precomp[g.selected], &g.scene, &g.last_pos[g.selected]);
            }
            if (!shift) {
                annunc_trigger_tab_fade(&g.hud.annunciators, g.selected);
                annunc_trigger_radar_wave(&g.hud.annunciators, g.selected);
                for (int p = 0; p < g.hud.pinned_count; p++) {
                    int pidx = g.hud.pinned[p];
                    if (pidx >= 0 && pidx < g.vehicle_count)
                        annunc_trigger_ring_bounce(&g.hud.annunciators, pidx);
                }
            }
            // Sync all drones to the same playback time after marker seek (main.c:1162)
            if (!shift && g.vehicle_count > 1) {
                float sync_time = g.sources[g.selected].playback.position_s;
                for (int i = 0; i < g.vehicle_count; i++) {
                    if (i == g.selected) continue;
                    data_source_seek(&g.sources[i], sync_time);
                    vehicle_reset_trail(&g.vehicles[i]);
                }
            }
        }
    }

    // Debug panel update (main.c:1174)
    debug_panel_update(&g.dbg_panel, frame_dt);

    // Camera update (main.c:1177)
    {
        Vector3 cam_target = g.vehicles[g.selected].position;
        if (g.hud.mode == HUD_TACTICAL)
            cam_target.y += g.scene.chase_distance * 0.10f;
        scene_update_camera(&g.scene, cam_target, g.vehicles[g.selected].rotation);
    }

    // Ortho panel render (main.c:1186)
    if (g.ortho.visible || g.hud.mode == HUD_TACTICAL) {
        ortho_panel_update(&g.ortho, g.vehicles[g.selected].position);
        ortho_panel_render(&g.ortho, g.vehicles, g.vehicle_count,
                           g.selected, g.scene.theme,
                           g.corr_mode, g.hud.pinned, g.hud.pinned_count);
    }

    int sw = GetScreenWidth();
    int sh = GetScreenHeight();

    // ── Render (main.c:1194-1428) ──────────────────────────────────────────
    BeginDrawing();

        scene_draw_sky(&g.scene);

        bool fs_ortho = (g.scene.ortho_mode != ORTHO_NONE);
        int  tm_3d    = fs_ortho ? 0 : g.trail_mode;

        BeginMode3D(g.scene.camera);
            scene_draw(&g.scene);
            for (int i = 0; i < g.vehicle_count; i++) {
                if (g.vehicles[i].active || g.vehicle_count == 1) {
                    vehicle_draw(&g.vehicles[i], g.scene.theme, i == g.selected,
                                 tm_3d, g.show_ground_track, g.scene.camera.position,
                                 g.classic_colors);
                }
            }

            // 3D markers for all drones (main.c:1213)
            for (int i = 0; i < g.vehicle_count; i++) {
                if (g.log_loaded && g.markers[i].count > 0) {
                    int cur = (i == g.selected && !g.sys_markers[i].selected)
                              ? g.markers[i].current : -1;
                    vehicle_draw_markers(g.markers[i].positions, g.markers[i].labels,
                                         g.markers[i].count, cur,
                                         g.scene.camera.position, g.scene.camera,
                                         g.markers[i].roll, g.markers[i].pitch,
                                         g.markers[i].vert, g.markers[i].speed,
                                         g.vehicles[i].trail_speed_max, g.scene.theme,
                                         g.trail_mode, MARKER_USER,
                                         g.vehicles[i].color);
                }
                if (g.log_loaded && g.sys_markers[i].count > 0) {
                    int cur = (i == g.selected && g.sys_markers[i].selected)
                              ? g.sys_markers[i].current : -1;
                    vehicle_draw_markers(g.sys_markers[i].positions, g.sys_markers[i].labels,
                                         g.sys_markers[i].count, cur,
                                         g.scene.camera.position, g.scene.camera,
                                         g.sys_markers[i].roll, g.sys_markers[i].pitch,
                                         g.sys_markers[i].vert, g.sys_markers[i].speed,
                                         g.vehicles[i].trail_speed_max, g.scene.theme,
                                         g.trail_mode, MARKER_SYSTEM,
                                         g.vehicles[i].color);
                }
            }

            // Home position markers in formation mode (main.c:1241)
            if (g.vehicle_count > 1 && !g.ghost_mode && !g.ghost_mode_grid) {
                for (int i = 0; i < g.vehicle_count; i++) {
                    if (!g.sources[i].home.valid) continue;
                    double lat = g.sources[i].home.lat * 1e-7 * (M_PI / 180.0);
                    double lon = g.sources[i].home.lon * 1e-7 * (M_PI / 180.0);
                    double alt = g.sources[i].home.alt * 1e-3;
                    float hx = (float)(EARTH_RADIUS * (lon - g.ref_lon_rad) * cos(g.ref_lat_rad)) + g.vehicles[i].grid_offset.x;
                    float hy = (float)(alt - g.min_alt) + 0.02f;
                    float hz = (float)(-(EARTH_RADIUS * (lat - g.ref_lat_rad))) + g.vehicles[i].grid_offset.z;
                    float half = 0.333f;
                    Color fill = g.vehicles[i].color; fill.a = 70;
                    Color border = g.vehicles[i].color; border.a = 180;
                    DrawPlane((Vector3){hx, hy, hz}, (Vector2){half * 2, half * 2}, fill);
                    DrawLine3D((Vector3){hx - half, hy, hz - half}, (Vector3){hx + half, hy, hz - half}, border);
                    DrawLine3D((Vector3){hx + half, hy, hz - half}, (Vector3){hx + half, hy, hz + half}, border);
                    DrawLine3D((Vector3){hx + half, hy, hz + half}, (Vector3){hx - half, hy, hz + half}, border);
                    DrawLine3D((Vector3){hx - half, hy, hz + half}, (Vector3){hx - half, hy, hz - half}, border);
                }
            }

            // Axis gizmo (main.c:1264)
            if (g.show_axes && g.vehicles[g.selected].active) {
                Vector3 com = g.vehicles[g.selected].position;
                com.y += g.vehicles[g.selected].model_scale * 0.15f;
                draw_axis_gizmo_3d(com, g.vehicles[g.selected].model_scale * 0.5f,
                                    g.vehicles[g.selected].rotation);
            }

            // Correlation overlay (main.c:1274)
            if (g.corr_mode > 0 && g.hud.pinned_count > 0) {
                for (int p = 0; p < g.hud.pinned_count; p++) {
                    int pidx = g.hud.pinned[p];
                    if (pidx >= 0 && pidx < g.vehicle_count && g.vehicles[pidx].active
                        && pidx != g.selected) {
                        if (g.corr_mode == 1 && !fs_ortho) {
                            vehicle_draw_correlation_line(
                                &g.vehicles[g.selected], &g.vehicles[pidx]);
                        } else if (g.corr_mode == 2) {
                            vehicle_draw_correlation_curtain(
                                &g.vehicles[g.selected], &g.vehicles[pidx],
                                g.scene.theme, g.scene.camera.position);
                        }
                    }
                }
            }
        EndMode3D();

        scene_draw_ortho_ground(&g.scene, sw, sh);

        // Edge indicators (main.c:1296)
        if (g.vehicle_count > 1 && g.show_edge_indicators) {
            float ei_scale = powf(sh / 720.0f, 0.7f);
            if (ei_scale < 1.0f) ei_scale = 1.0f;
            draw_edge_indicators(g.vehicles, g.vehicle_count, g.selected,
                                 g.scene.camera, g.hud.font_value, ei_scale);
        }

        // 2D marker labels (main.c:1304)
        if (g.log_loaded && g.show_marker_labels) {
            for (int i = 0; i < g.vehicle_count; i++) {
                if (g.markers[i].count > 0) {
                    int cur = (i == g.selected && !g.sys_markers[i].selected)
                              ? g.markers[i].current : -1;
                    vehicle_draw_marker_labels(g.markers[i].positions, g.markers[i].labels,
                                               g.markers[i].count, cur,
                                               g.scene.camera.position, g.scene.camera,
                                               g.hud.font_label, g.hud.font_value,
                                               g.markers[i].roll, g.markers[i].pitch,
                                               g.markers[i].vert, g.markers[i].speed,
                                               g.vehicles[i].trail_speed_max, g.scene.theme,
                                               g.trail_mode, MARKER_USER,
                                               g.vehicles[i].color);
                }
                if (g.sys_markers[i].count > 0) {
                    int cur = (i == g.selected && g.sys_markers[i].selected)
                              ? g.sys_markers[i].current : -1;
                    vehicle_draw_marker_labels(g.sys_markers[i].positions, g.sys_markers[i].labels,
                                               g.sys_markers[i].count, cur,
                                               g.scene.camera.position, g.scene.camera,
                                               g.hud.font_label, g.hud.font_value,
                                               g.sys_markers[i].roll, g.sys_markers[i].pitch,
                                               g.sys_markers[i].vert, g.sys_markers[i].speed,
                                               g.vehicles[i].trail_speed_max, g.scene.theme,
                                               g.trail_mode, MARKER_SYSTEM,
                                               g.vehicles[i].color);
                }
            }
        }

        // Fullscreen ortho 2D overlays (main.c:1336)
        ortho_draw_fullscreen_2d(&g.scene, g.vehicles, g.vehicle_count,
                                  g.selected, g.trail_mode,
                                  g.corr_mode, g.hud.pinned, g.hud.pinned_count,
                                  sw, sh, g.hud.font_label, g.show_corr_labels);

        // HUD (main.c:1343)
        if (g.show_hud) {
            bool has_awaiting_gps = g.vehicles[g.selected].active &&
                !g.vehicles[g.selected].origin_set && g.sources[g.selected].home.valid;
            hud_marker_data_t all_user_md[MAX_VEHICLES] = {0};
            hud_marker_data_t all_sys_md[MAX_VEHICLES] = {0};
            for (int i = 0; i < g.vehicle_count; i++) {
                all_user_md[i] = (hud_marker_data_t){
                    .times = g.markers[i].times,
                    .labels = g.markers[i].labels,
                    .roll = g.markers[i].roll,
                    .pitch = g.markers[i].pitch,
                    .vert = g.markers[i].vert,
                    .speed = g.markers[i].speed,
                    .speed_max = g.markers[i].speed_max,
                    .count = g.markers[i].count,
                    .current = g.markers[i].current,
                    .selected = true,
                    .color = g.vehicles[i].color,
                };
                all_sys_md[i] = (hud_marker_data_t){
                    .times = g.sys_markers[i].times,
                    .labels = g.sys_markers[i].labels,
                    .roll = g.sys_markers[i].roll,
                    .pitch = g.sys_markers[i].pitch,
                    .vert = g.sys_markers[i].vert,
                    .speed = g.sys_markers[i].speed,
                    .speed_max = g.markers[i].speed_max,
                    .count = g.sys_markers[i].count,
                    .current = g.sys_markers[i].current,
                    .selected = g.sys_markers[i].selected,
                    .color = g.vehicles[i].color,
                };
            }
            if (g.hud.mode == HUD_CONSOLE) {
                hud_draw(&g.hud, g.vehicles, g.sources, g.vehicle_count,
                         g.selected, sw, sh, g.scene.theme, g.trail_mode,
                         all_user_md, all_sys_md, g.vehicle_count,
                         g.ghost_mode, g.has_tier3, has_awaiting_gps);
            } else if (g.hud.mode == HUD_TACTICAL) {
                tactical_hud_draw(&g.hud, g.vehicles, g.sources, g.vehicle_count,
                                  g.selected, sw, sh, g.scene.theme, g.ghost_mode,
                                  g.has_tier3, has_awaiting_gps,
                                  &g.ortho, g.trail_mode, g.corr_mode,
                                  all_user_md, all_sys_md, g.vehicle_count);
            }
        }

        // Debug panel (main.c:1392)
        {
            int active_count = 0;
            int total_trail = 0;
            for (int i = 0; i < g.vehicle_count; i++) {
                if (g.vehicles[i].active) active_count++;
                total_trail += g.vehicles[i].trail_count;
            }
            debug_panel_draw(&g.dbg_panel, sw, sh, g.scene.theme, g.hud.font_label,
                             g.vehicle_count, active_count, total_trail,
                             g.vehicles[g.selected].position,
                             g.sources[g.selected].ref_rejected,
                             g.vehicle_tier[g.selected]);
        }

        // Ortho panel (main.c:1409)
        if (g.hud.mode != HUD_TACTICAL) {
            int bar_h = g.show_hud ? hud_bar_height(&g.hud, sh) : 0;
            ortho_panel_draw(&g.ortho, sh, bar_h, g.scene.theme, g.hud.font_label,
                             g.vehicles, g.vehicle_count, g.selected, g.trail_mode,
                             g.corr_mode, g.hud.pinned, g.hud.pinned_count,
                             g.show_axes);
        }

        // Fullscreen ortho label (main.c:1418)
        ortho_panel_draw_fullscreen_label(sw, sh, g.scene.ortho_mode, g.scene.ortho_span,
                                          g.scene.theme, g.hud.font_label, g.show_axes);

        // Marker label input overlay (main.c:1423)
        if (g.marker_input.active) {
            marker_input_draw(&g.marker_input, g.hud.font_label, g.hud.font_value,
                              g.scene.theme, sw, sh);
        }

        // Non-blocking prompt dialog. Renders on top of everything else and
        // captures 1..N keys. Replaces native's blocking draw_prompt_dialog.
        if (g.dialog.active) {
            int choice = wasm_dialog_draw_and_poll(&g.dialog, g.scene.theme,
                                                    g.hud.font_label, g.hud.font_value);
            if (choice > 0) {
                if (g.dialog_context == 0) {
                    // Initial conflict dialog. Playback was frozen while the
                    // dialog was up, so first-state still establishes origin.
                    //   1 = Cancel & reupload  (unload)
                    //   2 = Ghost mode
                    //   3 = Grid / Narrow grid
                    if (choice == 1) {
                        hawkeye_unload_internal();
                    } else if (choice == 2) {
                        apply_replay_mode(2, false);
                    } else if (choice == 3) {
                        apply_replay_mode(3, false);
                    }
                } else {
                    // P-key dialog. User is switching mid-flight — rebase
                    // origin to home so the chosen coordinate frame applies
                    // immediately (native main.c:705-783).
                    if (g.conflict_detected) {
                        if (choice == 1)      apply_replay_mode(4, true);
                        else if (choice == 2) apply_replay_mode(2, true);
                        else if (choice == 3) apply_replay_mode(3, true);
                    } else {
                        apply_replay_mode(choice, true);
                    }
                }
            }
        }

    EndDrawing();
}

// ---------------------------------------------------------------------------
// Exported API
// ---------------------------------------------------------------------------

EMSCRIPTEN_KEEPALIVE
int hawkeye_init(const char *canvas_id, int width, int height) {
    if (g.initialized) return -1;
    if (!canvas_id || width <= 0 || height <= 0) return -1;

    log_heap("init:entry");

    memset(&g, 0, sizeof(g));
    g.width = width;
    g.height = height;
    strncpy(g.canvas_id, canvas_id, sizeof(g.canvas_id) - 1);

    SetLoadFileTextCallback(hawkeye_load_file_text);
    asset_path_init();
    log_heap("init:after_asset_path");

    // DPI strategy (see rcore_web.c analysis in WASM_PORT_FAILURES.md):
    //   - Raylib's web backend has no DPI awareness (GetWindowScaleDPI()
    //     stub returns {1,1}; FLAG_WINDOW_HIGHDPI is ignored).
    //   - glfwCreateWindow(buf_w, buf_h) routes through emscripten's
    //     Browser.setCanvasSize, which sets both the GL buffer size AND
    //     canvas.style.* to the same DPR-scaled pixel count. On a Retina
    //     display that stretches the CSS size to 2x intended, making the
    //     canvas overflow the page.
    //   - We compensate by calling emscripten_set_element_css_size right
    //     after InitWindow to force the CSS size back to logical pixels.
    //     Browser then downscales the oversized GL buffer into the logical
    //     CSS box for crisp Retina rendering.
    double dpr = emscripten_get_device_pixel_ratio();
    int buf_w = (int)(width * dpr);
    int buf_h = (int)(height * dpr);

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(buf_w, buf_h, "Hawkeye");
    if (!IsWindowReady()) return -1;
    SetTargetFPS(60);
    log_heap("init:after_InitWindow");

    emscripten_set_element_css_size(canvas_id, (double)width, (double)height);

    scene_init(&g.scene);
    log_heap("init:after_scene_init");
    theme_registry_init(&g.scene.theme_reg);
    log_heap("init:after_theme_reg");
    g.scene.theme_index = 0;
    g.scene.theme = g.scene.theme_reg.themes[0];

    // Placeholder vehicle so the frame loop has something to render until the
    // first ULog loads. Replaced during finalize_multi_load.
    vehicle_init_ex(&g.vehicles[0], MODEL_QUADROTOR, g.scene.lighting_shader, 36000);
    log_heap("init:after_vehicle_init");
    g.vehicles[0].color = g.scene.theme->drone_palette[0];
    g.vehicle_count = 1;

    hud_init(&g.hud);
    log_heap("init:after_hud_init");
    g.hud.is_replay = true;
    g.show_hud = true;
    debug_panel_init(&g.dbg_panel);
    ortho_panel_init(&g.ortho);
    log_heap("init:after_panels");

    // Match main.c:421-465 defaults
    g.selected      = 0;
    g.prev_selected = 0;
    g.chord_first   = -1;
    g.trail_mode    = 1;
    g.show_ground_track = false;
    g.classic_colors = false;
    g.show_edge_indicators = true;
    g.corr_mode      = 0;
    g.show_corr_labels = true;
    g.show_axes      = false;
    g.show_marker_labels = true;
    g.tactical_chase_target = 1.6f;
    g.last_selected_for_insuf = -1;

    g.markers[0].current = -1;
    g.markers[0].last_drop_idx = -1;
    g.sys_markers[0].current = -1;
    precomp_trail_init(&g.precomp[0]);
    g.marker_input.target = -1;

    // Data-source shim starts disconnected with ops bound
    memset(&g.sources[0], 0, sizeof(g.sources[0]));
    g.sources[0].ops = &wasm_ds_ops;
    g.sources[0].playback.speed = 1.0f;
    g.sources[0].playback.looping = true;
    g.sources[0].playback.interpolation = true;

    g.initialized = true;
    emscripten_set_main_loop_arg(frame, &g, 0, 0);
    printf("hawkeye_init: %dx%d canvas=%s theme=%s\n",
           width, height, canvas_id, g.scene.theme ? g.scene.theme->name : "(none)");
    log_heap("init:done");
    return 0;
}

// Forward declarations for the staged multi-load API
EMSCRIPTEN_KEEPALIVE int hawkeye_begin_multi_load(int count);
EMSCRIPTEN_KEEPALIVE int hawkeye_stage_ulog(int index, const uint8_t *buf, size_t len);
EMSCRIPTEN_KEEPALIVE int hawkeye_finalize_multi_load(int mode);

EMSCRIPTEN_KEEPALIVE
int hawkeye_load_ulog_bytes(const uint8_t *buf, size_t len) {
    if (hawkeye_begin_multi_load(1) != 0) return -1;
    if (hawkeye_stage_ulog(0, buf, len) != 0) return -1;
    return hawkeye_finalize_multi_load(1);
}

EMSCRIPTEN_KEEPALIVE
int hawkeye_begin_multi_load(int count) {
    if (!g.initialized || count < 1 || count > MAX_VEHICLES) return -1;
    log_heap("begin_multi_load:entry");

    // Pre-grow the WASM heap in one shot to fit `count` drones plus the
    // transient raw-ULog staging buffer. Measured on 16 × 50 MB ULogs:
    //   - baseline after init:       ~3 MB
    //   - raw-file staging peak:     ~55 MB (one file at a time)
    //   - per-drone timeline + state: ~3 MB
    // One emscripten_resize_heap call avoids the cascading grow-and-copy
    // transients that hit N× peak during load.
    size_t target = (size_t)(8 + 60 + count * 4) * 1024 * 1024;
    emscripten_resize_heap(target);

    if (g.log_loaded) {
        for (int i = 0; i < g.vehicle_count; i++) {
            wasm_replay_close(&g.replays[i]);
            vehicle_cleanup(&g.vehicles[i]);
            precomp_trail_cleanup(&g.precomp[i]);
        }
        g.log_loaded = false;
    }
    g.vehicle_count = count;
    g.selected      = 0;
    g.prev_selected = 0;
    printf("hawkeye_begin_multi_load: preparing for %d drones\n", count);
    return 0;
}

EMSCRIPTEN_KEEPALIVE
int hawkeye_stage_ulog(int index, const uint8_t *buf, size_t len) {
    if (!g.initialized || index < 0 || index >= g.vehicle_count || !buf || len == 0)
        return -1;

    log_heap("stage_ulog:entry");
    if (wasm_replay_init_from_bytes(&g.replays[index], buf, len) != 0) {
        fprintf(stderr, "hawkeye_stage_ulog: extraction failed for index %d\n", index);
        return -1;
    }
    const ulog_timeline_t *tl = (const ulog_timeline_t *)g.replays[index].timeline;
    printf("hawkeye_stage_ulog[%d]: %zu bytes, %d att, %d lpos\n",
           index, len, tl->att_count, tl->lpos_count);
    log_heap("stage_ulog:after_extract");
    return 0;
}

// Finalize the multi-load. `mode` picks the layout:
//   0 = auto     (formation if no conflict, defaults to formation — conflict
//                 dialog is Phase 2; callers wanting non-default behavior
//                 should pass 1/2/3 explicitly)
//   1 = formation
//   2 = ghost
//   3 = grid
// Mirrors main.c:294-409.
EMSCRIPTEN_KEEPALIVE
int hawkeye_finalize_multi_load(int mode) {
    if (!g.initialized) return -1;
    int count = g.vehicle_count;

    g.log_loaded = true;

    // Vehicles (main.c:251-254)
    for (int i = 0; i < count; i++) {
        vehicle_init_ex(&g.vehicles[i], MODEL_QUADROTOR, g.scene.lighting_shader, 36000);
        g.vehicles[i].color = g.scene.theme->drone_palette[i % 16];
        vehicle_set_type(&g.vehicles[i], g.replays[i].vehicle_type);
        g.vehicles[i].origin_set = false;
        g.vehicles[i].origin_wait_count = 0;
    }

    // Initialize data_source shims (one per replay ctx)
    for (int i = 0; i < count; i++) {
        memset(&g.sources[i], 0, sizeof(g.sources[i]));
        ds_init_static(&g.sources[i], &g.replays[i]);
    }

    // Multi-file alignment seed (main.c:286-292)
    if (count > 1) {
        for (int i = 0; i < count; i++)
            g.sources[i].playback.time_offset_s = 0.0f;
    }

    // Conflict detection (main.c:294-337)
    g.conflict_detected = false;
    g.conflict_far = false;
    if (count > 1) {
        conflict_result_t cr = replay_detect_conflict(g.sources, count);
        g.conflict_detected = cr.conflict_detected;
        g.conflict_far      = cr.conflict_far;
    }

    // Shared NED reference (main.c:339-385). Computed up-front so formation
    // mode can snap to it without further arithmetic.
    g.ref_lat_rad = 0.0;
    g.ref_lon_rad = 0.0;
    g.min_alt     = 0.0;
    if (count > 1) {
        for (int i = 0; i < count; i++) {
            if (g.sources[i].home.valid) {
                g.ref_lat_rad = g.sources[i].home.lat / 1e7 * (M_PI / 180.0);
                g.ref_lon_rad = g.sources[i].home.lon / 1e7 * (M_PI / 180.0);
                break;
            }
        }
        g.min_alt = 1e9;
        for (int i = 0; i < count; i++) {
            if (g.sources[i].home.valid) {
                double a = g.sources[i].home.alt * 1e-3;
                if (a < g.min_alt) g.min_alt = a;
            }
        }
        if (g.min_alt > 1e8) g.min_alt = 0.0;
        printf("Multi-drone origin: lat=%.6f lon=%.6f alt=%.1f\n",
               g.ref_lat_rad * (180.0 / M_PI), g.ref_lon_rad * (180.0 / M_PI), g.min_alt);
    }

    // Position tiers (main.c:388-395)
    memset(g.vehicle_tier, 0, sizeof(g.vehicle_tier));
    for (int i = 0; i < count; i++) {
        if (g.sources[i].playback.home_from_topic) g.vehicle_tier[i] = 1;
        else if (g.sources[i].home.valid)          g.vehicle_tier[i] = 2;
        else                                       g.vehicle_tier[i] = 3;
    }
    g.has_tier3 = false;
    for (int i = 0; i < count; i++) {
        if (!g.sources[i].home.valid) { g.has_tier3 = true; break; }
    }

    // Markers (main.c:449-465). Must be initialized before the dialog opens so
    // precomp structures exist even while the user is choosing a mode.
    for (int i = 0; i < count; i++) {
        memset(&g.markers[i], 0, sizeof(g.markers[i]));
        g.markers[i].current = -1;
        g.markers[i].last_drop_idx = -1;
        memset(&g.sys_markers[i], 0, sizeof(g.sys_markers[i]));
        g.sys_markers[i].current = -1;
        precomp_trail_init(&g.precomp[i]);
        replay_init_sys_markers(&g.sys_markers[i], &g.sources[i]);
    }

    g.trail_mode = (count > 1) ? 3 : 1;

    memset(g.was_connected, 0, sizeof(g.was_connected));
    memset(g.prev_playback_pos, 0, sizeof(g.prev_playback_pos));
    memset(g.last_pos, 0, sizeof(g.last_pos));
    memset(g.insufficient_data, 0, sizeof(g.insufficient_data));
    g.insufficient_check_frames = 0;
    g.insufficient_toasted = false;
    memset(g.corr, 0, sizeof(g.corr));
    g.takeoff_aligned = false;

    // Resolve initial mode. mode 0 = auto, where the default depends on
    // conflict state: no conflict → formation; near conflict → ghost; far
    // conflict → grid. These defaults match what the dialog will propose,
    // so the view never renders drones "in the wrong country" even for the
    // one frame before the user picks.
    int initial_mode = mode;
    if (initial_mode == 0) {
        if (count < 2 || !g.conflict_detected) initial_mode = 1;
        else if (g.conflict_far)               initial_mode = 3;
        else                                   initial_mode = 2;
    }
    apply_replay_mode(initial_mode, false);

    // Open the conflict dialog so the user can override the auto pick.
    // Matches main.c:304-336 — only fires in replay multi-file mode with a
    // detected conflict, and only when the caller asked for auto (mode == 0).
    if (count > 1 && g.conflict_detected && mode == 0) {
        const char *grid_label = g.conflict_far ? "Narrow grid offset" : "Grid offset";
        char subtitle[64];
        if (g.conflict_far)
            snprintf(subtitle, sizeof(subtitle),
                     "  -  %d drones too far apart", count);
        else
            snprintf(subtitle, sizeof(subtitle),
                     "  -  %d drones overlap", count);
        const char *labels[3] = { "Cancel & reupload", "Ghost mode", grid_label };
        wasm_dialog_begin(&g.dialog, "POSITION CONFLICT", subtitle, labels, 3);
        g.dialog_context = 0;
    }

    printf("hawkeye_finalize_multi_load: %d drones, mode=%d (requested=%d), conflict=%d/far=%d\n",
           count, initial_mode, mode, g.conflict_detected, g.conflict_far);
    log_heap("finalize:done");
    return 0;
}

// Query conflict result so the host page can decide mode before finalizing.
// Bits: bit0 = conflict_detected, bit1 = conflict_far. Returns 0 if no log or
// single drone. Intended for Phase 2 dialog wiring.
EMSCRIPTEN_KEEPALIVE
int hawkeye_get_conflict_flags(void) {
    if (!g.initialized || g.vehicle_count < 2) return 0;
    conflict_result_t cr = replay_detect_conflict(g.sources, g.vehicle_count);
    int flags = 0;
    if (cr.conflict_detected) flags |= 1;
    if (cr.conflict_far)      flags |= 2;
    return flags;
}

// Accept a .mvt theme file as raw bytes (JS drag-drop replacement for native's
// IsFileDropped() block at main.c:632-671). Writes the bytes into Emscripten's
// MEMFS at /tmp/<name> so theme_load_mvt's fopen works unchanged, then calls
// theme_registry_add on that path. Returns 0 on success, -1 on failure.
EMSCRIPTEN_KEEPALIVE
int hawkeye_add_theme_bytes(const uint8_t *buf, size_t len, const char *name) {
    if (!g.initialized || !buf || len == 0 || !name) return -1;
    char path[256];
    snprintf(path, sizeof(path), "/tmp/%s", name);
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "hawkeye_add_theme_bytes: fopen(%s) failed\n", path); return -1; }
    size_t w = fwrite(buf, 1, len, f);
    fclose(f);
    if (w != len) return -1;
    if (!theme_registry_add(&g.scene.theme_reg, path)) return -1;
    int last = g.scene.theme_reg.user_count - 1;
    if (last >= 0) {
        char msg[80];
        snprintf(msg, sizeof(msg), "Theme: %s", g.scene.theme_reg.name_bufs[last]);
        hud_toast(&g.hud, msg, 3.0f);
    }
    return 0;
}

EMSCRIPTEN_KEEPALIVE
void hawkeye_resize(int width, int height) {
    if (!g.initialized || width <= 0 || height <= 0) return;
    g.width = width;
    g.height = height;
    double dpr = emscripten_get_device_pixel_ratio();
    SetWindowSize((int)(width * dpr), (int)(height * dpr));
    // SetWindowSize re-stretches canvas.style via emscripten's Browser
    // helper; undo that with the same CSS-size override we use at init.
    emscripten_set_element_css_size(g.canvas_id, (double)width, (double)height);
}

EMSCRIPTEN_KEEPALIVE
void hawkeye_destroy(void) {
    if (!g.initialized) return;
    emscripten_cancel_main_loop();
    for (int i = 0; i < g.vehicle_count; i++) {
        if (g.log_loaded) wasm_replay_close(&g.replays[i]);
        vehicle_cleanup(&g.vehicles[i]);
        precomp_trail_cleanup(&g.precomp[i]);
    }
    ortho_panel_cleanup(&g.ortho);
    hud_cleanup(&g.hud);
    scene_cleanup(&g.scene);
    CloseWindow();
    memset(&g, 0, sizeof(g));
}

EMSCRIPTEN_KEEPALIVE
void hawkeye_set_playing(int playing) {
    if (!g.initialized || !g.log_loaded) return;
    for (int i = 0; i < g.vehicle_count; i++)
        g.sources[i].playback.paused = (playing == 0);
}

EMSCRIPTEN_KEEPALIVE
void hawkeye_seek(double seconds) {
    if (!g.initialized || !g.log_loaded) return;
    if (seconds < 0.0) seconds = 0.0;
    for (int i = 0; i < g.vehicle_count; i++) {
        data_source_seek(&g.sources[i], (float)seconds);
        vehicle_reset_trail(&g.vehicles[i]);
    }
    memset(g.corr, 0, sizeof(g.corr));
}

EMSCRIPTEN_KEEPALIVE
double hawkeye_get_duration(void) {
    if (!g.log_loaded) return 0.0;
    uint64_t range = wasm_replay_duration_us(&g.replays[0]);
    return range == 0 ? 0.0 : (double)range / 1e6;
}

EMSCRIPTEN_KEEPALIVE
double hawkeye_get_position(void) {
    if (!g.log_loaded) return 0.0;
    return (double)g.sources[g.selected].playback.position_s;
}
