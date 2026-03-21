#include "vehicle.h"
#include "asset_path.h"
#include "raymath.h"
#include "rlgl.h"
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define EARTH_RADIUS 6371000.0
#define TRAIL_MAX 1800
#define TRAIL_INTERVAL 0.016f
#define TRAIL_DIST_INTERVAL 0.01f  // meters between ribbon samples

// ── Model registry ──────────────────────────────────────────────────────────
// To add a new model: append an entry here and increment nothing else.
const vehicle_model_info_t vehicle_models[] = {
    //  path                                name            scale  pitch    yaw       group
    { "models/px4_quadrotor.obj",         "Quadrotor",    1.0f,    0.0f, 180.0f,   GROUP_QUAD },
    { "models/px4_fixed_wing.obj",        "Fixed-wing",   1.15f,   0.0f, 180.0f,   GROUP_FIXED_WING },
    { "models/px4_tailsitter.obj",        "Tailsitter",   1.0f,    0.0f, 180.0f,   GROUP_TAILSITTER },
    { "models/fpv_quadrotor.obj",         "FPV Quad",     0.75f,   0.0f,   0.0f,   GROUP_QUAD },
    { "models/px4_hexarotor.obj",          "Hexarotor",    0.9f,    0.0f,   0.0f,   GROUP_HEX },
    { "models/fpv_hexarotor.obj",         "FPV Hex",      0.85f,   0.0f,   0.0f,   GROUP_HEX },
    { "models/vtol_wing.obj",             "VTOL",         1.5f,    0.0f, 180.0f,   GROUP_VTOL },
    { "models/rover_4.obj",              "Rover",        1.0f,    0.0f,   0.0f,   GROUP_ROVER },
    { "models/rov.obj",                  "ROV",          1.0f,    0.0f, 180.0f,   GROUP_ROV },
};
const int vehicle_model_count = sizeof(vehicle_models) / sizeof(vehicle_models[0]);

// ── Material remapping ──────────────────────────────────────────────────────
// Heuristic color detection works across OBJ models with standard MTL colors.
static void remap_materials(vehicle_t *v) {
    v->red_material_idx = -1;
    v->green_material_idx = -1;
    v->front_material_idx = -1;
    v->back_material_idx = -1;
    for (int i = 0; i < v->model.materialCount; i++) {
        Color *c = &v->model.materials[i].maps[MATERIAL_MAP_DIFFUSE].color;
        // Yellow_Metal (front arms) → #FFC832 — matches Grid trail forward
        if (c->r > 200 && c->g > 100 && c->b < 100) {
            *c = (Color){ 255, 200, 50, 255 };
            v->front_material_idx = i;
        }
        // Purple_Metal (back arms) → #A03CFF — matches Grid trail backward
        else if (c->b > 200 && c->r > 100 && c->g < 100) {
            *c = (Color){ 160, 60, 255, 255 };
            v->back_material_idx = i;
        }
        // Green_Metal (starboard/side arms) → #29FF4F
        else if (c->g > 200 && c->r < 100 && c->b < 100) {
            *c = (Color){ 41, 255, 79, 255 };
            v->green_material_idx = i;
        }
        // Red_Metal (port/side arms) → #CC2121
        else if (c->r > 100 && c->g < 50 && c->b < 50) {
            *c = (Color){ 204, 33, 33, 255 };
            v->red_material_idx = i;
        }
        // Blue_Metal (legacy) → #272fc5
        else if (c->b > 100 && c->r < 50)
            *c = (Color){ 39, 47, 197, 255 };
        // Gray_Plastic (props) → #5075a2
        else if (c->r > 60 && c->r < 140 && c->g > 60 && c->g < 140)
            *c = (Color){ 80, 117, 162, 255 };
        // Textolite (body, near-black) → #0e1f2f
        else if (c->r < 20 && c->g < 20 && c->b < 20)
            *c = (Color){ 14, 31, 47, 255 };
    }

    // Assign lighting shader to all materials (shader stored in vehicle_init)
    if (v->lighting_shader.id > 0) {
        for (int i = 0; i < v->model.materialCount; i++) {
            v->model.materials[i].shader = v->lighting_shader;
        }
    }
}

// ── Model loading ───────────────────────────────────────────────────────────
void vehicle_load_model(vehicle_t *v, int model_idx) {
    // Clamp to valid range
    if (model_idx < 0 || model_idx >= vehicle_model_count)
        model_idx = 0;

    // Unload previous model if loaded.
    // Reset material shaders to default first — UnloadModel calls UnloadMaterial
    // which destroys any non-default shader. Our shared lighting_shader would be
    // deleted, hanging the GPU on subsequent draws.
    if (v->model.meshCount > 0) {
        for (int i = 0; i < v->model.materialCount; i++)
            v->model.materials[i].shader.id = rlGetShaderIdDefault();
        UnloadModel(v->model);
    }

    const vehicle_model_info_t *info = &vehicle_models[model_idx];
    v->model_idx = model_idx;
    v->model_scale = info->scale;
    v->pitch_offset_deg = info->pitch_offset_deg;
    v->yaw_offset_deg = info->yaw_offset_deg;

    char model_path[512];
    asset_path(info->path, model_path, sizeof(model_path));
    v->model = LoadModel(model_path);
    if (v->model.meshCount == 0)
        printf("Warning: failed to load model %s\n", info->path);

    remap_materials(v);
    printf("Model: %s\n", info->name);
}

void vehicle_cycle_model(vehicle_t *v) {
    model_group_t group = v->model_group;
    int start = v->model_idx;
    int next = start;
    do {
        next = (next + 1) % vehicle_model_count;
    } while (vehicle_models[next].group != group && next != start);
    if (next != start) {
        vehicle_load_model(v, next);
    }
}

void vehicle_set_type(vehicle_t *v, uint8_t mav_type) {
    model_group_t group;
    int default_model;

    switch (mav_type) {
        case 2:  // MAV_TYPE_QUADROTOR
            group = GROUP_QUAD;
            default_model = MODEL_QUADROTOR;
            break;
        case 13: // MAV_TYPE_HEXAROTOR
        case 14: // MAV_TYPE_OCTOROTOR
            group = GROUP_HEX;
            default_model = MODEL_HEXAROTOR;
            break;
        case 1:  // MAV_TYPE_FIXED_WING
            group = GROUP_FIXED_WING;
            default_model = MODEL_FIXEDWING;
            break;
        case 19: // MAV_TYPE_VTOL_TAILSITTER_DUOROTOR
        case 20: // MAV_TYPE_VTOL_TAILSITTER_QUADROTOR
        case 23: // MAV_TYPE_VTOL_TAILSITTER
            group = GROUP_TAILSITTER;
            default_model = MODEL_TAILSITTER;
            break;
        case 21: // MAV_TYPE_VTOL_TILTROTOR
        case 22: // MAV_TYPE_VTOL_FIXEDROTOR
            group = GROUP_VTOL;
            default_model = MODEL_VTOL;
            break;
        case 10: // MAV_TYPE_GROUND_ROVER
        case 11: // MAV_TYPE_SURFACE_BOAT
            group = GROUP_ROVER;
            default_model = MODEL_ROVER;
            break;
        case 12: // MAV_TYPE_SUBMARINE
            group = GROUP_ROV;
            default_model = MODEL_ROV;
            break;
        default:
            group = GROUP_QUAD;
            default_model = MODEL_QUADROTOR;
            break;
    }

    v->model_group = group;
    if (v->model_idx != default_model) {
        vehicle_load_model(v, default_model);
    }
}

// ── Thermal palette (per-theme) ─────────────────────────────────────────────
static Color heat_to_color(float heat, unsigned char alpha, view_mode_t mode) {
    float cr, cg, cb;

    if (mode == VIEW_1988) {
        // 1988: neon pink → hot magenta → red → orange → yellow → white
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
        // Rez: indigo → teal-purple → cyan-red → orange → amber → white
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
        // Snow: deep navy → royal blue → teal → green → yellow → red
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
        // Grid (default): purple → magenta → red → orange → yellow → white
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

// ── Init / update / draw ────────────────────────────────────────────────────
void vehicle_init(vehicle_t *v, int model_idx, Shader lighting_shader) {
    memset(v, 0, sizeof(*v));
    v->position = (Vector3){0};
    v->rotation = QuaternionIdentity();
    v->origin_set = false;
    v->active = false;
    v->red_material_idx = -1;
    v->green_material_idx = -1;
    v->front_material_idx = -1;
    v->back_material_idx = -1;
    v->color = WHITE;
    v->trail = (Vector3 *)calloc(TRAIL_MAX, sizeof(Vector3));
    v->trail_roll = (float *)calloc(TRAIL_MAX, sizeof(float));
    v->trail_pitch = (float *)calloc(TRAIL_MAX, sizeof(float));
    v->trail_vert = (float *)calloc(TRAIL_MAX, sizeof(float));
    v->trail_speed = (float *)calloc(TRAIL_MAX, sizeof(float));
    v->trail_capacity = TRAIL_MAX;
    v->trail_count = 0;
    v->trail_head = 0;
    v->trail_timer = 0.0f;
    v->trail_speed_max = 0.0f;

    v->lighting_shader = lighting_shader;
    v->loc_matNormal = -1;
    v->ghost_alpha = 1.0f;
    v->loc_ghost_alpha = -1;
    if (lighting_shader.id > 0) {
        v->loc_matNormal = GetShaderLocation(lighting_shader, "matNormal");
        v->loc_ghost_alpha = GetShaderLocation(lighting_shader, "ghostAlpha");
        float one = 1.0f;
        SetShaderValue(lighting_shader, v->loc_ghost_alpha, &one, SHADER_UNIFORM_FLOAT);
    }

    vehicle_load_model(v, model_idx);
}

void vehicle_update(vehicle_t *v, const hil_state_t *state, const home_position_t *home) {
    if (!state->valid) return;

    double lat = state->lat * 1e-7 * (M_PI / 180.0);
    double lon = state->lon * 1e-7 * (M_PI / 180.0);
    double alt = state->alt * 1e-3;

    if (!v->origin_set) {
        // Wait for HOME_POSITION so we get the correct ground altitude.
        // Fall back to current altitude after ~1 second (20 HIL updates at 22Hz).
        v->origin_wait_count++;
        if (home && home->valid && (lat != 0.0 || lon != 0.0)) {
            v->lat0 = lat;
            v->lon0 = lon;
            v->alt0 = alt;
            v->origin_set = true;

        } else if (v->origin_wait_count > 20) {
            v->lat0 = lat;
            v->lon0 = lon;
            v->alt0 = alt;
            v->origin_set = true;

        } else {
            return;  // skip this update, wait for home
        }
    }

    v->active = true;

    // Local NED position relative to origin
    double jmav_x = EARTH_RADIUS * (lat - v->lat0);                // North
    double jmav_y = EARTH_RADIUS * (lon - v->lon0) * cos(v->lat0); // East
    double jmav_z = alt - v->alt0;                                   // Up

    // NED frame → Raylib (X=right, Y=up, Z=back) + grid deconfliction offset
    v->position.x = (float)jmav_y + v->grid_offset.x;
    v->position.y = (float)jmav_z + v->grid_offset.y;
    if (v->position.y < 0.0f) v->position.y = 0.0f;
    v->position.z = (float)(-jmav_x) + v->grid_offset.z;

    // MAVLink quaternion: w,x,y,z in NED frame → Raylib
    float qw = state->quaternion[0];
    float qx = state->quaternion[1];
    float qy = state->quaternion[2];
    float qz = state->quaternion[3];

    v->rotation.w = qw;
    v->rotation.x = qy;
    v->rotation.y = -qz;
    v->rotation.z = -qx;

    // Derived telemetry from NED quaternion
    float nw = qw, nx = qx, ny = qy, nz = qz;
    if (nw < 0.0f) { nw = -nw; nx = -nx; ny = -ny; nz = -nz; }

    float heading_rad = atan2f(2.0f * (nw * nz + nx * ny),
                               1.0f - 2.0f * (ny * ny + nz * nz));
    v->heading_deg = heading_rad * RAD2DEG;
    if (v->heading_deg < 0.0f) v->heading_deg += 360.0f;

    v->roll_deg = atan2f(2.0f * (nw * nx + ny * nz),
                         1.0f - 2.0f * (nx * nx + ny * ny)) * RAD2DEG;

    float sin_pitch = 2.0f * (nw * ny - nz * nx);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    v->pitch_deg = asinf(sin_pitch) * RAD2DEG;

    v->ground_speed = sqrtf((float)state->vx * state->vx +
                            (float)state->vy * state->vy) * 0.01f;
    v->vertical_speed = -state->vz * 0.01f;
    v->airspeed = state->ind_airspeed * 0.01f;
    v->altitude_rel = (float)(alt - v->alt0);

    // Adaptive trail sampling: record a point when direction changes (tight turns
    // get dense coverage) or after a max distance on straight runs (so they don't
    // go bare). Minimum distance gate prevents duplicate points when hovering.
    float dist_since = 0.0f;
    Vector3 cur_dir = {0};
    if (v->trail_count > 0) {
        int last = (v->trail_head - 1 + v->trail_capacity) % v->trail_capacity;
        float ddx = v->position.x - v->trail[last].x;
        float ddy = v->position.y - v->trail[last].y;
        float ddz = v->position.z - v->trail[last].z;
        dist_since = sqrtf(ddx*ddx + ddy*ddy + ddz*ddz);
        if (dist_since > 0.001f) {
            cur_dir = (Vector3){ ddx/dist_since, ddy/dist_since, ddz/dist_since };
        }
    }

    // Direction change: dot product < threshold means significant turn
    float dir_dot = 1.0f;
    if (v->trail_count > 1 && dist_since > 0.001f) {
        dir_dot = cur_dir.x * v->trail_last_dir.x +
                  cur_dir.y * v->trail_last_dir.y +
                  cur_dir.z * v->trail_last_dir.z;
    }

    // Sample triggers: dense on turns, sparse on straights
    v->trail_timer += GetFrameTime();
    bool dir_trigger  = (dir_dot < 0.995f) && (dist_since >= TRAIL_DIST_INTERVAL);
    bool dist_trigger = (dist_since >= 0.5f);
    bool time_trigger = (v->trail_timer >= TRAIL_INTERVAL * 4.0f);

    if (dir_trigger || dist_trigger || time_trigger) {
        v->trail_timer = 0.0f;
        if (dist_since > 0.001f) v->trail_last_dir = cur_dir;
        v->trail[v->trail_head] = v->position;
        v->trail_roll[v->trail_head] = v->roll_deg;
        v->trail_pitch[v->trail_head] = v->pitch_deg;
        v->trail_vert[v->trail_head] = v->vertical_speed;
        float spd = sqrtf(v->ground_speed * v->ground_speed +
                          v->vertical_speed * v->vertical_speed);
        v->trail_speed[v->trail_head] = spd;
        if (spd > v->trail_speed_max) v->trail_speed_max = spd;
        bool was_full = (v->trail_count >= v->trail_capacity);
        v->trail_head = (v->trail_head + 1) % v->trail_capacity;
        if (v->trail_count < v->trail_capacity) v->trail_count++;

        // Recompute max when buffer wraps (old peak may have scrolled out)
        if (was_full) {
            float mx = 0.0f;
            for (int i = 0; i < v->trail_count; i++)
                if (v->trail_speed[i] > mx) mx = v->trail_speed[i];
            v->trail_speed_max = mx;
        }
    }
}

void vehicle_draw(vehicle_t *v, view_mode_t view_mode, bool selected,
                  int trail_mode, bool show_ground_track, Vector3 cam_pos,
                  bool classic_colors) {
    // Per-mode arm recoloring: front/back arms match trail forward/backward colors
    Color saved_front = {0}, saved_back = {0}, saved_red = {0}, saved_green = {0};
    bool recolor_arms = true;
    if (recolor_arms) {
        Color front_col, back_col, side_col, red_col = {0}, green_col = {0};
        if (classic_colors) {
            // Classic: red front, blue back, body-dark sides (light grey for VTOL winglets)
            Color classic_side = (Color){ 14, 31, 47, 255 };  // body dark
            if (view_mode == VIEW_1988) {
                front_col = (Color){ 255,  20, 100, 255 };  // hot pink
                back_col  = (Color){  39,  47, 197, 255 };  // blue
            } else if (view_mode == VIEW_REZ) {
                front_col = (Color){ 255, 106,   0, 255 };  // orange-red
                back_col  = (Color){  39,  47, 197, 255 };  // blue
            } else if (view_mode == VIEW_SNOW) {
                front_col = (Color){ 200,  30,  30, 255 };  // bold red
                back_col  = (Color){  30,  60, 180, 255 };  // blue
                classic_side = (Color){ 160, 160, 170, 255 }; // light grey
            } else {
                front_col = (Color){ 255,  47,  43, 255 };  // red
                back_col  = (Color){  39,  47, 197, 255 };  // blue
            }
            side_col = classic_side;
        } else {
            // Modern: yellow front, purple back, red port, green starboard
            if (view_mode == VIEW_1988) {
                front_col = (Color){ 255, 220,  60, 255 };  // warm yellow
                back_col  = (Color){ 180,  40, 255, 255 };  // violet
                red_col   = (Color){ 255,  20, 100, 255 };  // hot pink
                green_col = (Color){  40, 255, 100, 255 };  // neon green
            } else if (view_mode == VIEW_REZ) {
                front_col = (Color){ 220, 180,  30, 255 };  // muted gold
                back_col  = (Color){ 160,  40, 240, 255 };  // purple
                red_col   = (Color){ 255, 106,   0, 255 };  // orange-red
                green_col = (Color){  30, 200,  80, 255 };  // muted green
            } else if (view_mode == VIEW_SNOW) {
                front_col = (Color){ 200, 140,  20, 255 };  // dark amber
                back_col  = (Color){ 140,  20, 200, 255 };  // purple
                red_col   = (Color){ 180,  30,  30, 255 };  // dark red
                green_col = (Color){  20, 160,  50, 255 };  // dark green
            } else {
                front_col = (Color){ 255, 200,  50, 255 };  // yellow
                back_col  = (Color){ 160,  60, 255, 255 };  // purple
                red_col   = (Color){ 204,  33,  33, 255 };  // red
                green_col = (Color){  41, 255,  79, 255 };  // green
            }
            side_col = (Color){ 0, 0, 0, 0 };  // unused in modern
        }
        if (v->front_material_idx >= 0) {
            Color *c = &v->model.materials[v->front_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
            saved_front = *c;
            *c = front_col;
        }
        if (v->back_material_idx >= 0) {
            Color *c = &v->model.materials[v->back_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
            saved_back = *c;
            *c = back_col;
        }
        if (classic_colors) {
            // In classic mode, sides become body-dark (or light grey for VTOL)
            if (v->red_material_idx >= 0) {
                Color *c = &v->model.materials[v->red_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
                saved_red = *c;
                *c = side_col;
            }
            if (v->green_material_idx >= 0) {
                Color *c = &v->model.materials[v->green_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
                saved_green = *c;
                *c = side_col;
            }
        } else {
            // Modern: per-view-mode red/green
            if (v->red_material_idx >= 0) {
                Color *c = &v->model.materials[v->red_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
                saved_red = *c;
                *c = red_col;
            }
            if (v->green_material_idx >= 0) {
                Color *c = &v->model.materials[v->green_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
                saved_green = *c;
                *c = green_col;
            }
        }
    }
    // OBJ model: flat in XY, thin in Z (Z is model's up).
    // Raylib: Y is up. Rotate +90° around X so model Z → Raylib Y,
    // then apply per-model yaw correction to align nose with -Z (forward).
    Matrix base_rot = MatrixMultiply(
        MatrixMultiply(
            MatrixRotateX(90.0f * DEG2RAD),
            MatrixRotateZ(v->pitch_offset_deg * DEG2RAD)),
        MatrixRotateY(v->yaw_offset_deg * DEG2RAD));
    Matrix rot = QuaternionToMatrix(v->rotation);
    Matrix scale = MatrixScale(v->model_scale, v->model_scale, v->model_scale);
    Matrix trans = MatrixTranslate(v->position.x, v->position.y, v->position.z);

    // Transform = Scale * BaseRot * AttitudeRot * Translation
    Matrix rot_only = MatrixMultiply(base_rot, rot);
    v->model.transform = MatrixMultiply(MatrixMultiply(scale, rot_only), trans);

    // Set normal matrix (rotation only, no scale/translation) before drawing
    if (v->loc_matNormal >= 0) {
        SetShaderValueMatrix(v->lighting_shader, v->loc_matNormal, rot_only);
    }

    // Set ghost alpha on shader
    if (v->loc_ghost_alpha >= 0) {
        SetShaderValue(v->lighting_shader, v->loc_ghost_alpha, &v->ghost_alpha, SHADER_UNIFORM_FLOAT);
    }

    if (v->ghost_alpha < 1.0f) rlDisableDepthMask();
    // Ghost drones get a 50% tint of their assigned color
    Color model_tint = WHITE;
    if (v->ghost_alpha < 1.0f) {
        float t = 0.5f;
        model_tint.r = (unsigned char)(255 * (1.0f - t) + v->color.r * t);
        model_tint.g = (unsigned char)(255 * (1.0f - t) + v->color.g * t);
        model_tint.b = (unsigned char)(255 * (1.0f - t) + v->color.b * t);
    }
    DrawModel(v->model, (Vector3){0}, 1.0f, model_tint);
    if (v->ghost_alpha < 1.0f) rlEnableDepthMask();

    // Draw path trail (mode 1), speed ribbon (mode 2), or drone color (mode 3)
    if (trail_mode > 0 && v->trail_count > 1) {
        int start = (v->trail_count < v->trail_capacity)
            ? 0
            : v->trail_head;

      if (trail_mode == 1) {
        // ── Normal directional trail ──
        Color trail_color;
        Color col_back, col_up, col_down, col_roll_pos, col_roll_neg;
        if (view_mode == VIEW_SNOW) {
            trail_color  = (Color){ 200, 140,  20, 200 };  // dark amber
            col_back     = (Color){ 140,  20, 200, 255 };  // purple
            col_up       = (Color){   0, 150,  60, 255 };  // dark green
            col_down     = (Color){ 200,  50,   0, 255 };  // dark red
            col_roll_pos = (Color){  20, 160,  40, 255 };  // green
            col_roll_neg = (Color){ 200,  20,  60, 255 };  // red
        } else if (view_mode == VIEW_1988) {
            trail_color  = (Color){ 255, 220,  60, 160 };  // warm yellow forward
            col_back     = (Color){ 180,  40, 255, 255 };  // violet
            col_up       = (Color){   0, 240, 255, 255 };  // cyan
            col_down     = (Color){ 255, 140,   0, 255 };  // orange
            col_roll_pos = (Color){  40, 255,  80, 255 };  // green (starboard)
            col_roll_neg = (Color){ 255,  40,  80, 255 };  // red (port)
        } else if (view_mode == VIEW_REZ) {
            trail_color  = (Color){ 220, 180,  30, 160 };  // muted gold forward
            col_back     = (Color){ 160,  40, 240, 255 };  // purple
            col_up       = (Color){   0, 200, 255, 255 };  // cyan
            col_down     = (Color){ 255, 160,   0, 255 };  // amber
            col_roll_pos = (Color){  40, 255, 100, 255 };  // green (starboard)
            col_roll_neg = (Color){ 255,  40,  80, 255 };  // red (port)
        } else {
            trail_color  = (Color){ 255, 200,  50, 180 };  // yellow forward
            col_back     = (Color){ 160,  60, 255, 255 };  // purple
            col_up       = (Color){   0, 220, 255, 255 };  // cyan
            col_down     = (Color){ 255, 140,   0, 255 };  // orange
            col_roll_pos = (Color){  40, 255,  80, 255 };  // green (starboard)
            col_roll_neg = (Color){ 255,  40,  80, 255 };  // red (port)
        }
        bool thick = (view_mode == VIEW_SNOW);

        // Batched trail: single rlBegin/rlEnd instead of per-segment DrawLine3D
        rlBegin(thick ? RL_TRIANGLES : RL_LINES);
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
            rlColor4ub(ccr, ccg, ccb, ca);
            if (!thick) {
                rlVertex3f(v->trail[idx0].x, v->trail[idx0].y, v->trail[idx0].z);
                rlVertex3f(v->trail[idx1].x, v->trail[idx1].y, v->trail[idx1].z);
            } else {
                // Flat ribbon for better visibility in snow mode
                Vector3 seg = Vector3Subtract(v->trail[idx1], v->trail[idx0]);
                Vector3 up = {0, 1, 0};
                Vector3 perp = Vector3CrossProduct(seg, up);
                float plen = Vector3Length(perp);
                if (plen < 0.001f) continue;
                float hw = 0.013f;
                perp = Vector3Scale(perp, hw / plen);
                Vector3 a = Vector3Add(v->trail[idx0], perp);
                Vector3 b = Vector3Subtract(v->trail[idx0], perp);
                Vector3 d = Vector3Add(v->trail[idx1], perp);
                Vector3 e = Vector3Subtract(v->trail[idx1], perp);
                rlVertex3f(a.x, a.y, a.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(d.x, d.y, d.z);
                rlVertex3f(b.x, b.y, b.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(d.x, d.y, d.z);
                rlVertex3f(d.x, d.y, d.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(a.x, a.y, a.z);
                rlVertex3f(d.x, d.y, d.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(b.x, b.y, b.z);
            }
        }
        rlEnd();
      } else if (trail_mode == 3) {
        // ── Drone-color trail: solid vehicle color with age fade ──
        rlBegin(RL_LINES);
        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;
            float t = (float)i / (float)v->trail_count;
            unsigned char ca = (unsigned char)(t * 200 * v->ghost_alpha);
            rlColor4ub(v->color.r, v->color.g, v->color.b, ca);
            rlVertex3f(v->trail[idx0].x, v->trail[idx0].y, v->trail[idx0].z);
            rlVertex3f(v->trail[idx1].x, v->trail[idx1].y, v->trail[idx1].z);
        }
        rlEnd();
      } else {
        // ── Speed ribbon trail (mode 2) ──
        // Batched triangle ribbon with adaptive sampling (dense on turns, sparse on straights).
        // Perpendicular blending (70% previous + 30% current) prevents twisting on tight turns.
        float max_speed = v->trail_speed_max > 1.0f ? v->trail_speed_max : 1.0f;
        float max_half_w = v->model_scale * 0.25f;
        float min_half_w = 0.02f;
        Vector3 prev_perp = {0};
        bool have_prev = false;

        rlBegin(RL_TRIANGLES);
        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;

            Vector3 p0 = v->trail[idx0];
            Vector3 p1 = v->trail[idx1];
            float spd0 = v->trail_speed[idx0];
            float spd1 = v->trail_speed[idx1];

            Vector3 seg = { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z };
            float seg_len = sqrtf(seg.x*seg.x + seg.y*seg.y + seg.z*seg.z);
            if (seg_len < 0.001f) continue;
            Vector3 dir = { seg.x/seg_len, seg.y/seg_len, seg.z/seg_len };

            // Compute perpendicular: use world-up cross for horizontal flight,
            // camera-based billboard for vertical flight
            float vt = fabsf(dir.y);
            Vector3 perp;
            if (vt < 0.7f) {
                Vector3 up = { 0, 1, 0 };
                perp = (Vector3){ dir.z * up.y - dir.y * up.z,
                                  dir.x * up.z - dir.z * up.x,
                                  dir.y * up.x - dir.x * up.y };
            } else {
                Vector3 fwd = { 0, 0, 1 };
                perp = (Vector3){ dir.y * fwd.z - dir.z * fwd.y,
                                  dir.z * fwd.x - dir.x * fwd.z,
                                  dir.x * fwd.y - dir.y * fwd.x };
            }
            float plen = sqrtf(perp.x*perp.x + perp.y*perp.y + perp.z*perp.z);
            if (plen < 0.001f) {
                if (have_prev) { perp = prev_perp; }
                else continue;
            } else {
                perp.x /= plen; perp.y /= plen; perp.z /= plen;
            }

            // Ensure consistent orientation then blend with previous to reduce twisting
            if (have_prev) {
                float dot = perp.x*prev_perp.x + perp.y*prev_perp.y + perp.z*prev_perp.z;
                if (dot < 0.0f) { perp.x = -perp.x; perp.y = -perp.y; perp.z = -perp.z; }
                float blend = 0.3f;
                perp.x = prev_perp.x * (1.0f - blend) + perp.x * blend;
                perp.y = prev_perp.y * (1.0f - blend) + perp.y * blend;
                perp.z = prev_perp.z * (1.0f - blend) + perp.z * blend;
                float nlen = sqrtf(perp.x*perp.x + perp.y*perp.y + perp.z*perp.z);
                if (nlen > 0.001f) { perp.x /= nlen; perp.y /= nlen; perp.z /= nlen; }
            }
            prev_perp = perp;
            have_prev = true;

            // Power law width: stays thin longer, ramps at high speed
            float s0 = spd0 / max_speed; if (s0 > 1.0f) s0 = 1.0f;
            float s1 = spd1 / max_speed; if (s1 > 1.0f) s1 = 1.0f;
            float hw0 = min_half_w + powf(s0, 2.0f) * max_half_w;
            float hw1 = min_half_w + powf(s1, 2.0f) * max_half_w;

            float spd_avg = (spd0 + spd1) * 0.5f;
            float accel = (spd1 - spd0) / TRAIL_INTERVAL;
            float accel_shift = accel / 40.0f;
            if (accel_shift > 0.15f) accel_shift = 0.15f;
            if (accel_shift < -0.15f) accel_shift = -0.15f;
            float heat = spd_avg / max_speed + accel_shift;
            if (heat > 1.0f) heat = 1.0f;
            if (heat < 0.0f) heat = 0.0f;

            float t = (float)i / (float)v->trail_count;
            Color c = heat_to_color(heat, (unsigned char)(t * 200), view_mode);
            c.a = (unsigned char)(c.a * v->ghost_alpha);

            Vector3 a = { p0.x + perp.x*hw0, p0.y + perp.y*hw0, p0.z + perp.z*hw0 };
            Vector3 b = { p0.x - perp.x*hw0, p0.y - perp.y*hw0, p0.z - perp.z*hw0 };
            Vector3 d = { p1.x + perp.x*hw1, p1.y + perp.y*hw1, p1.z + perp.z*hw1 };
            Vector3 e = { p1.x - perp.x*hw1, p1.y - perp.y*hw1, p1.z - perp.z*hw1 };

            rlColor4ub(c.r, c.g, c.b, c.a);
            rlVertex3f(a.x, a.y, a.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(d.x, d.y, d.z);
            rlVertex3f(b.x, b.y, b.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(d.x, d.y, d.z);
            rlVertex3f(d.x, d.y, d.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(a.x, a.y, a.z);
            rlVertex3f(d.x, d.y, d.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(b.x, b.y, b.z);
        }
        rlEnd();
      }
    }

    // Ground projection (shadow / ring) at Y=0
    if (show_ground_track && v->position.y > 0.1f) {

        Vector3 ground = { v->position.x, 0.02f, v->position.z };
        float radius = 1.0f + v->position.y * 0.1f;
        if (radius > 5.0f) radius = 5.0f;

        // Dotted vertical drop line
        float dash = 0.08f;
        float gap = 0.12f;
        Color drop;

        Color fill_col, edge_col;
        if (view_mode == VIEW_GRID) {
            fill_col = (Color){ 0, 0, 0, 100 };
            edge_col = (Color){ 80, 80, 80, 180 };
            drop = (Color){ 150, 150, 150, 120 };
        } else if (view_mode == VIEW_1988) {
            fill_col = (Color){ 255, 20, 100, 80 };
            edge_col = (Color){ 255, 20, 100, 200 };
            drop = (Color){ 255, 20, 100, 140 };
        } else {
            fill_col = (Color){ 0, 204, 218, 80 };
            edge_col = (Color){ 0, 204, 218, 200 };
            drop = (Color){ 0, 204, 218, 140 };
        }

        // Filled disc: concentric rings when close, cylinder when far
        float dx = cam_pos.x - ground.x;
        float dy = cam_pos.y - ground.y;
        float dz = cam_pos.z - ground.z;
        float cam_dist = sqrtf(dx*dx + dy*dy + dz*dz);
        if (cam_dist < 40.0f) {
            for (float r = 0.05f; r <= radius; r += 0.05f) {
                DrawCircle3D(ground, r, (Vector3){1, 0, 0}, 90.0f, fill_col);
            }
        } else {
            Vector3 cyl_pos = { ground.x, 0.01f, ground.z };
            DrawCylinder(cyl_pos, radius, radius, 0.25f, 36, fill_col);
        }
        DrawCircle3D(ground, radius, (Vector3){1, 0, 0}, 90.0f, edge_col);

        // Targeting cross
        float cs = radius * 0.5f;
        DrawLine3D((Vector3){ ground.x - cs, ground.y, ground.z },
                   (Vector3){ ground.x + cs, ground.y, ground.z }, edge_col);
        DrawLine3D((Vector3){ ground.x, ground.y, ground.z - cs },
                   (Vector3){ ground.x, ground.y, ground.z + cs }, edge_col);

        float y_top = v->position.y;
        float y_bot = 0.02f;
        float y = y_top;
        rlSetLineWidth(2.5f);
        while (y > y_bot) {
            float y_end = y - dash;
            if (y_end < y_bot) y_end = y_bot;
            DrawLine3D((Vector3){ v->position.x, y, v->position.z },
                       (Vector3){ v->position.x, y_end, v->position.z }, drop);
            y -= dash + gap;
        }
    }

    // Restore original colors
    if (recolor_arms) {
        if (v->front_material_idx >= 0)
            v->model.materials[v->front_material_idx].maps[MATERIAL_MAP_DIFFUSE].color = saved_front;
        if (v->back_material_idx >= 0)
            v->model.materials[v->back_material_idx].maps[MATERIAL_MAP_DIFFUSE].color = saved_back;
        if (v->red_material_idx >= 0)
            v->model.materials[v->red_material_idx].maps[MATERIAL_MAP_DIFFUSE].color = saved_red;
        if (v->green_material_idx >= 0)
            v->model.materials[v->green_material_idx].maps[MATERIAL_MAP_DIFFUSE].color = saved_green;
    }
}

void vehicle_reset_trail(vehicle_t *v) {
    v->trail_count = 0;
    v->trail_head = 0;
    v->trail_timer = 0.0f;
    v->trail_speed_max = 0.0f;
}

void vehicle_set_ghost_alpha(vehicle_t *v, float alpha) {
    v->ghost_alpha = alpha;
}

void vehicle_draw_correlation_curtain(
    const vehicle_t *va, const vehicle_t *vb,
    view_mode_t view_mode, Vector3 cam_pos) {
    if (va->trail_count < 2 || vb->trail_count < 2) return;

    int n = va->trail_count < vb->trail_count ? va->trail_count : vb->trail_count;
    if (n < 2) return;

    int start_a = (va->trail_count < va->trail_capacity) ? 0 : va->trail_head;
    int start_b = (vb->trail_count < vb->trail_capacity) ? 0 : vb->trail_head;

    Color ca = va->color;
    Color cb = vb->color;

    rlDisableDepthMask();
    rlBegin(RL_TRIANGLES);
    for (int i = 1; i < n; i++) {
        // Map index through fractional position for trail length alignment
        int idx_a0 = (start_a + (int)((float)(i - 1) / n * va->trail_count)) % va->trail_capacity;
        int idx_a1 = (start_a + (int)((float)i / n * va->trail_count)) % va->trail_capacity;
        int idx_b0 = (start_b + (int)((float)(i - 1) / n * vb->trail_count)) % vb->trail_capacity;
        int idx_b1 = (start_b + (int)((float)i / n * vb->trail_count)) % vb->trail_capacity;

        Vector3 pa0 = va->trail[idx_a0];
        Vector3 pa1 = va->trail[idx_a1];
        Vector3 pb0 = vb->trail[idx_b0];
        Vector3 pb1 = vb->trail[idx_b1];

        float t = (float)i / (float)n;
        unsigned char alpha_a = (unsigned char)(t * 255 * va->ghost_alpha);
        unsigned char alpha_b = (unsigned char)(t * 255 * vb->ghost_alpha);

        // Front face: pa0 → pb0 → pa1, then pb0 → pb1 → pa1
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa0.x, pa0.y, pa0.z);
        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);

        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
        rlVertex3f(pb1.x, pb1.y, pb1.z);
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);

        // Back face: pa1 → pb0 → pa0, then pa1 → pb1 → pb0
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);
        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa0.x, pa0.y, pa0.z);

        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);
        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb1.x, pb1.y, pb1.z);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
    }
    rlEnd();
    rlEnableDepthMask();
}

void vehicle_draw_correlation_line(
    const vehicle_t *va, const vehicle_t *vb) {
    // Offset to model center of mass (model origin is at bottom)
    float off_a = va->model_scale * 0.15f;
    float off_b = vb->model_scale * 0.15f;
    Vector3 pa = { va->position.x, va->position.y + off_a, va->position.z };
    Vector3 pb = { vb->position.x, vb->position.y + off_b, vb->position.z };

    Vector3 seg = { pb.x - pa.x, pb.y - pa.y, pb.z - pa.z };
    float seg_len = sqrtf(seg.x*seg.x + seg.y*seg.y + seg.z*seg.z);
    if (seg_len < 0.001f) return;

    Vector3 dir = { seg.x/seg_len, seg.y/seg_len, seg.z/seg_len };

    // Build two perpendicular vectors to form the cylinder cross-section
    Vector3 perp1, perp2;
    if (fabsf(dir.y) < 0.9f) {
        Vector3 up = {0, 1, 0};
        perp1 = (Vector3){ dir.y*up.z - dir.z*up.y,
                           dir.z*up.x - dir.x*up.z,
                           dir.x*up.y - dir.y*up.x };
    } else {
        Vector3 right = {1, 0, 0};
        perp1 = (Vector3){ dir.y*right.z - dir.z*right.y,
                           dir.z*right.x - dir.x*right.z,
                           dir.x*right.y - dir.y*right.x };
    }
    float p1len = sqrtf(perp1.x*perp1.x + perp1.y*perp1.y + perp1.z*perp1.z);
    perp1.x /= p1len; perp1.y /= p1len; perp1.z /= p1len;
    perp2 = (Vector3){ dir.y*perp1.z - dir.z*perp1.y,
                       dir.z*perp1.x - dir.x*perp1.z,
                       dir.x*perp1.y - dir.y*perp1.x };

    Color ca = va->color;
    Color cb = vb->color;
    unsigned char alpha_a = (unsigned char)(220 * va->ghost_alpha);
    unsigned char alpha_b = (unsigned char)(220 * vb->ghost_alpha);
    float radius = 0.04f;
    int slices = 8;
    int rings = 6;

    // Precompute sin/cos for the circle
    float sin_t[9], cos_t[9];  // slices + 1
    for (int s = 0; s <= slices; s++) {
        float a = (float)s / slices * 2.0f * M_PI;
        sin_t[s] = sinf(a);
        cos_t[s] = cosf(a);
    }

    rlBegin(RL_TRIANGLES);
    for (int r = 0; r < rings; r++) {
        float t0 = (float)r / rings;
        float t1 = (float)(r + 1) / rings;

        // Interpolated positions along axis
        Vector3 c0 = { pa.x + seg.x*t0, pa.y + seg.y*t0, pa.z + seg.z*t0 };
        Vector3 c1 = { pa.x + seg.x*t1, pa.y + seg.y*t1, pa.z + seg.z*t1 };

        // Interpolated colors
        unsigned char r0 = (unsigned char)(ca.r + (cb.r - ca.r) * t0);
        unsigned char g0 = (unsigned char)(ca.g + (cb.g - ca.g) * t0);
        unsigned char b0 = (unsigned char)(ca.b + (cb.b - ca.b) * t0);
        unsigned char a0 = (unsigned char)(alpha_a + (alpha_b - alpha_a) * t0);
        unsigned char r1 = (unsigned char)(ca.r + (cb.r - ca.r) * t1);
        unsigned char g1 = (unsigned char)(ca.g + (cb.g - ca.g) * t1);
        unsigned char b1 = (unsigned char)(ca.b + (cb.b - ca.b) * t1);
        unsigned char a1 = (unsigned char)(alpha_a + (alpha_b - alpha_a) * t1);

        for (int s = 0; s < slices; s++) {
            // Four corners of this quad on the cylinder surface
            float ox0 = radius * (perp1.x*cos_t[s]   + perp2.x*sin_t[s]);
            float oy0 = radius * (perp1.y*cos_t[s]   + perp2.y*sin_t[s]);
            float oz0 = radius * (perp1.z*cos_t[s]   + perp2.z*sin_t[s]);
            float ox1 = radius * (perp1.x*cos_t[s+1] + perp2.x*sin_t[s+1]);
            float oy1 = radius * (perp1.y*cos_t[s+1] + perp2.y*sin_t[s+1]);
            float oz1 = radius * (perp1.z*cos_t[s+1] + perp2.z*sin_t[s+1]);

            // Triangle 1
            rlColor4ub(r0, g0, b0, a0);
            rlVertex3f(c0.x+ox0, c0.y+oy0, c0.z+oz0);
            rlColor4ub(r1, g1, b1, a1);
            rlVertex3f(c1.x+ox0, c1.y+oy0, c1.z+oz0);
            rlColor4ub(r0, g0, b0, a0);
            rlVertex3f(c0.x+ox1, c0.y+oy1, c0.z+oz1);

            // Triangle 2
            rlColor4ub(r0, g0, b0, a0);
            rlVertex3f(c0.x+ox1, c0.y+oy1, c0.z+oz1);
            rlColor4ub(r1, g1, b1, a1);
            rlVertex3f(c1.x+ox0, c1.y+oy0, c1.z+oz0);
            rlVertex3f(c1.x+ox1, c1.y+oy1, c1.z+oz1);
        }
    }
    rlEnd();
}

void vehicle_cleanup(vehicle_t *v) {
    for (int i = 0; i < v->model.materialCount; i++)
        v->model.materials[i].shader.id = rlGetShaderIdDefault();
    UnloadModel(v->model);
    free(v->trail);
    free(v->trail_roll);
    free(v->trail_pitch);
    free(v->trail_vert);
    free(v->trail_speed);
    v->trail = NULL;
    v->trail_roll = NULL;
    v->trail_pitch = NULL;
    v->trail_vert = NULL;
    v->trail_speed = NULL;
}
