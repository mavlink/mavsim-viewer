#include "vehicle.h"
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
#define TRAIL_MAX 10000
#define TRAIL_INTERVAL 0.05f
#define TRAIL_DIST_INTERVAL 0.01f  // meters between ribbon samples

// ── Model registry ──────────────────────────────────────────────────────────
// To add a new model: append an entry here and increment nothing else.
const vehicle_model_info_t vehicle_models[] = {
    //  path                                name            scale  pitch    yaw
    { "models/px4_quadrotor.obj",         "Quadrotor",    1.0f,    0.0f,   0.0f },
    { "models/cessna.obj",                "Fixed-wing",   1.33f,   0.0f,  90.0f },
    { "models/x_vert.obj",                "Tailsitter",   1.0f,  -90.0f,  90.0f },
    { "models/fpv_quadrotor.obj",         "FPV Quad",     0.75f,   0.0f,   0.0f },
    { "models/px4_hexarotor.obj",         "Hexarotor",    1.05f,   0.0f,   0.0f },
    { "models/vtol_wing.obj",             "VTOL",         1.5f,    0.0f, 180.0f },
    { "models/rover_4.obj",              "Rover",        1.0f,    0.0f,   0.0f },
};
const int vehicle_model_count = sizeof(vehicle_models) / sizeof(vehicle_models[0]);

// ── Material remapping ──────────────────────────────────────────────────────
// Heuristic color detection works across OBJ models with standard MTL colors.
static void remap_materials(vehicle_t *v) {
    v->red_material_idx = -1;
    for (int i = 0; i < v->model.materialCount; i++) {
        Color *c = &v->model.materials[i].maps[MATERIAL_MAP_DIFFUSE].color;
        // Blue_Metal → #272fc5
        if (c->b > 100 && c->r < 50)
            *c = (Color){ 39, 47, 197, 255 };
        // Red_Metal → #ff2f2b
        else if (c->r > 100 && c->g < 50 && c->b < 50) {
            *c = (Color){ 255, 47, 43, 255 };
            v->red_material_idx = i;
        }
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

    // Unload previous model if loaded
    if (v->model.meshCount > 0)
        UnloadModel(v->model);

    const vehicle_model_info_t *info = &vehicle_models[model_idx];
    v->model_idx = model_idx;
    v->model_scale = info->scale;
    v->pitch_offset_deg = info->pitch_offset_deg;
    v->yaw_offset_deg = info->yaw_offset_deg;

    v->model = LoadModel(info->path);
    if (v->model.meshCount == 0)
        printf("Warning: failed to load model %s\n", info->path);

    remap_materials(v);
    printf("Model: %s\n", info->name);
}

void vehicle_cycle_model(vehicle_t *v) {
    int next = (v->model_idx + 1) % vehicle_model_count;
    vehicle_load_model(v, next);
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

    v->lighting_shader = lighting_shader;
    v->loc_matNormal = -1;
    if (lighting_shader.id > 0) {
        v->loc_matNormal = GetShaderLocation(lighting_shader, "matNormal");
    }

    vehicle_load_model(v, model_idx);
}

void vehicle_update(vehicle_t *v, const hil_state_t *state) {
    if (!state->valid) return;

    double lat = state->lat * 1e-7 * (M_PI / 180.0);
    double lon = state->lon * 1e-7 * (M_PI / 180.0);
    double alt = state->alt * 1e-3;

    if (!v->origin_set) {
        v->lat0 = lat;
        v->lon0 = lon;
        v->alt0 = alt;
        v->origin_set = true;
    }

    v->active = true;

    // Local NED position relative to origin
    double jmav_x = EARTH_RADIUS * (lat - v->lat0);                // North
    double jmav_y = EARTH_RADIUS * (lon - v->lon0) * cos(v->lat0); // East
    double jmav_z = alt - v->alt0;                                   // Up

    // NED frame → Raylib (X=right, Y=up, Z=back)
    v->position.x = (float)jmav_y;
    v->position.y = (float)jmav_z;
    if (v->position.y < 0.0f) v->position.y = 0.0f;
    v->position.z = (float)(-jmav_x);

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

    // Sample trail by time OR distance (whichever triggers first)
    float dist_since = 0.0f;
    if (v->trail_count > 0) {
        int last = (v->trail_head - 1 + v->trail_capacity) % v->trail_capacity;
        float ddx = v->position.x - v->trail[last].x;
        float ddy = v->position.y - v->trail[last].y;
        float ddz = v->position.z - v->trail[last].z;
        dist_since = sqrtf(ddx*ddx + ddy*ddy + ddz*ddz);
    }
    v->trail_timer += GetFrameTime();
    if (v->trail_timer >= TRAIL_INTERVAL || dist_since >= TRAIL_DIST_INTERVAL) {
        v->trail_timer = 0.0f;
        v->trail[v->trail_head] = v->position;
        v->trail_roll[v->trail_head] = v->roll_deg;
        v->trail_pitch[v->trail_head] = v->pitch_deg;
        v->trail_vert[v->trail_head] = v->vertical_speed;
        v->trail_speed[v->trail_head] = sqrtf(v->ground_speed * v->ground_speed +
                                               v->vertical_speed * v->vertical_speed);
        v->trail_head = (v->trail_head + 1) % v->trail_capacity;
        if (v->trail_count < v->trail_capacity) v->trail_count++;
    }
}

void vehicle_draw(vehicle_t *v, view_mode_t view_mode, bool selected,
                  int trail_mode, bool show_ground_track, Vector3 cam_pos) {
    // In Rez/1988 mode, swap red arm color
    Color saved_red = {0};
    if ((view_mode == VIEW_REZ || view_mode == VIEW_1988) && v->red_material_idx >= 0) {
        Color *c = &v->model.materials[v->red_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
        saved_red = *c;
        if (view_mode == VIEW_1988)
            *c = (Color){ 255, 20, 100, 255 }; // hot pink
        else
            *c = (Color){ 255, 106, 0, 255 }; // #ff6a00 orange
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

    DrawModel(v->model, (Vector3){0}, 1.0f, WHITE);

    // Draw path trail (mode 1) or speed ribbon (mode 2)
    if (trail_mode > 0 && v->trail_count > 1) {
        int start = (v->trail_count < v->trail_capacity)
            ? 0
            : v->trail_head;

      if (trail_mode == 1) {
        // ── Normal directional trail ──
        Color trail_color;
        Color col_back, col_up, col_down, col_roll_pos, col_roll_neg;
        if (view_mode == VIEW_1988) {
            trail_color  = (Color){ 255, 220,  60, 160 };  // warm yellow forward
            col_back     = (Color){ 180,  40, 255, 255 };  // violet
            col_up       = (Color){   0, 240, 255, 255 };  // cyan
            col_down     = (Color){ 255, 140,   0, 255 };  // orange
            col_roll_pos = (Color){  40, 255,  80, 255 };  // green (starboard)
            col_roll_neg = (Color){ 255,  40,  80, 255 };  // red (port)
        } else if (view_mode == VIEW_REZ) {
            trail_color  = (Color){   0, 255, 200, 160 };  // teal forward
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

            Color c;
            c.r = (unsigned char)(cr > 255 ? 255 : cr);
            c.g = (unsigned char)(cg > 255 ? 255 : cg);
            c.b = (unsigned char)(cb > 255 ? 255 : cb);
            c.a = (unsigned char)(t * trail_color.a);
            DrawLine3D(v->trail[idx0], v->trail[idx1], c);
        }
      } else {
        // ── Speed ribbon trail (mode 2) ──
        float max_speed = 27.78f;  // 100 km/h in m/s
        float max_half_w = v->model_scale * 0.5f;
        float min_half_w = 0.02f;

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

            Vector3 up = { 0, 1, 0 };
            Vector3 perp = { dir.z * up.y - dir.y * up.z,
                             dir.x * up.z - dir.z * up.x,
                             dir.y * up.x - dir.x * up.y };
            float plen = sqrtf(perp.x*perp.x + perp.y*perp.y + perp.z*perp.z);
            if (plen < 0.001f) {
                Vector3 to_cam = { cam_pos.x - p0.x, cam_pos.y - p0.y, cam_pos.z - p0.z };
                perp = (Vector3){ dir.y * to_cam.z - dir.z * to_cam.y,
                                  dir.z * to_cam.x - dir.x * to_cam.z,
                                  dir.x * to_cam.y - dir.y * to_cam.x };
                plen = sqrtf(perp.x*perp.x + perp.y*perp.y + perp.z*perp.z);
                if (plen < 0.001f) continue;
            }
            perp.x /= plen; perp.y /= plen; perp.z /= plen;

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

            Vector3 a = { p0.x + perp.x*hw0, p0.y + perp.y*hw0, p0.z + perp.z*hw0 };
            Vector3 b = { p0.x - perp.x*hw0, p0.y - perp.y*hw0, p0.z - perp.z*hw0 };
            Vector3 d = { p1.x + perp.x*hw1, p1.y + perp.y*hw1, p1.z + perp.z*hw1 };
            Vector3 e = { p1.x - perp.x*hw1, p1.y - perp.y*hw1, p1.z - perp.z*hw1 };

            DrawTriangle3D(a, b, d, c);
            DrawTriangle3D(b, e, d, c);
            DrawTriangle3D(d, b, a, c);
            DrawTriangle3D(d, e, b, c);
        }
      }
    }

    // Ground projection (shadow / ring) at Y=0
    if (show_ground_track && v->position.y > 0.1f) {
        Vector3 ground = { v->position.x, 0.02f, v->position.z };
        float radius = 0.3f + v->position.y * 0.02f;
        if (radius > 1.5f) radius = 1.5f;

        // Dotted vertical drop line
        float dash = 0.08f;
        float gap = 0.12f;
        Color drop;

        Color fill_col, edge_col;
        if (view_mode == VIEW_GRID) {
            fill_col = (Color){ 0, 0, 0, 50 };
            edge_col = (Color){ 60, 60, 60, 80 };
            drop = (Color){ 150, 150, 150, 40 };
        } else if (view_mode == VIEW_1988) {
            fill_col = (Color){ 255, 20, 100, 30 };
            edge_col = (Color){ 255, 20, 100, 120 };
            drop = (Color){ 255, 20, 100, 50 };
        } else {
            fill_col = (Color){ 0, 204, 218, 30 };
            edge_col = (Color){ 0, 204, 218, 120 };
            drop = (Color){ 0, 204, 218, 50 };
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

    // Restore original color
    if ((view_mode == VIEW_REZ || view_mode == VIEW_1988) && v->red_material_idx >= 0) {
        v->model.materials[v->red_material_idx].maps[MATERIAL_MAP_DIFFUSE].color = saved_red;
    }
}

void vehicle_reset_trail(vehicle_t *v) {
    v->trail_count = 0;
    v->trail_head = 0;
    v->trail_timer = 0.0f;
}

void vehicle_cleanup(vehicle_t *v) {
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
