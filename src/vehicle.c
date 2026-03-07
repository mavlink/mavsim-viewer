#include "vehicle.h"
#include "raymath.h"
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define EARTH_RADIUS 6371000.0
#define TRAIL_MAX 2000
#define TRAIL_INTERVAL 0.05f

static const char *model_paths[] = {
    [VEHICLE_MULTICOPTER] = "models/3dr_arducopter_quad_x.obj",
    [VEHICLE_FIXEDWING]   = "models/cessna.obj",
    [VEHICLE_TAILSITTER]  = "models/x_vert.obj",
};

static const float model_scales[] = {
    [VEHICLE_MULTICOPTER] = 1.0f,
    [VEHICLE_FIXEDWING]   = 1.0f,
    [VEHICLE_TAILSITTER]  = 1.0f,
};

// Per-model yaw correction (degrees) applied after the Z-up → Y-up base rotation.
// Aligns each model's nose with the -Z (forward/North) direction in Raylib space.
static const float model_yaw_offset_deg[] = {
    [VEHICLE_MULTICOPTER] = 0.0f,
    [VEHICLE_FIXEDWING]   = 90.0f,
    [VEHICLE_TAILSITTER]  = 0.0f,
};

void vehicle_init(vehicle_t *v, vehicle_type_t type) {
    memset(v, 0, sizeof(*v));
    v->type = type;
    v->position = (Vector3){0};
    v->rotation = QuaternionIdentity();
    v->origin_set = false;
    v->active = false;
    v->model_scale = model_scales[type];
    v->red_material_idx = -1;
    v->color = WHITE;
    v->trail = (Vector3 *)calloc(TRAIL_MAX, sizeof(Vector3));
    v->trail_roll = (float *)calloc(TRAIL_MAX, sizeof(float));
    v->trail_pitch = (float *)calloc(TRAIL_MAX, sizeof(float));
    v->trail_vert = (float *)calloc(TRAIL_MAX, sizeof(float));
    v->trail_capacity = TRAIL_MAX;
    v->trail_count = 0;
    v->trail_head = 0;
    v->trail_timer = 0.0f;

    v->model = LoadModel(model_paths[type]);
    if (v->model.meshCount == 0) {
        printf("Warning: failed to load model %s\n", model_paths[type]);
    }

    // Remap material colors
    // MTL order: 0=default, 1=Blue_Metal, 2=Gray_Plastic, 3=Red_Metal, 4=Textolite
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

    // Sample trail position at fixed interval
    v->trail_timer += GetFrameTime();
    if (v->trail_timer >= TRAIL_INTERVAL) {
        v->trail_timer = 0.0f;
        v->trail[v->trail_head] = v->position;
        v->trail_roll[v->trail_head] = v->roll_deg;
        v->trail_pitch[v->trail_head] = v->pitch_deg;
        v->trail_vert[v->trail_head] = v->vertical_speed;
        v->trail_head = (v->trail_head + 1) % v->trail_capacity;
        if (v->trail_count < v->trail_capacity) v->trail_count++;
    }
}

void vehicle_draw(vehicle_t *v, view_mode_t view_mode, bool selected) {
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
        MatrixRotateX(90.0f * DEG2RAD),
        MatrixRotateY(model_yaw_offset_deg[v->type] * DEG2RAD));
    Matrix rot = QuaternionToMatrix(v->rotation);
    Matrix scale = MatrixScale(v->model_scale, v->model_scale, v->model_scale);
    Matrix trans = MatrixTranslate(v->position.x, v->position.y, v->position.z);

    // Transform = Scale * BaseRot * AttitudeRot * Translation
    v->model.transform = MatrixMultiply(MatrixMultiply(MatrixMultiply(scale, base_rot), rot), trans);

    DrawModel(v->model, (Vector3){0}, 1.0f, WHITE);

    // Draw path trail
    if (v->trail_count > 1) {
        int start = (v->trail_count < v->trail_capacity)
            ? 0
            : v->trail_head;
        Color trail_color;
        if (view_mode == VIEW_1988)
            trail_color = (Color){ 21, 190, 254, 160 };   // teal
        else if (view_mode == VIEW_REZ)
            trail_color = (Color){ 255, 106, 0, 160 };    // orange
        else
            trail_color = (Color){ 255, 200, 50, 180 };   // yellow
        // Per-mode color palette
        Color col_back, col_up, col_down, col_roll_pos, col_roll_neg;
        if (view_mode == VIEW_1988) {
            col_back     = (Color){ 255, 20, 100, 255 };  // pink
            col_up       = (Color){   0, 255, 140, 255 }; // mint green
            col_down     = (Color){ 255, 106,   0, 255 }; // orange
            col_roll_pos = (Color){ 255, 106,   0, 255 }; // orange
            col_roll_neg = (Color){  40,  80, 255, 255 }; // blue
        } else if (view_mode == VIEW_REZ) {
            col_back     = (Color){ 180,  60, 220, 255 }; // purple
            col_up       = (Color){  80, 220, 120, 255 }; // green
            col_down     = (Color){ 220,  60,  40, 255 }; // red
            col_roll_pos = (Color){ 255,  40,  40, 255 }; // red
            col_roll_neg = (Color){  40,  80, 255, 255 }; // blue
        } else {
            col_back     = (Color){ 180,  80, 255, 255 }; // purple
            col_up       = (Color){  80, 255, 120, 255 }; // green
            col_down     = (Color){ 255,  80,  40, 255 }; // orange-red
            col_roll_pos = (Color){ 255,  40,  40, 255 }; // red
            col_roll_neg = (Color){  40,  80, 255, 255 }; // blue
        }

        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;
            float t = (float)i / (float)v->trail_count;

            float pitch = v->trail_pitch[idx1];
            float vert  = v->trail_vert[idx1];
            float roll  = v->trail_roll[idx1];

            // Start with forward color as base
            float cr = (float)trail_color.r;
            float cg = (float)trail_color.g;
            float cb = (float)trail_color.b;

            // Backward: pitch > 0 means nose up / moving backward
            float back_t = pitch / 15.0f;
            if (back_t < 0.0f) back_t = 0.0f;
            if (back_t > 1.0f) back_t = 1.0f;
            cr += (col_back.r - cr) * back_t;
            cg += (col_back.g - cg) * back_t;
            cb += (col_back.b - cb) * back_t;

            // Vertical: ascending / descending
            float vert_t = vert / 5.0f;  // ±5 m/s = full tint
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

            // Roll tint on top
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
    v->trail = NULL;
    v->trail_roll = NULL;
    v->trail_pitch = NULL;
    v->trail_vert = NULL;
}
