#include "vehicle.h"
#include "raymath.h"
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>

#define EARTH_RADIUS 6371000.0

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

void vehicle_init(vehicle_t *v, vehicle_type_t type) {
    memset(v, 0, sizeof(*v));
    v->type = type;
    v->position = (Vector3){0};
    v->rotation = QuaternionIdentity();
    v->origin_set = false;
    v->model_scale = model_scales[type];
    v->red_material_idx = -1;

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

    // jMAVSim local position: X=North, Y=East, Z=Up (same as MAVLinkDisplayOnly.java)
    double jmav_x = EARTH_RADIUS * (lat - v->lat0);               // North
    double jmav_y = EARTH_RADIUS * (lon - v->lon0) * cos(v->lat0); // East
    double jmav_z = alt - v->alt0;                                  // Up

    // jMAVSim frame (X=North,Y=East,Z=Up) → Raylib (X=right,Y=up,Z=back)
    // jMAVSim X (North) → Raylib -Z
    // jMAVSim Y (East)  → Raylib +X
    // jMAVSim Z (Up)    → Raylib +Y
    v->position.x = (float)jmav_y;
    v->position.y = (float)jmav_z;
    if (v->position.y < 0.0f) v->position.y = 0.0f;
    v->position.z = (float)(-jmav_x);

    // MAVLink quaternion: w,x,y,z in NED frame
    // NED rotation axes: X=North, Y=East, Z=Down
    // Raylib rotation axes: X=East, Y=Up, Z=South
    // Mapping: NED_X→-Ray_Z, NED_Y→Ray_X, NED_Z→-Ray_Y
    float qw = state->quaternion[0];
    float qx = state->quaternion[1]; // NED X (North) → -Raylib Z
    float qy = state->quaternion[2]; // NED Y (East)  → +Raylib X
    float qz = state->quaternion[3]; // NED Z (Down)  → -Raylib Y

    v->rotation.w = qw;
    v->rotation.x = qy;
    v->rotation.y = -qz;
    v->rotation.z = -qx;

    // Derived telemetry from NED quaternion
    // Normalize quaternion sign so qw >= 0 (avoids flipped Euler angles)
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
    v->altitude_rel = (float)((state->alt * 1e-3) - v->alt0);
}

void vehicle_draw(vehicle_t *v, view_mode_t view_mode) {
    // In Rez mode, swap red arms to orange
    Color saved_red = {0};
    if (view_mode == VIEW_REZ && v->red_material_idx >= 0) {
        Color *c = &v->model.materials[v->red_material_idx].maps[MATERIAL_MAP_DIFFUSE].color;
        saved_red = *c;
        *c = (Color){ 255, 106, 0, 255 }; // #ff6a00 orange
    }
    // OBJ model: flat in XY, thin in Z (Z is model's up).
    // Raylib: Y is up. Rotate +90° around X so model Z → Raylib Y.
    Matrix base_rot = MatrixRotateX(90.0f * DEG2RAD);
    Matrix rot = QuaternionToMatrix(v->rotation);
    Matrix scale = MatrixScale(v->model_scale, v->model_scale, v->model_scale);
    Matrix trans = MatrixTranslate(v->position.x, v->position.y, v->position.z);

    // Transform = Scale * BaseRot * AttitudeRot * Translation
    v->model.transform = MatrixMultiply(MatrixMultiply(MatrixMultiply(scale, base_rot), rot), trans);

    DrawModel(v->model, (Vector3){0}, 1.0f, WHITE);

    // Restore original color
    if (view_mode == VIEW_REZ && v->red_material_idx >= 0) {
        v->model.materials[v->red_material_idx].maps[MATERIAL_MAP_DIFFUSE].color = saved_red;
    }
}

void vehicle_cleanup(vehicle_t *v) {
    UnloadModel(v->model);
}
