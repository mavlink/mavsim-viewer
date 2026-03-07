#include "scene.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdio.h>
#include <math.h>

#define GROUND_SIZE 5000.0f   // half-size in meters (10km total)

// Grid shared settings
#define GRID_EXTENT      500.0f
#define GRID_SPACING     10.0f
#define GRID_MAJOR_EVERY 5

// Grid mode colors
#define GRID_SKY       (Color){ 56,  56,  60, 255 }
#define GRID_GROUND    (Color){ 32,  32,  34, 255 }
#define GRID_MINOR     (Color){ 97,  97,  97, 128 }
#define GRID_MAJOR     (Color){ 143, 143, 143, 128 }
#define GRID_AXIS_X    (Color){ 200,  60,  60, 180 }
#define GRID_AXIS_Z    (Color){ 60,   60, 200, 180 }

// Rez mode colors
#define REZ_SKY        (Color){ 12,  12,  18, 255 }  // near-black
#define REZ_GROUND     (Color){ 2,    2,   4, 255 }  // black
#define REZ_MINOR      (Color){ 0,  204, 218, 50 }   // teal, subtle
#define REZ_MAJOR      (Color){ 0,  204, 218, 140 }  // teal, bright
#define REZ_AXIS_X     (Color){ 0,  204, 218, 220 }  // teal, full
#define REZ_AXIS_Z     (Color){ 0,  204, 218, 220 }  // teal, full

// 1988 mode colors (synthwave)
#define SYNTH_SKY      (Color){ 8,   8,  20, 255 }
#define SYNTH_GROUND   (Color){ 5,   5,  16, 255 }
#define SYNTH_MINOR    (Color){ 255, 20, 100, 50 }   // hot pink, subtle
#define SYNTH_MAJOR    (Color){ 255, 20, 100, 160 }   // hot pink, bright
#define SYNTH_AXIS_X   (Color){ 255, 20, 100, 220 }   // hot pink, full
#define SYNTH_AXIS_Z   (Color){ 255, 20, 100, 220 }   // hot pink, full

void scene_init(scene_t *s) {
    s->cam_mode = CAM_MODE_CHASE;
    s->view_mode = VIEW_GRID;
    s->chase_distance = 3.0f;
    s->chase_yaw = 0.0f;
    s->chase_pitch = 0.4f;  // ~23° above horizontal
    s->fpv_yaw = 0.0f;
    s->fpv_pitch = 0.0f;

    // Camera
    s->camera = (Camera3D){
        .position = (Vector3){0.0f, 5.0f, 10.0f},
        .target   = (Vector3){0.0f, 0.0f, 0.0f},
        .up       = (Vector3){0.0f, 1.0f, 0.0f},
        .fovy     = 60.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    // Grid shader and plane (100x100 subdivisions for fwidth() precision)
    s->grid_shader = LoadShader("shaders/grid.vs", "shaders/grid.fs");
    s->loc_colGround  = GetShaderLocation(s->grid_shader, "colGround");
    s->loc_colMinor   = GetShaderLocation(s->grid_shader, "colMinor");
    s->loc_colMajor   = GetShaderLocation(s->grid_shader, "colMajor");
    s->loc_colAxisX   = GetShaderLocation(s->grid_shader, "colAxisX");
    s->loc_colAxisZ   = GetShaderLocation(s->grid_shader, "colAxisZ");
    s->loc_spacing    = GetShaderLocation(s->grid_shader, "spacing");
    s->loc_majorEvery = GetShaderLocation(s->grid_shader, "majorEvery");
    s->loc_axisWidth  = GetShaderLocation(s->grid_shader, "axisWidth");
    s->loc_matModel   = GetShaderLocation(s->grid_shader, "matModel");

    Mesh grid_mesh = GenMeshPlane(GROUND_SIZE * 2, GROUND_SIZE * 2, 100, 100);
    s->grid_plane = LoadModelFromMesh(grid_mesh);
    s->grid_plane.materials[0].shader = s->grid_shader;
}

static void update_chase_camera(scene_t *s, Vector3 pos) {
    float dist = s->chase_distance;

    // Spherical coordinates around the target
    float cam_x = pos.x + dist * cosf(s->chase_pitch) * sinf(s->chase_yaw);
    float cam_y = pos.y + dist * sinf(s->chase_pitch);
    float cam_z = pos.z + dist * cosf(s->chase_pitch) * cosf(s->chase_yaw);

    s->camera.target = pos;
    s->camera.position = (Vector3){cam_x, cam_y, cam_z};
    s->camera.up = (Vector3){0, 1, 0};
}

static void update_fpv_camera(scene_t *s, Vector3 pos, Quaternion rot) {
    s->camera.position = pos;

    // Start with vehicle forward and up
    Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, -1}, rot);
    Vector3 up = Vector3RotateByQuaternion((Vector3){0, 1, 0}, rot);
    Vector3 right = Vector3CrossProduct(forward, up);

    // Apply gimbal pitch offset (rotate around vehicle's right axis)
    if (s->fpv_pitch != 0.0f) {
        Quaternion pitch_q = QuaternionFromAxisAngle(right, s->fpv_pitch);
        forward = Vector3RotateByQuaternion(forward, pitch_q);
        up = Vector3RotateByQuaternion(up, pitch_q);
    }

    // Apply gimbal yaw offset (rotate around world up axis)
    if (s->fpv_yaw != 0.0f) {
        Quaternion yaw_q = QuaternionFromAxisAngle((Vector3){0, 1, 0}, -s->fpv_yaw);
        forward = Vector3RotateByQuaternion(forward, yaw_q);
        up = Vector3RotateByQuaternion(up, yaw_q);
    }

    s->camera.target = Vector3Add(pos, forward);
    s->camera.up = up;
}

void scene_update_camera(scene_t *s, Vector3 vehicle_pos, Quaternion vehicle_rot) {
    switch (s->cam_mode) {
        case CAM_MODE_CHASE:
            update_chase_camera(s, vehicle_pos);
            break;
        case CAM_MODE_FPV:
            update_fpv_camera(s, vehicle_pos, vehicle_rot);
            break;
        default:
            break;
    }
}

void scene_handle_input(scene_t *s) {
    if (IsKeyPressed(KEY_C)) {
        s->cam_mode = (s->cam_mode + 1) % CAM_MODE_COUNT;
        const char *names[] = {"Chase", "FPV"};
        printf("Camera: %s\n", names[s->cam_mode]);

        s->camera.up = (Vector3){0, 1, 0};
        s->fpv_yaw = 0.0f;
        s->fpv_pitch = 0.0f;
    }

    if (IsKeyPressed(KEY_V)) {
        // If in hidden mode, return to Grid; otherwise cycle public modes
        if (s->view_mode >= VIEW_COUNT)
            s->view_mode = VIEW_GRID;
        else
            s->view_mode = (s->view_mode + 1) % VIEW_COUNT;
        const char *names[] = {"Grid", "Rez"};
        printf("View: %s\n", names[s->view_mode]);
    }

    // Ctrl+1988 sequence detection for hidden mode
    if (IsKeyDown(KEY_LEFT_CONTROL)) {
        int expected[] = { KEY_ONE, KEY_NINE, KEY_EIGHT, KEY_EIGHT };
        if (s->seq_1988 < 4 && IsKeyPressed(expected[s->seq_1988])) {
            s->seq_1988++;
            if (s->seq_1988 == 4) {
                s->view_mode = (s->view_mode == VIEW_1988) ? VIEW_GRID : VIEW_1988;
                s->seq_1988 = 0;
            }
        } else if (IsKeyPressed(KEY_ONE) || IsKeyPressed(KEY_TWO) || IsKeyPressed(KEY_THREE) ||
                   IsKeyPressed(KEY_FOUR) || IsKeyPressed(KEY_FIVE) || IsKeyPressed(KEY_SIX) ||
                   IsKeyPressed(KEY_SEVEN) || IsKeyPressed(KEY_EIGHT) || IsKeyPressed(KEY_NINE) ||
                   IsKeyPressed(KEY_ZERO)) {
            s->seq_1988 = IsKeyPressed(KEY_ONE) ? 1 : 0;
        }
    } else {
        s->seq_1988 = 0;
    }

    // Mouse drag to orbit/look (left button)
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        if (s->cam_mode == CAM_MODE_CHASE) {
            s->chase_yaw   -= delta.x * 0.005f;
            s->chase_pitch += delta.y * 0.005f;
            if (s->chase_pitch < -1.2f) s->chase_pitch = -1.2f;
            if (s->chase_pitch > 1.4f) s->chase_pitch = 1.4f;
        } else if (s->cam_mode == CAM_MODE_FPV) {
            s->fpv_yaw   -= delta.x * 0.005f;
            s->fpv_pitch -= delta.y * 0.005f;
            // Full yaw, pitch only downward (0 = forward, -PI/2 = straight down)
            if (s->fpv_pitch < -1.57f) s->fpv_pitch = -1.57f;
            if (s->fpv_pitch > 0.0f) s->fpv_pitch = 0.0f;
        }
    }

    // Scroll wheel FOV zoom
    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        s->camera.fovy -= wheel * 5.0f;
        if (s->camera.fovy < 10.0f) s->camera.fovy = 10.0f;
        if (s->camera.fovy > 120.0f) s->camera.fovy = 120.0f;
    }
}

static void color_to_vec4(Color c, float out[4]) {
    out[0] = c.r / 255.0f;
    out[1] = c.g / 255.0f;
    out[2] = c.b / 255.0f;
    out[3] = c.a / 255.0f;
}

static void draw_shader_grid(const scene_t *s,
    Color ground, Color minor, Color major, Color axis_x, Color axis_z)
{
    float v[4];
    color_to_vec4(ground, v); SetShaderValue(s->grid_shader, s->loc_colGround, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(minor, v);  SetShaderValue(s->grid_shader, s->loc_colMinor, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(major, v);  SetShaderValue(s->grid_shader, s->loc_colMajor, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(axis_x, v); SetShaderValue(s->grid_shader, s->loc_colAxisX, v, SHADER_UNIFORM_VEC4);
    color_to_vec4(axis_z, v); SetShaderValue(s->grid_shader, s->loc_colAxisZ, v, SHADER_UNIFORM_VEC4);

    float spacing = GRID_SPACING;
    float majorEvery = (float)GRID_MAJOR_EVERY;
    float axisWidth = 1.5f;
    SetShaderValue(s->grid_shader, s->loc_spacing, &spacing, SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->grid_shader, s->loc_majorEvery, &majorEvery, SHADER_UNIFORM_FLOAT);
    SetShaderValue(s->grid_shader, s->loc_axisWidth, &axisWidth, SHADER_UNIFORM_FLOAT);

    // Pass identity model matrix (plane is at origin)
    Matrix model = MatrixIdentity();
    SetShaderValueMatrix(s->grid_shader, s->loc_matModel, model);

    DrawModel(s->grid_plane, (Vector3){0, 0, 0}, 1.0f, WHITE);
}

void scene_draw(const scene_t *s) {
    if (s->view_mode == VIEW_GRID) {
        draw_shader_grid(s, GRID_GROUND, GRID_MINOR, GRID_MAJOR, GRID_AXIS_X, GRID_AXIS_Z);
    } else if (s->view_mode == VIEW_REZ) {
        draw_shader_grid(s, REZ_GROUND, REZ_MINOR, REZ_MAJOR, REZ_AXIS_X, REZ_AXIS_Z);
    } else if (s->view_mode == VIEW_1988) {
        draw_shader_grid(s, SYNTH_GROUND, SYNTH_MINOR, SYNTH_MAJOR, SYNTH_AXIS_X, SYNTH_AXIS_Z);
    }
}

void scene_draw_sky(const scene_t *s) {
    switch (s->view_mode) {
        case VIEW_GRID: ClearBackground(GRID_SKY); break;
        case VIEW_REZ:  ClearBackground(REZ_SKY);   break;
        case VIEW_1988: ClearBackground(SYNTH_SKY); break;
        default:        ClearBackground(GRID_SKY); break;
    }
}

void scene_cleanup(scene_t *s) {
    UnloadModel(s->grid_plane);
    UnloadShader(s->grid_shader);
}
