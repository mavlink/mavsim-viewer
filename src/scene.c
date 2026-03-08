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

// Snow mode colors (outdoor high-contrast)
#define SNOW_SKY       (Color){ 240, 242, 245, 255 }
#define SNOW_GROUND    (Color){ 228, 230, 233, 255 }
#define SNOW_MINOR     (Color){ 155, 160, 168, 140 }
#define SNOW_MAJOR     (Color){ 70,  75,  85, 200 }
#define SNOW_AXIS_X    (Color){ 210,  30,  30, 230 }
#define SNOW_AXIS_Z    (Color){ 30,   30, 210, 230 }

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
    s->ortho_mode = ORTHO_NONE;
    s->ortho_span = 60.0f;
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

    // Vehicle lighting shader — single directional light
    s->lighting_shader = LoadShader("shaders/lighting.vs", "shaders/lighting.fs");
    s->loc_lightDir  = GetShaderLocation(s->lighting_shader, "lightDir");
    s->loc_ambient   = GetShaderLocation(s->lighting_shader, "ambient");
    s->loc_matNormal = GetShaderLocation(s->lighting_shader, "matNormal");

    // Sun from upper-left-front
    Vector3 sun = Vector3Normalize((Vector3){ -0.4f, 0.8f, -0.3f });
    SetShaderValue(s->lighting_shader, s->loc_lightDir, &sun, SHADER_UNIFORM_VEC3);
    float ambient = 0.35f;
    SetShaderValue(s->lighting_shader, s->loc_ambient, &ambient, SHADER_UNIFORM_FLOAT);
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

static void update_ortho_camera(scene_t *s, Vector3 pos) {
    float span = s->ortho_span;
    s->camera.projection = CAMERA_ORTHOGRAPHIC;
    s->camera.fovy = span;

    switch (s->ortho_mode) {
        case ORTHO_TOP:
            s->camera.position = (Vector3){ pos.x, pos.y + 100.0f, pos.z };
            s->camera.target = pos;
            s->camera.up = (Vector3){ 0, 0, -1 };
            break;
        case ORTHO_BOTTOM:
            s->camera.position = (Vector3){ pos.x, pos.y - 100.0f, pos.z };
            s->camera.target = pos;
            s->camera.up = (Vector3){ 0, 0, 1 };
            break;
        case ORTHO_FRONT:
            s->camera.position = (Vector3){ pos.x, pos.y, pos.z - 100.0f };
            s->camera.target = pos;
            s->camera.up = (Vector3){ 0, 1, 0 };
            break;
        case ORTHO_BACK:
            s->camera.position = (Vector3){ pos.x, pos.y, pos.z + 100.0f };
            s->camera.target = pos;
            s->camera.up = (Vector3){ 0, 1, 0 };
            break;
        case ORTHO_LEFT:
            s->camera.position = (Vector3){ pos.x - 100.0f, pos.y, pos.z };
            s->camera.target = pos;
            s->camera.up = (Vector3){ 0, 1, 0 };
            break;
        case ORTHO_RIGHT:
            s->camera.position = (Vector3){ pos.x + 100.0f, pos.y, pos.z };
            s->camera.target = pos;
            s->camera.up = (Vector3){ 0, 1, 0 };
            break;
        default:
            break;
    }
}

void scene_update_camera(scene_t *s, Vector3 vehicle_pos, Quaternion vehicle_rot) {
    if (s->ortho_mode != ORTHO_NONE) {
        update_ortho_camera(s, vehicle_pos);
        return;
    }

    s->camera.projection = CAMERA_PERSPECTIVE;
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
    // Alt+number: fullscreen ortho views
    if (IsKeyDown(KEY_LEFT_ALT) || IsKeyDown(KEY_RIGHT_ALT)) {
        static const int keys[] = { KEY_ONE, KEY_TWO, KEY_THREE, KEY_FOUR, KEY_FIVE, KEY_SIX, KEY_SEVEN };
        static const ortho_mode_t modes[] = { ORTHO_NONE, ORTHO_TOP, ORTHO_FRONT, ORTHO_LEFT, ORTHO_RIGHT, ORTHO_BOTTOM, ORTHO_BACK };
        static const char *names[] = { "Perspective", "Top", "Front", "Left", "Right", "Bottom", "Back" };
        for (int i = 0; i < 7; i++) {
            if (IsKeyPressed(keys[i])) {
                s->ortho_mode = modes[i];
                printf("View: %s\n", names[i]);
                break;
            }
        }
        // Alt+scroll to zoom ortho span
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f && s->ortho_mode != ORTHO_NONE) {
            s->ortho_span -= wheel * 10.0f;
            if (s->ortho_span < 10.0f) s->ortho_span = 10.0f;
            if (s->ortho_span > 500.0f) s->ortho_span = 500.0f;
        }
    }

    if (IsKeyPressed(KEY_C)) {
        s->ortho_mode = ORTHO_NONE;  // return to perspective on camera toggle
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
        const char *names[] = {"Grid", "Rez", "Snow"};
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

    // Scroll wheel FOV zoom (only in perspective mode, not when Alt held)
    if (!IsKeyDown(KEY_LEFT_ALT) && !IsKeyDown(KEY_RIGHT_ALT) && s->ortho_mode == ORTHO_NONE) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) {
            s->camera.fovy -= wheel * 5.0f;
            if (s->camera.fovy < 10.0f) s->camera.fovy = 10.0f;
            if (s->camera.fovy > 120.0f) s->camera.fovy = 120.0f;
        }
    }
    // Scroll wheel ortho span zoom (fullscreen ortho, no Alt needed)
    if (s->ortho_mode != ORTHO_NONE && !IsKeyDown(KEY_LEFT_ALT) && !IsKeyDown(KEY_RIGHT_ALT)) {
        float wheel = GetMouseWheelMove();
        if (wheel != 0.0f) {
            s->ortho_span -= wheel * 10.0f;
            if (s->ortho_span < 10.0f) s->ortho_span = 10.0f;
            if (s->ortho_span > 500.0f) s->ortho_span = 500.0f;
        }
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
    } else if (s->view_mode == VIEW_SNOW) {
        draw_shader_grid(s, SNOW_GROUND, SNOW_MINOR, SNOW_MAJOR, SNOW_AXIS_X, SNOW_AXIS_Z);
    } else if (s->view_mode == VIEW_1988) {
        draw_shader_grid(s, SYNTH_GROUND, SYNTH_MINOR, SYNTH_MAJOR, SYNTH_AXIS_X, SYNTH_AXIS_Z);
    }

    // Fullscreen ortho: distance grid + ground line
    if (s->ortho_mode != ORTHO_NONE) {
        float ext = s->ortho_span * 2.0f;
        float spacing = 10.0f;
        if (s->ortho_span > 200.0f) spacing = 50.0f;
        else if (s->ortho_span > 80.0f) spacing = 20.0f;
        else if (s->ortho_span < 20.0f) spacing = 2.0f;

        Color grid_minor = { 40, 40, 55, 60 };
        Color grid_major = { 60, 60, 80, 100 };
        Vector3 center = s->camera.target;

        if (s->ortho_mode == ORTHO_TOP || s->ortho_mode == ORTHO_BOTTOM) {
            float snap_x = floorf(center.x / spacing) * spacing;
            float snap_z = floorf(center.z / spacing) * spacing;
            for (float g = -ext; g <= ext; g += spacing) {
                bool major = fabsf(fmodf(snap_x + g, spacing * 5)) < 0.1f;
                DrawLine3D((Vector3){snap_x + g, 0.01f, center.z - ext},
                           (Vector3){snap_x + g, 0.01f, center.z + ext}, major ? grid_major : grid_minor);
            }
            for (float g = -ext; g <= ext; g += spacing) {
                bool major = fabsf(fmodf(snap_z + g, spacing * 5)) < 0.1f;
                DrawLine3D((Vector3){center.x - ext, 0.01f, snap_z + g},
                           (Vector3){center.x + ext, 0.01f, snap_z + g}, major ? grid_major : grid_minor);
            }
        } else if (s->ortho_mode == ORTHO_FRONT || s->ortho_mode == ORTHO_BACK) {
            float snap_x = floorf(center.x / spacing) * spacing;
            float snap_y = floorf(center.y / spacing) * spacing;
            float z = center.z;
            for (float g = -ext; g <= ext; g += spacing) {
                bool major = fabsf(fmodf(snap_x + g, spacing * 5)) < 0.1f;
                DrawLine3D((Vector3){snap_x + g, center.y - ext, z},
                           (Vector3){snap_x + g, center.y + ext, z}, major ? grid_major : grid_minor);
            }
            for (float g = -ext; g <= ext; g += spacing) {
                bool major = fabsf(fmodf(snap_y + g, spacing * 5)) < 0.1f;
                DrawLine3D((Vector3){center.x - ext, snap_y + g, z},
                           (Vector3){center.x + ext, snap_y + g, z}, major ? grid_major : grid_minor);
            }
        } else {
            // Left / Right: ZY grid
            float snap_z = floorf(center.z / spacing) * spacing;
            float snap_y = floorf(center.y / spacing) * spacing;
            float x = center.x;
            for (float g = -ext; g <= ext; g += spacing) {
                bool major = fabsf(fmodf(snap_z + g, spacing * 5)) < 0.1f;
                DrawLine3D((Vector3){x, center.y - ext, snap_z + g},
                           (Vector3){x, center.y + ext, snap_z + g}, major ? grid_major : grid_minor);
            }
            for (float g = -ext; g <= ext; g += spacing) {
                bool major = fabsf(fmodf(snap_y + g, spacing * 5)) < 0.1f;
                DrawLine3D((Vector3){x, snap_y + g, center.z - ext},
                           (Vector3){x, snap_y + g, center.z + ext}, major ? grid_major : grid_minor);
            }
        }

        // Ground line for side views (fill is drawn as 2D overlay after EndMode3D)
        if (s->ortho_mode >= ORTHO_FRONT && s->ortho_mode <= ORTHO_RIGHT) {
            Color gnd_line = { 120, 120, 150, 200 };
            DrawLine3D((Vector3){-ext, 0, -ext}, (Vector3){ ext, 0, -ext}, gnd_line);
            DrawLine3D((Vector3){-ext, 0,  ext}, (Vector3){ ext, 0,  ext}, gnd_line);
            DrawLine3D((Vector3){-ext, 0, -ext}, (Vector3){-ext, 0,  ext}, gnd_line);
            DrawLine3D((Vector3){ ext, 0, -ext}, (Vector3){ ext, 0,  ext}, gnd_line);
        }
    }
}

void scene_draw_ortho_ground(const scene_t *s, int screen_w, int screen_h) {
    // Only for side ortho views (front/back/left/right)
    if (s->ortho_mode < ORTHO_FRONT || s->ortho_mode > ORTHO_RIGHT) return;

    // Project Y=0 world position to screen Y
    Vector3 ground_pt = s->camera.target;
    ground_pt.y = 0.0f;
    Vector2 screen_pos = GetWorldToScreen(ground_pt, s->camera);
    int ground_y = (int)screen_pos.y;

    // Clamp to screen bounds
    if (ground_y < 0) ground_y = 0;
    if (ground_y > screen_h) ground_y = screen_h;

    // Dark fill from just below ground line to bottom
    int fill_h = screen_h - ground_y - 1;
    if (fill_h > 0) {
        DrawRectangle(0, ground_y + 1, screen_w, fill_h, (Color){ 2, 2, 6, 180 });
    }
}

void scene_draw_sky(const scene_t *s) {
    switch (s->view_mode) {
        case VIEW_GRID: ClearBackground(GRID_SKY); break;
        case VIEW_REZ:  ClearBackground(REZ_SKY);   break;
        case VIEW_SNOW: ClearBackground(SNOW_SKY); break;
        case VIEW_1988: ClearBackground(SYNTH_SKY); break;
        default:        ClearBackground(GRID_SKY); break;
    }
}

void scene_cleanup(scene_t *s) {
    UnloadModel(s->grid_plane);
    UnloadShader(s->grid_shader);
    UnloadShader(s->lighting_shader);
}
