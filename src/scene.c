#include "scene.h"
#include "raymath.h"
#include "rlgl.h"
#include <stdio.h>
#include <math.h>

#define GROUND_SIZE 5000.0f   // half-size in meters (10km total, matching jMAVSim)
#define GROUND_TILES 100.0f   // texture repeat count
#define SKY_RADIUS 6000.0f    // sky sphere radius (larger than ground diagonal)

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

void scene_init(scene_t *s) {
    s->cam_mode = CAM_MODE_CHASE;
    s->view_mode = VIEW_GRID;
    s->chase_distance = 3.0f;
    s->chase_yaw = 0.0f;
    s->chase_pitch = 0.4f;  // ~23° above horizontal

    // Camera
    s->camera = (Camera3D){
        .position = (Vector3){0.0f, 5.0f, 10.0f},
        .target   = (Vector3){0.0f, 0.0f, 0.0f},
        .up       = (Vector3){0.0f, 1.0f, 0.0f},
        .fovy     = 60.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    // Ground plane mesh (10x10 subdivisions to avoid diagonal tearing)
    Mesh ground_mesh = GenMeshPlane(GROUND_SIZE * 2, GROUND_SIZE * 2, 10, 10);
    s->ground = LoadModelFromMesh(ground_mesh);

    // Ground texture
    Image grass_img = LoadImage("textures/grass3.jpg");
    if (grass_img.data != NULL) {
        s->ground_tex = LoadTextureFromImage(grass_img);
        SetTextureFilter(s->ground_tex, TEXTURE_FILTER_TRILINEAR);
        SetTextureWrap(s->ground_tex, TEXTURE_WRAP_REPEAT);
        UnloadImage(grass_img);

        s->ground.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = s->ground_tex;

        // Scale UVs for tiling
        Mesh *m = &s->ground.meshes[0];
        for (int i = 0; i < m->vertexCount; i++) {
            m->texcoords[i * 2 + 0] *= GROUND_TILES;
            m->texcoords[i * 2 + 1] *= GROUND_TILES;
        }
        UpdateMeshBuffer(*m, 1, m->texcoords, m->vertexCount * 2 * sizeof(float), 0);
    }

    // Sky sphere — we draw from inside with backface culling disabled
    Mesh sky_mesh = GenMeshSphere(SKY_RADIUS, 36, 36);
    s->sky_sphere = LoadModelFromMesh(sky_mesh);

    // Sky texture
    Image sky_img = LoadImage("textures/HDR_040_Field_Bg.jpg");
    if (sky_img.data != NULL) {
        s->sky_tex = LoadTextureFromImage(sky_img);
        SetTextureFilter(s->sky_tex, TEXTURE_FILTER_BILINEAR);
        UnloadImage(sky_img);

        s->sky_sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = s->sky_tex;
    }

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
    Vector3 forward = Vector3RotateByQuaternion((Vector3){0, 0, -1}, rot);
    s->camera.target = Vector3Add(pos, forward);
    Vector3 up = Vector3RotateByQuaternion((Vector3){0, 1, 0}, rot);
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
    }

    if (IsKeyPressed(KEY_V)) {
        s->view_mode = (s->view_mode + 1) % VIEW_COUNT;
        const char *names[] = {"Grid", "jMAVSim", "Rez"};
        printf("View: %s\n", names[s->view_mode]);
    }

    // Mouse drag to orbit (left button)
    if (s->cam_mode == CAM_MODE_CHASE && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        s->chase_yaw   -= delta.x * 0.005f;
        s->chase_pitch += delta.y * 0.005f;

        // Clamp pitch to avoid flipping
        if (s->chase_pitch < -1.2f) s->chase_pitch = -1.2f;
        if (s->chase_pitch > 1.4f) s->chase_pitch = 1.4f;
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
    } else {
        // Sky sphere centered on camera — disable culling so we see it from inside
        rlDisableBackfaceCulling();
        DrawModel(s->sky_sphere, s->camera.position, 1.0f, WHITE);
        rlEnableBackfaceCulling();

        // Ground
        DrawModel(s->ground, (Vector3){0, 0, 0}, 1.0f, WHITE);


    }
}

void scene_draw_sky(const scene_t *s) {
    switch (s->view_mode) {
        case VIEW_GRID: ClearBackground(GRID_SKY); break;
        case VIEW_REZ:  ClearBackground(REZ_SKY);  break;
        default:        ClearBackground((Color){135, 206, 235, 255}); break;
    }
}

void scene_cleanup(scene_t *s) {
    UnloadModel(s->ground);
    UnloadModel(s->sky_sphere);
    UnloadModel(s->grid_plane);
    UnloadShader(s->grid_shader);
    if (s->ground_tex.id > 0) UnloadTexture(s->ground_tex);
    if (s->sky_tex.id > 0) UnloadTexture(s->sky_tex);
}
