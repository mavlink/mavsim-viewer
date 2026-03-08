#ifndef SCENE_H
#define SCENE_H

#include "raylib.h"
#include <stdbool.h>

typedef enum {
    CAM_MODE_CHASE = 0,
    CAM_MODE_FPV,
    CAM_MODE_COUNT
} camera_mode_t;

typedef enum {
    VIEW_GRID = 0,
    VIEW_REZ,
    VIEW_SNOW,
    VIEW_COUNT,     // public modes end here
    VIEW_1988,      // hidden mode (not in V cycle)
} view_mode_t;

typedef struct {
    Camera3D camera;
    camera_mode_t cam_mode;
    view_mode_t view_mode;
    float chase_distance;
    float chase_yaw;    // horizontal orbit angle (radians)
    float chase_pitch;  // vertical orbit angle (radians)
    float fpv_yaw;      // gimbal yaw offset (radians)
    float fpv_pitch;    // gimbal pitch offset (radians, 0 = forward, negative = down)
    // Grid shader
    Shader grid_shader;
    int loc_colGround;
    int loc_colMinor;
    int loc_colMajor;
    int loc_colAxisX;
    int loc_colAxisZ;
    int loc_spacing;
    int loc_majorEvery;
    int loc_axisWidth;
    int loc_matModel;
    Model grid_plane;    // separate ground plane for grid modes
    int seq_1988;        // key sequence tracker
} scene_t;

// Initialize scene (ground plane, sky, camera, lighting).
void scene_init(scene_t *s);

// Update camera based on vehicle position and current mode.
void scene_update_camera(scene_t *s, Vector3 vehicle_pos, Quaternion vehicle_rot);

// Handle camera mode toggle input.
void scene_handle_input(scene_t *s);

// Draw the world (ground, sky). Call between BeginMode3D/EndMode3D.
void scene_draw(const scene_t *s);

// Draw sky background. Call before BeginMode3D.
void scene_draw_sky(const scene_t *s);

// Cleanup scene resources.
void scene_cleanup(scene_t *s);

#endif
