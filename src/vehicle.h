#ifndef VEHICLE_H
#define VEHICLE_H

#include "raylib.h"
#include "mavlink_receiver.h"
#include "scene.h"

// Model descriptor — add new entries to vehicle_models[] in vehicle.c
typedef struct {
    const char *path;           // OBJ file path
    const char *name;           // display name
    float scale;
    float pitch_offset_deg;    // pitch correction (applied before yaw, after X-90 base)
    float yaw_offset_deg;      // yaw correction after Z-up → Y-up rotation
} vehicle_model_info_t;

extern const vehicle_model_info_t vehicle_models[];
extern const int vehicle_model_count;

// Default model indices (match vehicle_models[] order in vehicle.c)
#define MODEL_QUADROTOR   0
#define MODEL_FIXEDWING   1
#define MODEL_TAILSITTER  2
#define MODEL_FPV_QUAD    3
#define MODEL_HEXAROTOR   4
#define MODEL_VTOL        5
#define MODEL_ROVER       6

typedef struct {
    Model model;
    Vector3 position;        // Raylib coords (Y-up, right-handed)
    Quaternion rotation;     // Raylib quaternion
    int model_idx;           // index into vehicle_models[]
    bool origin_set;
    bool active;             // has received data
    double lat0;             // radians
    double lon0;             // radians
    double alt0;             // meters
    float model_scale;
    float pitch_offset_deg;  // per-model pitch correction
    float yaw_offset_deg;    // per-model yaw correction
    float heading_deg;       // yaw 0-360
    float roll_deg;          // roll in degrees
    float pitch_deg;         // pitch in degrees
    float ground_speed;      // m/s
    float vertical_speed;    // m/s (positive = climbing)
    float airspeed;          // m/s
    float altitude_rel;      // meters above origin
    int red_material_idx;    // material index for red arms (-1 if not found)
    uint8_t sysid;
    Color color;
    Vector3 *trail;
    float *trail_roll;           // roll angle at each trail sample
    float *trail_pitch;          // pitch angle at each trail sample
    float *trail_vert;           // vertical speed at each trail sample
    float *trail_speed;          // ground speed (m/s) at each trail sample
    int trail_count;
    int trail_head;
    int trail_capacity;
    float trail_timer;
    Shader lighting_shader;  // shared lighting shader (id=0 if none)
    int loc_matNormal;       // shader uniform for normal matrix
} vehicle_t;

// Initialize vehicle state and load the model at model_idx. shader is optional lighting shader (id=0 to skip).
void vehicle_init(vehicle_t *v, int model_idx, Shader lighting_shader);

// Swap to a different model at runtime (unloads old, loads new).
void vehicle_load_model(vehicle_t *v, int model_idx);

// Cycle to the next model in the registry.
void vehicle_cycle_model(vehicle_t *v);

// Update position/rotation from HIL_STATE_QUATERNION data.
void vehicle_update(vehicle_t *v, const hil_state_t *state);

// trail_mode: 0=off, 1=normal trail, 2=speed ribbon
void vehicle_draw(vehicle_t *v, view_mode_t view_mode, bool selected,
                  int trail_mode, bool show_ground_track, Vector3 cam_pos);

// Reset the path trail.
void vehicle_reset_trail(vehicle_t *v);

// Unload model resources.
void vehicle_cleanup(vehicle_t *v);

#endif
