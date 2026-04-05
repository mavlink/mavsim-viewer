#ifndef VEHICLE_H
#define VEHICLE_H

#include "raylib.h"
#include "mavlink_receiver.h"
#include "scene.h"
#include "theme.h"

// Marker type — user (sphere) vs system (cube)
typedef enum {
    MARKER_USER,
    MARKER_SYSTEM,
} marker_type_t;

// Model group — determines which models M-key cycles through
typedef enum {
    GROUP_QUAD,
    GROUP_HEX,
    GROUP_FIXED_WING,
    GROUP_VTOL,
    GROUP_TAILSITTER,
    GROUP_ROVER,
    GROUP_ROV,
    GROUP_COUNT,
} model_group_t;

// Model descriptor — add new entries to vehicle_models[] in vehicle.c
typedef struct {
    const char *path;           // OBJ file path
    const char *name;           // display name
    float scale;
    float pitch_offset_deg;    // pitch correction (applied before yaw, after X-90 base)
    float yaw_offset_deg;      // yaw correction after Z-up → Y-up rotation
    model_group_t group;       // which group this model belongs to
} vehicle_model_info_t;

extern const vehicle_model_info_t vehicle_models[];
extern const int vehicle_model_count;

// Default model indices (match vehicle_models[] order in vehicle.c)
#define MODEL_QUADROTOR   0
#define MODEL_FIXEDWING   1
#define MODEL_TAILSITTER  2
#define MODEL_FPV_QUAD    3
#define MODEL_HEXAROTOR   4
#define MODEL_FPV_HEX     5
#define MODEL_VTOL        6
#define MODEL_ROVER       7
#define MODEL_ROV         8

typedef struct {
    Model model;
    Vector3 position;        // Raylib coords (Y-up, right-handed)
    Vector3 grid_offset;     // spatial offset for deconfliction (meters, Raylib coords)
    Quaternion rotation;     // Raylib quaternion
    int model_idx;           // index into vehicle_models[]
    model_group_t model_group; // active group for M-key cycling
    bool origin_set;
    int origin_wait_count;   // HIL updates received while waiting for HOME_POSITION
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
    int red_material_idx;    // material index for red/port arms (-1 if not found)
    int green_material_idx;  // material index for green/starboard arms (-1 if not found)
    int front_material_idx;  // material index for front arms (yellow, -1 if not found)
    int back_material_idx;   // material index for back arms (purple, -1 if not found)
    uint8_t sysid;
    Color color;
    Vector3 *trail;
    float *trail_roll;           // roll angle at each trail sample
    float *trail_pitch;          // pitch angle at each trail sample
    float *trail_vert;           // vertical speed at each trail sample
    float *trail_speed;          // 3D speed (m/s) at each trail sample
    float *trail_time;           // replay timestamp (seconds) at each trail sample
    float trail_speed_max;       // max speed seen so far (for adaptive ribbon)
    int trail_count;
    int trail_head;
    int trail_capacity;
    float trail_timer;
    float current_time;          // set externally: current replay position (seconds)
    Vector3 trail_last_dir;  // direction of last recorded segment (for adaptive sampling)
    Shader lighting_shader;  // shared lighting shader (id=0 if none)
    int loc_matNormal;       // shader uniform for normal matrix
    float ghost_alpha;       // 1.0 = fully opaque, 0.35 = ghost
    int   loc_ghost_alpha;   // shader uniform location for ghostAlpha
} vehicle_t;

// Initialize vehicle state and load the model at model_idx. shader is optional lighting shader (id=0 to skip).
void vehicle_init(vehicle_t *v, int model_idx, Shader lighting_shader);

// Initialize with custom trail capacity (for replay persistent trails).
void vehicle_init_ex(vehicle_t *v, int model_idx, Shader lighting_shader, int trail_capacity);

// Set ghost alpha for translucent rendering (1.0 = opaque, 0.35 = ghost).
void vehicle_set_ghost_alpha(vehicle_t *v, float alpha);

// Swap to a different model at runtime (unloads old, loads new).
void vehicle_load_model(vehicle_t *v, int model_idx);

// Cycle to the next model within the active group.
void vehicle_cycle_model(vehicle_t *v);

// Select model group and default model based on MAV_TYPE from heartbeat.
void vehicle_set_type(vehicle_t *v, uint8_t mav_type);

// Update position/rotation from HIL_STATE_QUATERNION data.
// home may be NULL; if valid, its altitude is used as ground reference.
void vehicle_update(vehicle_t *v, const hil_state_t *state, const home_position_t *home);

// trail_mode: 0=off, 1=normal trail, 2=speed ribbon
// classic_colors: false = modern (yellow/purple), true = classic (red/blue)
void vehicle_draw(vehicle_t *v, const theme_t *theme, bool selected,
                  int trail_mode, bool show_ground_track, Vector3 cam_pos,
                  bool classic_colors);

// Reset the path trail.
void vehicle_reset_trail(vehicle_t *v);

// Truncate trail to only include points at or before the given time.
void vehicle_truncate_trail(vehicle_t *v, float time_s);

// Draw marker shapes (spheres for MARKER_USER, cubes for MARKER_SYSTEM).
// Call inside BeginMode3D. drone_color used when trail_mode == 3 (ID trails).
void vehicle_draw_markers(Vector3 *positions, char labels[][48], int count,
                          int current_marker, Vector3 cam_pos, Camera3D camera,
                          float *m_roll, float *m_pitch, float *m_vert, float *m_speed,
                          float speed_max, const theme_t *theme, int trail_mode,
                          marker_type_t type, Color drone_color);

// Draw billboarded marker labels (call AFTER EndMode3D, in 2D pass).
void vehicle_draw_marker_labels(Vector3 *positions, char labels[][48], int count,
                                int current_marker, Vector3 cam_pos, Camera3D camera,
                                Font font_label, Font font_value,
                                float *m_roll, float *m_pitch, float *m_vert, float *m_speed,
                                float speed_max, const theme_t *theme, int trail_mode,
                                marker_type_t type, Color drone_color);

// Compute marker color from snapshotted telemetry.
// drone_color used when trail_mode == 3 (ID trails) or multi-drone mode.
Color vehicle_marker_color(float roll, float pitch, float vert, float speed,
                           float speed_max, const theme_t *theme, int trail_mode,
                           Color drone_color);

// Draw correlation curtain between two vehicles (cross-vehicle overlay).
void vehicle_draw_correlation_curtain(
    const vehicle_t *va, const vehicle_t *vb,
    const theme_t *theme, Vector3 cam_pos);

// Draw thick correlation line between two vehicles at current positions.
void vehicle_draw_correlation_line(
    const vehicle_t *va, const vehicle_t *vb);

// Unload model resources.
void vehicle_cleanup(vehicle_t *v);

#endif
