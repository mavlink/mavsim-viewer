#ifndef ULOG_REPLAY_H
#define ULOG_REPLAY_H

#include "ulog_parser.h"
#include "mavlink_receiver.h"  // hil_state_t, home_position_t
#include <stdbool.h>

// Flight mode change event (from nav_state transitions)
#define ULOG_MAX_MODE_CHANGES 256

typedef struct {
    float time_s;          // seconds from log start
    uint8_t nav_state;     // PX4 nav_state value
} ulog_mode_change_t;

// Cached field offsets for fast extraction (resolved once on init)
typedef struct {
    int att_q_offset;           // vehicle_attitude: q[4]

    int gpos_lat_offset;        // vehicle_global_position: lat (double, deg)
    int gpos_lon_offset;        // lon (double, deg)
    int gpos_alt_offset;        // alt (float, m)

    int lpos_x_offset;          // vehicle_local_position: x (float, m NED)
    int lpos_y_offset;
    int lpos_z_offset;
    int lpos_vx_offset;         // vx (float, m/s NED)
    int lpos_vy_offset;
    int lpos_vz_offset;
    int lpos_ref_lat_offset;    // ref_lat (double, deg)
    int lpos_ref_lon_offset;    // ref_lon (double, deg)
    int lpos_ref_alt_offset;    // ref_alt (float, m)
    int lpos_xy_global_offset;  // xy_global (bool)
    int lpos_z_global_offset;   // z_global (bool)

    int aspd_ias_offset;        // airspeed_validated: indicated_airspeed_m_s
    int aspd_tas_offset;        // true_airspeed_m_s

    int vstatus_type_offset;    // vehicle_status: vehicle_type
    int vstatus_is_vtol_offset; // vehicle_status: is_vtol
    int vstatus_nav_state_offset; // vehicle_status: nav_state

    int home_lat_offset;        // home_position: lat (double, deg)
    int home_lon_offset;        // home_position: lon (double, deg)
    int home_alt_offset;        // home_position: alt (float, m)
} ulog_field_cache_t;

typedef struct {
    ulog_parser_t parser;

    // Subscription indices (-1 if topic not found in log)
    int sub_attitude;
    int sub_global_pos;
    int sub_local_pos;
    int sub_airspeed;
    int sub_vehicle_status;
    int sub_home_pos;

    ulog_field_cache_t cache;

    // Current output state
    hil_state_t state;
    home_position_t home;
    uint8_t vehicle_type;
    uint8_t current_nav_state;  // current PX4 nav_state (0xFF = unknown)

    // Flight mode transitions (populated during pre-scan)
    ulog_mode_change_t mode_changes[ULOG_MAX_MODE_CHANGES];
    int mode_change_count;

    // Timing
    double wall_accum;          // accumulated playback time in seconds

    // Position mode: true if vehicle_global_position has data,
    // false means we derive lat/lon/alt from local position
    bool has_global_pos;

    // Reference point for local→global conversion (from vehicle_local_position)
    double ref_lat;             // degrees
    double ref_lon;             // degrees
    float ref_alt;              // meters
    bool ref_set;

    // Last position sample for dead-reckoning interpolation
    uint64_t last_pos_usec;     // timestamp of last position update
    float last_x, last_y, last_z;   // local NED position (m)
    float last_vx, last_vy, last_vz; // local NED velocity (m/s)
    // For gpos mode: last known lat/lon/alt and derived velocity
    double last_lat_deg, last_lon_deg;
    float last_alt_m;
    double prev_lat_deg, prev_lon_deg; // previous GPOS sample for velocity derivation
    float prev_alt_m;
    uint64_t prev_gpos_usec;
    float gpos_vx, gpos_vy, gpos_vz; // velocity derived from consecutive GPOS samples (m/s NED)

    // First valid position becomes home
    bool first_pos_set;
    bool home_from_topic;  // true = home set from home_position topic (Tier 1)
} ulog_replay_ctx_t;

// Initialize replay context, parse file, build index. Returns 0 on success.
int ulog_replay_init(ulog_replay_ctx_t *ctx, const char *filepath);

// Advance replay by dt seconds at given speed. Returns true if still playing.
bool ulog_replay_advance(ulog_replay_ctx_t *ctx, float dt, float speed, bool looping, bool interpolation);

// Seek to a specific position in seconds from log start.
void ulog_replay_seek(ulog_replay_ctx_t *ctx, float target_s);

// Close and free resources.
void ulog_replay_close(ulog_replay_ctx_t *ctx);

// Return short display name for a PX4 nav_state value.
const char *ulog_nav_state_name(uint8_t nav_state);

// Future scope: multi-file swarm replay with time synchronization.
// Each vehicle would get its own ulog_replay_ctx_t with a time_offset
// to align different log start times to a common playback clock.

#endif
