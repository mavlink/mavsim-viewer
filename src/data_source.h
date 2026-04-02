#ifndef DATA_SOURCE_H
#define DATA_SOURCE_H

#include <stdbool.h>
#include <stdint.h>
#include "mavlink_receiver.h"  // hil_state_t, home_position_t

// Flight mode change event for timeline markers
typedef struct {
    float time_s;          // seconds from log start
    uint8_t nav_state;     // PX4 nav_state value
} playback_mode_change_t;

// Playback state (only meaningful for replay sources)
typedef struct {
    bool paused;
    float speed;           // playback speed multiplier (1.0 = realtime)
    bool looping;
    bool interpolation;    // dead-reckoning interpolation between position samples
    float progress;        // 0.0 to 1.0
    float duration_s;      // total log duration in seconds
    float position_s;      // current position in seconds
    uint8_t current_nav_state;  // current flight mode (0xFF = unknown)
    const playback_mode_change_t *mode_changes;  // array of mode transitions (NULL for MAVLink)
    int mode_change_count;
    float takeoff_conf;         // CUSUM confidence (0.0-1.0, -1 = N/A)
    float takeoff_time_s;       // seconds from log start when takeoff detected (0 = not detected)
    bool  takeoff_detected;     // true if CUSUM found a clear takeoff event
    bool  home_from_topic;      // true = home set from home_position topic (Tier 1)
    float correlation;          // Pearson r vs reference drone (NAN = N/A)
    float rmse;                 // RMS position error vs reference (m) (NAN = N/A)
    float time_offset_s;        // alignment offset for display
} playback_state_t;

typedef struct data_source data_source_t;

// Vtable for polymorphic data sources
typedef struct {
    void (*poll)(data_source_t *ds, float dt);
    void (*seek)(data_source_t *ds, float target_s);   // seek to time (replay only, NULL for MAVLink)
    void (*set_time_offset)(data_source_t *ds, double offset_s); // set alignment offset (replay only)
    void (*close)(data_source_t *ds);
} data_source_ops_t;

struct data_source {
    const data_source_ops_t *ops;

    // Shared state — both backends populate these
    hil_state_t state;
    home_position_t home;
    bool connected;
    uint8_t sysid;
    uint8_t mav_type;
    bool debug;
    bool ref_rejected;     // LPOS reference coordinates were invalid

    // Replay controls (ignored by MAVLink backend)
    playback_state_t playback;

    // Backend-specific opaque data
    void *impl;
};

static inline void data_source_poll(data_source_t *ds, float dt) {
    ds->ops->poll(ds, dt);
}

static inline void data_source_seek(data_source_t *ds, float target_s) {
    if (ds->ops->seek) ds->ops->seek(ds, target_s);
}

static inline void data_source_set_time_offset(data_source_t *ds, double offset_s) {
    if (ds->ops->set_time_offset) ds->ops->set_time_offset(ds, offset_s);
}

static inline void data_source_close(data_source_t *ds) {
    ds->ops->close(ds);
}

// Create a live MAVLink data source. Returns 0 on success.
int data_source_mavlink_create(data_source_t *ds, uint16_t port, uint8_t channel, bool debug_flag);

// Create a ULog replay data source. Returns 0 on success.
int data_source_ulog_create(data_source_t *ds, const char *filepath);

// Return short display name for a PX4 nav_state value.
const char *ulog_nav_state_name(uint8_t nav_state);

#endif
