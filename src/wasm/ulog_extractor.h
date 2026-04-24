#ifndef WASM_ULOG_EXTRACTOR_H
#define WASM_ULOG_EXTRACTOR_H

// WASM-only ULog extractor. Walks a ULog byte buffer once, populating a
// flat-array timeline and pre-scan state. Does not depend on src/ulog_parser.c
// or src/ulog_replay.c — this is a freshly-written parser that reuses the
// same ULog format knowledge without sharing implementation.

#include <stdint.h>
#include <stddef.h>

#include "ulog_events.h"
#include "../mavlink_receiver.h"   // home_position_t (type only)
#include "../ulog_replay.h"        // ulog_mode_change_t, ULOG_MAX_MODE_CHANGES (type only)

// Pre-scan result populated alongside the timeline. Mirrors the relevant
// fields from ulog_replay_ctx_t that the replay output surface exposes to
// the rest of the application.
typedef struct {
    ulog_timeline_t *timeline;          // in/out: owned by caller, populated by extractor

    // Vehicle type and current mode state
    uint8_t            vehicle_type;
    uint8_t            current_nav_state;

    // Flight mode transitions (copied to replay ctx after extraction)
    ulog_mode_change_t mode_changes[ULOG_MAX_MODE_CHANGES];
    int                mode_change_count;

    // Home position resolution
    home_position_t    home;
    uint8_t            home_from_topic;   // true = tier 1 (from home_position topic)
    uint8_t            home_rejected;     // pre-scan rejected home (no GPOS to confirm)

    // LPOS reference point (for local→global conversion fallback)
    double             ref_lat_deg;
    double             ref_lon_deg;
    float              ref_alt_m;
    uint8_t            ref_set;
    uint8_t            ref_rejected;

    // CUSUM takeoff detection
    float              takeoff_time_s;
    float              takeoff_conf;
    uint8_t            takeoff_detected;
} ulog_extract_result_t;

// Parse a ULog from memory, populate timeline and pre-scan state.
// Caller owns `out->timeline` and must call ulog_timeline_init on it before
// calling this function. On success, returns 0 and the timeline plus all
// pre-scan fields are populated. On failure, returns -1 and the timeline
// may be partially populated (caller should ulog_timeline_free it).
int ulog_extract(const uint8_t *buf, size_t len, ulog_extract_result_t *out);

#endif
