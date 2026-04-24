#ifndef ULOG_EVENTS_H
#define ULOG_EVENTS_H

// Per-type event structs for the pre-extracted replay timeline. On WASM these
// are populated by the ULog extractor at init time and consumed by cursor
// iteration; on native they are the interchange format between the per-branch
// decode stage of process_message() and the shared apply helpers.
//
// Each struct is laid out for tight packing and 8-byte alignment. The sizes
// in the comments are the expected sizeof on a 32-bit Emscripten target.

#include <stdint.h>
#include "ulog_replay.h"   // STATUSTEXT_MSG_MAX, statustext_ring_t (type defs only)

typedef struct {
    uint64_t timestamp_us;
    float q[4];                     // w, x, y, z  (direct copy from vehicle_attitude)
} ulog_att_event_t;                 // 24 bytes

typedef struct {
    uint64_t timestamp_us;
    double lat_deg;
    double lon_deg;
    float  alt_m;
    float  _pad;                    // explicit pad keeps sizeof stable on both 32-bit and 64-bit
} ulog_gpos_event_t;                // 32 bytes

typedef struct {
    uint64_t timestamp_us;
    float x, y, z;                  // NED meters
    float vx, vy, vz;               // NED m/s
} ulog_lpos_event_t;                // 32 bytes

typedef struct {
    uint64_t timestamp_us;
    uint16_t ias_cms;               // indicated_airspeed_m_s * 100
    uint16_t tas_cms;               // true_airspeed_m_s * 100
    uint32_t _pad;
} ulog_aspd_event_t;                // 16 bytes

typedef struct {
    uint64_t timestamp_us;
    uint8_t  vehicle_type;          // PX4 enum, pre-translated to MAV_TYPE
    uint8_t  is_vtol;
    uint8_t  nav_state;
    uint8_t  _pad[5];
} ulog_vstatus_event_t;             // 16 bytes

typedef struct {
    uint64_t timestamp_us;
    double   lat_deg;
    double   lon_deg;
    float    alt_m;
    uint8_t  valid_hpos;
    uint8_t  _pad[3];
} ulog_home_event_t;                // 32 bytes

typedef struct {
    uint64_t timestamp_us;
    uint8_t  severity;              // 0=EMERG .. 7=DEBUG
    uint8_t  _pad[7];
    char     text[STATUSTEXT_MSG_MAX];
} ulog_statustext_event_t;          // 144 bytes

// ---------------------------------------------------------------------------
// Flat timeline container — one instance per loaded ULog, owned by the WASM
// replay context. All arrays grow with doubling realloc during extraction and
// are final-sized (realloc shrink) after the walk completes.
// ---------------------------------------------------------------------------

typedef struct {
    ulog_att_event_t        *att;         int att_count;         int att_cap;
    ulog_lpos_event_t       *lpos;        int lpos_count;        int lpos_cap;
    ulog_gpos_event_t       *gpos;        int gpos_count;        int gpos_cap;
    ulog_aspd_event_t       *aspd;        int aspd_count;        int aspd_cap;
    ulog_vstatus_event_t    *vstatus;     int vstatus_count;     int vstatus_cap;
    ulog_home_event_t       *home;        int home_count;        int home_cap;
    ulog_statustext_event_t *statustext;  int statustext_count;  int statustext_cap;

    // Log-level timestamp bounds (microseconds, absolute from log header)
    uint64_t start_timestamp_us;
    uint64_t end_timestamp_us;
} ulog_timeline_t;

#endif
