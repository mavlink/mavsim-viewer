#include "ulog_replay_apply.h"

#include <math.h>
#include <string.h>

// PX4 nav_state display names. Lives in the shared file so both native (which
// compiles this alongside ulog_replay.c) and WASM (which compiles only this
// subset of the replay layer) link against a single copy.
const char *ulog_nav_state_name(uint8_t nav_state) {
    switch (nav_state) {
        case 0:  return "Manual";
        case 1:  return "Altitude";
        case 2:  return "Position";
        case 3:  return "Mission";
        case 4:  return "Loiter";
        case 5:  return "RTL";
        case 10: return "Acro";
        case 12: return "Descend";
        case 13: return "Terminate";
        case 14: return "Offboard";
        case 15: return "Stabilized";
        case 17: return "Takeoff";
        case 18: return "Land";
        case 19: return "Follow";
        case 20: return "Precland";
        case 21: return "Orbit";
        case 22: return "VTOL Takeoff";
        default: return "Unknown";
    }
}

// Approximate meters-per-degree at a given latitude (matches the inline helper
// that used to live in ulog_replay.c and wasm/wasm_replay.c).
void ulog_local_to_global(double ref_lat, double ref_lon, float ref_alt,
                           float x, float y, float z,
                           int32_t *lat_e7, int32_t *lon_e7, int32_t *alt_mm) {
    double meters_per_deg_lat = 111132.92;
    double meters_per_deg_lon = 111132.92 * cos(ref_lat * ULOG_DEG_TO_RAD);
    if (meters_per_deg_lon < 1.0) meters_per_deg_lon = 1.0;

    double lat = ref_lat + (double)x / meters_per_deg_lat;
    double lon = ref_lon + (double)y / meters_per_deg_lon;
    float  alt = ref_alt - z;  // NED z is down, alt is up

    *lat_e7 = (int32_t)(lat * 1e7);
    *lon_e7 = (int32_t)(lon * 1e7);
    *alt_mm = (int32_t)(alt * 1000.0f);
}

void ulog_apply_attitude(ulog_replay_ctx_t *ctx, const ulog_att_event_t *ev) {
    ctx->state.time_usec = ev->timestamp_us;
    if (ctx->first_pos_set) ctx->state.valid = true;
    ctx->state.quaternion[0] = ev->q[0];
    ctx->state.quaternion[1] = ev->q[1];
    ctx->state.quaternion[2] = ev->q[2];
    ctx->state.quaternion[3] = ev->q[3];
}

void ulog_apply_gpos(ulog_replay_ctx_t *ctx, const ulog_gpos_event_t *ev) {
    ctx->state.time_usec = ev->timestamp_us;
    if (ctx->first_pos_set) ctx->state.valid = true;

    ctx->state.lat = (int32_t)(ev->lat_deg * 1e7);
    ctx->state.lon = (int32_t)(ev->lon_deg * 1e7);
    ctx->state.alt = (int32_t)(ev->alt_m * 1000.0f);

    // GPOS has no velocity fields — derive from consecutive samples.
    if (ctx->prev_gpos_usec > 0 && ev->timestamp_us > ctx->prev_gpos_usec) {
        double dt_gpos = (double)(ev->timestamp_us - ctx->prev_gpos_usec) / 1e6;
        if (dt_gpos > 0.01 && dt_gpos < 5.0) {
            double meters_per_deg_lat = 111132.92;
            double meters_per_deg_lon = 111132.92 * cos(ev->lat_deg * ULOG_DEG_TO_RAD);
            if (meters_per_deg_lon < 1.0) meters_per_deg_lon = 1.0;
            ctx->gpos_vx = (float)((ev->lat_deg - ctx->prev_lat_deg) * meters_per_deg_lat / dt_gpos);
            ctx->gpos_vy = (float)((ev->lon_deg - ctx->prev_lon_deg) * meters_per_deg_lon / dt_gpos);
            ctx->gpos_vz = (float)(-(ev->alt_m - ctx->prev_alt_m) / dt_gpos);
        }
    }
    ctx->prev_lat_deg   = ev->lat_deg;
    ctx->prev_lon_deg   = ev->lon_deg;
    ctx->prev_alt_m     = ev->alt_m;
    ctx->prev_gpos_usec = ev->timestamp_us;

    ctx->has_global_pos = true;

    ctx->last_lat_deg  = ev->lat_deg;
    ctx->last_lon_deg  = ev->lon_deg;
    ctx->last_alt_m    = ev->alt_m;
    ctx->last_pos_usec = ev->timestamp_us;

    if (!ctx->first_pos_set) {
        if (!ctx->home_from_topic) {
            ctx->home.lat   = ctx->state.lat;
            ctx->home.lon   = ctx->state.lon;
            ctx->home.alt   = ctx->state.alt;
            ctx->home.valid = true;
        }
        ctx->first_pos_set = true;
        ctx->state.valid   = true;
    }
}

void ulog_apply_lpos(ulog_replay_ctx_t *ctx, const ulog_lpos_event_t *ev) {
    ctx->state.time_usec = ev->timestamp_us;
    if (ctx->first_pos_set) ctx->state.valid = true;

    ctx->state.vx = (int16_t)(ev->vx * 100.0f);
    ctx->state.vy = (int16_t)(ev->vy * 100.0f);
    ctx->state.vz = (int16_t)(ev->vz * 100.0f);

    ctx->last_vx = ev->vx;
    ctx->last_vy = ev->vy;
    ctx->last_vz = ev->vz;
    ctx->last_x  = ev->x;
    ctx->last_y  = ev->y;
    ctx->last_z  = ev->z;

    if (!ctx->has_global_pos) {
        ctx->last_pos_usec = ev->timestamp_us;

        // Require horizontal position (x/y or reference frame) to produce a
        // meaningful global position. z-only (baro) without a reference
        // can't yield valid lat/lon. Ref capture is handled by the caller
        // (native: inline in process_message; WASM: extractor pre-pass).
        bool has_pos = (ev->x != 0.0f || ev->y != 0.0f) || ctx->ref_set;

        if (has_pos) {
            ulog_local_to_global(ctx->ref_lat, ctx->ref_lon, ctx->ref_alt,
                                  ev->x, ev->y, ev->z,
                                  &ctx->state.lat, &ctx->state.lon, &ctx->state.alt);

            if (!ctx->first_pos_set) {
                if (!ctx->home_from_topic) {
                    ctx->home.lat   = ctx->state.lat;
                    ctx->home.lon   = ctx->state.lon;
                    ctx->home.alt   = ctx->state.alt;
                    ctx->home.valid = true;
                }
                ctx->first_pos_set = true;
                ctx->state.valid   = true;
            }
        }
    }
}

void ulog_apply_aspd(ulog_replay_ctx_t *ctx, const ulog_aspd_event_t *ev) {
    ctx->state.time_usec = ev->timestamp_us;
    if (ctx->first_pos_set) ctx->state.valid = true;
    ctx->state.ind_airspeed  = ev->ias_cms;
    ctx->state.true_airspeed = ev->tas_cms;
}

void ulog_apply_vstatus(ulog_replay_ctx_t *ctx, const ulog_vstatus_event_t *ev) {
    ctx->state.time_usec = ev->timestamp_us;
    if (ctx->first_pos_set) ctx->state.valid = true;
    // vehicle_type already pre-translated to MAV_TYPE by the decode step
    ctx->vehicle_type      = ev->vehicle_type;
    ctx->current_nav_state = ev->nav_state;
}

void ulog_apply_home(ulog_replay_ctx_t *ctx, const ulog_home_event_t *ev) {
    ctx->state.time_usec = ev->timestamp_us;
    if (ctx->first_pos_set) ctx->state.valid = true;

    // home_rejected: pre-scan determined this log has no GPOS to confirm
    // home, so ignore home_position topic updates (native-compatible).
    if (ctx->home_rejected) return;
    if (ev->lat_deg == 0.0 && ev->lon_deg == 0.0) return;

    ctx->home.lat = (int32_t)(ev->lat_deg * 1e7);
    ctx->home.lon = (int32_t)(ev->lon_deg * 1e7);
    ctx->home.alt = (int32_t)(ev->alt_m   * 1000.0f);
    ctx->home.valid       = true;
    ctx->home_from_topic  = true;
}
