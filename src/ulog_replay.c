#include "ulog_replay.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// PX4 nav_state display names
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

// Approximate meters-per-degree at a given latitude
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
static void local_to_global(double ref_lat, double ref_lon, float ref_alt,
                            float x, float y, float z,
                            int32_t *lat_e7, int32_t *lon_e7, int32_t *alt_mm) {
    // x = North, y = East, z = Down (NED)
    double meters_per_deg_lat = 111132.92;
    double meters_per_deg_lon = 111132.92 * cos(ref_lat * DEG_TO_RAD);
    if (meters_per_deg_lon < 1.0) meters_per_deg_lon = 1.0;

    double lat = ref_lat + (double)x / meters_per_deg_lat;
    double lon = ref_lon + (double)y / meters_per_deg_lon;
    float alt = ref_alt - z;  // NED z is down, alt is up

    *lat_e7 = (int32_t)(lat * 1e7);
    *lon_e7 = (int32_t)(lon * 1e7);
    *alt_mm = (int32_t)(alt * 1000.0f);
}

// ---------------------------------------------------------------------------
// Process a single data message — dispatch by subscription
// ---------------------------------------------------------------------------

static void process_message(ulog_replay_ctx_t *ctx, const ulog_data_msg_t *dmsg) {
    // Find which subscription this msg_id belongs to
    int sub_idx = -1;
    for (int i = 0; i < ctx->parser.sub_count; i++) {
        if (ctx->parser.subs[i].msg_id == dmsg->msg_id) {
            sub_idx = i;
            break;
        }
    }
    if (sub_idx < 0) return;

    ctx->state.time_usec = dmsg->timestamp;
    // Only mark state valid once we have position data (not just attitude)
    if (ctx->first_pos_set)
        ctx->state.valid = true;

    // home_position topic — authoritative home (Tier 1)
    // Skip if pre-scan rejected this log's home (no GPOS data to confirm it)
    if (sub_idx == ctx->sub_home_pos && ctx->sub_home_pos >= 0 && !ctx->home_rejected) {
        double lat = 0, lon = 0;
        float alt = 0;
        if (ctx->cache.home_lat_offset >= 0)
            lat = ulog_parser_get_double(dmsg, ctx->cache.home_lat_offset);
        if (ctx->cache.home_lon_offset >= 0)
            lon = ulog_parser_get_double(dmsg, ctx->cache.home_lon_offset);
        if (ctx->cache.home_alt_offset >= 0)
            alt = ulog_parser_get_float(dmsg, ctx->cache.home_alt_offset);
        if (lat != 0.0 || lon != 0.0) {
            ctx->home.lat = (int32_t)(lat * 1e7);
            ctx->home.lon = (int32_t)(lon * 1e7);
            ctx->home.alt = (int32_t)(alt * 1000.0f);
            ctx->home.valid = true;
            ctx->home_from_topic = true;
        }
    }

    if (sub_idx == ctx->sub_attitude) {
        // float[4] q — NED quaternion (w,x,y,z) — direct copy
        int off = ctx->cache.att_q_offset;
        ctx->state.quaternion[0] = ulog_parser_get_float(dmsg, off + 0);
        ctx->state.quaternion[1] = ulog_parser_get_float(dmsg, off + 4);
        ctx->state.quaternion[2] = ulog_parser_get_float(dmsg, off + 8);
        ctx->state.quaternion[3] = ulog_parser_get_float(dmsg, off + 12);
    }
    else if (ctx->sub_global_pos >= 0 && sub_idx == ctx->sub_global_pos) {
        // lat/lon: double (degrees) -> int32_t (degE7)
        // alt: float (meters) -> int32_t (mm)
        double lat = ulog_parser_get_double(dmsg, ctx->cache.gpos_lat_offset);
        double lon = ulog_parser_get_double(dmsg, ctx->cache.gpos_lon_offset);
        float alt = ulog_parser_get_float(dmsg, ctx->cache.gpos_alt_offset);

        ctx->state.lat = (int32_t)(lat * 1e7);
        ctx->state.lon = (int32_t)(lon * 1e7);
        ctx->state.alt = (int32_t)(alt * 1000.0f);

        // Derive velocity from consecutive GPOS samples (GPOS has no velocity fields)
        if (ctx->prev_gpos_usec > 0 && dmsg->timestamp > ctx->prev_gpos_usec) {
            double dt_gpos = (double)(dmsg->timestamp - ctx->prev_gpos_usec) / 1e6;
            if (dt_gpos > 0.01 && dt_gpos < 5.0) {
                double meters_per_deg_lat = 111132.92;
                double meters_per_deg_lon = 111132.92 * cos(lat * DEG_TO_RAD);
                if (meters_per_deg_lon < 1.0) meters_per_deg_lon = 1.0;
                ctx->gpos_vx = (float)((lat - ctx->prev_lat_deg) * meters_per_deg_lat / dt_gpos);
                ctx->gpos_vy = (float)((lon - ctx->prev_lon_deg) * meters_per_deg_lon / dt_gpos);
                ctx->gpos_vz = (float)(-(alt - ctx->prev_alt_m) / dt_gpos); // alt up, z down
            }
        }
        ctx->prev_lat_deg = lat;
        ctx->prev_lon_deg = lon;
        ctx->prev_alt_m = alt;
        ctx->prev_gpos_usec = dmsg->timestamp;

        ctx->has_global_pos = true;

        // Store for dead-reckoning interpolation
        ctx->last_lat_deg = lat;
        ctx->last_lon_deg = lon;
        ctx->last_alt_m = alt;
        ctx->last_pos_usec = dmsg->timestamp;

        if (!ctx->first_pos_set) {
            if (!ctx->home_from_topic) {
                ctx->home.lat = ctx->state.lat;
                ctx->home.lon = ctx->state.lon;
                ctx->home.alt = ctx->state.alt;
                ctx->home.valid = true;
            }
            ctx->first_pos_set = true;
            ctx->state.valid = true;
        }
    }
    else if (sub_idx == ctx->sub_local_pos) {
        // vx/vy/vz: float (m/s NED) -> int16_t (cm/s)
        float vx = ulog_parser_get_float(dmsg, ctx->cache.lpos_vx_offset);
        float vy = ulog_parser_get_float(dmsg, ctx->cache.lpos_vy_offset);
        float vz = ulog_parser_get_float(dmsg, ctx->cache.lpos_vz_offset);
        ctx->state.vx = (int16_t)(vx * 100.0f);
        ctx->state.vy = (int16_t)(vy * 100.0f);
        ctx->state.vz = (int16_t)(vz * 100.0f);

        // Store velocity and local position for dead-reckoning
        ctx->last_vx = vx;
        ctx->last_vy = vy;
        ctx->last_vz = vz;

        float x = ulog_parser_get_float(dmsg, ctx->cache.lpos_x_offset);
        float y = ulog_parser_get_float(dmsg, ctx->cache.lpos_y_offset);
        float z = ulog_parser_get_float(dmsg, ctx->cache.lpos_z_offset);
        ctx->last_x = x;
        ctx->last_y = y;
        ctx->last_z = z;

        if (!ctx->has_global_pos) {
            ctx->last_pos_usec = dmsg->timestamp;

            // Try to get reference point from local position if not yet set
            if (!ctx->ref_set && ctx->cache.lpos_xy_global_offset >= 0) {
                uint8_t xy_global = ulog_parser_get_uint8(dmsg, ctx->cache.lpos_xy_global_offset);
                if (xy_global) {
                    double rlat = ulog_parser_get_double(dmsg, ctx->cache.lpos_ref_lat_offset);
                    double rlon = ulog_parser_get_double(dmsg, ctx->cache.lpos_ref_lon_offset);
                    // Sanity check: reject garbage reference coordinates
                    if (rlat < -90.0 || rlat > 90.0 || rlon < -180.0 || rlon > 180.0) {
                        if (!ctx->ref_rejected) {
                            printf("  Warning: LPOS ref out of range (lat=%.1f, lon=%.1f), ignoring\n", rlat, rlon);
                            ctx->ref_rejected = true;
                        }
                    } else {
                        ctx->ref_lat = rlat;
                        ctx->ref_lon = rlon;
                        if (ctx->cache.lpos_z_global_offset >= 0 &&
                            ulog_parser_get_uint8(dmsg, ctx->cache.lpos_z_global_offset)) {
                            ctx->ref_alt = ulog_parser_get_float(dmsg, ctx->cache.lpos_ref_alt_offset);
                        }
                        ctx->ref_set = true;
                    }
                }
            }

            // Require horizontal position (x/y or reference frame) to produce
            // a meaningful global position. z-only (baro) without a reference
            // can't give valid lat/lon and would mix incompatible altitudes.
            bool has_pos = (x != 0.0f || y != 0.0f) || ctx->ref_set;

            if (has_pos) {
                local_to_global(ctx->ref_lat, ctx->ref_lon, ctx->ref_alt,
                                x, y, z,
                                &ctx->state.lat, &ctx->state.lon, &ctx->state.alt);

                if (!ctx->first_pos_set) {
                    if (!ctx->home_from_topic) {
                        ctx->home.lat = ctx->state.lat;
                        ctx->home.lon = ctx->state.lon;
                        ctx->home.alt = ctx->state.alt;
                        ctx->home.valid = true;
                    }
                    ctx->first_pos_set = true;
                    ctx->state.valid = true;
                }
            }
        }
    }
    else if (ctx->sub_airspeed >= 0 && sub_idx == ctx->sub_airspeed) {
        // float (m/s) -> uint16_t (cm/s)
        float ias = ulog_parser_get_float(dmsg, ctx->cache.aspd_ias_offset);
        float tas = ulog_parser_get_float(dmsg, ctx->cache.aspd_tas_offset);
        ctx->state.ind_airspeed = (uint16_t)(ias * 100.0f);
        ctx->state.true_airspeed = (uint16_t)(tas * 100.0f);
    }
    else if (ctx->sub_vehicle_status >= 0 && sub_idx == ctx->sub_vehicle_status) {
        // PX4 vehicle_type enum → MAVLink MAV_TYPE
        uint8_t px4_type = ulog_parser_get_uint8(dmsg, ctx->cache.vstatus_type_offset);
        bool is_vtol = ctx->cache.vstatus_is_vtol_offset >= 0 &&
                       ulog_parser_get_uint8(dmsg, ctx->cache.vstatus_is_vtol_offset);
        if (is_vtol) {
            ctx->vehicle_type = 22; // MAV_TYPE_VTOL_FIXEDROTOR
        } else {
            switch (px4_type) {
                case 1: ctx->vehicle_type = 2;  break; // ROTARY_WING → MAV_TYPE_QUADROTOR
                case 2: ctx->vehicle_type = 1;  break; // FIXED_WING  → MAV_TYPE_FIXED_WING
                case 3: ctx->vehicle_type = 10; break; // ROVER       → MAV_TYPE_GROUND_ROVER
                default: ctx->vehicle_type = 2; break;
            }
        }
        // Track current nav_state for HUD display
        if (ctx->cache.vstatus_nav_state_offset >= 0) {
            ctx->current_nav_state = ulog_parser_get_uint8(dmsg, ctx->cache.vstatus_nav_state_offset);
        }
    }
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

int ulog_replay_init(ulog_replay_ctx_t *ctx, const char *filepath) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->sub_attitude = -1;
    ctx->sub_global_pos = -1;
    ctx->sub_local_pos = -1;
    ctx->sub_airspeed = -1;
    ctx->sub_vehicle_status = -1;
    ctx->sub_home_pos = -1;

    int ret = ulog_parser_open(&ctx->parser, filepath);
    if (ret != 0) return ret;

    // Find subscriptions
    ctx->sub_attitude = ulog_parser_find_subscription(&ctx->parser, "vehicle_attitude");
    ctx->sub_global_pos = ulog_parser_find_subscription(&ctx->parser, "vehicle_global_position");
    ctx->sub_local_pos = ulog_parser_find_subscription(&ctx->parser, "vehicle_local_position");
    ctx->sub_airspeed = ulog_parser_find_subscription(&ctx->parser, "airspeed_validated");
    ctx->sub_vehicle_status = ulog_parser_find_subscription(&ctx->parser, "vehicle_status");
    ctx->sub_home_pos = ulog_parser_find_subscription(&ctx->parser, "home_position");

    // Required topics
    if (ctx->sub_attitude < 0) {
        fprintf(stderr, "ulog_replay: vehicle_attitude topic not found\n");
        ulog_parser_close(&ctx->parser);
        return -1;
    }
    if (ctx->sub_local_pos < 0) {
        fprintf(stderr, "ulog_replay: vehicle_local_position topic not found\n");
        ulog_parser_close(&ctx->parser);
        return -1;
    }

    // has_global_pos starts false and gets set to true when first gpos DATA arrives
    ctx->has_global_pos = false;

    // Cache field offsets for vehicle_attitude
    int att_fmt = ctx->parser.subs[ctx->sub_attitude].format_idx;
    ctx->cache.att_q_offset = ulog_parser_find_field(&ctx->parser, att_fmt, "q");

    // Cache field offsets for vehicle_global_position (if subscription exists)
    if (ctx->sub_global_pos >= 0) {
        int gpos_fmt = ctx->parser.subs[ctx->sub_global_pos].format_idx;
        ctx->cache.gpos_lat_offset = ulog_parser_find_field(&ctx->parser, gpos_fmt, "lat");
        ctx->cache.gpos_lon_offset = ulog_parser_find_field(&ctx->parser, gpos_fmt, "lon");
        ctx->cache.gpos_alt_offset = ulog_parser_find_field(&ctx->parser, gpos_fmt, "alt");
    }

    // Cache field offsets for vehicle_local_position
    int lpos_fmt = ctx->parser.subs[ctx->sub_local_pos].format_idx;
    ctx->cache.lpos_x_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "x");
    ctx->cache.lpos_y_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "y");
    ctx->cache.lpos_z_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "z");
    ctx->cache.lpos_vx_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "vx");
    ctx->cache.lpos_vy_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "vy");
    ctx->cache.lpos_vz_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "vz");
    ctx->cache.lpos_ref_lat_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "ref_lat");
    ctx->cache.lpos_ref_lon_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "ref_lon");
    ctx->cache.lpos_ref_alt_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "ref_alt");
    ctx->cache.lpos_xy_global_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "xy_global");
    ctx->cache.lpos_z_global_offset = ulog_parser_find_field(&ctx->parser, lpos_fmt, "z_global");

    // Optional: airspeed
    if (ctx->sub_airspeed >= 0) {
        int aspd_fmt = ctx->parser.subs[ctx->sub_airspeed].format_idx;
        ctx->cache.aspd_ias_offset = ulog_parser_find_field(&ctx->parser, aspd_fmt, "indicated_airspeed_m_s");
        ctx->cache.aspd_tas_offset = ulog_parser_find_field(&ctx->parser, aspd_fmt, "true_airspeed_m_s");
    }

    // Optional: vehicle_status
    if (ctx->sub_vehicle_status >= 0) {
        int vs_fmt = ctx->parser.subs[ctx->sub_vehicle_status].format_idx;
        ctx->cache.vstatus_type_offset = ulog_parser_find_field(&ctx->parser, vs_fmt, "vehicle_type");
        ctx->cache.vstatus_is_vtol_offset = ulog_parser_find_field(&ctx->parser, vs_fmt, "is_vtol");
        ctx->cache.vstatus_nav_state_offset = ulog_parser_find_field(&ctx->parser, vs_fmt, "nav_state");
    }

    // Optional: home_position
    if (ctx->sub_home_pos >= 0) {
        int hp_fmt = ctx->parser.subs[ctx->sub_home_pos].format_idx;
        ctx->cache.home_lat_offset = ulog_parser_find_field(&ctx->parser, hp_fmt, "lat");
        ctx->cache.home_lon_offset = ulog_parser_find_field(&ctx->parser, hp_fmt, "lon");
        ctx->cache.home_alt_offset = ulog_parser_find_field(&ctx->parser, hp_fmt, "alt");
        ctx->cache.home_valid_hpos_offset = ulog_parser_find_field(&ctx->parser, hp_fmt, "valid_hpos");
    }

    // Pre-scan for vehicle_type, flight mode transitions, and home position
    ctx->vehicle_type = 2;  // default MAV_TYPE_QUADROTOR
    ctx->current_nav_state = 0xFF;
    ctx->mode_change_count = 0;
    {
        bool got_type = false;
        bool got_home = false;
        bool seen_gpos_data = false;
        uint8_t prev_nav = 0xFF;

        uint16_t vstatus_msg_id = (ctx->sub_vehicle_status >= 0)
            ? ctx->parser.subs[ctx->sub_vehicle_status].msg_id : 0xFFFF;
        uint16_t home_msg_id = (ctx->sub_home_pos >= 0)
            ? ctx->parser.subs[ctx->sub_home_pos].msg_id : 0xFFFF;
        uint16_t gpos_msg_id = (ctx->sub_global_pos >= 0)
            ? ctx->parser.subs[ctx->sub_global_pos].msg_id : 0xFFFF;

        ulog_data_msg_t scan_msg;
        while (ulog_parser_next(&ctx->parser, &scan_msg)) {
            // Vehicle status: type + nav_state transitions
            if (scan_msg.msg_id == vstatus_msg_id && ctx->sub_vehicle_status >= 0) {
                if (!got_type) {
                    uint8_t px4_type = ulog_parser_get_uint8(&scan_msg, ctx->cache.vstatus_type_offset);
                    bool is_vtol = ctx->cache.vstatus_is_vtol_offset >= 0 &&
                                   ulog_parser_get_uint8(&scan_msg, ctx->cache.vstatus_is_vtol_offset);
                    if (is_vtol) {
                        ctx->vehicle_type = 22;
                    } else {
                        switch (px4_type) {
                            case 1: ctx->vehicle_type = 2;  break;
                            case 2: ctx->vehicle_type = 1;  break;
                            case 3: ctx->vehicle_type = 10; break;
                            default: ctx->vehicle_type = 2; break;
                        }
                    }
                    got_type = true;
                }
                if (ctx->cache.vstatus_nav_state_offset >= 0) {
                    uint8_t nav = ulog_parser_get_uint8(&scan_msg, ctx->cache.vstatus_nav_state_offset);
                    if (nav != prev_nav && ctx->mode_change_count < ULOG_MAX_MODE_CHANGES) {
                        float t = (float)((double)(scan_msg.timestamp - ctx->parser.start_timestamp) / 1e6);
                        ctx->mode_changes[ctx->mode_change_count].time_s = t;
                        ctx->mode_changes[ctx->mode_change_count].nav_state = nav;
                        ctx->mode_change_count++;
                        prev_nav = nav;
                    }
                }
            }

            // Home position: Tier 1 (highest priority)
            // Require valid_hpos. After the scan, we also verify that GPOS data
            // actually exists — subscription alone isn't enough (PX4 subscribes
            // even when EKF never produces data).
            if (!got_home && scan_msg.msg_id == home_msg_id) {
                bool hpos_valid = true;  // default true for older logs without the field
                if (ctx->cache.home_valid_hpos_offset >= 0)
                    hpos_valid = ulog_parser_get_uint8(&scan_msg, ctx->cache.home_valid_hpos_offset) != 0;
                double lat = ulog_parser_get_double(&scan_msg, ctx->cache.home_lat_offset);
                double lon = ulog_parser_get_double(&scan_msg, ctx->cache.home_lon_offset);
                float alt = ulog_parser_get_float(&scan_msg, ctx->cache.home_alt_offset);
                if (hpos_valid && (lat != 0.0 || lon != 0.0)) {
                    ctx->home.lat = (int32_t)(lat * 1e7);
                    ctx->home.lon = (int32_t)(lon * 1e7);
                    ctx->home.alt = (int32_t)(alt * 1000.0f);
                    ctx->home.valid = true;
                    ctx->home_from_topic = true;
                    got_home = true;
                }
            }

            // GPOS: track whether any data exists, and use as Tier 2 fallback
            if (scan_msg.msg_id == gpos_msg_id && gpos_msg_id != 0xFFFF)
                seen_gpos_data = true;
            if (!got_home && scan_msg.msg_id == gpos_msg_id) {
                double lat = ulog_parser_get_double(&scan_msg, ctx->cache.gpos_lat_offset);
                double lon = ulog_parser_get_double(&scan_msg, ctx->cache.gpos_lon_offset);
                float alt = ulog_parser_get_float(&scan_msg, ctx->cache.gpos_alt_offset);
                if (lat != 0.0 || lon != 0.0) {
                    ctx->home.lat = (int32_t)(lat * 1e7);
                    ctx->home.lon = (int32_t)(lon * 1e7);
                    ctx->home.alt = (int32_t)(alt * 1000.0f);
                    ctx->home.valid = true;
                    got_home = true;
                }
            }
        }

        // If home came from home_position topic but no GPOS data exists,
        // the position is baro-only and unreliable as an absolute datum.
        if (got_home && ctx->home_from_topic && !seen_gpos_data) {
            ctx->home.valid = false;
            ctx->home_from_topic = false;
            ctx->home_rejected = true;
        }

        if (ctx->mode_change_count > 0)
            ctx->current_nav_state = ctx->mode_changes[0].nav_state;
        ulog_parser_rewind(&ctx->parser);
    }

    float dur = (float)((double)(ctx->parser.end_timestamp - ctx->parser.start_timestamp) / 1e6);
    printf("ULog replay: %s (%.1fs, %d index entries)\n", filepath, dur, ctx->parser.index_count);
    if (ctx->mode_change_count > 0) {
        printf("  Flight modes:");
        for (int i = 0; i < ctx->mode_change_count; i++) {
            printf(" %s@%.0fs", ulog_nav_state_name(ctx->mode_changes[i].nav_state),
                   ctx->mode_changes[i].time_s);
        }
        printf("\n");
    }
    if (ctx->sub_global_pos < 0) printf("  Note: vehicle_global_position not found (using local position)\n");
    if (ctx->sub_airspeed < 0) printf("  Note: airspeed_validated not found (no airspeed data)\n");
    if (ctx->sub_vehicle_status < 0) printf("  Note: vehicle_status not found (defaulting to quadrotor)\n");
    if (ctx->sub_home_pos < 0) printf("  Note: home_position not found (deriving from first GPS fix)\n");

    return 0;
}

// ---------------------------------------------------------------------------
// Advance playback
// ---------------------------------------------------------------------------

bool ulog_replay_advance(ulog_replay_ctx_t *ctx, float dt, float speed, bool looping, bool interpolation) {
    ctx->wall_accum += (double)dt * speed;
    uint64_t target = ctx->parser.start_timestamp + (uint64_t)(ctx->wall_accum * 1e6);

    // Clamp to end
    if (target > ctx->parser.end_timestamp) {
        if (looping) {
            ulog_parser_rewind(&ctx->parser);
            ctx->wall_accum = 0.0;
            ctx->first_pos_set = false;
            ctx->has_global_pos = false;
            ctx->prev_gpos_usec = 0;
            ctx->last_pos_usec = 0;
            ctx->gpos_vx = ctx->gpos_vy = ctx->gpos_vz = 0.0f;
            return true;
        }
        return false;
    }

    // Read all messages up to target timestamp
    ulog_data_msg_t dmsg;
    while (ulog_parser_next(&ctx->parser, &dmsg)) {
        process_message(ctx, &dmsg);
        if (dmsg.timestamp >= target) break;
    }

    // Dead-reckon position forward from last sample using velocity.
    // This smooths the 5-10 Hz position updates to match the render rate.
    if (interpolation && ctx->last_pos_usec > 0 && target > ctx->last_pos_usec) {
        float dt_pos = (float)((double)(target - ctx->last_pos_usec) / 1e6);
        // Clamp extrapolation to avoid runaway if data is stale
        if (dt_pos > 0.5f) dt_pos = 0.5f;

        if (ctx->has_global_pos) {
            // Dead-reckon in global coords using velocity.
            // Prefer LPOS velocity if non-zero, otherwise use GPOS-derived velocity.
            float vx = ctx->last_vx, vy = ctx->last_vy, vz = ctx->last_vz;
            if (vx == 0.0f && vy == 0.0f && vz == 0.0f) {
                vx = ctx->gpos_vx;
                vy = ctx->gpos_vy;
                vz = ctx->gpos_vz;
            }

            double meters_per_deg_lat = 111132.92;
            double meters_per_deg_lon = 111132.92 * cos(ctx->last_lat_deg * DEG_TO_RAD);
            if (meters_per_deg_lon < 1.0) meters_per_deg_lon = 1.0;

            double lat = ctx->last_lat_deg + (double)(vx * dt_pos) / meters_per_deg_lat;
            double lon = ctx->last_lon_deg + (double)(vy * dt_pos) / meters_per_deg_lon;
            float alt = ctx->last_alt_m - vz * dt_pos;

            ctx->state.lat = (int32_t)(lat * 1e7);
            ctx->state.lon = (int32_t)(lon * 1e7);
            ctx->state.alt = (int32_t)(alt * 1000.0f);
        } else {
            // Dead-reckon in local NED, then convert
            float x = ctx->last_x + ctx->last_vx * dt_pos;
            float y = ctx->last_y + ctx->last_vy * dt_pos;
            float z = ctx->last_z + ctx->last_vz * dt_pos;

            local_to_global(ctx->ref_lat, ctx->ref_lon, ctx->ref_alt,
                            x, y, z,
                            &ctx->state.lat, &ctx->state.lon, &ctx->state.alt);
        }
    }

    if (ctx->parser.eof) {
        if (looping) {
            ulog_parser_rewind(&ctx->parser);
            ctx->wall_accum = 0.0;
            ctx->first_pos_set = false;
            return true;
        }
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Seek
// ---------------------------------------------------------------------------

void ulog_replay_seek(ulog_replay_ctx_t *ctx, float target_s) {
    if (target_s < 0.0f) target_s = 0.0f;

    float max_s = (float)((double)(ctx->parser.end_timestamp -
                                    ctx->parser.start_timestamp) / 1e6);
    if (target_s > max_s) target_s = max_s;

    uint64_t target_usec = ctx->parser.start_timestamp +
                           (uint64_t)(target_s * 1e6);
    ulog_parser_seek(&ctx->parser, target_usec);
    ctx->wall_accum = (double)target_s;

    // Update current_nav_state for the seek position
    ctx->current_nav_state = 0xFF;
    for (int i = ctx->mode_change_count - 1; i >= 0; i--) {
        if (ctx->mode_changes[i].time_s <= target_s) {
            ctx->current_nav_state = ctx->mode_changes[i].nav_state;
            break;
        }
    }

    // Read forward to populate state at the seek point
    ulog_data_msg_t dmsg;
    for (int i = 0; i < 2000 && ulog_parser_next(&ctx->parser, &dmsg); i++) {
        process_message(ctx, &dmsg);
        if (dmsg.timestamp >= target_usec) break;
    }
}

// ---------------------------------------------------------------------------
// Close
// ---------------------------------------------------------------------------

void ulog_replay_close(ulog_replay_ctx_t *ctx) {
    ulog_parser_close(&ctx->parser);
}
