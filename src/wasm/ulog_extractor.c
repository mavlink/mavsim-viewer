// WASM-only ULog extractor. See header for purpose. Structure mirrors
// src/ulog_parser.c + the pre-scan loop inside src/ulog_replay.c at
// init-time, but is a freshly-written implementation so that no modification
// to native source files is required.
//
// Single-pass design: one read of the file walks both the definition section
// and the data section. Format tables, subscription tables, field offset
// caches, and pre-scan state are all built up as messages arrive. Events are
// appended to the timeline as they stream past. After the walk, the raw
// file bytes can be freed by the caller.

#include "ulog_extractor.h"
#include "ulog_timeline.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// ULog binary format constants
// ---------------------------------------------------------------------------

#define ULOG_HEADER_SIZE     16
#define ULOG_MSG_HEADER_SIZE 3

#define ULOG_MSG_FORMAT      'F'
#define ULOG_MSG_ADD_LOGGED  'A'
#define ULOG_MSG_DATA        'D'
#define ULOG_MSG_LOGGING     'L'

static const uint8_t ULOG_MAGIC[7] = { 'U', 'L', 'o', 'g', 0x01, 0x12, 0x35 };

#define EXTRACTOR_MAX_FORMATS       256
#define EXTRACTOR_MAX_SUBS          256
#define EXTRACTOR_MAX_FIELDS        64

// ---------------------------------------------------------------------------
// Internal types (freshly written, parallel to native's equivalents)
// ---------------------------------------------------------------------------

typedef enum {
    FT_UINT8, FT_INT8,
    FT_UINT16, FT_INT16,
    FT_UINT32, FT_INT32,
    FT_UINT64, FT_INT64,
    FT_FLOAT, FT_DOUBLE,
    FT_BOOL, FT_CHAR,
    FT_UNKNOWN,
} field_type_t;

typedef struct {
    char         name[64];
    field_type_t type;
    int          array_size;
    int          offset;          // byte offset within DATA payload (after msg_id)
} field_t;

typedef struct {
    char    name[64];
    field_t fields[EXTRACTOR_MAX_FIELDS];
    int     field_count;
    int     size;                 // total payload bytes
} format_t;

typedef struct {
    uint16_t msg_id;
    uint8_t  multi_id;
    char     message_name[64];
    int      format_idx;          // index into formats[]
} subscription_t;

// Field offset cache, resolved lazily when the first DATA message for each
// subscription is seen. Matches the slots defined in ulog_field_cache_t on
// native, same field names where they carry the same meaning.
typedef struct {
    int att_q_offset;

    int gpos_lat_offset, gpos_lon_offset, gpos_alt_offset;

    int lpos_x_offset, lpos_y_offset, lpos_z_offset;
    int lpos_vx_offset, lpos_vy_offset, lpos_vz_offset;
    int lpos_ref_lat_offset, lpos_ref_lon_offset, lpos_ref_alt_offset;
    int lpos_xy_global_offset, lpos_z_global_offset;

    int aspd_ias_offset, aspd_tas_offset;

    int vstatus_type_offset, vstatus_is_vtol_offset, vstatus_nav_state_offset;

    int home_lat_offset, home_lon_offset, home_alt_offset, home_valid_hpos_offset;
} field_cache_t;

// ---------------------------------------------------------------------------
// Walker state — everything the walk needs, kept in one struct so the
// recursive helpers don't need long argument lists
// ---------------------------------------------------------------------------

typedef struct {
    const uint8_t *buf;
    size_t         len;
    size_t         pos;           // current read offset into buf

    format_t       formats[EXTRACTOR_MAX_FORMATS];
    int            format_count;

    subscription_t subs[EXTRACTOR_MAX_SUBS];
    int            sub_count;

    field_cache_t  cache;

    // Cached subscription indices (-1 if topic not present in this log)
    int sub_attitude;
    int sub_global_pos;
    int sub_local_pos;
    int sub_airspeed;
    int sub_vehicle_status;
    int sub_home_pos;

    // Cached msg_ids for faster dispatch
    uint16_t att_msg_id;
    uint16_t gpos_msg_id;
    uint16_t lpos_msg_id;
    uint16_t aspd_msg_id;
    uint16_t vstatus_msg_id;
    uint16_t home_msg_id;

    // Output (populated during walk)
    ulog_extract_result_t *out;
    uint64_t               start_timestamp_us;   // populated from first DATA message
    uint8_t                first_seen;

    // Pre-scan bookkeeping
    uint8_t got_vehicle_type;
    uint8_t got_home;
    uint8_t seen_gpos_data;
    uint8_t prev_nav;

    // CUSUM takeoff detection state
    float cusum_s;
    float cusum_k;
    float cusum_h;
    uint8_t cusum_triggered;
    float cusum_trigger_time;
    float cusum_peak;

    // read scratch buffer sized to the current message
    uint8_t *read_buf;
    int      read_buf_cap;
} walker_t;

// ---------------------------------------------------------------------------
// Byte-level reads (unaligned-safe via memcpy)
// ---------------------------------------------------------------------------

static float read_float_at(const uint8_t *buf, int offset) {
    float v; memcpy(&v, buf + offset, sizeof(v)); return v;
}
static double read_double_at(const uint8_t *buf, int offset) {
    double v; memcpy(&v, buf + offset, sizeof(v)); return v;
}
static uint8_t read_uint8_at(const uint8_t *buf, int offset) {
    return buf[offset];
}

// Buffer reads (bounds-checked against walker)
static int w_read(walker_t *w, void *dst, size_t n) {
    if (w->pos + n > w->len) return -1;
    memcpy(dst, w->buf + w->pos, n);
    w->pos += n;
    return 0;
}

static int w_skip(walker_t *w, size_t n) {
    if (w->pos + n > w->len) return -1;
    w->pos += n;
    return 0;
}

static int ensure_read_buf(walker_t *w, int needed) {
    if (needed <= w->read_buf_cap) return 0;
    int new_cap = w->read_buf_cap ? w->read_buf_cap : 4096;
    while (new_cap < needed) new_cap *= 2;
    uint8_t *new_buf = realloc(w->read_buf, new_cap);
    if (!new_buf) return -1;
    w->read_buf = new_buf;
    w->read_buf_cap = new_cap;
    return 0;
}

// ---------------------------------------------------------------------------
// Format string parsing (matches native's parse_format / parse_type_str)
// ---------------------------------------------------------------------------

static field_type_t parse_type_str(const char *s) {
    if (strcmp(s, "float") == 0)    return FT_FLOAT;
    if (strcmp(s, "double") == 0)   return FT_DOUBLE;
    if (strcmp(s, "uint8_t") == 0)  return FT_UINT8;
    if (strcmp(s, "int8_t") == 0)   return FT_INT8;
    if (strcmp(s, "uint16_t") == 0) return FT_UINT16;
    if (strcmp(s, "int16_t") == 0)  return FT_INT16;
    if (strcmp(s, "uint32_t") == 0) return FT_UINT32;
    if (strcmp(s, "int32_t") == 0)  return FT_INT32;
    if (strcmp(s, "uint64_t") == 0) return FT_UINT64;
    if (strcmp(s, "int64_t") == 0)  return FT_INT64;
    if (strcmp(s, "bool") == 0)     return FT_BOOL;
    if (strcmp(s, "char") == 0)     return FT_CHAR;
    return FT_UNKNOWN;
}

static int field_elem_size(field_type_t t) {
    switch (t) {
        case FT_UINT8: case FT_INT8: case FT_BOOL: case FT_CHAR: return 1;
        case FT_UINT16: case FT_INT16:                           return 2;
        case FT_UINT32: case FT_INT32: case FT_FLOAT:            return 4;
        case FT_UINT64: case FT_INT64: case FT_DOUBLE:           return 8;
        default: return 0;
    }
}

static int parse_format(const char *fmt_str, format_t *fmt) {
    memset(fmt, 0, sizeof(*fmt));
    const char *colon = strchr(fmt_str, ':');
    if (!colon) return -1;

    int name_len = (int)(colon - fmt_str);
    if (name_len >= (int)sizeof(fmt->name)) name_len = (int)sizeof(fmt->name) - 1;
    memcpy(fmt->name, fmt_str, name_len);
    fmt->name[name_len] = '\0';

    const char *p = colon + 1;
    int offset = 0;

    while (*p && fmt->field_count < EXTRACTOR_MAX_FIELDS) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == ';') { if (*p) p++; continue; }

        const char *space = strchr(p, ' ');
        if (!space) break;

        char type_buf[64];
        int type_len = (int)(space - p);
        if (type_len >= (int)sizeof(type_buf)) type_len = (int)sizeof(type_buf) - 1;
        memcpy(type_buf, p, type_len);
        type_buf[type_len] = '\0';

        const char *name_start = space + 1;
        const char *semi = strchr(name_start, ';');
        const char *name_end = semi ? semi : name_start + strlen(name_start);

        field_t *f = &fmt->fields[fmt->field_count];
        int fname_len = (int)(name_end - name_start);
        if (fname_len >= (int)sizeof(f->name)) fname_len = (int)sizeof(f->name) - 1;
        memcpy(f->name, name_start, fname_len);
        f->name[fname_len] = '\0';

        f->array_size = 1;
        char base_type[64];
        char *bracket = strchr(type_buf, '[');
        if (bracket) {
            int base_len = (int)(bracket - type_buf);
            memcpy(base_type, type_buf, base_len);
            base_type[base_len] = '\0';
            f->array_size = atoi(bracket + 1);
            if (f->array_size < 1) f->array_size = 1;
        } else {
            strcpy(base_type, type_buf);
        }

        f->type = parse_type_str(base_type);
        int elem_sz = field_elem_size(f->type);
        if (elem_sz == 0) {
            p = semi ? semi + 1 : name_end;
            continue;
        }

        f->offset = offset;
        offset += elem_sz * f->array_size;
        fmt->field_count++;

        p = semi ? semi + 1 : name_end;
    }

    fmt->size = offset;
    return 0;
}

static int find_field(const format_t *fmt, const char *name) {
    if (!fmt) return -1;
    for (int i = 0; i < fmt->field_count; i++) {
        if (strcmp(fmt->fields[i].name, name) == 0)
            return fmt->fields[i].offset;
    }
    return -1;
}

static int find_subscription(const walker_t *w, const char *name) {
    for (int i = 0; i < w->sub_count; i++) {
        if (strcmp(w->subs[i].message_name, name) == 0 && w->subs[i].multi_id == 0)
            return i;
    }
    return -1;
}

static void process_add_logged(walker_t *w, const uint8_t *buf, uint16_t msg_size) {
    if (w->sub_count >= EXTRACTOR_MAX_SUBS) return;
    if (msg_size < 3) return;
    subscription_t *sub = &w->subs[w->sub_count];
    sub->multi_id = buf[0];
    sub->msg_id = (uint16_t)(buf[1] | (buf[2] << 8));
    int name_len = msg_size - 3;
    if (name_len >= (int)sizeof(sub->message_name))
        name_len = (int)sizeof(sub->message_name) - 1;
    memcpy(sub->message_name, buf + 3, name_len);
    sub->message_name[name_len] = '\0';

    sub->format_idx = -1;
    for (int i = 0; i < w->format_count; i++) {
        if (strcmp(w->formats[i].name, sub->message_name) == 0) {
            sub->format_idx = i;
            break;
        }
    }
    w->sub_count++;
}

// ---------------------------------------------------------------------------
// Cache resolution — called once after we've seen all the subscriptions we
// need in the definition section. Any additional subs picked up in the data
// section re-resolve individual entries.
// ---------------------------------------------------------------------------

static void init_cache(field_cache_t *c) {
    memset(c, 0xFF, sizeof(*c));  // -1 = not found
}

static void resolve_subs_and_cache(walker_t *w) {
    w->sub_attitude       = find_subscription(w, "vehicle_attitude");
    w->sub_global_pos     = find_subscription(w, "vehicle_global_position");
    w->sub_local_pos      = find_subscription(w, "vehicle_local_position");
    w->sub_airspeed       = find_subscription(w, "airspeed_validated");
    w->sub_vehicle_status = find_subscription(w, "vehicle_status");
    w->sub_home_pos       = find_subscription(w, "home_position");

    w->att_msg_id     = (w->sub_attitude       >= 0) ? w->subs[w->sub_attitude].msg_id       : 0xFFFF;
    w->gpos_msg_id    = (w->sub_global_pos     >= 0) ? w->subs[w->sub_global_pos].msg_id     : 0xFFFF;
    w->lpos_msg_id    = (w->sub_local_pos      >= 0) ? w->subs[w->sub_local_pos].msg_id      : 0xFFFF;
    w->aspd_msg_id    = (w->sub_airspeed       >= 0) ? w->subs[w->sub_airspeed].msg_id       : 0xFFFF;
    w->vstatus_msg_id = (w->sub_vehicle_status >= 0) ? w->subs[w->sub_vehicle_status].msg_id : 0xFFFF;
    w->home_msg_id    = (w->sub_home_pos       >= 0) ? w->subs[w->sub_home_pos].msg_id       : 0xFFFF;

    if (w->sub_attitude >= 0) {
        const format_t *f = &w->formats[w->subs[w->sub_attitude].format_idx];
        w->cache.att_q_offset = find_field(f, "q");
    }

    if (w->sub_global_pos >= 0) {
        const format_t *f = &w->formats[w->subs[w->sub_global_pos].format_idx];
        w->cache.gpos_lat_offset = find_field(f, "lat");
        w->cache.gpos_lon_offset = find_field(f, "lon");
        w->cache.gpos_alt_offset = find_field(f, "alt");
    }

    if (w->sub_local_pos >= 0) {
        const format_t *f = &w->formats[w->subs[w->sub_local_pos].format_idx];
        w->cache.lpos_x_offset  = find_field(f, "x");
        w->cache.lpos_y_offset  = find_field(f, "y");
        w->cache.lpos_z_offset  = find_field(f, "z");
        w->cache.lpos_vx_offset = find_field(f, "vx");
        w->cache.lpos_vy_offset = find_field(f, "vy");
        w->cache.lpos_vz_offset = find_field(f, "vz");
        w->cache.lpos_ref_lat_offset   = find_field(f, "ref_lat");
        w->cache.lpos_ref_lon_offset   = find_field(f, "ref_lon");
        w->cache.lpos_ref_alt_offset   = find_field(f, "ref_alt");
        w->cache.lpos_xy_global_offset = find_field(f, "xy_global");
        w->cache.lpos_z_global_offset  = find_field(f, "z_global");
    }

    if (w->sub_airspeed >= 0) {
        const format_t *f = &w->formats[w->subs[w->sub_airspeed].format_idx];
        w->cache.aspd_ias_offset = find_field(f, "indicated_airspeed_m_s");
        w->cache.aspd_tas_offset = find_field(f, "true_airspeed_m_s");
    }

    if (w->sub_vehicle_status >= 0) {
        const format_t *f = &w->formats[w->subs[w->sub_vehicle_status].format_idx];
        w->cache.vstatus_type_offset      = find_field(f, "vehicle_type");
        w->cache.vstatus_is_vtol_offset   = find_field(f, "is_vtol");
        w->cache.vstatus_nav_state_offset = find_field(f, "nav_state");
    }

    if (w->sub_home_pos >= 0) {
        const format_t *f = &w->formats[w->subs[w->sub_home_pos].format_idx];
        w->cache.home_lat_offset        = find_field(f, "lat");
        w->cache.home_lon_offset        = find_field(f, "lon");
        w->cache.home_alt_offset        = find_field(f, "alt");
        w->cache.home_valid_hpos_offset = find_field(f, "valid_hpos");
    }
}

// ---------------------------------------------------------------------------
// Per-type extraction. Each function reads the fields it cares about from
// the payload, builds an event struct, and appends it to the timeline. It
// also does the pre-scan state updates that native does during its init walk.
//
// The `payload` pointer here is the DATA message payload AFTER the 2-byte
// msg_id (matches native's ulog_data_msg_t.data semantics: data points at
// "msg_id + 2", i.e. starts with the timestamp).
// ---------------------------------------------------------------------------

static void extract_attitude(walker_t *w, uint64_t ts_us, const uint8_t *payload) {
    if (w->cache.att_q_offset < 0) return;
    ulog_att_event_t ev;
    ev.timestamp_us = ts_us;
    int off = w->cache.att_q_offset;
    ev.q[0] = read_float_at(payload, off + 0);
    ev.q[1] = read_float_at(payload, off + 4);
    ev.q[2] = read_float_at(payload, off + 8);
    ev.q[3] = read_float_at(payload, off + 12);
    ulog_timeline_append_att(w->out->timeline, &ev);
}

static void extract_gpos(walker_t *w, uint64_t ts_us, const uint8_t *payload) {
    w->seen_gpos_data = 1;
    if (w->cache.gpos_lat_offset < 0 ||
        w->cache.gpos_lon_offset < 0 ||
        w->cache.gpos_alt_offset < 0) return;

    double lat = read_double_at(payload, w->cache.gpos_lat_offset);
    double lon = read_double_at(payload, w->cache.gpos_lon_offset);
    float  alt = read_float_at (payload, w->cache.gpos_alt_offset);

    ulog_gpos_event_t ev;
    ev.timestamp_us = ts_us;
    ev.lat_deg = lat;
    ev.lon_deg = lon;
    ev.alt_m   = alt;
    ev._pad = 0.0f;
    ulog_timeline_append_gpos(w->out->timeline, &ev);

    // Home fallback: if we haven't resolved home yet via the home_position
    // topic, first gpos sample with non-zero coordinates becomes home.
    if (!w->got_home && (lat != 0.0 || lon != 0.0)) {
        w->out->home.lat = (int32_t)(lat * 1e7);
        w->out->home.lon = (int32_t)(lon * 1e7);
        w->out->home.alt = (int32_t)(alt * 1000.0f);
        w->out->home.valid = true;
        w->got_home = 1;
    }
}

static void extract_lpos(walker_t *w, uint64_t ts_us, const uint8_t *payload) {
    if (w->cache.lpos_x_offset < 0) return;

    ulog_lpos_event_t ev;
    ev.timestamp_us = ts_us;
    ev.x  = read_float_at(payload, w->cache.lpos_x_offset);
    ev.y  = read_float_at(payload, w->cache.lpos_y_offset);
    ev.z  = read_float_at(payload, w->cache.lpos_z_offset);
    ev.vx = read_float_at(payload, w->cache.lpos_vx_offset);
    ev.vy = read_float_at(payload, w->cache.lpos_vy_offset);
    ev.vz = read_float_at(payload, w->cache.lpos_vz_offset);
    ulog_timeline_append_lpos(w->out->timeline, &ev);

    // LPOS reference point — capture once from first lpos sample with xy_global.
    if (!w->out->ref_set && !w->out->ref_rejected && w->cache.lpos_xy_global_offset >= 0) {
        uint8_t xy_global = read_uint8_at(payload, w->cache.lpos_xy_global_offset);
        if (xy_global) {
            double rlat = read_double_at(payload, w->cache.lpos_ref_lat_offset);
            double rlon = read_double_at(payload, w->cache.lpos_ref_lon_offset);
            if (rlat < -90.0 || rlat > 90.0 || rlon < -180.0 || rlon > 180.0) {
                w->out->ref_rejected = 1;
            } else {
                w->out->ref_lat_deg = rlat;
                w->out->ref_lon_deg = rlon;
                if (w->cache.lpos_z_global_offset >= 0 &&
                    read_uint8_at(payload, w->cache.lpos_z_global_offset)) {
                    w->out->ref_alt_m = read_float_at(payload, w->cache.lpos_ref_alt_offset);
                }
                w->out->ref_set = 1;
            }
        }
    }

    // CUSUM takeoff detection on upward velocity (-vz = upward in NED)
    if (!w->cusum_triggered && w->cache.lpos_vz_offset >= 0) {
        float up = -ev.vz;
        w->cusum_s += (up - w->cusum_k);
        if (w->cusum_s < 0.0f) w->cusum_s = 0.0f;
        if (w->cusum_s > w->cusum_h) {
            w->cusum_trigger_time = (float)((double)(ts_us - w->start_timestamp_us) / 1e6);
            w->cusum_peak = w->cusum_s;
            w->cusum_triggered = 1;
        }
    }
}

static void extract_aspd(walker_t *w, uint64_t ts_us, const uint8_t *payload) {
    if (w->cache.aspd_ias_offset < 0 || w->cache.aspd_tas_offset < 0) return;
    float ias = read_float_at(payload, w->cache.aspd_ias_offset);
    float tas = read_float_at(payload, w->cache.aspd_tas_offset);
    ulog_aspd_event_t ev;
    ev.timestamp_us = ts_us;
    ev.ias_cms = (uint16_t)(ias * 100.0f);
    ev.tas_cms = (uint16_t)(tas * 100.0f);
    ev._pad = 0;
    ulog_timeline_append_aspd(w->out->timeline, &ev);
}

static void extract_vstatus(walker_t *w, uint64_t ts_us, const uint8_t *payload) {
    if (w->cache.vstatus_type_offset < 0) return;

    uint8_t px4_type = read_uint8_at(payload, w->cache.vstatus_type_offset);
    uint8_t is_vtol  = (w->cache.vstatus_is_vtol_offset >= 0)
                         ? read_uint8_at(payload, w->cache.vstatus_is_vtol_offset) : 0;
    uint8_t nav      = (w->cache.vstatus_nav_state_offset >= 0)
                         ? read_uint8_at(payload, w->cache.vstatus_nav_state_offset) : 0xFF;

    uint8_t mav_type;
    if (is_vtol) {
        mav_type = 22;  // MAV_TYPE_VTOL_FIXEDROTOR
    } else {
        switch (px4_type) {
            case 1: mav_type = 2;  break;   // ROTARY_WING → MAV_TYPE_QUADROTOR
            case 2: mav_type = 1;  break;   // FIXED_WING  → MAV_TYPE_FIXED_WING
            case 3: mav_type = 10; break;   // ROVER       → MAV_TYPE_GROUND_ROVER
            default: mav_type = 2; break;
        }
    }

    ulog_vstatus_event_t ev;
    ev.timestamp_us = ts_us;
    ev.vehicle_type = mav_type;
    ev.is_vtol = is_vtol;
    ev.nav_state = nav;
    memset(ev._pad, 0, sizeof(ev._pad));
    ulog_timeline_append_vstatus(w->out->timeline, &ev);

    // First vstatus sets the extract-result vehicle_type
    if (!w->got_vehicle_type) {
        w->out->vehicle_type = mav_type;
        w->got_vehicle_type = 1;
    }

    // Mode change tracking
    if (nav != w->prev_nav && w->out->mode_change_count < ULOG_MAX_MODE_CHANGES) {
        float t = (float)((double)(ts_us - w->start_timestamp_us) / 1e6);
        w->out->mode_changes[w->out->mode_change_count].time_s = t;
        w->out->mode_changes[w->out->mode_change_count].nav_state = nav;
        w->out->mode_change_count++;
        w->prev_nav = nav;
    }
}

static void extract_home(walker_t *w, uint64_t ts_us, const uint8_t *payload) {
    if (w->cache.home_lat_offset < 0) return;

    uint8_t hpos_valid = 1;
    if (w->cache.home_valid_hpos_offset >= 0)
        hpos_valid = read_uint8_at(payload, w->cache.home_valid_hpos_offset) != 0;

    double lat = read_double_at(payload, w->cache.home_lat_offset);
    double lon = read_double_at(payload, w->cache.home_lon_offset);
    float  alt = read_float_at (payload, w->cache.home_alt_offset);

    ulog_home_event_t ev;
    ev.timestamp_us = ts_us;
    ev.lat_deg = lat;
    ev.lon_deg = lon;
    ev.alt_m = alt;
    ev.valid_hpos = hpos_valid;
    memset(ev._pad, 0, sizeof(ev._pad));
    ulog_timeline_append_home(w->out->timeline, &ev);

    // Pre-scan home resolution (Tier 1: authoritative home_position topic)
    if (!w->got_home && hpos_valid && (lat != 0.0 || lon != 0.0)) {
        w->out->home.lat = (int32_t)(lat * 1e7);
        w->out->home.lon = (int32_t)(lon * 1e7);
        w->out->home.alt = (int32_t)(alt * 1000.0f);
        w->out->home.valid = true;
        w->out->home_from_topic = 1;
        w->got_home = 1;
    }
}

static void extract_statustext(walker_t *w, uint64_t ts_us,
                                uint8_t severity, const uint8_t *text, int text_len) {
    ulog_statustext_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.timestamp_us = ts_us;
    ev.severity = severity;
    int n = text_len < STATUSTEXT_MSG_MAX - 1 ? text_len : STATUSTEXT_MSG_MAX - 1;
    memcpy(ev.text, text, n);
    ev.text[n] = '\0';
    ulog_timeline_append_statustext(w->out->timeline, &ev);
}

// ---------------------------------------------------------------------------
// Dispatch one DATA message
// ---------------------------------------------------------------------------

static void dispatch_data(walker_t *w, uint16_t msg_id, uint64_t ts_us, const uint8_t *payload) {
    if (msg_id == w->att_msg_id)          extract_attitude(w, ts_us, payload);
    else if (msg_id == w->lpos_msg_id)    extract_lpos(w, ts_us, payload);
    else if (msg_id == w->gpos_msg_id)    extract_gpos(w, ts_us, payload);
    else if (msg_id == w->aspd_msg_id)    extract_aspd(w, ts_us, payload);
    else if (msg_id == w->vstatus_msg_id) extract_vstatus(w, ts_us, payload);
    else if (msg_id == w->home_msg_id)    extract_home(w, ts_us, payload);
    // Other subscriptions are logged by PX4 but ignored by Hawkeye's replay.
}

// ---------------------------------------------------------------------------
// Public entry point: ulog_extract
// ---------------------------------------------------------------------------

int ulog_extract(const uint8_t *buf, size_t len, ulog_extract_result_t *out) {
    if (!buf || !out || !out->timeline) return -1;

    // walker_t is ~1.3 MB (formats[256] of ~5 KB each) — too big for the
    // WASM stack (default 1 MB). Heap-allocate to avoid stack overflow.
    walker_t *w = calloc(1, sizeof(walker_t));
    if (!w) return -1;

    w->buf = buf;
    w->len = len;
    w->pos = 0;
    w->out = out;
    w->sub_attitude = -1;
    w->sub_global_pos = -1;
    w->sub_local_pos = -1;
    w->sub_airspeed = -1;
    w->sub_vehicle_status = -1;
    w->sub_home_pos = -1;
    w->att_msg_id = w->gpos_msg_id = w->lpos_msg_id = 0xFFFF;
    w->aspd_msg_id = w->vstatus_msg_id = w->home_msg_id = 0xFFFF;
    w->prev_nav = 0xFF;
    w->cusum_k = 0.3f;
    w->cusum_h = 2.0f;
    init_cache(&w->cache);

    // Initialize extract result fields that need non-zero defaults
    out->vehicle_type = 2;           // default MAV_TYPE_QUADROTOR
    out->current_nav_state = 0xFF;
    out->mode_change_count = 0;
    out->home.valid = false;
    out->home_from_topic = 0;
    out->home_rejected = 0;
    out->ref_set = 0;
    out->ref_rejected = 0;
    out->ref_lat_deg = 0.0;
    out->ref_lon_deg = 0.0;
    out->ref_alt_m = 0.0f;
    out->takeoff_detected = 0;
    out->takeoff_time_s = 0.0f;
    out->takeoff_conf = 0.0f;

    // --- Validate ULog header ---
    uint8_t header[ULOG_HEADER_SIZE];
    if (w_read(w, header, ULOG_HEADER_SIZE) != 0) {
        fprintf(stderr, "ulog_extract: file too short\n");
        goto fail;
    }
    if (memcmp(header, ULOG_MAGIC, 7) != 0) {
        fprintf(stderr, "ulog_extract: invalid magic\n");
        goto fail;
    }

    // --- Definition section: read FORMAT and ADD_LOGGED_MSG until first DATA ---
    uint8_t hit_data_section = 0;
    while (!hit_data_section) {
        uint8_t msg_hdr[ULOG_MSG_HEADER_SIZE];
        if (w_read(w, msg_hdr, ULOG_MSG_HEADER_SIZE) != 0) goto fail;
        uint16_t msg_size = (uint16_t)(msg_hdr[0] | (msg_hdr[1] << 8));
        uint8_t  msg_type = msg_hdr[2];

        if (ensure_read_buf(w, msg_size) != 0) goto fail;
        if (w_read(w, w->read_buf, msg_size) != 0) goto fail;

        switch ((char)msg_type) {
            case ULOG_MSG_FORMAT: {
                if (w->format_count < EXTRACTOR_MAX_FORMATS) {
                    char fmt_str[2048];
                    int n = msg_size < (int)sizeof(fmt_str) - 1 ? msg_size : (int)sizeof(fmt_str) - 1;
                    memcpy(fmt_str, w->read_buf, n);
                    fmt_str[n] = '\0';
                    parse_format(fmt_str, &w->formats[w->format_count]);
                    w->format_count++;
                }
                break;
            }
            case ULOG_MSG_ADD_LOGGED:
                process_add_logged(w, w->read_buf, msg_size);
                break;
            case ULOG_MSG_DATA: {
                // First DATA — back up to let the main loop read it properly.
                w->pos -= (ULOG_MSG_HEADER_SIZE + msg_size);
                hit_data_section = 1;
                break;
            }
            default:
                // Skip other message types (INFO, PARAMETER, SYNC, etc.)
                break;
        }
    }

    // Resolve subscription indices and field offset cache now that we have
    // (most of) the definitions. Any ADD_LOGGED that shows up in the data
    // section will trigger a re-resolve.
    resolve_subs_and_cache(w);

    // --- Data section: walk DATA + LOGGING + in-stream ADD_LOGGED ---
    for (;;) {
        uint8_t msg_hdr[ULOG_MSG_HEADER_SIZE];
        if (w_read(w, msg_hdr, ULOG_MSG_HEADER_SIZE) != 0) break;   // EOF — normal exit
        uint16_t msg_size = (uint16_t)(msg_hdr[0] | (msg_hdr[1] << 8));
        uint8_t  msg_type = msg_hdr[2];

        if (ensure_read_buf(w, msg_size) != 0) goto fail;
        if (w_read(w, w->read_buf, msg_size) != 0) break;             // partial trailing msg — tolerate

        if ((char)msg_type == ULOG_MSG_ADD_LOGGED) {
            process_add_logged(w, w->read_buf, msg_size);
            resolve_subs_and_cache(w);  // refresh cache for the new sub
            continue;
        }

        if ((char)msg_type == ULOG_MSG_DATA && msg_size >= 10) {
            uint16_t msg_id = (uint16_t)(w->read_buf[0] | (w->read_buf[1] << 8));
            uint64_t ts_us;
            memcpy(&ts_us, w->read_buf + 2, 8);

            if (!w->first_seen) {
                w->start_timestamp_us = ts_us;
                w->first_seen = 1;
                out->timeline->start_timestamp_us = ts_us;
            }
            out->timeline->end_timestamp_us = ts_us;

            // payload in dispatch_data points at w->read_buf + 2, matching native's
            // ulog_data_msg_t.data semantics (starts with timestamp).
            dispatch_data(w, msg_id, ts_us, w->read_buf + 2);
            continue;
        }

        if ((char)msg_type == ULOG_MSG_LOGGING && msg_size >= 9) {
            uint8_t  severity = w->read_buf[0];
            uint64_t ts_us;
            memcpy(&ts_us, w->read_buf + 1, 8);
            const uint8_t *text = w->read_buf + 9;
            int text_len = msg_size - 9;
            extract_statustext(w, ts_us, severity, text, text_len);
            continue;
        }

        // Other message types: ignored (DROPOUT, INFO_MULTI, PARAM, SYNC, ...)
    }

    // --- Finalize pre-scan results ---

    // Home Tier validation: if home came from the home_position topic but
    // there's no gpos data to confirm the position is globally referenced,
    // the home is unreliable (baro-only) and gets rejected.
    if (w->got_home && out->home_from_topic && !w->seen_gpos_data) {
        out->home.valid = false;
        out->home_from_topic = 0;
        out->home_rejected = 1;
    }

    // Takeoff detection results
    out->takeoff_detected = w->cusum_triggered;
    out->takeoff_time_s   = w->cusum_triggered ? w->cusum_trigger_time : 0.0f;

    // Confidence scoring: CUSUM sharpness + nav_state corroboration
    float conf = 0.0f;
    if (w->cusum_triggered) {
        float sharpness = (w->cusum_peak - w->cusum_h) / 8.0f;
        if (sharpness < 0.0f) sharpness = 0.0f;
        if (sharpness > 1.0f) sharpness = 1.0f;
        conf = 0.3f + 0.4f * sharpness;

        // Find the mode active at cusum_trigger_time
        uint8_t active_ns = 0xFF;
        for (int m = out->mode_change_count - 1; m >= 0; m--) {
            if (out->mode_changes[m].time_s <= w->cusum_trigger_time) {
                active_ns = out->mode_changes[m].nav_state;
                break;
            }
        }
        // Takeoff(17) Mission(3) VTOL-Takeoff(22) Offboard(14) Position(2)
        if (active_ns == 17 || active_ns == 3 || active_ns == 22 ||
            active_ns == 14 || active_ns == 2) {
            conf += 0.3f;
        }
        if (conf > 1.0f) conf = 1.0f;
    } else {
        // No CUSUM trigger — check if already airborne at log start
        if (out->mode_change_count > 0) {
            uint8_t ns = out->mode_changes[0].nav_state;
            if (ns == 3 || ns == 4 || ns == 14)
                conf = 0.5f;
        }
    }
    out->takeoff_conf = conf;

    if (out->mode_change_count > 0)
        out->current_nav_state = out->mode_changes[0].nav_state;

    // Shrink timeline arrays to final size so idle capacity is released.
    ulog_timeline_shrink_to_fit(out->timeline);

    free(w->read_buf);
    free(w);
    return 0;

fail:
    free(w->read_buf);
    free(w);
    return -1;
}
