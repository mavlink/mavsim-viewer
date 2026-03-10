#include "ulog_parser.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define ULOG_HEADER_SIZE     16   // 7 magic + 1 version + 8 timestamp
#define ULOG_MSG_HEADER_SIZE 3    // 2 size + 1 type
#define READ_BUF_INITIAL     4096
#define INDEX_INTERVAL_USEC  1000000  // 1 second between index entries

static const uint8_t ULOG_MAGIC[7] = { 'U', 'L', 'o', 'g', 0x01, 0x12, 0x35 };

// ---------------------------------------------------------------------------
// Type helpers
// ---------------------------------------------------------------------------

static ulog_field_type_t parse_type_str(const char *s) {
    if (strcmp(s, "float") == 0)    return ULOG_FIELD_FLOAT;
    if (strcmp(s, "double") == 0)   return ULOG_FIELD_DOUBLE;
    if (strcmp(s, "uint8_t") == 0)  return ULOG_FIELD_UINT8;
    if (strcmp(s, "int8_t") == 0)   return ULOG_FIELD_INT8;
    if (strcmp(s, "uint16_t") == 0) return ULOG_FIELD_UINT16;
    if (strcmp(s, "int16_t") == 0)  return ULOG_FIELD_INT16;
    if (strcmp(s, "uint32_t") == 0) return ULOG_FIELD_UINT32;
    if (strcmp(s, "int32_t") == 0)  return ULOG_FIELD_INT32;
    if (strcmp(s, "uint64_t") == 0) return ULOG_FIELD_UINT64;
    if (strcmp(s, "int64_t") == 0)  return ULOG_FIELD_INT64;
    if (strcmp(s, "bool") == 0)     return ULOG_FIELD_BOOL;
    if (strcmp(s, "char") == 0)     return ULOG_FIELD_CHAR;
    return ULOG_FIELD_UNKNOWN;
}

static int field_elem_size(ulog_field_type_t t) {
    switch (t) {
        case ULOG_FIELD_UINT8:  case ULOG_FIELD_INT8:
        case ULOG_FIELD_BOOL:   case ULOG_FIELD_CHAR:   return 1;
        case ULOG_FIELD_UINT16: case ULOG_FIELD_INT16:  return 2;
        case ULOG_FIELD_UINT32: case ULOG_FIELD_INT32:
        case ULOG_FIELD_FLOAT:                           return 4;
        case ULOG_FIELD_UINT64: case ULOG_FIELD_INT64:
        case ULOG_FIELD_DOUBLE:                          return 8;
        default: return 0;
    }
}

// ---------------------------------------------------------------------------
// Format string parser
// ---------------------------------------------------------------------------

// Parse "topic_name:type1 field1;type2[N] field2;..."
static int parse_format(const char *fmt_str, ulog_format_t *fmt) {
    memset(fmt, 0, sizeof(*fmt));

    const char *colon = strchr(fmt_str, ':');
    if (!colon) return -1;

    int name_len = (int)(colon - fmt_str);
    if (name_len >= (int)sizeof(fmt->name)) name_len = (int)sizeof(fmt->name) - 1;
    memcpy(fmt->name, fmt_str, name_len);
    fmt->name[name_len] = '\0';

    const char *p = colon + 1;
    int offset = 0;

    while (*p && fmt->field_count < ULOG_MAX_FIELDS) {
        // Skip whitespace
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == ';') { if (*p) p++; continue; }

        // Find the space separating type from name
        const char *space = strchr(p, ' ');
        if (!space) break;

        // Extract type string (may contain [N])
        char type_buf[64];
        int type_len = (int)(space - p);
        if (type_len >= (int)sizeof(type_buf)) type_len = (int)sizeof(type_buf) - 1;
        memcpy(type_buf, p, type_len);
        type_buf[type_len] = '\0';

        // Extract field name (until ; or end)
        const char *name_start = space + 1;
        const char *semi = strchr(name_start, ';');
        const char *name_end = semi ? semi : name_start + strlen(name_start);

        ulog_field_t *f = &fmt->fields[fmt->field_count];
        int fname_len = (int)(name_end - name_start);
        if (fname_len >= (int)sizeof(f->name)) fname_len = (int)sizeof(f->name) - 1;
        memcpy(f->name, name_start, fname_len);
        f->name[fname_len] = '\0';

        // Parse array size from type (e.g. "float[4]")
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

        // Skip unknown/nested types but still account for padding-like fields
        if (elem_sz == 0) {
            // Can't compute size — skip this field
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

// ---------------------------------------------------------------------------
// Index management
// ---------------------------------------------------------------------------

static void index_add(ulog_parser_t *p, uint64_t timestamp, long file_offset) {
    if (p->index_count >= p->index_capacity) {
        int new_cap = p->index_capacity ? p->index_capacity * 2 : 256;
        ulog_index_entry_t *new_idx = realloc(p->index, new_cap * sizeof(ulog_index_entry_t));
        if (!new_idx) return;
        p->index = new_idx;
        p->index_capacity = new_cap;
    }
    p->index[p->index_count].timestamp = timestamp;
    p->index[p->index_count].file_offset = file_offset;
    p->index_count++;
}

// ---------------------------------------------------------------------------
// Buffer management
// ---------------------------------------------------------------------------

static int ensure_buf(ulog_parser_t *p, int needed) {
    if (needed <= p->read_buf_size) return 0;
    int new_size = p->read_buf_size ? p->read_buf_size : READ_BUF_INITIAL;
    while (new_size < needed) new_size *= 2;
    uint8_t *new_buf = realloc(p->read_buf, new_size);
    if (!new_buf) return -1;
    p->read_buf = new_buf;
    p->read_buf_size = new_size;
    return 0;
}

// ---------------------------------------------------------------------------
// Process an ADD_LOGGED_MSG payload (shared by Pass 1, Pass 2, and playback)
// ---------------------------------------------------------------------------

static void process_add_logged(ulog_parser_t *p, const uint8_t *buf, uint16_t msg_size) {
    if (p->sub_count >= ULOG_MAX_SUBSCRIPTIONS) return;
    if (msg_size < 3) return;
    ulog_subscription_t *sub = &p->subs[p->sub_count];
    sub->multi_id = buf[0];
    sub->msg_id = buf[1] | (buf[2] << 8);
    int name_len = msg_size - 3;
    if (name_len >= (int)sizeof(sub->message_name))
        name_len = (int)sizeof(sub->message_name) - 1;
    memcpy(sub->message_name, buf + 3, name_len);
    sub->message_name[name_len] = '\0';

    // Match to format by name
    sub->format_idx = -1;
    for (int i = 0; i < p->format_count; i++) {
        if (strcmp(p->formats[i].name, sub->message_name) == 0) {
            sub->format_idx = i;
            break;
        }
    }
    p->sub_count++;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

int ulog_parser_open(ulog_parser_t *p, const char *filepath) {
    memset(p, 0, sizeof(*p));

    p->fp = fopen(filepath, "rb");
    if (!p->fp) {
        fprintf(stderr, "ulog_parser: cannot open %s\n", filepath);
        return -1;
    }

    // Validate header
    uint8_t header[ULOG_HEADER_SIZE];
    if (fread(header, 1, ULOG_HEADER_SIZE, p->fp) != ULOG_HEADER_SIZE) {
        fprintf(stderr, "ulog_parser: file too short\n");
        fclose(p->fp); p->fp = NULL;
        return -1;
    }
    if (memcmp(header, ULOG_MAGIC, 7) != 0) {
        fprintf(stderr, "ulog_parser: invalid magic\n");
        fclose(p->fp); p->fp = NULL;
        return -1;
    }

    // Pass 1: read definition section (FORMAT messages only before data section)
    // ADD_LOGGED_MSG messages can appear both here and in the data section
    bool in_data_section = false;
    while (!in_data_section) {
        uint8_t msg_hdr[ULOG_MSG_HEADER_SIZE];
        if (fread(msg_hdr, 1, ULOG_MSG_HEADER_SIZE, p->fp) != ULOG_MSG_HEADER_SIZE) break;

        uint16_t msg_size = msg_hdr[0] | (msg_hdr[1] << 8);
        uint8_t msg_type = msg_hdr[2];

        if (ensure_buf(p, msg_size) != 0) break;
        if (fread(p->read_buf, 1, msg_size, p->fp) != msg_size) break;

        switch ((char)msg_type) {
            case ULOG_MSG_FORMAT: {
                if (p->format_count >= ULOG_MAX_SUBSCRIPTIONS) break;
                char fmt_str[2048];
                int len = msg_size < (int)sizeof(fmt_str) - 1 ? msg_size : (int)sizeof(fmt_str) - 1;
                memcpy(fmt_str, p->read_buf, len);
                fmt_str[len] = '\0';
                parse_format(fmt_str, &p->formats[p->format_count]);
                p->format_count++;
                break;
            }

            case ULOG_MSG_ADD_LOGGED: {
                process_add_logged(p, p->read_buf, msg_size);
                break;
            }

            case ULOG_MSG_DATA: {
                // We've hit the data section — back up and record offset
                p->data_start_offset = ftell(p->fp) - ULOG_MSG_HEADER_SIZE - msg_size;
                in_data_section = true;
                break;
            }

            default:
                break;
        }
    }

    if (p->data_start_offset == 0) {
        fprintf(stderr, "ulog_parser: no data section found\n");
        fclose(p->fp); p->fp = NULL;
        return -1;
    }

    // Pass 2: scan data section to build timestamp index
    // Also pick up ADD_LOGGED_MSG messages that appear in the data section
    fseek(p->fp, p->data_start_offset, SEEK_SET);
    uint64_t last_indexed = 0;
    bool first = true;

    for (;;) {
        long msg_offset = ftell(p->fp);
        uint8_t msg_hdr[ULOG_MSG_HEADER_SIZE];
        if (fread(msg_hdr, 1, ULOG_MSG_HEADER_SIZE, p->fp) != ULOG_MSG_HEADER_SIZE) break;

        uint16_t msg_size = msg_hdr[0] | (msg_hdr[1] << 8);
        uint8_t msg_type = msg_hdr[2];

        if ((char)msg_type == ULOG_MSG_ADD_LOGGED) {
            // Process ADD_LOGGED messages in the data section
            if (ensure_buf(p, msg_size) != 0) break;
            if (fread(p->read_buf, 1, msg_size, p->fp) != msg_size) break;
            process_add_logged(p, p->read_buf, msg_size);
        } else if ((char)msg_type == ULOG_MSG_DATA && msg_size >= 10) {
            // Read msg_id (2) + timestamp (8) = 10 bytes
            uint8_t ts_buf[10];
            if (fread(ts_buf, 1, 10, p->fp) != 10) break;

            uint64_t ts;
            memcpy(&ts, ts_buf + 2, 8);

            if (first) {
                p->start_timestamp = ts;
                last_indexed = ts;
                index_add(p, ts, msg_offset);
            }

            if (first || ts >= last_indexed + INDEX_INTERVAL_USEC) {
                if (!first) index_add(p, ts, msg_offset);
                last_indexed = ts;
                first = false;
            }

            p->end_timestamp = ts;

            // Skip remaining payload
            int remaining = msg_size - 10;
            if (remaining > 0) fseek(p->fp, remaining, SEEK_CUR);
        } else {
            // Skip non-DATA message
            fseek(p->fp, msg_size, SEEK_CUR);
        }
    }

    // Seek back to data start for playback
    fseek(p->fp, p->data_start_offset, SEEK_SET);
    p->eof = false;

    return 0;
}

bool ulog_parser_next(ulog_parser_t *p, ulog_data_msg_t *out) {
    for (;;) {
        uint8_t msg_hdr[ULOG_MSG_HEADER_SIZE];
        if (fread(msg_hdr, 1, ULOG_MSG_HEADER_SIZE, p->fp) != ULOG_MSG_HEADER_SIZE) {
            p->eof = true;
            return false;
        }

        uint16_t msg_size = msg_hdr[0] | (msg_hdr[1] << 8);
        uint8_t msg_type = msg_hdr[2];

        if (ensure_buf(p, msg_size) != 0) {
            p->eof = true;
            return false;
        }

        if (fread(p->read_buf, 1, msg_size, p->fp) != msg_size) {
            p->eof = true;
            return false;
        }

        if ((char)msg_type == ULOG_MSG_DATA && msg_size >= 10) {
            out->msg_id = p->read_buf[0] | (p->read_buf[1] << 8);
            memcpy(&out->timestamp, p->read_buf + 2, 8);
            out->data = p->read_buf + 2;  // skip msg_id, include timestamp
            out->data_len = msg_size - 2;
            return true;
        }
        // Skip non-DATA messages (DROPOUT, LOGGING, etc.)
    }
}

void ulog_parser_rewind(ulog_parser_t *p) {
    fseek(p->fp, p->data_start_offset, SEEK_SET);
    p->eof = false;
}

int ulog_parser_seek(ulog_parser_t *p, uint64_t target_usec) {
    if (p->index_count == 0) {
        ulog_parser_rewind(p);
        return 0;
    }

    // Binary search for largest entry with timestamp <= target
    int lo = 0, hi = p->index_count - 1;
    int best = 0;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (p->index[mid].timestamp <= target_usec) {
            best = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }

    fseek(p->fp, p->index[best].file_offset, SEEK_SET);
    p->eof = false;
    return 0;
}

int ulog_parser_find_subscription(const ulog_parser_t *p, const char *name) {
    for (int i = 0; i < p->sub_count; i++) {
        if (strcmp(p->subs[i].message_name, name) == 0 && p->subs[i].multi_id == 0)
            return i;
    }
    return -1;
}

int ulog_parser_find_field(const ulog_parser_t *p, int format_idx, const char *name) {
    if (format_idx < 0 || format_idx >= p->format_count) return -1;
    const ulog_format_t *fmt = &p->formats[format_idx];
    for (int i = 0; i < fmt->field_count; i++) {
        if (strcmp(fmt->fields[i].name, name) == 0)
            return fmt->fields[i].offset;
    }
    return -1;
}

// Field accessors — use memcpy for safe unaligned reads

float ulog_parser_get_float(const ulog_data_msg_t *msg, int offset) {
    float v;
    memcpy(&v, msg->data + offset, sizeof(v));
    return v;
}

double ulog_parser_get_double(const ulog_data_msg_t *msg, int offset) {
    double v;
    memcpy(&v, msg->data + offset, sizeof(v));
    return v;
}

int32_t ulog_parser_get_int32(const ulog_data_msg_t *msg, int offset) {
    int32_t v;
    memcpy(&v, msg->data + offset, sizeof(v));
    return v;
}

uint8_t ulog_parser_get_uint8(const ulog_data_msg_t *msg, int offset) {
    return msg->data[offset];
}

uint64_t ulog_parser_get_uint64(const ulog_data_msg_t *msg, int offset) {
    uint64_t v;
    memcpy(&v, msg->data + offset, sizeof(v));
    return v;
}

void ulog_parser_close(ulog_parser_t *p) {
    if (p->fp) {
        fclose(p->fp);
        p->fp = NULL;
    }
    free(p->read_buf);
    p->read_buf = NULL;
    p->read_buf_size = 0;
    free(p->index);
    p->index = NULL;
    p->index_count = 0;
    p->index_capacity = 0;
}
