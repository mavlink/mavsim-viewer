#ifndef ULOG_PARSER_H
#define ULOG_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define ULOG_MAX_SUBSCRIPTIONS 256
#define ULOG_MAX_FIELDS        64

// ULog message types
#define ULOG_MSG_FLAG_BITS    'B'
#define ULOG_MSG_FORMAT       'F'
#define ULOG_MSG_INFO         'I'
#define ULOG_MSG_PARAMETER    'P'
#define ULOG_MSG_ADD_LOGGED   'A'
#define ULOG_MSG_DROPOUT      'O'
#define ULOG_MSG_DATA         'D'
#define ULOG_MSG_LOGGING      'L'
#define ULOG_MSG_SYNC         'S'
#define ULOG_MSG_INFO_MULTI   'M'
#define ULOG_MSG_PARAM_DEF    'Q'

typedef enum {
    ULOG_FIELD_UINT8,
    ULOG_FIELD_INT8,
    ULOG_FIELD_UINT16,
    ULOG_FIELD_INT16,
    ULOG_FIELD_UINT32,
    ULOG_FIELD_INT32,
    ULOG_FIELD_UINT64,
    ULOG_FIELD_INT64,
    ULOG_FIELD_FLOAT,
    ULOG_FIELD_DOUBLE,
    ULOG_FIELD_BOOL,
    ULOG_FIELD_CHAR,
    ULOG_FIELD_UNKNOWN,
} ulog_field_type_t;

typedef struct {
    char name[64];
    ulog_field_type_t type;
    int array_size;       // 1 for scalar, >1 for arrays (e.g. q[4] -> 4)
    int offset;           // byte offset within data payload (after msg_id)
} ulog_field_t;

typedef struct {
    char name[64];
    ulog_field_t fields[ULOG_MAX_FIELDS];
    int field_count;
    int size;             // total payload bytes
} ulog_format_t;

typedef struct {
    uint16_t msg_id;
    uint8_t multi_id;
    char message_name[64];
    int format_idx;       // index into formats[]
} ulog_subscription_t;

typedef struct {
    uint16_t msg_id;
    uint64_t timestamp;
    const uint8_t *data;  // payload after msg_id, valid until next read
    int data_len;
} ulog_data_msg_t;

// Timestamp index entry for seeking
typedef struct {
    uint64_t timestamp;
    long file_offset;     // fseek offset to the message header
} ulog_index_entry_t;

typedef struct {
    FILE *fp;
    long data_start_offset;

    ulog_format_t formats[ULOG_MAX_SUBSCRIPTIONS];
    int format_count;

    ulog_subscription_t subs[ULOG_MAX_SUBSCRIPTIONS];
    int sub_count;

    uint8_t *read_buf;
    int read_buf_size;

    // Timestamp index for seeking (~1 entry per second of log)
    ulog_index_entry_t *index;
    int index_count;
    int index_capacity;

    uint64_t start_timestamp;
    uint64_t end_timestamp;

    bool eof;
} ulog_parser_t;

// Open and parse ULog header + definitions, build seek index. Returns 0 on success.
int ulog_parser_open(ulog_parser_t *p, const char *filepath);

// Read next data message. Returns true if a message was read.
bool ulog_parser_next(ulog_parser_t *p, ulog_data_msg_t *out);

// Seek to the beginning of the data section.
void ulog_parser_rewind(ulog_parser_t *p);

// Seek to nearest index entry at or before target timestamp.
int ulog_parser_seek(ulog_parser_t *p, uint64_t target_usec);

// Find subscription index by topic name. Returns -1 if not found.
int ulog_parser_find_subscription(const ulog_parser_t *p, const char *name);

// Find field byte offset within a format. Returns -1 if not found.
int ulog_parser_find_field(const ulog_parser_t *p, int format_idx, const char *name);

// Type-safe field accessors (read from data at given byte offset)
float    ulog_parser_get_float(const ulog_data_msg_t *msg, int offset);
double   ulog_parser_get_double(const ulog_data_msg_t *msg, int offset);
int32_t  ulog_parser_get_int32(const ulog_data_msg_t *msg, int offset);
uint8_t  ulog_parser_get_uint8(const ulog_data_msg_t *msg, int offset);
uint64_t ulog_parser_get_uint64(const ulog_data_msg_t *msg, int offset);

// Close file and free resources.
void ulog_parser_close(ulog_parser_t *p);

#endif
