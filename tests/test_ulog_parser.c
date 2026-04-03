#include "ulog_parser.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define QUAD_LOG   FIXTURES_DIR "/dde9a24c-34c5-4868-b09c-bd3481ed1029.ulg"
#define FW_LOG     FIXTURES_DIR "/6dfd8372-8bb9-4827-8c43-8895f3fe7e7a.ulg"
#define TRUNC_LOG  FIXTURES_DIR "/c83373e1-ee62-4e7e-850b-69316e8641a9.ulg"

static void test_open_valid(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    int ret = ulog_parser_open(p, QUAD_LOG);
    assert(ret == 0);
    ulog_parser_close(p);
    free(p);
    printf("  PASS open_valid\n");
}

static void test_open_invalid(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    int ret = ulog_parser_open(p, "/nonexistent/file.ulg");
    assert(ret != 0);
    free(p);
    printf("  PASS open_invalid\n");
}

static void test_format_count(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    assert(p->format_count > 0);
    int fc = p->format_count;
    ulog_parser_close(p);
    free(p);
    printf("  PASS format_count (%d formats)\n", fc);
}

static void test_sub_count(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    assert(p->sub_count > 0);
    int sc = p->sub_count;
    ulog_parser_close(p);
    free(p);
    printf("  PASS sub_count (%d subscriptions)\n", sc);
}

static void test_find_subscription(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    int idx = ulog_parser_find_subscription(p, "vehicle_attitude");
    assert(idx >= 0);
    assert(strcmp(p->subs[idx].message_name, "vehicle_attitude") == 0);
    ulog_parser_close(p);
    free(p);
    printf("  PASS find_subscription\n");
}

static void test_find_subscription_missing(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    int idx = ulog_parser_find_subscription(p, "nonexistent_topic");
    assert(idx == -1);
    ulog_parser_close(p);
    free(p);
    printf("  PASS find_subscription_missing\n");
}

static void test_find_field(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    int sub_idx = ulog_parser_find_subscription(p, "vehicle_attitude");
    assert(sub_idx >= 0);
    int fmt_idx = p->subs[sub_idx].format_idx;
    assert(fmt_idx >= 0);
    int offset = ulog_parser_find_field(p, fmt_idx, "q");
    assert(offset >= 0);
    ulog_parser_close(p);
    free(p);
    printf("  PASS find_field (q offset=%d)\n", offset);
}

static void test_find_field_missing(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    int sub_idx = ulog_parser_find_subscription(p, "vehicle_attitude");
    int fmt_idx = p->subs[sub_idx].format_idx;
    int offset = ulog_parser_find_field(p, fmt_idx, "nonexistent_field");
    assert(offset == -1);
    ulog_parser_close(p);
    free(p);
    printf("  PASS find_field_missing\n");
}

static void test_timestamp_range(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    assert(p->start_timestamp > 0);
    assert(p->end_timestamp > 0);
    assert(p->end_timestamp > p->start_timestamp);
    ulog_parser_close(p);
    free(p);
    printf("  PASS timestamp_range\n");
}

static void test_index_built(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    assert(p->index_count > 0);
    // Verify monotonically increasing timestamps
    for (int i = 1; i < p->index_count; i++) {
        assert(p->index[i].timestamp >= p->index[i - 1].timestamp);
    }
    int ic = p->index_count;
    ulog_parser_close(p);
    free(p);
    printf("  PASS index_built (%d entries)\n", ic);
}

static void test_iterate_first(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);
    ulog_data_msg_t msg;
    bool ok = ulog_parser_next(p, &msg);
    assert(ok);
    assert(msg.timestamp > 0);
    assert(msg.data != NULL);
    assert(msg.data_len > 0);
    ulog_parser_close(p);
    free(p);
    printf("  PASS iterate_first\n");
}

static void test_rewind(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);

    // Read first message
    ulog_data_msg_t first;
    assert(ulog_parser_next(p, &first));
    uint64_t first_ts = first.timestamp;

    // Read a few more
    ulog_data_msg_t msg;
    for (int i = 0; i < 100; i++) ulog_parser_next(p, &msg);

    // Rewind and verify first message matches
    ulog_parser_rewind(p);
    assert(ulog_parser_next(p, &msg));
    assert(msg.timestamp == first_ts);

    ulog_parser_close(p);
    free(p);
    printf("  PASS rewind\n");
}

static void test_seek_midpoint(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);

    uint64_t mid = (p->start_timestamp + p->end_timestamp) / 2;
    ulog_parser_seek(p, mid);

    ulog_data_msg_t msg;
    assert(ulog_parser_next(p, &msg));
    // Should be within ~2 seconds of the target
    int64_t diff = (int64_t)(msg.timestamp - mid);
    if (diff < 0) diff = -diff;
    assert(diff < 2000000); // 2 seconds in microseconds

    ulog_parser_close(p);
    free(p);
    printf("  PASS seek_midpoint\n");
}

static void test_truncated_file(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    int ret = ulog_parser_open(p, TRUNC_LOG);
    assert(ret == 0);

    // Iterate through all messages — should hit EOF gracefully
    ulog_data_msg_t msg;
    int count = 0;
    while (ulog_parser_next(p, &msg)) count++;
    assert(count > 0);
    assert(p->eof);

    ulog_parser_close(p);
    free(p);
    printf("  PASS truncated_file (%d messages)\n", count);
}

static void test_field_accessors(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, QUAD_LOG) == 0);

    int sub_idx = ulog_parser_find_subscription(p, "vehicle_attitude");
    int fmt_idx = p->subs[sub_idx].format_idx;
    int q_offset = ulog_parser_find_field(p, fmt_idx, "q");
    uint16_t att_msg_id = p->subs[sub_idx].msg_id;

    // Find first attitude message
    ulog_data_msg_t msg;
    bool found = false;
    while (ulog_parser_next(p, &msg)) {
        if (msg.msg_id == att_msg_id) {
            found = true;
            break;
        }
    }
    assert(found);

    // Read quaternion — should be normalized (w²+x²+y²+z² ≈ 1.0)
    float w = ulog_parser_get_float(&msg, q_offset + 0);
    float x = ulog_parser_get_float(&msg, q_offset + 4);
    float y = ulog_parser_get_float(&msg, q_offset + 8);
    float z = ulog_parser_get_float(&msg, q_offset + 12);
    float norm = w * w + x * x + y * y + z * z;
    assert(!isnan(norm));
    assert(fabs(norm - 1.0f) < 0.01f);

    ulog_parser_close(p);
    free(p);
    printf("  PASS field_accessors (q norm=%.4f)\n", norm);
}

static void test_fixedwing_subscriptions(void) {
    ulog_parser_t *p = malloc(sizeof(*p));
    assert(p);
    assert(ulog_parser_open(p, FW_LOG) == 0);
    int idx = ulog_parser_find_subscription(p, "airspeed_validated");
    assert(idx >= 0);
    ulog_parser_close(p);
    free(p);
    printf("  PASS fixedwing_subscriptions\n");
}

int main(void) {
    printf("test_ulog_parser:\n");
    test_open_valid();
    test_open_invalid();
    test_format_count();
    test_sub_count();
    test_find_subscription();
    test_find_subscription_missing();
    test_find_field();
    test_find_field_missing();
    test_timestamp_range();
    test_index_built();
    test_iterate_first();
    test_rewind();
    test_seek_midpoint();
    test_truncated_file();
    test_field_accessors();
    test_fixedwing_subscriptions();
    printf("All tests passed.\n");
    return 0;
}
