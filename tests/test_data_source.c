#include "data_source.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define QUAD_LOG  FIXTURES_DIR "/dde9a24c-34c5-4868-b09c-bd3481ed1029.ulg"
#define FW_LOG    FIXTURES_DIR "/6dfd8372-8bb9-4827-8c43-8895f3fe7e7a.ulg"

static void test_create(void) {
    data_source_t ds;
    int ret = data_source_ulog_create(&ds, QUAD_LOG);
    assert(ret == 0);
    assert(ds.connected);
    assert(ds.impl != NULL);
    data_source_close(&ds);
    printf("  PASS create\n");
}

static void test_defaults(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);
    assert(ds.playback.speed == 1.0f);
    assert(!ds.playback.paused);
    assert(!ds.playback.looping);
    assert(ds.playback.interpolation);
    data_source_close(&ds);
    printf("  PASS defaults\n");
}

static void test_duration(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);
    data_source_poll(&ds, 0.1f);
    assert(ds.playback.duration_s > 0.0f);
    data_source_close(&ds);
    printf("  PASS duration (%.1fs)\n", ds.playback.duration_s);
}

static void test_progress_advances(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);

    data_source_poll(&ds, 0.1f);
    float p1 = ds.playback.progress;
    data_source_poll(&ds, 0.5f);
    float p2 = ds.playback.progress;
    assert(p2 > p1);

    data_source_close(&ds);
    printf("  PASS progress_advances (%.4f -> %.4f)\n", p1, p2);
}

static void test_state_propagated(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);

    for (int i = 0; i < 500; i++) {
        data_source_poll(&ds, 0.05f);
        if (ds.state.valid) break;
    }
    assert(ds.state.valid);

    data_source_close(&ds);
    printf("  PASS state_propagated\n");
}

static void test_mav_type_propagated(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, FW_LOG) == 0);
    data_source_poll(&ds, 0.1f);
    assert(ds.mav_type == 1); // MAV_TYPE_FIXED_WING
    data_source_close(&ds);
    printf("  PASS mav_type_propagated\n");
}

static void test_close(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);
    data_source_poll(&ds, 0.1f);
    data_source_close(&ds);
    assert(ds.impl == NULL);
    printf("  PASS close\n");
}

static void test_seek(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);

    // Advance playback several seconds
    for (int i = 0; i < 100; i++)
        data_source_poll(&ds, 0.05f);

    float pos_before = ds.playback.position_s;
    assert(pos_before > 1.0f);

    // Seek back to start
    data_source_seek(&ds, 0.0f);
    float pos_after = ds.playback.position_s;
    assert(pos_after < 1.0f);

    data_source_close(&ds);
    printf("  PASS seek (%.2fs -> %.2fs)\n", pos_before, pos_after);
}

static void test_set_time_offset(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);

    assert(ds.playback.time_offset_s == 0.0f);

    data_source_set_time_offset(&ds, 3.5);
    assert(fabsf(ds.playback.time_offset_s - 3.5f) < 0.01f);

    data_source_set_time_offset(&ds, -1.25);
    assert(fabsf(ds.playback.time_offset_s - (-1.25f)) < 0.01f);

    data_source_close(&ds);
    printf("  PASS set_time_offset\n");
}

static void test_playback_state_fields(void) {
    data_source_t ds;
    assert(data_source_ulog_create(&ds, QUAD_LOG) == 0);

    // Poll enough to parse log metadata and detect takeoff
    for (int i = 0; i < 500; i++)
        data_source_poll(&ds, 0.05f);

    // Duration must be positive after polling
    assert(ds.playback.duration_s > 0.0f);

    // Takeoff detection fields: takeoff_detected is a bool, just verify it's accessible.
    // If detected, takeoff_time_s should be within the log duration.
    if (ds.playback.takeoff_detected) {
        assert(ds.playback.takeoff_time_s > 0.0f);
        assert(ds.playback.takeoff_time_s <= ds.playback.duration_s);
    }

    // home_from_topic should be a valid bool (either true or false, both are ok)
    // Just verify the field is readable and has a deterministic value
    bool home_set = ds.playback.home_from_topic;
    (void)home_set; // suppress unused warning

    data_source_close(&ds);
    printf("  PASS playback_state_fields (duration=%.1fs takeoff_detected=%d takeoff_time=%.1fs home_from_topic=%d)\n",
           ds.playback.duration_s,
           ds.playback.takeoff_detected,
           ds.playback.takeoff_time_s,
           ds.playback.home_from_topic);
}

static void test_multi_file_independence(void) {
    data_source_t ds1, ds2;
    assert(data_source_ulog_create(&ds1, QUAD_LOG) == 0);
    assert(data_source_ulog_create(&ds2, FW_LOG) == 0);

    // Advance both sources
    for (int i = 0; i < 100; i++) {
        data_source_poll(&ds1, 0.05f);
        data_source_poll(&ds2, 0.05f);
    }

    float pos1_before = ds1.playback.position_s;
    float pos2_before = ds2.playback.position_s;
    assert(pos1_before > 1.0f);
    assert(pos2_before > 1.0f);

    // Seek only ds1 back to start; ds2 should be unaffected
    data_source_seek(&ds1, 0.0f);
    assert(ds1.playback.position_s < 1.0f);
    assert(ds2.playback.position_s >= pos2_before - 0.01f);

    // Set time offset on ds2 only; ds1 should be unaffected
    data_source_set_time_offset(&ds2, 7.0);
    assert(fabsf(ds2.playback.time_offset_s - 7.0f) < 0.01f);
    assert(fabsf(ds1.playback.time_offset_s) < 0.01f);

    // Continue polling ds2 — ds1 state should remain at seek position
    float ds1_pos_after_seek = ds1.playback.position_s;
    for (int i = 0; i < 20; i++)
        data_source_poll(&ds2, 0.05f);
    assert(fabsf(ds1.playback.position_s - ds1_pos_after_seek) < 0.01f);

    data_source_close(&ds1);
    data_source_close(&ds2);
    printf("  PASS multi_file_independence\n");
}

int main(void) {
    printf("test_data_source:\n");
    test_create();
    test_defaults();
    test_duration();
    test_progress_advances();
    test_state_propagated();
    test_mav_type_propagated();
    test_close();
    test_seek();
    test_set_time_offset();
    test_playback_state_fields();
    test_multi_file_independence();
    printf("All tests passed.\n");
    return 0;
}
