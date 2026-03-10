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

int main(void) {
    printf("test_data_source:\n");
    test_create();
    test_defaults();
    test_duration();
    test_progress_advances();
    test_state_propagated();
    test_mav_type_propagated();
    test_close();
    printf("All tests passed.\n");
    return 0;
}
