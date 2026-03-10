#include "ulog_replay.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define QUAD_LOG   FIXTURES_DIR "/dde9a24c-34c5-4868-b09c-bd3481ed1029.ulg"
#define FW_LOG     FIXTURES_DIR "/6dfd8372-8bb9-4827-8c43-8895f3fe7e7a.ulg"

static void test_init_quadrotor(void) {
    ulog_replay_ctx_t ctx;
    int ret = ulog_replay_init(&ctx, QUAD_LOG);
    assert(ret == 0);
    assert(ctx.vehicle_type == 2); // MAV_TYPE_QUADROTOR
    ulog_replay_close(&ctx);
    printf("  PASS init_quadrotor\n");
}

static void test_init_fixedwing(void) {
    ulog_replay_ctx_t ctx;
    int ret = ulog_replay_init(&ctx, FW_LOG);
    assert(ret == 0);
    assert(ctx.vehicle_type == 1); // MAV_TYPE_FIXED_WING
    ulog_replay_close(&ctx);
    printf("  PASS init_fixedwing\n");
}

static void test_state_valid_after_advance(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    // Advance until state becomes valid (may take a few iterations
    // for position data to arrive)
    for (int i = 0; i < 500; i++) {
        ulog_replay_advance(&ctx, 0.05f, 1.0f, false, true);
        if (ctx.state.valid) break;
    }
    assert(ctx.state.valid);

    ulog_replay_close(&ctx);
    printf("  PASS state_valid_after_advance\n");
}

static void test_quaternion_normalized(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    for (int i = 0; i < 500; i++) {
        ulog_replay_advance(&ctx, 0.05f, 1.0f, false, true);
        if (ctx.state.valid) break;
    }
    assert(ctx.state.valid);

    float w = ctx.state.quaternion[0];
    float x = ctx.state.quaternion[1];
    float y = ctx.state.quaternion[2];
    float z = ctx.state.quaternion[3];
    float norm = w * w + x * x + y * y + z * z;
    assert(!isnan(norm));
    assert(fabs(norm - 1.0f) < 0.01f);

    ulog_replay_close(&ctx);
    printf("  PASS quaternion_normalized (norm=%.4f)\n", norm);
}

static void test_gps_path(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, FW_LOG) == 0);

    for (int i = 0; i < 1000; i++) {
        ulog_replay_advance(&ctx, 0.05f, 1.0f, false, true);
        if (ctx.has_global_pos && ctx.state.valid) break;
    }
    assert(ctx.has_global_pos);
    assert(ctx.state.lat != 0 || ctx.state.lon != 0);
    assert(ctx.state.alt != 0);

    ulog_replay_close(&ctx);
    printf("  PASS gps_path (lat=%d lon=%d alt=%d)\n",
           ctx.state.lat, ctx.state.lon, ctx.state.alt);
}

static void test_local_pos_path(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    for (int i = 0; i < 500; i++) {
        ulog_replay_advance(&ctx, 0.05f, 1.0f, false, true);
        if (ctx.state.valid) break;
    }
    assert(ctx.state.valid);
    // Quad log uses local position, not GPS
    assert(!ctx.has_global_pos);

    ulog_replay_close(&ctx);
    printf("  PASS local_pos_path\n");
}

static void test_home_valid(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, FW_LOG) == 0);

    for (int i = 0; i < 1000; i++) {
        ulog_replay_advance(&ctx, 0.05f, 1.0f, false, true);
        if (ctx.home.valid) break;
    }
    assert(ctx.home.valid);

    ulog_replay_close(&ctx);
    printf("  PASS home_valid\n");
}

static void test_seek_and_advance(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    float duration = (float)((double)(ctx.parser.end_timestamp -
                                       ctx.parser.start_timestamp) / 1e6);
    float target = duration * 0.5f;
    ulog_replay_seek(&ctx, target);

    // wall_accum should be near the seek target
    assert(fabs(ctx.wall_accum - target) < 2.0);

    ulog_replay_close(&ctx);
    printf("  PASS seek_and_advance (target=%.1f, accum=%.1f)\n",
           target, (float)ctx.wall_accum);
}

static void test_loop_resets(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    float duration = (float)((double)(ctx.parser.end_timestamp -
                                       ctx.parser.start_timestamp) / 1e6);

    // Seek near the end and advance past it with looping
    ulog_replay_seek(&ctx, duration - 0.5f);
    bool playing = ulog_replay_advance(&ctx, 2.0f, 1.0f, true, true);
    assert(playing);
    assert(ctx.wall_accum < 2.0); // should have looped back near 0

    ulog_replay_close(&ctx);
    printf("  PASS loop_resets (accum=%.2f)\n", (float)ctx.wall_accum);
}

static void test_end_returns_false(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    float duration = (float)((double)(ctx.parser.end_timestamp -
                                       ctx.parser.start_timestamp) / 1e6);

    ulog_replay_seek(&ctx, duration - 0.1f);
    bool playing = true;
    for (int i = 0; i < 100 && playing; i++) {
        playing = ulog_replay_advance(&ctx, 0.1f, 1.0f, false, true);
    }
    assert(!playing);

    ulog_replay_close(&ctx);
    printf("  PASS end_returns_false\n");
}

static void test_airspeed(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, FW_LOG) == 0);

    bool found_airspeed = false;
    for (int i = 0; i < 5000; i++) {
        ulog_replay_advance(&ctx, 0.05f, 1.0f, false, true);
        if (ctx.state.ind_airspeed > 0) {
            found_airspeed = true;
            break;
        }
    }
    assert(found_airspeed);

    ulog_replay_close(&ctx);
    printf("  PASS airspeed (ias=%u cm/s)\n", ctx.state.ind_airspeed);
}

static void test_velocity(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, FW_LOG) == 0);

    for (int i = 0; i < 1000; i++) {
        ulog_replay_advance(&ctx, 0.05f, 1.0f, false, true);
        if (ctx.state.valid) break;
    }
    assert(ctx.state.valid);

    // Velocity should be within plausible range for an aircraft
    float vx = ctx.state.vx / 100.0f;
    float vy = ctx.state.vy / 100.0f;
    float vz = ctx.state.vz / 100.0f;
    float speed = sqrtf(vx * vx + vy * vy + vz * vz);
    assert(speed < 200.0f); // less than 200 m/s

    ulog_replay_close(&ctx);
    printf("  PASS velocity (speed=%.1f m/s)\n", speed);
}

int main(void) {
    printf("test_ulog_replay:\n");
    test_init_quadrotor();
    test_init_fixedwing();
    test_state_valid_after_advance();
    test_quaternion_normalized();
    test_gps_path();
    test_local_pos_path();
    test_home_valid();
    test_seek_and_advance();
    test_loop_resets();
    test_end_returns_false();
    test_airspeed();
    test_velocity();
    printf("All tests passed.\n");
    return 0;
}
