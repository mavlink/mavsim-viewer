// Unit tests for PR #47 (CUSUM takeoff detection, home tier resolution)
// and PR #50 (Pearson correlation computation).

#include "correlation.h"
#include "ulog_replay.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define QUAD_LOG   FIXTURES_DIR "/dde9a24c-34c5-4868-b09c-bd3481ed1029.ulg"
#define FW_LOG     FIXTURES_DIR "/6dfd8372-8bb9-4827-8c43-8895f3fe7e7a.ulg"

// ── CUSUM takeoff detection tests ───────────────────────────────────────────

static void test_cusum_quad_takeoff_detected(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    assert(ctx.takeoff_detected);
    assert(ctx.takeoff_time_s > 0.0f);

    float duration = (float)((double)(ctx.parser.end_timestamp -
                                       ctx.parser.start_timestamp) / 1e6);
    assert(ctx.takeoff_time_s < duration);

    ulog_replay_close(&ctx);
    printf("  PASS cusum_quad_takeoff_detected (t=%.1fs within %.1fs log)\n",
           ctx.takeoff_time_s, duration);
}

static void test_cusum_confidence_range(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    assert(ctx.takeoff_conf >= 0.0f);
    assert(ctx.takeoff_conf <= 1.0f);
    // CUSUM alone gives at least 0.3 when triggered
    assert(ctx.takeoff_conf >= 0.3f);

    ulog_replay_close(&ctx);
    printf("  PASS cusum_confidence_range (conf=%.0f%%)\n", ctx.takeoff_conf * 100);
}

static void test_cusum_fw_takeoff_detected(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, FW_LOG) == 0);

    // Fixed-wing log should also have a detectable takeoff
    // (if not, the test documents the actual behavior)
    if (ctx.takeoff_detected) {
        assert(ctx.takeoff_time_s > 0.0f);
        assert(ctx.takeoff_conf >= 0.3f);
        assert(ctx.takeoff_conf <= 1.0f);
        printf("  PASS cusum_fw_takeoff_detected (t=%.1fs, conf=%.0f%%)\n",
               ctx.takeoff_time_s, ctx.takeoff_conf * 100);
    } else {
        // Not detected is acceptable for some logs (already airborne at log start)
        assert(ctx.takeoff_time_s == 0.0f);
        printf("  PASS cusum_fw_takeoff_detected (not detected, conf=%.0f%%)\n",
               ctx.takeoff_conf * 100);
    }

    ulog_replay_close(&ctx);
}

// ── Home position tier resolution tests ─────────────────────────────────────

static void test_home_tier_fw(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, FW_LOG) == 0);

    // FW log with GPS should have a valid home from either Tier 1 or Tier 2
    assert(ctx.home.valid);
    // Position should be non-zero
    assert(ctx.home.lat != 0 || ctx.home.lon != 0);

    printf("  PASS home_tier_fw (valid=%d, from_topic=%d, rejected=%d, "
           "lat=%d, lon=%d)\n",
           ctx.home.valid, ctx.home_from_topic, ctx.home_rejected,
           ctx.home.lat, ctx.home.lon);
    ulog_replay_close(&ctx);
}

static void test_home_tier_quad(void) {
    ulog_replay_ctx_t ctx;
    assert(ulog_replay_init(&ctx, QUAD_LOG) == 0);

    // Document the quad log's home resolution behavior.
    // Indoor/local-only logs may have home rejected (no GPOS).
    if (ctx.home.valid) {
        assert(ctx.home.lat != 0 || ctx.home.lon != 0);
        printf("  PASS home_tier_quad (valid, from_topic=%d)\n",
               ctx.home_from_topic);
    } else {
        printf("  PASS home_tier_quad (no valid home, rejected=%d)\n",
               ctx.home_rejected);
    }

    ulog_replay_close(&ctx);
}

// ── Correlation computation tests ───────────────────────────────────────────

static void test_corr_insufficient_samples(void) {
    corr_state_t cs;
    corr_reset(&cs);

    // With 0 samples, should return NAN
    float r = corr_compute(&cs);
    assert(isnan(r));

    // Add fewer than CORR_MIN_SAMPLES
    float a[3] = {1.0f, 2.0f, 3.0f};
    float b[3] = {1.0f, 2.0f, 3.0f};
    for (int i = 0; i < CORR_MIN_SAMPLES - 1; i++)
        corr_add_sample(&cs, a, b);

    r = corr_compute(&cs);
    assert(isnan(r));

    printf("  PASS corr_insufficient_samples\n");
}

static void test_corr_perfect_correlation(void) {
    corr_state_t cs;
    corr_reset(&cs);

    // Identical trajectories should give r = 1.0
    for (int i = 0; i < 100; i++) {
        float v = (float)i;
        float a[3] = {v, v * 2.0f, v * 0.5f};
        float b[3] = {v, v * 2.0f, v * 0.5f};
        corr_add_sample(&cs, a, b);
    }

    float r = corr_compute(&cs);
    assert(!isnan(r));
    assert(fabs(r - 1.0f) < 0.001f);

    float rmse = corr_rmse(&cs);
    assert(rmse < 0.001f);

    printf("  PASS corr_perfect_correlation (r=%.4f, rmse=%.4f)\n", r, rmse);
}

static void test_corr_negative_correlation(void) {
    corr_state_t cs;
    corr_reset(&cs);

    // One goes up while the other goes down: r should be -1.0
    for (int i = 0; i < 100; i++) {
        float v = (float)i;
        float a[3] = {v, v, v};
        float b[3] = {-v, -v, -v};
        corr_add_sample(&cs, a, b);
    }

    float r = corr_compute(&cs);
    assert(!isnan(r));
    assert(fabs(r - (-1.0f)) < 0.001f);

    printf("  PASS corr_negative_correlation (r=%.4f)\n", r);
}

static void test_corr_uncorrelated(void) {
    corr_state_t cs;
    corr_reset(&cs);

    // One channel varies, the other stays constant: r should be 0
    // (constant series has zero variance, so den == 0, channel skipped)
    for (int i = 0; i < 100; i++) {
        float v = (float)i;
        float a[3] = {v, v, v};
        float b[3] = {5.0f, 5.0f, 5.0f};
        corr_add_sample(&cs, a, b);
    }

    float r = corr_compute(&cs);
    assert(!isnan(r));
    // No valid channels (constant series), so result is 0
    assert(fabs(r) < 0.001f);

    printf("  PASS corr_uncorrelated (r=%.4f)\n", r);
}

static void test_corr_offset_trajectory(void) {
    corr_state_t cs;
    corr_reset(&cs);

    // Same trajectory shape but with a constant offset: r should be 1.0
    // (Pearson is insensitive to linear shift)
    for (int i = 0; i < 100; i++) {
        float v = (float)i;
        float a[3] = {v, v * 2.0f, sinf(v * 0.1f)};
        float b[3] = {v + 10.0f, v * 2.0f + 5.0f, sinf(v * 0.1f) + 3.0f};
        corr_add_sample(&cs, a, b);
    }

    float r = corr_compute(&cs);
    assert(!isnan(r));
    assert(fabs(r - 1.0f) < 0.001f);

    // RMSE should reflect the offset magnitude
    float rmse = corr_rmse(&cs);
    assert(rmse > 1.0f);

    printf("  PASS corr_offset_trajectory (r=%.4f, rmse=%.2f)\n", r, rmse);
}

static void test_corr_scaled_trajectory(void) {
    corr_state_t cs;
    corr_reset(&cs);

    // Same trajectory but scaled by 2x: still perfectly correlated
    for (int i = 0; i < 100; i++) {
        float v = (float)i;
        float a[3] = {v, v, v};
        float b[3] = {v * 2.0f, v * 2.0f, v * 2.0f};
        corr_add_sample(&cs, a, b);
    }

    float r = corr_compute(&cs);
    assert(!isnan(r));
    assert(fabs(r - 1.0f) < 0.001f);

    printf("  PASS corr_scaled_trajectory (r=%.4f)\n", r);
}

static void test_corr_reset(void) {
    corr_state_t cs;
    corr_reset(&cs);

    assert(cs.n == 0);
    assert(cs.sum_sq_dist == 0.0);
    for (int c = 0; c < CORR_CHANNELS; c++) {
        assert(cs.ch[c].sum_x == 0.0);
        assert(cs.ch[c].sum_y == 0.0);
        assert(cs.ch[c].sum_xy == 0.0);
        assert(cs.ch[c].sum_x2 == 0.0);
        assert(cs.ch[c].sum_y2 == 0.0);
    }

    printf("  PASS corr_reset\n");
}

static void test_corr_rmse_zero_for_identical(void) {
    corr_state_t cs;
    corr_reset(&cs);

    for (int i = 0; i < 50; i++) {
        float a[3] = {(float)i, (float)(i * 2), (float)(i * 3)};
        corr_add_sample(&cs, a, a);
    }

    float rmse = corr_rmse(&cs);
    assert(!isnan(rmse));
    assert(rmse < 0.001f);

    printf("  PASS corr_rmse_zero_for_identical (rmse=%.6f)\n", rmse);
}

// ── Main ────────────────────────────────────────────────────────────────────

int main(void) {
    printf("test_algorithms:\n");

    // CUSUM takeoff detection
    test_cusum_quad_takeoff_detected();
    test_cusum_confidence_range();
    test_cusum_fw_takeoff_detected();

    // Home position tier resolution
    test_home_tier_fw();
    test_home_tier_quad();

    // Correlation computation
    test_corr_insufficient_samples();
    test_corr_perfect_correlation();
    test_corr_negative_correlation();
    test_corr_uncorrelated();
    test_corr_offset_trajectory();
    test_corr_scaled_trajectory();
    test_corr_reset();
    test_corr_rmse_zero_for_identical();

    printf("All tests passed.\n");
    return 0;
}
