// Incremental Pearson correlation for 3-channel position data (x, y, z).
// Extracted from main.c for testability and reuse.

#ifndef CORRELATION_H
#define CORRELATION_H

#include <math.h>

#define CORR_CHANNELS   3
#define CORR_MIN_SAMPLES 30

typedef struct {
    double sum_x, sum_y, sum_xy, sum_x2, sum_y2;
} corr_channel_t;

typedef struct {
    corr_channel_t ch[CORR_CHANNELS];
    double sum_sq_dist;  // running sum of squared Euclidean distances
    int n;
} corr_state_t;

// Reset correlation state to zero.
static inline void corr_reset(corr_state_t *cs) {
    for (int c = 0; c < CORR_CHANNELS; c++) {
        cs->ch[c].sum_x  = 0.0;
        cs->ch[c].sum_y  = 0.0;
        cs->ch[c].sum_xy = 0.0;
        cs->ch[c].sum_x2 = 0.0;
        cs->ch[c].sum_y2 = 0.0;
    }
    cs->sum_sq_dist = 0.0;
    cs->n = 0;
}

// Add one paired sample (reference position a[3] vs tracked position b[3]).
static inline void corr_add_sample(corr_state_t *cs,
                                   const float a[3], const float b[3]) {
    double sq_dist = 0.0;
    for (int c = 0; c < CORR_CHANNELS; c++) {
        double x = a[c], y = b[c];
        cs->ch[c].sum_x  += x;
        cs->ch[c].sum_y  += y;
        cs->ch[c].sum_xy += x * y;
        cs->ch[c].sum_x2 += x * x;
        cs->ch[c].sum_y2 += y * y;
        double d = x - y;
        sq_dist += d * d;
    }
    cs->sum_sq_dist += sq_dist;
    cs->n++;
}

// Compute the averaged Pearson r across the 3 channels.
// Returns NAN if fewer than CORR_MIN_SAMPLES have been added.
static inline float corr_compute(const corr_state_t *cs) {
    if (cs->n < CORR_MIN_SAMPLES) return NAN;
    float r_sum = 0.0f;
    int valid = 0;
    double n = cs->n;
    for (int c = 0; c < CORR_CHANNELS; c++) {
        const corr_channel_t *ch = &cs->ch[c];
        double num = n * ch->sum_xy - ch->sum_x * ch->sum_y;
        double d1  = n * ch->sum_x2 - ch->sum_x * ch->sum_x;
        double d2  = n * ch->sum_y2 - ch->sum_y * ch->sum_y;
        double den = sqrt(d1 * d2);
        if (den > 1e-9) {
            r_sum += (float)(num / den);
            valid++;
        }
    }
    return (valid > 0) ? r_sum / valid : 0.0f;
}

// Compute RMSE from accumulated squared distances.
static inline float corr_rmse(const corr_state_t *cs) {
    if (cs->n == 0) return NAN;
    return (float)sqrt(cs->sum_sq_dist / cs->n);
}

#endif // CORRELATION_H
