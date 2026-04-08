#include "hud_annunciators.h"
#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

void annunc_init(hud_annunciators_t *a) {
    memset(a, 0, sizeof(*a));
    a->tab_fade.duration = 1.0f;
    a->ring_bounce.duration = 0.8f;
    a->radar_wave.duration = 1.2f;
    a->ticker_flash.duration = 0.3f;
    a->ring_shake.duration = 0.4f;
    a->ring_shake.cycles = 3;
    a->ring_shake.amplitude = 2.0f;
}

void annunc_update(hud_annunciators_t *a, float dt) {
    // Tab fade (per-drone)
    for (int i = 0; i < ANNUNC_MAX_VEHICLES; i++) {
        if (a->tab_fade.timer[i] > 0.0f) {
            a->tab_fade.timer[i] -= dt;
            if (a->tab_fade.timer[i] < 0.0f) a->tab_fade.timer[i] = 0.0f;
        }
    }
    // Ring bounce
    for (int i = 0; i < ANNUNC_MAX_VEHICLES; i++) {
        if (a->ring_bounce.timer[i] > 0.0f) {
            a->ring_bounce.timer[i] -= dt;
            if (a->ring_bounce.timer[i] < 0.0f) a->ring_bounce.timer[i] = 0.0f;
        }
    }
    // Radar wave
    for (int i = 0; i < ANNUNC_MAX_VEHICLES; i++) {
        if (a->radar_wave.timer[i] > 0.0f) {
            a->radar_wave.timer[i] -= dt;
            if (a->radar_wave.timer[i] < 0.0f) a->radar_wave.timer[i] = 0.0f;
        }
    }
    // Ticker flash
    for (int i = 0; i < 4; i++) {
        if (a->ticker_flash.timer[i] > 0.0f) {
            a->ticker_flash.timer[i] -= dt;
            if (a->ticker_flash.timer[i] < 0.0f) a->ticker_flash.timer[i] = 0.0f;
        }
    }
    // Ring shake
    for (int i = 0; i < ANNUNC_MAX_VEHICLES; i++) {
        if (a->ring_shake.timer[i] > 0.0f) {
            a->ring_shake.timer[i] -= dt;
            if (a->ring_shake.timer[i] < 0.0f) a->ring_shake.timer[i] = 0.0f;
        }
    }
}

// ── Triggers ────────────────────────────────────────────────────────────────

void annunc_trigger_tab_fade(hud_annunciators_t *a, int drone_idx) {
    if (drone_idx >= 0 && drone_idx < ANNUNC_MAX_VEHICLES)
        a->tab_fade.timer[drone_idx] = a->tab_fade.duration;
}

void annunc_trigger_ring_bounce(hud_annunciators_t *a, int drone_idx) {
    if (drone_idx >= 0 && drone_idx < ANNUNC_MAX_VEHICLES)
        a->ring_bounce.timer[drone_idx] = a->ring_bounce.duration;
}

void annunc_trigger_radar_wave(hud_annunciators_t *a, int drone_idx) {
    if (drone_idx >= 0 && drone_idx < ANNUNC_MAX_VEHICLES)
        a->radar_wave.timer[drone_idx] = a->radar_wave.duration;
}

void annunc_trigger_ticker_flash(hud_annunciators_t *a, int slot) {
    if (slot >= 0 && slot < 4)
        a->ticker_flash.timer[slot] = a->ticker_flash.duration;
}

void annunc_trigger_ring_shake(hud_annunciators_t *a, int drone_idx) {
    if (drone_idx >= 0 && drone_idx < ANNUNC_MAX_VEHICLES)
        a->ring_shake.timer[drone_idx] = a->ring_shake.duration;
}

// ── Queries ─────────────────────────────────────────────────────────────────

// Tab fade: returns alpha multiplier (1.0 = normal, 0.0 = fully faded out)
// Animation: fade out then back in (V-shaped)
float annunc_tab_fade_alpha(const hud_annunciators_t *a, int drone_idx) {
    if (drone_idx < 0 || drone_idx >= ANNUNC_MAX_VEHICLES) return 1.0f;
    if (a->tab_fade.timer[drone_idx] <= 0.0f) return 1.0f;
    float progress = 1.0f - (a->tab_fade.timer[drone_idx] / a->tab_fade.duration);
    // Double pulse: fade out, in, out, in
    float cycle = progress * 2.0f;  // two full cycles in duration
    if (cycle > 1.0f) cycle -= 1.0f;
    // Each cycle: out then in (V-shape)
    if (cycle < 0.4f)
        return 1.0f - (cycle / 0.4f);
    else
        return (cycle - 0.4f) / 0.6f;
}

// Ring bounce: returns Y offset in pixels (before scaling)
float annunc_ring_bounce_offset(const hud_annunciators_t *a, int drone_idx) {
    if (drone_idx < 0 || drone_idx >= ANNUNC_MAX_VEHICLES) return 0.0f;
    float t = a->ring_bounce.timer[drone_idx];
    if (t <= 0.0f) return 0.0f;
    float progress = 1.0f - (t / a->ring_bounce.duration);
    // Two up-down cycles, second one smaller
    float cycle = progress * 2.0f;
    if (cycle <= 1.0f)
        return sinf(cycle * PI) * -8.0f;
    else
        return sinf((cycle - 1.0f) * PI) * -4.0f;
}

// Peak scale: update max tracking and at_peak flag
// Radar wave: returns progress 0.0-1.0 (0 = idle)
float annunc_radar_wave_progress(const hud_annunciators_t *a, int drone_idx) {
    if (drone_idx < 0 || drone_idx >= ANNUNC_MAX_VEHICLES) return 0.0f;
    float t = a->radar_wave.timer[drone_idx];
    if (t <= 0.0f) return 0.0f;
    return 1.0f - (t / a->radar_wave.duration);
}

// Ticker flash: returns alpha 0.0-1.0 for background fill
float annunc_ticker_flash_alpha(const hud_annunciators_t *a, int slot) {
    if (slot < 0 || slot >= 4) return 0.0f;
    float t = a->ticker_flash.timer[slot];
    if (t <= 0.0f) return 0.0f;
    return t / a->ticker_flash.duration;
}

// Ring shake: returns X offset in pixels (before scaling)
float annunc_ring_shake_offset(const hud_annunciators_t *a, int drone_idx) {
    if (drone_idx < 0 || drone_idx >= ANNUNC_MAX_VEHICLES) return 0.0f;
    float t = a->ring_shake.timer[drone_idx];
    if (t <= 0.0f) return 0.0f;
    float progress = 1.0f - (t / a->ring_shake.duration);
    float amp = a->ring_shake.amplitude;
    int cycles = a->ring_shake.cycles;
    // Damped oscillation
    return sinf(progress * cycles * 2.0f * PI) * amp * (1.0f - progress);
}
