#ifndef HUD_ANNUNCIATORS_H
#define HUD_ANNUNCIATORS_H

#include <stdbool.h>

#define ANNUNC_MAX_VEHICLES 16

// (a) Console tab fade — per-drone color bar fades out and back in
typedef struct {
    float timer[ANNUNC_MAX_VEHICLES];
    float duration;
} annunc_tab_fade_t;

// (b) Gimbal ring bounce — pinned drone cell bounces on marker
typedef struct {
    float timer[ANNUNC_MAX_VEHICLES];
    float duration;
} annunc_ring_bounce_t;

// (c) Radar droplet wave — two expanding rings from drone blip
typedef struct {
    float timer[ANNUNC_MAX_VEHICLES];
    float duration;
} annunc_radar_wave_t;

// (e) Ticker warning flash — background fill on severity <= 4
typedef struct {
    float timer[4];  // HUD_TICKER_MAX
    float duration;
} annunc_ticker_flash_t;

// (f) Ring shake — horizontal oscillation on warning
typedef struct {
    float timer[ANNUNC_MAX_VEHICLES];
    float duration;
    int cycles;
    float amplitude;
} annunc_ring_shake_t;

// Aggregate
typedef struct {
    annunc_tab_fade_t     tab_fade;
    annunc_ring_bounce_t  ring_bounce;
    annunc_radar_wave_t   radar_wave;
    annunc_ticker_flash_t ticker_flash;
    annunc_ring_shake_t   ring_shake;
} hud_annunciators_t;

void annunc_init(hud_annunciators_t *a);
void annunc_update(hud_annunciators_t *a, float dt);

// Triggers
void annunc_trigger_tab_fade(hud_annunciators_t *a, int drone_idx);
void annunc_trigger_ring_bounce(hud_annunciators_t *a, int drone_idx);
void annunc_trigger_radar_wave(hud_annunciators_t *a, int drone_idx);
void annunc_trigger_ticker_flash(hud_annunciators_t *a, int slot);
void annunc_trigger_ring_shake(hud_annunciators_t *a, int drone_idx);

// Queries (return animation values)
float annunc_tab_fade_alpha(const hud_annunciators_t *a, int drone_idx);
float annunc_ring_bounce_offset(const hud_annunciators_t *a, int drone_idx);
float annunc_radar_wave_progress(const hud_annunciators_t *a, int drone_idx);
float annunc_ticker_flash_alpha(const hud_annunciators_t *a, int slot);
float annunc_ring_shake_offset(const hud_annunciators_t *a, int drone_idx);

#endif
