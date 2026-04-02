#ifndef HUD_H
#define HUD_H

#include "vehicle.h"
#include "data_source.h"
#include <stdbool.h>

#define HUD_MAX_PINNED 15

typedef struct {
    float sim_time_s;
    int pinned[HUD_MAX_PINNED];   // indices of pinned vehicles (-1 = empty)
    int pinned_count;
    bool show_help;
    bool is_replay;     // true when data source is ULog replay (affects layout)
    bool show_yaw;      // Y key: swap HDG for YAW display
    Font font_value;    // JetBrains Mono for telemetry numbers
    Font font_label;    // Inter for labels and status text

    // Toast notification (fades in/out above timeline)
    char toast_text[64];
    float toast_timer;  // seconds remaining (0 = hidden)
    float toast_total;  // total duration for fade calc
} hud_t;

void hud_init(hud_t *h);
void hud_update(hud_t *h, uint64_t time_usec, bool connected, float dt);
void hud_draw(const hud_t *h, const vehicle_t *vehicles,
              const data_source_t *sources, int vehicle_count,
              int selected, int screen_w, int screen_h, view_mode_t view_mode,
              int trail_mode,
              const float *marker_times, const char (*marker_labels)[48],
              int marker_count, int current_marker,
              const float *marker_roll, const float *marker_pitch,
              const float *marker_vert, const float *marker_speed, float marker_speed_max,
              const float *sys_marker_times, const char (*sys_marker_labels)[48],
              int sys_marker_count, int current_sys_marker, bool sys_marker_selected,
              const float *sys_marker_roll, const float *sys_marker_pitch,
              const float *sys_marker_vert, const float *sys_marker_speed,
              bool ghost_mode, bool has_tier3, bool has_awaiting_gps);
void hud_cleanup(hud_t *h);
void hud_toast(hud_t *h, const char *text, float duration_s);

// Returns the total height of the HUD bar in pixels (for layout by other panels).
int hud_bar_height(const hud_t *h, int screen_h);

#endif
