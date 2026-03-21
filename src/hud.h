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
    Color toast_color;  // custom color (0,0,0,0 = use default)
} hud_t;

void hud_init(hud_t *h);
void hud_update(hud_t *h, uint64_t time_usec, bool connected, float dt);
void hud_draw(const hud_t *h, const vehicle_t *vehicles,
              const data_source_t *sources, int vehicle_count,
              int selected, int screen_w, int screen_h, view_mode_t view_mode,
              bool ghost_mode, bool has_tier3, bool has_awaiting_gps);
void hud_cleanup(hud_t *h);
void hud_toast(hud_t *h, const char *text, float duration_s);
void hud_toast_color(hud_t *h, const char *text, float duration_s, Color color);

// Returns the total height of the HUD bar in pixels (for layout by other panels).
int hud_bar_height(const hud_t *h, int screen_h);

#endif
