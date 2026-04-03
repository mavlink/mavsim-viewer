#ifndef DEBUG_PANEL_H
#define DEBUG_PANEL_H

#include "raylib.h"
#include "theme.h"
#include <stdbool.h>

#define DEBUG_FPS_HISTORY 120  // 2 seconds at 60fps

typedef struct {
    bool visible;
    float fps_history[DEBUG_FPS_HISTORY];
    float frametime_history[DEBUG_FPS_HISTORY];
    int history_idx;
    float update_timer;
    float peak_frametime;
} debug_panel_t;

void debug_panel_init(debug_panel_t *d);
void debug_panel_update(debug_panel_t *d, float dt);
void debug_panel_draw(const debug_panel_t *d, int screen_w, int screen_h,
                      const theme_t *theme, Font font,
                      int vehicle_count, int active_count,
                      int total_trail_points, Vector3 vehicle_pos,
                      bool ref_rejected, int position_tier);

#endif
