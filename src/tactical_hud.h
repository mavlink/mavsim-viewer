#ifndef TACTICAL_HUD_H
#define TACTICAL_HUD_H

#include "hud.h"
#include "ortho_panel.h"

void tactical_hud_draw(const hud_t *h, const vehicle_t *vehicles,
                       const data_source_t *sources, int vehicle_count,
                       int selected, int screen_w, int screen_h,
                       const theme_t *theme, bool ghost_mode,
                       bool has_tier3, bool has_awaiting_gps,
                       const ortho_panel_t *ortho,
                       int trail_mode, int corr_mode,
                       const hud_marker_data_t *markers_all,
                       const hud_marker_data_t *sys_markers_all,
                       int marker_vehicle_count);

#endif
