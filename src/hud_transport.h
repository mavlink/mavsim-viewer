#ifndef HUD_TRANSPORT_H
#define HUD_TRANSPORT_H

#include "raylib.h"
#include "hud.h"
#include "data_source.h"
#include "vehicle.h"
#include "theme.h"

void hud_draw_transport(const hud_t *h,
                        const playback_state_t *pb, bool connected,
                        const vehicle_t *selected_vehicle,
                        const hud_marker_data_t *markers_all,
                        const hud_marker_data_t *sys_markers_all,
                        int marker_vehicle_count, int selected_idx,
                        int screen_w, float bar_y, float transport_h,
                        float scale, int trail_mode, const theme_t *theme);

#endif
