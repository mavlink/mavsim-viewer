#ifndef HUD_H
#define HUD_H

#include "vehicle.h"
#include <stdbool.h>

typedef struct {
    float sim_time_s;
} hud_t;

void hud_init(hud_t *h);
void hud_update(hud_t *h, uint64_t time_usec);
void hud_draw(const hud_t *h, const vehicle_t *v, bool connected, int screen_w, int screen_h,
              int vehicle_idx, int vehicle_total, uint8_t sysid, view_mode_t view_mode);
void hud_cleanup(hud_t *h);

#endif
