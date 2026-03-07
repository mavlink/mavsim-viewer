#ifndef HUD_H
#define HUD_H

#include "vehicle.h"
#include "mavlink_receiver.h"
#include <stdbool.h>

#define HUD_MAX_PINNED 15

typedef struct {
    float sim_time_s;
    int pinned[HUD_MAX_PINNED];   // indices of pinned vehicles (-1 = empty)
    int pinned_count;
    bool show_help;
    Font font_value;    // JetBrains Mono for telemetry numbers
    Font font_label;    // Inter for labels and status text
} hud_t;

void hud_init(hud_t *h);
void hud_update(hud_t *h, uint64_t time_usec, bool connected, float dt);
void hud_draw(const hud_t *h, const vehicle_t *vehicles,
              const mavlink_receiver_t *receivers, int vehicle_count,
              int selected, int screen_w, int screen_h, view_mode_t view_mode);
void hud_cleanup(hud_t *h);

#endif
