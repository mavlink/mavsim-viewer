#ifndef HUD_TELEMETRY_H
#define HUD_TELEMETRY_H

#include "raylib.h"
#include "hud.h"
#include "vehicle.h"
#include "data_source.h"
#include "theme.h"
#include <stdbool.h>

typedef struct {
    float nav_start;
    float nav_step;
    float energy_start;
    float energy_step;
    float item_x0;
    float item_step;
    float sep3_x;
    float timer_x;
    float status_x;
    int bar_y;
    int primary_h;
    int label_y;
    int value_y;
    float unit_y_off;
    float fs_label;
    float fs_value;
    float fs_unit;
    float fs_dim;
    float scale;
    Color label_color;
    Color value_color;
    Color dim_color;
    Color warn;
    Color climb_color;
    Color connected_color;
    int selected;  // selected drone index (for peak scale annunciator)
} hud_telemetry_layout_t;

void hud_draw_telemetry(const hud_t *h, const vehicle_t *v,
                        const hud_telemetry_layout_t *lay);

void hud_draw_status(const hud_t *h, const vehicle_t *v,
                     const data_source_t *source,
                     const hud_telemetry_layout_t *lay,
                     bool ghost_mode);

#endif
