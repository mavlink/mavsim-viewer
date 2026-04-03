#ifndef UI_MARKER_INPUT_H
#define UI_MARKER_INPUT_H

#include "raylib.h"
#include "replay_markers.h"
#include "replay_trail.h"
#include "data_source.h"
#include "vehicle.h"
#include "theme.h"
#include "hud.h"
#include <stdbool.h>

typedef struct {
    bool active;
    char buf[HUD_MARKER_LABEL_MAX];
    int len;
    int target;
} marker_input_t;

void marker_input_update(marker_input_t *mi, user_markers_t *um,
                         data_source_t *source, vehicle_t *vehicle);

void marker_input_draw(const marker_input_t *mi, Font font_label, Font font_value,
                       const theme_t *theme, int screen_w, int screen_h);

void marker_input_begin(marker_input_t *mi, int marker_idx,
                        data_source_t *source);

#endif
