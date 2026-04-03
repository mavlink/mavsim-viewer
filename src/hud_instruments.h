#ifndef HUD_INSTRUMENTS_H
#define HUD_INSTRUMENTS_H

#include "raylib.h"
#include "theme.h"

void hud_draw_compass(float cx, float cy, float radius,
                      float heading_deg, const theme_t *theme, Font font_value);

void hud_draw_attitude(float cx, float cy, float radius,
                       float roll_deg, float pitch_deg, const theme_t *theme);

#endif
