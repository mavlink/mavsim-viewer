#ifndef HUD_HELP_H
#define HUD_HELP_H

#include "raylib.h"
#include "theme.h"

void hud_draw_help(Font font_value, Font font_label,
                   int screen_w, int screen_h, const theme_t *theme);

#endif
