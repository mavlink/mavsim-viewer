#ifndef THEME_H
#define THEME_H

#include "raylib.h"

#define THEME_DRONE_PALETTE_SIZE 16
#define THEME_THERMAL_STOPS 7

typedef struct {
    const char *name;

    // Scene
    Color sky;
    Color ground;
    Color grid_minor;
    Color grid_major;
    Color axis_x;
    Color axis_z;
    Color fog;
    Color tint;

    // HUD
    Color hud_accent;
    Color hud_accent_dim;
    Color hud_highlight;     // secondary accent (e.g., 1988 cyan vs pink)
    Color hud_warn;
    Color hud_bg;
    Color hud_border;
    Color adi_sky;           // attitude indicator sky half
    Color adi_ground;        // attitude indicator ground half
    Color adi_horizon;       // horizon line
    Color adi_wing;          // wing/reference marker

    // Trail direction
    Color trail_forward;
    Color trail_backward;
    Color trail_climb;
    Color trail_descend;
    Color trail_roll_pos;
    Color trail_roll_neg;

    // Arms (modern)
    Color arm_front;
    Color arm_back;
    Color arm_port;
    Color arm_starboard;

    // Arms (classic)
    Color arm_classic_front;
    Color arm_classic_back;
    Color arm_classic_side;

    // Thermal gradient (speed ribbon) — 7 stops at heat 0.0, 0.16, 0.33, 0.50, 0.66, 0.83, 1.0
    Color thermal[THEME_THERMAL_STOPS];

    // Debug panel
    Color dbg_accent;
    Color dbg_text;
    Color dbg_dim;
    Color dbg_bg;
    Color dbg_graph_bg;
    Color dbg_graph_grid;
    Color dbg_tier1;
    Color dbg_tier2;
    Color dbg_tier3;

    // Deconfliction prompt
    Color prompt_scrim;
    Color prompt_box_bg;
    Color prompt_border;
    Color prompt_subtitle;
    Color prompt_hint;
    Color prompt_key;
    Color prompt_text;
    Color prompt_title;

    // Ground track marker
    Color ground_track_fill;
    Color ground_track_edge;
    Color ground_track_shadow;

    // Ortho panel grid
    Color ortho_grid_minor;
    Color ortho_grid_major;

    // Drone color palette (16 colors)
    Color drone_palette[THEME_DRONE_PALETTE_SIZE];

} theme_t;

// Built-in themes
extern const theme_t theme_grid;
extern const theme_t theme_rez;
extern const theme_t theme_snow;
extern const theme_t theme_1988;

// Get theme for a view mode
const theme_t *theme_for_view_mode(int view_mode);

// Thermal gradient lookup — returns interpolated color for heat 0.0–1.0
Color theme_heat_color(const theme_t *t, float heat, unsigned char alpha);

#endif
