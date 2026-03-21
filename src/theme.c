#include "theme.h"
#include "scene.h"
#include <math.h>

// ── Grid (default) ──────────────────────────────────────────────────────────

const theme_t theme_grid = {
    .name = "Grid",

    // Scene
    .sky        = { 56, 56, 60, 255 },
    .ground     = { 32, 32, 34, 255 },
    .grid_minor = { 97, 97, 97, 128 },
    .grid_major = { 143, 143, 143, 128 },
    .axis_x     = { 200, 60, 60, 180 },
    .axis_z     = { 60, 60, 200, 180 },
    .fog        = { 30, 34, 28, 255 },
    .tint       = { 42, 38, 32, 255 },

    // HUD
    .hud_accent     = { 0, 180, 204, 255 },
    .hud_accent_dim = { 0, 180, 204, 140 },
    .hud_highlight  = { 255, 255, 255, 255 },
    .hud_warn       = { 255, 140, 0, 255 },
    .hud_bg         = { 10, 14, 20, 220 },
    .hud_border     = { 0, 180, 204, 100 },
    .adi_sky        = { 80, 140, 200, 200 },
    .adi_ground     = { 120, 85, 50, 200 },
    .adi_horizon    = { 255, 255, 255, 255 },
    .adi_wing       = { 255, 255, 0, 255 },

    // Trail direction
    .trail_forward  = { 255, 200, 50, 180 },
    .trail_backward = { 160, 60, 255, 255 },
    .trail_climb    = { 0, 220, 255, 255 },
    .trail_descend  = { 255, 140, 0, 255 },
    .trail_roll_pos = { 40, 255, 80, 255 },
    .trail_roll_neg = { 255, 40, 80, 255 },

    // Arms (modern)
    .arm_front      = { 255, 200, 50, 255 },
    .arm_back       = { 160, 60, 255, 255 },
    .arm_port       = { 204, 33, 33, 255 },
    .arm_starboard  = { 41, 255, 79, 255 },

    // Arms (classic)
    .arm_classic_front = { 255, 47, 43, 255 },
    .arm_classic_back  = { 39, 47, 197, 255 },
    .arm_classic_side  = { 14, 31, 47, 255 },

    // Thermal gradient
    .thermal = {
        { 120, 10, 160, 255 },   // 0.00
        { 160, 20, 180, 255 },   // 0.16
        { 200, 20, 120, 255 },   // 0.33
        { 255, 20, 0, 255 },     // 0.50
        { 255, 160, 0, 255 },    // 0.66
        { 255, 255, 40, 255 },   // 0.83
        { 255, 255, 255, 255 },  // 1.00
    },

    // Debug panel
    .dbg_accent    = { 120, 180, 255, 220 },
    .dbg_text      = { 200, 210, 225, 230 },
    .dbg_dim       = { 120, 180, 255, 100 },
    .dbg_bg        = { 8, 8, 16, 210 },
    .dbg_graph_bg  = { 4, 4, 10, 200 },
    .dbg_graph_grid = { 40, 40, 60, 80 },
    .dbg_tier1     = { 80, 255, 120, 255 },
    .dbg_tier2     = { 255, 220, 40, 255 },
    .dbg_tier3     = { 255, 100, 40, 255 },

    // Deconfliction prompt
    .prompt_scrim    = { 0, 0, 0, 140 },
    .prompt_box_bg   = { 10, 14, 20, 235 },
    .prompt_border   = { 0, 180, 204, 100 },
    .prompt_subtitle = { 140, 150, 170, 255 },
    .prompt_hint     = { 90, 95, 110, 200 },
    .prompt_key      = { 0, 255, 255, 220 },
    .prompt_text     = { 255, 255, 255, 255 },
    .prompt_title    = { 255, 255, 0, 255 },

    // Ground track
    .ground_track_fill   = { 0, 0, 0, 100 },
    .ground_track_edge   = { 80, 80, 80, 180 },
    .ground_track_shadow = { 150, 150, 150, 120 },

    // Ortho panel grid
    .ortho_grid_minor = { 115, 115, 120, 200 },
    .ortho_grid_major = { 175, 175, 180, 240 },

    // Drone palette
    .drone_palette = {
        { 230, 230, 230, 255 },  //  0: white (primary)
        {  40, 120, 255, 255 },  //  1: blue
        { 255,  40,  80, 255 },  //  2: red
        { 255, 200,  40, 255 },  //  3: yellow
        {  40, 220,  80, 255 },  //  4: green
        { 255, 140,   0, 255 },  //  5: orange
        { 180,  60, 255, 255 },  //  6: purple
        { 255, 100, 160, 255 },  //  7: pink
        {   0, 200, 200, 255 },  //  8: teal
        { 255, 220, 100, 255 },  //  9: gold
        { 100, 100, 255, 255 },  // 10: indigo
        { 255, 180, 140, 255 },  // 11: peach
        { 140, 255, 200, 255 },  // 12: mint
        { 255,  60, 200, 255 },  // 13: magenta
        { 140, 200, 255, 255 },  // 14: sky blue
        { 200, 255,  60, 255 },  // 15: lime
    },
};

// ── Rez ─────────────────────────────────────────────────────────────────────

const theme_t theme_rez = {
    .name = "Rez",

    // Scene
    .sky        = { 12, 12, 18, 255 },
    .ground     = { 2, 2, 4, 255 },
    .grid_minor = { 0, 204, 218, 50 },
    .grid_major = { 0, 204, 218, 140 },
    .axis_x     = { 0, 204, 218, 220 },
    .axis_z     = { 0, 204, 218, 220 },
    .fog        = { 31, 31, 59, 255 },
    .tint       = { 31, 31, 59, 255 },

    // HUD
    .hud_accent     = { 0, 204, 218, 255 },
    .hud_accent_dim = { 0, 204, 218, 140 },
    .hud_highlight  = { 0, 204, 218, 255 },
    .hud_warn       = { 255, 106, 0, 255 },
    .hud_bg         = { 8, 8, 12, 220 },
    .hud_border     = { 0, 204, 218, 100 },
    .adi_sky        = { 0, 40, 45, 200 },
    .adi_ground     = { 20, 10, 2, 200 },
    .adi_horizon    = { 0, 204, 218, 255 },
    .adi_wing       = { 255, 106, 0, 255 },

    // Trail direction
    .trail_forward  = { 220, 180, 30, 160 },
    .trail_backward = { 160, 40, 240, 255 },
    .trail_climb    = { 0, 200, 255, 255 },
    .trail_descend  = { 255, 160, 0, 255 },
    .trail_roll_pos = { 40, 255, 100, 255 },
    .trail_roll_neg = { 255, 40, 80, 255 },

    // Arms (modern)
    .arm_front      = { 220, 180, 30, 255 },
    .arm_back       = { 160, 40, 240, 255 },
    .arm_port       = { 255, 106, 0, 255 },
    .arm_starboard  = { 30, 200, 80, 255 },

    // Arms (classic)
    .arm_classic_front = { 255, 106, 0, 255 },
    .arm_classic_back  = { 39, 47, 197, 255 },
    .arm_classic_side  = { 14, 31, 47, 255 },

    // Thermal gradient
    .thermal = {
        { 50, 20, 140, 255 },    // 0.00 indigo
        { 70, 50, 180, 255 },    // 0.16
        { 0, 150, 200, 255 },    // 0.33
        { 200, 80, 40, 255 },    // 0.50
        { 255, 160, 0, 255 },    // 0.66
        { 255, 220, 60, 255 },   // 0.83
        { 255, 255, 240, 255 },  // 1.00
    },

    // Debug panel
    .dbg_accent    = { 0, 204, 218, 220 },
    .dbg_text      = { 180, 230, 235, 230 },
    .dbg_dim       = { 0, 204, 218, 100 },
    .dbg_bg        = { 8, 8, 12, 210 },
    .dbg_graph_bg  = { 4, 4, 10, 200 },
    .dbg_graph_grid = { 0, 60, 70, 80 },
    .dbg_tier1     = { 30, 200, 80, 255 },
    .dbg_tier2     = { 220, 180, 30, 255 },
    .dbg_tier3     = { 255, 106, 0, 255 },

    // Deconfliction prompt
    .prompt_scrim    = { 0, 0, 0, 150 },
    .prompt_box_bg   = { 8, 8, 12, 235 },
    .prompt_border   = { 0, 204, 218, 100 },
    .prompt_subtitle = { 0, 140, 150, 160 },
    .prompt_hint     = { 0, 140, 150, 160 },
    .prompt_key      = { 0, 204, 218, 220 },
    .prompt_text     = { 200, 208, 218, 255 },
    .prompt_title    = { 255, 255, 0, 255 },

    // Ground track
    .ground_track_fill   = { 0, 204, 218, 80 },
    .ground_track_edge   = { 0, 204, 218, 200 },
    .ground_track_shadow = { 0, 204, 218, 140 },

    // Ortho panel grid
    .ortho_grid_minor = { 0, 204, 218, 80 },
    .ortho_grid_major = { 0, 204, 218, 190 },

    // Drone palette
    .drone_palette = {
        { 200, 210, 220, 255 },  //  0: light gray
        {  40, 160, 255, 255 },  //  1: cyan-blue
        { 255,  60,  80, 255 },  //  2: red
        { 220, 200,  40, 255 },  //  3: gold
        {  30, 200, 100, 255 },  //  4: green
        { 255, 130,   0, 255 },  //  5: orange
        { 160,  40, 240, 255 },  //  6: purple
        { 255, 100, 160, 255 },  //  7: pink
        {   0, 204, 218, 255 },  //  8: teal
        { 255, 220, 100, 255 },  //  9: gold
        { 100, 120, 255, 255 },  // 10: indigo
        { 255, 180, 140, 255 },  // 11: peach
        { 140, 255, 200, 255 },  // 12: mint
        { 255,  60, 200, 255 },  // 13: magenta
        { 140, 200, 255, 255 },  // 14: sky blue
        { 200, 255,  60, 255 },  // 15: lime
    },
};

// ── Snow ────────────────────────────────────────────────────────────────────

const theme_t theme_snow = {
    .name = "Snow",

    // Scene
    .sky        = { 240, 242, 245, 255 },
    .ground     = { 228, 230, 233, 255 },
    .grid_minor = { 155, 160, 168, 140 },
    .grid_major = { 70, 75, 85, 200 },
    .axis_x     = { 210, 30, 30, 230 },
    .axis_z     = { 30, 30, 210, 230 },
    .fog        = { 215, 218, 222, 255 },
    .tint       = { 215, 218, 222, 255 },

    // HUD
    .hud_accent     = { 15, 15, 20, 255 },
    .hud_accent_dim = { 60, 65, 75, 200 },
    .hud_highlight  = { 255, 255, 255, 255 },
    .hud_warn       = { 200, 40, 0, 255 },
    .hud_bg         = { 248, 248, 250, 235 },
    .hud_border     = { 15, 15, 20, 140 },
    .adi_sky        = { 160, 185, 215, 220 },
    .adi_ground     = { 20, 30, 55, 220 },
    .adi_horizon    = { 255, 255, 255, 255 },
    .adi_wing       = { 200, 40, 0, 255 },

    // Trail direction
    .trail_forward  = { 200, 140, 20, 200 },
    .trail_backward = { 140, 20, 200, 255 },
    .trail_climb    = { 0, 150, 60, 255 },
    .trail_descend  = { 200, 50, 0, 255 },
    .trail_roll_pos = { 20, 160, 40, 255 },
    .trail_roll_neg = { 200, 20, 60, 255 },

    // Arms (modern)
    .arm_front      = { 200, 140, 20, 255 },
    .arm_back       = { 140, 20, 200, 255 },
    .arm_port       = { 180, 30, 30, 255 },
    .arm_starboard  = { 20, 160, 50, 255 },

    // Arms (classic)
    .arm_classic_front = { 200, 30, 30, 255 },
    .arm_classic_back  = { 30, 60, 180, 255 },
    .arm_classic_side  = { 160, 160, 170, 255 },

    // Thermal gradient
    .thermal = {
        { 20, 30, 80, 255 },     // 0.00 deep navy
        { 40, 60, 180, 255 },    // 0.16
        { 0, 140, 160, 255 },    // 0.33
        { 40, 160, 60, 255 },    // 0.50
        { 200, 200, 0, 255 },    // 0.66
        { 220, 80, 0, 255 },     // 0.83
        { 200, 30, 30, 255 },    // 1.00
    },

    // Debug panel
    .dbg_accent    = { 40, 50, 70, 220 },
    .dbg_text      = { 30, 35, 50, 230 },
    .dbg_dim       = { 100, 110, 130, 140 },
    .dbg_bg        = { 235, 237, 240, 230 },
    .dbg_graph_bg  = { 220, 222, 226, 220 },
    .dbg_graph_grid = { 180, 185, 195, 100 },
    .dbg_tier1     = { 20, 140, 60, 255 },
    .dbg_tier2     = { 180, 130, 0, 255 },
    .dbg_tier3     = { 200, 60, 20, 255 },

    // Deconfliction prompt
    .prompt_scrim    = { 255, 255, 255, 140 },
    .prompt_box_bg   = { 248, 248, 250, 240 },
    .prompt_border   = { 15, 15, 20, 120 },
    .prompt_subtitle = { 60, 65, 75, 255 },
    .prompt_hint     = { 120, 125, 135, 200 },
    .prompt_key      = { 15, 15, 20, 220 },
    .prompt_text     = { 10, 10, 15, 255 },
    .prompt_title    = { 200, 140, 0, 255 },

    // Ground track
    .ground_track_fill   = { 0, 0, 0, 100 },
    .ground_track_edge   = { 80, 80, 80, 180 },
    .ground_track_shadow = { 150, 150, 150, 120 },

    // Ortho panel grid
    .ortho_grid_minor = { 140, 145, 155, 180 },
    .ortho_grid_major = { 50, 55, 65, 230 },

    // Drone palette
    .drone_palette = {
        {  60,  65,  75, 255 },  //  0: dark gray (primary)
        {  30,  80, 200, 255 },  //  1: blue
        { 200,  30,  50, 255 },  //  2: red
        { 180, 140,  20, 255 },  //  3: gold
        {  20, 140,  50, 255 },  //  4: green
        { 200, 100,   0, 255 },  //  5: orange
        { 130,  30, 200, 255 },  //  6: purple
        { 200,  70, 120, 255 },  //  7: pink
        {   0, 140, 140, 255 },  //  8: teal
        { 180, 160,  60, 255 },  //  9: gold
        {  60,  60, 180, 255 },  // 10: indigo
        { 200, 130, 100, 255 },  // 11: peach
        { 100, 180, 140, 255 },  // 12: mint
        { 200,  40, 140, 255 },  // 13: magenta
        { 100, 140, 200, 255 },  // 14: sky blue
        { 140, 180,  40, 255 },  // 15: lime
    },
};

// ── 1988 ────────────────────────────────────────────────────────────────────

const theme_t theme_1988 = {
    .name = "1988",

    // Scene
    .sky        = { 8, 8, 20, 255 },
    .ground     = { 5, 5, 16, 255 },
    .grid_minor = { 255, 20, 100, 50 },
    .grid_major = { 255, 20, 100, 160 },
    .axis_x     = { 255, 20, 100, 220 },
    .axis_z     = { 255, 20, 100, 220 },
    .fog        = { 25, 25, 82, 255 },
    .tint       = { 25, 25, 82, 255 },

    // HUD
    .hud_accent     = { 255, 20, 100, 255 },
    .hud_accent_dim = { 255, 20, 100, 140 },
    .hud_highlight  = { 21, 190, 254, 255 },
    .hud_warn       = { 255, 140, 0, 255 },
    .hud_bg         = { 5, 5, 16, 220 },
    .hud_border     = { 255, 20, 100, 100 },
    .adi_sky        = { 0, 4, 12, 200 },
    .adi_ground     = { 251, 153, 54, 200 },
    .adi_horizon    = { 112, 40, 21, 255 },
    .adi_wing       = { 21, 190, 254, 255 },

    // Trail direction
    .trail_forward  = { 255, 220, 60, 160 },
    .trail_backward = { 180, 40, 255, 255 },
    .trail_climb    = { 0, 240, 255, 255 },
    .trail_descend  = { 255, 140, 0, 255 },
    .trail_roll_pos = { 40, 255, 80, 255 },
    .trail_roll_neg = { 255, 40, 80, 255 },

    // Arms (modern)
    .arm_front      = { 255, 220, 60, 255 },
    .arm_back       = { 180, 40, 255, 255 },
    .arm_port       = { 255, 20, 100, 255 },
    .arm_starboard  = { 40, 255, 100, 255 },

    // Arms (classic)
    .arm_classic_front = { 255, 20, 100, 255 },
    .arm_classic_back  = { 39, 47, 197, 255 },
    .arm_classic_side  = { 14, 31, 47, 255 },

    // Thermal gradient
    .thermal = {
        { 60, 5, 120, 255 },     // 0.00 deep purple
        { 100, 10, 160, 255 },   // 0.16
        { 160, 10, 180, 255 },   // 0.33
        { 220, 30, 140, 255 },   // 0.50
        { 255, 130, 0, 255 },    // 0.66
        { 255, 225, 0, 255 },    // 0.83
        { 255, 255, 200, 255 },  // 1.00
    },

    // Debug panel
    .dbg_accent    = { 255, 20, 100, 220 },
    .dbg_text      = { 255, 200, 210, 230 },
    .dbg_dim       = { 255, 20, 100, 100 },
    .dbg_bg        = { 5, 5, 16, 210 },
    .dbg_graph_bg  = { 4, 4, 10, 200 },
    .dbg_graph_grid = { 60, 20, 40, 80 },
    .dbg_tier1     = { 40, 255, 80, 255 },
    .dbg_tier2     = { 255, 220, 60, 255 },
    .dbg_tier3     = { 255, 40, 80, 255 },

    // Deconfliction prompt
    .prompt_scrim    = { 5, 0, 15, 160 },
    .prompt_box_bg   = { 5, 5, 16, 240 },
    .prompt_border   = { 255, 20, 100, 140 },
    .prompt_subtitle = { 255, 20, 100, 200 },
    .prompt_hint     = { 180, 60, 120, 160 },
    .prompt_key      = { 255, 220, 60, 255 },
    .prompt_text     = { 255, 220, 60, 255 },
    .prompt_title    = { 255, 20, 100, 255 },

    // Ground track
    .ground_track_fill   = { 255, 20, 100, 80 },
    .ground_track_edge   = { 255, 20, 100, 200 },
    .ground_track_shadow = { 255, 20, 100, 140 },

    // Ortho panel grid
    .ortho_grid_minor = { 255, 20, 100, 80 },
    .ortho_grid_major = { 255, 20, 100, 200 },

    // Drone palette
    .drone_palette = {
        { 230, 230, 230, 255 },  //  0: white
        {  40, 120, 255, 255 },  //  1: blue
        { 255,  40,  80, 255 },  //  2: red
        { 255, 220,  60, 255 },  //  3: yellow
        {  40, 255, 100, 255 },  //  4: neon green
        { 255, 140,   0, 255 },  //  5: orange
        { 180,  40, 255, 255 },  //  6: purple
        { 255,  80, 160, 255 },  //  7: hot pink
        {   0, 240, 255, 255 },  //  8: cyan
        { 255, 220, 100, 255 },  //  9: gold
        { 100, 100, 255, 255 },  // 10: indigo
        { 255, 180, 140, 255 },  // 11: peach
        { 140, 255, 200, 255 },  // 12: mint
        { 255,  20, 200, 255 },  // 13: magenta
        { 140, 200, 255, 255 },  // 14: sky blue
        { 200, 255,  60, 255 },  // 15: lime
    },
};

// ── Lookup ──────────────────────────────────────────────────────────────────

const theme_t *theme_for_view_mode(int view_mode) {
    switch (view_mode) {
        case VIEW_REZ:  return &theme_rez;
        case VIEW_SNOW: return &theme_snow;
        case VIEW_1988: return &theme_1988;
        default:        return &theme_grid;
    }
}

// ── Thermal gradient interpolation ──────────────────────────────────────────

Color theme_heat_color(const theme_t *t, float heat, unsigned char alpha) {
    if (heat < 0.0f) heat = 0.0f;
    if (heat > 1.0f) heat = 1.0f;

    // 7 stops at positions 0.0, 0.166, 0.333, 0.5, 0.666, 0.833, 1.0
    float scaled = heat * 6.0f;
    int idx = (int)scaled;
    if (idx >= 6) idx = 5;
    float frac = scaled - (float)idx;

    const Color *a = &t->thermal[idx];
    const Color *b = &t->thermal[idx + 1];

    return (Color){
        (unsigned char)(a->r + (b->r - a->r) * frac),
        (unsigned char)(a->g + (b->g - a->g) * frac),
        (unsigned char)(a->b + (b->b - a->b) * frac),
        alpha
    };
}
