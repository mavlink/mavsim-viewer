#include "theme.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#ifdef _WIN32
#define NOGDI
#define NOUSER
#include <windows.h>
#else
#include <dirent.h>
#endif

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
    .hud_value      = { 200, 208, 218, 255 },
    .hud_dim        = { 200, 208, 218, 100 },
    .hud_climb      = { 0, 255, 0, 255 },
    .hud_connected  = { 100, 200, 100, 255 },

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

    .thick_trails = false,
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
    .hud_value      = { 255, 220, 60, 255 },
    .hud_dim        = { 255, 20, 100, 100 },
    .hud_climb      = { 40, 255, 80, 255 },
    .hud_connected  = { 40, 255, 100, 255 },

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

    .thick_trails = false,
};

// ── .mvt field lookup table ─────────────────────────────────────────────────

typedef struct {
    const char *section;
    const char *key;
    size_t offset;       // offsetof into theme_t
    int array_index;     // -1 for non-array, >= 0 for array element
} mvt_field_t;

#define F(sec, k, field)       { sec, k, offsetof(theme_t, field), -1 }
#define FA(sec, k, field, idx) { sec, k, offsetof(theme_t, field), idx }

static const mvt_field_t mvt_fields[] = {
    // [scene]
    F("scene", "sky",        sky),
    F("scene", "ground",     ground),
    F("scene", "grid_minor", grid_minor),
    F("scene", "grid_major", grid_major),
    F("scene", "axis_x",     axis_x),
    F("scene", "axis_z",     axis_z),
    F("scene", "fog",        fog),
    F("scene", "tint",       tint),

    // [hud]
    F("hud", "accent",       hud_accent),
    F("hud", "accent_dim",   hud_accent_dim),
    F("hud", "highlight",    hud_highlight),
    F("hud", "warn",         hud_warn),
    F("hud", "bg",           hud_bg),
    F("hud", "border",       hud_border),
    F("hud", "adi_sky",      adi_sky),
    F("hud", "adi_ground",   adi_ground),
    F("hud", "adi_horizon",  adi_horizon),
    F("hud", "adi_wing",     adi_wing),
    F("hud", "value",        hud_value),
    F("hud", "dim",          hud_dim),
    F("hud", "climb",        hud_climb),
    F("hud", "connected",    hud_connected),

    // [trail]
    F("trail", "forward",    trail_forward),
    F("trail", "backward",   trail_backward),
    F("trail", "climb",      trail_climb),
    F("trail", "descend",    trail_descend),
    F("trail", "roll_pos",   trail_roll_pos),
    F("trail", "roll_neg",   trail_roll_neg),

    // [arms]
    F("arms", "front",         arm_front),
    F("arms", "back",          arm_back),
    F("arms", "port",          arm_port),
    F("arms", "starboard",     arm_starboard),
    F("arms", "classic_front", arm_classic_front),
    F("arms", "classic_back",  arm_classic_back),
    F("arms", "classic_side",  arm_classic_side),

    // [ribbon_gradient]
    FA("ribbon_gradient", "stop_0", thermal, 0),
    FA("ribbon_gradient", "stop_1", thermal, 1),
    FA("ribbon_gradient", "stop_2", thermal, 2),
    FA("ribbon_gradient", "stop_3", thermal, 3),
    FA("ribbon_gradient", "stop_4", thermal, 4),
    FA("ribbon_gradient", "stop_5", thermal, 5),
    FA("ribbon_gradient", "stop_6", thermal, 6),

    // [debug]
    F("debug", "accent",     dbg_accent),
    F("debug", "text",       dbg_text),
    F("debug", "dim",        dbg_dim),
    F("debug", "bg",         dbg_bg),
    F("debug", "graph_bg",   dbg_graph_bg),
    F("debug", "graph_grid", dbg_graph_grid),
    F("debug", "tier1",      dbg_tier1),
    F("debug", "tier2",      dbg_tier2),
    F("debug", "tier3",      dbg_tier3),

    // [prompt]
    F("prompt", "scrim",     prompt_scrim),
    F("prompt", "box_bg",    prompt_box_bg),
    F("prompt", "border",    prompt_border),
    F("prompt", "subtitle",  prompt_subtitle),
    F("prompt", "hint",      prompt_hint),
    F("prompt", "key",       prompt_key),
    F("prompt", "text",      prompt_text),
    F("prompt", "title",     prompt_title),

    // [ground_track]
    F("ground_track", "fill",   ground_track_fill),
    F("ground_track", "edge",   ground_track_edge),
    F("ground_track", "shadow", ground_track_shadow),

    // [ortho]
    F("ortho", "grid_minor", ortho_grid_minor),
    F("ortho", "grid_major", ortho_grid_major),

    // [drone_palette]
    FA("drone_palette", "color_0",  drone_palette, 0),
    FA("drone_palette", "color_1",  drone_palette, 1),
    FA("drone_palette", "color_2",  drone_palette, 2),
    FA("drone_palette", "color_3",  drone_palette, 3),
    FA("drone_palette", "color_4",  drone_palette, 4),
    FA("drone_palette", "color_5",  drone_palette, 5),
    FA("drone_palette", "color_6",  drone_palette, 6),
    FA("drone_palette", "color_7",  drone_palette, 7),
    FA("drone_palette", "color_8",  drone_palette, 8),
    FA("drone_palette", "color_9",  drone_palette, 9),
    FA("drone_palette", "color_10", drone_palette, 10),
    FA("drone_palette", "color_11", drone_palette, 11),
    FA("drone_palette", "color_12", drone_palette, 12),
    FA("drone_palette", "color_13", drone_palette, 13),
    FA("drone_palette", "color_14", drone_palette, 14),
    FA("drone_palette", "color_15", drone_palette, 15),
};

#define MVT_FIELD_COUNT (sizeof(mvt_fields) / sizeof(mvt_fields[0]))

#undef F
#undef FA

// ── .mvt parser ────────────────────────────────────────────────────────────

bool theme_load_mvt(const char *path, theme_t *out, char *name_buf, int name_buf_size, int *priority_out) {
    FILE *f = fopen(path, "r");
    if (!f) return false;

    // Pre-fill with Grid defaults so partial files work
    *out = theme_grid;
    out->name = NULL;
    if (priority_out) *priority_out = 0;

    char section[64] = {0};
    char line[256];

    while (fgets(line, sizeof(line), f)) {
        // Strip trailing whitespace/newline
        int len = (int)strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r' || line[len-1] == ' '))
            line[--len] = '\0';

        // Skip empty lines and comments
        if (len == 0 || line[0] == '#') continue;

        // Section header: [section_name]
        if (line[0] == '[') {
            char *end = strchr(line, ']');
            if (end) {
                *end = '\0';
                strncpy(section, line + 1, sizeof(section) - 1);
                section[sizeof(section) - 1] = '\0';
            }
            continue;
        }

        // Key: value line
        char *colon = strchr(line, ':');
        if (!colon) continue;

        *colon = '\0';
        char *key = line;
        char *val = colon + 1;

        // Trim trailing spaces from key
        int klen = (int)strlen(key);
        while (klen > 0 && key[klen-1] == ' ') key[--klen] = '\0';

        // Trim leading spaces from value
        while (*val == ' ') val++;

        // Handle theme name
        if (strcmp(section, "theme") == 0 && strcmp(key, "name") == 0) {
            if (name_buf && name_buf_size > 0) {
                strncpy(name_buf, val, name_buf_size - 1);
                name_buf[name_buf_size - 1] = '\0';
            }
            continue;
        }

        // Handle priority
        if (strcmp(section, "theme") == 0 && strcmp(key, "priority") == 0) {
            if (priority_out) *priority_out = atoi(val);
            continue;
        }

        // Handle thick_trails
        if (strcmp(section, "theme") == 0 && strcmp(key, "thick_trails") == 0) {
            out->thick_trails = (atoi(val) != 0 || strcmp(val, "true") == 0);
            continue;
        }

        // Parse R G B A
        int r, g, b, a;
        if (sscanf(val, "%d %d %d %d", &r, &g, &b, &a) != 4) continue;
        if (r < 0) r = 0; if (r > 255) r = 255;
        if (g < 0) g = 0; if (g > 255) g = 255;
        if (b < 0) b = 0; if (b > 255) b = 255;
        if (a < 0) a = 0; if (a > 255) a = 255;
        Color color = { (unsigned char)r, (unsigned char)g, (unsigned char)b, (unsigned char)a };

        // Look up field
        for (int i = 0; i < (int)MVT_FIELD_COUNT; i++) {
            if (strcmp(section, mvt_fields[i].section) == 0 &&
                strcmp(key, mvt_fields[i].key) == 0) {
                Color *target;
                if (mvt_fields[i].array_index >= 0) {
                    target = (Color *)((char *)out + mvt_fields[i].offset) + mvt_fields[i].array_index;
                } else {
                    target = (Color *)((char *)out + mvt_fields[i].offset);
                }
                *target = color;
                break;
            }
        }
    }

    fclose(f);
    return true;
}

// ── Theme registry ─────────────────────────────────────────────────────────

void theme_registry_init(theme_registry_t *reg) {
    memset(reg, 0, sizeof(*reg));

    // Built-in themes (only Grid is hardcoded; others ship as .mvt files)
    reg->themes[0] = &theme_grid;
    reg->priorities[0] = 100;
    reg->count = 1;
    reg->cyclable = 1;
    reg->user_count = 0;

    // Scan ./themes/ for .mvt files (includes shipped Rez, Snow, and user themes)
#ifdef _WIN32
    WIN32_FIND_DATAA fd;
    HANDLE hFind = FindFirstFileA("./themes/*.mvt", &fd);
    if (hFind == INVALID_HANDLE_VALUE) {
        printf("Loaded %d themes (1 built-in + 0 from themes/)\n", reg->count);
        return;
    }
    do {
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        const char *name = fd.cFileName;
#else
    DIR *dir = opendir("./themes");
    if (!dir) {
        printf("Loaded %d themes (1 built-in + 0 from themes/)\n", reg->count);
        return;
    }
    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && reg->user_count < MAX_USER_THEMES) {
        const char *name = ent->d_name;
#endif
        int nlen = (int)strlen(name);
        if (nlen < 5 || strcmp(name + nlen - 4, ".mvt") != 0) continue;

        char filepath[512];
        snprintf(filepath, sizeof(filepath), "./themes/%s", name);

        int idx = reg->user_count;
        int priority = 0;
        if (theme_load_mvt(filepath, &reg->user_storage[idx], reg->name_bufs[idx], 64, &priority)) {
            reg->user_storage[idx].name = reg->name_bufs[idx];
            if (reg->name_bufs[idx][0] == '\0') {
                int copy_len = nlen - 4;
                if (copy_len > 63) copy_len = 63;
                memcpy(reg->name_bufs[idx], name, copy_len);
                reg->name_bufs[idx][copy_len] = '\0';
            }
            reg->themes[reg->count] = &reg->user_storage[idx];
            reg->priorities[reg->count] = priority;
            reg->count++;
            reg->cyclable++;
            reg->user_count++;
            printf("Theme loaded: %s (%s, priority %d)\n", reg->name_bufs[idx], name, priority);
        }
#ifdef _WIN32
    } while (FindNextFileA(hFind, &fd) && reg->user_count < MAX_USER_THEMES);
    FindClose(hFind);
#else
    }
    closedir(dir);
#endif

    // Sort by priority (descending) — higher weight = earlier in cycle
    // Simple insertion sort, skip index 0 (Grid is always first)
    for (int i = 2; i < reg->count; i++) {
        const theme_t *t = reg->themes[i];
        int p = reg->priorities[i];
        int j = i - 1;
        while (j >= 1 && reg->priorities[j] < p) {
            reg->themes[j + 1] = reg->themes[j];
            reg->priorities[j + 1] = reg->priorities[j];
            j--;
        }
        reg->themes[j + 1] = t;
        reg->priorities[j + 1] = p;
    }

    printf("Loaded %d themes (1 built-in + %d from themes/)\n", reg->count, reg->user_count);
}

// ── Runtime theme add (drag-and-drop) ──────────────────────────────────────

bool theme_registry_add(theme_registry_t *reg, const char *path) {
    if (reg->user_count >= MAX_USER_THEMES || reg->count >= MAX_THEMES) {
        printf("Theme registry full, cannot load: %s\n", path);
        return false;
    }

    // Check it's a .mvt file
    int plen = (int)strlen(path);
    if (plen < 5 || strcmp(path + plen - 4, ".mvt") != 0) {
        return false;
    }

    int idx = reg->user_count;
    int priority = 0;
    if (!theme_load_mvt(path, &reg->user_storage[idx], reg->name_bufs[idx], 64, &priority)) {
        return false;
    }

    reg->user_storage[idx].name = reg->name_bufs[idx];
    if (reg->name_bufs[idx][0] == '\0') {
        // Extract filename without path and extension
        const char *fname = path;
        for (const char *p = path; *p; p++) {
            if (*p == '/' || *p == '\\') fname = p + 1;
        }
        int copy_len = (int)strlen(fname) - 4;
        if (copy_len > 63) copy_len = 63;
        if (copy_len < 0) copy_len = 0;
        memcpy(reg->name_bufs[idx], fname, copy_len);
        reg->name_bufs[idx][copy_len] = '\0';
    }

    // Insert sorted by priority (descending), after index 0 (Grid)
    int insert_pos = 1;
    while (insert_pos < reg->count && reg->priorities[insert_pos] >= priority) {
        insert_pos++;
    }

    // Shift everything after insert_pos down
    for (int i = reg->count; i > insert_pos; i--) {
        reg->themes[i] = reg->themes[i - 1];
        reg->priorities[i] = reg->priorities[i - 1];
    }

    reg->themes[insert_pos] = &reg->user_storage[idx];
    reg->priorities[insert_pos] = priority;
    reg->count++;
    reg->cyclable++;
    reg->user_count++;

    printf("Theme added: %s (priority %d)\n", reg->name_bufs[idx], priority);
    return true;
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
