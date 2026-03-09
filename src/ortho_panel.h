#ifndef ORTHO_PANEL_H
#define ORTHO_PANEL_H

#include "raylib.h"
#include "vehicle.h"
#include "scene.h"

#define ORTHO_VIEW_COUNT 3
#define ORTHO_TEX_SIZE 512   // max render texture size

typedef struct {
    RenderTexture2D targets[ORTHO_VIEW_COUNT]; // top, front, side
    Camera3D cameras[ORTHO_VIEW_COUNT];
    float ortho_span;   // world units visible in each view
    bool visible;
} ortho_panel_t;

void ortho_panel_init(ortho_panel_t *op);
void ortho_panel_update(ortho_panel_t *op, Vector3 vehicle_pos);
void ortho_panel_render(ortho_panel_t *op, const scene_t *s,
                        const vehicle_t *vehicles, int vehicle_count,
                        int selected, view_mode_t view_mode,
                        int trail_mode);
void ortho_panel_draw(const ortho_panel_t *op, int screen_h, int hud_bar_h,
                      view_mode_t view_mode, Font font);
void ortho_panel_draw_fullscreen_label(int screen_w, int screen_h, int ortho_mode,
                                       float ortho_span, int view_mode, Font font);
void ortho_panel_cleanup(ortho_panel_t *op);

#endif
