#ifndef ORTHO_PANEL_H
#define ORTHO_PANEL_H

#include "raylib.h"
#include "vehicle.h"
#include "scene.h"
#include "theme.h"

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
void ortho_panel_render(ortho_panel_t *op, const vehicle_t *vehicles,
                        int vehicle_count, int selected, const theme_t *theme,
                        int corr_mode, const int *pinned, int pinned_count);
void ortho_panel_draw(const ortho_panel_t *op, int screen_h, int hud_bar_h,
                      const theme_t *theme, Font font,
                      const vehicle_t *vehicles, int vehicle_count,
                      int selected, int trail_mode,
                      int corr_mode, const int *pinned, int pinned_count,
                      bool show_axes);
void ortho_draw_fullscreen_2d(const scene_t *s, const vehicle_t *vehicles,
                              int vehicle_count, int selected, int trail_mode,
                              int corr_mode, const int *pinned, int pinned_count,
                              int screen_w, int screen_h, Font font,
                              bool show_labels);
void ortho_panel_draw_fullscreen_label(int screen_w, int screen_h, int ortho_mode,
                                       float ortho_span, const theme_t *theme,
                                       Font font, bool show_axes);
void ortho_panel_cleanup(ortho_panel_t *op);

// Draw XYZ axis gizmo at a world position, rotated by quaternion (call inside BeginMode3D).
void draw_axis_gizmo_3d(Vector3 pos, float scale, Quaternion rot);

#endif
