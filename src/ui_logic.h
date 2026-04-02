/*
 * ui_logic.h -- Pure math/logic helpers extracted for unit testing.
 *
 * These functions have zero Raylib dependency and can be compiled standalone
 * with only <math.h>, <stdbool.h>, <string.h>.
 */
#ifndef UI_LOGIC_H
#define UI_LOGIC_H

#include <math.h>
#include <stdbool.h>
#include <string.h>

#ifndef HUD_MAX_PINNED
#define HUD_MAX_PINNED 15
#endif

/* ── Vehicle selection / pin logic ────────────────────────────────────────── */

/*
 * Apply a vehicle select or pin action on the HUD.
 *
 *   pinned[]      – array of HUD_MAX_PINNED ints, -1 = empty slot
 *   pinned_count  – pointer to current count of pinned vehicles
 *   idx           – vehicle index to select or pin
 *   pin           – false = select (clears pins), true = toggle pin
 *   selected      – pointer to the currently selected vehicle index
 *   vehicle_count – total number of vehicles (limits max pins)
 */
static inline void apply_vehicle_selection(int pinned[HUD_MAX_PINNED],
                                           int *pinned_count,
                                           int idx, bool pin,
                                           int *selected, int vehicle_count)
{
    if (pin) {
        if (idx != *selected) {
            int found = -1;
            for (int p = 0; p < *pinned_count; p++)
                if (pinned[p] == idx) { found = p; break; }
            if (found >= 0) {
                for (int p = found; p < *pinned_count - 1; p++)
                    pinned[p] = pinned[p + 1];
                (*pinned_count)--;
                pinned[*pinned_count] = -1;
            } else if (*pinned_count < HUD_MAX_PINNED &&
                       *pinned_count < vehicle_count - 1) {
                pinned[(*pinned_count)++] = idx;
            }
        }
    } else {
        *selected = idx;
        *pinned_count = 0;
        memset(pinned, -1, sizeof(int) * HUD_MAX_PINNED);
    }
}

/* ── Edge-indicator behind-camera direction math ─────────────────────────── */

typedef struct { float x, y, z; } ui_vec3_t;

/*
 * Compute the 2-D screen-space direction from camera to a drone when the
 * drone is behind (or beside) the camera and the regular screen-projection
 * is unreliable.
 *
 * Inputs:
 *   drone_pos – world position of the drone
 *   cam_pos   – world position of the camera
 *   cam_fwd   – unit forward vector of the camera
 *
 * Outputs:
 *   out_dx, out_dy – normalised screen-space direction (right / down)
 *
 * The caller is responsible for scaling/clamping the result.
 */
static inline void compute_screen_direction(ui_vec3_t drone_pos,
                                            ui_vec3_t cam_pos,
                                            ui_vec3_t cam_fwd,
                                            float *out_dx, float *out_dy)
{
    /* to_drone vector */
    float tx = drone_pos.x - cam_pos.x;
    float ty = drone_pos.y - cam_pos.y;
    float tz = drone_pos.z - cam_pos.z;

    /* cam_right = normalize(cam_fwd x world_up)  where world_up = {0,1,0}
     * cross = {fwd.y*0 - fwd.z*1, fwd.z*0 - fwd.x*0, fwd.x*1 - fwd.y*0}
     *       = {-fwd.z, 0, fwd.x} */
    float rx = -cam_fwd.z;
    float ry = 0.0f;
    float rz = cam_fwd.x;
    float rlen = sqrtf(rx * rx + ry * ry + rz * rz);
    if (rlen > 1e-6f) { rx /= rlen; ry /= rlen; rz /= rlen; }

    /* cam_up_approx = cam_right x cam_fwd */
    float ux = ry * cam_fwd.z - rz * cam_fwd.y;
    float uy = rz * cam_fwd.x - rx * cam_fwd.z;
    float uz = rx * cam_fwd.y - ry * cam_fwd.x;

    /* Project to_drone onto cam_right and cam_up_approx */
    float dx = tx * rx + ty * ry + tz * rz;           /* dot(to_drone, cam_right) */
    float dy = -(tx * ux + ty * uy + tz * uz);        /* -dot(to_drone, cam_up)   */

    float len = sqrtf(dx * dx + dy * dy);
    if (len > 0.01f) { dx /= len; dy /= len; }

    *out_dx = dx;
    *out_dy = dy;
}

#endif /* UI_LOGIC_H */
