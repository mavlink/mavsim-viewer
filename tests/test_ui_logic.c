/*
 * Unit tests for pure math/logic functions extracted into ui_logic.h.
 * No Raylib dependency -- compiles standalone with only libc.
 */
#include "ui_logic.h"
#include <assert.h>
#include <stdio.h>

/* ── apply_vehicle_selection tests ────────────────────────────────────────── */

static void init_pins(int pinned[HUD_MAX_PINNED], int *pinned_count)
{
    *pinned_count = 0;
    memset(pinned, -1, sizeof(int) * HUD_MAX_PINNED);
}

static void test_select_updates_selected(void)
{
    int pinned[HUD_MAX_PINNED];
    int pinned_count;
    int selected = 0;
    init_pins(pinned, &pinned_count);

    apply_vehicle_selection(pinned, &pinned_count, 3, false, &selected, 8);
    assert(selected == 3);
    assert(pinned_count == 0);
    printf("  PASS select_updates_selected\n");
}

static void test_select_clears_pins(void)
{
    int pinned[HUD_MAX_PINNED];
    int pinned_count;
    int selected = 0;
    init_pins(pinned, &pinned_count);

    /* Pin vehicles 1 and 2 first */
    apply_vehicle_selection(pinned, &pinned_count, 1, true, &selected, 8);
    apply_vehicle_selection(pinned, &pinned_count, 2, true, &selected, 8);
    assert(pinned_count == 2);

    /* Now select vehicle 5 -- pins should be cleared */
    apply_vehicle_selection(pinned, &pinned_count, 5, false, &selected, 8);
    assert(selected == 5);
    assert(pinned_count == 0);
    for (int i = 0; i < HUD_MAX_PINNED; i++)
        assert(pinned[i] == -1);
    printf("  PASS select_clears_pins\n");
}

static void test_pin_adds_to_array(void)
{
    int pinned[HUD_MAX_PINNED];
    int pinned_count;
    int selected = 0;
    init_pins(pinned, &pinned_count);

    apply_vehicle_selection(pinned, &pinned_count, 2, true, &selected, 8);
    assert(pinned_count == 1);
    assert(pinned[0] == 2);

    apply_vehicle_selection(pinned, &pinned_count, 4, true, &selected, 8);
    assert(pinned_count == 2);
    assert(pinned[0] == 2);
    assert(pinned[1] == 4);
    printf("  PASS pin_adds_to_array\n");
}

static void test_unpin_already_pinned(void)
{
    int pinned[HUD_MAX_PINNED];
    int pinned_count;
    int selected = 0;
    init_pins(pinned, &pinned_count);

    /* Pin 1, 3, 5 */
    apply_vehicle_selection(pinned, &pinned_count, 1, true, &selected, 8);
    apply_vehicle_selection(pinned, &pinned_count, 3, true, &selected, 8);
    apply_vehicle_selection(pinned, &pinned_count, 5, true, &selected, 8);
    assert(pinned_count == 3);

    /* Unpin 3 (middle element) */
    apply_vehicle_selection(pinned, &pinned_count, 3, true, &selected, 8);
    assert(pinned_count == 2);
    assert(pinned[0] == 1);
    assert(pinned[1] == 5);
    assert(pinned[2] == -1);
    printf("  PASS unpin_already_pinned\n");
}

static void test_cannot_pin_selected(void)
{
    int pinned[HUD_MAX_PINNED];
    int pinned_count;
    int selected = 3;
    init_pins(pinned, &pinned_count);

    /* Try to pin the selected vehicle -- should be ignored */
    apply_vehicle_selection(pinned, &pinned_count, 3, true, &selected, 8);
    assert(pinned_count == 0);
    printf("  PASS cannot_pin_selected\n");
}

static void test_cannot_exceed_max_pinned(void)
{
    int pinned[HUD_MAX_PINNED];
    int pinned_count;
    int selected = 0;
    init_pins(pinned, &pinned_count);

    /* Fill all pin slots (vehicle_count must be large enough) */
    int total_vehicles = HUD_MAX_PINNED + 5;
    for (int i = 1; i <= HUD_MAX_PINNED; i++)
        apply_vehicle_selection(pinned, &pinned_count, i, true, &selected, total_vehicles);
    assert(pinned_count == HUD_MAX_PINNED);

    /* Try to pin one more -- should be rejected */
    apply_vehicle_selection(pinned, &pinned_count, HUD_MAX_PINNED + 1, true, &selected, total_vehicles);
    assert(pinned_count == HUD_MAX_PINNED);
    printf("  PASS cannot_exceed_max_pinned\n");
}

static void test_pin_limited_by_vehicle_count(void)
{
    int pinned[HUD_MAX_PINNED];
    int pinned_count;
    int selected = 0;
    init_pins(pinned, &pinned_count);

    /* With 3 vehicles (selected + 2 others), max pins = vehicle_count - 1 = 2 */
    apply_vehicle_selection(pinned, &pinned_count, 1, true, &selected, 3);
    apply_vehicle_selection(pinned, &pinned_count, 2, true, &selected, 3);
    assert(pinned_count == 2);

    /* Third pin exceeds vehicle_count - 1, should be rejected even though < HUD_MAX_PINNED */
    apply_vehicle_selection(pinned, &pinned_count, 3, true, &selected, 3);
    assert(pinned_count == 2);
    printf("  PASS pin_limited_by_vehicle_count\n");
}

/* ── compute_screen_direction tests ──────────────────────────────────────── */

static void test_drone_directly_right(void)
{
    /* Camera at origin looking along -Z (standard OpenGL), drone to the right (+X) and behind */
    ui_vec3_t cam   = {0, 0, 0};
    ui_vec3_t fwd   = {0, 0, -1};
    ui_vec3_t drone = {10, 0, 5};  /* behind and to the right */
    float dx, dy;
    compute_screen_direction(drone, cam, fwd, &dx, &dy);
    /* dx should be positive (right on screen), dy near zero */
    assert(dx > 0.5f);
    assert(fabsf(dy) < 0.3f);
    printf("  PASS drone_directly_right\n");
}

static void test_drone_directly_left(void)
{
    ui_vec3_t cam   = {0, 0, 0};
    ui_vec3_t fwd   = {0, 0, -1};
    ui_vec3_t drone = {-10, 0, 5};
    float dx, dy;
    compute_screen_direction(drone, cam, fwd, &dx, &dy);
    assert(dx < -0.5f);
    printf("  PASS drone_directly_left\n");
}

static void test_drone_above(void)
{
    /* Drone above and behind -- dy should be negative (up on screen in screen coords) */
    ui_vec3_t cam   = {0, 0, 0};
    ui_vec3_t fwd   = {0, 0, -1};
    ui_vec3_t drone = {0, 10, 5};
    float dx, dy;
    compute_screen_direction(drone, cam, fwd, &dx, &dy);
    assert(dy < -0.5f);
    assert(fabsf(dx) < 0.3f);
    printf("  PASS drone_above\n");
}

static void test_drone_below(void)
{
    ui_vec3_t cam   = {0, 0, 0};
    ui_vec3_t fwd   = {0, 0, -1};
    ui_vec3_t drone = {0, -10, 5};
    float dx, dy;
    compute_screen_direction(drone, cam, fwd, &dx, &dy);
    assert(dy > 0.5f);
    printf("  PASS drone_below\n");
}

static void test_direction_is_normalised(void)
{
    ui_vec3_t cam   = {0, 0, 0};
    ui_vec3_t fwd   = {0, 0, -1};
    ui_vec3_t drone = {100, 50, 200};
    float dx, dy;
    compute_screen_direction(drone, cam, fwd, &dx, &dy);
    float len = sqrtf(dx * dx + dy * dy);
    assert(fabsf(len - 1.0f) < 0.01f);
    printf("  PASS direction_is_normalised\n");
}

static void test_camera_looking_along_x(void)
{
    /* Camera looking along +X: drone behind at -X and to the right (+Z) */
    ui_vec3_t cam   = {0, 0, 0};
    ui_vec3_t fwd   = {1, 0, 0};
    ui_vec3_t drone = {-5, 0, -10};  /* behind and to the right (cam_right is -Z) */
    float dx, dy;
    compute_screen_direction(drone, cam, fwd, &dx, &dy);
    float len = sqrtf(dx * dx + dy * dy);
    assert(fabsf(len - 1.0f) < 0.01f);
    printf("  PASS camera_looking_along_x\n");
}

/* ── main ─────────────────────────────────────────────────────────────────── */

int main(void)
{
    printf("apply_vehicle_selection:\n");
    test_select_updates_selected();
    test_select_clears_pins();
    test_pin_adds_to_array();
    test_unpin_already_pinned();
    test_cannot_pin_selected();
    test_cannot_exceed_max_pinned();
    test_pin_limited_by_vehicle_count();

    printf("compute_screen_direction:\n");
    test_drone_directly_right();
    test_drone_directly_left();
    test_drone_above();
    test_drone_below();
    test_direction_is_normalised();
    test_camera_looking_along_x();

    printf("All ui_logic tests passed.\n");
    return 0;
}
