/*
 * Unit tests for theme .mvt parser and trail LOD skip logic.
 *
 * Theme tests exercise theme_load_mvt() and theme_heat_color() from src/theme.c.
 * Trail LOD tests re-declare the static trail_lod_skip() constants and function
 * (it is a small, self-contained helper) to avoid pulling in all of vehicle.c.
 */

#include "theme.h"
#include "asset_path.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MVT_VALID     FIXTURES_DIR "/test_theme.mvt"
#define MVT_PARTIAL   FIXTURES_DIR "/test_theme_partial.mvt"
#define MVT_MALFORMED FIXTURES_DIR "/test_theme_malformed.mvt"

/* ── Helpers ────────────────────────────────────────────────────────────── */

static int color_eq(Color a, Color b)
{
    return a.r == b.r && a.g == b.g && a.b == b.b && a.a == b.a;
}

/* ── Theme parser tests ─────────────────────────────────────────────────── */

static void test_load_valid(void)
{
    theme_t t;
    char name[64] = {0};
    int priority = -1;
    bool ok = theme_load_mvt(MVT_VALID, &t, name, sizeof(name), &priority);
    assert(ok);
    assert(strcmp(name, "TestTheme") == 0);
    assert(priority == 42);
    assert(t.thick_trails == true);

    /* Check explicitly set colors */
    assert(color_eq(t.sky,          (Color){10, 20, 30, 255}));
    assert(color_eq(t.ground,       (Color){40, 50, 60, 128}));
    assert(color_eq(t.hud_accent,   (Color){100, 200, 150, 255}));
    assert(color_eq(t.trail_forward,(Color){255, 128, 64, 200}));

    printf("  PASS load_valid\n");
}

static void test_rgba_clamping(void)
{
    theme_t t;
    char name[64] = {0};
    bool ok = theme_load_mvt(MVT_VALID, &t, name, sizeof(name), NULL);
    assert(ok);

    /* debug accent was "300 -10 128 500" — must be clamped to 0-255 */
    assert(t.dbg_accent.r == 255);  /* 300 clamped to 255 */
    assert(t.dbg_accent.g == 0);    /* -10 clamped to 0   */
    assert(t.dbg_accent.b == 128);
    assert(t.dbg_accent.a == 255);  /* 500 clamped to 255 */

    printf("  PASS rgba_clamping\n");
}

static void test_partial_defaults(void)
{
    theme_t t;
    char name[64] = {0};
    int priority = -1;
    bool ok = theme_load_mvt(MVT_PARTIAL, &t, name, sizeof(name), &priority);
    assert(ok);
    assert(strcmp(name, "Partial") == 0);
    assert(priority == 0);  /* default priority */

    /* sky was overridden */
    assert(color_eq(t.sky, (Color){1, 2, 3, 255}));

    /* ground should be Grid default */
    assert(color_eq(t.ground, theme_grid.ground));

    /* hud_accent should be Grid default */
    assert(color_eq(t.hud_accent, theme_grid.hud_accent));

    /* thick_trails should be Grid default (false) */
    assert(t.thick_trails == theme_grid.thick_trails);

    printf("  PASS partial_defaults\n");
}

static void test_malformed_lines(void)
{
    theme_t t;
    char name[64] = {0};
    bool ok = theme_load_mvt(MVT_MALFORMED, &t, name, sizeof(name), NULL);
    assert(ok);
    assert(strcmp(name, "Malformed") == 0);

    /* "sky: not a color value" — sscanf fails, sky stays at Grid default */
    assert(color_eq(t.sky, theme_grid.sky));

    /* "ground: 10 20" — only 2 values, sscanf expects 4, stays default */
    assert(color_eq(t.ground, theme_grid.ground));

    /* "grid_minor: 99 88 77 66" — valid color, should parse despite other bad lines */
    assert(color_eq(t.grid_minor, (Color){99, 88, 77, 66}));

    printf("  PASS malformed_lines\n");
}

static void test_load_nonexistent(void)
{
    theme_t t;
    char name[64] = {0};
    bool ok = theme_load_mvt("/nonexistent/path.mvt", &t, name, sizeof(name), NULL);
    assert(!ok);
    printf("  PASS load_nonexistent\n");
}

static void test_heat_color_endpoints(void)
{
    /* Use Grid theme thermal gradient for predictable values */
    Color c0 = theme_heat_color(&theme_grid, 0.0f, 200);
    assert(c0.r == theme_grid.thermal[0].r);
    assert(c0.g == theme_grid.thermal[0].g);
    assert(c0.b == theme_grid.thermal[0].b);
    assert(c0.a == 200);

    Color c1 = theme_heat_color(&theme_grid, 1.0f, 180);
    assert(c1.r == theme_grid.thermal[6].r);
    assert(c1.g == theme_grid.thermal[6].g);
    assert(c1.b == theme_grid.thermal[6].b);
    assert(c1.a == 180);

    printf("  PASS heat_color_endpoints\n");
}

static void test_heat_color_midpoint(void)
{
    /* heat=0.5 maps to scaled=3.0, idx=3, frac=0.0 — exactly thermal[3] */
    Color c = theme_heat_color(&theme_grid, 0.5f, 255);
    assert(c.r == theme_grid.thermal[3].r);
    assert(c.g == theme_grid.thermal[3].g);
    assert(c.b == theme_grid.thermal[3].b);
    assert(c.a == 255);

    printf("  PASS heat_color_midpoint\n");
}

static void test_heat_color_clamps_input(void)
{
    /* Negative heat should clamp to 0.0 */
    Color cn = theme_heat_color(&theme_grid, -1.0f, 255);
    Color c0 = theme_heat_color(&theme_grid, 0.0f, 255);
    assert(color_eq(cn, c0));

    /* Heat > 1.0 should clamp to 1.0 */
    Color cp = theme_heat_color(&theme_grid, 2.0f, 255);
    Color c1 = theme_heat_color(&theme_grid, 1.0f, 255);
    assert(color_eq(cp, c1));

    printf("  PASS heat_color_clamps_input\n");
}

/* ── Trail LOD skip tests ───────────────────────────────────────────────── */
/*
 * Re-declare the constants and function from vehicle.c to test in isolation.
 * This avoids linking vehicle.c (which depends on raylib rendering functions).
 */

#define TRAIL_LOD_DIST_FAR_SQ  2250000.0f  /* 1500 m squared */
#define TRAIL_LOD_DIST_MED_SQ   160000.0f  /*  400 m squared */
#define TRAIL_LOD_SKIP_FAR  3
#define TRAIL_LOD_SKIP_MED  1

static int trail_lod_skip(Vector3 a, Vector3 b, Vector3 cam)
{
    float dx = (a.x + b.x) * 0.5f - cam.x;
    float dy = (a.y + b.y) * 0.5f - cam.y;
    float dz = (a.z + b.z) * 0.5f - cam.z;
    float dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq > TRAIL_LOD_DIST_FAR_SQ) return TRAIL_LOD_SKIP_FAR;
    if (dist_sq > TRAIL_LOD_DIST_MED_SQ) return TRAIL_LOD_SKIP_MED;
    return 0;
}

static void test_trail_lod_near(void)
{
    /* Midpoint at 100m from camera — should return 0 (no skip) */
    Vector3 a = {100.0f, 0.0f, 0.0f};
    Vector3 b = {100.0f, 0.0f, 0.0f};
    Vector3 cam = {0.0f, 0.0f, 0.0f};
    int skip = trail_lod_skip(a, b, cam);
    assert(skip == 0);

    /* Right at the boundary: 399m — still near */
    Vector3 a2 = {399.0f, 0.0f, 0.0f};
    Vector3 b2 = {399.0f, 0.0f, 0.0f};
    skip = trail_lod_skip(a2, b2, cam);
    assert(skip == 0);

    printf("  PASS trail_lod_near\n");
}

static void test_trail_lod_medium(void)
{
    /* Midpoint at 500m from camera — medium distance */
    Vector3 a = {500.0f, 0.0f, 0.0f};
    Vector3 b = {500.0f, 0.0f, 0.0f};
    Vector3 cam = {0.0f, 0.0f, 0.0f};
    int skip = trail_lod_skip(a, b, cam);
    assert(skip == TRAIL_LOD_SKIP_MED);

    /* Just over boundary: 401m */
    Vector3 a2 = {401.0f, 0.0f, 0.0f};
    Vector3 b2 = {401.0f, 0.0f, 0.0f};
    skip = trail_lod_skip(a2, b2, cam);
    assert(skip == TRAIL_LOD_SKIP_MED);

    printf("  PASS trail_lod_medium\n");
}

static void test_trail_lod_far(void)
{
    /* Midpoint at 2000m from camera — far distance */
    Vector3 a = {2000.0f, 0.0f, 0.0f};
    Vector3 b = {2000.0f, 0.0f, 0.0f};
    Vector3 cam = {0.0f, 0.0f, 0.0f};
    int skip = trail_lod_skip(a, b, cam);
    assert(skip == TRAIL_LOD_SKIP_FAR);

    /* Just over boundary: 1501m */
    Vector3 a2 = {1501.0f, 0.0f, 0.0f};
    Vector3 b2 = {1501.0f, 0.0f, 0.0f};
    skip = trail_lod_skip(a2, b2, cam);
    assert(skip == TRAIL_LOD_SKIP_FAR);

    printf("  PASS trail_lod_far\n");
}

static void test_trail_lod_midpoint_averaging(void)
{
    /* a at 0, b at 600 — midpoint at 300m, should be near */
    Vector3 a = {0.0f, 0.0f, 0.0f};
    Vector3 b = {600.0f, 0.0f, 0.0f};
    Vector3 cam = {0.0f, 0.0f, 0.0f};
    int skip = trail_lod_skip(a, b, cam);
    assert(skip == 0);

    /* a at 0, b at 1000 — midpoint at 500m, should be medium */
    Vector3 b2 = {1000.0f, 0.0f, 0.0f};
    skip = trail_lod_skip(a, b2, cam);
    assert(skip == TRAIL_LOD_SKIP_MED);

    printf("  PASS trail_lod_midpoint_averaging\n");
}

/* ── Main ───────────────────────────────────────────────────────────────── */

int main(void)
{
    asset_path_init();
    printf("test_theme:\n");

    /* Theme parser */
    test_load_valid();
    test_rgba_clamping();
    test_partial_defaults();
    test_malformed_lines();
    test_load_nonexistent();

    /* Thermal gradient */
    test_heat_color_endpoints();
    test_heat_color_midpoint();
    test_heat_color_clamps_input();

    /* Trail LOD skip */
    test_trail_lod_near();
    test_trail_lod_medium();
    test_trail_lod_far();
    test_trail_lod_midpoint_averaging();

    printf("All 12 tests passed.\n");
    return 0;
}
