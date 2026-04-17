#include "raylib.h"
#include "raymath.h"
#include "scene.h"
#include "vehicle.h"
#include <android/asset_manager.h>
#include <android_native_app_glue.h>
#include <math.h>
#include <string.h>

#define TOUCH_ORBIT_SENSITIVITY  0.005f
#define TOUCH_ZOOM_SENSITIVITY   0.01f
#define TOUCH_PAN_SENSITIVITY    0.002f
#define TOUCH_MIN_ORBIT_DIST     2.0f

// Forward-declare Raylib's Android accessor (defined in rcore_android.c, not in raylib.h).
struct android_app *GetAndroidApp(void);

// Patch #version 330 → #version 300 es (+ precision mediump float for fragment shaders)
// so the original desktop shaders work on OpenGL ES 3.0 without copying them.
// Loads via AAssetManager directly to avoid recursing back into this callback.
static char *patch_shader_source(const char *fileName) {
    struct android_app *app = GetAndroidApp();
    AAsset *asset = AAssetManager_open(app->activity->assetManager, fileName, AASSET_MODE_BUFFER);
    if (!asset) return NULL;

    off_t size = AAsset_getLength(asset);
    char *src = (char *)MemAlloc((unsigned int)size + 1);
    if (!src) { AAsset_close(asset); return NULL; }
    int bytes_read = AAsset_read(asset, src, (size_t)size);
    AAsset_close(asset);
    if (bytes_read != (int)size) { MemFree(src); return NULL; }
    src[size] = '\0';

    const char *old_ver = "#version 330";
    const char *new_ver = "#version 300 es";
    char *pos = strstr(src, old_ver);
    if (!pos) return src; // not a versioned shader, return as-is

    size_t len     = strlen(fileName);
    int    is_fs   = (len >= 3 && strcmp(fileName + len - 3, ".fs") == 0);

    const char *precision = "\nprecision mediump float;";
    size_t old_len  = strlen(old_ver);
    size_t new_len  = strlen(new_ver);
    size_t prec_len = is_fs ? strlen(precision) : 0;
    size_t src_len  = (size_t)size;
    size_t out_len  = src_len - old_len + new_len + prec_len + 1;

    char  *out    = (char *)MemAlloc((unsigned int)out_len);
    if (!out) { MemFree(src); return NULL; }
    size_t prefix = (size_t)(pos - src);
    memcpy(out, src, prefix);
    memcpy(out + prefix, new_ver, new_len);
    if (is_fs) memcpy(out + prefix + new_len, precision, prec_len);
    memcpy(out + prefix + new_len + prec_len, pos + old_len, src_len - prefix - old_len + 1);

    MemFree(src);
    return out;
}

static void handle_touch(Camera3D *cam, Vector3 *orbit_target,
                          int *prev_count, Vector2 *prev_touch,
                          float *prev_pinch_dist, Vector2 *prev_mid) {
    int count = GetTouchPointCount();

    // On finger count change: reset prev values to current to avoid a jump.
    if (count != *prev_count) {
        if (count >= 1) *prev_touch = GetTouchPosition(0);
        if (count >= 2) {
            Vector2 t0 = GetTouchPosition(0);
            Vector2 t1 = GetTouchPosition(1);
            *prev_pinch_dist = Vector2Distance(t0, t1);
            *prev_mid = (Vector2){ (t0.x + t1.x) * 0.5f, (t0.y + t1.y) * 0.5f };
        }
        *prev_count = count;
        return;
    }

    if (count == 1) {
        Vector2 touch = GetTouchPosition(0);
        Vector2 delta = { touch.x - prev_touch->x, touch.y - prev_touch->y };
        *prev_touch = touch;

        Vector3 offset = Vector3Subtract(cam->position, *orbit_target);
        float r = Vector3Length(offset);
        if (r < 0.001f) return;

        // Yaw: rotate offset around world Y
        Matrix yaw = MatrixRotate((Vector3){0, 1, 0}, -delta.x * TOUCH_ORBIT_SENSITIVITY);
        offset = Vector3Transform(offset, yaw);

        // Pitch: rotate offset around camera right axis
        Vector3 forward = Vector3Normalize(Vector3Negate(offset));
        Vector3 right   = Vector3Normalize(Vector3CrossProduct(forward, (Vector3){0, 1, 0}));
        Matrix  pitch_m = MatrixRotate(right, -delta.y * TOUCH_ORBIT_SENSITIVITY);
        offset = Vector3Transform(offset, pitch_m);

        // Clamp elevation to ±89° to prevent gimbal flip
        float len = Vector3Length(offset);
        if (len < 0.001f) return;
        float elevation = asinf(Clamp(offset.y / len, -1.0f, 1.0f));
        float max_elev  = 89.0f * DEG2RAD;
        if (elevation > max_elev || elevation < -max_elev) {
            float azimuth = atan2f(offset.x, offset.z);
            elevation = Clamp(elevation, -max_elev, max_elev);
            offset.x  = len * cosf(elevation) * sinf(azimuth);
            offset.y  = len * sinf(elevation);
            offset.z  = len * cosf(elevation) * cosf(azimuth);
        }

        cam->position = Vector3Add(*orbit_target, Vector3Scale(Vector3Normalize(offset), r));
        cam->target   = *orbit_target;
        cam->up       = (Vector3){0, 1, 0};
    } else if (count == 2) {
        cam->up = (Vector3){0, 1, 0};

        Vector2 t0  = GetTouchPosition(0);
        Vector2 t1  = GetTouchPosition(1);
        Vector2 mid = { (t0.x + t1.x) * 0.5f, (t0.y + t1.y) * 0.5f };
        float   dist = Vector2Distance(t0, t1);

        // Zoom: scale orbit radius proportionally so speed feels consistent at any distance
        float   dist_delta = dist - *prev_pinch_dist;
        Vector3 offset     = Vector3Subtract(cam->position, *orbit_target);
        float   r          = Vector3Length(offset);
        r *= (1.0f - dist_delta * TOUCH_ZOOM_SENSITIVITY);
        if (r < TOUCH_MIN_ORBIT_DIST) r = TOUCH_MIN_ORBIT_DIST;
        if (r > 0.001f)
            cam->position = Vector3Add(*orbit_target,
                                       Vector3Scale(Vector3Normalize(offset), r));

        // Pan: translate orbit_target along camera right and up
        Vector2 mid_delta = { mid.x - prev_mid->x, mid.y - prev_mid->y };
        float   pan_sens  = r * TOUCH_PAN_SENSITIVITY;
        Vector3 forward   = Vector3Normalize(Vector3Subtract(cam->target, cam->position));
        Vector3 right     = Vector3Normalize(Vector3CrossProduct(forward, cam->up));
        Vector3 pan       = Vector3Add(
            Vector3Scale(right,    -mid_delta.x * pan_sens),
            Vector3Scale(cam->up,   mid_delta.y * pan_sens)
        );
        *orbit_target = Vector3Add(*orbit_target, pan);
        cam->position = Vector3Add(cam->position, pan);
        cam->target   = *orbit_target;

        *prev_pinch_dist = dist;
        *prev_mid        = mid;
        *prev_touch      = GetTouchPosition(0);
    }
}

// Raylib's rcore_android.c owns android_main and calls user main() after platform setup.
int main(int argc, char *argv[]) {
    (void)argc; (void)argv;
    // 0, 0 = use the device's native screen resolution (fullscreen)
    InitWindow(0, 0, "Hawkeye");
    // Register after InitWindow so GetAndroidApp() is valid (platform is initialized).
    SetLoadFileTextCallback(patch_shader_source);
    SetTargetFPS(60);

    SetGesturesEnabled(GESTURE_DRAG | GESTURE_PINCH_IN | GESTURE_PINCH_OUT);

    scene_t scene = {0};
    scene_init(&scene);

    vehicle_t vehicle = {0};
    vehicle_init(&vehicle, MODEL_QUADROTOR, scene.lighting_shader);

    // Position vehicle slightly above origin so it's visible with the default camera
    vehicle.position = (Vector3){ 0.0f, 0.5f, 0.0f };

    scene.cam_mode   = CAM_MODE_FREE;
    scene.free_track = true;

    Vector3 orbit_target    = vehicle.position;
    Vector2 prev_touch      = {0};
    float   prev_pinch_dist = 0.0f;
    Vector2 prev_mid        = {0};
    int     prev_count      = 0;

    scene.camera.target = orbit_target;

    while (!WindowShouldClose()) {
        handle_touch(&scene.camera, &orbit_target,
                     &prev_count, &prev_touch,
                     &prev_pinch_dist, &prev_mid);
        BeginDrawing();
            scene_draw_sky(&scene);
            BeginMode3D(scene.camera);
                scene_draw(&scene);
                vehicle_draw(&vehicle, scene.theme,
                             /*selected=*/true,
                             /*trail_mode=*/0,
                             /*show_ground_track=*/false,
                             /*cam_pos=*/scene.camera.position,
                             /*classic_colors=*/false);
            EndMode3D();
        EndDrawing();
    }

    vehicle_cleanup(&vehicle);
    scene_cleanup(&scene);
    SetLoadFileTextCallback(NULL);
    CloseWindow();
    return 0;
}
