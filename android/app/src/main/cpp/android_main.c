#include "raylib.h"
#include "scene.h"
#include "vehicle.h"
#include <android/asset_manager.h>
#include <android_native_app_glue.h>
#include <stdlib.h>
#include <string.h>

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
    char *src = (char *)malloc((size_t)size + 1);
    if (!src) { AAsset_close(asset); return NULL; }
    int bytes_read = AAsset_read(asset, src, (size_t)size);
    AAsset_close(asset);
    if (bytes_read != (int)size) { free(src); return NULL; }
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

    char  *out    = (char *)malloc(out_len);
    if (!out) { free(src); return NULL; }
    size_t prefix = (size_t)(pos - src);
    memcpy(out, src, prefix);
    memcpy(out + prefix, new_ver, new_len);
    if (is_fs) memcpy(out + prefix + new_len, precision, prec_len);
    memcpy(out + prefix + new_len + prec_len, pos + old_len, src_len - prefix - old_len + 1);

    free(src);
    return out;
}

// Raylib's rcore_android.c owns android_main and calls user main() after platform setup.
int main(int argc, char *argv[]) {
    (void)argc; (void)argv;
    // 0, 0 = use the device's native screen resolution (fullscreen)
    InitWindow(0, 0, "Hawkeye");
    // Register after InitWindow so GetAndroidApp() is valid (platform is initialized).
    SetLoadFileTextCallback(patch_shader_source);
    SetTargetFPS(60);

    scene_t scene = {0};
    scene_init(&scene);

    vehicle_t vehicle = {0};
    vehicle_init(&vehicle, MODEL_QUADROTOR, scene.lighting_shader);

    // Position vehicle slightly above origin so it's visible with the default camera
    vehicle.position = (Vector3){ 0.0f, 0.5f, 0.0f };

    while (!WindowShouldClose()) {
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
