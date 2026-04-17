# Touch Controls Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add 1-finger orbit, 2-finger pinch-zoom, and 2-finger pan to the Android renderer via a self-contained `handle_touch()` function in `android_main.c`.

**Architecture:** All touch logic lives in a new `static void handle_touch()` function in `android_main.c`, called once per frame before `BeginDrawing()`. It directly mutates `scene.camera` and a local `orbit_target` vector. `scene_update_camera()` and `scene_handle_input()` are not called on Android. No other files are modified.

**Tech Stack:** Raylib 5.5 gesture API (`GetTouchCount`, `GetTouchPosition`, `Vector2Distance`), raymath (matrix rotation, vector ops), C99 math (`asinf`, `atan2f`, `cosf`, `sinf`).

---

## File Map

| File | Change |
|---|---|
| `app/src/main/cpp/android_main.c` | Add includes, constants, `handle_touch()`, initialization, loop call |

No new files. No CMake changes.

---

## Note on Testing

This is an Android NDK native project with no C unit test framework. "Test" steps are: build verification (`./gradlew assembleDebug`) followed by manual device verification (`./gradlew installDebug`). Each task ends with both.

---

## Task 1: Add includes, constants, and initialize gesture + camera state

**Files:**
- Modify: `app/src/main/cpp/android_main.c`

- [ ] **Step 1: Add `raymath.h` and `math.h` includes**

In `android_main.c`, replace the current include block:

```c
#include "raylib.h"
#include "scene.h"
#include "vehicle.h"
#include <android/asset_manager.h>
#include <android_native_app_glue.h>
#include <string.h>
```

with:

```c
#include "raylib.h"
#include "raymath.h"
#include "scene.h"
#include "vehicle.h"
#include <android/asset_manager.h>
#include <android_native_app_glue.h>
#include <math.h>
#include <string.h>
```

- [ ] **Step 2: Add touch sensitivity constants**

After the include block (before the `GetAndroidApp` forward declaration), add:

```c
#define TOUCH_ORBIT_SENSITIVITY  0.005f
#define TOUCH_ZOOM_SENSITIVITY   0.01f
#define TOUCH_PAN_SENSITIVITY    0.002f
#define TOUCH_MIN_ORBIT_DIST     2.0f
```

- [ ] **Step 3: Enable gestures and set initial camera state in `main()`**

After `SetTargetFPS(60);` and before `scene_t scene = {0};`, add:

```c
SetGesturesEnabled(GESTURE_DRAG | GESTURE_PINCH_IN | GESTURE_PINCH_OUT);
```

After `vehicle.position = (Vector3){ 0.0f, 0.5f, 0.0f };`, add:

```c
scene.cam_mode   = CAM_MODE_FREE;
scene.free_track = true;

Vector3 orbit_target    = vehicle.position;
Vector2 prev_touch      = {0};
float   prev_pinch_dist = 0.0f;
Vector2 prev_mid        = {0};
int     prev_count      = 0;

scene.camera.target = orbit_target;
```

- [ ] **Step 4: Build to verify it compiles**

```bash
cd /Users/diegomedina/Documents/Projects/Hawkeye/android
./gradlew assembleDebug
```

Expected: `BUILD SUCCESSFUL` (only third-party warnings, no new errors).

- [ ] **Step 5: Commit**

```bash
git add app/src/main/cpp/android_main.c
git commit -m "feat(android): enable touch gestures and set orbit camera state"
```

---

## Task 2: Implement `handle_touch()` with 1-finger orbit

**Files:**
- Modify: `app/src/main/cpp/android_main.c`

- [ ] **Step 1: Add `handle_touch()` before `main()`**

Insert the following function between `patch_shader_source()` and `main()`:

```c
static void handle_touch(Camera3D *cam, Vector3 *orbit_target,
                          int *prev_count, Vector2 *prev_touch,
                          float *prev_pinch_dist, Vector2 *prev_mid) {
    int count = GetTouchCount();

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
    }
}
```

- [ ] **Step 2: Call `handle_touch()` in the render loop**

In `main()`, add the call immediately before `BeginDrawing()`:

```c
while (!WindowShouldClose()) {
    handle_touch(&scene.camera, &orbit_target,
                 &prev_count, &prev_touch,
                 &prev_pinch_dist, &prev_mid);
    BeginDrawing();
```

- [ ] **Step 3: Build**

```bash
./gradlew assembleDebug
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 4: Deploy and test 1-finger orbit**

```bash
./gradlew installDebug
```

Launch the app. Place one finger on screen and drag:
- Dragging left/right should rotate the camera around the vehicle horizontally.
- Dragging up/down should change the camera elevation.
- Camera should not flip upside-down near vertical extremes.

- [ ] **Step 5: Commit**

```bash
git add app/src/main/cpp/android_main.c
git commit -m "feat(android): add 1-finger orbit touch control"
```

---

## Task 3: Add 2-finger zoom and pan to `handle_touch()`

**Files:**
- Modify: `app/src/main/cpp/android_main.c`

- [ ] **Step 1: Fill in the `count == 2` branch**

Inside `handle_touch()`, find the line that reads `cam->up = (Vector3){0, 1, 0};` followed by the closing `}` of the `if (count == 1)` block. Replace that closing `}` with the following (turning the standalone `if` into an `if/else if`):

```c
        cam->up       = (Vector3){0, 1, 0};
    } else if (count == 2) {
        Vector2 t0  = GetTouchPosition(0);
        Vector2 t1  = GetTouchPosition(1);
        Vector2 mid = { (t0.x + t1.x) * 0.5f, (t0.y + t1.y) * 0.5f };
        float   dist = Vector2Distance(t0, t1);

        // Zoom: scale orbit radius by pinch distance change
        float   dist_delta = dist - *prev_pinch_dist;
        Vector3 offset     = Vector3Subtract(cam->position, *orbit_target);
        float   r          = Vector3Length(offset);
        r -= dist_delta * TOUCH_ZOOM_SENSITIVITY;
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
    }
```

- [ ] **Step 2: Build**

```bash
./gradlew assembleDebug
```

Expected: `BUILD SUCCESSFUL`.

- [ ] **Step 3: Deploy and test 2-finger gestures**

```bash
./gradlew installDebug
```

Launch the app. Test each gesture:

- **Pinch in** (bring fingers together): camera should zoom out (orbit radius increases).
- **Pinch out** (spread fingers): camera should zoom in (orbit radius decreases). Stops at ~2m from vehicle.
- **2-finger drag** (move both fingers together in same direction): scene should pan — vehicle moves away from screen center in the drag direction.
- **Transition 1→2 fingers**: no jump on the frame the second finger lands.
- **Transition 2→1 finger**: no jump on the frame a finger lifts.

- [ ] **Step 4: Commit**

```bash
git add app/src/main/cpp/android_main.c
git commit -m "feat(android): add 2-finger pinch-zoom and pan touch controls"
```
