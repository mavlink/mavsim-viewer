# Touch Controls Design — Android

**Date:** 2026-04-16
**Branch:** android_integration/as_project_definition

## Summary

Add 1-finger orbit, 2-finger pinch-zoom, and 2-finger pan to the Android renderer. All logic is self-contained in `android_main.c`; `scene.c` and `scene.h` are untouched.

## Architecture

Touch handling lives in a new `handle_touch()` function called once per frame before `BeginDrawing()`. It directly mutates `scene.camera` and the local `orbit_target` vector. `scene_update_camera` is not called on Android — `handle_touch()` owns camera updates entirely.

The camera is placed in `CAM_MODE_FREE` + `free_track = true` at startup so it orbits the vehicle by default. This state is set once in `main()` and never changed by touch logic.

### Frame state (locals in `main()`)

| Variable | Type | Purpose |
|---|---|---|
| `orbit_target` | `Vector3` | Point being orbited; initialized to `vehicle.position` |
| `prev_pinch_dist` | `float` | Previous frame's pinch distance; for zoom delta |
| `prev_mid` | `Vector2` | Previous frame's 2-finger midpoint; for pan delta |
| `prev_touch` | `Vector2` | Previous frame's 1-finger position; for orbit delta |
| `prev_touch_count` | `int` | Finger count last frame; used to detect transitions |

## Gesture Handling

Enabled at startup: `SetGesturesEnabled(GESTURE_DRAG | GESTURE_PINCH_IN | GESTURE_PINCH_OUT)`.

### 1 finger — orbit

Compute `delta = GetTouchPosition(0) - prev_touch`. Apply yaw (rotate around world Y axis) and pitch (rotate around camera right axis) to the offset vector `camera.position - orbit_target`, using the same matrix rotation math as the existing desktop orbit (`scene.c:457–461`). Pitch is clamped to ±89° to prevent gimbal flip.

Sensitivity: `0.005f` rad/px.

### 2 fingers — zoom + pan (simultaneous)

Each frame compute `current_dist` (distance between the two touches) and `current_mid` (midpoint).

**Zoom:** `dist_delta = current_dist - prev_pinch_dist`. Scale the length of `camera.position - orbit_target` by `1.0f - dist_delta * zoom_sensitivity`. Minimum orbit distance clamped to 2m.

**Pan:** `mid_delta = current_mid - prev_mid`. Translate `orbit_target` along camera right and camera up axes. `camera.position` receives the same translation so the orbit offset is preserved.

Pan sensitivity scales with current orbit distance so panning feels consistent at any zoom level: `pan_sensitivity = orbit_dist * 0.002f`.

Zoom sensitivity: `0.01f` per px of pinch delta.

### Finger count transitions

When `GetTouchCount()` changes (1→2 or 2→1), reset `prev_pinch_dist`, `prev_mid`, and `prev_touch` to current values on that frame. This prevents a jump caused by stale previous-frame values.

## Initialization

In `main()`, after `InitWindow()` and `SetLoadFileTextCallback()`:

```c
SetGesturesEnabled(GESTURE_DRAG | GESTURE_PINCH_IN | GESTURE_PINCH_OUT);
scene.cam_mode  = CAM_MODE_FREE;
scene.free_track = true;
Vector3 orbit_target = vehicle.position;
```

## Edge Cases

- **Zero-length offset:** Guard with a length check before normalizing to avoid NaN when camera is exactly at target.
- **Pitch clamp:** Convert offset to spherical coordinates, clamp elevation to [-89°, 89°], convert back.
- **No new files, no CMake changes:** Everything is additive to `android_main.c`.

## Out of Scope

- Camera mode switching via touch (Chase, FPV) — no MAVLink yet, orbit-only is sufficient.
- Gesture feedback (haptics, UI indicators).
- Desktop/PC touch path — `scene.c` is untouched.
