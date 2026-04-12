# Hawkeye Android

Android port of the Hawkeye 3D renderer, running the desktop C/Raylib scene natively on Android via NativeActivity and OpenGL ES 3.0.

## Architecture

The app has no Java/Kotlin logic beyond the manifest declaration. `NativeActivity` loads `libhawkeye.so`, which runs a standard C `main()` entry point via Raylib's Android platform backend.

```
android/
├── app/src/main/
│   ├── AndroidManifest.xml       — declares NativeActivity, requires OpenGL ES 3.0
│   ├── assets/                   — all symlinks into the parent Hawkeye project
│   │   ├── fonts  -> ../../../../fonts
│   │   ├── models -> ../../../../models
│   │   ├── shaders -> ../../../../shaders
│   │   └── themes -> ../../../../themes
│   └── cpp/
│       ├── CMakeLists.txt        — fetches Raylib 5.5, compiles rendering subset
│       └── android_main.c        — entry point: scene_init + vehicle_init + render loop
```

The Hawkeye source files compiled in are: `scene.c`, `vehicle.c`, `asset_path.c`, `theme.c`, `ortho_panel.c`. MAVLink, ULog, HUD, and replay code are excluded.

## Shader Compatibility

The original shaders use `#version 330` (desktop OpenGL). On Android they are patched at load time via Raylib's `SetLoadFileTextCallback`:

- `#version 330` → `#version 300 es`
- `precision mediump float;` is injected into fragment shaders

No shader copies are kept in the Android project — the `shaders/` asset dir is a symlink to the originals.

## Requirements

- Android SDK (API 29+)
- NDK 30.0.14904198
- CMake 3.22+
- A device or emulator with OpenGL ES 3.0 support

## Building

```bash
./gradlew assembleDebug
```

Raylib 5.5 is fetched automatically by CMake on the first build. Built ABIs: `arm64-v8a`, `x86_64`.

## Deploying

```bash
adb install -r app/build/outputs/apk/debug/app-debug.apk
adb shell am start -n com.px4.hawkeye.android/android.app.NativeActivity
```
