cmake_minimum_required(VERSION 3.22)
project(hawkeye C)

# Tell Raylib to target Android with OpenGL ES 3.0
set(PLATFORM "Android" CACHE STRING "" FORCE)
set(GRAPHICS "GRAPHICS_API_OPENGL_ES3" CACHE STRING "" FORCE)

# Fetch Raylib 5.5 — same version as the desktop project
include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raylib)

# Hawkeye rendering source subset — no MAVLink/ULog/HUD
# CMakeLists.txt lives at android/app/src/main/cpp/ — five levels up is the repo root
set(HAWKEYE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../src")

add_library(hawkeye SHARED
    android_main.c
    ${HAWKEYE_SRC}/scene.c
    ${HAWKEYE_SRC}/vehicle.c
    ${HAWKEYE_SRC}/asset_path.c
    ${HAWKEYE_SRC}/theme.c
    ${HAWKEYE_SRC}/ortho_panel.c
)

target_include_directories(hawkeye PRIVATE
    ${HAWKEYE_SRC}
    ${ANDROID_NDK}/sources/android/native_app_glue
)

target_link_libraries(hawkeye
    raylib
    android
    log
    EGL
    GLESv3
)

# ANativeActivity_onCreate is called by the Android Java runtime via JNI, not by any
# C code — so the linker would silently strip it from the shared library without this.
target_link_options(hawkeye PRIVATE "-Wl,-u,ANativeActivity_onCreate")
