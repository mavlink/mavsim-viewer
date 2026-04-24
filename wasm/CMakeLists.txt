cmake_minimum_required(VERSION 3.14)
project(hawkeye-wasm C)

# -----------------------------------------------------------------------------
# Hawkeye WASM build. Standalone CMake subproject invoked via:
#
#     emcmake cmake -S wasm -B wasm/build -DCMAKE_BUILD_TYPE=Release
#     cmake --build wasm/build -j
#
# Does NOT modify the root CMakeLists.txt or any file under src/*.c / src/*.h.
# All WASM-specific C code lives under src/wasm/ and this directory.
# -----------------------------------------------------------------------------

if(NOT EMSCRIPTEN)
    message(FATAL_ERROR
        "The Hawkeye WASM build requires the Emscripten toolchain. "
        "Configure with: emcmake cmake -S wasm -B wasm/build")
endif()

set(CMAKE_C_STANDARD 11)

set(HAWKEYE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(HAWKEYE_SRC ${HAWKEYE_ROOT}/src)

# -----------------------------------------------------------------------------
# Raylib via FetchContent, retargeted to Web + OpenGL ES 3
# -----------------------------------------------------------------------------
# Raylib's own CMake responds to PLATFORM/GRAPHICS cache variables set BEFORE
# FetchContent_MakeAvailable(). Setting them here compiles Raylib's web
# backend (rcore_web.c + GLFW/emscripten glue) instead of its desktop backend.

set(PLATFORM "Web" CACHE STRING "Raylib target platform" FORCE)
set(GRAPHICS "GRAPHICS_API_OPENGL_ES3" CACHE STRING "Raylib graphics API" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# Raylib module trims — every disabled module shrinks the WASM binary by a
# meaningful amount. Drop audio (no MAVLink beeps in Hawkeye). Drop image
# format parsers except PNG. Drop model format parsers except OBJ + GLTF.
set(SUPPORT_MODULE_RAUDIO     OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_JPG    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_BMP    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_TGA    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_GIF    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_QOI    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_PSD    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_DDS    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_HDR    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_PIC    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_KTX    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_ASTC   OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_PKM    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_PVR    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_IQM    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_VOX    OFF CACHE BOOL "" FORCE)
set(SUPPORT_FILEFORMAT_M3D    OFF CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        5.5
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(raylib)

# -----------------------------------------------------------------------------
# Hawkeye WASM executable
# -----------------------------------------------------------------------------
# Skeleton stage (Task #4 of plan): wasm_main.c is a stub that initializes
# Raylib, clears the canvas, and implements the six embed API functions as
# no-ops. Scene / vehicle / HUD / replay wire-up happens in later tasks by
# adding to this source list, not by rewriting the build.
#
# Source files included at this stage:
#   - wasm/wasm_main.c                      (entry point, stub API impl)
#   - src/wasm/ulog_timeline.c              (flat-array event container)
#   - src/wasm/ulog_extractor.c             (ULog walker + pre-scan)
#   - src/wasm/wasm_replay.c                (cursor-based playback)

add_executable(hawkeye
    ${CMAKE_CURRENT_SOURCE_DIR}/wasm_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/wasm_dialog.c

    # Shared apply helpers (native + WASM). Update-once source of truth for
    # per-message state updates; decode stage is platform-specific.
    ${HAWKEYE_SRC}/ulog_replay_apply.c

    # WASM-only C code (freshly written, separate from native src/)
    ${HAWKEYE_SRC}/wasm/ulog_timeline.c
    ${HAWKEYE_SRC}/wasm/ulog_extractor.c
    ${HAWKEYE_SRC}/wasm/wasm_replay.c

    # Rendering spine — shared with native (no modifications to these files)
    ${HAWKEYE_SRC}/scene.c
    ${HAWKEYE_SRC}/vehicle.c
    ${HAWKEYE_SRC}/theme.c
    ${HAWKEYE_SRC}/asset_path.c

    # HUD subsystem — shared with native
    ${HAWKEYE_SRC}/hud.c
    ${HAWKEYE_SRC}/hud_help.c
    ${HAWKEYE_SRC}/hud_instruments.c
    ${HAWKEYE_SRC}/hud_transport.c
    ${HAWKEYE_SRC}/hud_telemetry.c
    ${HAWKEYE_SRC}/hud_annunciators.c
    ${HAWKEYE_SRC}/tactical_hud.c
    ${HAWKEYE_SRC}/debug_panel.c
    ${HAWKEYE_SRC}/ortho_panel.c
    ${HAWKEYE_SRC}/replay_trail.c
    ${HAWKEYE_SRC}/replay_markers.c
    ${HAWKEYE_SRC}/replay_conflict.c
    ${HAWKEYE_SRC}/ui_marker_input.c
)

target_include_directories(hawkeye PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}        # wasm/wasm_api.h
    ${HAWKEYE_SRC}                     # src/*.h (type-only includes)
    ${HAWKEYE_SRC}/wasm                # src/wasm/*.h
    ${HAWKEYE_ROOT}/lib/c_library_v2
    ${HAWKEYE_ROOT}/lib/c_library_v2/common
)

target_compile_definitions(hawkeye PRIVATE MAVLINK_COMM_NUM_BUFFERS=16)

target_link_libraries(hawkeye PRIVATE raylib)

# emcc emits <target>.js + <target>.wasm (+ <target>.data when preload is used)
set_target_properties(hawkeye PROPERTIES SUFFIX ".js")

target_link_options(hawkeye PRIVATE
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sEXPORT_NAME=createHawkeye"
    "-sEXPORTED_FUNCTIONS=['_hawkeye_init','_hawkeye_load_ulog_bytes','_hawkeye_begin_multi_load','_hawkeye_stage_ulog','_hawkeye_finalize_multi_load','_hawkeye_get_conflict_flags','_hawkeye_add_theme_bytes','_hawkeye_resize','_hawkeye_destroy','_hawkeye_set_playing','_hawkeye_seek','_hawkeye_get_duration','_hawkeye_get_position','_malloc','_free']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','HEAPU8']"
    "-sINITIAL_MEMORY=16MB"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sMAXIMUM_MEMORY=512MB"
    "-sUSE_WEBGL2=1"
    "-sFULL_ES3=1"
    "-sENVIRONMENT=web"
    "-sALLOW_TABLE_GROWTH=1"
    "-sSTACK_SIZE=1MB"
    "SHELL:--preload-file ${HAWKEYE_ROOT}/shaders@shaders"
    "SHELL:--preload-file ${HAWKEYE_ROOT}/models@models"
    "SHELL:--preload-file ${HAWKEYE_ROOT}/fonts@fonts"
    "SHELL:--preload-file ${HAWKEYE_ROOT}/textures@textures"
    "SHELL:--preload-file ${HAWKEYE_ROOT}/themes@themes"
)

# Release: size-optimize and run the closure compiler on JS glue.
# Debug: leave symbols + assertions on for early bring-up.
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(hawkeye PRIVATE -Os)
    target_link_options(hawkeye PRIVATE -Os)
else()
    target_compile_options(hawkeye PRIVATE -O0 -g3)
    target_link_options(hawkeye PRIVATE -O0 -g3 "-sASSERTIONS=2" "-sSAFE_HEAP=1")
endif()

# Copy the dev-harness shell into the build dir as index.html so the
# python http.server can serve it at the root URL. Runs on every build
# via a dedicated custom target that depends on shell.html's mtime.
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/index.html
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/shell.html
            ${CMAKE_CURRENT_BINARY_DIR}/index.html
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/shell.html
)
add_custom_target(hawkeye_index ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/index.html)

add_custom_command(TARGET hawkeye POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "hawkeye WASM build complete. See wasm/README.md for smoke-test instructions."
)
