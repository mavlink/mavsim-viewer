#ifndef HAWKEYE_WASM_API_H
#define HAWKEYE_WASM_API_H

// Hawkeye WASM embed API.
//
// This header is the stable interface contract between the host page
// (Flight Review embed, self-hosted builder hub, or the local dev harness)
// and the WASM build of Hawkeye. All functions are exported via
// EMSCRIPTEN_KEEPALIVE and accessible from JavaScript via Module.ccall or
// Module.cwrap.
//
// Threading
// ---------
// All functions must be called from the main browser thread. Hawkeye WASM
// is single-threaded — it does not require SharedArrayBuffer, does not set
// COOP/COEP headers, and does not use pthreads. This is intentional so the
// build can embed inside host pages (e.g. Flight Review) that cannot
// guarantee cross-origin isolation.
//
// Lifecycle
// ---------
//     createHawkeye({canvas: ...})  // JS side, returns Module promise
//     hawkeye_init(canvas_id, w, h) // bind to canvas, init scene+hud, start frame loop
//     hawkeye_load_ulog_bytes(p, n) // (optional) extract a ULog and start playback
//     hawkeye_set_playing(1/0)      // play/pause
//     hawkeye_seek(seconds)         // jump to a time offset from log start
//     hawkeye_resize(w, h)          // respond to host-page layout changes
//     hawkeye_destroy()             // tear down and release resources
//
// Error handling
// --------------
// Integer-returning functions return 0 on success, nonzero on failure.
// Failures are logged to the browser console via printf. They are not
// signalled via postMessage — the host JS wrapper is expected to check
// return values if it cares.

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#else
// Allow this header to be read by native tooling (IDEs, clangd) even though
// the functions are only defined in the WASM build.
#define EMSCRIPTEN_KEEPALIVE
#endif

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Query the loaded log's duration in seconds. Returns 0 if no log is loaded.
EMSCRIPTEN_KEEPALIVE
double hawkeye_get_duration(void);

// Query the current playback position in seconds from log start.
// Returns 0 if no log is loaded.
EMSCRIPTEN_KEEPALIVE
double hawkeye_get_position(void);

// Multi-drone loading: stage files one at a time, then finalize.
//
// Usage from JS:
//     hawkeye_begin_multi_load(count)
//     for each file: hawkeye_stage_ulog(index, ptr, len)
//     hawkeye_finalize_multi_load(mode)
//
// mode: 0 = auto, 1 = formation, 2 = ghost, 3 = grid

EMSCRIPTEN_KEEPALIVE
int hawkeye_begin_multi_load(int count);

EMSCRIPTEN_KEEPALIVE
int hawkeye_stage_ulog(int index, const uint8_t *buf, size_t len);

EMSCRIPTEN_KEEPALIVE
int hawkeye_finalize_multi_load(int mode);

// Initialize Hawkeye, bind the renderer to the HTML canvas identified by
// canvas_id (a selector string e.g. "#hawkeye-canvas"), and start the
// per-frame update loop via emscripten_set_main_loop. Safe to call once
// per module instance.
//
// Preconditions: canvas_id is non-null; width/height > 0; WebGL2 available.
// Postconditions: on success, the main loop is running and rendering into
// the canvas. Returns 0 on success, -1 if the canvas cannot be acquired or
// WebGL2 init fails.
EMSCRIPTEN_KEEPALIVE
int hawkeye_init(const char *canvas_id, int width, int height);

// Load a ULog from a byte buffer. The buffer must contain a complete,
// valid ULog file. On success, the WASM replay context owns the extracted
// in-memory timeline and the caller may free (or let JS garbage collect)
// the input buffer immediately after this call returns.
//
// This function walks the file once to populate the flat event timeline
// and pre-scan state (mode changes, takeoff detection, home resolution).
// Memory footprint after return: approximately 1-5 MB per typical single
// drone flight, regardless of the input file size.
//
// Calling this while a previous log is loaded releases the previous log's
// timeline and replaces it. Returns 0 on success, -1 on parse failure or
// allocation failure.
EMSCRIPTEN_KEEPALIVE
int hawkeye_load_ulog_bytes(const uint8_t *buf, size_t len);

// Notify Hawkeye that the canvas size has changed. Updates the viewport,
// projection, and any cached framebuffer sizes. Cheap; safe to call on
// every resize event.
EMSCRIPTEN_KEEPALIVE
void hawkeye_resize(int width, int height);

// Tear down the Hawkeye instance: stop the main loop, release the ULog
// timeline, release GL resources (textures, VBOs, framebuffers), free the
// heap. After this returns, the module should not be used except by
// re-instantiating from createHawkeye().
EMSCRIPTEN_KEEPALIVE
void hawkeye_destroy(void);

// Toggle playback. When paused, the scene remains rendered and the HUD
// continues to draw, but replay time does not advance. Ignored if no
// log is loaded.
EMSCRIPTEN_KEEPALIVE
void hawkeye_set_playing(int playing);

// Seek absolute playback position, measured in seconds from the start of
// the loaded log. Clamped to [0, log_duration]. Ignored if no log is
// loaded. After seek, replay state is settled by walking cursors forward
// from a point slightly before the target, matching native's seek-early
// + forward-scan semantics.
EMSCRIPTEN_KEEPALIVE
void hawkeye_seek(double seconds);

// Query conflict detection result after staging files but before finalize.
// Bit0 = conflict_detected, bit1 = conflict_far. Returns 0 if not applicable
// (single drone, no log, or not initialized). Intended for host-side dialog
// wiring — call after all hawkeye_stage_ulog() calls complete.
EMSCRIPTEN_KEEPALIVE
int hawkeye_get_conflict_flags(void);

// Accept a .mvt theme file as raw bytes (drag-drop replacement for native's
// IsFileDropped handler). `name` is the filename used as the MEMFS key so
// theme_load_mvt's fopen path has something to open. Returns 0 on success.
EMSCRIPTEN_KEEPALIVE
int hawkeye_add_theme_bytes(const uint8_t *buf, size_t len, const char *name);

#ifdef __cplusplus
}
#endif

#endif
