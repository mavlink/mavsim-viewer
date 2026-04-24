#ifndef WASM_REPLAY_H
#define WASM_REPLAY_H

// WASM replay path API. The replay context type is an alias for the native
// ulog_replay_ctx_t — WASM-specific fields (timeline, cursors, wall_accum)
// are added to the native struct behind #ifdef __EMSCRIPTEN__ so the shared
// apply helpers (src/ulog_replay_apply.{h,c}) can take a single ctx pointer
// on both targets.

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ulog_replay.h"    // ulog_replay_ctx_t (with WASM-only fields under ifdef)
#include "ulog_events.h"    // ulog_timeline_t + event structs

typedef ulog_replay_ctx_t wasm_replay_ctx_t;

// Initialize a replay context from a ULog byte buffer. The buffer may be
// freed by the caller immediately after this call returns — the extractor
// does not retain any reference to it. Returns 0 on success, -1 on failure.
int  wasm_replay_init_from_bytes(wasm_replay_ctx_t *ctx,
                                  const uint8_t *buf, size_t len);

// Advance playback by dt seconds at the given speed. Returns true if still
// playing (cursors have not all reached end-of-timeline).
bool wasm_replay_advance(wasm_replay_ctx_t *ctx, float dt, float speed,
                          bool looping, bool interpolation);

// Seek to an absolute position (seconds from log start).
void wasm_replay_seek(wasm_replay_ctx_t *ctx, float target_s);

// Free all owned resources.
void wasm_replay_close(wasm_replay_ctx_t *ctx);

// Log duration in microseconds (timeline end - start). Zero if not loaded.
uint64_t wasm_replay_duration_us(const wasm_replay_ctx_t *ctx);

#endif
