#ifndef ULOG_REPLAY_APPLY_H
#define ULOG_REPLAY_APPLY_H

// Shared per-message state-update helpers for ULog replay. One source of truth
// between native (decodes live from the parser and calls apply) and WASM
// (pre-extracts events at init time, then iterates cursors calling apply).
//
// Each apply function takes a fully-decoded event struct and a ulog_replay_ctx_t.
// The decode step is platform-specific (native reads raw parser bytes, WASM
// reads pre-extracted arrays); the apply step is shared.

#include <stdint.h>

#include "ulog_replay.h"
#include "ulog_events.h"

#define ULOG_DEG_TO_RAD (3.14159265358979323846 / 180.0)

// Convert NED local coordinates (x,y,z meters) to lat/lon/alt (e7 / mm)
// given a reference frame. Shared by native lpos handling and WASM
// interpolation/apply paths.
void ulog_local_to_global(double ref_lat, double ref_lon, float ref_alt,
                           float x, float y, float z,
                           int32_t *lat_e7, int32_t *lon_e7, int32_t *alt_mm);

// Per-message apply helpers. Each updates ctx state from a decoded event.
// Callers must have already populated ctx->cache + subscription indices (on
// native) or ctx->ref_* (on WASM, set by the extractor pre-pass).
void ulog_apply_attitude(ulog_replay_ctx_t *ctx, const ulog_att_event_t     *ev);
void ulog_apply_gpos    (ulog_replay_ctx_t *ctx, const ulog_gpos_event_t    *ev);
void ulog_apply_lpos    (ulog_replay_ctx_t *ctx, const ulog_lpos_event_t    *ev);
void ulog_apply_aspd    (ulog_replay_ctx_t *ctx, const ulog_aspd_event_t    *ev);
void ulog_apply_vstatus (ulog_replay_ctx_t *ctx, const ulog_vstatus_event_t *ev);
void ulog_apply_home    (ulog_replay_ctx_t *ctx, const ulog_home_event_t    *ev);

#endif
