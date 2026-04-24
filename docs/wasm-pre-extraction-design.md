# WASM ULog pre-extraction — design

Design document for the WASM build's ULog replay path. Input to Task #2 of `velvety-dazzling-dolphin` plan. Not a commit artifact yet; lives here for user review before any code is written.

## Goal

On the WASM build, walk each ULog once at load time, extract every field the replay engine actually consumes into a flat in-memory timeline, then release the raw file bytes. Playback consumes the flat arrays instead of re-reading the file.

**Why:** a multi-drone swarm (Lucky Seven or similar) with typical ULog sizes would otherwise keep 80-300 MB of raw file bytes per drone resident in WASM heap. Mobile browsers OOM. Desktop browsers waste memory. The existing file-backed parser is excellent on native (leave it alone) and wrong on WASM (replace it for playback).

**Non-goal:** native behavior changes. Native keeps the exact code path it has today. Every `#ifdef __EMSCRIPTEN__` branch in the parser and replay code must have a byte-identical native fallthrough.

## Memory target

Target: ≤5 MB per drone for a 1000-second flight at typical PX4 logging rates. 16-drone case ≤80 MB total.

Rough math for one drone, 1000 s flight:
- `vehicle_attitude` at 100 Hz: 100,000 events × 24 bytes = 2.4 MB
- `vehicle_local_position` at 50 Hz: 50,000 events × 32 bytes = 1.6 MB
- `vehicle_global_position` at 10 Hz: 10,000 events × 32 bytes = 320 KB
- `airspeed_validated` at 20 Hz: 20,000 events × 16 bytes = 320 KB
- `vehicle_status` at 1 Hz: 1,000 events × 8 bytes = 8 KB
- `home_position` (rare): <1,000 events × 32 bytes < 32 KB
- `STATUSTEXT`: typically <200 entries, ~144 bytes each = <29 KB
- Subtotal: ~4.7 MB

Well within target. The plan's original 1-5 MB estimate is realistic.

## Event data model

Separate arrays per event type, not a tagged union. A union would pad every event to the size of the largest member (STATUSTEXT's 128-byte text buffer), bloating attitude from 24 B to ~144 B — a 6× waste on the dominant event type. Per-type arrays also let us seek efficiently: each array is sorted by timestamp, binary search per type.

### Struct definitions (C)

Proposed shape. Field ordering picked for packing; `uint64_t` timestamps aligned to 8. These go in a new `ulog_events.h` header that both native and WASM can include (native never instantiates the arrays, only uses the apply helpers).

```c
// --- per-event structs ---

typedef struct {
    uint64_t timestamp_us;
    float q[4];                         // w, x, y, z
} ulog_att_event_t;                     // 24 bytes

typedef struct {
    uint64_t timestamp_us;
    double lat_deg;
    double lon_deg;
    float  alt_m;
    float  _pad;                        // keep 8-byte alignment on 32-bit WASM
} ulog_gpos_event_t;                    // 32 bytes

typedef struct {
    uint64_t timestamp_us;
    float x, y, z;
    float vx, vy, vz;
} ulog_lpos_event_t;                    // 32 bytes

typedef struct {
    uint64_t timestamp_us;
    uint16_t ias_cms;                   // indicated_airspeed * 100
    uint16_t tas_cms;                   // true_airspeed * 100
    uint32_t _pad;
} ulog_aspd_event_t;                    // 16 bytes

typedef struct {
    uint64_t timestamp_us;
    uint8_t vehicle_type;               // PX4 enum, pre-translated to MAV_TYPE
    uint8_t is_vtol;
    uint8_t nav_state;
    uint8_t _pad[5];
} ulog_vstatus_event_t;                 // 16 bytes

typedef struct {
    uint64_t timestamp_us;
    double lat_deg;
    double lon_deg;
    float  alt_m;
    uint8_t valid_hpos;
    uint8_t _pad[3];
} ulog_home_event_t;                    // 32 bytes

typedef struct {
    uint64_t timestamp_us;
    uint8_t severity;
    uint8_t _pad[7];
    char    text[STATUSTEXT_MSG_MAX];   // 128 bytes
} ulog_statustext_event_t;              // 144 bytes
```

### Timeline container

A single `ulog_timeline_t` holds all arrays plus the one-shot LPOS reference point (which is set once from the first LPOS message that has `xy_global`). Arrays grow with `realloc` during extraction and get final-sized at the end of the walk.

```c
typedef struct {
    // Per-type event arrays (grown during extraction, final-sized after)
    ulog_att_event_t        *att;       int att_count;       int att_cap;
    ulog_lpos_event_t       *lpos;      int lpos_count;      int lpos_cap;
    ulog_gpos_event_t       *gpos;      int gpos_count;      int gpos_cap;
    ulog_aspd_event_t       *aspd;      int aspd_count;      int aspd_cap;
    ulog_vstatus_event_t    *vstatus;   int vstatus_count;   int vstatus_cap;
    ulog_home_event_t       *home;      int home_count;      int home_cap;
    ulog_statustext_event_t *statustext; int statustext_count; int statustext_cap;

    // One-shot LPOS reference point (captured on first LPOS with xy_global)
    double ref_lat_deg;
    double ref_lon_deg;
    float  ref_alt_m;
    uint8_t ref_set;
    uint8_t ref_rejected;
} ulog_timeline_t;
```

### Where the timeline lives

On WASM, `ulog_replay_ctx_t` gains a `ulog_timeline_t timeline;` field (gated by `#ifdef __EMSCRIPTEN__` so native sizeof stays unchanged). Playback cursors per array (`att_cursor`, `lpos_cursor`, etc.) also live on the ctx under the same ifdef.

## Extraction pass

The existing init already performs a full-file scan at `src/ulog_replay.c:377` (`while (ulog_parser_next(&ctx->parser, &scan_msg))`). Current scan purposes: `mode_changes`, CUSUM takeoff, home resolution, position tier. The WASM extraction **piggybacks on this same walk** — it does not add a second file read.

Inside the scan loop, after the existing native-side work is done, a new `#ifdef __EMSCRIPTEN__` block dispatches the current message to an appender:

```c
#ifdef __EMSCRIPTEN__
    if (scan_msg.msg_id == att_msg_id)
        extract_attitude(&ctx->timeline, &scan_msg, &ctx->cache);
    else if (scan_msg.msg_id == lpos_msg_id)
        extract_lpos(&ctx->timeline, &scan_msg, &ctx->cache);
    // ... etc
#endif
```

Each `extract_*` helper decodes its subscription's fields using the existing `ulog_parser_get_*` accessors (which work on in-memory `ulog_data_msg_t` structs and have nothing to do with file I/O) and appends a fully-populated event struct to the corresponding array. Amortized O(1) append via doubling `realloc`.

**STATUSTEXT special case**: the current `logging_cb` (line 8) is enabled *after* the pre-scan via `ulog_parser_set_logging_callback` so the ring buffer only captures messages encountered during real playback, not the pre-scan walk. On WASM this needs to change — we need to capture all STATUSTEXT entries during the pre-scan walk into the flat `statustext` array, then dispatch them to the ring buffer at playback time when the cursor crosses each one. Simple enough to gate.

**After the extraction walk**:

- Native path (unchanged): `ulog_parser_rewind(&ctx->parser)` at line 507. Playback re-reads the file during `_advance`.
- WASM path: `ulog_parser_close(&ctx->parser)` right after the walk (or immediately free `p->read_buf` and the raw file bytes that `--preload-file` loaded into VFS). Playback never re-reads the file. The parser state (formats, subs, index) can be freed too — it's only needed by `ulog_parser_next`.

Actually — subtle point worth flagging. The WASM build under `emcc` with `--preload-file` bakes the asset into `.data` which Emscripten's MEMFS stages into the WASM heap at startup. For bundled shipped assets (models, shaders, fonts, themes, textures) that's fine — they live for the lifetime of the process. For a user-uploaded ULog passed through `hawkeye_load_ulog_bytes`, the bytes come in via JS → `HEAPU8.set` → a C buffer we own. We're responsible for freeing that buffer. We free it immediately after extraction. No MEMFS involvement for ULogs.

## Playback

### Native path — byte-identical to today

`ulog_replay_advance` on native keeps calling `ulog_parser_next` exactly as it does now. No change. Literal zero diff in the execution path.

### WASM path — cursor iteration over flat arrays

`ulog_replay_advance` on WASM iterates each array in parallel, advancing cursors while `timeline.X[cursor].timestamp_us <= target_usec`. For each advance, it calls the shared apply helper. This is a linear scan per call; cursors only move forward during play. On seek, cursors are binary-searched to the new target.

```c
#ifdef __EMSCRIPTEN__
static bool replay_advance_wasm(ulog_replay_ctx_t *ctx, uint64_t target_usec) {
    ulog_timeline_t *tl = &ctx->timeline;

    while (ctx->att_cursor < tl->att_count &&
           tl->att[ctx->att_cursor].timestamp_us <= target_usec) {
        apply_attitude(ctx, &tl->att[ctx->att_cursor]);
        ctx->att_cursor++;
    }
    while (ctx->lpos_cursor < tl->lpos_count &&
           tl->lpos[ctx->lpos_cursor].timestamp_us <= target_usec) {
        apply_lpos(ctx, &tl->lpos[ctx->lpos_cursor]);
        ctx->lpos_cursor++;
    }
    // ... similarly for gpos, aspd, vstatus, home, statustext
    return ctx->att_cursor < tl->att_count ||
           ctx->lpos_cursor < tl->lpos_count;  // etc
}
#endif
```

## Shared apply helpers (drift prevention)

This is the critical part. If native and WASM have separate state-update implementations, they will desync. Instead, both call the same per-type apply helpers. Native paths *decode* fields from a raw `ulog_data_msg_t` then call the helper; WASM paths read fields directly from the pre-extracted event struct then call the helper.

### Refactor to extract

`process_message` at `src/ulog_replay.c:69-258` currently mixes field decoding and state update. Split each branch into:

- `decode_attitude(dmsg, cache) → ulog_att_event_t`  (decode only, no state touched)
- `apply_attitude(ctx, event)`                       (update ctx->state, touch nothing that requires dmsg)

Then `process_message` on native becomes:

```c
static void process_message(ulog_replay_ctx_t *ctx, const ulog_data_msg_t *dmsg) {
    int sub_idx = find_subscription(ctx, dmsg->msg_id);
    if (sub_idx < 0) return;
    ctx->state.time_usec = dmsg->timestamp;
    if (ctx->first_pos_set) ctx->state.valid = true;

    if (sub_idx == ctx->sub_attitude) {
        ulog_att_event_t ev;
        decode_attitude(dmsg, &ctx->cache, &ev);
        apply_attitude(ctx, &ev);
    }
    else if (sub_idx == ctx->sub_global_pos) { /* ... */ }
    // etc
}
```

WASM's `replay_advance_wasm` skips the decode step entirely and calls `apply_*` directly on the pre-extracted events.

**Why this is safe:** the apply functions take fully-decoded field values, not raw bytes. They don't know or care whether the data came from a live parser read or an in-memory array. Any bug fix to `apply_attitude` (e.g. state calculation, first-pos flag handling) automatically applies to both paths because there's exactly one implementation.

**Native cost of the refactor**: two function calls per message instead of one. No heap allocation (the decoded event goes on the stack — ~32 bytes). Inlining will likely collapse this back to the original code size. Benchmarks would be nice but the overhead is negligible.

### The extraction path reuses decode_* too

This is the elegant part: `extract_attitude` during the pre-scan walk is literally `decode_attitude` + `timeline_append_att`. Same decode logic as native playback. One source of truth for field decoding, one source of truth for state update. The only thing that differs between native and WASM is **where the data lives** — on disk (re-read per frame) vs in RAM (iterated per frame).

## Seek semantics

### Native path — unchanged

`ulog_replay_seek` already calls `ulog_parser_seek_early` + forward-scan up to 2000 messages (line 653). Leave it alone.

### WASM path

`ulog_replay_seek` under `#ifdef __EMSCRIPTEN__` binary-searches each per-type cursor to the largest index where `timestamp_us <= target_usec`. Then it rewinds the cursors slightly and replays forward to rebuild `ctx->state` coherently — same principle as the native path's "seek_early + forward scan" trick, but on in-memory arrays.

To rebuild state coherently after seek, all arrays must be replayed in timestamp order from some point before `target_usec`. Easiest implementation: on seek, set all cursors to zero (or binary-search each back by N seconds to a chosen resync point), then call `replay_advance_wasm(ctx, target_usec)` which advances each cursor forward to the target. This is O(messages-since-resync), same asymptotic cost as the native forward scan.

Decision to make during implementation: fully-from-zero rewind on every seek (simple, correct, O(N) per seek) vs. banded resync with a snapshot every K seconds (faster for long flights, more complex). Start with from-zero; profile if it feels sluggish.

## Subscription indices and cache

Both native and WASM need `ctx->sub_attitude`, `ctx->sub_local_pos`, etc. and the corresponding field offset cache (`ctx->cache`). These are computed by `ulog_parser_find_subscription` and `ulog_parser_find_field` during init, **before** the walk. That stays unchanged. Even on WASM, the cache gets populated because we need it for the decode step during extraction.

After extraction, on WASM, `ctx->cache` and `ctx->parser.subs` could be freed since they're only used for decoding raw messages. Keeping them is cheap (~a few KB) and leaves the ctx structurally symmetric between native and WASM, which simplifies debugging. Recommend keeping.

## Memory lifecycle on WASM

1. JS handles user-supplied ULog: reads file into a `Uint8Array`, calls `hawkeye_load_ulog_bytes(ptr, len)`
2. C side: `malloc(len)`, `memcpy` from `HEAPU8`, releases the JS ArrayBuffer pressure
3. C side: opens an in-memory "file" (or the parser uses the buffer directly if we pick Option A plumbing — see next section)
4. `ulog_replay_init` runs the normal init, pre-scan extracts events into `ctx->timeline`
5. C side: frees the raw ULog buffer from step 2
6. Playback runs against `ctx->timeline` with zero file I/O
7. On `hawkeye_destroy`, `ulog_replay_close` frees all timeline arrays

Peak memory during step 2-4 is the raw file size plus the growing timeline arrays. After step 5, only the timeline. For a typical big ULog, peak might be ~90 MB briefly, settling to ~5 MB. Per drone loaded sequentially, peak is contained.

## Plumbing: how does the parser read from memory during extraction?

The parser currently uses `FILE *fp`. For the extraction walk to even run on WASM, the parser needs a memory backend. **Good news**: Emscripten provides `fmemopen(buf, len, "rb")` via its libc, which returns a `FILE *` backed by a memory buffer. All `fread`/`fseek`/`ftell` calls work unchanged against it.

This means the parser code does **not** need to be modified for WASM at all. A single `#ifdef __EMSCRIPTEN__` at the `fopen` site in `ulog_parser_open` can route to `fmemopen` when called from a new `ulog_parser_open_mem(parser, buf, len)` API:

```c
int ulog_parser_open_mem(ulog_parser_t *p, const uint8_t *buf, size_t len);
```

This wrapper does the header validation and Pass 1/2 logic exactly like `ulog_parser_open`, but uses `fmemopen` instead of `fopen`. Or more elegantly, refactor `ulog_parser_open` to take a `FILE *` already-opened, and add two thin wrappers (`_open_file`, `_open_mem`). Native uses `_open_file`; WASM uses `_open_mem`.

**Verification needed before committing to this**: confirm Emscripten's `fmemopen` is reliable and supports `fseek` correctly. I'm 95% sure it does (it's a glibc API Emscripten has supported for years), but this is exactly the kind of claim I should verify before building on it rather than discovering a regression during implementation.

## What changes to `ulog_replay_ctx_t` (header)

New fields, all gated by `#ifdef __EMSCRIPTEN__` so native sizeof is byte-identical:

```c
#ifdef __EMSCRIPTEN__
    ulog_timeline_t timeline;
    int att_cursor;
    int lpos_cursor;
    int gpos_cursor;
    int aspd_cursor;
    int vstatus_cursor;
    int home_cursor;
    int statustext_cursor;
#endif
```

New public function:

```c
int ulog_replay_init_from_bytes(ulog_replay_ctx_t *ctx,
                                const uint8_t *buf, size_t len);
```

Implemented only under `#ifdef __EMSCRIPTEN__`. Calls `ulog_parser_open_mem` then runs the same init body as `ulog_replay_init`.

## What does NOT change

- `ulog_parser.h` public API — unchanged (one new `ulog_parser_open_mem` function; existing functions untouched)
- `ulog_parser.c` core logic — unchanged; only a new wrapper function added
- Native `ulog_replay_init(ctx, filepath)` — unchanged
- Native `ulog_replay_advance` — unchanged (calls `process_message` which calls `decode_*` + `apply_*`)
- Native `ulog_replay_seek` — unchanged
- All fixtures, tests, existing replay behavior on desktop — byte-identical output

The refactor of `process_message` into `decode_*` + `apply_*` helpers is a **pure refactor with no behavioral change**. It ships on native with no feature flag. Each fixture ULog should produce byte-identical replay state when walked through the refactored code, which is what the prerequisite's verification gate checks.

## Resolved decisions

1. **Parser memory backend.** Use libc `fmemopen` via Emscripten's musl-derived stdio. No shim, no custom memory FILE wrapper. Verify during implementation that the Emscripten build links the musl implementation (five-minute check of their libc config), but this is a build-flag check, not a reliability concern — `fmemopen` is a stable POSIX API and musl's implementation is battle-tested. A new `ulog_parser_open_mem(p, buf, len)` wrapper calls `fmemopen` then runs the existing Pass 1/2 validation logic unchanged.
2. **Refactor scope: WASM-only (Option B).** The `decode_*` / `apply_*` split lives entirely under `#ifdef __EMSCRIPTEN__`. Native `process_message` is byte-for-byte unchanged — not a line touched, not a function reordered. Trade-off accepted: a small ongoing drift risk between the two apply paths in exchange for zero risk of regressing native today. If native ever needs the same refactor for its own reasons, that's a separate assessment later.
3. **STATUSTEXT capture timing.** On WASM, the logging callback is registered *before* the extraction walk and routes LOGGING messages into a flat `statustext` array on `ulog_timeline_t`, each entry tagged with its original timestamp. During playback, the statustext cursor advances alongside the other cursors and dispatches entries to the `statustext_ring_t` when their timestamps are crossed. End-user behavior identical to native (ring buffer fills in the right order at the right times). Native path is untouched — it continues to register the callback post-scan and fill the ring during live re-reads of the file.
4. **Seek strategy — mirror native.** Native already solved this at `src/ulog_replay.c:639`: jump close via `ulog_parser_seek_early`, then forward-scan up to 2000 messages calling `process_message` to settle state. WASM does the direct equivalent: binary-search each cursor to a point slightly before the target (the analogue of `seek_early`), then advance forward calling `apply_*` on each event until cursors reach the target. Same bounded cost, same user-facing latency, same "replay a window to settle state" invariant. Not a new design decision — just preserve the strategy and swap the mechanism.
5. **Multi-drone memory pressure on mobile.** Measurement deferred to Step 11 (cross-browser smoke test). No preemptive caps, no preemptive struct packing, no preemptive feature deferral. We find out what iOS Safari actually does with 16 drones when we can measure on a real device, and react then if we overshoot budget.

### Secondary decisions (deferred, premature to decide now)

- **LPOS reference point capture timing** — native captures on-the-fly during playback at the first LPOS with `xy_global`. WASM will do the same by having the extraction walk call the same `apply_lpos` helper that sets `ref_set` on first qualifying sample. No need for a one-shot field on `ulog_timeline_t`; the `ctx->ref_*` fields get populated naturally by the extraction walk because it calls `apply_lpos`.
- **Skipping the parser's timestamp index on WASM** — the index is only used for `ulog_parser_seek_early`, which WASM doesn't call. We could skip building it. Tiny optimization; revisit after first green build if binary size or init time is an issue.

## Verification gate for Task #2

This is the same gate from the plan, restated with specific checks:

1. `git diff src/ulog_parser.h src/ulog_parser.c src/ulog_replay.h src/ulog_replay.c` shows only:
   - The `decode_*` / `apply_*` refactor (pure behavioral no-op on native)
   - New `#ifdef __EMSCRIPTEN__` blocks
   - New `ulog_parser_open_mem` function
   - New `ulog_replay_init_from_bytes` function
2. `make release` and `make test` pass on a clean checkout, output byte-identical to pre-change baseline.
3. All fixture ULogs in `tests/fixtures/` (or wherever they live) produce byte-identical replay state when loaded through the refactored native path. This is the strongest drift-prevention check — if native output drifts by even one bit after the refactor, we have a bug.
4. User reviews the diff before any commit. No push.

---

**Summary of deltas from the original plan's Step 2**: same goal, same target memory footprint, same hard-prerequisite role. New specifics:

- Uses `fmemopen` (pending verification) instead of a bespoke memory backend, keeping `ulog_parser.c` almost untouched
- Refactors `process_message` into `decode_*` + `apply_*` helpers as a pure no-op change on native, shared between both paths to prevent drift
- Per-type event arrays instead of a tagged union, for memory efficiency
- STATUSTEXT capture moves from playback-callback to extraction-callback on WASM, with a flat statustext event array feeding the ring buffer during playback
- Pre-scan walk is **reused** rather than duplicated — the extraction piggybacks on the existing init-time walk, not a separate pass
