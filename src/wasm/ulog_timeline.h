#ifndef WASM_ULOG_TIMELINE_H
#define WASM_ULOG_TIMELINE_H

// Append/free helpers for the flat-array event timeline used by the WASM
// replay path. All appends are amortized O(1) via doubling realloc.

#include "ulog_events.h"

void ulog_timeline_init(ulog_timeline_t *tl);
void ulog_timeline_free(ulog_timeline_t *tl);

// Shrink every array's capacity to its count. Call once after extraction
// finishes so idle capacity doesn't sit in the WASM heap for the lifetime
// of the replay.
void ulog_timeline_shrink_to_fit(ulog_timeline_t *tl);

// Appenders. Each returns 0 on success, -1 on allocation failure.
int ulog_timeline_append_att       (ulog_timeline_t *tl, const ulog_att_event_t        *ev);
int ulog_timeline_append_lpos      (ulog_timeline_t *tl, const ulog_lpos_event_t       *ev);
int ulog_timeline_append_gpos      (ulog_timeline_t *tl, const ulog_gpos_event_t       *ev);
int ulog_timeline_append_aspd      (ulog_timeline_t *tl, const ulog_aspd_event_t       *ev);
int ulog_timeline_append_vstatus   (ulog_timeline_t *tl, const ulog_vstatus_event_t    *ev);
int ulog_timeline_append_home      (ulog_timeline_t *tl, const ulog_home_event_t       *ev);
int ulog_timeline_append_statustext(ulog_timeline_t *tl, const ulog_statustext_event_t *ev);

// Binary-search each array for the largest index where timestamp_us <= target.
// Returns -1 if the array is empty or the first event is already past target.
int ulog_timeline_find_att_at       (const ulog_timeline_t *tl, uint64_t target_us);
int ulog_timeline_find_lpos_at      (const ulog_timeline_t *tl, uint64_t target_us);
int ulog_timeline_find_gpos_at      (const ulog_timeline_t *tl, uint64_t target_us);
int ulog_timeline_find_aspd_at      (const ulog_timeline_t *tl, uint64_t target_us);
int ulog_timeline_find_vstatus_at   (const ulog_timeline_t *tl, uint64_t target_us);
int ulog_timeline_find_home_at      (const ulog_timeline_t *tl, uint64_t target_us);
int ulog_timeline_find_statustext_at(const ulog_timeline_t *tl, uint64_t target_us);

#endif
