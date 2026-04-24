#include "ulog_timeline.h"

#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Generic growth helper — doubles capacity on overflow
// ---------------------------------------------------------------------------

static int grow_if_needed(void **arr_ptr, int *cap, int count, size_t elem_size) {
    if (count < *cap) return 0;
    int new_cap = *cap ? *cap * 2 : 64;
    void *new_arr = realloc(*arr_ptr, (size_t)new_cap * elem_size);
    if (!new_arr) return -1;
    *arr_ptr = new_arr;
    *cap = new_cap;
    return 0;
}

static void shrink_array(void **arr_ptr, int *cap, int count, size_t elem_size) {
    if (count == *cap) return;
    if (count == 0) {
        free(*arr_ptr);
        *arr_ptr = NULL;
        *cap = 0;
        return;
    }
    void *new_arr = realloc(*arr_ptr, (size_t)count * elem_size);
    // realloc shrink is allowed to fail, in which case we keep the larger
    // buffer — still correct, just wastes capacity.
    if (new_arr) {
        *arr_ptr = new_arr;
        *cap = count;
    }
}

// ---------------------------------------------------------------------------
// Binary search: returns largest index i where arr[i].timestamp_us <= target.
// Returns -1 if count == 0 or target < arr[0].timestamp_us.
//
// The timeline arrays are sorted by timestamp_us (guaranteed because ULog
// DATA messages are emitted in monotonic timestamp order during extraction),
// so binary search is valid.
// ---------------------------------------------------------------------------

#define DEFINE_FIND(TYPE, FIELD)                                          \
    int ulog_timeline_find_##FIELD##_at(const ulog_timeline_t *tl,         \
                                         uint64_t target_us) {             \
        if (tl->FIELD##_count == 0) return -1;                             \
        if (tl->FIELD[0].timestamp_us > target_us) return -1;              \
        int lo = 0, hi = tl->FIELD##_count - 1, best = 0;                  \
        while (lo <= hi) {                                                 \
            int mid = (lo + hi) / 2;                                       \
            if (tl->FIELD[mid].timestamp_us <= target_us) {                \
                best = mid;                                                \
                lo = mid + 1;                                              \
            } else {                                                       \
                hi = mid - 1;                                              \
            }                                                              \
        }                                                                  \
        return best;                                                       \
    }

DEFINE_FIND(ulog_att_event_t,        att)
DEFINE_FIND(ulog_lpos_event_t,       lpos)
DEFINE_FIND(ulog_gpos_event_t,       gpos)
DEFINE_FIND(ulog_aspd_event_t,       aspd)
DEFINE_FIND(ulog_vstatus_event_t,    vstatus)
DEFINE_FIND(ulog_home_event_t,       home)
DEFINE_FIND(ulog_statustext_event_t, statustext)

#undef DEFINE_FIND

// ---------------------------------------------------------------------------
// Init / free
// ---------------------------------------------------------------------------

void ulog_timeline_init(ulog_timeline_t *tl) {
    memset(tl, 0, sizeof(*tl));
}

void ulog_timeline_free(ulog_timeline_t *tl) {
    free(tl->att);         tl->att = NULL;         tl->att_count = 0;         tl->att_cap = 0;
    free(tl->lpos);        tl->lpos = NULL;        tl->lpos_count = 0;        tl->lpos_cap = 0;
    free(tl->gpos);        tl->gpos = NULL;        tl->gpos_count = 0;        tl->gpos_cap = 0;
    free(tl->aspd);        tl->aspd = NULL;        tl->aspd_count = 0;        tl->aspd_cap = 0;
    free(tl->vstatus);     tl->vstatus = NULL;     tl->vstatus_count = 0;     tl->vstatus_cap = 0;
    free(tl->home);        tl->home = NULL;        tl->home_count = 0;        tl->home_cap = 0;
    free(tl->statustext);  tl->statustext = NULL;  tl->statustext_count = 0;  tl->statustext_cap = 0;
    tl->start_timestamp_us = 0;
    tl->end_timestamp_us = 0;
}

void ulog_timeline_shrink_to_fit(ulog_timeline_t *tl) {
    shrink_array((void **)&tl->att,        &tl->att_cap,        tl->att_count,        sizeof(ulog_att_event_t));
    shrink_array((void **)&tl->lpos,       &tl->lpos_cap,       tl->lpos_count,       sizeof(ulog_lpos_event_t));
    shrink_array((void **)&tl->gpos,       &tl->gpos_cap,       tl->gpos_count,       sizeof(ulog_gpos_event_t));
    shrink_array((void **)&tl->aspd,       &tl->aspd_cap,       tl->aspd_count,       sizeof(ulog_aspd_event_t));
    shrink_array((void **)&tl->vstatus,    &tl->vstatus_cap,    tl->vstatus_count,    sizeof(ulog_vstatus_event_t));
    shrink_array((void **)&tl->home,       &tl->home_cap,       tl->home_count,       sizeof(ulog_home_event_t));
    shrink_array((void **)&tl->statustext, &tl->statustext_cap, tl->statustext_count, sizeof(ulog_statustext_event_t));
}

// ---------------------------------------------------------------------------
// Appenders
// ---------------------------------------------------------------------------

#define DEFINE_APPEND(TYPE, FIELD)                                             \
    int ulog_timeline_append_##FIELD(ulog_timeline_t *tl, const TYPE *ev) {    \
        if (grow_if_needed((void **)&tl->FIELD, &tl->FIELD##_cap,              \
                           tl->FIELD##_count, sizeof(TYPE)) != 0)              \
            return -1;                                                         \
        tl->FIELD[tl->FIELD##_count++] = *ev;                                  \
        return 0;                                                              \
    }

DEFINE_APPEND(ulog_att_event_t,        att)
DEFINE_APPEND(ulog_lpos_event_t,       lpos)
DEFINE_APPEND(ulog_gpos_event_t,       gpos)
DEFINE_APPEND(ulog_aspd_event_t,       aspd)
DEFINE_APPEND(ulog_vstatus_event_t,    vstatus)
DEFINE_APPEND(ulog_home_event_t,       home)
DEFINE_APPEND(ulog_statustext_event_t, statustext)

#undef DEFINE_APPEND
