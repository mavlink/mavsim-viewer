#include "replay_markers.h"
#include "replay_trail.h"
#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

int marker_drop(user_markers_t *um, float time, Vector3 pos,
                const vehicle_t *v, sys_markers_t *sm) {
    int insert = um->count;
    for (int m = 0; m < um->count; m++) {
        if (um->times[m] > time) { insert = m; break; }
    }
    for (int m = um->count; m > insert; m--) {
        um->times[m] = um->times[m - 1];
        um->positions[m] = um->positions[m - 1];
        memcpy(um->labels[m], um->labels[m - 1], HUD_MARKER_LABEL_MAX);
        um->roll[m] = um->roll[m - 1];
        um->pitch[m] = um->pitch[m - 1];
        um->vert[m] = um->vert[m - 1];
        um->speed[m] = um->speed[m - 1];
    }
    um->times[insert] = time;
    um->positions[insert] = pos;
    um->labels[insert][0] = '\0';
    um->roll[insert] = v->roll_deg;
    um->pitch[insert] = v->pitch_deg;
    um->vert[insert] = v->vertical_speed;
    um->speed[insert] = sqrtf(v->ground_speed * v->ground_speed +
                               v->vertical_speed * v->vertical_speed);
    um->count++;
    um->current = insert;
    sm->selected = false;
    sm->current = -1;
    um->last_drop_time = GetTime();
    um->last_drop_idx = insert;
    return insert;
}

void marker_delete(user_markers_t *um) {
    if (um->current < 0 || um->current >= um->count) return;
    for (int m = um->current; m < um->count - 1; m++) {
        um->times[m] = um->times[m + 1];
        um->positions[m] = um->positions[m + 1];
        memcpy(um->labels[m], um->labels[m + 1], HUD_MARKER_LABEL_MAX);
        um->roll[m] = um->roll[m + 1];
        um->pitch[m] = um->pitch[m + 1];
        um->vert[m] = um->vert[m + 1];
        um->speed[m] = um->speed[m + 1];
    }
    um->count--;
    if (um->current >= um->count) um->current = um->count - 1;
    um->last_drop_idx = -1;
}

void marker_cycle(user_markers_t *um, sys_markers_t *sm,
                  int direction, bool shift,
                  data_source_t *source, vehicle_t *vehicle,
                  const precomp_trail_t *precomp,
                  scene_t *scene, Vector3 *last_pos) {
    int total = um->count + sm->count;
    if (total == 0) return;

    typedef struct { float time; int idx; bool is_sys; } merged_marker_t;
    merged_marker_t merged[REPLAY_MAX_MARKERS + REPLAY_MAX_SYS_MARKERS];
    int mi = 0;
    for (int m = 0; m < um->count; m++) {
        merged[mi++] = (merged_marker_t){um->times[m], m, false};
    }
    for (int m = 0; m < sm->count; m++) {
        merged[mi++] = (merged_marker_t){sm->times[m], m, true};
    }
    for (int a = 1; a < total; a++) {
        merged_marker_t key = merged[a];
        int b = a - 1;
        while (b >= 0 && merged[b].time > key.time) {
            merged[b + 1] = merged[b];
            b--;
        }
        merged[b + 1] = key;
    }

    int cur_merged = -1;
    if (sm->selected && sm->current >= 0) {
        for (int m = 0; m < total; m++) {
            if (merged[m].is_sys && merged[m].idx == sm->current) {
                cur_merged = m; break;
            }
        }
    } else if (!sm->selected && um->current >= 0) {
        for (int m = 0; m < total; m++) {
            if (!merged[m].is_sys && merged[m].idx == um->current) {
                cur_merged = m; break;
            }
        }
    }

    int target_merged = -1;
    if (direction < 0) {
        // Previous
        if (cur_merged > 0) {
            target_merged = cur_merged - 1;
        } else if (cur_merged == 0) {
            target_merged = total - 1;
        } else {
            float t = source->playback.position_s;
            for (int m = total - 1; m >= 0; m--) {
                if (merged[m].time <= t) { target_merged = m; break; }
            }
            if (target_merged < 0) target_merged = total - 1;
        }
    } else {
        // Next
        if (cur_merged >= 0 && cur_merged < total - 1) {
            target_merged = cur_merged + 1;
        } else if (cur_merged < 0) {
            float t = source->playback.position_s;
            for (int m = 0; m < total; m++) {
                if (merged[m].time >= t) { target_merged = m; break; }
            }
            if (target_merged < 0) target_merged = 0;
        } else {
            target_merged = 0;
        }
    }

    if (target_merged >= 0 && target_merged < total) {
        merged_marker_t tgt = merged[target_merged];
        float seek_time = tgt.time;
        Vector3 seek_pos = tgt.is_sys ? sm->positions[tgt.idx] : um->positions[tgt.idx];

        sm->selected = tgt.is_sys;
        if (tgt.is_sys) {
            sm->current = tgt.idx;
            um->current = -1;
        } else {
            um->current = tgt.idx;
            sm->current = -1;
        }

        if (shift) {
            scene->cam_mode = CAM_MODE_FREE;
            scene->free_track = true;
            scene->camera.position = seek_pos;
            scene->camera.target = vehicle->position;
        } else {
            // Return to chase camera if free mode was active
            if (scene->cam_mode == CAM_MODE_FREE) {
                scene->cam_mode = CAM_MODE_CHASE;
                scene->free_track = false;
            }
            // Seek and sync state
            data_source_seek(source, seek_time);
            vehicle->current_time = source->playback.position_s;

            // Restore trail from pre-computed data up to seek_time
            if (precomp->ready && precomp->count > 0) {
                int lo = 0, hi = precomp->count - 1, cut = 0;
                while (lo <= hi) {
                    int mid = (lo + hi) / 2;
                    if (precomp->time[mid] <= seek_time) {
                        cut = mid + 1;
                        lo = mid + 1;
                    } else {
                        hi = mid - 1;
                    }
                }
                int n = cut;
                if (n > vehicle->trail_capacity) n = vehicle->trail_capacity;
                int src_start = cut - n;
                vehicle->trail_count = n;
                vehicle->trail_head = n % vehicle->trail_capacity;
                vehicle->trail_speed_max = 0.0f;
                for (int ti = 0; ti < n; ti++) {
                    int si = src_start + ti;
                    vehicle->trail[ti] = precomp->trail[si];
                    vehicle->trail_roll[ti] = precomp->roll[si];
                    vehicle->trail_pitch[ti] = precomp->pitch[si];
                    vehicle->trail_vert[ti] = precomp->vert[si];
                    vehicle->trail_speed[ti] = precomp->speed[si];
                    vehicle->trail_time[ti] = precomp->time[si];
                    if (precomp->speed[si] > vehicle->trail_speed_max)
                        vehicle->trail_speed_max = precomp->speed[si];
                }
            }
            // Update vehicle state and suppress jump detector on next frame
            vehicle_update(vehicle, &source->state, &source->home);
            *last_pos = vehicle->position;
        }
    }
}

marker_cycle_result_t marker_cycle_global(
    user_markers_t *all_markers, sys_markers_t *all_sys,
    int vehicle_count, int current_drone, int direction,
    data_source_t *sources, vehicle_t *vehicles,
    const precomp_trail_t *precomps,
    scene_t *scene, Vector3 *last_pos)
{
    marker_cycle_result_t result = {current_drone, false};

    // Build merged list of all drones' markers
    typedef struct { float time; int idx; int drone; bool is_sys; } gm_t;
    int max_entries = vehicle_count * (REPLAY_MAX_MARKERS + REPLAY_MAX_SYS_MARKERS);
    gm_t *merged = (gm_t *)malloc(max_entries * sizeof(gm_t));
    if (!merged) return result;

    int total = 0;
    for (int d = 0; d < vehicle_count; d++) {
        for (int m = 0; m < all_markers[d].count; m++)
            merged[total++] = (gm_t){all_markers[d].times[m], m, d, false};
        for (int m = 0; m < all_sys[d].count; m++)
            merged[total++] = (gm_t){all_sys[d].times[m], m, d, true};
    }

    if (total == 0) { free(merged); return result; }

    // Insertion sort by time
    for (int a = 1; a < total; a++) {
        gm_t key = merged[a];
        int b = a - 1;
        while (b >= 0 && merged[b].time > key.time) {
            merged[b + 1] = merged[b];
            b--;
        }
        merged[b + 1] = key;
    }

    // Find current position: check if any drone has a selected marker
    int cur = -1;
    for (int d = 0; d < vehicle_count; d++) {
        if (all_sys[d].selected && all_sys[d].current >= 0) {
            for (int m = 0; m < total; m++) {
                if (merged[m].is_sys && merged[m].drone == d
                    && merged[m].idx == all_sys[d].current) {
                    cur = m; break;
                }
            }
            if (cur >= 0) break;
        } else if (!all_sys[d].selected && all_markers[d].current >= 0) {
            for (int m = 0; m < total; m++) {
                if (!merged[m].is_sys && merged[m].drone == d
                    && merged[m].idx == all_markers[d].current) {
                    cur = m; break;
                }
            }
            if (cur >= 0) break;
        }
    }

    // Step to target
    int target = -1;
    if (direction < 0) {
        if (cur > 0) target = cur - 1;
        else if (cur == 0) target = total - 1;
        else {
            float t = sources[current_drone].playback.position_s;
            for (int m = total - 1; m >= 0; m--) {
                if (merged[m].time <= t) { target = m; break; }
            }
            if (target < 0) target = total - 1;
        }
    } else {
        if (cur >= 0 && cur < total - 1) target = cur + 1;
        else if (cur < 0) {
            float t = sources[current_drone].playback.position_s;
            for (int m = 0; m < total; m++) {
                if (merged[m].time >= t) { target = m; break; }
            }
            if (target < 0) target = 0;
        } else {
            target = 0;
        }
    }

    if (target >= 0 && target < total) {
        gm_t tgt = merged[target];
        int d = tgt.drone;
        float seek_time = tgt.time;

        // Clear all drones' marker selection
        for (int i = 0; i < vehicle_count; i++) {
            all_sys[i].selected = false;
            all_sys[i].current = -1;
            all_markers[i].current = -1;
        }

        // Set target drone's marker as current
        all_sys[d].selected = tgt.is_sys;
        if (tgt.is_sys) {
            all_sys[d].current = tgt.idx;
        } else {
            all_markers[d].current = tgt.idx;
        }

        // Seek only the target drone's source and restore its trail
        data_source_seek(&sources[d], seek_time);
        vehicles[d].current_time = sources[d].playback.position_s;

        if (precomps[d].ready && precomps[d].count > 0) {
            int lo = 0, hi = precomps[d].count - 1, cut = 0;
            while (lo <= hi) {
                int mid = (lo + hi) / 2;
                if (precomps[d].time[mid] <= seek_time) {
                    cut = mid + 1; lo = mid + 1;
                } else {
                    hi = mid - 1;
                }
            }
            int n = cut;
            if (n > vehicles[d].trail_capacity) n = vehicles[d].trail_capacity;
            int src_start = cut - n;
            vehicles[d].trail_count = n;
            vehicles[d].trail_head = n % vehicles[d].trail_capacity;
            vehicles[d].trail_speed_max = 0.0f;
            for (int ti = 0; ti < n; ti++) {
                int si = src_start + ti;
                vehicles[d].trail[ti] = precomps[d].trail[si];
                vehicles[d].trail_roll[ti] = precomps[d].roll[si];
                vehicles[d].trail_pitch[ti] = precomps[d].pitch[si];
                vehicles[d].trail_vert[ti] = precomps[d].vert[si];
                vehicles[d].trail_speed[ti] = precomps[d].speed[si];
                vehicles[d].trail_time[ti] = precomps[d].time[si];
                if (precomps[d].speed[si] > vehicles[d].trail_speed_max)
                    vehicles[d].trail_speed_max = precomps[d].speed[si];
            }
        }

        vehicle_update(&vehicles[d], &sources[d].state, &sources[d].home);
        last_pos[d] = vehicles[d].position;

        result.drone_idx = d;
        result.jumped = true;
    }

    free(merged);
    return result;
}
