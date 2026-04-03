#include "replay_trail.h"
#include "ulog_replay.h"
#include "raylib.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void replay_sync_vehicle(data_source_t *src, vehicle_t *veh) {
    veh->current_time = src->playback.position_s;
    vehicle_update(veh, &src->state, &src->home);
    vehicle_truncate_trail(veh, src->playback.position_s);
}

void replay_init_sys_markers(sys_markers_t *sm, const data_source_t *source) {
    const playback_state_t *pb = &source->playback;
    int count = pb->mode_change_count;
    if (count > REPLAY_MAX_SYS_MARKERS) count = REPLAY_MAX_SYS_MARKERS;
    if (count > 0) {
        for (int i = 0; i < count; i++) {
            sm->times[i] = pb->mode_changes[i].time_s;
            const char *name = ulog_nav_state_name(pb->mode_changes[i].nav_state);
            snprintf(sm->labels[i], HUD_MARKER_LABEL_MAX, "%s", name);
            sm->positions[i] = (Vector3){0};
        }
        sm->count = count;
    }
}

void replay_resolve_and_build_trail(sys_markers_t *sm, precomp_trail_t *pt,
                                    data_source_t *source, vehicle_t *vehicle) {
    float saved_pos = source->playback.position_s;
    int valid = 0;
    for (int i = 0; i < sm->count; i++) {
        data_source_seek(source, sm->times[i]);
        replay_sync_vehicle(source, vehicle);
        if (source->state.lat == 0 && source->state.lon == 0) continue;
        Vector3 mp = vehicle->position;
        if (mp.x == 0.0f && mp.y == 0.0f && mp.z == 0.0f) continue;
        float mdist = sqrtf(mp.x * mp.x + mp.y * mp.y + mp.z * mp.z);
        if (mdist > 5000.0f) continue;
        sm->times[valid] = sm->times[i];
        memcpy(sm->labels[valid], sm->labels[i], HUD_MARKER_LABEL_MAX);
        sm->positions[valid] = vehicle->position;
        sm->roll[valid] = vehicle->roll_deg;
        sm->pitch[valid] = vehicle->pitch_deg;
        sm->vert[valid] = vehicle->vertical_speed;
        sm->speed[valid] = sqrtf(vehicle->ground_speed * vehicle->ground_speed +
                                  vehicle->vertical_speed * vehicle->vertical_speed);
        valid++;
    }
    sm->count = valid;

    // Pre-compute entire flight trail from ULog position data
    if (!pt->ready) {
        pt->trail = calloc(PRECOMP_TRAIL_MAX, sizeof(Vector3));
        pt->roll  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
        pt->pitch = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
        pt->vert  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
        pt->speed = calloc(PRECOMP_TRAIL_MAX, sizeof(float));
        pt->time  = calloc(PRECOMP_TRAIL_MAX, sizeof(float));

        if (pt->trail && pt->roll && pt->pitch &&
            pt->vert && pt->speed && pt->time) {
            double lat0 = vehicle->lat0;
            double lon0 = vehicle->lon0;
            double alt0 = vehicle->alt0;
            float cos_lat0 = (float)cos(lat0);
            Vector3 prev_pos = {0};
            bool prev_valid = false;
            int pc = 0;

            // Save and override playback controls for pre-computation
            playback_state_t saved_pb = source->playback;
            source->playback.speed = 1.0f;
            source->playback.paused = false;
            source->playback.looping = false;
            source->playback.interpolation = false;

            data_source_seek(source, 0.0f);
            float step = 0.2f;
            while (pc < PRECOMP_TRAIL_MAX) {
                float prev_wall = source->playback.position_s;
                data_source_poll(source, step);
                if (!source->connected || source->playback.position_s <= prev_wall) break;

                hil_state_t *st = &source->state;
                if (!st->valid) continue;
                if (st->lat == 0 && st->lon == 0) continue;

                double lat = st->lat * 1e-7 * (M_PI / 180.0);
                double lon = st->lon * 1e-7 * (M_PI / 180.0);
                double alt = st->alt * 1e-3;
                double ned_x = 6371000.0 * (lat - lat0);
                double ned_y = 6371000.0 * (lon - lon0) * cos_lat0;
                double ned_z = alt - alt0;
                Vector3 pos = {
                    (float)ned_y,
                    (float)ned_z < 0.0f ? 0.0f : (float)ned_z,
                    (float)(-ned_x)
                };

                if (prev_valid) {
                    float dx = pos.x - prev_pos.x;
                    float dy = pos.y - prev_pos.y;
                    float dz = pos.z - prev_pos.z;
                    if (dx*dx + dy*dy + dz*dz < 0.25f) continue;
                }

                float qw = st->quaternion[0], qx = st->quaternion[1];
                float qy = st->quaternion[2], qz = st->quaternion[3];
                if (qw < 0) { qw = -qw; qx = -qx; qy = -qy; qz = -qz; }
                float roll_v = atan2f(2.0f*(qw*qx + qy*qz),
                                    1.0f - 2.0f*(qx*qx + qy*qy)) * RAD2DEG;
                float sin_p = 2.0f*(qw*qy - qz*qx);
                if (sin_p > 1.0f) sin_p = 1.0f;
                if (sin_p < -1.0f) sin_p = -1.0f;
                float pitch_v = asinf(sin_p) * RAD2DEG;
                float vert_s = -st->vz * 0.01f;
                float gs = sqrtf((float)st->vx*st->vx + (float)st->vy*st->vy) * 0.01f;
                float spd = sqrtf(gs*gs + vert_s*vert_s);

                pt->trail[pc] = pos;
                pt->roll[pc] = roll_v;
                pt->pitch[pc] = pitch_v;
                pt->vert[pc] = vert_s;
                pt->speed[pc] = spd;
                pt->time[pc] = source->playback.position_s;
                if (spd > pt->speed_max) pt->speed_max = spd;
                pc++;
                prev_pos = pos;
                prev_valid = true;
            }
            pt->count = pc;
            pt->ready = true;

            // Restore playback controls
            source->playback = saved_pb;
        }
    }

    // Restore playback to where it was
    data_source_seek(source, saved_pos);
    vehicle_reset_trail(vehicle);
    replay_sync_vehicle(source, vehicle);
    sm->resolved = true;
}

void precomp_trail_init(precomp_trail_t *t) {
    memset(t, 0, sizeof(*t));
}

void precomp_trail_cleanup(precomp_trail_t *t) {
    free(t->trail); free(t->roll); free(t->pitch);
    free(t->vert); free(t->speed); free(t->time);
    memset(t, 0, sizeof(*t));
}
