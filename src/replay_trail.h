#ifndef REPLAY_TRAIL_H
#define REPLAY_TRAIL_H

#include "raylib.h"
#include "data_source.h"
#include "vehicle.h"
#include "hud.h"
#include <stdbool.h>

#define PRECOMP_TRAIL_MAX 36000
#define REPLAY_MAX_SYS_MARKERS 256

typedef struct {
    Vector3 *trail;
    float *roll;
    float *pitch;
    float *vert;
    float *speed;
    float *time;
    int count;
    float speed_max;
    bool ready;
} precomp_trail_t;

typedef struct {
    float times[REPLAY_MAX_SYS_MARKERS];
    Vector3 positions[REPLAY_MAX_SYS_MARKERS];
    char labels[REPLAY_MAX_SYS_MARKERS][HUD_MARKER_LABEL_MAX];
    float roll[REPLAY_MAX_SYS_MARKERS];
    float pitch[REPLAY_MAX_SYS_MARKERS];
    float vert[REPLAY_MAX_SYS_MARKERS];
    float speed[REPLAY_MAX_SYS_MARKERS];
    int count;
    int current;
    bool selected;
    bool resolved;
} sys_markers_t;

void replay_sync_vehicle(data_source_t *src, vehicle_t *veh);

void replay_init_sys_markers(sys_markers_t *sm, const data_source_t *source);

void replay_resolve_and_build_trail(sys_markers_t *sm, precomp_trail_t *trail,
                                    data_source_t *source, vehicle_t *vehicle);

void precomp_trail_init(precomp_trail_t *t);
void precomp_trail_cleanup(precomp_trail_t *t);

#endif
