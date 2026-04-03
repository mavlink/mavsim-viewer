#ifndef REPLAY_MARKERS_H
#define REPLAY_MARKERS_H

#include "raylib.h"
#include "replay_trail.h"
#include "data_source.h"
#include "vehicle.h"
#include "scene.h"
#include "hud.h"
#include <stdbool.h>

#define REPLAY_MAX_MARKERS 256

typedef struct {
    float times[REPLAY_MAX_MARKERS];
    Vector3 positions[REPLAY_MAX_MARKERS];
    char labels[REPLAY_MAX_MARKERS][HUD_MARKER_LABEL_MAX];
    float roll[REPLAY_MAX_MARKERS];
    float pitch[REPLAY_MAX_MARKERS];
    float vert[REPLAY_MAX_MARKERS];
    float speed[REPLAY_MAX_MARKERS];
    float speed_max;
    int count;
    int current;
    double last_drop_time;
    int last_drop_idx;
} user_markers_t;

int marker_drop(user_markers_t *um, float time, Vector3 pos,
                const vehicle_t *v, sys_markers_t *sm);

void marker_delete(user_markers_t *um);

void marker_cycle(user_markers_t *um, sys_markers_t *sm,
                  int direction, bool shift,
                  data_source_t *source, vehicle_t *vehicle,
                  const precomp_trail_t *precomp,
                  scene_t *scene, Vector3 *last_pos);

#endif
