#ifndef REPLAY_CONFLICT_H
#define REPLAY_CONFLICT_H

#include "raylib.h"
#include "data_source.h"
#include "theme.h"
#include "scene.h"
#include <stdbool.h>

typedef struct {
    bool conflict_detected;
    bool conflict_far;
} conflict_result_t;

conflict_result_t replay_detect_conflict(const data_source_t *sources,
                                         int num_files);

int draw_prompt_dialog(const char *title, const char *subtitle,
                       const char **options, int option_count,
                       const theme_t *theme, Font font_label, Font font_value,
                       const scene_t *scene);

#endif
