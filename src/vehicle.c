#include "vehicle.h"
#include "theme.h"
#include "asset_path.h"
#include "raymath.h"
#include "rlgl.h"
#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define EARTH_RADIUS 6371000.0
#define TRAIL_MAX 2700
#define TRAIL_INTERVAL 0.016f

/* Trail LOD distance thresholds (squared) and skip counts */
#define TRAIL_LOD_DIST_FAR_SQ  2250000.0f  /* 1500 m squared */
#define TRAIL_LOD_DIST_MED_SQ   160000.0f  /*  400 m squared */
#define TRAIL_LOD_SKIP_FAR  3
#define TRAIL_LOD_SKIP_MED  1

/* Return number of trail segments to skip based on midpoint distance to camera. */
static int trail_lod_skip(Vector3 a, Vector3 b, Vector3 cam)
{
    float dx = (a.x + b.x) * 0.5f - cam.x;
    float dy = (a.y + b.y) * 0.5f - cam.y;
    float dz = (a.z + b.z) * 0.5f - cam.z;
    float dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq > TRAIL_LOD_DIST_FAR_SQ) return TRAIL_LOD_SKIP_FAR;
    if (dist_sq > TRAIL_LOD_DIST_MED_SQ) return TRAIL_LOD_SKIP_MED;
    return 0;
}

#define TRAIL_DIST_INTERVAL 0.01f  // meters between ribbon samples

// ── Model registry ──────────────────────────────────────────────────────────
// To add a new model: append an entry here and increment nothing else.
const vehicle_model_info_t vehicle_models[] = {
    //  path                         name          scale  roll  pitch  yaw    group              mirror  width
    { "models/quad_quarter.obj",   "Quadrotor",   1.0f, -90.0f, 0.0f, 0.0f, GROUP_QUAD,        2,     0.6f },
    { "models/px4_fixed_wing.obj", "Fixed-wing",  1.15f, 0.0f, 0.0f, 180.0f, GROUP_FIXED_WING,  0,     2.0f },
    { "models/tailsitter3.obj",    "Tailsitter",  1.0f, -90.0f, 0.0f, 0.0f, GROUP_TAILSITTER,  1,     1.2f },
    { "models/quadfpv9.obj",       "FPV Quad",    0.75f, -90.0f, 0.0f, 0.0f, GROUP_QUAD,        1,     0.3f },
    { "models/hex10.obj",          "Hexarotor",   0.9f, -90.0f, 0.0f,  0.0f, GROUP_HEX,         1,     1.0f },
    { "models/hexfpv2.obj",        "FPV Hex",     0.85f, -90.0f, 0.0f, 0.0f, GROUP_HEX,         1,     0.3f },
    { "models/vtol4.obj",          "VTOL",        1.5f, -90.0f, 0.0f, 0.0f, GROUP_VTOL,        1,     2.0f },
    { "models/rover7.obj",         "Rover",       1.0f, -90.0f, 0.0f,  0.0f, GROUP_ROVER,       2,     0.5f },
    { "models/rov5.obj",           "ROV",         1.0f, -90.0f, 0.0f, 0.0f, GROUP_ROV,         1,     0.75f },
};
const int vehicle_model_count = sizeof(vehicle_models) / sizeof(vehicle_models[0]);

// ── Material name classification ────────────────────────────────────────────
static mat_role_t classify_material_name(const char *name) {
    if (!name) return MAT_ROLE_NONE;
    if (strncmp(name, "Port", 4) == 0 || strncmp(name, "Red_", 4) == 0)
        return MAT_ROLE_PORT;
    if (strncmp(name, "Starboard", 9) == 0 || strncmp(name, "Green_", 6) == 0)
        return MAT_ROLE_STARBOARD;
    if (strncmp(name, "Fore", 4) == 0 || strncmp(name, "Yellow_", 7) == 0)
        return MAT_ROLE_FORE;
    if (strncmp(name, "Aft", 3) == 0 || strncmp(name, "Purple_", 7) == 0
        || strncmp(name, "Back_", 5) == 0)
        return MAT_ROLE_AFT;
    if (strncmp(name, "Body_Mid", 8) == 0)
        return MAT_ROLE_BODY_MID;
    if (strncmp(name, "Wing", 4) == 0 || strncmp(name, "Body", 4) == 0
        || strncmp(name, "Fuse", 4) == 0)
        return MAT_ROLE_BODY;
    if (strncmp(name, "Prop", 4) == 0)
        return MAT_ROLE_PROP;
    if (strncmp(name, "Motor", 5) == 0 || strncmp(name, "Motror", 6) == 0)
        return MAT_ROLE_MOTOR;
    return MAT_ROLE_NONE;
}

// ── MTL pre-parser ─────────────────────────────────────────────────────────
#define MTL_MAX_ENTRIES 32

typedef struct {
    char name[64];
    Color kd;
    mat_role_t role;
} mtl_entry_t;

static int parse_mtl_names(const char *obj_path, mtl_entry_t *entries, int max_entries) {
    // Derive MTL path from OBJ path (replace .obj with .mtl)
    char mtl_path[512];
    snprintf(mtl_path, sizeof(mtl_path), "%s", obj_path);
    int len = (int)strlen(mtl_path);
    if (len > 4 && strcmp(mtl_path + len - 4, ".obj") == 0) {
        mtl_path[len - 3] = 'm';
        mtl_path[len - 2] = 't';
        mtl_path[len - 1] = 'l';
    } else {
        return 0;
    }

    FILE *fp = fopen(mtl_path, "r");
    if (!fp) return 0;

    int count = 0;
    char current_name[64] = {0};
    char line[256];

    while (fgets(line, sizeof(line), fp) && count < max_entries) {
        // Strip newline
        int ll = (int)strlen(line);
        while (ll > 0 && (line[ll-1] == '\n' || line[ll-1] == '\r'))
            line[--ll] = '\0';

        if (strncmp(line, "newmtl ", 7) == 0) {
            snprintf(current_name, sizeof(current_name), "%s", line + 7);
        } else if (strncmp(line, "Kd ", 3) == 0 && current_name[0]) {
            float r = 0, g = 0, b = 0;
            if (sscanf(line + 3, "%f %f %f", &r, &g, &b) == 3) {
                mtl_entry_t *e = &entries[count];
                snprintf(e->name, sizeof(e->name), "%s", current_name);
                e->kd = (Color){
                    (unsigned char)(r * 255.0f),
                    (unsigned char)(g * 255.0f),
                    (unsigned char)(b * 255.0f),
                    255
                };
                e->role = classify_material_name(current_name);
                count++;
            }
            current_name[0] = '\0';
        }
    }

    fclose(fp);
    return count;
}

// ── Material remapping ──────────────────────────────────────────────────────
// Two-pass: name-based matching first, color heuristic fallback for legacy models.
static void remap_materials(vehicle_t *v) {
    v->red_material_idx = -1;
    v->green_material_idx = -1;
    v->fore_material_idx = -1;
    v->aft_material_idx = -1;

    // Build MTL path from model path
    char model_path[512];
    asset_path(vehicle_models[v->model_idx].path, model_path, sizeof(model_path));

    // Pass 1: name-based matching via MTL pre-parse
    mtl_entry_t mtl_entries[MTL_MAX_ENTRIES];
    int mtl_count = parse_mtl_names(model_path, mtl_entries, MTL_MAX_ENTRIES);
    bool name_matched = false;

    if (mtl_count > 0) {
        printf("  MTL parse: %d entries\n", mtl_count);
        for (int m = 0; m < mtl_count; m++)
            printf("    MTL[%d] '%s' kd=(%d,%d,%d) role=%d\n",
                   m, mtl_entries[m].name, mtl_entries[m].kd.r,
                   mtl_entries[m].kd.g, mtl_entries[m].kd.b, mtl_entries[m].role);
        for (int i = 0; i < v->model.materialCount; i++) {
            Color mc = v->model.materials[i].maps[MATERIAL_MAP_DIFFUSE].color;
            printf("  Raylib mat[%d] color=(%d,%d,%d)", i, mc.r, mc.g, mc.b);
            bool found = false;
            // Find MTL entry matching this material's diffuse color
            for (int m = 0; m < mtl_count; m++) {
                if (mc.r == mtl_entries[m].kd.r &&
                    mc.g == mtl_entries[m].kd.g &&
                    mc.b == mtl_entries[m].kd.b) {
                    mat_role_t role = mtl_entries[m].role;
                    if (v->material_roles) v->material_roles[i] = role;
                    switch (role) {
                        case MAT_ROLE_PORT:      v->red_material_idx = i; break;
                        case MAT_ROLE_STARBOARD: v->green_material_idx = i; break;
                        case MAT_ROLE_FORE:      v->fore_material_idx = i; break;
                        case MAT_ROLE_AFT:       v->aft_material_idx = i; break;
                        default: break;
                    }
                    name_matched = true;
                    found = true;
                    printf(" -> '%s' role=%d", mtl_entries[m].name, role);
                    break;
                }
            }
            if (!found) printf(" -> NO MATCH");
            printf("\n");
        }
    }

    // Pass 2: color heuristic fallback for legacy models
    if (!name_matched) {
        for (int i = 0; i < v->model.materialCount; i++) {
            Color *c = &v->model.materials[i].maps[MATERIAL_MAP_DIFFUSE].color;
            if (c->r > 200 && c->g > 100 && c->b < 100) {
                *c = (Color){ 255, 200, 50, 255 };
                v->fore_material_idx = i;
                if (v->material_roles) v->material_roles[i] = MAT_ROLE_FORE;
            } else if (c->b > 200 && c->r > 100 && c->g < 100) {
                *c = (Color){ 160, 60, 255, 255 };
                v->aft_material_idx = i;
                if (v->material_roles) v->material_roles[i] = MAT_ROLE_AFT;
            } else if (c->g > 200 && c->r < 100 && c->b < 100) {
                *c = (Color){ 41, 255, 79, 255 };
                v->green_material_idx = i;
                if (v->material_roles) v->material_roles[i] = MAT_ROLE_STARBOARD;
            } else if (c->r > 100 && c->g < 50 && c->b < 50) {
                *c = (Color){ 204, 33, 33, 255 };
                v->red_material_idx = i;
                if (v->material_roles) v->material_roles[i] = MAT_ROLE_PORT;
            } else if (c->b > 100 && c->r < 50) {
                *c = (Color){ 39, 47, 197, 255 };
                if (v->material_roles) v->material_roles[i] = MAT_ROLE_BODY;
            } else if (c->r > 60 && c->r < 140 && c->g > 60 && c->g < 140) {
                *c = (Color){ 80, 117, 162, 255 };
                if (v->material_roles) v->material_roles[i] = MAT_ROLE_PROP;
            } else if (c->r < 20 && c->g < 20 && c->b < 20) {
                *c = (Color){ 14, 31, 47, 255 };
                if (v->material_roles) v->material_roles[i] = MAT_ROLE_BODY;
            }
        }
    }

    // Assign lighting shader to all materials
    if (v->lighting_shader.id > 0) {
        for (int i = 0; i < v->model.materialCount; i++) {
            v->model.materials[i].shader = v->lighting_shader;
        }
    }
}

// ── Runtime mesh mirroring ──────────────────────────────────────────────────

// Duplicate a mesh with one axis negated, normals flipped, winding reversed.
static Mesh duplicate_mesh_mirrored(Mesh src, int axis) {
    // axis: 0=X, 2=Z
    Mesh dst = {0};
    dst.vertexCount = src.vertexCount;
    dst.triangleCount = src.triangleCount;

    // Vertices — negate the specified axis
    if (src.vertices) {
        dst.vertices = (float *)RL_MALLOC(src.vertexCount * 3 * sizeof(float));
        memcpy(dst.vertices, src.vertices, src.vertexCount * 3 * sizeof(float));
        for (int i = 0; i < src.vertexCount; i++)
            dst.vertices[i * 3 + axis] = -dst.vertices[i * 3 + axis];
    }

    // Normals — negate the same axis
    if (src.normals) {
        dst.normals = (float *)RL_MALLOC(src.vertexCount * 3 * sizeof(float));
        memcpy(dst.normals, src.normals, src.vertexCount * 3 * sizeof(float));
        for (int i = 0; i < src.vertexCount; i++)
            dst.normals[i * 3 + axis] = -dst.normals[i * 3 + axis];
    }

    // Texcoords — unchanged
    if (src.texcoords) {
        dst.texcoords = (float *)RL_MALLOC(src.vertexCount * 2 * sizeof(float));
        memcpy(dst.texcoords, src.texcoords, src.vertexCount * 2 * sizeof(float));
    }

    // Colors — unchanged
    if (src.colors) {
        dst.colors = (unsigned char *)RL_MALLOC(src.vertexCount * 4);
        memcpy(dst.colors, src.colors, src.vertexCount * 4);
    }

    // Indices — reverse winding
    if (src.indices) {
        dst.indices = (unsigned short *)RL_MALLOC(src.triangleCount * 3 * sizeof(unsigned short));
        for (int t = 0; t < src.triangleCount; t++) {
            dst.indices[t * 3 + 0] = src.indices[t * 3 + 0];
            dst.indices[t * 3 + 1] = src.indices[t * 3 + 2];
            dst.indices[t * 3 + 2] = src.indices[t * 3 + 1];
        }
    } else {
        // Non-indexed: swap vertices within each triangle triple
        for (int t = 0; t < src.triangleCount; t++) {
            int b = t * 3;
            for (int c = 0; c < 3; c++) {
                float tmp = dst.vertices[(b + 1) * 3 + c];
                dst.vertices[(b + 1) * 3 + c] = dst.vertices[(b + 2) * 3 + c];
                dst.vertices[(b + 2) * 3 + c] = tmp;
            }
            if (dst.normals) {
                for (int c = 0; c < 3; c++) {
                    float tmp = dst.normals[(b + 1) * 3 + c];
                    dst.normals[(b + 1) * 3 + c] = dst.normals[(b + 2) * 3 + c];
                    dst.normals[(b + 2) * 3 + c] = tmp;
                }
            }
            if (dst.texcoords) {
                for (int c = 0; c < 2; c++) {
                    float tmp = dst.texcoords[(b + 1) * 2 + c];
                    dst.texcoords[(b + 1) * 2 + c] = dst.texcoords[(b + 2) * 2 + c];
                    dst.texcoords[(b + 2) * 2 + c] = tmp;
                }
            }
        }
    }

    UploadMesh(&dst, false);
    return dst;
}

// Get the counterpart role for mirroring
static mat_role_t mirror_role_x(mat_role_t role) {
    if (role == MAT_ROLE_PORT) return MAT_ROLE_STARBOARD;
    if (role == MAT_ROLE_STARBOARD) return MAT_ROLE_PORT;
    return role;
}

static mat_role_t mirror_role_z(mat_role_t role) {
    if (role == MAT_ROLE_FORE) return MAT_ROLE_AFT;
    if (role == MAT_ROLE_AFT) return MAT_ROLE_FORE;
    return role;
}

// Mirror model along an axis. No new materials created — uses per-mesh role overrides.
static void mirror_model_axis(vehicle_t *v, int axis,
                               mat_role_t (*role_swap)(mat_role_t)) {
    Model *model = &v->model;
    int orig_count = model->meshCount;
    int new_count = orig_count * 2;

    // Expand mesh arrays
    model->meshes = (Mesh *)RL_REALLOC(model->meshes, new_count * sizeof(Mesh));
    model->meshMaterial = (int *)RL_REALLOC(model->meshMaterial, new_count * sizeof(int));

    // Expand mesh_roles
    v->mesh_roles = (mat_role_t *)realloc(v->mesh_roles, new_count * sizeof(mat_role_t));

    for (int i = 0; i < orig_count; i++) {
        model->meshes[orig_count + i] = duplicate_mesh_mirrored(model->meshes[i], axis);
        model->meshMaterial[orig_count + i] = model->meshMaterial[i]; // same material

        // Mirrored mesh gets swapped role
        int orig_mat = model->meshMaterial[i];
        mat_role_t orig_role = v->mesh_roles[i];
        v->mesh_roles[orig_count + i] = role_swap(orig_role);
    }

    model->meshCount = new_count;
    v->mesh_roles_count = new_count;
}

// ── Model loading ───────────────────────────────────────────────────────────
void vehicle_load_model(vehicle_t *v, int model_idx) {
    // Clamp to valid range
    if (model_idx < 0 || model_idx >= vehicle_model_count)
        model_idx = 0;

    // Unload previous model if loaded.
    // Reset material shaders to default first — UnloadModel calls UnloadMaterial
    // which destroys any non-default shader. Our shared lighting_shader would be
    // deleted, hanging the GPU on subsequent draws.
    if (v->model.meshCount > 0) {
        for (int i = 0; i < v->model.materialCount; i++)
            v->model.materials[i].shader.id = rlGetShaderIdDefault();
        UnloadModel(v->model);
    }

    // Free previous material_roles if reloading
    if (v->material_roles) {
        free(v->material_roles);
        v->material_roles = NULL;
    }

    const vehicle_model_info_t *info = &vehicle_models[model_idx];
    v->model_idx = model_idx;
    v->model_scale = info->scale;
    v->roll_offset_deg = info->roll_offset_deg;
    v->pitch_offset_deg = info->pitch_offset_deg;
    v->yaw_offset_deg = info->yaw_offset_deg;

    char model_path[512];
    asset_path(info->path, model_path, sizeof(model_path));
    v->model = LoadModel(model_path);
    if (v->model.meshCount == 0)
        printf("Warning: failed to load model %s\n", info->path);

    // Allocate material roles array
    v->material_roles = (mat_role_t *)calloc(v->model.materialCount, sizeof(mat_role_t));

    remap_materials(v);

    // Initialize per-mesh roles from material roles
    free(v->mesh_roles);
    v->mesh_roles = (mat_role_t *)calloc(v->model.meshCount, sizeof(mat_role_t));
    v->mesh_roles_count = v->model.meshCount;
    for (int i = 0; i < v->model.meshCount; i++) {
        int mat = v->model.meshMaterial[i];
        v->mesh_roles[i] = v->material_roles[mat];
    }

    // Runtime mirroring — no new materials, uses per-mesh role overrides
    if (info->mirror_axes >= 1)
        mirror_model_axis(v, 0, mirror_role_x);
    if (info->mirror_axes >= 2)
        mirror_model_axis(v, 2, mirror_role_z);

    // Debug prints removed
    printf("Model: %s\n", info->name);
}

void vehicle_cycle_model(vehicle_t *v) {
    model_group_t group = v->model_group;
    int start = v->model_idx;
    int next = start;
    do {
        next = (next + 1) % vehicle_model_count;
    } while (vehicle_models[next].group != group && next != start);
    if (next != start) {
        vehicle_load_model(v, next);
    }
}

void vehicle_set_type(vehicle_t *v, uint8_t mav_type) {
    model_group_t group;
    int default_model;

    switch (mav_type) {
        case 2:  // MAV_TYPE_QUADROTOR
            group = GROUP_QUAD;
            default_model = MODEL_QUADROTOR;
            break;
        case 13: // MAV_TYPE_HEXAROTOR
        case 14: // MAV_TYPE_OCTOROTOR
            group = GROUP_HEX;
            default_model = MODEL_HEXAROTOR;
            break;
        case 1:  // MAV_TYPE_FIXED_WING
            group = GROUP_FIXED_WING;
            default_model = MODEL_FIXEDWING;
            break;
        case 19: // MAV_TYPE_VTOL_TAILSITTER_DUOROTOR
        case 20: // MAV_TYPE_VTOL_TAILSITTER_QUADROTOR
        case 23: // MAV_TYPE_VTOL_TAILSITTER
            group = GROUP_TAILSITTER;
            default_model = MODEL_TAILSITTER;
            break;
        case 21: // MAV_TYPE_VTOL_TILTROTOR
        case 22: // MAV_TYPE_VTOL_FIXEDROTOR
            group = GROUP_VTOL;
            default_model = MODEL_VTOL;
            break;
        case 10: // MAV_TYPE_GROUND_ROVER
        case 11: // MAV_TYPE_SURFACE_BOAT
            group = GROUP_ROVER;
            default_model = MODEL_ROVER;
            break;
        case 12: // MAV_TYPE_SUBMARINE
            group = GROUP_ROV;
            default_model = MODEL_ROV;
            break;
        default:
            group = GROUP_QUAD;
            default_model = MODEL_QUADROTOR;
            break;
    }

    v->model_group = group;
    if (v->model_idx != default_model) {
        vehicle_load_model(v, default_model);
    }
}

// ── Thermal palette (per-theme) ─────────────────────────────────────────────
// heat_to_color removed — use theme_heat_color(theme, heat, alpha) instead

// ── Init / update / draw ────────────────────────────────────────────────────
static void vehicle_init_common(vehicle_t *v, int model_idx, Shader lighting_shader, int capacity) {
    memset(v, 0, sizeof(*v));
    v->position = (Vector3){0};
    v->rotation = QuaternionIdentity();
    v->model_idx = model_idx;
    v->model_group = vehicle_models[model_idx].group;
    v->red_material_idx = -1;
    v->green_material_idx = -1;
    v->fore_material_idx = -1;
    v->aft_material_idx = -1;
    v->color = WHITE;
    v->trail = (Vector3 *)calloc(capacity, sizeof(Vector3));
    v->trail_roll = (float *)calloc(capacity, sizeof(float));
    v->trail_pitch = (float *)calloc(capacity, sizeof(float));
    v->trail_vert = (float *)calloc(capacity, sizeof(float));
    v->trail_speed = (float *)calloc(capacity, sizeof(float));
    v->trail_time = (float *)calloc(capacity, sizeof(float));
    v->trail_capacity = capacity;
    v->trail_count = 0;
    v->trail_head = 0;
    v->trail_timer = 0.0f;
    v->trail_speed_max = 0.0f;

    v->lighting_shader = lighting_shader;
    v->loc_matNormal = -1;
    v->ghost_alpha = 1.0f;
    v->loc_ghost_alpha = -1;
    if (lighting_shader.id > 0) {
        v->loc_matNormal = GetShaderLocation(lighting_shader, "matNormal");
        v->loc_ghost_alpha = GetShaderLocation(lighting_shader, "ghostAlpha");
        float one = 1.0f;
        SetShaderValue(lighting_shader, v->loc_ghost_alpha, &one, SHADER_UNIFORM_FLOAT);
    }

    vehicle_load_model(v, model_idx);
}

void vehicle_init(vehicle_t *v, int model_idx, Shader lighting_shader) {
    vehicle_init_common(v, model_idx, lighting_shader, TRAIL_MAX);
}

void vehicle_init_ex(vehicle_t *v, int model_idx, Shader lighting_shader, int trail_capacity) {
    vehicle_init_common(v, model_idx, lighting_shader, trail_capacity);
}

void vehicle_update(vehicle_t *v, const hil_state_t *state, const home_position_t *home) {
    if (!state->valid) return;

    double lat = state->lat * 1e-7 * (M_PI / 180.0);
    double lon = state->lon * 1e-7 * (M_PI / 180.0);
    double alt = state->alt * 1e-3;

    if (!v->origin_set) {
        // Wait for HOME_POSITION so we get the correct ground altitude.
        // Fall back to current altitude after ~1 second (20 HIL updates at 22Hz).
        v->origin_wait_count++;
        if (home && home->valid && (lat != 0.0 || lon != 0.0)) {
            v->lat0 = lat;
            v->lon0 = lon;
            v->alt0 = alt;
            v->origin_set = true;

        } else if (v->origin_wait_count > 20) {
            v->lat0 = lat;
            v->lon0 = lon;
            v->alt0 = alt;
            v->origin_set = true;

        } else {
            return;  // skip this update, wait for home
        }
    }

    v->active = true;

    // Local NED position relative to origin
    double jmav_x = EARTH_RADIUS * (lat - v->lat0);                // North
    double jmav_y = EARTH_RADIUS * (lon - v->lon0) * cos(v->lat0); // East
    double jmav_z = alt - v->alt0;                                   // Up

    // NED frame → Raylib (X=right, Y=up, Z=back) + grid deconfliction offset
    v->position.x = (float)jmav_y + v->grid_offset.x;
    v->position.y = (float)jmav_z + v->grid_offset.y;
    if (v->position.y < 0.0f) v->position.y = 0.0f;
    v->position.z = (float)(-jmav_x) + v->grid_offset.z;

    // MAVLink quaternion: w,x,y,z in NED frame → Raylib
    float qw = state->quaternion[0];
    float qx = state->quaternion[1];
    float qy = state->quaternion[2];
    float qz = state->quaternion[3];

    v->rotation.w = qw;
    v->rotation.x = qy;
    v->rotation.y = -qz;
    v->rotation.z = -qx;

    // Derived telemetry from NED quaternion
    float nw = qw, nx = qx, ny = qy, nz = qz;
    if (nw < 0.0f) { nw = -nw; nx = -nx; ny = -ny; nz = -nz; }

    float heading_rad = atan2f(2.0f * (nw * nz + nx * ny),
                               1.0f - 2.0f * (ny * ny + nz * nz));
    v->heading_deg = heading_rad * RAD2DEG;
    if (v->heading_deg < 0.0f) v->heading_deg += 360.0f;

    v->roll_deg = atan2f(2.0f * (nw * nx + ny * nz),
                         1.0f - 2.0f * (nx * nx + ny * ny)) * RAD2DEG;

    float sin_pitch = 2.0f * (nw * ny - nz * nx);
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    v->pitch_deg = asinf(sin_pitch) * RAD2DEG;

    v->ground_speed = sqrtf((float)state->vx * state->vx +
                            (float)state->vy * state->vy) * 0.01f;
    v->vertical_speed = -state->vz * 0.01f;
    v->airspeed = state->ind_airspeed * 0.01f;
    v->altitude_rel = (float)(alt - v->alt0);

    // Adaptive trail sampling: record a point when direction changes (tight turns
    // get dense coverage) or after a max distance on straight runs (so they don't
    // go bare). Minimum distance gate prevents duplicate points when hovering.
    float dist_since = 0.0f;
    Vector3 cur_dir = {0};
    if (v->trail_count > 0) {
        int last = (v->trail_head - 1 + v->trail_capacity) % v->trail_capacity;
        float ddx = v->position.x - v->trail[last].x;
        float ddy = v->position.y - v->trail[last].y;
        float ddz = v->position.z - v->trail[last].z;
        dist_since = sqrtf(ddx*ddx + ddy*ddy + ddz*ddz);
        if (dist_since > 0.001f) {
            cur_dir = (Vector3){ ddx/dist_since, ddy/dist_since, ddz/dist_since };
        }
    }

    // Direction change: dot product < threshold means significant turn
    float dir_dot = 1.0f;
    if (v->trail_count > 1 && dist_since > 0.001f) {
        dir_dot = cur_dir.x * v->trail_last_dir.x +
                  cur_dir.y * v->trail_last_dir.y +
                  cur_dir.z * v->trail_last_dir.z;
    }

    // Sample triggers: dense on turns, sparse on straights
    v->trail_timer += GetFrameTime();
    bool dir_trigger  = (dir_dot < 0.995f) && (dist_since >= TRAIL_DIST_INTERVAL);
    bool dist_trigger = (dist_since >= 0.5f);
    bool time_trigger = (v->trail_timer >= TRAIL_INTERVAL * 4.0f);

    if (dir_trigger || dist_trigger || time_trigger) {
        v->trail_timer = 0.0f;
        if (dist_since > 0.001f) v->trail_last_dir = cur_dir;
        v->trail[v->trail_head] = v->position;
        v->trail_roll[v->trail_head] = v->roll_deg;
        v->trail_pitch[v->trail_head] = v->pitch_deg;
        v->trail_vert[v->trail_head] = v->vertical_speed;
        v->trail_time[v->trail_head] = v->current_time;
        float spd = sqrtf(v->ground_speed * v->ground_speed +
                          v->vertical_speed * v->vertical_speed);
        v->trail_speed[v->trail_head] = spd;
        if (spd > v->trail_speed_max) v->trail_speed_max = spd;
        bool was_full = (v->trail_count >= v->trail_capacity);
        v->trail_head = (v->trail_head + 1) % v->trail_capacity;
        if (v->trail_count < v->trail_capacity) v->trail_count++;

        // Recompute max when buffer wraps (old peak may have scrolled out)
        if (was_full) {
            float mx = 0.0f;
            for (int i = 0; i < v->trail_count; i++)
                if (v->trail_speed[i] > mx) mx = v->trail_speed[i];
            v->trail_speed_max = mx;
        }
    }
}

void vehicle_draw(vehicle_t *v, const theme_t *theme, bool selected,
                  int trail_mode, bool show_ground_track, Vector3 cam_pos,
                  bool classic_colors) {
    // Per-mode arm recoloring: front/back arms match trail forward/backward colors
    // Compute role colors
    Color role_colors[MAT_ROLE_COUNT] = {0};
    if (classic_colors) {
        role_colors[MAT_ROLE_FORE] = theme->arm_classic_front;
        role_colors[MAT_ROLE_AFT]  = theme->arm_classic_back;
        role_colors[MAT_ROLE_PORT] = theme->arm_classic_side;
        role_colors[MAT_ROLE_STARBOARD] = theme->arm_classic_side;
    } else {
        role_colors[MAT_ROLE_FORE] = theme->arm_front;
        role_colors[MAT_ROLE_AFT]  = theme->arm_back;
        role_colors[MAT_ROLE_PORT] = theme->arm_port;
        role_colors[MAT_ROLE_STARBOARD] = theme->arm_starboard;
    }
    role_colors[MAT_ROLE_BODY]     = (Color){ 4, 4, 60, 255 };
    role_colors[MAT_ROLE_BODY_MID] = (Color){ 22, 22, 74, 255 };
    role_colors[MAT_ROLE_PROP]     = (Color){ 80, 117, 162, 255 };
    role_colors[MAT_ROLE_MOTOR]    = (Color){ 14, 20, 30, 255 };
    // OBJ model: flat in XY, thin in Z (Z is model's up).
    // Raylib: Y is up. Rotate +90° around X so model Z → Raylib Y,
    // then apply per-model roll/pitch/yaw corrections.
    Matrix base_rot = MatrixMultiply(
        MatrixMultiply(
            MatrixMultiply(
                MatrixRotateX(90.0f * DEG2RAD),
                MatrixRotateX(v->roll_offset_deg * DEG2RAD)),
            MatrixRotateZ(v->pitch_offset_deg * DEG2RAD)),
        MatrixRotateY(v->yaw_offset_deg * DEG2RAD));
    Matrix rot = QuaternionToMatrix(v->rotation);
    Matrix scale = MatrixScale(v->model_scale, v->model_scale, v->model_scale);
    Matrix trans = MatrixTranslate(v->position.x, v->position.y, v->position.z);

    // Transform = Scale * BaseRot * AttitudeRot * Translation
    Matrix rot_only = MatrixMultiply(base_rot, rot);
    v->model.transform = MatrixMultiply(MatrixMultiply(scale, rot_only), trans);

    // Set normal matrix (rotation only, no scale/translation) before drawing
    if (v->loc_matNormal >= 0) {
        SetShaderValueMatrix(v->lighting_shader, v->loc_matNormal, rot_only);
    }

    // Set ghost alpha on shader
    if (v->loc_ghost_alpha >= 0) {
        SetShaderValue(v->lighting_shader, v->loc_ghost_alpha, &v->ghost_alpha, SHADER_UNIFORM_FLOAT);
    }

    if (v->ghost_alpha < 1.0f) rlDisableDepthMask();
    // Ghost drones get a 50% tint of their assigned color
    Color model_tint = WHITE;
    if (v->ghost_alpha < 1.0f) {
        float t = 0.5f;
        model_tint.r = (unsigned char)(255 * (1.0f - t) + v->color.r * t);
        model_tint.g = (unsigned char)(255 * (1.0f - t) + v->color.g * t);
        model_tint.b = (unsigned char)(255 * (1.0f - t) + v->color.b * t);
    }
    // Draw meshes individually with per-mesh role coloring
    for (int mi = 0; mi < v->model.meshCount; mi++) {
        int mat_idx = v->model.meshMaterial[mi];
        Material mat = v->model.materials[mat_idx];

        // Apply tint
        Color orig = mat.maps[MATERIAL_MAP_DIFFUSE].color;
        mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){
            (unsigned char)(((int)orig.r * (int)model_tint.r) / 255),
            (unsigned char)(((int)orig.g * (int)model_tint.g) / 255),
            (unsigned char)(((int)orig.b * (int)model_tint.b) / 255),
            (unsigned char)(((int)orig.a * (int)model_tint.a) / 255),
        };

        // Override with role color if this mesh has a role
        if (v->mesh_roles && mi < v->mesh_roles_count) {
            mat_role_t role = v->mesh_roles[mi];
            if (role > 0 && role < MAT_ROLE_COUNT) {
                Color ac = role_colors[role];
                mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){
                    (unsigned char)(((int)ac.r * (int)model_tint.r) / 255),
                    (unsigned char)(((int)ac.g * (int)model_tint.g) / 255),
                    (unsigned char)(((int)ac.b * (int)model_tint.b) / 255),
                    (unsigned char)(((int)ac.a * (int)model_tint.a) / 255),
                };
            }
        }

        DrawMesh(v->model.meshes[mi], mat, v->model.transform);
    }
    if (v->ghost_alpha < 1.0f) rlEnableDepthMask();

    // Draw path trail (mode 1), speed ribbon (mode 2), or drone color (mode 3)
    if (trail_mode > 0 && v->trail_count > 1) {
        int start = (v->trail_count < v->trail_capacity)
            ? 0
            : v->trail_head;

      if (trail_mode == 1) {
        // ── Normal directional trail ──
        Color trail_color = theme->trail_forward;
        // Keep original alpha for trail_forward (varies by theme)
        Color col_back     = theme->trail_backward;
        Color col_up       = theme->trail_climb;
        Color col_down     = theme->trail_descend;
        Color col_roll_pos = theme->trail_roll_pos;
        Color col_roll_neg = theme->trail_roll_neg;
        bool thick = theme->thick_trails;

        // Batched trail: single rlBegin/rlEnd instead of per-segment DrawLine3D
        rlBegin(thick ? RL_TRIANGLES : RL_LINES);
        int lod_skip = 0;
        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;

            // LOD: skip segments far from camera to reduce vertex count
            int skip = trail_lod_skip(v->trail[idx0], v->trail[idx1], cam_pos);
            if (lod_skip > 0) { lod_skip--; continue; }
            lod_skip = skip;

            float t = (float)i / (float)v->trail_count;

            float pitch = v->trail_pitch[idx1];
            float vert  = v->trail_vert[idx1];
            float roll  = v->trail_roll[idx1];

            float cr = (float)trail_color.r;
            float cg = (float)trail_color.g;
            float cb = (float)trail_color.b;

            float back_t = pitch / 15.0f;
            if (back_t < 0.0f) back_t = 0.0f;
            if (back_t > 1.0f) back_t = 1.0f;
            cr += (col_back.r - cr) * back_t;
            cg += (col_back.g - cg) * back_t;
            cb += (col_back.b - cb) * back_t;

            float vert_t = vert / 5.0f;
            if (vert_t > 1.0f) vert_t = 1.0f;
            if (vert_t < -1.0f) vert_t = -1.0f;
            if (vert_t > 0.0f) {
                cr += (col_up.r - cr) * vert_t;
                cg += (col_up.g - cg) * vert_t;
                cb += (col_up.b - cb) * vert_t;
            } else if (vert_t < 0.0f) {
                float dt2 = -vert_t;
                cr += (col_down.r - cr) * dt2;
                cg += (col_down.g - cg) * dt2;
                cb += (col_down.b - cb) * dt2;
            }

            float roll_t = roll / 15.0f;
            if (roll_t > 1.0f) roll_t = 1.0f;
            if (roll_t < -1.0f) roll_t = -1.0f;
            if (roll_t > 0.0f) {
                cr += (col_roll_pos.r - cr) * roll_t * 0.7f;
                cg += (col_roll_pos.g - cg) * roll_t * 0.7f;
                cb += (col_roll_pos.b - cb) * roll_t * 0.7f;
            } else if (roll_t < 0.0f) {
                float rt = -roll_t;
                cr += (col_roll_neg.r - cr) * rt * 0.7f;
                cg += (col_roll_neg.g - cg) * rt * 0.7f;
                cb += (col_roll_neg.b - cb) * rt * 0.7f;
            }

            unsigned char ccr = (unsigned char)(cr > 255 ? 255 : cr);
            unsigned char ccg = (unsigned char)(cg > 255 ? 255 : cg);
            unsigned char ccb = (unsigned char)(cb > 255 ? 255 : cb);
            unsigned char ca  = (unsigned char)(t * trail_color.a * v->ghost_alpha);
            rlColor4ub(ccr, ccg, ccb, ca);
            if (!thick) {
                rlVertex3f(v->trail[idx0].x, v->trail[idx0].y, v->trail[idx0].z);
                rlVertex3f(v->trail[idx1].x, v->trail[idx1].y, v->trail[idx1].z);
            } else {
                // Flat ribbon for better visibility in snow mode
                Vector3 seg = Vector3Subtract(v->trail[idx1], v->trail[idx0]);
                Vector3 up = {0, 1, 0};
                Vector3 perp = Vector3CrossProduct(seg, up);
                float plen = Vector3Length(perp);
                if (plen < 0.001f) continue;
                float hw = 0.013f;
                perp = Vector3Scale(perp, hw / plen);
                Vector3 a = Vector3Add(v->trail[idx0], perp);
                Vector3 b = Vector3Subtract(v->trail[idx0], perp);
                Vector3 d = Vector3Add(v->trail[idx1], perp);
                Vector3 e = Vector3Subtract(v->trail[idx1], perp);
                rlVertex3f(a.x, a.y, a.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(d.x, d.y, d.z);
                rlVertex3f(b.x, b.y, b.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(d.x, d.y, d.z);
                rlVertex3f(d.x, d.y, d.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(a.x, a.y, a.z);
                rlVertex3f(d.x, d.y, d.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(b.x, b.y, b.z);
            }
        }
        rlEnd();
      } else if (trail_mode == 3) {
        // ── Drone-color trail: solid vehicle color with age fade ──
        rlBegin(RL_LINES);
        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;
            float t = (float)i / (float)v->trail_count;
            unsigned char ca = (unsigned char)(t * 200 * v->ghost_alpha);
            rlColor4ub(v->color.r, v->color.g, v->color.b, ca);
            rlVertex3f(v->trail[idx0].x, v->trail[idx0].y, v->trail[idx0].z);
            rlVertex3f(v->trail[idx1].x, v->trail[idx1].y, v->trail[idx1].z);
        }
        rlEnd();
      } else {
        // ── Speed ribbon trail (mode 2) ──
        // Batched triangle ribbon with adaptive sampling (dense on turns, sparse on straights).
        // Perpendicular blending (70% previous + 30% current) prevents twisting on tight turns.
        float max_speed = v->trail_speed_max > 1.0f ? v->trail_speed_max : 1.0f;
        float max_half_w = v->model_scale * 0.25f;
        float min_half_w = 0.02f;
        Vector3 prev_perp = {0};
        bool have_prev = false;

        rlBegin(RL_TRIANGLES);
        int lod_skip = 0;
        for (int i = 1; i < v->trail_count; i++) {
            int idx0 = (start + i - 1) % v->trail_capacity;
            int idx1 = (start + i) % v->trail_capacity;

            // LOD: skip segments far from camera to reduce vertex count
            int skip = trail_lod_skip(v->trail[idx0], v->trail[idx1], cam_pos);
            if (lod_skip > 0) { lod_skip--; continue; }
            lod_skip = skip;

            Vector3 p0 = v->trail[idx0];
            Vector3 p1 = v->trail[idx1];
            float spd0 = v->trail_speed[idx0];
            float spd1 = v->trail_speed[idx1];

            Vector3 seg = { p1.x - p0.x, p1.y - p0.y, p1.z - p0.z };
            float seg_len = sqrtf(seg.x*seg.x + seg.y*seg.y + seg.z*seg.z);
            if (seg_len < 0.001f) continue;
            Vector3 dir = { seg.x/seg_len, seg.y/seg_len, seg.z/seg_len };

            // Compute perpendicular: use world-up cross for horizontal flight,
            // camera-based billboard for vertical flight
            float vt = fabsf(dir.y);
            Vector3 perp;
            if (vt < 0.7f) {
                Vector3 up = { 0, 1, 0 };
                perp = (Vector3){ dir.z * up.y - dir.y * up.z,
                                  dir.x * up.z - dir.z * up.x,
                                  dir.y * up.x - dir.x * up.y };
            } else {
                Vector3 fwd = { 0, 0, 1 };
                perp = (Vector3){ dir.y * fwd.z - dir.z * fwd.y,
                                  dir.z * fwd.x - dir.x * fwd.z,
                                  dir.x * fwd.y - dir.y * fwd.x };
            }
            float plen = sqrtf(perp.x*perp.x + perp.y*perp.y + perp.z*perp.z);
            if (plen < 0.001f) {
                if (have_prev) { perp = prev_perp; }
                else continue;
            } else {
                perp.x /= plen; perp.y /= plen; perp.z /= plen;
            }

            // Ensure consistent orientation then blend with previous to reduce twisting
            if (have_prev) {
                float dot = perp.x*prev_perp.x + perp.y*prev_perp.y + perp.z*prev_perp.z;
                if (dot < 0.0f) { perp.x = -perp.x; perp.y = -perp.y; perp.z = -perp.z; }
                float blend = 0.3f;
                perp.x = prev_perp.x * (1.0f - blend) + perp.x * blend;
                perp.y = prev_perp.y * (1.0f - blend) + perp.y * blend;
                perp.z = prev_perp.z * (1.0f - blend) + perp.z * blend;
                float nlen = sqrtf(perp.x*perp.x + perp.y*perp.y + perp.z*perp.z);
                if (nlen > 0.001f) { perp.x /= nlen; perp.y /= nlen; perp.z /= nlen; }
            }
            prev_perp = perp;
            have_prev = true;

            // Power law width: stays thin longer, ramps at high speed
            float s0 = spd0 / max_speed; if (s0 > 1.0f) s0 = 1.0f;
            float s1 = spd1 / max_speed; if (s1 > 1.0f) s1 = 1.0f;
            float hw0 = min_half_w + powf(s0, 2.0f) * max_half_w;
            float hw1 = min_half_w + powf(s1, 2.0f) * max_half_w;

            float spd_avg = (spd0 + spd1) * 0.5f;
            float accel = (spd1 - spd0) / TRAIL_INTERVAL;
            float accel_shift = accel / 40.0f;
            if (accel_shift > 0.15f) accel_shift = 0.15f;
            if (accel_shift < -0.15f) accel_shift = -0.15f;
            float heat = spd_avg / max_speed + accel_shift;
            if (heat > 1.0f) heat = 1.0f;
            if (heat < 0.0f) heat = 0.0f;

            float t = (float)i / (float)v->trail_count;
            Color c = theme_heat_color(theme, heat, (unsigned char)(t * 200));
            c.a = (unsigned char)(c.a * v->ghost_alpha);

            Vector3 a = { p0.x + perp.x*hw0, p0.y + perp.y*hw0, p0.z + perp.z*hw0 };
            Vector3 b = { p0.x - perp.x*hw0, p0.y - perp.y*hw0, p0.z - perp.z*hw0 };
            Vector3 d = { p1.x + perp.x*hw1, p1.y + perp.y*hw1, p1.z + perp.z*hw1 };
            Vector3 e = { p1.x - perp.x*hw1, p1.y - perp.y*hw1, p1.z - perp.z*hw1 };

            rlColor4ub(c.r, c.g, c.b, c.a);
            rlVertex3f(a.x, a.y, a.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(d.x, d.y, d.z);
            rlVertex3f(b.x, b.y, b.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(d.x, d.y, d.z);
            rlVertex3f(d.x, d.y, d.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(a.x, a.y, a.z);
            rlVertex3f(d.x, d.y, d.z); rlVertex3f(e.x, e.y, e.z); rlVertex3f(b.x, b.y, b.z);
        }
        rlEnd();
      }
    }

    // Ground projection (shadow / ring) at Y=0
    if (show_ground_track && v->position.y > 0.1f) {

        Vector3 ground = { v->position.x, 0.02f, v->position.z };
        float radius = 1.0f + v->position.y * 0.1f;
        if (radius > 5.0f) radius = 5.0f;

        // Dotted vertical drop line
        float dash = 0.08f;
        float gap = 0.12f;
        Color drop;

        Color fill_col = theme->ground_track_fill;
        Color edge_col = theme->ground_track_edge;
        drop = theme->ground_track_shadow;

        // Filled disc: concentric rings when close, cylinder when far
        float dx = cam_pos.x - ground.x;
        float dy = cam_pos.y - ground.y;
        float dz = cam_pos.z - ground.z;
        float cam_dist = sqrtf(dx*dx + dy*dy + dz*dz);
        if (cam_dist < 40.0f) {
            for (float r = 0.05f; r <= radius; r += 0.05f) {
                DrawCircle3D(ground, r, (Vector3){1, 0, 0}, 90.0f, fill_col);
            }
        } else {
            Vector3 cyl_pos = { ground.x, 0.01f, ground.z };
            DrawCylinder(cyl_pos, radius, radius, 0.25f, 36, fill_col);
        }
        DrawCircle3D(ground, radius, (Vector3){1, 0, 0}, 90.0f, edge_col);

        // Targeting cross
        float cs = radius * 0.5f;
        DrawLine3D((Vector3){ ground.x - cs, ground.y, ground.z },
                   (Vector3){ ground.x + cs, ground.y, ground.z }, edge_col);
        DrawLine3D((Vector3){ ground.x, ground.y, ground.z - cs },
                   (Vector3){ ground.x, ground.y, ground.z + cs }, edge_col);

        float y_top = v->position.y;
        float y_bot = 0.02f;
        float y = y_top;
        rlSetLineWidth(2.5f);
        while (y > y_bot) {
            float y_end = y - dash;
            if (y_end < y_bot) y_end = y_bot;
            DrawLine3D((Vector3){ v->position.x, y, v->position.z },
                       (Vector3){ v->position.x, y_end, v->position.z }, drop);
            y -= dash + gap;
        }
    }

    // No restore needed — we drew with local Material copies, not modifying model.materials
}

void vehicle_reset_trail(vehicle_t *v) {
    v->trail_count = 0;
    v->trail_head = 0;
    v->trail_timer = 0.0f;
    v->trail_speed_max = 0.0f;
}

void vehicle_truncate_trail(vehicle_t *v, float time_s) {
    if (v->trail_count == 0) return;

    int keep = 0;
    if (v->trail_count <= v->trail_capacity) {
        int start = (v->trail_head - v->trail_count + v->trail_capacity) % v->trail_capacity;
        for (int i = 0; i < v->trail_count; i++) {
            int idx = (start + i) % v->trail_capacity;
            if (v->trail_time[idx] <= time_s + 0.001f)
                keep = i + 1;
            else
                break;
        }
    }

    if (keep < v->trail_count) {
        int start = (v->trail_head - v->trail_count + v->trail_capacity) % v->trail_capacity;
        v->trail_count = keep;
        v->trail_head = (start + keep) % v->trail_capacity;
        v->trail_timer = 0.0f;

        float mx = 0.0f;
        for (int i = 0; i < v->trail_count; i++) {
            int idx = (start + i) % v->trail_capacity;
            if (v->trail_speed[idx] > mx) mx = v->trail_speed[idx];
        }
        v->trail_speed_max = mx;
    }
}

Color vehicle_marker_color(float roll, float pitch, float vert, float speed,
                           float speed_max, const theme_t *theme, int trail_mode,
                           Color drone_color) {
    if (trail_mode == 3) {
        return drone_color;
    }
    if (trail_mode == 2) {
        float ms = speed_max > 1.0f ? speed_max : 1.0f;
        float heat = speed / ms;
        if (heat > 1.0f) heat = 1.0f;
        if (heat < 0.0f) heat = 0.0f;
        return theme_heat_color(theme, heat, 255);
    }
    Color trail_color  = theme->trail_forward;
    Color col_back     = theme->trail_backward;
    Color col_up       = theme->trail_climb;
    Color col_down     = theme->trail_descend;
    Color col_roll_pos = theme->trail_roll_pos;
    Color col_roll_neg = theme->trail_roll_neg;

    float cr = (float)trail_color.r, cg = (float)trail_color.g, cb = (float)trail_color.b;

    float back_t = pitch / 15.0f;
    if (back_t < 0.0f) back_t = 0.0f;
    if (back_t > 1.0f) back_t = 1.0f;
    cr += (col_back.r - cr) * back_t;
    cg += (col_back.g - cg) * back_t;
    cb += (col_back.b - cb) * back_t;

    float vert_t = vert / 5.0f;
    if (vert_t > 1.0f) vert_t = 1.0f;
    if (vert_t < -1.0f) vert_t = -1.0f;
    if (vert_t > 0.0f) {
        cr += (col_up.r - cr) * vert_t;
        cg += (col_up.g - cg) * vert_t;
        cb += (col_up.b - cb) * vert_t;
    } else if (vert_t < 0.0f) {
        float dt2 = -vert_t;
        cr += (col_down.r - cr) * dt2;
        cg += (col_down.g - cg) * dt2;
        cb += (col_down.b - cb) * dt2;
    }

    float roll_t = roll / 15.0f;
    if (roll_t > 1.0f) roll_t = 1.0f;
    if (roll_t < -1.0f) roll_t = -1.0f;
    if (roll_t > 0.0f) {
        cr += (col_roll_pos.r - cr) * roll_t * 0.7f;
        cg += (col_roll_pos.g - cg) * roll_t * 0.7f;
        cb += (col_roll_pos.b - cb) * roll_t * 0.7f;
    } else if (roll_t < 0.0f) {
        float rt = -roll_t;
        cr += (col_roll_neg.r - cr) * rt * 0.7f;
        cg += (col_roll_neg.g - cg) * rt * 0.7f;
        cb += (col_roll_neg.b - cb) * rt * 0.7f;
    }

    return (Color){
        (unsigned char)(cr > 255 ? 255 : (cr < 0 ? 0 : cr)),
        (unsigned char)(cg > 255 ? 255 : (cg < 0 ? 0 : cg)),
        (unsigned char)(cb > 255 ? 255 : (cb < 0 ? 0 : cb)),
        255
    };
}

void vehicle_draw_markers(Vector3 *positions, char labels[][48], int count,
                          int current_marker, Vector3 cam_pos, Camera3D camera,
                          float *m_roll, float *m_pitch, float *m_vert, float *m_speed,
                          float speed_max, const theme_t *theme, int trail_mode,
                          marker_type_t type, Color drone_color) {
    (void)labels;
    bool is_system = (type == MARKER_SYSTEM);

    for (int i = 0; i < count; i++) {
        Vector3 p = positions[i];
        float dx = p.x - cam_pos.x, dy = p.y - cam_pos.y, dz = p.z - cam_pos.z;
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        float base = 0.05f + dist * 0.001f;
        // System markers use larger cube size; user markers use sphere radius directly
        float size = is_system ? base * 2.0f * 0.9f : base;

        bool is_current = (i == current_marker);

        Color col = vehicle_marker_color(m_roll[i], m_pitch[i], m_vert[i], m_speed[i],
                                         speed_max, theme, trail_mode, drone_color);

        if (is_current) {
            if (theme->thick_trails) {
                col.r = (unsigned char)(col.r * 0.55f);
                col.g = (unsigned char)(col.g * 0.55f);
                col.b = (unsigned char)(col.b * 0.55f);
            } else {
                col.r = (unsigned char)(col.r + (230 - col.r) * 0.7f);
                col.g = (unsigned char)(col.g + (230 - col.g) * 0.7f);
                col.b = (unsigned char)(col.b + (230 - col.b) * 0.7f);
            }
        }
        col.a = is_current ? 240 : 210;

        if (is_system) {
            float cs = is_current ? size * 1.4f : size;
            DrawCube(p, cs, cs, cs, col);
            DrawLine3D(p, (Vector3){p.x, 0.0f, p.z}, (Color){col.r, col.g, col.b, 60});

            if (is_current) {
                Vector3 fwd = Vector3Normalize(Vector3Subtract(cam_pos, p));
                Vector3 up = {0, 1, 0};
                Vector3 right = Vector3Normalize(Vector3CrossProduct(up, fwd));
                Vector3 cam_up = Vector3CrossProduct(fwd, right);
                float sq = size * 2.5f;
                Color sq_col = {col.r, col.g, col.b, 200};

                Vector3 tl = {p.x + (-right.x + cam_up.x) * sq, p.y + (-right.y + cam_up.y) * sq, p.z + (-right.z + cam_up.z) * sq};
                Vector3 tr = {p.x + ( right.x + cam_up.x) * sq, p.y + ( right.y + cam_up.y) * sq, p.z + ( right.z + cam_up.z) * sq};
                Vector3 br = {p.x + ( right.x - cam_up.x) * sq, p.y + ( right.y - cam_up.y) * sq, p.z + ( right.z - cam_up.z) * sq};
                Vector3 bl = {p.x + (-right.x - cam_up.x) * sq, p.y + (-right.y - cam_up.y) * sq, p.z + (-right.z - cam_up.z) * sq};

                DrawLine3D(tl, tr, sq_col);
                DrawLine3D(tr, br, sq_col);
                DrawLine3D(br, bl, sq_col);
                DrawLine3D(bl, tl, sq_col);
            }
        } else {
            DrawSphere(p, is_current ? size * 1.4f : size, col);
            DrawLine3D(p, (Vector3){p.x, 0.0f, p.z}, (Color){col.r, col.g, col.b, 80});

            if (is_current) {
                float ring_r = size * 2.5f;
                Color ring_col = {col.r, col.g, col.b, 200};
                DrawCircle3D(p, ring_r, Vector3Subtract(cam_pos, p), 0.0f, ring_col);
            }
        }
    }
}

void vehicle_draw_marker_labels(Vector3 *positions, char labels[][48], int count,
                                int current_marker, Vector3 cam_pos, Camera3D camera,
                                Font font_label, Font font_value,
                                float *m_roll, float *m_pitch, float *m_vert, float *m_speed,
                                float speed_max, const theme_t *theme, int trail_mode,
                                marker_type_t type, Color drone_color) {
    bool is_system = (type == MARKER_SYSTEM);
    Vector3 cam_fwd = Vector3Normalize(Vector3Subtract(camera.target, camera.position));

    float sh = (float)GetScreenHeight();
    float sw = (float)GetScreenWidth();
    float s = powf(sh / 720.0f, 0.7f);

    for (int i = 0; i < count; i++) {
        Vector3 p = positions[i];
        float dx = p.x - cam_pos.x, dy = p.y - cam_pos.y, dz = p.z - cam_pos.z;
        float dist = sqrtf(dx*dx + dy*dy + dz*dz);
        float base = 0.05f + dist * 0.001f;
        float size = is_system ? base * 2.0f * 0.9f : base;

        Vector3 to_marker = {dx, dy, dz};
        float dot = to_marker.x * cam_fwd.x + to_marker.y * cam_fwd.y + to_marker.z * cam_fwd.z;
        if (dot < 0.0f) continue;

        bool is_current = (i == current_marker);

        char display[64];
        if (is_system) {
            if (labels[i][0] != '\0')
                snprintf(display, sizeof(display), "S: %s", labels[i]);
            else
                snprintf(display, sizeof(display), "S%d", i + 1);
        } else {
            if (labels[i][0] != '\0')
                snprintf(display, sizeof(display), "%d: %s", i + 1, labels[i]);
            else
                snprintf(display, sizeof(display), "%d", i + 1);
        }

        Vector2 screen = GetWorldToScreen(
            (Vector3){p.x, p.y + size * 3.0f, p.z}, camera);

        if (screen.x < -50 || screen.x > sw + 50 ||
            screen.y < -50 || screen.y > sh + 50) continue;

        float fs = (is_current ? 18 : 15) * s;
        Vector2 tw = MeasureTextEx(font_value, display, fs, 0.5f);
        float pad_x = 8 * s, pad_y = 5 * s;

        float rx = screen.x - tw.x / 2 - pad_x;
        float ry = screen.y - tw.y / 2 - pad_y;
        float rw = tw.x + pad_x * 2;
        float rh = tw.y + pad_y * 2;

        Color text_col = vehicle_marker_color(m_roll[i], m_pitch[i], m_vert[i], m_speed[i],
                                              speed_max, theme, trail_mode, drone_color);
        if (is_current) {
            if (theme->thick_trails) {
                text_col.r = (unsigned char)(text_col.r * 0.55f);
                text_col.g = (unsigned char)(text_col.g * 0.55f);
                text_col.b = (unsigned char)(text_col.b * 0.55f);
            } else {
                text_col.r = (unsigned char)(text_col.r + (230 - text_col.r) * 0.7f);
                text_col.g = (unsigned char)(text_col.g + (230 - text_col.g) * 0.7f);
                text_col.b = (unsigned char)(text_col.b + (230 - text_col.b) * 0.7f);
            }
        }
        text_col.a = 255;

        Color label_bg = theme->hud_bg;
        Color label_border = theme->hud_border;

        DrawRectangleRounded(
            (Rectangle){rx, ry, rw, rh},
            0.3f, 6, label_bg);
        DrawRectangleRoundedLinesEx(
            (Rectangle){rx, ry, rw, rh},
            0.3f, 6, 1.0f, label_border);
        DrawTextEx(font_value, display,
                   (Vector2){screen.x - tw.x / 2, screen.y - tw.y / 2},
                   fs, 0.5f, text_col);
    }
}

void vehicle_set_ghost_alpha(vehicle_t *v, float alpha) {
    v->ghost_alpha = alpha;
}

void vehicle_draw_correlation_curtain(
    const vehicle_t *va, const vehicle_t *vb,
    const theme_t *theme, Vector3 cam_pos) {
    (void)theme;  // colors come from vehicle->color, theme kept for API consistency
    if (va->trail_count < 2 || vb->trail_count < 2) return;

    int n = va->trail_count < vb->trail_count ? va->trail_count : vb->trail_count;
    if (n < 2) return;

    int start_a = (va->trail_count < va->trail_capacity) ? 0 : va->trail_head;
    int start_b = (vb->trail_count < vb->trail_capacity) ? 0 : vb->trail_head;

    Color ca = va->color;
    Color cb = vb->color;

    rlDisableDepthMask();
    rlBegin(RL_TRIANGLES);
    for (int i = 1; i < n; i++) {
        // Map index through fractional position for trail length alignment
        int idx_a0 = (start_a + (int)((float)(i - 1) / n * va->trail_count)) % va->trail_capacity;
        int idx_a1 = (start_a + (int)((float)i / n * va->trail_count)) % va->trail_capacity;
        int idx_b0 = (start_b + (int)((float)(i - 1) / n * vb->trail_count)) % vb->trail_capacity;
        int idx_b1 = (start_b + (int)((float)i / n * vb->trail_count)) % vb->trail_capacity;

        Vector3 pa0 = va->trail[idx_a0];
        Vector3 pa1 = va->trail[idx_a1];
        Vector3 pb0 = vb->trail[idx_b0];
        Vector3 pb1 = vb->trail[idx_b1];

        float t = (float)i / (float)n;
        unsigned char alpha_a = (unsigned char)(t * 255 * va->ghost_alpha);
        unsigned char alpha_b = (unsigned char)(t * 255 * vb->ghost_alpha);

        // Front face: pa0 → pb0 → pa1, then pb0 → pb1 → pa1
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa0.x, pa0.y, pa0.z);
        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);

        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
        rlVertex3f(pb1.x, pb1.y, pb1.z);
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);

        // Back face: pa1 → pb0 → pa0, then pa1 → pb1 → pb0
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);
        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa0.x, pa0.y, pa0.z);

        rlColor4ub(ca.r, ca.g, ca.b, alpha_a);
        rlVertex3f(pa1.x, pa1.y, pa1.z);
        rlColor4ub(cb.r, cb.g, cb.b, alpha_b);
        rlVertex3f(pb1.x, pb1.y, pb1.z);
        rlVertex3f(pb0.x, pb0.y, pb0.z);
    }
    rlEnd();
    rlEnableDepthMask();
}

void vehicle_draw_correlation_line(
    const vehicle_t *va, const vehicle_t *vb) {
    // Offset to model center of mass (model origin is at bottom)
    float off_a = va->model_scale * 0.15f;
    float off_b = vb->model_scale * 0.15f;
    Vector3 pa = { va->position.x, va->position.y + off_a, va->position.z };
    Vector3 pb = { vb->position.x, vb->position.y + off_b, vb->position.z };

    Vector3 seg = { pb.x - pa.x, pb.y - pa.y, pb.z - pa.z };
    float seg_len = sqrtf(seg.x*seg.x + seg.y*seg.y + seg.z*seg.z);
    if (seg_len < 0.001f) return;

    Vector3 dir = { seg.x/seg_len, seg.y/seg_len, seg.z/seg_len };

    // Build two perpendicular vectors to form the cylinder cross-section
    Vector3 perp1, perp2;
    if (fabsf(dir.y) < 0.9f) {
        Vector3 up = {0, 1, 0};
        perp1 = (Vector3){ dir.y*up.z - dir.z*up.y,
                           dir.z*up.x - dir.x*up.z,
                           dir.x*up.y - dir.y*up.x };
    } else {
        Vector3 right = {1, 0, 0};
        perp1 = (Vector3){ dir.y*right.z - dir.z*right.y,
                           dir.z*right.x - dir.x*right.z,
                           dir.x*right.y - dir.y*right.x };
    }
    float p1len = sqrtf(perp1.x*perp1.x + perp1.y*perp1.y + perp1.z*perp1.z);
    perp1.x /= p1len; perp1.y /= p1len; perp1.z /= p1len;
    perp2 = (Vector3){ dir.y*perp1.z - dir.z*perp1.y,
                       dir.z*perp1.x - dir.x*perp1.z,
                       dir.x*perp1.y - dir.y*perp1.x };

    Color ca = va->color;
    Color cb = vb->color;
    unsigned char alpha_a = (unsigned char)(220 * va->ghost_alpha);
    unsigned char alpha_b = (unsigned char)(220 * vb->ghost_alpha);
    float radius = 0.04f;
    int slices = 8;
    int rings = 6;

    // Precompute sin/cos for the circle
    float sin_t[9], cos_t[9];  // slices + 1
    for (int s = 0; s <= slices; s++) {
        float a = (float)s / slices * 2.0f * M_PI;
        sin_t[s] = sinf(a);
        cos_t[s] = cosf(a);
    }

    rlBegin(RL_TRIANGLES);
    for (int r = 0; r < rings; r++) {
        float t0 = (float)r / rings;
        float t1 = (float)(r + 1) / rings;

        // Interpolated positions along axis
        Vector3 c0 = { pa.x + seg.x*t0, pa.y + seg.y*t0, pa.z + seg.z*t0 };
        Vector3 c1 = { pa.x + seg.x*t1, pa.y + seg.y*t1, pa.z + seg.z*t1 };

        // Interpolated colors
        unsigned char r0 = (unsigned char)(ca.r + (cb.r - ca.r) * t0);
        unsigned char g0 = (unsigned char)(ca.g + (cb.g - ca.g) * t0);
        unsigned char b0 = (unsigned char)(ca.b + (cb.b - ca.b) * t0);
        unsigned char a0 = (unsigned char)(alpha_a + (alpha_b - alpha_a) * t0);
        unsigned char r1 = (unsigned char)(ca.r + (cb.r - ca.r) * t1);
        unsigned char g1 = (unsigned char)(ca.g + (cb.g - ca.g) * t1);
        unsigned char b1 = (unsigned char)(ca.b + (cb.b - ca.b) * t1);
        unsigned char a1 = (unsigned char)(alpha_a + (alpha_b - alpha_a) * t1);

        for (int s = 0; s < slices; s++) {
            // Four corners of this quad on the cylinder surface
            float ox0 = radius * (perp1.x*cos_t[s]   + perp2.x*sin_t[s]);
            float oy0 = radius * (perp1.y*cos_t[s]   + perp2.y*sin_t[s]);
            float oz0 = radius * (perp1.z*cos_t[s]   + perp2.z*sin_t[s]);
            float ox1 = radius * (perp1.x*cos_t[s+1] + perp2.x*sin_t[s+1]);
            float oy1 = radius * (perp1.y*cos_t[s+1] + perp2.y*sin_t[s+1]);
            float oz1 = radius * (perp1.z*cos_t[s+1] + perp2.z*sin_t[s+1]);

            // Triangle 1
            rlColor4ub(r0, g0, b0, a0);
            rlVertex3f(c0.x+ox0, c0.y+oy0, c0.z+oz0);
            rlColor4ub(r1, g1, b1, a1);
            rlVertex3f(c1.x+ox0, c1.y+oy0, c1.z+oz0);
            rlColor4ub(r0, g0, b0, a0);
            rlVertex3f(c0.x+ox1, c0.y+oy1, c0.z+oz1);

            // Triangle 2
            rlColor4ub(r0, g0, b0, a0);
            rlVertex3f(c0.x+ox1, c0.y+oy1, c0.z+oz1);
            rlColor4ub(r1, g1, b1, a1);
            rlVertex3f(c1.x+ox0, c1.y+oy0, c1.z+oz0);
            rlVertex3f(c1.x+ox1, c1.y+oy1, c1.z+oz1);
        }
    }
    rlEnd();
}

void vehicle_cleanup(vehicle_t *v) {
    for (int i = 0; i < v->model.materialCount; i++)
        v->model.materials[i].shader.id = rlGetShaderIdDefault();
    UnloadModel(v->model);
    free(v->trail);
    free(v->trail_roll);
    free(v->trail_pitch);
    free(v->trail_vert);
    free(v->trail_speed);
    free(v->trail_time);
    free(v->material_roles);
    free(v->mesh_roles);
    v->trail = NULL;
    v->material_roles = NULL;
    v->mesh_roles = NULL;
    v->trail_roll = NULL;
    v->trail_pitch = NULL;
    v->trail_vert = NULL;
    v->trail_speed = NULL;
    v->trail_time = NULL;
}
