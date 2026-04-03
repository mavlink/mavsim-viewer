#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#endif

#ifndef MODELS_DIR
#define MODELS_DIR "models"
#endif

// Raylib 5.5's OBJ loader crashes when a model has no texture coordinates.
// The face format "f v//vn" (skipping texcoord index) causes an out-of-bounds
// read in tinyobj_loader_c. This only manifests in Release builds.
//
// To fix an OBJ exported without texcoords:
//   1. Add a single "vt 0.0 0.0" line after the last "vn" line
//   2. Change all faces from "f v//vn" to "f v/1/vn"
//
// Example sed one-liner (backup first!):
//   sed -i 's|//|/1/|g' model.obj
//   sed -i '/^vn /a vt 0.0 0.0' model.obj  (add after first vn block)

static int failures = 0;
static int checked = 0;

static void check_obj(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) {
        printf("  SKIP %s (not found)\n", path);
        return;
    }

    int has_faces = 0;
    int has_vt = 0;
    int bare_face_line = 0;
    int line_num = 0;
    char line[512];

    while (fgets(line, sizeof(line), f)) {
        line_num++;
        if (line[0] == 'f' && line[1] == ' ') {
            has_faces = 1;
            if (!bare_face_line && strstr(line, "//"))
                bare_face_line = line_num;
        }
        if (line[0] == 'v' && line[1] == 't' && line[2] == ' ')
            has_vt = 1;
    }
    fclose(f);
    checked++;

    if (!has_faces) {
        printf("  PASS %s (no faces)\n", path);
        return;
    }

    if (!has_vt || bare_face_line) {
        printf("\n  FAIL %s\n", path);
        if (!has_vt)
            printf("       Missing texture coordinates (no 'vt' lines).\n");
        if (bare_face_line)
            printf("       Uses 'f v//vn' format at line %d (missing texcoord index).\n", bare_face_line);
        printf("       Raylib 5.5 will segfault in Release builds.\n");
        printf("       Run: python3 scripts/fix_obj_texcoords.py %s\n\n", path);
        failures++;
        return;
    }

    printf("  PASS %s\n", path);
}

int main(void) {
    printf("test_obj_models:\n");

#ifdef _WIN32
    char pattern[512];
    snprintf(pattern, sizeof(pattern), "%s\\*.obj", MODELS_DIR);
    WIN32_FIND_DATAA fd;
    HANDLE hFind = FindFirstFileA(pattern, &fd);
    assert(hFind != INVALID_HANDLE_VALUE && "Cannot open models directory");
    do {
        char path[512];
        snprintf(path, sizeof(path), "%s\\%s", MODELS_DIR, fd.cFileName);
        check_obj(path);
    } while (FindNextFileA(hFind, &fd));
    FindClose(hFind);
#else
    DIR *dir = opendir(MODELS_DIR);
    assert(dir && "Cannot open models directory");

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        size_t len = strlen(entry->d_name);
        if (len > 4 && strcmp(entry->d_name + len - 4, ".obj") == 0) {
            char path[512];
            snprintf(path, sizeof(path), "%s/%s", MODELS_DIR, entry->d_name);
            check_obj(path);
        }
    }
    closedir(dir);
#endif

    if (failures > 0) {
        printf("\n%d model(s) failed validation.\n", failures);
        return 1;
    }

    printf("All %d OBJ models passed.\n", checked);
    return 0;
}
