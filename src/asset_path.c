#include "asset_path.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define mkdir_p(path) _mkdir(path)
#else
#include <unistd.h>
#define mkdir_p(path) mkdir(path, 0755)
#endif

// Cached base directories (set once in asset_path_init)
static char s_user_data[512];    // user-writable data dir
static char s_install_data[512]; // system install dir (compile-time)

static int file_exists(const char *path) {
#ifdef _WIN32
    struct _stat st;
    return _stat(path, &st) == 0;
#else
    return access(path, R_OK) == 0;
#endif
}

// Recursively create directories for a file path
static void mkdirs_for_file(const char *filepath) {
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", filepath);

    // Find last separator
    char *last_sep = strrchr(tmp, '/');
#ifdef _WIN32
    char *last_bsep = strrchr(tmp, '\\');
    if (last_bsep > last_sep) last_sep = last_bsep;
#endif
    if (!last_sep) return;
    *last_sep = '\0';

    // Create each directory component
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/'
#ifdef _WIN32
            || *p == '\\'
#endif
        ) {
            *p = '\0';
            mkdir_p(tmp);
            *p = '/';
        }
    }
    mkdir_p(tmp);
}

void asset_path_init(void) {
    s_user_data[0] = '\0';
    s_install_data[0] = '\0';

#ifdef __APPLE__
    const char *home = getenv("HOME");
    if (home) {
        snprintf(s_user_data, sizeof(s_user_data),
                 "%s/Library/Application Support/hawkeye", home);
    }
#elif !defined(_WIN32)
    // Linux: XDG_DATA_HOME or ~/.local/share
    const char *xdg = getenv("XDG_DATA_HOME");
    if (xdg && xdg[0]) {
        snprintf(s_user_data, sizeof(s_user_data),
                 "%s/hawkeye", xdg);
    } else {
        const char *home = getenv("HOME");
        if (home) {
            snprintf(s_user_data, sizeof(s_user_data),
                     "%s/.local/share/hawkeye", home);
        }
    }
#endif

    // Compile-time install prefix (set by CMake)
#ifdef HAWKEYE_INSTALL_DATADIR
    snprintf(s_install_data, sizeof(s_install_data),
             "%s", HAWKEYE_INSTALL_DATADIR);
#endif
}

void asset_path(const char *subpath, char *out, size_t out_size) {
    char candidate[512];

    // 1. User data directory
    if (s_user_data[0]) {
        snprintf(candidate, sizeof(candidate), "%s/%s", s_user_data, subpath);
        if (file_exists(candidate)) {
            snprintf(out, out_size, "%s", candidate);
            return;
        }
    }

    // 2. System install directory
    if (s_install_data[0]) {
        snprintf(candidate, sizeof(candidate), "%s/%s", s_install_data, subpath);
        if (file_exists(candidate)) {
            snprintf(out, out_size, "%s", candidate);
            return;
        }
    }

    // 3. Relative path (dev/build fallback)
    snprintf(out, out_size, "%s", subpath);
}

void asset_write_path(const char *subpath, char *out, size_t out_size) {
    if (s_user_data[0]) {
        snprintf(out, out_size, "%s/%s", s_user_data, subpath);
        mkdirs_for_file(out);
    } else {
        // Fallback to relative path
        snprintf(out, out_size, "%s", subpath);
    }
}
