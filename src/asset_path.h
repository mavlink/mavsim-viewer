#ifndef ASSET_PATH_H
#define ASSET_PATH_H

#include <stddef.h>

// Initialize asset path resolution. Call once at startup.
void asset_path_init(void);

// Resolve an asset subpath (e.g. "models/px4_quadrotor.obj") to a full path.
// Writes into caller-provided buffer. Search order:
//   macOS:  ~/Library/Application Support/mavsim-viewer/<subpath>
//   Linux:  $XDG_DATA_HOME/mavsim-viewer/<subpath>
//   Then:   MAVSIM_INSTALL_DATADIR/<subpath>  (compile-time install prefix)
//   Then:   ./<subpath>                        (dev/build fallback)
void asset_path(const char *subpath, char *out, size_t out_size);

// Get a writable path for an asset (for generated files like terrain texture).
// Always returns the user data directory path, creating directories as needed.
//   macOS:  ~/Library/Application Support/mavsim-viewer/<subpath>
//   Linux:  $XDG_DATA_HOME/mavsim-viewer/<subpath>
void asset_write_path(const char *subpath, char *out, size_t out_size);

#endif
