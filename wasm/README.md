# Hawkeye WASM build

Standalone Emscripten build of Hawkeye, targeting embedding into host web pages (PX4 Flight Review, the self-hosted builder hub, or any other viewer that wants a 3D ULog replay canvas).

**This directory is entirely separate from the native build.** The root `CMakeLists.txt` does not reference anything here, and building the native desktop binary (`make release` from the repo root) does not require Emscripten or touch any file under `wasm/` or `src/wasm/`.

## Layout

```
wasm/
├── CMakeLists.txt      Standalone emscripten build subproject
├── wasm_api.h          Exported C API (interface contract for host pages)
├── wasm_main.c         Entry point, embed API implementation, frame loop
├── shell.html          Local dev harness (not shipping UI)
├── README.md           This file
└── .gitignore          build/ output

src/wasm/               WASM-only C implementation (separate from native src/)
├── ulog_events.h       Per-type event structs + flat timeline container
├── ulog_timeline.h/.c  Append/free helpers for the timeline
├── ulog_extractor.h/.c Reads a ULog from memory into a timeline + pre-scan state
└── wasm_replay.h/.c    Cursor-based playback over the pre-extracted timeline
```

Native source files (`src/*.c`, `src/*.h`) are untouched. The WASM build includes a few native headers (`mavlink_receiver.h`, `ulog_replay.h`) for type definitions only — no native `.c` files are compiled in the WASM build.

## Prerequisites

- [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) installed and `emsdk_env.sh` sourced in the current shell.
- CMake 3.14+
- `python3` (for serving the local dev harness during smoke testing)

Installing emsdk for the first time:

```bash
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh
```

## Building

From the repo root:

```bash
emcmake cmake -S wasm -B wasm/build -DCMAKE_BUILD_TYPE=Release
cmake --build wasm/build -j
```

The build emits three files into `wasm/build/`:

- `hawkeye.js` — JS glue code exposing the WASM module
- `hawkeye.wasm` — the compiled binary
- `hawkeye.data` — preloaded asset blob (once asset preloading is wired in Task #5; the skeleton stage has no `.data` file)

## Local smoke test

Copy the dev harness next to the build artifacts and serve it:

```bash
cp wasm/shell.html wasm/build/index.html
cd wasm/build
python3 -m http.server 8000
```

Then open `http://localhost:8000/` in a browser. You should see:
- A dark canvas
- The log panel showing `hawkeye.js loaded` and `hawkeye_init → 0`
- Clicking Play / Pause / dragging the seek slider should not error (no log is loaded yet in the skeleton stage)

Drag a `.ulg` file into the file input and the log panel should report `hawkeye_load_ulog_bytes → 0` and print the number of extracted events to the console.

## Current status (Task progress)

Tracked in `/Users/m/.claude/plans/velvety-dazzling-dolphin.md` plan file.

- **Task #2** ULog pre-extraction — **done** (`src/wasm/ulog_extractor.*`, `wasm_replay.*`)
- **Task #3** Interface contract — **done** (`wasm/wasm_api.h`)
- **Task #4** Skeleton build — **in progress** (`wasm/CMakeLists.txt`, `wasm/wasm_main.c`, this README)
- **Task #5** Asset audit — pending (needs a first successful build to measure)
- **Task #6** Dev harness — **done** (`wasm/shell.html`)
- **Task #7** Rendering spine wire-up — pending
- **Task #8** ULog replay wire-up — pending
- **Task #9** HUD wire-up — pending
- **Task #10** Embed API polish — pending
- **Task #11** Cross-browser smoke test — pending
- **Task #12** Size measurements — pending
- **Task #13** CI workflow — pending

## Size targets

To be recorded here after the first green build and the asset audit:

| Artifact           | Target (Brotli-compressed) | Actual |
| ------------------ | -------------------------- | ------ |
| `hawkeye.wasm`     | <1.5 MB                    | TBD    |
| `hawkeye.data`     | <1.0 MB                    | TBD    |
| `hawkeye.js` glue  | <100 KB                    | TBD    |

## Known limitations

- **Single-threaded only.** No `SharedArrayBuffer`, no pthreads. Necessary because host pages (e.g. Flight Review) cannot be required to set COOP/COEP headers.
- **No MAVLink UDP.** The WASM build does not include `src/mavlink_receiver.c` or `src/data_source_mavlink.c`. Live telemetry is a native-only feature.
- **WebGL2 required.** iOS Safari supports WebGL2 on iOS 15+. Older browsers will fail `hawkeye_init`.
- **Asset preloading pending.** The skeleton stage does not yet preload models, shaders, fonts, textures, or themes. Once Task #7 (rendering spine wire-up) lands, `--preload-file` will be added to the link flags.
