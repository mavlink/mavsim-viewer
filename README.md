# MAVSim Viewer

[![Release](https://img.shields.io/github/v/release/mavlink/mavsim-viewer)](https://github.com/mavlink/mavsim-viewer/releases/latest)

Lightweight 3D viewer for MAVLink-based flight simulators. Receives `HIL_STATE_QUATERNION` messages over UDP and renders the vehicle with real-time telemetry.

Built with [Raylib](https://www.raylib.com/) and [MAVLink](https://mavlink.io/).

![MAVSim Viewer](docs/screenshot.png)

## Features

- **Real-time 3D visualization** — vehicle position and orientation from MAVLink telemetry
- **Multi-vehicle swarm support** — up to 16 vehicles simultaneously
- **Multi-drone ULog replay** — load multiple `.ulg` files and replay swarm flights side by side
- **Ghost mode** — overlay flights with translucent non-primary drones for visual comparison
- **Automatic deconfliction** — detects position conflicts and offers formation, ghost, or grid offset modes
- **Correlation analysis** — real-time Pearson correlation and RMSE between pinned drones with visual overlays (line, curtain, ribbon)
- **CUSUM takeoff detection** — automatically detects launch times and aligns multi-drone timelines for synchronized comparison
- **Frame markers** — drop markers at any point during replay (`B`), add labels (`B` then `L`), and navigate between them
- **Theme engine** — `.mvt` theme files with drag-and-drop loading and `V` key cycling
- **Vehicle models** — quadrotor, hexarotor, fixed-wing, tailsitter, VTOL, rover, ROV
- **Three camera modes** — Chase, FPV, and Free (WASDQE fly with shift boost) — press `C` to cycle
- **Vehicle selection** — TAB cycles, 1-9 select, Shift+number pins, two-digit chords for drones 10-16
- **Mouse controls** — orbit (left-drag), FOV zoom (scroll wheel), ortho pan (right-drag)
- **HUD** — compass, attitude indicator, telemetry, CONF/PRSN/RMSE badges, and ESTIMATED POSITION warnings
- **Orthographic views** — sidebar (Top/Front/Right) and fullscreen (Alt+2-7) with 2D trail rendering
- **Edge indicators** — screen-edge chevrons pointing toward off-screen drones in multi-vehicle mode
- **Color palettes** — theme-driven colors for drones, trails, grid, and HUD
- **Cross-platform** — macOS, Linux, and Windows supported out of the box thanks to Raylib

## Install

### macOS (Homebrew)

```bash
brew tap mavlink/tap
brew install mavlink/tap/mavsim-viewer
```

### Linux (Debian/Ubuntu)

Download the `.deb` from the [latest release](https://github.com/mavlink/mavsim-viewer/releases/latest):

```bash
sudo dpkg -i mavsim-viewer-*.deb
```

### From source

#### macOS

```bash
brew install cmake git
git clone --recursive https://github.com/mavlink/mavsim-viewer.git
cd mavsim-viewer
make release
```

#### Linux (Debian/Ubuntu)

```bash
sudo apt-get install -y cmake git build-essential \
  libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev

git clone --recursive https://github.com/mavlink/mavsim-viewer.git
cd mavsim-viewer
make release
```

#### Windows

```powershell
# Requires Visual Studio with C/C++ workload, CMake, and Git
git clone --recursive https://github.com/mavlink/mavsim-viewer.git
cd mavsim-viewer
make release
```

The binary will be at `build/mavsim-viewer` (or `build\Release\mavsim-viewer.exe` on Windows).

#### Makefile targets

| Target | Description |
|---|---|
| `make` | Debug build (default) |
| `make release` | Release build |
| `make test` | Build and run all tests |
| `make run` | Build and launch the viewer |
| `make clean` | Remove build directory |

## Usage

```bash
./mavsim-viewer [options]
```

| Option | Description |
|---|---|
| `-udp <port>` | UDP base port (default: 19410) |
| `-n <count>` | Number of vehicles (default: 1, max: 16) |
| `-origin <lat> <lon> <alt>` | NED origin in degrees/meters (default: PX4 SIH default) |
| `-mc` | Multicopter model (default) |
| `-fw` | Fixed-wing model |
| `-ts` | Tailsitter model |
| `-w <width>` | Window width (default: 1280) |
| `-h <height>` | Window height (default: 720) |
| `--replay <f1.ulg> [f2.ulg ...]` | Replay one or more PX4 ULog flight logs |
| `--ghost <f1.ulg> [f2.ulg ...]` | Replay in ghost mode (non-primary drones at 35% opacity) |
| `-d` | Debug output |

Each vehicle listens on its own UDP port: `base_port`, `base_port+1`, ..., `base_port+n-1`.

### With PX4 SITL (single vehicle)

```bash
# Terminal 1: Start PX4 SITL with SIH
make px4_sitl sihsim_quadx

# Terminal 2: Launch viewer
./mavsim-viewer
```

### Multi-vehicle swarm

<video src="https://github.com/user-attachments/assets/eb14ef60-4447-4d14-99b1-aacbaa0fc29f" autoplay loop muted playsinline></video>

PX4 SIH supports multi-instance SITL where each instance sends `HIL_STATE_QUATERNION` on port `19410+N`. The viewer simply opens N sockets and renders all vehicles — all formation/spawn logic is handled externally (e.g. by setting `SIH_LOC_LAT0`/`SIH_LOC_LON0` params per instance).

A MAVSDK Python test script is included that orchestrates a complete swarm mission: sets spawn offsets for line formation, arms all vehicles, takes off in parallel, flies a waypoint, and lands.

#### Step-by-step swarm example

```bash
# 1. Build PX4 (once)
cd PX4-Autopilot
make px4_sitl_sih

# 2. Clear any persisted spawn params from previous runs
rm -rf build/px4_sitl_sih/instance_*/parameters*.bson

# 3. Launch 5 PX4 SIH instances (10x speed optional)
PX4_SIM_SPEED_FACTOR=10 ./Tools/simulation/sitl_multiple_run.sh 5 sihsim_quadx px4_sitl_sih

# 4. In a new terminal, launch the viewer
cd mavsim-viewer
./build/mavsim-viewer -n 5

# 5. In a new terminal, install MAVSDK and run the swarm test
pip install mavsdk
python tests/swarm_test.py --n 5 --speed 10
```

The test script accepts these options:

| Option | Description |
|---|---|
| `--n <count>` | Number of vehicles (default: 5) |
| `--spacing <meters>` | Formation spacing (default: 2.0) |
| `--altitude <meters>` | Takeoff altitude AGL (default: 10.0) |
| `--base-port <port>` | PX4 MAVLink base UDP port (default: 14540) |
| `--grpc-base <port>` | MAVSDK gRPC base port (default: 50051) |
| `--speed <factor>` | Sim speed factor to scale wait times (default: 1.0) |

**Note:** If vehicles appear in formation before running the test script, it's because `SIH_LOC_LON0` params were persisted from a previous run. Clear them with `rm -rf instance_*/parameters*.bson` in the PX4 build directory.

### ULog Replay

Replay PX4 `.ulg` flight logs directly in the viewer:

```bash
./mavsim-viewer --replay path/to/flight.ulg
```

The viewer parses `vehicle_attitude`, `vehicle_local_position`, and `vehicle_global_position` topics from the log. Vehicle type is auto-detected from `vehicle_status`. Logs without GPS data (e.g. indoor optical flow flights) fall back to local position with reference point conversion.

Position data in ULog files is typically logged at 5-10 Hz. Dead-reckoning interpolation (toggled with `I`) smooths playback to the render frame rate using velocity data.

### Multi-Drone Replay

Load multiple ULog files to replay swarm flights together:

```bash
# Formation mode — drones at real relative GPS positions
./mavsim-viewer --replay drone1.ulg drone2.ulg drone3.ulg

# Ghost mode — overlay flights for visual comparison
./mavsim-viewer --ghost flight_before.ulg flight_after.ulg
```

On startup, the viewer pre-scans each log for home position data (`home_position` topic with GPOS fallback) and detects conflicts automatically. If drones share the same launch point, are too far apart (>1km), or lack position data, a deconfliction prompt offers resolution options:

- **Formation** — shared NED origin, drones at real geographic positions. Home position markers show each launch site.
- **Ghost** — non-primary drones render at 35% opacity with color tint. Useful for overlaying two flights of the same path (e.g. before/after PID tuning).
- **Grid Offset** — shifts each drone +5m apart for visual separation when they'd otherwise overlap.
- **Narrow Grid** — collapses drones from different geographic locations into the same view area with 1m spacing.

Press `P` during replay to switch between modes.

#### Takeoff Alignment

Press `A` to align all drones to their detected takeoff times. Uses CUSUM (cumulative sum control chart) on vertical velocity to find the moment each drone lifts off, then shifts timelines so all takeoffs are synchronized. Useful when comparing flights that launched at different times.

#### Correlation Analysis

Pin a secondary drone (`Shift+1-9`) to see real-time correlation statistics in the HUD sidebar:

- **PRSN** — Pearson correlation coefficient (0.0–1.0) measuring how closely the pinned drone's trajectory tracks the selected drone
- **RMSE** — root mean square position error in meters
- **CONF** — CUSUM takeoff detection confidence (%)

Press `Shift+T` to cycle visual overlays between the selected and pinned drones:

- **Correlation Line** — direct line between current positions (drawn in 2D in ortho views)
- **Correlation Curtain** — semi-transparent 3D surface showing path divergence over time

#### Drone Color Trails

With multiple drones loaded, trail mode 3 (`T` to cycle) renders each drone's trail in its assigned fleet color. Three view-mode-aware palettes (Grid, Rez, Snow) provide 16 distinct colors with warm/cool alternation for adjacent drones.

## Controls

Press `?` to show the help overlay in-app.

| Key/Input | Action |
|---|---|
| **View** | |
| `C` | Cycle camera mode (Chase / FPV / Free) |
| `V` | Cycle theme (Grid / Rez / Snow, or drag-and-drop `.mvt` files) |
| `H` | Toggle HUD visibility |
| `T` | Cycle trail mode (off / directional / speed ribbon / drone color) |
| `Shift+T` | Cycle correlation overlay: off / line / curtain (multi-drone) |
| `G` | Toggle ground track projection |
| `F` | Toggle terrain texture |
| `K` | Toggle classic (red/blue) / modern (yellow/purple) arm colors |
| `O` | Toggle orthographic side panel (Top / Front / Right) |
| `Alt+1` | Return to perspective camera |
| `Alt+2`-`7` | Fullscreen ortho view (Top / Front / Left / Right / Bottom / Back) |
| `Ctrl+D` | Toggle debug panel |
| `?` | Toggle help overlay |
| **Camera** | |
| Left-drag | Orbit camera (chase) / look (free) |
| Scroll wheel | Zoom FOV (perspective) or span (ortho) |
| `WASDQE` | Fly in free camera mode (Shift: 3x boost) |
| `Alt+Scroll` | Zoom ortho view span |
| Right-drag | Pan in ortho mode |
| **Vehicle** | |
| `M` | Cycle vehicle model within group (`Shift+M`: all models) |
| `TAB` | Cycle to next vehicle (clears pins) |
| `[` / `]` | Previous / next vehicle (clears pins) |
| `1`-`9` | Select vehicle directly (two-digit chords for drones 10-16) |
| `Shift+1`-`9` | Toggle pin/unpin vehicle to HUD sidebar |
| `Ctrl+L` | Toggle screen edge indicators for off-screen drones |
| **Replay** | |
| `Space` | Pause / resume |
| `+` / `-` | Increase / decrease playback speed |
| `←` / `→` | Seek 5s backward / forward |
| `Shift+←/→` | Frame step (20ms) |
| `Ctrl+Shift+←/→` | Seek 1s |
| `L` | Toggle marker labels |
| `Shift+L` | Toggle loop |
| `I` | Toggle interpolation |
| `R` | Restart from beginning |
| `Y` | Toggle yaw display |
| `A` | Toggle takeoff time alignment (multi-drone) |
| `P` | Reopen deconfliction menu (multi-drone) |
| **Markers** | |
| `B` | Drop marker at current position |
| `B` then `L` | Drop marker and open label input |
| `Shift+B` | Delete current marker |
| `[` / `]` | Jump to previous / next marker (in replay) |
| `Shift+[` / `]` | Track drone from marker |

### View Modes

- **Grid** (default) — Debug grid with minor/major lines and colored axes
- **Rez** — Teal grid on black ground with dark sky
- **Snow** — High-contrast outdoor mode with bright white ground, dark grid lines, and black-outlined HUD for visibility on low-nit screens in sunlight

### Debug Overlay

Press `Ctrl+D` to toggle a performance debug panel on the right side of the screen. It displays:

- **FPS** — live value with color-coded graph and min/max range
- **Frame time** — millisecond readout with graph, 16.67ms target line, and peak tracker
- **Render stats** — active vehicle count and total trail points
- **Memory estimates** — trail buffer usage and estimated total footprint
- **Position** — XYZ coordinates, BAD REF warning, and position data tier (T1: home_position, T2: GPOS/LPOS ref, T3: estimated)

Colors adapt to the active view mode (Grid / Rez / Snow).

### Orthographic Views

Press `O` to open a side panel with three synchronized orthographic views (Top, Front, Right) that follow the selected vehicle. Use `Alt+2`-`7` to switch the main viewport to a fullscreen orthographic camera. Scroll wheel adjusts the visible span. Side views (Front, Back, Left, Right) include a ground plane fill and distance grid with adaptive spacing.

## Acknowledgments

This project was inspired by [jMAVSim](https://github.com/PX4/jMAVSim), the Java-based MAVLink simulator and viewer. Vehicle 3D models are derived from jMAVSim's assets.

## License

BSD-3-Clause
