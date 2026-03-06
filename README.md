# MAVSim Viewer

Lightweight 3D viewer for MAVLink-based flight simulators. Receives `HIL_STATE_QUATERNION` messages over UDP and renders the vehicle with real-time telemetry.

Built with [Raylib](https://www.raylib.com/) and [MAVLink](https://mavlink.io/).

![MAVSim Viewer](docs/screenshot.png)

## Features

- Real-time 3D visualization of vehicle position and orientation
- **Multi-vehicle swarm support** — up to 16 vehicles simultaneously
- Multiple vehicle models: multicopter, fixed-wing, tailsitter
- Chase and FPV camera modes (press `C` to toggle)
- Vehicle selection: TAB, `[`/`]`, or number keys `1`-`9`
- Mouse orbit (left-drag) and FOV zoom (scroll wheel)
- HUD with compass, telemetry readouts, vehicle selector indicator
- Panoramic sky with ground texture
- Cross-platform: macOS, Linux, and Windows supported out of the box thanks to Raylib

## Requirements

- CMake 3.14+
- C compiler (GCC, Clang, MSVC)
- Git (for submodules and Raylib fetch)

## Building

```bash
git clone --recursive git@github.com:mavlink/mavsim-viewer.git
cd mavsim-viewer
mkdir build && cd build
cmake ..
make -j$(nproc)
```

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

## Controls

| Key/Input | Action |
|---|---|
| `C` | Toggle camera mode (Chase / FPV) |
| `V` | Cycle view mode (Grid / jMAVSim / Rez) |
| `TAB` | Cycle to next vehicle |
| `[` / `]` | Previous / next vehicle |
| `1`-`9` | Select vehicle directly |
| Left-drag | Orbit camera (chase mode) |
| Scroll wheel | Zoom FOV |

### View Modes

- **Grid** (default) — Debug grid with minor/major lines and colored axes
- **jMAVSim** — Panoramic sky with ground texture
- **Rez** — Teal grid on black ground with dark sky

## Acknowledgments

This project was inspired by [jMAVSim](https://github.com/PX4/jMAVSim), the Java-based MAVLink simulator and viewer. Vehicle 3D models and sky/ground textures are derived from jMAVSim's assets.

## License

BSD-3-Clause
