# MAVSim Viewer

Lightweight 3D viewer for MAVLink-based flight simulators. Receives `HIL_STATE_QUATERNION` messages over UDP and renders the vehicle with real-time telemetry.

Built with [Raylib](https://www.raylib.com/) and [MAVLink](https://mavlink.io/).

![MAVSim Viewer](docs/screenshot.png)

## Features

- Real-time 3D visualization of vehicle position and orientation
- Multiple vehicle models: multicopter, fixed-wing, tailsitter
- Chase and FPV camera modes (press `C` to toggle)
- Mouse orbit (left-drag) and FOV zoom (scroll wheel)
- HUD with compass, telemetry readouts (altitude, heading, roll, pitch, ground speed, vertical speed, airspeed, flight timer)
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
| `-udp <port>` | UDP listen port (default: 19410) |
| `-mc` | Multicopter model (default) |
| `-fw` | Fixed-wing model |
| `-ts` | Tailsitter model |
| `-w <width>` | Window width (default: 1280) |
| `-h <height>` | Window height (default: 720) |
| `-d` | Debug output |

### With PX4 SITL

```bash
# Terminal 1: Start PX4 SITL with SIH
make px4_sitl sihsim_quadx

# Terminal 2: Launch viewer
./mavsim-viewer
```

## Controls

| Key/Input | Action |
|---|---|
| `C` | Toggle camera mode (Chase / FPV) |
| `G` | Toggle Grid view mode |
| `Ctrl+R` | Toggle Rez view mode |
| Left-drag | Orbit camera (chase mode) |
| Scroll wheel | Zoom FOV |

### View Modes

- **Texture** (default) — Panoramic sky with ground texture
- **Grid** — Debug grid with minor/major lines and colored axes
- **Rez** — Teal grid on black ground with dark sky

## Acknowledgments

This project was inspired by [jMAVSim](https://github.com/PX4/jMAVSim), the Java-based MAVLink simulator and viewer. Vehicle 3D models and sky/ground textures are derived from jMAVSim's assets.

## License

BSD-3-Clause
