# Hawkeye

[![Release](https://img.shields.io/github/v/release/PX4/Hawkeye)](https://github.com/PX4/Hawkeye/releases/latest)

Real-time 3D flight visualizer for PX4 — watch live SITL simulations, replay ULog flights, and analyze multi-drone swarms with correlation tracking, takeoff alignment, and deconfliction. Supports up to 16 vehicles simultaneously with ghost overlays, formation views, and frame-by-frame inspection.

Built with [Raylib](https://www.raylib.com/) and [MAVLink](https://mavlink.io/). Zero dependencies to install — just build and fly.

![Hawkeye](docs/assets/screenshot.png)

**Full documentation: [px4.github.io/Hawkeye](https://px4.github.io/Hawkeye/)**

## Install

### macOS (Homebrew)

```bash
brew tap PX4/px4
brew install PX4/px4/hawkeye
```

### Linux (Debian/Ubuntu)

Download the `.deb` from the [latest release](https://github.com/PX4/Hawkeye/releases/latest):

```bash
sudo dpkg -i hawkeye-*.deb
```

### From source

#### macOS

```bash
brew install cmake git
git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

#### Linux (Debian/Ubuntu)

```bash
sudo apt-get install -y cmake git build-essential \
  libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev

git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

#### Windows

```powershell
# Requires Visual Studio with C/C++ workload, CMake, and Git
git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

The binary will be at `build/hawkeye` (or `build\Release\hawkeye.exe` on Windows).

#### Makefile targets

| Target | Description |
|---|---|
| `make` | Debug build (default) |
| `make release` | Release build |
| `make test` | Build and run all tests |
| `make run` | Build and launch the viewer |
| `make clean` | Remove build directory |

## Quickstart

Launch with PX4 SITL (single vehicle):

```bash
# Terminal 1: Start PX4 SITL with SIH
make px4_sitl sihsim_quadx

# Terminal 2: Launch viewer
./hawkeye
```

Replay a ULog file:

```bash
./hawkeye --replay path/to/flight.ulg
```

See the [full documentation](https://px4.github.io/Hawkeye/) for CLI options, multi-vehicle swarms, ULog replay, HUD modes, camera views, correlation analysis, and keyboard shortcuts.

## Acknowledgments

This project was inspired by [jMAVSim](https://github.com/PX4/jMAVSim), the Java-based MAVLink simulator and viewer. Vehicle 3D models are derived from jMAVSim's assets.

## License

BSD-3-Clause
