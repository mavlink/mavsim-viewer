# Installation

Hawkeye ships as a Homebrew formula on macOS, a `.deb` package on Debian/Ubuntu, and as a source build on all three major platforms.

## macOS (Homebrew)

```sh
brew tap PX4/px4
brew install PX4/px4/hawkeye
```

After install, the `hawkeye` binary is on your `PATH`.
Launch it from any terminal:

```sh
hawkeye
```

## Linux (Debian/Ubuntu)

Download the latest `.deb` from the [Hawkeye releases page](https://github.com/PX4/Hawkeye/releases/latest) and install it:

```sh
sudo dpkg -i hawkeye-*.deb
```

ARM64 builds (Raspberry Pi, Jetson, cloud ARM instances) are published in the same release.

## Building from source

Required on Windows, optional on macOS and Linux if you want the latest development builds.

::: info
Clone with `--recursive`.
Hawkeye uses the MAVLink `c_library_v2` as a submodule.
If you forget and hit `fatal: destination path 'c_library_v2' already exists and is not an empty directory`, fix it with `git submodule update --init`.
:::

### macOS

```sh
brew install cmake git
git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

### Linux (Debian/Ubuntu)

```sh
sudo apt-get install -y cmake git build-essential \
  libgl1-mesa-dev libx11-dev libxrandr-dev \
  libxinerama-dev libxcursor-dev libxi-dev

git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

### Windows

Requires [Visual Studio 2022](https://visualstudio.microsoft.com/) with the "Desktop development with C++" workload, [CMake](https://cmake.org/download/), and [Git](https://git-scm.com/).

```sh
git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

## Binary location

After a source build, the binary is at:

| Platform | Path                        |
| -------- | --------------------------- |
| macOS    | `build/hawkeye`             |
| Linux    | `build/hawkeye`             |
| Windows  | `build\Release\hawkeye.exe` |

## Makefile targets

| Target         | Description                 |
| -------------- | --------------------------- |
| `make`         | Debug build (default)       |
| `make release` | Release build               |
| `make test`    | Build and run all tests     |
| `make run`     | Build and launch the viewer |
| `make clean`   | Remove the build directory  |

## Verifying the install

Launch Hawkeye with no arguments:

```sh
./build/hawkeye     # source build
hawkeye             # package install
```

A window opens with the default quadrotor model on a grid backdrop, waiting for MAVLink telemetry on UDP port 19410.

If the window doesn't appear or you hit a build error, see [Troubleshooting](./troubleshooting.md).

## Next steps

- [First SITL run](./first-sitl.md) — Hawkeye + PX4 SIH
- [First replay](./first-replay.md) — Load a `.ulg` file
- [First swarm](./first-swarm.md) — Replay multiple logs together
- [Command-Line Reference](./cli.md) — Every CLI flag with examples
