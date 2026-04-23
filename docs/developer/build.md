# Building from source

You only need to build from source if you want to modify Hawkeye or run pre-release code.
For everyday use, the Homebrew formula, `.deb` package, or Windows ZIP described in [Installation](../installation.md) is the faster path.

::: info
Clone with `--recursive`.
Hawkeye uses the MAVLink `c_library_v2` as a submodule.
If you forget and hit `fatal: destination path 'c_library_v2' already exists and is not an empty directory`, fix it with `git submodule update --init`.
:::

## macOS

```sh
brew install cmake git
git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

## Linux (Debian/Ubuntu)

```sh
sudo apt-get install -y cmake git build-essential \
  libgl1-mesa-dev libx11-dev libxrandr-dev \
  libxinerama-dev libxcursor-dev libxi-dev

git clone --recursive https://github.com/PX4/Hawkeye.git
cd Hawkeye
make release
```

## Windows

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

| Target         | Description                                                             |
| -------------- | ----------------------------------------------------------------------- |
| `make`         | Debug build (default)                                                   |
| `make release` | Release build                                                           |
| `make run`     | Build and launch the viewer                                             |
| `make test`    | Build and run all tests (see [Testing](./testing.md))                   |
| `make test-core` | Build and run core tests only (no raylib, fast CI path)               |
| `make sanitize` | Build with AddressSanitizer and UndefinedBehaviorSanitizer enabled     |
| `make clean`   | Remove the build directory                                              |

The `BUILD_TYPE` variable defaults to `Debug`; override with `make BUILD_TYPE=Release` or use the `release` target.
Extra CMake flags can be passed via `CMAKE_EXTRA`.

## Verifying the build

Launch the binary with no arguments:

```sh
./build/hawkeye
```

A window opens with the default quadrotor model on a grid backdrop, waiting for MAVLink telemetry on UDP port 19410.
If the window doesn't appear, see [Troubleshooting](../troubleshooting.md).

## Next steps

- [Testing](./testing.md) — Running the ctest suite and sanitizer builds
- [First SITL run](../first-sitl.md) — Connect the built binary to PX4 SIH
