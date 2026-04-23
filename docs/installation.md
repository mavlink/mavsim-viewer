# Installation

Hawkeye ships as a Homebrew formula on macOS, a `.deb` package on Debian/Ubuntu, and a portable ZIP on Windows.
If you want the latest development build, see [Building from source](./developer/build.md).

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

## Windows

Download `hawkeye-<version>-windows-x64.zip` from the [Hawkeye releases page](https://github.com/PX4/Hawkeye/releases/latest), extract it anywhere (e.g. `C:\Tools\hawkeye`), and double-click `hawkeye.exe`.

The ZIP is self-contained — no Visual Studio or runtime installer required. User data (screenshots, markers) is written to `%APPDATA%\hawkeye`.

::: info First-launch warning
Windows SmartScreen may warn that `hawkeye.exe` is from an unknown publisher. Click "More info" → "Run anyway". The binary is unsigned; signing and winget distribution are on the roadmap.
:::

To launch from any terminal, add the extracted folder to your `PATH`.

## Verifying the install

Launch Hawkeye with no arguments:

```sh
hawkeye
```

A window opens with the default quadrotor model on a grid backdrop, waiting for MAVLink telemetry on UDP port 19410.

If the window doesn't appear, see [Troubleshooting](./troubleshooting.md).

## Next steps

- [First SITL run](./first-sitl.md) — Hawkeye + PX4 SIH
- [First replay](./first-replay.md) — Load a `.ulg` file
- [First swarm](./first-swarm.md) — Replay multiple logs together
- [Command-Line Reference](./cli.md) — Every CLI flag with examples
