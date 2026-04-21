# Troubleshooting (Hawkeye)

Common problems and fixes, organized by symptom.
If your issue isn't covered here, file it at the [Hawkeye issue tracker](https://github.com/PX4/Hawkeye/issues) with your OS, build type, reproduction steps, and console output.

## Build errors

### Linux: `Could NOT find X11`

Missing development headers.
Install the required packages:

```sh
sudo apt-get install -y libgl1-mesa-dev libx11-dev libxrandr-dev \
  libxinerama-dev libxcursor-dev libxi-dev
```

### macOS: `cmake: command not found`

Install CMake via Homebrew:

```sh
brew install cmake
```

### Windows: `MSBuild not found`

Install Visual Studio 2022 with the "Desktop development with C++" workload.
Community edition is sufficient and free.

### All platforms: `fatal: destination path 'c_library_v2' already exists and is not an empty directory`

You forgot the `--recursive` flag when cloning.
Fix it by initializing submodules:

```sh
cd Hawkeye
git submodule update --init
```

Then retry the build.

### Windows: binary builds but won't launch

Missing Visual C++ redistributable runtime.
Install the latest x64 redistributable from [Microsoft's download page](https://aka.ms/vs/17/release/vc_redist.x64.exe).

## Nothing appears in the window

Hawkeye launches and shows the grid backdrop, but no vehicle is visible.
Checklist:

1. **Is PX4 SITL actually running?** Check the PX4 terminal for errors.
   No PX4 means no telemetry.
2. **Is PX4 sending to the right port?** Hawkeye binds UDP 19410 by default.
   If you changed PX4's MAVLink target to a different port, use `-udp <port>` when launching Hawkeye to match.
3. **Firewall blocking UDP?** Temporarily disable your firewall and retry.
   On Linux, `sudo ufw disable`. On macOS, System Settings → Network → Firewall.
4. **Running PX4 and Hawkeye on different machines?** PX4 must be configured to stream to Hawkeye's IP address explicitly, not just localhost. See the PX4 [Remote Server Simulation](https://docs.px4.io/main/en/simulation/#running-simulation-on-a-remote-server) guide.

### Verifying UDP telemetry is arriving

```sh
# macOS / Linux
sudo lsof -i UDP:19410
```

If nothing is listening, Hawkeye isn't running.
If something is listening but no data flows, PX4 isn't sending to that port.

## ULog won't load

### `ulog_parser: vehicle_attitude topic not found`

The log is missing orientation data. Usually indicates a corrupted or truncated log.
Check the file size and try a different log from the same flight.

### `ulog_parser: vehicle_local_position topic not found`

The log has no position data at all.
Hawkeye cannot render a flight without position information.
This typically means the autopilot wasn't logging position.
Check the PX4 `SDLOG_PROFILE` parameter for the flight.

### Log loads but the drone sits at the origin

The home position couldn't be determined.
The log fell through to **Tier 3** (estimated origin).
See [Position Data Tiers](./position-tiers.md) for the trust hierarchy.

Check the debug overlay (`Ctrl+D`); the tier indicator will show T3 and the `BAD REF` warning will likely be visible.
Options:

- Load the log alongside other logs that do have home data, use Formation mode, and let the other drones anchor the reference frame
- Use Narrow Grid deconfliction mode to collapse T3 drones into a local view

## Multi-drone replay issues

### The deconfliction prompt appears every time I load these files

This is intentional.
The prompt appears whenever Hawkeye detects conflicts between loaded logs, because it has no persistent memory of your previous choice.
Pick your preferred mode each time, or use the `--ghost` CLI flag to force Ghost mode without prompting.

### Drones share a launch point, so Formation isn't available

If two or more drones have identical or near-identical home positions, Hawkeye detects the conflict at load time and does not offer Formation mode.
You'll see the deconfliction prompt with Ghost, Grid Offset, and Narrow Grid instead.
Pick **Grid Offset** for visual separation by +5 meters, or switch mid-replay with `P`.

Formation is only available when no conflict is detected at load time.
There's no in-app override.

### Drones are too far apart to see together

Switch to **Narrow Grid** mode.
It collapses geographically distant drones into the same view area with 1-meter spacing.

### Takeoff alignment doesn't detect my takeoff

The CUSUM algorithm needs a clean vertical velocity climb to trigger.
Check:

- Did the drone actually leave the ground in the log?
- Is vertical velocity data clean (no gyro or accelerometer errors)?
- Was the log recorded with a fast enough sample rate to catch the takeoff moment?

Low confidence scores (under 30%) in the CONF badge indicate the detection was ambiguous. Alignment still works at low confidence but may be off by a few seconds.

## Multi-instance SITL gotchas

### Vehicles stuck in formation without running the swarm script

The `SIH_LOC_LAT0`, `SIH_LOC_LON0`, and `SIH_LOC_ALT0` parameters persisted from a previous run. Clear them:

```sh
rm -rf PX4-Autopilot/build/px4_sitl_sih/instance_*/parameters*.bson
```

Then relaunch the instances.

### `sitl_multiple_run.sh` fails to start all instances

Port conflicts or leftover processes.
Check for zombie PX4 instances:

```sh
ps aux | grep px4
pkill -f px4
```

Then retry the launch script.

## Performance

### Low FPS at 16 drones

Check the debug overlay (`Ctrl+D`).
Frame time above 16.67 ms means you're below 60 FPS.

Things to try, in order of effort:

1. **Switch to Tactical HUD** (`H`) for simpler rendering and fewer draw calls
2. **Reset trail history** (`R`), since accumulated trail geometry is expensive
3. **Close ortho panels** (`O` to hide sidebar, `Alt+1` to return from fullscreen ortho)
4. **Reduce window size**, since rendering cost scales with pixel count

### Stutter on large ULog files

Hawkeye's ULog parser reads from disk on every playback seek.
Solid-state storage is recommended for large logs (50 MB or more).
If you're on spinning rust and seeing stutter, copy the log to an SSD before loading.

## Display issues

### HUD text unreadable at small window sizes

HUD has minimum readable floors designed for 1280×720 and up.
Below 640×480, the design target is exceeded and text may be unreadable.
Resize the window to at least 1280×720.

### Colors look wrong on my monitor

Try switching themes with `V`.
Snow theme is designed for high-contrast outdoor visibility.
If none of the built-ins look right, create a custom `.mvt` with your preferred palette and drag-drop it onto the window.
See [Themes](views.md#themes).

### Black screen on launch

GPU driver issue.
Check the first lines of the console output for the reported OpenGL version.
Hawkeye requires OpenGL 3.3 or newer.
If your GPU only supports older OpenGL, you'll need to update drivers or use a different machine.

Common fixes:

- **macOS**: update to the latest OS version available for your hardware
- **Linux**: install the proprietary NVIDIA or AMD driver (Mesa's OpenGL implementation should work but has been a source of issues)
- **Windows**: reinstall the latest graphics driver from the GPU vendor's website

## MAVLink port conflicts

### `bind: Address already in use`

Another app is holding the UDP port Hawkeye wants.
Find the offender:

```sh
# macOS / Linux
sudo lsof -i UDP:19410
```

Kill the process or use `-udp` to bind Hawkeye to a different port.

Common culprits: another Hawkeye instance still running, a PX4 ground station (QGroundControl), or an IDE's running simulation.

## Getting help

If this page doesn't solve your problem:

1. **Check the Hawkeye GitHub issues** for similar reports: [github.com/PX4/Hawkeye/issues](https://github.com/PX4/Hawkeye/issues)
2. **File a new issue** with:
   - Your OS and version
   - Build type (`make` debug / `make release` / package install)
   - Reproduction steps
   - Full console output from launch to crash / error
   - Relevant log file if it's a ULog parsing issue (sanitize private data first)
3. **PX4 Discourse** for broader SITL and MAVLink questions that aren't Hawkeye-specific: [discuss.px4.io](https://discuss.px4.io)
