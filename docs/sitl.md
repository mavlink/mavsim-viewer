# Live SITL Integration (Hawkeye)

Hawkeye connects to running PX4 SITL instances over UDP and visualizes them in real time.
This page covers the three main workflows: a single vehicle from PX4 SIH, multi-instance swarms from `sitl_multiple_run.sh`, and orchestrated swarm missions via the included MAVSDK test script.

For quickstart instructions, see [First SITL run](./first-sitl.md).
This page covers the topics in more depth.

PX4 SIH itself is documented in the [PX4 user guide](https://docs.px4.io/main/en/sim_sih/).

## Single vehicle

The simplest live workflow: one PX4 SIH instance, one Hawkeye window, telemetry streaming over the default UDP port.

::: tip
You don't need a PX4 source tree anymore.
PX4 publishes prebuilt `.deb` packages and Docker containers for SITL SIH.
The fastest path, on any OS with Docker:

```sh
docker run --rm -it -p 19410:19410/udp px4io/px4-sitl:latest
```

Port 19410 is the SIH display port that Hawkeye binds to.
Add `-p 14550:14550/udp` if you also want to connect QGroundControl.

For `.deb` installs, other vehicle models, and multi-instance setup, see [Prebuilt PX4 SITL packages](https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages).
The source-build path below still works if you're already set up for development.
:::

**Terminal 1, start PX4 SIH from source:**

```sh
cd PX4-Autopilot
make px4_sitl sihsim_quadx
```

PX4 SIH (Simulation In Hardware) is a lightweight, lockstep-free flight dynamics simulator built into PX4.
It's the fastest way to get telemetry flowing without installing Gazebo.
See the [PX4 SIH documentation](https://docs.px4.io/main/en/sim_sih/) for background on the simulator itself.

**Terminal 2, launch Hawkeye:**

```sh
hawkeye
```

Hawkeye binds to UDP port 19410 by default.
PX4 SIH sends `HIL_STATE_QUATERNION` messages to the same port
The vehicle appears immediately in the Hawkeye window at the origin.

<iframe width="720" height="405" src="https://www.youtube.com/embed/WudOSKFC0pc" frameborder="0" allowfullscreen></iframe>

_<!-- 08-vid-01: Single vehicle SITL quickstart. -->_

**Arm and take off** in the PX4 shell:

```sh
commander takeoff
```

Watch Hawkeye: the vehicle arms, climbs to hover altitude, and the HUD telemetry updates as the flight progresses.

### Alternate ports

If port 19410 is already in use, bind Hawkeye to a different port with `-udp`:

```sh
hawkeye -udp 14540
```

Configure PX4 to stream to the same port.
See [Command-Line Reference](./cli.md) for the full option list.

### Using different vehicle types

PX4 SIH ships vehicle presets beyond `sihsim_quadx`.
Some common ones:

| PX4 target                      | Vehicle type |
| ------------------------------- | ------------ |
| `make px4_sitl sihsim_quadx`    | Quadrotor    |
| `make px4_sitl sihsim_airplane` | Fixed-wing   |
| `make px4_sitl sihsim_xvert`    | Tailsitter   |

Hawkeye auto-detects the vehicle type from `HEARTBEAT` and loads the matching 3D model.
No Hawkeye configuration needed; just launch the appropriate PX4 target and Hawkeye adapts.

## Multi-instance swarm

PX4 supports running multiple SITL instances simultaneously via `sitl_multiple_run.sh`.
Each instance runs its own PX4 process with its own UDP port, and Hawkeye binds multiple sockets automatically when launched with `-n`.
For PX4-side setup and parameter details, see [Multi-Vehicle Simulation](https://docs.px4.io/main/en/sim_sih/#multi-vehicle-simulation) in the PX4 SIH docs.

### Port allocation

Each vehicle binds `base_port + N`:

_<!-- 08-dia-01: SVG showing base_port + N layout for PX4 instances and Hawkeye sockets. -->_

With default `-udp 19410 -n 5`, Hawkeye listens on ports 19410 through 19414 simultaneously.
PX4 instance 1 sends to 19410, instance 2 to 19411, and so on, so PX4 instance N and Hawkeye vehicle N automatically match up.

### Step-by-step launch

**1. Build PX4 SIH (once):**

```sh
cd PX4-Autopilot
make px4_sitl_sih
```

**2. Clear any persisted spawn parameters from previous runs:**

```sh
rm -rf build/px4_sitl_sih/instance_*/parameters*.bson
```

::: warning
PX4 persists the `SIH_LOC_LAT0`, `SIH_LOC_LON0`, and `SIH_LOC_ALT0` parameters per instance between runs.
If you forget to clear them, your vehicles will spawn in whatever formation the last run left behind, possibly far from the expected origin.
Clearing the `parameters*.bson` files is the reliable way to reset them.
:::

**3. Launch N instances in parallel:**

```sh
PX4_SIM_SPEED_FACTOR=10 ./Tools/simulation/sitl_multiple_run.sh 5 sihsim_quadx px4_sitl_sih
```

The `PX4_SIM_SPEED_FACTOR=10` runs the simulation at 10× real time, which significantly speeds up swarm missions.
Set it to `1` for real-time playback.

**4. In a new terminal, launch Hawkeye with matching vehicle count:**

```sh
cd Hawkeye
./build/hawkeye -n 5
```

All five vehicles appear in Hawkeye immediately.
By default they spawn stacked on top of each other at the origin.
Use the swarm test script (next section) to spread them into formation and fly a mission.

_<!-- 08-vid-02: 45-60s video of the complete swarm demo: clear params → launch 5 instances → launch Hawkeye → run swarm_test.py → watch formation takeoff + waypoint + landing. -->_

## MAVSDK swarm test script

Hawkeye includes a Python script at `tests/swarm_test.py` that orchestrates a complete swarm mission using MAVSDK.
It sets per-instance spawn offsets, arms all vehicles, takes off in parallel, flies to a waypoint, and lands.

### Prerequisites

```sh
pip install mavsdk
```

### Running

With the five PX4 instances already running from the previous section:

```sh
python tests/swarm_test.py --n 5 --speed 10
```

The script connects to each PX4 instance via MAVSDK gRPC, configures spawn offsets for a line formation, and orchestrates the full mission.

### Options

| Flag                  | Description                             | Default |
| --------------------- | --------------------------------------- | ------- |
| `--n <count>`         | Number of vehicles                      | 5       |
| `--spacing <meters>`  | Formation spacing between vehicles      | 2.0     |
| `--altitude <meters>` | Takeoff altitude AGL                    | 10.0    |
| `--base-port <port>`  | PX4 MAVLink base UDP port               | 14540   |
| `--grpc-base <port>`  | MAVSDK gRPC base port                   | 50051   |
| `--speed <factor>`    | Sim speed factor for scaling wait times | 1.0     |

The `--speed` flag should match the `PX4_SIM_SPEED_FACTOR` used when launching SITL, so the script can scale its internal timeouts to match accelerated time.

### Mission sequence

The script runs this sequence:

1. Set `SIH_LOC_LAT0`, `SIH_LOC_LON0`, `SIH_LOC_ALT0` per instance for line formation
2. Wait for PX4 to pick up the new parameters (requires reboot of each instance)
3. Arm all vehicles in parallel
4. Command takeoff to `--altitude` meters
5. Fly to a waypoint
6. Land all vehicles
7. Disarm

Watch in Hawkeye as the swarm executes the mission.
Pressing `T` during the flight cycles trail modes; trail mode 3 (drone color) is particularly useful for keeping track of which drone is which.

## Next steps

- [Command-Line Reference](./cli.md) for the full list of `-n`, `-udp`, and replay flags
- [Multi-Drone Replay](./multi_drone.md) to replay the ULogs recorded from your live SITL session with correlation analysis and takeoff alignment
- [The HUD](./hud.md) for Console mode (analysis) and Tactical mode (recording demos)
- [PX4 SIH docs](https://docs.px4.io/main/en/sim_sih/) for the upstream simulator reference
- [PX4 SIH — Multi-Vehicle Simulation](https://docs.px4.io/main/en/sim_sih/#multi-vehicle-simulation) for PX4-side multi-instance setup
- [Prebuilt PX4 SITL packages](https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages) for running SITL without a source build
